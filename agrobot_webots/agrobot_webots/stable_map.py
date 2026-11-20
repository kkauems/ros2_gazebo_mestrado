#!/usr/bin/env python3
"""
Grade de ocupação do VSLAM que não acumula ruído nas passadas repetidas.

O RTAB-Map guarda, em cada nó do grafo, a grade local vista dali (células
ocupadas e livres do LiDAR de segurança, no frame do nó) e monta a grade
global somando todas, cada uma na pose otimizada do nó. Toda passada por um
lugar já mapeado soma de novo. Se a passada nova volta com a pose deslocada
da anterior (o EKF deriva entre fechamentos de laço; na serpentina a
diferença passa de 0,3 m), as fileiras aparecem de novo ao lado, e os raios
livres da passada torta apagam as fileiras certas. Quanto mais o robô anda,
pior o mapa.

Este nó monta a grade global ele mesmo, com os mesmos dados (o tópico
mapData do rtabmap: o grafo otimizado e a grade local de cada nó), mesma
resolução e mesmo log-odds (ProbHit, ProbMiss e limites), e compara o que
cada nó novo vê com o mapa que já existe:

1. Mapeamento inicial: uma célula vista de perto (até settle_range do nó)
   por menos de settle_count nós ainda está sendo mapeada e é atualizada
   como no RTAB-Map. Lugar novo entra no mapa normalmente. O que o nó vê
   longe (um colmo a 8 m é atingido por poucos raios, e 1° de erro de yaw
   vira 14 cm) só preenche células ainda não assentadas.
2. Conferência do nó: os pontos ocupados que o nó vê em células já mapeadas
   são comparados com os obstáculos do mapa, na pose do nó e em poses
   deslocadas até search_radius e search_yaw. Se numa pose deslocada pelo
   menos min_shift a fração que bate (a até match_radius de um obstáculo)
   for margin maior que na pose do nó, a passada está desalinhada com o
   mapa e o nó fica de fora. Comparar com a melhor pose, e não com um
   mínimo fixo, funciona com um mapa ralo (um colmo de 2 cm nem sempre é
   atingido). Um nó com menos de min_checked pontos em lugar já mapeado
   (cabeceira, o robô olhando para fora do campo) não dá para conferir: só
   entra nas células ainda não assentadas.
3. Nó conferido, em células assentadas:
   - ponto ocupado perto (same_obstacle_radius) de um obstáculo do mapa é
     o mesmo obstáculo visto com um pouco de erro de pose: reforça o que
     já está ocupado e não cria célula nova ao lado (a fileira não
     engorda). Longe dos obstáculos do mapa, é um obstáculo novo (galhos,
     uma pedra, uma caixa) e soma evidência como no RTAB-Map: numa célula
     livre assentada são precisos uns 3 nós para ela virar ocupada;
   - raio livre sobre um obstáculo do mapa só conta se o nó não tem ponto
     ocupado a até same_obstacle_radius dali: um raio que passa ao lado
     de um colmo, ou uma passada com a fileira um pouco deslocada, não o
     apaga. Um obstáculo que saiu de verdade some com os raios livres.

Quando o grafo muda (um fechamento de laço move nós), a grade é refeita do
zero, na ordem dos nós, com as poses novas (no máximo a cada
rebuild_period s). Sem mudança, só os nós novos entram.

A camada de segurança e os costmaps do Nav2 não usam esta grade: eles leem
o /scan cru, e um obstáculo novo é sempre visto por eles na hora. Aqui ele
entra no mapa (checagem dos waypoints, RViz, página do operador) depois de
alguns nós.

Na localização (vslam_localization:=true) o mapa salvo é carregado no
início (serviço publish_map do rtabmap) e os nós da sessão passam pela
mesma conferência. O banco não é alterado: o que mudar na grade vale só
para a sessão.

Tópicos:
    /rtabmap/mapData   (rtabmap_msgs/MapData)    entrada
    /rtabmap/map       (nav_msgs/OccupancyGrid)  saída (o launch remapeia a
                                                 grade do próprio rtabmap
                                                 para /rtabmap/map_rtabmap)
"""

import array
import inspect
import math
import struct
import time
import zlib

import numpy as np


def logit(p):
    """Log-odds de uma probabilidade."""
    return math.log(p / (1.0 - p))


def uncompress_cells(blob):
    """
    Pontos (N, 2) de uma grade local comprimida pelo RTAB-Map.

    O formato (util3d::compressData2) é o cv::Mat em zlib seguido de três
    int32: linhas, colunas e tipo. As grades locais são CV_32FC2 (x, y) ou
    CV_32FC3 (x, y, z) no frame do nó, numa linha só.
    """
    blob = bytes(blob)
    if len(blob) <= 12:
        return np.zeros((0, 2), dtype=np.float32)
    rows, cols, cv_type = struct.unpack('<iii', blob[-12:])
    channels = (cv_type >> 3) + 1
    data = np.frombuffer(zlib.decompress(blob[:-12]), dtype=np.float32)
    points = data.reshape(rows * cols, channels)
    return points[:, :2]


def disk_offsets(radius_cells):
    """Deslocamentos (di, dj) de um disco de raio radius_cells."""
    r = int(math.ceil(radius_cells))
    di, dj = np.mgrid[-r:r + 1, -r:r + 1]
    inside = di * di + dj * dj <= radius_cells * radius_cells + 1e-9
    return di[inside], dj[inside]


class StableGrid:
    """
    Grade global montada nó a nó, com a conferência contra o mapa.

    check=False monta a grade como o RTAB-Map (todas as passadas somam),
    para comparar.
    """

    def __init__(self, resolution=0.05, prob_hit=0.7, prob_miss=0.47,
                 clamp_min=0.1192, clamp_max=0.971, settle_count=3,
                 settle_range=4.0, match_radius=0.10, margin=0.25,
                 min_shift=0.25, min_checked=20, search_radius=0.8,
                 search_yaw=0.04, same_obstacle_radius=0.30, check=True):
        """Parâmetros: ver o docstring do módulo e o rtabmap.yaml."""
        self.res = resolution
        self.hit = logit(prob_hit)
        self.miss = logit(prob_miss)
        self.lmin = logit(clamp_min)
        self.lmax = logit(clamp_max)
        self.settle = settle_count
        self.settle_range = settle_range
        self.margin = margin
        self.min_shift = min_shift
        self.search = int(round(search_radius / resolution))
        self.search_yaw = search_yaw
        self.yaw_steps = [0.0] + ([-search_yaw, -search_yaw / 2,
                                   search_yaw / 2, search_yaw]
                                  if search_yaw > 0 else [])
        self.min_checked = min_checked
        self.check = check
        self.match_off = disk_offsets(match_radius / resolution)
        self.same_off = disk_offsets(same_obstacle_radius / resolution)
        self.shape = None
        self.origin = (0.0, 0.0)

    # Grade -----------------------------------------------------------------

    def reset(self, xmin, ymin, xmax, ymax):
        """Grade vazia cobrindo [xmin, xmax] x [ymin, ymax] (m)."""
        res = self.res
        self.origin = (math.floor(xmin / res) * res,
                       math.floor(ymin / res) * res)
        width = int(math.ceil((xmax - self.origin[0]) / res)) + 1
        height = int(math.ceil((ymax - self.origin[1]) / res)) + 1
        self.shape = (height, width)
        size = height * width
        self.logodds = np.zeros(size, dtype=np.float32)
        self.seen = np.zeros(size, dtype=np.uint16)
        self.trusted = np.zeros(size, dtype=np.uint16)
        self.nodes = 0
        self.rejected = 0

    def contains(self, xmin, ymin, xmax, ymax):
        """Diz se a grade atual cobre o retângulo."""
        if self.shape is None:
            return False
        h, w = self.shape
        return (xmin >= self.origin[0] and ymin >= self.origin[1]
                and xmax < self.origin[0] + (w - 1) * self.res
                and ymax < self.origin[1] + (h - 1) * self.res)

    def cells(self, points, x, y, yaw):
        """Índices (planos, únicos) das células dos pontos do nó."""
        if not len(points):
            return np.zeros(0, dtype=np.int64)
        c, s = math.cos(yaw), math.sin(yaw)
        px = x + c * points[:, 0] - s * points[:, 1]
        py = y + s * points[:, 0] + c * points[:, 1]
        j = np.floor((px - self.origin[0]) / self.res).astype(np.int64)
        i = np.floor((py - self.origin[1]) / self.res).astype(np.int64)
        h, w = self.shape
        ok = (i >= 0) & (i < h) & (j >= 0) & (j < w)
        return np.unique(i[ok] * w + j[ok])

    def near(self, mask, flat, offsets):
        """Diz, para cada célula de flat, se há célula de mask no disco em volta."""
        found = np.zeros(len(flat), dtype=bool)
        if not len(flat):
            return found
        h, w = self.shape
        i, j = np.divmod(flat, w)
        for di, dj in zip(*offsets):
            ii, jj = i + di, j + dj
            ok = (ii >= 0) & (ii < h) & (jj >= 0) & (jj < w) & ~found
            found[ok] = mask[ii[ok] * w + jj[ok]]
        return found

    def alignment(self, points, x, y, yaw, occupied):
        """
        Confere os pontos ocupados do nó contra o mapa.

        Mede quanto eles batem com o mapa na pose do nó e na melhor pose em
        volta dela.
        Só contam os pontos que caem em células já mapeadas (vistas por
        settle_count nós). Bater = cair a até match_radius de um obstáculo do
        mapa. A melhor pose é procurada a até search_radius (passo de uma
        célula) e search_yaw. Devolve (fração na pose, melhor fração, desvio
        da melhor pose em m), ou None se há menos de min_checked pontos.
        Comparar com a melhor pose, e não com um mínimo fixo, não depende de
        o mapa ter todos os colmos: um mapa ralo baixa as duas frações.
        """
        if len(points) < self.min_checked:
            return None
        c, s = math.cos(yaw), math.sin(yaw)
        px = x + c * points[:, 0] - s * points[:, 1]
        py = y + s * points[:, 0] + c * points[:, 1]
        h, w = self.shape
        res = self.res
        j = np.floor((px - self.origin[0]) / res).astype(np.int64)
        i = np.floor((py - self.origin[1]) / res).astype(np.int64)
        ok = (i >= 0) & (i < h) & (j >= 0) & (j < w)
        ok[ok] = self.seen[i[ok] * w + j[ok]] >= self.settle
        if ok.sum() < self.min_checked:
            return None
        local = points[ok].astype(np.float64)
        # Janela da grade em volta dos pontos, com os obstáculos engordados
        # de match_radius.
        pad = self.search + 3 + int(math.ceil(
            np.abs(local).max() * self.search_yaw / res))
        i0, i1 = max(i[ok].min() - pad, 0), min(i[ok].max() + pad + 1, h)
        j0, j1 = max(j[ok].min() - pad, 0), min(j[ok].max() + pad + 1, w)
        window = occupied.reshape(h, w)[i0:i1, j0:j1]
        near = np.zeros_like(window)
        for di, dj in zip(*self.match_off):
            src = window[max(di, 0):window.shape[0] + min(di, 0),
                         max(dj, 0):window.shape[1] + min(dj, 0)]
            near[max(-di, 0):window.shape[0] + min(-di, 0),
                 max(-dj, 0):window.shape[1] + min(-dj, 0)] |= src
        wh, ww = near.shape
        steps = np.arange(-self.search, self.search + 1) * res
        dx, dy = np.meshgrid(steps, steps)
        dx, dy = dx.ravel(), dy.ravel()
        best, best_shift, at_pose = 0.0, 0.0, 0.0
        for dyaw in self.yaw_steps:
            c, s = math.cos(yaw + dyaw), math.sin(yaw + dyaw)
            qx = x + c * local[:, 0] - s * local[:, 1] - self.origin[0]
            qy = y + s * local[:, 0] + c * local[:, 1] - self.origin[1]
            jj = np.floor((qx[None, :] + dx[:, None]) / res).astype(
                np.int64) - j0
            ii = np.floor((qy[None, :] + dy[:, None]) / res).astype(
                np.int64) - i0
            inside = (ii >= 0) & (ii < wh) & (jj >= 0) & (jj < ww)
            hit = np.zeros(ii.shape, dtype=bool)
            hit[inside] = near[ii[inside], jj[inside]]
            frac = hit.mean(axis=1)
            k = int(np.argmax(frac))
            if frac[k] > best + 1e-9:
                best = float(frac[k])
                best_shift = math.hypot(dx[k], dy[k])
            if dyaw == 0.0:
                at_pose = float(frac[(len(frac) - 1) // 2])
        return at_pose, best, best_shift

    def update(self, flat, delta):
        """Soma delta ao log-odds das células, dentro dos limites."""
        if len(flat):
            self.logodds[flat] = np.clip(self.logodds[flat] + delta,
                                         self.lmin, self.lmax)

    # Nó --------------------------------------------------------------------

    def add(self, x, y, yaw, obstacles, empty):
        """
        Soma a grade local de um nó na pose (x, y, yaw) do mapa.

        obstacles e empty: pontos (N, 2) no frame do nó. Devolve o resultado
        da conferência (alignment(), ou None) e se o nó foi aceito.
        """
        self.nodes += 1
        if not self.check:
            hits = self.cells(obstacles, x, y, yaw)
            free = np.setdiff1d(self.cells(empty, x, y, yaw), hits,
                                assume_unique=True)
            self.update(hits, self.hit)
            self.update(free, self.miss)
            self.seen[hits] += 1
            self.seen[free] += 1
            return None, True

        # Perto do nó (settle_range) a grade local é boa: um colmo a 8 m é
        # atingido por poucos raios, e 1° de erro de yaw vira 14 cm. Só o
        # que está perto conta para assentar uma célula e pode mudar uma
        # célula já assentada; o que está longe só preenche o que ainda não
        # foi assentado.
        reach = self.settle_range
        close_ob = np.hypot(obstacles[:, 0], obstacles[:, 1]) <= reach
        close_em = np.hypot(empty[:, 0], empty[:, 1]) <= reach
        hits = self.cells(obstacles[close_ob], x, y, yaw)
        far_hits = np.setdiff1d(self.cells(obstacles[~close_ob], x, y, yaw),
                                hits, assume_unique=True)
        all_hits = np.union1d(hits, far_hits)
        free = np.setdiff1d(self.cells(empty[close_em], x, y, yaw),
                            all_hits, assume_unique=True)
        far_free = np.setdiff1d(
            self.cells(empty[~close_em], x, y, yaw),
            np.union1d(all_hits, free), assume_unique=True)

        occupied = self.logodds > 0.0
        settled = self.trusted >= self.settle

        # Conferência: o que o nó vê bate bem melhor com o mapa numa pose
        # deslocada? Então a pose dele está errada (a passada nova desalinhada
        # com a antiga) e ele fica de fora da grade inteira.
        score = self.alignment(obstacles, x, y, yaw, occupied)
        if score is not None and (score[1] - score[0] >= self.margin
                                  and score[2] >= self.min_shift):
            self.rejected += 1
            return score, False

        # Mapeamento inicial (células ainda não assentadas): como o
        # RTAB-Map. De longe, sem contar para assentar.
        for flat, delta, close in ((hits, self.hit, True),
                                   (free, self.miss, True),
                                   (far_hits, self.hit, False),
                                   (far_free, self.miss, False)):
            flat = flat[~settled[flat]]
            self.update(flat, delta)
            self.seen[flat] = np.minimum(self.seen[flat] + 1, 60000)
            if close:
                self.trusted[flat] += 1

        # Poucos pontos em células já mapeadas (lugar novo, cabeceira, o
        # robô olhando para fora do campo): não dá para conferir, e o nó não
        # mexe no que já estava mapeado.
        if score is None:
            return score, True

        # Células assentadas, vistas de perto: comparar com o mapa.
        old_hits = hits[settled[hits]]
        old_free = np.union1d(free[settled[free]],
                              far_free[settled[far_free]])
        # Ocupado onde o mapa tinha livre, perto de um obstáculo do mapa: é
        # o mesmo obstáculo visto com erro de pose, e não entra (senão a
        # fileira engorda ou aparece de novo ao lado). Longe, é um
        # obstáculo novo e soma evidência como no RTAB-Map: numa célula
        # livre assentada são precisos uns 3 nós para ela virar ocupada.
        fresh = old_hits[~occupied[old_hits]]
        fresh = fresh[~self.near(occupied, fresh, self.same_off)]
        self.update(np.concatenate([old_hits[occupied[old_hits]], fresh]),
                    self.hit)
        # Livre onde o mapa tinha obstáculo: só conta se o nó não tem
        # ponto ocupado a até same_obstacle_radius. Um raio passa ao lado
        # de um colmo de 2 cm sem atingi-lo; com o obstáculo deslocado pelo
        # erro de pose, os raios livres da passada nova cortariam a fileira
        # do mapa.
        cleared = old_free[occupied[old_free]]
        if len(cleared):
            node_hits = np.zeros(len(occupied), dtype=bool)
            node_hits[all_hits] = True
            cleared = cleared[~self.near(node_hits, cleared,
                                         self.same_off)]
        self.update(np.concatenate([old_free[~occupied[old_free]], cleared]),
                    self.miss)
        return score, True

    def occupancy(self):
        """Grade int8 (altura, largura): -1 desconhecido, 0 livre, 100 ocupado."""
        grid = np.full(self.logodds.shape, -1, dtype=np.int8)
        seen = self.seen > 0
        grid[seen] = np.where(self.logodds[seen] > 0.0, 100, 0)
        return grid.reshape(self.shape)


def yaw_of(q):
    """Yaw de um quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main(args=None):
    """Roda o nó stable_map."""
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from rtabmap_msgs.msg import MapData
    from rtabmap_msgs.srv import PublishMap

    class StableMap(Node):
        """Nó ROS: mapData do rtabmap -> grade conferida."""

        def __init__(self):
            super().__init__('stable_map')
            p = self.declare_parameter
            p('map_data_topic', '/rtabmap/mapData')
            p('map_topic', '/rtabmap/map')
            p('publish_map_service', '/rtabmap/rtabmap/publish_map')
            p('frame_id', 'map')
            # Mesmos valores do rtabmap.yaml (Grid/CellSize,
            # GridGlobal/ProbHit, ProbMiss e os limites padrão).
            p('resolution', 0.05)
            p('prob_hit', 0.7)
            p('prob_miss', 0.47)
            p('clamp_min', 0.1192)
            p('clamp_max', 0.971)
            # Alcance da grade local (Grid/RangeMax) mais folga: tamanho da
            # grade em volta dos nós.
            p('range_max', 9.0)
            # Conferência (ver o docstring do módulo e StableGrid).
            names = ('check', 'settle_count', 'settle_range', 'match_radius',
                     'margin', 'min_shift', 'min_checked', 'search_radius',
                     'search_yaw', 'same_obstacle_radius')
            signature = inspect.signature(StableGrid)
            for name in names:
                p(name, signature.parameters[name].default)
            # Nó que mexeu mais que isto (m ou rad) desde a última montagem:
            # refaz a grade do zero.
            p('rebuild_translation', 0.03)
            p('rebuild_rotation', 0.01)
            # Montagem do zero no máximo a cada rebuild_period s (com
            # milhares de nós ela leva alguns segundos); entre elas, os nós
            # novos entram na grade que já existe.
            p('rebuild_period', 10.0)
            p('publish_period', 1.0)
            get = lambda name: self.get_parameter(name).value  # noqa: E731
            grid_names = ('resolution', 'prob_hit', 'prob_miss', 'clamp_min',
                          'clamp_max') + names
            self.grid = StableGrid(**{n: get(n) for n in grid_names})
            self.range = get('range_max')
            self.frame_id = get('frame_id')
            self.rebuild_t = get('rebuild_translation')
            self.rebuild_r = get('rebuild_rotation')
            self.rebuild_period = get('rebuild_period')
            self.last_rebuild = -math.inf
            self.data = {}       # id -> (obstáculos, livres) no frame do nó
            self.poses = {}      # id -> (x, y, yaw) do último grafo
            self.used = {}       # id -> pose com que o nó entrou na grade
            self.changed = False
            self.publisher = self.create_publisher(
                OccupancyGrid, get('map_topic'), QoSProfile(
                    depth=1, reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(MapData, get('map_data_topic'),
                                     self.map_data_callback, 10)
            # O mapa que já existe (localização, ou o nó reiniciado): o
            # rtabmap publica uma vez o mapData com todos os nós.
            self.client = self.create_client(
                PublishMap, get('publish_map_service'))
            self.requested = None    # instante do último pedido
            self.create_timer(get('publish_period'), self.tick)
            self.stats_time = time.monotonic()
            self.build_time = 0.0

        def map_data_callback(self, message):
            """Guarda as grades locais novas e as poses do grafo."""
            for node in message.nodes:
                d = node.data
                if node.id in self.data or not d.grid_cell_size:
                    continue
                obstacles = uncompress_cells(d.grid_obstacles)
                empty = np.concatenate([uncompress_cells(d.grid_empty_cells),
                                        uncompress_cells(d.grid_ground)])
                self.data[node.id] = (obstacles, empty)
            graph = message.graph
            poses = {i: (p.position.x, p.position.y, yaw_of(p.orientation))
                     for i, p in zip(graph.poses_id, graph.poses) if i > 0}
            if poses:
                self.poses = poses
                self.changed = True

        def request_map(self):
            """Pede o mapa inteiro ao rtabmap enquanto faltarem nós."""
            # O mapData de cada quadro só traz os dados do nó novo. Os nós
            # que já existiam (o mapa salvo na localização, ou tudo se este
            # nó reiniciou) vêm de um pedido ao rtabmap, que publica o mapa
            # inteiro uma vez. Pede de novo enquanto faltar algum: o serviço
            # aparece antes de o rtabmap terminar de carregar o banco.
            newest = max(self.poses, default=0)
            missing = [i for i in self.poses
                       if i not in self.data and i != newest]
            if self.requested is not None and not missing:
                if self.requested is not False and self.data:
                    self.get_logger().info(
                        f'Mapa recebido do rtabmap: {len(self.data)} nós.')
                    self.requested = False
                return
            if not self.client.service_is_ready():
                return
            now = time.monotonic()
            if self.requested and now - self.requested < 5.0:
                return
            request = PublishMap.Request()
            request.global_map = True
            request.optimized = True
            request.graph_only = False
            self.client.call_async(request)
            if missing:
                self.get_logger().info(
                    f'Pedindo ao rtabmap o mapa inteiro ({len(missing)} nós '
                    'sem dados).')
            self.requested = now

        def needs_rebuild(self):
            """Diz se algum nó já somado mudou de pose no grafo."""
            for i, (x, y, yaw) in self.used.items():
                nx, ny, nyaw = self.poses[i]
                dyaw = abs(math.remainder(nyaw - yaw, math.tau))
                if (math.hypot(nx - x, ny - y) > self.rebuild_t
                        or dyaw > self.rebuild_r):
                    return True
            return False

        def tick(self):
            """Soma os nós novos (ou refaz a grade) e publica."""
            self.request_map()
            if not self.changed:
                return
            self.changed = False
            ids = sorted(i for i in self.poses if i in self.data)
            if not ids:
                return
            start = time.monotonic()
            xy = np.array([self.poses[i][:2] for i in ids])
            box = (xy[:, 0].min() - self.range, xy[:, 1].min() - self.range,
                   xy[:, 0].max() + self.range, xy[:, 1].max() + self.range)
            # Nó que saiu do grafo (na localização, o nó da sessão anterior
            # sai a cada quadro) não refaz a grade: o que ele viu fica até a
            # próxima montagem do zero.
            self.used = {i: p for i, p in self.used.items()
                         if i in self.poses}
            outside = not self.grid.contains(*box)
            if outside or (self.needs_rebuild() and time.monotonic()
                           - self.last_rebuild >= self.rebuild_period):
                self.last_rebuild = time.monotonic()
                margin = self.range
                self.grid.reset(box[0] - margin, box[1] - margin,
                                box[2] + margin, box[3] + margin)
                self.used = {}
            for i in ids:
                if i in self.used:
                    continue
                pose = self.poses[i]
                self.grid.add(*pose, *self.data[i])
                self.used[i] = pose
            # Nós que saíram do grafo não voltam.
            for i in [i for i in self.data if i not in self.poses]:
                del self.data[i]
            self.build_time += time.monotonic() - start
            self.publish()
            if time.monotonic() - self.stats_time > 60.0:
                self.get_logger().info(
                    f'Grade: {len(self.used)} nós, {self.grid.rejected} '
                    'recusados por não bater com o mapa; '
                    f'{self.build_time:.1f} s montando no último minuto.')
                self.stats_time = time.monotonic()
                self.build_time = 0.0

        def publish(self):
            """Publica a grade como nav_msgs/OccupancyGrid."""
            message = OccupancyGrid()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self.frame_id
            info = message.info
            info.resolution = self.grid.res
            info.height, info.width = self.grid.shape
            info.origin.position.x, info.origin.position.y = self.grid.origin
            info.origin.orientation.w = 1.0
            message.data = array.array(
                'b', self.grid.occupancy().ravel().tobytes())
            self.publisher.publish(message)

    rclpy.init(args=args)
    node = StableMap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
