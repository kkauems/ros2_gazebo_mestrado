"""
Checagem de um waypoint contra a grade de ocupação, pelas medidas do robô.

Sem ROS: recebe a grade como lista (OccupancyGrid.data) e devolve se o robô
cabe no ponto. Usada pelo waypoint_mission para recusar, antes de a missão
começar (ou no clique do RViz), um ponto onde o robô não cabe ou não consegue
girar.

Duas perguntas por waypoint:
- a pegada (retângulo do robô mais uma folga) cabe no ponto, olhando na
  direção em que ele chega?
- o robô consegue girar ali, da direção em que chega até a direção em que
  sai? O Nav2 chega a cada waypoint e gira no lugar para o yaw dele (o
  regulated pure pursuit gira pelo lado mais curto), então a pegada tem de
  caber em todos os ângulos do meio.

A direção de chegada sai de um caminho estimado na própria grade (A*, como
o planejador do Nav2: o centro do robô fica a pelo menos meia largura dos
obstáculos; o desconhecido é passável). Sem caminho, o ponto é inalcançável.
A linha reta do anterior até o ponto não serve: perto de obstáculo o Nav2
chega por outro lado (num teste, ela recusou um ponto bom).
"""

import heapq
import math

import numpy as np

OK = 'ok'
BLOCKED = 'colide'        # a pegada bate em obstáculo na direção de chegada
NO_TURN = 'sem_giro'      # cabe, mas não consegue girar até o yaw de saída
UNKNOWN = 'desconhecido'  # fora do mapa ou em área ainda não vista
UNREACHABLE = 'sem_caminho'  # não há caminho por onde o robô passe

# Passo angular ao varrer o giro.
TURN_STEP = math.radians(5.0)


class Grid:
    """Grade de ocupação 2D (como nav_msgs/OccupancyGrid), em linha por y."""

    def __init__(self, data, width, height, resolution, origin_x, origin_y,
                 origin_yaw=0.0, occupied=65, speck_radius=0.0,
                 speck_neighbors=2):
        """
        Guarda a grade; data tem -1 (desconhecido) ou 0 a 100.

        speck_radius > 0 tira os ciscos (despeckle): célula ocupada com
        menos de speck_neighbors outras ocupadas a até speck_radius vira
        livre. Veja remove_specks.
        """
        self.data = np.asarray(data, dtype=np.int16).reshape(
            int(height), int(width))
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.cos = math.cos(origin_yaw)
        self.sin = math.sin(origin_yaw)
        self.occupied = occupied
        self.specks = 0
        if speck_radius > 0.0:
            self.data, self.specks = remove_specks(
                self.data, occupied, speck_radius / self.resolution,
                speck_neighbors)

    def to_index(self, x, y):
        """Célula (i, j) do ponto (x, y); pode cair fora da grade."""
        dx, dy = x - self.origin_x, y - self.origin_y
        gx = self.cos * dx + self.sin * dy
        gy = -self.sin * dx + self.cos * dy
        return (int(math.floor(gx / self.resolution)),
                int(math.floor(gy / self.resolution)))

    def to_world(self, i, j):
        """Centro da célula (i, j) no frame da grade."""
        gx = (i + 0.5) * self.resolution
        gy = (j + 0.5) * self.resolution
        return (self.origin_x + self.cos * gx - self.sin * gy,
                self.origin_y + self.sin * gx + self.cos * gy)

    def inflated(self, radius):
        """Células onde o centro do robô (raio radius) bate (cacheado)."""
        key = round(radius, 3)
        cache = getattr(self, '_inflated', {})
        if key not in cache:
            occupied = self.data >= self.occupied
            out = occupied.copy()
            r = int(math.ceil(radius / self.resolution))
            h, w = occupied.shape
            for di in range(-r, r + 1):
                for dj in range(-r, r + 1):
                    if (di == 0 and dj == 0) or \
                            math.hypot(di, dj) * self.resolution > radius:
                        continue
                    out[max(0, dj):h + min(0, dj), max(0, di):w + min(0, di)] |= \
                        occupied[max(0, -dj):h + min(0, -dj),
                                 max(0, -di):w + min(0, -di)]
            cache[key] = out
            self._inflated = cache
        return cache[key]

    def values(self, x, y):
        """Valores das células nos pontos (x, y) (arrays); -1 fora da grade."""
        dx, dy = x - self.origin_x, y - self.origin_y
        gx = self.cos * dx + self.sin * dy
        gy = -self.sin * dx + self.cos * dy
        i = np.floor(gx / self.resolution).astype(int)
        j = np.floor(gy / self.resolution).astype(int)
        inside = (i >= 0) & (j >= 0) & (i < self.width) & (j < self.height)
        out = np.full(i.shape, -1, dtype=np.int16)
        out[inside] = self.data[j[inside], i[inside]]
        return out


def remove_specks(data, occupied, radius_cells, neighbors):
    """
    Tira da grade as células ocupadas isoladas: (grade nova, quantas).

    Na grade do RTAB-Map (5 cm) os torrões, as pedrinhas e os galhos das
    entrelinhas, por onde o robô passa por cima, e os toques do LiDAR no chão
    ondulado viram células soltas no meio da entrelinha, e uma só já
    recusava o waypoint. O que barra o robô tem vizinhas: as plantas da
    fileira ficam a cada 0,3 m, e pedra grande, tronco e mourão ocupam
    várias células. Uma célula ocupada com menos de neighbors outras a até
    radius_cells células vira livre (0).
    """
    solid = data >= occupied
    count = np.zeros(data.shape, dtype=np.int32)
    r = int(math.ceil(radius_cells))
    h, w = data.shape
    for di in range(-r, r + 1):
        for dj in range(-r, r + 1):
            if (di == 0 and dj == 0) or math.hypot(di, dj) > radius_cells:
                continue
            count[max(0, dj):h + min(0, dj), max(0, di):w + min(0, di)] += \
                solid[max(0, -dj):h + min(0, -dj), max(0, -di):w + min(0, -di)]
    specks = solid & (count < neighbors)
    out = data.copy()
    out[specks] = 0
    return out, int(np.count_nonzero(specks))


class Footprint:
    """Retângulo do robô centrado no base_link, com folga em volta."""

    def __init__(self, length, width, margin, spacing=0.05):
        """Pontos de amostra a cada spacing (no máximo) dentro do retângulo."""
        self.length = length
        self.width = width
        self.margin = margin
        half_x = 0.5 * length + margin
        half_y = 0.5 * width + margin
        nx = max(1, math.ceil(2.0 * half_x / spacing))
        ny = max(1, math.ceil(2.0 * half_y / spacing))
        px, py = np.meshgrid(np.linspace(-half_x, half_x, nx + 1),
                             np.linspace(-half_y, half_y, ny + 1))
        self.px = px.ravel()
        self.py = py.ravel()
        self.radius = math.hypot(half_x, half_y)

    def cells(self, grid, x, y, yaw):
        """Valores da grade sob a pegada na pose (x, y, yaw)."""
        c, s = math.cos(yaw), math.sin(yaw)
        return grid.values(x + c * self.px - s * self.py,
                           y + s * self.px + c * self.py)


def turn_angles(yaw_from, yaw_to, step=TURN_STEP):
    """Ângulos de yaw_from a yaw_to pelo lado mais curto, a cada step."""
    delta = math.atan2(math.sin(yaw_to - yaw_from),
                       math.cos(yaw_to - yaw_from))
    n = max(1, math.ceil(abs(delta) / step))
    return [yaw_from + delta * k / n for k in range(n + 1)]


def pose_status(grid, footprint, x, y, yaw, unknown_limit=0.5):
    """OK, BLOCKED ou UNKNOWN para a pegada parada em (x, y, yaw)."""
    values = footprint.cells(grid, x, y, yaw)
    if np.any(values >= grid.occupied):
        return BLOCKED
    if np.count_nonzero(values < 0) > unknown_limit * values.size:
        return UNKNOWN
    return OK


def check_waypoint(grid, footprint, x, y, yaw_in, yaw_out,
                   unknown_limit=0.5):
    """
    Diz se o robô cabe e gira no waypoint (x, y).

    O robô chega olhando para yaw_in e sai em yaw_out (o yaw do waypoint).
    Devolve (status, ângulo em rad onde bateu, ou None).
    """
    status = pose_status(grid, footprint, x, y, yaw_in, unknown_limit)
    if status != OK:
        return status, yaw_in
    for angle in turn_angles(yaw_in, yaw_out)[1:]:
        status = pose_status(grid, footprint, x, y, angle, unknown_limit)
        if status == BLOCKED:
            return NO_TURN, angle
    return OK, None


def plan(grid, start, goal, radius, start_clear=0.7, max_cells=400000):
    """
    Caminho (lista de (x, y)) de start a goal na grade, ou None.

    radius: meia largura do robô (o centro não chega mais perto que isso de
    um obstáculo). Em volta da partida (start_clear) as células bloqueadas
    passam, como o Nav2 faz com a célula do robô. Fora da grade não passa.
    """
    blocked = grid.inflated(radius)
    h, w = blocked.shape
    si, sj = grid.to_index(*start)
    gi, gj = grid.to_index(*goal)
    if not (0 <= si < w and 0 <= sj < h and 0 <= gi < w and 0 <= gj < h):
        return None
    if blocked[gj, gi]:
        return None
    clear = (start_clear / grid.resolution) ** 2
    steps = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
             (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]

    def heuristic(i, j):
        di, dj = abs(i - gi), abs(j - gj)
        return max(di, dj) + (math.sqrt(2) - 1.0) * min(di, dj)

    best = {(si, sj): 0.0}
    parent = {}
    queue = [(heuristic(si, sj), 0.0, si, sj)]
    expanded = 0
    while queue:
        _, cost, i, j = heapq.heappop(queue)
        if (i, j) == (gi, gj):
            cells = [(i, j)]
            while cells[-1] in parent:
                cells.append(parent[cells[-1]])
            return [grid.to_world(a, b) for a, b in reversed(cells)]
        if cost > best.get((i, j), math.inf):
            continue
        expanded += 1
        if expanded > max_cells:
            return None
        for di, dj, step in steps:
            ni, nj = i + di, j + dj
            if not (0 <= ni < w and 0 <= nj < h):
                continue
            if blocked[nj, ni] and (ni - si) ** 2 + (nj - sj) ** 2 > clear:
                continue
            new = cost + step
            if new < best.get((ni, nj), math.inf):
                best[(ni, nj)] = new
                parent[(ni, nj)] = (i, j)
                heapq.heappush(queue, (new + heuristic(ni, nj), new, ni, nj))
    return None


def arrival_heading(path, distance=0.5):
    """Direção do caminho nos últimos distance metros antes do fim."""
    if not path or len(path) < 2:
        return None
    end = path[-1]
    for point in reversed(path[:-1]):
        if math.hypot(end[0] - point[0], end[1] - point[1]) >= distance:
            break
    return math.atan2(end[1] - point[1], end[0] - point[0])


def inside(grid, x, y):
    """Diz se (x, y) cai dentro da grade."""
    i, j = grid.to_index(x, y)
    return 0 <= i < grid.width and 0 <= j < grid.height


def check_leg(grid, footprint, start, x, y, yaw_out):
    """
    Confere o waypoint (x, y) vindo de start (o anterior ou o robô).

    Devolve (status, ângulo onde bateu ou None, direção de chegada). A
    direção de chegada sai do caminho estimado de start até o ponto.
    """
    if not (inside(grid, *start) and inside(grid, x, y)):
        return UNKNOWN, None, yaw_out
    path = plan(grid, start, (x, y), 0.5 * footprint.width)
    if path is None:
        status = pose_status(grid, footprint, x, y, yaw_out)
        if status == OK:
            status = UNREACHABLE
        return status, None, yaw_out
    yaw_in = arrival_heading(path)
    if yaw_in is None:
        yaw_in = yaw_out
    status, angle = check_waypoint(grid, footprint, x, y, yaw_in, yaw_out)
    return status, angle, yaw_in


def suggest(grid, footprint, x, y, yaw_in, yaw_out, max_distance=1.0,
            step=0.1):
    """
    Acha o ponto válido mais perto de (x, y), com os mesmos yaws.

    Procura em anéis de step em step até max_distance; None se não houver.
    """
    rings = int(round(max_distance / step))
    for ring in range(1, rings + 1):
        radius = ring * step
        count = max(8, int(math.ceil(2.0 * math.pi * radius / step)))
        for k in range(count):
            angle = 2.0 * math.pi * k / count
            cx = x + radius * math.cos(angle)
            cy = y + radius * math.sin(angle)
            status, _ = check_waypoint(grid, footprint, cx, cy, yaw_in,
                                       yaw_out)
            if status == OK:
                return cx, cy
    return None
