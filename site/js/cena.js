// Cena 3D: o robô (medidas do agrobot.proto), os mundos e as camadas
// (LiDAR, câmeras, frames, trilhas, costmap). Usa o three.js r147 da pasta
// vendor/ como script comum, para abrir o site direto do disco.
//
// Dentro do grupo "mundo" as coordenadas são as do ROS: x para a frente,
// y para a esquerda, z para cima, em metros, no frame da partida (odom/map).

(function () {
  const T = THREE;

  const COR = {
    verdade: 0x2fbf71,
    'ekf': 0xf29e38,
    'ekf+vslam': 0x4aa3ff,
    'ekf+slam': 0xc678dd,
    destaque: 0xffb627,
  };

  // Medidas do agrobot.proto.
  const ALTURA_BASE = 0.24;          // base_link acima do chão (rodas r 0,22, eixo z -0,02)
  const RODA_R = 0.22;
  const MEIA_BITOLA = 0.48;
  const LIDAR = [0.65, 0, -0.10];
  const RGBD = [0.45, 0, 0.38];
  const RGBD_PITCH = 0.2618;
  const CAMDADOS = [0, 0, 0.66];

  // Waypoints dos casos (config/waypoints/).
  const WAYPOINTS = {
    plantacao: [
      ['meio_y0', 6.0, 0.0], ['cabeceira_leste_y0', 11.3, 0.0],
      ['cabeceira_leste_y1.8', 11.3, 1.8], ['meio_y1.8', 6.0, 1.8],
      ['partida', 0.0, 0.0],
    ],
    olival: [
      ['y0_x11', 11, 0], ['y0_x22', 22, 0], ['y0_x33', 33, 0], ['y0_x44', 44, 0],
      ['y0_x55', 55, 0], ['y0_x66', 66, 0], ['cabeceira_leste_y0', 71, 0],
      ['cabeceira_leste_y7', 71, 7], ['y7_x60', 60, 7], ['y7_x49', 49, 7],
      ['y7_x38', 38, 7], ['y7_x27', 27, 7], ['y7_x16', 16, 7], ['y7_x5', 5, 7],
      ['partida', 0, 0],
    ],
  };

  const S = {
    camadas: new Set(),
    partes: [],
    mundo: 'plantacao',
    metodo: 'ekf+vslam',
    t: 0,
    tocando: true,
    velocidade: 4,
    seguir: true,
    labels: [],
    obstaculos: { plantacao: [], olival: [] },
  };

  // ROS (x, y, z) -> three (x, z, -y)
  function v3(x, y, z) { return new T.Vector3(x, z, -y); }

  function rng(seed) {
    let s = seed >>> 0;
    return function () {
      s = (s * 1664525 + 1013904223) >>> 0;
      return s / 4294967296;
    };
  }

  function mat(cor, opts) {
    return new T.MeshStandardMaterial(Object.assign({ color: cor, roughness: 0.8, metalness: 0.05 }, opts || {}));
  }

  // ---------------------------------------------------------------- robô
  function construirRobo() {
    const g = new T.Group();
    const partes = { chassi: [], rodas: [], lidar: [], rgbd: [], camdados: [], imu: [] };
    function add(parte, malha, pai) {
      malha.castShadow = true;
      malha.userData.parte = parte;
      partes[parte].push(malha);
      (pai || g).add(malha);
      return malha;
    }
    function caixa(parte, sx, sy, sz, x, y, z, cor, pai) {
      const m = new T.Mesh(new T.BoxGeometry(sx, sy, sz), mat(cor, { metalness: 0.3, roughness: 0.55 }));
      m.position.set(x, y, z);
      return add(parte, m, pai);
    }
    const ALUMINIO = 0xa9b1ba;

    // Chassi do CAD (malha embutida em malha_chassi.js).
    try {
      const bin = atob(window.MALHA_CHASSI);
      const buf = new Uint8Array(bin.length);
      for (let i = 0; i < bin.length; i++) buf[i] = bin.charCodeAt(i);
      const geo = new T.STLLoader().parse(buf.buffer);
      const m = new T.Mesh(geo, mat(0x8f9ba8, { metalness: 0.35, roughness: 0.5 }));
      m.position.set(-0.5, -0.275, 0.05);
      m.scale.setScalar(0.001);
      add('chassi', m);
    } catch (e) {
      caixa('chassi', 0.88, 0.58, 0.28, 0, 0, 0.075, 0x8f9ba8);
    }
    // Suportes visuais.
    for (const [x, y] of [[0.35, 0.2925], [0.35, -0.2925], [-0.35, 0.2925], [-0.35, -0.2925]]) {
      const eixo = new T.Mesh(new T.CylinderGeometry(0.01, 0.01, 0.255, 12), mat(ALUMINIO, { metalness: 0.4 }));
      eixo.position.set(x, y, -0.02);
      add('rodas', eixo);
    }
    caixa('chassi', 0.045, 0.49, 0.045, 0, 0, 0.1925, ALUMINIO);
    caixa('imu', 0.03, 0.03, 0.12, 0, 0, 0.275, ALUMINIO);
    caixa('imu', 0.08, 0.05, 0.03, 0, 0, 0.35, 0x1d1f24);
    caixa('camdados', 0.03, 0.03, 0.47, 0, -0.055, 0.45, ALUMINIO);
    caixa('rgbd', 0.03, 0.03, 0.155, 0.4175, 0, 0.2925, ALUMINIO);
    caixa('lidar', 0.27, 0.04, 0.025, 0.535, 0, 0.0475, ALUMINIO);
    caixa('lidar', 0.03, 0.03, 0.115, 0.65, 0, -0.0225, ALUMINIO);

    // LiDAR.
    const lidar = new T.Mesh(new T.CylinderGeometry(0.04, 0.04, 0.04, 24), mat(0x111111));
    lidar.rotation.x = Math.PI / 2;
    lidar.position.set(...LIDAR);
    add('lidar', lidar);

    // Câmera RGB-D: frame com x para a frente, inclinado 15° para baixo.
    const rgbd = new T.Group();
    rgbd.position.set(...RGBD);
    rgbd.rotation.y = RGBD_PITCH;
    g.add(rgbd);
    caixa('rgbd', 0.026, 0.124, 0.029, -0.013, 0, 0, 0x15171a, rgbd);
    for (const y of [-0.047, 0.047, 0.0]) {
      const lente = new T.Mesh(new T.CylinderGeometry(0.007, 0.007, 0.004, 12), mat(0x3a4a66, { metalness: 0.6, roughness: 0.2 }));
      lente.rotation.z = Math.PI / 2;
      lente.position.set(0.001, y, 0);
      add('rgbd', lente, rgbd);
    }

    // Câmera de aquisição: olha para +y.
    const camdados = new T.Group();
    camdados.position.set(...CAMDADOS);
    camdados.rotation.z = Math.PI / 2;
    g.add(camdados);
    caixa('camdados', 0.04, 0.05, 0.05, -0.02, 0, 0, 0x15171a, camdados);
    const lente = new T.Mesh(new T.CylinderGeometry(0.015, 0.015, 0.012, 16), mat(0x3a4a66, { metalness: 0.6, roughness: 0.2 }));
    lente.rotation.z = Math.PI / 2;
    lente.position.set(0.005, 0, 0);
    add('camdados', lente, camdados);

    // Rodas com garras.
    const rodas = [];
    const pneu = mat(0x2a2b2e, { roughness: 0.95 });
    const cubo = mat(0xd8dadd, { metalness: 0.3 });
    const garraGeo = new T.BoxGeometry(0.05, 0.14, 0.025);
    for (const [x, y] of [[0.35, MEIA_BITOLA], [0.35, -MEIA_BITOLA], [-0.35, MEIA_BITOLA], [-0.35, -MEIA_BITOLA]]) {
      const roda = new T.Group();
      roda.position.set(x, y, -0.02);
      g.add(roda);
      add('rodas', new T.Mesh(new T.CylinderGeometry(0.195, 0.195, 0.14, 32), pneu.clone()), roda);
      const lado = Math.sign(y);
      const c1 = add('rodas', new T.Mesh(new T.CylinderGeometry(0.12, 0.12, 0.02, 24), cubo.clone()), roda);
      c1.position.y = 0.075 * lado;
      const c2 = add('rodas', new T.Mesh(new T.CylinderGeometry(0.045, 0.045, 0.035, 16), mat(ALUMINIO, { metalness: 0.4 })), roda);
      c2.position.y = 0.09 * lado;
      for (let k = 0; k < 16; k++) {
        const a = k * Math.PI / 8;
        const garra = add('rodas', new T.Mesh(garraGeo, pneu.clone()), roda);
        garra.position.set(0.2075 * Math.sin(a), 0, 0.2075 * Math.cos(a));
        garra.rotation.y = a;
      }
      rodas.push({ grupo: roda, lado: lado });
    }

    // Cada parte com o próprio material, para o destaque não vazar.
    for (const lista of Object.values(partes)) {
      for (const m of lista) {
        m.material = m.material.clone();
        m.material.emissive = new T.Color(COR.destaque);
        m.material.emissiveIntensity = 0;
      }
    }
    return { grupo: g, partes, rodas, rgbd, camdados };
  }

  // Seta de eixos (x vermelho, y verde, z azul) em coordenadas do ROS.
  function eixos(tam, espessura) {
    const g = new T.Group();
    const defs = [[0xe5484d, [0, 0, -Math.PI / 2]], [0x30a46c, [0, 0, 0]], [0x3e63dd, [Math.PI / 2, 0, 0]]];
    for (const [cor, rot] of defs) {
      const haste = new T.Mesh(new T.CylinderGeometry(espessura, espessura, tam, 8), new T.MeshBasicMaterial({ color: cor }));
      haste.position.y = tam / 2;
      const ponta = new T.Mesh(new T.ConeGeometry(espessura * 2.5, tam * 0.2, 10), new T.MeshBasicMaterial({ color: cor }));
      ponta.position.y = tam;
      const braco = new T.Group();
      braco.add(haste, ponta);
      braco.rotation.set(...rot);
      g.add(braco);
    }
    return g;
  }

  // Pirâmide de visão de uma câmera que olha para +x do próprio frame.
  function piramide(hfov, vfov, perto, longe, cor, opacidade) {
    const g = new T.Group();
    const th = Math.tan(hfov / 2), tv = Math.tan(vfov / 2);
    const canto = (d, sy, sz) => new T.Vector3(d, sy * d * th, sz * d * tv);
    const p = [], l = [];
    const cs = [[1, 1], [-1, 1], [-1, -1], [1, -1]];
    for (let i = 0; i < 4; i++) {
      const [a, b] = cs[i], [c, d] = cs[(i + 1) % 4];
      const n1 = canto(perto, a, b), f1 = canto(longe, a, b), n2 = canto(perto, c, d), f2 = canto(longe, c, d);
      p.push(n1, f1, f2, n1, f2, n2);
      l.push(new T.Vector3(0, 0, 0), f1, n1, n2, f1, f2);
    }
    const geo = new T.BufferGeometry().setFromPoints(p);
    const faces = new T.Mesh(geo, new T.MeshBasicMaterial({ color: cor, transparent: true, opacity: opacidade, side: T.DoubleSide, depthWrite: false }));
    const linhas = new T.LineSegments(new T.BufferGeometry().setFromPoints(l), new T.LineBasicMaterial({ color: cor, transparent: true, opacity: 0.9 }));
    g.add(faces, linhas);
    // Retângulo do plano de fundo (o que a foto cobre).
    const fundo = [canto(longe, 1, 1), canto(longe, -1, 1), canto(longe, -1, -1), canto(longe, 1, -1)];
    const plano = new T.Mesh(new T.BufferGeometry().setFromPoints([fundo[0], fundo[1], fundo[2], fundo[0], fundo[2], fundo[3]]),
      new T.MeshBasicMaterial({ color: cor, transparent: true, opacity: opacidade * 1.8, side: T.DoubleSide, depthWrite: false }));
    g.add(plano);
    return g;
  }

  // Faixa no chão ao longo de uma lista de pontos [x, y] (trilha).
  function faixa(pontos, largura, cor, z) {
    const pos = new Float32Array(Math.max(1, pontos.length - 1) * 18);
    for (let i = 0; i + 1 < pontos.length; i++) {
      const [x1, y1] = pontos[i], [x2, y2] = pontos[i + 1];
      let dx = x2 - x1, dy = y2 - y1;
      const n = Math.hypot(dx, dy) || 1;
      const nx = -dy / n * largura / 2, ny = dx / n * largura / 2;
      const q = [[x1 + nx, y1 + ny], [x1 - nx, y1 - ny], [x2 - nx, y2 - ny], [x1 + nx, y1 + ny], [x2 - nx, y2 - ny], [x2 + nx, y2 + ny]];
      for (let k = 0; k < 6; k++) {
        pos[i * 18 + k * 3] = q[k][0];
        pos[i * 18 + k * 3 + 1] = q[k][1];
        pos[i * 18 + k * 3 + 2] = z;
      }
    }
    const geo = new T.BufferGeometry();
    geo.setAttribute('position', new T.BufferAttribute(pos, 3));
    const m = new T.Mesh(geo, new T.MeshBasicMaterial({ color: cor, side: T.DoubleSide, transparent: true, opacity: 0.9, depthWrite: false }));
    m.renderOrder = 2;
    return m;
  }

  // Textura de solo procedural.
  function texturaSolo(base, variacao, seed, riscos) {
    const c = document.createElement('canvas');
    c.width = c.height = 256;
    const ctx = c.getContext('2d');
    ctx.fillStyle = base;
    ctx.fillRect(0, 0, 256, 256);
    const r = rng(seed);
    for (let i = 0; i < 2600; i++) {
      const v = (r() - 0.5) * variacao;
      ctx.fillStyle = `rgba(${v > 0 ? 255 : 0},${v > 0 ? 240 : 0},${v > 0 ? 220 : 0},${Math.abs(v)})`;
      const s = 1 + r() * 3;
      ctx.fillRect(r() * 256, r() * 256, s, s);
    }
    if (riscos) {
      for (let i = 0; i < 60; i++) {
        ctx.fillStyle = 'rgba(200,200,190,0.5)';
        ctx.fillRect(r() * 256, r() * 256, 2 + r() * 3, 1 + r() * 2);
      }
    }
    const t = new T.CanvasTexture(c);
    t.wrapS = t.wrapT = T.RepeatWrapping;
    return t;
  }

  // ------------------------------------------------------------- mundos
  function construirPlantacao() {
    const g = new T.Group();
    const r = rng(7);
    // Solo (centro das fileiras em x = 6).
    const tex = texturaSolo('#7a5a3c', 0.25, 3);
    tex.repeat.set(16, 12);
    const chao = new T.Mesh(new T.PlaneGeometry(40, 30), mat(0xffffff, { map: tex }));
    chao.position.set(6, 0, 0);
    chao.receiveShadow = true;
    g.add(chao);
    // Camalhões sob as fileiras.
    const fileiras = [-4.5, -2.7, -0.9, 0.9, 2.7, 4.5];
    for (const y of fileiras) {
      const c = new T.Mesh(new T.BoxGeometry(10.2, 0.45, 0.05), mat(0x6a4b30));
      c.position.set(6, y, 0.0);
      c.receiveShadow = true;
      g.add(c);
    }
    // Plantas (milho): colmo + folhas, instanciadas.
    const plantas = [];
    for (const y of fileiras) {
      for (let x = 1.5; x <= 10.51; x += 0.3) {
        if (r() < 0.08) continue;
        plantas.push([x + (r() - 0.5) * 0.06, y + (r() - 0.5) * 0.06, 0.9 + r() * 0.7, r() * Math.PI * 2]);
      }
    }
    const colmo = new T.InstancedMesh(new T.CylinderGeometry(0.012, 0.018, 1, 6), mat(0x7f9a3a), plantas.length);
    const nFolhas = 6;
    const folhaGeo = new T.PlaneGeometry(0.6, 0.07);
    folhaGeo.translate(0.3, 0, 0);
    const folhas = new T.InstancedMesh(folhaGeo, mat(0x4f8a2b, { side: T.DoubleSide }), plantas.length * nFolhas);
    const pendao = new T.InstancedMesh(new T.ConeGeometry(0.04, 0.25, 5), mat(0xc9b25a), plantas.length);
    const m4 = new T.Matrix4(), q = new T.Quaternion(), e = new T.Euler(), s = new T.Vector3(), p = new T.Vector3();
    const corF = new T.Color();
    plantas.forEach(([x, y, h, a], i) => {
      // CylinderGeometry é em y; gira para z.
      e.set(Math.PI / 2, 0, 0);
      q.setFromEuler(e);
      m4.compose(p.set(x, y, h / 2), q, s.set(1, h, 1));
      colmo.setMatrixAt(i, m4);
      m4.compose(p.set(x, y, h + 0.1), q, s.set(1, 1, 1));
      pendao.setMatrixAt(i, m4);
      for (let k = 0; k < nFolhas; k++) {
        const altura = h * (0.25 + 0.7 * k / nFolhas);
        const giro = a + k * 2.4;
        const caida = -0.35 - r() * 0.5;
        e.set(0, caida, giro, 'ZYX');
        q.setFromEuler(e);
        const tam = 0.55 + 0.35 * (1 - Math.abs(k / nFolhas - 0.45));
        m4.compose(p.set(x, y, altura), q, s.set(tam, 1, 1));
        folhas.setMatrixAt(i * nFolhas + k, m4);
        corF.setHSL(0.27 + r() * 0.04, 0.5, 0.28 + r() * 0.1);
        folhas.setColorAt(i * nFolhas + k, corF);
      }
      S.obstaculos.plantacao.push([x, y, 0.035]);
    });
    for (const m of [colmo, folhas, pendao]) { m.castShadow = true; g.add(m); }
    // Pedras, torrões e galhos nas entrelinhas.
    const pedraGeo = new T.DodecahedronGeometry(1, 0);
    for (let i = 0; i < 70; i++) {
      const y = [-3.6, -1.8, 0, 1.8, 3.6][Math.floor(r() * 5)] + (r() - 0.5) * 1.1;
      const x = 0.5 + r() * 11;
      const t = 0.03 + r() * 0.05;
      const pedra = new T.Mesh(pedraGeo, mat(r() < 0.5 ? 0x8d877d : 0x5e4632));
      pedra.scale.set(t, t * (0.8 + r() * 0.5), t * 0.6);
      pedra.position.set(x, y, t * 0.2);
      pedra.rotation.set(r() * 3, r() * 3, r() * 3);
      pedra.castShadow = true;
      g.add(pedra);
    }
    // Cerca de mourões (world ±7,6 × ±6,6; o odom começa em x = -6).
    const mourao = new T.CylinderGeometry(0.05, 0.06, 1.2, 8);
    const madeira = mat(0x6e5236);
    const cantos = [[-1.6, -6.6], [13.6, -6.6], [13.6, 6.6], [-1.6, 6.6]];
    for (let i = 0; i < 4; i++) {
      const [x1, y1] = cantos[i], [x2, y2] = cantos[(i + 1) % 4];
      const n = Math.round(Math.hypot(x2 - x1, y2 - y1) / 2.5);
      for (let k = 0; k < n; k++) {
        const x = x1 + (x2 - x1) * k / n, y = y1 + (y2 - y1) * k / n;
        const m = new T.Mesh(mourao, madeira);
        m.rotation.x = Math.PI / 2;
        m.position.set(x, y, 0.6);
        m.castShadow = true;
        g.add(m);
        S.obstaculos.plantacao.push([x, y, 0.06]);
      }
      const arame = new T.Line(new T.BufferGeometry().setFromPoints([new T.Vector3(x1, y1, 0.9), new T.Vector3(x2, y2, 0.9)]), new T.LineBasicMaterial({ color: 0x555555 }));
      g.add(arame);
    }
    return g;
  }

  function construirOlival() {
    const g = new T.Group();
    const r = rng(11);
    const tex = texturaSolo('#8a4b31', 0.3, 5, true);
    tex.repeat.set(40, 30);
    const chao = new T.Mesh(new T.PlaneGeometry(110, 80), mat(0xffffff, { map: tex }));
    chao.position.set(36, 3.5, 0);
    chao.receiveShadow = true;
    g.add(chao);
    // Estrada de terra ao sul e vinha ao norte (world y -30,5 a -27 e 28,5 a 36).
    const estrada = new T.Mesh(new T.PlaneGeometry(100, 3.5), mat(0xb59473));
    estrada.position.set(36, -25.25, 0.01);
    estrada.receiveShadow = true;
    g.add(estrada);
    const arvores = [];
    for (let j = 0; j < 7; j++) {
      for (let k = 0; k < 10; k++) {
        if (r() < 0.05) continue;
        arvores.push([4.5 + 7 * k + (r() - 0.5) * 0.3, -17.5 + 7 * j + (r() - 0.5) * 0.3, r() < 0.12 ? 0.55 : 1]);
      }
    }
    const tronco = new T.InstancedMesh(new T.CylinderGeometry(0.13, 0.22, 1, 7), mat(0x5b4a3a), arvores.length);
    const nCopa = 5;
    const copa = new T.InstancedMesh(new T.IcosahedronGeometry(1, 1), mat(0x7a8657, { flatShading: true }), arvores.length * nCopa);
    const m4 = new T.Matrix4(), q = new T.Quaternion(), e = new T.Euler(), s = new T.Vector3(), p = new T.Vector3();
    const cor = new T.Color();
    arvores.forEach(([x, y, esc], i) => {
      const h = (1.0 + r() * 0.25) * esc;
      e.set(Math.PI / 2 + (r() - 0.5) * 0.15, 0, (r() - 0.5) * 0.15);
      q.setFromEuler(e);
      m4.compose(p.set(x, y, h / 2), q, s.set(esc, h, esc));
      tronco.setMatrixAt(i, m4);
      for (let k = 0; k < nCopa; k++) {
        const a = r() * Math.PI * 2, d = (k === 0 ? 0 : 0.9) * esc;
        const raio = (k === 0 ? 1.5 : 0.9 + r() * 0.4) * esc;
        q.identity();
        m4.compose(p.set(x + d * Math.cos(a), y + d * Math.sin(a), h + (0.9 + r() * 0.5) * esc), q, s.set(raio, raio, raio * 0.7));
        copa.setMatrixAt(i * nCopa + k, m4);
        cor.setHSL(0.2 + r() * 0.04, 0.18 + r() * 0.08, 0.36 + r() * 0.08);
        copa.setColorAt(i * nCopa + k, cor);
      }
      S.obstaculos.olival.push([x, y, 0.18 * esc]);
    });
    tronco.castShadow = copa.castShadow = true;
    g.add(tronco, copa);
    // Vinha: postes e arame.
    for (const y of [32, 34.5, 37, 39.5]) {
      const fio = new T.Mesh(new T.BoxGeometry(68, 0.25, 0.9), mat(0x557a35));
      fio.position.set(36, y, 0.75);
      fio.castShadow = true;
      g.add(fio);
    }
    // Pedras na linha das árvores.
    const pedraGeo = new T.DodecahedronGeometry(1, 0);
    for (let i = 0; i < 60; i++) {
      const y = -17.5 + 7 * Math.floor(r() * 7) + (r() - 0.5) * 1.5;
      const t = 0.05 + r() * 0.12;
      const pedra = new T.Mesh(pedraGeo, mat(0xa39d92));
      pedra.scale.set(t * 1.4, t, t * 0.7);
      pedra.position.set(2 + r() * 68, y, t * 0.2);
      pedra.castShadow = true;
      g.add(pedra);
    }
    return g;
  }

  // ------------------------------------------------------------ trajetos
  // Plantação: rodadas reais do caso 2 (trajetorias.js).
  function amostra(metodo, t) {
    const a = window.TRAJETORIAS[metodo].amostras;
    if (t <= a[0][0]) return a[0];
    if (t >= a[a.length - 1][0]) return a[a.length - 1];
    let lo = 0, hi = a.length - 1;
    while (hi - lo > 1) {
      const m = (lo + hi) >> 1;
      if (a[m][0] <= t) lo = m; else hi = m;
    }
    const u = (t - a[lo][0]) / (a[hi][0] - a[lo][0]);
    const r = a[lo].slice();
    for (const k of [1, 2, 4, 5]) r[k] = a[lo][k] + (a[hi][k] - a[lo][k]) * u;
    for (const k of [3, 6]) {
      let d = a[hi][k] - a[lo][k];
      d = Math.atan2(Math.sin(d), Math.cos(d));
      r[k] = a[lo][k] + d * u;
    }
    r.indice = lo;
    return r;
  }

  // Olival: rota do caso 4 com velocidade constante, giro no lugar e
  // parada de 3 s em cada waypoint (sem rodadas medidas ainda).
  function rotaOlival() {
    const seg = [];
    let x = 0, y = 0, yaw = 0, t = 0;
    for (const [, wx, wy] of WAYPOINTS.olival) {
      const alvo = Math.atan2(wy - y, wx - x);
      let d = alvo - yaw;
      d = Math.atan2(Math.sin(d), Math.cos(d));
      const tg = Math.abs(d) / 0.8;
      seg.push({ t0: t, t1: t + tg, x0: x, y0: y, x1: x, y1: y, a0: yaw, a1: yaw + d });
      t += tg; yaw += d;
      const dist = Math.hypot(wx - x, wy - y);
      seg.push({ t0: t, t1: t + dist / 0.5, x0: x, y0: y, x1: wx, y1: wy, a0: yaw, a1: yaw });
      t += dist / 0.5; x = wx; y = wy;
      seg.push({ t0: t, t1: t + 3, x0: x, y0: y, x1: x, y1: y, a0: yaw, a1: yaw });
      t += 3;
    }
    return { seg, fim: t };
  }

  function poseOlival(t) {
    const R = S.rotaOlival;
    let wp = 0, k = 0;
    for (; k < R.seg.length - 1; k++) {
      if (t < R.seg[k].t1) break;
    }
    const s = R.seg[k];
    wp = Math.floor(k / 3) + 1;
    const u = Math.min(1, Math.max(0, (t - s.t0) / Math.max(1e-6, s.t1 - s.t0)));
    const x = s.x0 + (s.x1 - s.x0) * u, y = s.y0 + (s.y1 - s.y0) * u, a = s.a0 + (s.a1 - s.a0) * u;
    return [t, x, y, a, x, y, a, wp];
  }

  function duracao() {
    if (S.mundo === 'olival') return S.rotaOlival.fim;
    const a = window.TRAJETORIAS[S.metodo].amostras;
    return a[a.length - 1][0];
  }

  function poseAtual() {
    return S.mundo === 'olival' ? poseOlival(S.t) : amostra(S.metodo, S.t);
  }

  // ------------------------------------------------------------- camadas
  function construirTrilhas() {
    if (S.trilhas) { S.mundoG.remove(S.trilhas.g); }
    const g = new T.Group();
    let verdade, estimada = null, tempos;
    if (S.mundo === 'olival') {
      const pts = [];
      tempos = [];
      for (let t = 0; t <= S.rotaOlival.fim; t += 1) {
        const p = poseOlival(t);
        pts.push([p[4], p[5]]);
        tempos.push(t);
      }
      verdade = faixa(pts, 0.12, COR.verdade, 0.04);
    } else {
      const a = window.TRAJETORIAS[S.metodo].amostras;
      tempos = a.map(r => r[0]);
      verdade = faixa(a.map(r => [r[4], r[5]]), 0.12, COR.verdade, 0.035);
      estimada = faixa(a.map(r => [r[1], r[2]]), 0.12, COR[S.metodo], 0.045);
      g.add(estimada);
    }
    g.add(verdade);
    S.mundoG.add(g);
    S.trilhas = { g, verdade, estimada, tempos };
  }

  function construirWaypoints() {
    if (S.wps) { S.mundoG.remove(S.wps.g); S.wps.labels.forEach(l => l.el.remove()); }
    const g = new T.Group();
    const labels = [];
    const lista = WAYPOINTS[S.mundo];
    const marcadores = lista.map(([nome, x, y], i) => {
      const m = new T.Group();
      const anel = new T.Mesh(new T.RingGeometry(0.22, 0.3, 32), new T.MeshBasicMaterial({ color: 0xffffff, side: T.DoubleSide, transparent: true, opacity: 0.9 }));
      anel.position.z = 0.05;
      const haste = new T.Mesh(new T.CylinderGeometry(0.015, 0.015, 1.4, 6), new T.MeshBasicMaterial({ color: 0xffffff }));
      haste.rotation.x = Math.PI / 2;
      haste.position.z = 0.7;
      const bandeira = new T.Mesh(new T.PlaneGeometry(0.3, 0.2), new T.MeshBasicMaterial({ color: 0xffffff, side: T.DoubleSide }));
      bandeira.rotation.x = Math.PI / 2;
      bandeira.position.set(0.15, 0, 1.3);
      m.add(anel, haste, bandeira);
      m.position.set(x, y, 0);
      g.add(m);
      labels.push(criarLabel(`${i + 1}. ${nome}`, 'wp', () => m.localToWorld(new T.Vector3(0, 0, 1.55)), 'waypoints'));
      return { m, anel, bandeira, haste };
    });
    S.mundoG.add(g);
    S.wps = { g, marcadores, labels };
  }

  function construirLidar() {
    const N = 100;
    const pos = new Float32Array(N * 6);
    const geo = new T.BufferGeometry();
    geo.setAttribute('position', new T.BufferAttribute(pos, 3));
    const cores = new Float32Array(N * 6);
    geo.setAttribute('color', new T.BufferAttribute(cores, 3));
    const linhas = new T.LineSegments(geo, new T.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.55 }));
    const pgeo = new T.BufferGeometry();
    pgeo.setAttribute('position', new T.BufferAttribute(new Float32Array(N * 3), 3));
    const pontos = new T.Points(pgeo, new T.PointsMaterial({ color: 0xff3b3b, size: 0.07 }));
    const g = new T.Group();
    g.add(linhas, pontos);
    linhas.frustumCulled = pontos.frustumCulled = false;
    S.mundoG.add(g);
    S.lidar = { g, N, linhas, pontos, acertos: [] };
  }

  function atualizarLidar(x, y, yaw) {
    const L = S.lidar;
    const ox = x + Math.cos(yaw) * LIDAR[0], oy = y + Math.sin(yaw) * LIDAR[0];
    const z = ALTURA_BASE + LIDAR[2];
    const obs = S.obstaculos[S.mundo].filter(([a, b]) => Math.hypot(a - ox, b - oy) < 8.5);
    const pos = L.linhas.geometry.attributes.position.array;
    const cor = L.linhas.geometry.attributes.color.array;
    const pp = L.pontos.geometry.attributes.position.array;
    L.acertos = [];
    let np = 0;
    for (let i = 0; i < L.N; i++) {
      const a = yaw - 1.5 + 3.0 * i / (L.N - 1);
      const dx = Math.cos(a), dy = Math.sin(a);
      let best = 8.0;
      for (const [cx, cy, r] of obs) {
        const fx = cx - ox, fy = cy - oy;
        const tca = fx * dx + fy * dy;
        if (tca < 0.1) continue;
        const d2 = fx * fx + fy * fy - tca * tca;
        if (d2 > r * r) continue;
        const th = tca - Math.sqrt(r * r - d2);
        if (th > 0.1 && th < best) best = th;
      }
      const hit = best < 8.0;
      pos.set([ox, oy, z, ox + dx * best, oy + dy * best, z], i * 6);
      const c = hit ? [1, 0.35, 0.3] : [1, 0.6, 0.55];
      const c0 = hit ? [1, 0.35, 0.3] : [1, 0.75, 0.7];
      cor.set([...c0, ...(hit ? c : [0.75, 0.75, 0.75])], i * 6);
      if (hit) {
        pp.set([ox + dx * best, oy + dy * best, z], np * 3);
        np++;
        L.acertos.push([ox + dx * best, oy + dy * best]);
      }
    }
    L.linhas.material.opacity = 0.35;
    L.linhas.geometry.attributes.position.needsUpdate = true;
    L.linhas.geometry.attributes.color.needsUpdate = true;
    L.pontos.geometry.setDrawRange(0, np);
    L.pontos.geometry.attributes.position.needsUpdate = true;
  }

  // Costmap local: 4 × 4 m alinhado ao odom, com inflação de 0,7 m.
  function construirCostmap() {
    const c = document.createElement('canvas');
    c.width = c.height = 128;
    const tex = new T.CanvasTexture(c);
    const plano = new T.Mesh(new T.PlaneGeometry(4, 4), new T.MeshBasicMaterial({ map: tex, transparent: true, depthWrite: false }));
    plano.renderOrder = 1;
    const borda = new T.LineSegments(new T.EdgesGeometry(new T.PlaneGeometry(4, 4)), new T.LineBasicMaterial({ color: 0x2fbf71 }));
    const g = new T.Group();
    g.add(plano, borda);
    S.mundoG.add(g);
    S.costmap = { g, c, ctx: c.getContext('2d'), tex, ultimo: 0 };
  }

  function atualizarCostmap(x, y, agora) {
    const C = S.costmap;
    C.g.position.set(x, y, 0.06);
    if (agora - C.ultimo < 0.1) return;
    C.ultimo = agora;
    const ctx = C.ctx, px = 128 / 4;
    ctx.clearRect(0, 0, 128, 128);
    ctx.fillStyle = 'rgba(47,191,113,0.10)';
    ctx.fillRect(0, 0, 128, 128);
    for (const [hx, hy] of S.lidar.acertos) {
      const u = (hx - x + 2) * px, v = (2 - (hy - y)) * px;
      if (u < -30 || u > 158 || v < -30 || v > 158) continue;
      const gr = ctx.createRadialGradient(u, v, 0, u, v, 0.7 * px);
      gr.addColorStop(0, 'rgba(255,0,180,0.85)');
      gr.addColorStop(0.15, 'rgba(255,60,60,0.7)');
      gr.addColorStop(1, 'rgba(80,120,255,0)');
      ctx.fillStyle = gr;
      ctx.fillRect(u - 0.7 * px, v - 0.7 * px, 1.4 * px, 1.4 * px);
    }
    C.tex.needsUpdate = true;
  }

  function construirCaminho() {
    const geo = new T.BufferGeometry().setFromPoints([new T.Vector3(), new T.Vector3()]);
    const linha = new T.Line(geo, new T.LineDashedMaterial({ color: 0xffffff, dashSize: 0.25, gapSize: 0.15 }));
    linha.frustumCulled = false;
    S.mundoG.add(linha);
    S.caminho = linha;
  }

  function construirFantasma() {
    const g = new T.Group();
    const pts = [[0.6, 0.56], [-0.6, 0.56], [-0.6, -0.56], [0.6, -0.56], [0.6, 0.56]].map(([a, b]) => new T.Vector3(a, b, 0));
    const contorno = new T.Line(new T.BufferGeometry().setFromPoints(pts), new T.LineBasicMaterial({ color: 0xffffff }));
    const seta = new T.Mesh(new T.ConeGeometry(0.15, 0.4, 3), new T.MeshBasicMaterial({ color: 0xffffff }));
    seta.rotation.z = -Math.PI / 2;
    seta.position.x = 0.35;
    seta.scale.z = 0.05;
    g.add(contorno, seta);
    S.mundoG.add(g);
    S.fantasma = { g, contorno, seta };
  }

  // ------------------------------------------------------------- rótulos
  function criarLabel(texto, classe, posicao, camada, parte) {
    const el = document.createElement('div');
    el.className = 'label3d ' + classe;
    el.textContent = texto;
    if (parte) {
      el.dataset.parte = parte;
      el.addEventListener('click', () => S.aoClicarParte && S.aoClicarParte(parte));
    }
    S.camadaLabels.appendChild(el);
    const l = { el, posicao, camada };
    S.labels.push(l);
    return l;
  }

  function atualizarLabels() {
    const w = S.el.clientWidth, h = S.el.clientHeight;
    for (const l of S.labels) {
      const visivel = S.camadas.has(l.camada) && document.body.contains(l.el);
      if (!visivel) { l.el.style.display = 'none'; continue; }
      const p = l.posicao().project(S.camera);
      if (p.z > 1 || p.x < -1.1 || p.x > 1.1 || p.y < -1.1 || p.y > 1.1) { l.el.style.display = 'none'; continue; }
      l.el.style.display = '';
      l.el.style.transform = `translate(-50%, -100%) translate(${(p.x + 1) / 2 * w}px, ${(1 - p.y) / 2 * h}px)`;
    }
  }

  // ------------------------------------------------------------ câmera 3D
  const PRESETS = {
    geral: { rel: true, pos: [-1.1, 2.6, 1.5], alvo: [0.1, 0, 0.1] },
    sensores: { rel: true, pos: [1.35, 1.05, 0.75], alvo: [0.3, 0, 0.12] },
    lado: { rel: true, pos: [-1.7, -0.25, 2.5], alvo: [0.25, 0.85, 0.45] },
    segue: { rel: true, pos: [-3.8, -1.2, 4.6], alvo: [1.2, 0, 0] },
    cima: {
      plantacao: { pos: [6, -5.5, 19], alvo: [6, 0.5, 0] },
      olival: { pos: [36, -32, 80], alvo: [36, 3.5, 0] },
    },
    alto: {
      plantacao: { pos: [-6, -11, 8], alvo: [6, 0, 0] },
      olival: { pos: [-8, -22, 16], alvo: [20, 3.5, 0] },
    },
  };

  function irPara(nome, instantaneo) {
    const pr = PRESETS[nome] || PRESETS.geral;
    const pose = poseAtual();
    let pos, alvo;
    if (pr.rel) {
      const [x, y, yaw] = [pose[4], pose[5], pose[6]];
      const loc = (v) => [x + Math.cos(yaw) * v[0] - Math.sin(yaw) * v[1], y + Math.sin(yaw) * v[0] + Math.cos(yaw) * v[1], ALTURA_BASE + v[2]];
      pos = v3(...loc(pr.pos));
      alvo = v3(...loc(pr.alvo));
      S.seguir = true;
    } else {
      const m = pr[S.mundo];
      pos = v3(...m.pos);
      alvo = v3(...m.alvo);
      S.seguir = false;
    }
    S.presetAtual = nome;
    if (instantaneo) {
      S.camera.position.copy(pos);
      S.controls.target.copy(alvo);
      S.transicao = null;
    } else {
      S.transicao = { de: S.camera.position.clone(), deAlvo: S.controls.target.clone(), para: pos, paraAlvo: alvo, t: 0 };
    }
  }

  // ------------------------------------------------------------- destaque
  function aplicarDestaque(tempo) {
    const k = 0.35 + 0.35 * Math.sin(tempo * 4);
    for (const [nome, lista] of Object.entries(S.robo.partes)) {
      const on = S.partes.includes(nome);
      for (const m of lista) m.material.emissiveIntensity = on ? k : 0;
    }
  }

  // --------------------------------------------------------------- início
  function iniciar(el, opcoes) {
    S.el = el;
    S.aoClicarParte = opcoes && opcoes.aoClicarParte;
    S.aoAtualizar = opcoes && opcoes.aoAtualizar;
    const renderer = new T.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = T.PCFSoftShadowMap;
    renderer.outputEncoding = T.sRGBEncoding;
    el.appendChild(renderer.domElement);
    S.renderer = renderer;
    S.camadaLabels = document.createElement('div');
    S.camadaLabels.className = 'labels3d';
    el.appendChild(S.camadaLabels);

    const scene = new T.Scene();
    scene.background = new T.Color(0xc9dbea);
    scene.fog = new T.Fog(0xc9dbea, 45, 160);
    S.scene = scene;
    S.camera = new T.PerspectiveCamera(45, 1, 0.05, 600);
    S.controls = new T.OrbitControls(S.camera, renderer.domElement);
    S.controls.enableDamping = true;
    S.controls.maxPolarAngle = Math.PI * 0.49;
    S.controls.minDistance = 0.6;
    S.controls.maxDistance = 140;

    scene.add(new T.HemisphereLight(0xf2f6ff, 0x6b5a40, 0.75));
    const sol = new T.DirectionalLight(0xfff4e0, 0.95);
    sol.castShadow = true;
    sol.shadow.mapSize.set(2048, 2048);
    const sc = sol.shadow.camera;
    sc.left = sc.bottom = -9; sc.right = sc.top = 9; sc.near = 1; sc.far = 60;
    sol.shadow.bias = -0.0005;
    scene.add(sol, sol.target);
    S.sol = sol;

    const mundoG = new T.Group();
    mundoG.rotation.x = -Math.PI / 2;
    scene.add(mundoG);
    S.mundoG = mundoG;

    S.cenarios = { plantacao: construirPlantacao(), olival: construirOlival() };
    mundoG.add(S.cenarios.plantacao, S.cenarios.olival);
    S.rotaOlival = rotaOlival();

    S.robo = construirRobo();
    mundoG.add(S.robo.grupo);
    // Camadas presas ao robô.
    const rgbdFov = piramide(1.518, 2 * Math.atan(Math.tan(1.518 / 2) * 240 / 424), 0.5, 6.0, 0x4aa3ff, 0.07);
    S.robo.rgbd.add(rgbdFov);
    const camFov = piramide(Math.PI / 2, 1.2864, 0.05, 0.9, 0xf2c94c, 0.12);
    S.robo.camdados.add(camFov);
    S.fov = { rgbd: rgbdFov, camdados: camFov };
    // Frames TF.
    const tf = new T.Group();
    const fBase = eixos(0.45, 0.012);
    fBase.position.z = 0.02;
    S.robo.grupo.add(fBase);
    const fLidar = eixos(0.18, 0.006); fLidar.position.set(...LIDAR); S.robo.grupo.add(fLidar);
    const fRgbd = eixos(0.18, 0.006); S.robo.rgbd.add(fRgbd);
    const fCam = eixos(0.18, 0.006); fCam.rotation.z = -Math.PI / 2; S.robo.camdados.add(fCam);
    const fImu = eixos(0.15, 0.006); fImu.position.set(0, 0, 0.35); S.robo.grupo.add(fImu);
    const fMap = eixos(1.0, 0.025); fMap.position.z = 0.02; tf.add(fMap);
    mundoG.add(tf);
    S.tf = [fBase, fLidar, fRgbd, fCam, fImu, tf];

    construirLidar();
    construirCostmap();
    construirCaminho();
    construirFantasma();

    // Rótulos das partes e dos frames.
    const R = S.robo.grupo;
    const em = (obj, x, y, z) => () => obj.localToWorld(new T.Vector3(x, y, z));
    criarLabel('LiDAR · /scan', 'parte', em(R, ...LIDAR), 'rotulos', 'lidar');
    criarLabel('Câmera RGB-D · VSLAM', 'parte', em(R, RGBD[0] + 0.02, 0, RGBD[2] + 0.06), 'rotulos', 'rgbd');
    criarLabel('Câmera de aquisição', 'parte', em(R, 0, 0, CAMDADOS[2] + 0.06), 'rotulos', 'camdados');
    criarLabel('IMU · /imu/data', 'parte', em(R, -0.05, 0, 0.37), 'rotulos', 'imu');
    criarLabel('Rodas · /wheel/odom', 'parte', em(R, -0.35, -0.5, 0.22), 'rotulos', 'rodas');
    criarLabel('Chassi', 'parte', em(R, -0.3, 0.3, 0.2), 'rotulos', 'chassi');
    criarLabel('base_link', 'tf', em(R, 0.47, 0, 0.05), 'tf');
    criarLabel('front_lidar_link', 'tf', em(R, LIDAR[0] + 0.2, 0, LIDAR[2]), 'tf');
    criarLabel('vslam_camera_link', 'tf', em(S.robo.rgbd, 0.22, 0, 0.02), 'tf');
    criarLabel('data_camera_link', 'tf', em(S.robo.camdados, 0.05, 0, 0.2), 'tf');
    criarLabel('imu_link', 'tf', em(R, 0, -0.18, 0.35), 'tf');
    criarLabel('map / odom (partida)', 'tf', em(fMap, 0, 0, 1.15), 'tf');
    S.labelFantasma = criarLabel('estimativa', 'fantasma', () => S.fantasma.g.localToWorld(new T.Vector3(0, 0, 0.35)), 'fantasma');

    // Clique numa parte do robô.
    const ray = new T.Raycaster();
    const ponteiro = new T.Vector2();
    let baixo = null;
    renderer.domElement.addEventListener('pointerdown', (e) => { baixo = [e.clientX, e.clientY]; });
    renderer.domElement.addEventListener('pointerup', (e) => {
      if (!baixo || Math.hypot(e.clientX - baixo[0], e.clientY - baixo[1]) > 5) return;
      const r = renderer.domElement.getBoundingClientRect();
      ponteiro.set((e.clientX - r.left) / r.width * 2 - 1, -(e.clientY - r.top) / r.height * 2 + 1);
      ray.setFromCamera(ponteiro, S.camera);
      const alvos = [].concat(...Object.values(S.robo.partes));
      const hit = ray.intersectObjects(alvos, false)[0];
      if (hit && S.aoClicarParte) S.aoClicarParte(hit.object.userData.parte);
    });

    new ResizeObserver(redimensionar).observe(el);
    redimensionar();
    definirMundo('plantacao');
    irPara('geral', true);
    let anterior = performance.now();
    function quadro(agora) {
      const dt = Math.min(0.1, (agora - anterior) / 1000);
      anterior = agora;
      passo(dt, agora / 1000);
      requestAnimationFrame(quadro);
    }
    requestAnimationFrame(quadro);
  }

  function redimensionar() {
    const w = S.el.clientWidth, h = S.el.clientHeight;
    if (!w || !h) return;
    S.renderer.setSize(w, h);
    S.camera.aspect = w / h;
    S.camera.updateProjectionMatrix();
  }

  function passo(dt, agora) {
    if (S.tocando) {
      S.t += dt * S.velocidade;
      const fim = duracao();
      if (S.t > fim + 3) S.t = 0;
    }
    const pose = poseAtual();
    const [, ex, ey, eyaw, x, y, yaw, wp] = pose;
    // Rodas: ângulo pelo deslocamento de cada lado (skid-steer).
    if (S.poseAnterior) {
      const [px, py, pyaw] = S.poseAnterior;
      const ds = (x - px) * Math.cos(yaw) + (y - py) * Math.sin(yaw);
      let dyaw = yaw - pyaw;
      dyaw = Math.atan2(Math.sin(dyaw), Math.cos(dyaw));
      if (Math.abs(ds) < 1 && Math.abs(dyaw) < 1) {
        for (const r of S.robo.rodas) r.grupo.rotation.y += (ds - r.lado * dyaw * MEIA_BITOLA) / RODA_R;
      }
    }
    S.poseAnterior = [x, y, yaw];
    const R = S.robo.grupo;
    const dx = x - R.position.x, dy = y - R.position.y;
    R.position.set(x, y, ALTURA_BASE);
    R.rotation.z = yaw;

    // Câmera acompanha o robô.
    if (S.transicao) {
      const tr = S.transicao;
      tr.t = Math.min(1, tr.t + dt / 1.1);
      const u = tr.t < 0.5 ? 2 * tr.t * tr.t : 1 - Math.pow(-2 * tr.t + 2, 2) / 2;
      S.camera.position.lerpVectors(tr.de, tr.para, u);
      S.controls.target.lerpVectors(tr.deAlvo, tr.paraAlvo, u);
      if (S.seguir) { const d = v3(dx, dy, 0); tr.para.add(d); tr.paraAlvo.add(d); }
      if (tr.t >= 1) S.transicao = null;
    } else if (S.seguir && Math.hypot(dx, dy) < 2) {
      const d = v3(dx, dy, 0);
      S.camera.position.add(d);
      S.controls.target.add(d);
    }
    S.controls.update();
    // Sombra acompanha o robô.
    const centro = v3(x, y, 0);
    S.sol.position.copy(centro).add(new T.Vector3(8, 18, 6));
    S.sol.target.position.copy(centro);

    // Camadas.
    const on = (c) => S.camadas.has(c);
    S.lidar.g.visible = on('lidar');
    if (on('lidar') || on('costmap')) atualizarLidar(x, y, yaw);
    S.costmap.g.visible = on('costmap');
    if (on('costmap')) atualizarCostmap(x, y, agora);
    S.fov.rgbd.visible = on('rgbd');
    S.fov.camdados.visible = on('camdados');
    for (const f of S.tf) f.visible = on('tf');
    S.wps.g.visible = on('waypoints');
    S.wps.marcadores.forEach((m, i) => {
      const atual = i + 1 === wp;
      const c = atual ? 0xffb627 : (i + 1 < wp ? 0x2fbf71 : 0xffffff);
      m.anel.material.color.setHex(c);
      m.bandeira.material.color.setHex(c);
    });
    // Trilhas até o instante atual.
    const tr = S.trilhas;
    tr.g.visible = on('trilhas');
    let n = 0;
    while (n < tr.tempos.length && tr.tempos[n] <= S.t) n++;
    tr.verdade.geometry.setDrawRange(0, Math.max(0, n - 1) * 6);
    if (tr.estimada) tr.estimada.geometry.setDrawRange(0, Math.max(0, n - 1) * 6);
    // Fantasma: onde o método acha que o robô está.
    const temEst = S.mundo === 'plantacao';
    S.fantasma.g.visible = on('fantasma') && temEst;
    if (temEst) {
      S.fantasma.g.position.set(ex, ey, 0.08);
      S.fantasma.g.rotation.z = eyaw;
      const cor = COR[S.metodo];
      S.fantasma.contorno.material.color.setHex(cor);
      S.fantasma.seta.material.color.setHex(cor);
    }
    // Caminho até o waypoint atual, a partir da pose estimada.
    const lista = WAYPOINTS[S.mundo];
    const alvo = lista[Math.min(lista.length, Math.max(1, wp)) - 1];
    S.caminho.visible = on('caminho') && S.t <= duracao();
    if (S.caminho.visible) {
      const ox = temEst ? ex : x, oy = temEst ? ey : y;
      S.caminho.geometry.setFromPoints([new T.Vector3(ox, oy, 0.1), new T.Vector3(alvo[1], alvo[2], 0.1)]);
      S.caminho.computeLineDistances();
    }

    aplicarDestaque(agora);
    if (S.el.clientWidth > 0) {
      S.renderer.render(S.scene, S.camera);
      atualizarLabels();
    }
    if (S.aoAtualizar) {
      const erro = temEst ? Math.hypot(ex - x, ey - y) : null;
      S.aoAtualizar({ t: S.t, fim: duracao(), erro, wp, nWp: lista.length, nomeWp: alvo[0] });
    }
  }

  function definirMundo(nome) {
    S.mundo = nome;
    S.cenarios.plantacao.visible = nome === 'plantacao';
    S.cenarios.olival.visible = nome === 'olival';
    S.t = 0;
    S.poseAnterior = null;
    construirTrilhas();
    construirWaypoints();
  }

  window.Cena = {
    iniciar,
    definirMundo(nome) { if (nome !== S.mundo) { definirMundo(nome); irPara(S.presetAtual || 'geral', true); } },
    definirMetodo(m) { if (m !== S.metodo) { S.metodo = m; construirTrilhas(); } },
    definirCamadas(lista) { S.camadas = new Set(lista); },
    alternarCamada(c, on) { if (on) S.camadas.add(c); else S.camadas.delete(c); },
    camadas() { return new Set(S.camadas); },
    destacar(partes) { S.partes = partes || []; },
    irPara,
    tocar(on) { S.tocando = on; },
    tocando() { return S.tocando; },
    velocidade(v) { S.velocidade = v; },
    irParaTempo(t) { S.t = t; S.poseAnterior = null; },
    mundo() { return S.mundo; },
    metodo() { return S.metodo; },
  };
})();
