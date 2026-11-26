// Varredura em serpentina (boustrofédon) dentro de um retângulo.
//
// O primeiro canto (a) é o começo da primeira passada; as passadas seguem o
// eixo escolhido ('x' ou 'y') e se afastam de a em direção a b, separadas
// por spacing. Cada passada ganha pontos intermediários a cada maxLeg metros
// no máximo: o Nav2 só aceita objetivos a até ~12 m do robô (o costmap
// global é uma janela de 24 x 24 m).
//
// Cada ponto leva o yaw da própria passada (olhando ao longo da fileira).
// Sem isso o waypoint_mission completaria o yaw olhando para o próximo
// ponto, e no fim de cada passada o robô teria de girar 90° no lugar, entre
// as fileiras, onde ele não cabe girando (1,2 x 1,12 m numa entrelinha de
// 1,8 m): a checagem recusava quase todos os pontos. Com o yaw da passada,
// a curva para a passada seguinte fica por conta do planejador do Nav2,
// que sai pela cabeceira.

export function generateSweep(a, b, { spacing, axis = 'x', maxLeg = 10 } = {}) {
  if (!(spacing > 0)) throw new Error('espaçamento inválido');
  const along = axis === 'x' ? 'x' : 'y';
  const across = along === 'x' ? 'y' : 'x';
  const width = Math.abs(b[across] - a[across]);
  const sign = b[across] >= a[across] ? 1 : -1;
  const lanes = Math.floor(width / spacing + 1e-6) + 1;
  const start = a[along];
  const end = b[along];
  const length = Math.abs(end - start);
  if (length < 0.5) throw new Error('retângulo curto demais nessa direção');
  const pieces = Math.max(1, Math.ceil(length / maxLeg - 1e-9));
  const ahead = along === 'x' ? (end >= start ? 0 : Math.PI) : (end >= start ? Math.PI / 2 : -Math.PI / 2);
  const back = ahead === 0 ? Math.PI : ahead === Math.PI ? 0 : -ahead;
  const points = [];
  for (let lane = 0; lane < lanes; lane += 1) {
    const offset = a[across] + sign * lane * spacing;
    const forward = lane % 2 === 0;
    for (let k = 0; k <= pieces; k += 1) {
      const t = forward ? k / pieces : 1 - k / pieces;
      const p = {};
      p[along] = start + (end - start) * t;
      p[across] = offset;
      p.name = `p${lane + 1}_${k === 0 ? 'ini' : k === pieces ? 'fim' : k}`;
      points.push({ name: p.name, x: round(p.x), y: round(p.y), yaw: forward ? ahead : back });
    }
  }
  return points;
}

function round(v) { return Math.round(v * 1000) / 1000; }

// Comprimento total da rota (m), contando do primeiro ponto.
export function routeLength(points) {
  let total = 0;
  for (let i = 1; i < points.length; i += 1) {
    total += Math.hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
  }
  return total;
}
