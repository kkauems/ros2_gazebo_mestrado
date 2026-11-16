// Varredura em serpentina (boustrofédon) dentro de um retângulo.
//
// O primeiro canto (a) é o começo da primeira passada; as passadas seguem o
// eixo escolhido ('x' ou 'y') e se afastam de a em direção a b, separadas
// por spacing. Cada passada ganha pontos intermediários a cada maxLeg metros
// no máximo: o Nav2 só aceita objetivos a até ~12 m do robô (o costmap
// global é uma janela de 24 x 24 m).

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
      points.push({ name: p.name, x: round(p.x), y: round(p.y) });
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
