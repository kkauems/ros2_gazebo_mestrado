// Mapa 2D da página: grade (do operator_bridge), robô, scan, caminho do
// Nav2, waypoints e a prévia da varredura. Coordenadas em metros no frame
// global (map com VSLAM, odom sem), x para a direita e y para cima.

const ROBOT_LENGTH = 1.2;
const ROBOT_WIDTH = 1.12;

export const STATUS_COLORS = {
  pendente: '#f0a020',
  atual: '#2a7fff',
  atingido: '#2fbf4f',
  perdido: '#e5484d',
  invalido: '#a855f7',
  cancelado: '#8a8f98',
};

export class MapView {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.scale = 30;          // px por metro
    this.center = { x: 4, y: 0 };
    this.follow = false;
    this.map = null;          // {image, info}
    this.state = null;        // /operator/state
    this.waypoints = [];
    this.preview = [];        // prévia da varredura
    this.rect = null;         // retângulo sendo desenhado {a, b}
    this.hover = null;
    this.tool = 'pan';
    this.handlers = {};
    this.drag = null;
    this.locked = false;      // missão rodando: os pontos não se movem
    // Checagem do ponto sendo arrastado (resposta do probe do
    // waypoint_mission): {x, y, status, text, suggestion} ou null.
    this.probe = null;
    this.setupEvents();
    new ResizeObserver(() => this.resize()).observe(canvas.parentElement);
    this.resize();
    const loop = () => { this.draw(); requestAnimationFrame(loop); };
    requestAnimationFrame(loop);
  }

  on(event, handler) { this.handlers[event] = handler; }

  fire(event, ...args) { if (this.handlers[event]) this.handlers[event](...args); }

  resize() {
    const box = this.canvas.parentElement.getBoundingClientRect();
    const ratio = window.devicePixelRatio || 1;
    this.width = box.width;
    this.height = box.height;
    this.canvas.width = Math.round(box.width * ratio);
    this.canvas.height = Math.round(box.height * ratio);
    this.canvas.style.width = `${box.width}px`;
    this.canvas.style.height = `${box.height}px`;
    this.ratio = ratio;
  }

  toScreen(x, y) {
    return [
      this.width / 2 + (x - this.center.x) * this.scale,
      this.height / 2 - (y - this.center.y) * this.scale,
    ];
  }

  toWorld(px, py) {
    return {
      x: this.center.x + (px - this.width / 2) / this.scale,
      y: this.center.y - (py - this.height / 2) / this.scale,
    };
  }

  setMap(info, image) { this.map = { info, image }; }

  centerOnRobot() {
    const pose = this.state && this.state.pose;
    if (pose) this.center = { x: pose[0], y: pose[1] };
  }

  fitWaypoints() {
    const pts = this.waypoints.concat(this.preview);
    if (this.state && this.state.pose) pts.push({ x: this.state.pose[0], y: this.state.pose[1] });
    if (!pts.length) return;
    const xs = pts.map((p) => p.x);
    const ys = pts.map((p) => p.y);
    const [x0, x1, y0, y1] = [Math.min(...xs), Math.max(...xs), Math.min(...ys), Math.max(...ys)];
    this.center = { x: (x0 + x1) / 2, y: (y0 + y1) / 2 };
    // Pelo menos 10 m na tela, para o robô sozinho não ficar enorme.
    const sx = (this.width - 80) / Math.max(x1 - x0, 10);
    const sy = (this.height - 80) / Math.max(y1 - y0, 10);
    this.scale = Math.max(2, Math.min(120, sx, sy));
  }

  // Ponto sob o cursor: {list: 'waypoints' | 'preview', index} ou null.
  // A prévia da varredura vem primeiro (fica por cima quando existe).
  pointAt(px, py) {
    for (const list of ['preview', 'waypoints']) {
      const points = this[list];
      const radius = list === 'preview' ? 9 : 12;
      for (let i = points.length - 1; i >= 0; i -= 1) {
        const [sx, sy] = this.toScreen(points[i].x, points[i].y);
        if (Math.hypot(sx - px, sy - py) < radius) return { list, index: i };
      }
    }
    return null;
  }

  waypointAt(px, py) {
    const hit = this.pointAt(px, py);
    return hit && hit.list === 'waypoints' ? hit.index : -1;
  }

  // Arrastar pontos vale em qualquer ferramenta, menos com a missão rodando.
  draggableAt(px, py) {
    return this.locked ? null : this.pointAt(px, py);
  }

  setupEvents() {
    const c = this.canvas;
    const pos = (e) => {
      const r = c.getBoundingClientRect();
      return [e.clientX - r.left, e.clientY - r.top];
    };
    c.addEventListener('wheel', (e) => {
      e.preventDefault();
      const [px, py] = pos(e);
      const before = this.toWorld(px, py);
      this.scale = Math.max(2, Math.min(200, this.scale * Math.exp(-e.deltaY * 0.0015)));
      const after = this.toWorld(px, py);
      this.center.x += before.x - after.x;
      this.center.y += before.y - after.y;
    }, { passive: false });
    c.addEventListener('contextmenu', (e) => {
      e.preventDefault();
      const [px, py] = pos(e);
      const hit = this.locked ? null : this.pointAt(px, py);
      if (hit) this.fire(hit.list === 'preview' ? 'remove-preview' : 'remove', hit.index);
    });
    c.addEventListener('pointerdown', (e) => {
      if (e.button !== 0) return;
      c.setPointerCapture(e.pointerId);
      const [px, py] = pos(e);
      const world = this.toWorld(px, py);
      const hit = this.draggableAt(px, py);
      if (hit) {
        const p = this[hit.list][hit.index];
        this.drag = {
          kind: 'point', list: hit.list, index: hit.index, px, py, moved: false,
          // Onde o ponto estava e a distância do clique até o centro dele.
          from: { x: p.x, y: p.y }, offset: { x: p.x - world.x, y: p.y - world.y },
        };
      } else if (this.tool === 'sweep') {
        this.rect = { a: world, b: world };
        this.drag = { kind: 'rect' };
      } else {
        this.drag = {
          kind: 'pan', px, py, center: { ...this.center }, moved: false, world,
        };
      }
      this.updateCursor(px, py);
    });
    c.addEventListener('pointermove', (e) => {
      const [px, py] = pos(e);
      const world = this.toWorld(px, py);
      this.hover = world;
      this.fire('hover', world);
      this.updateCursor(px, py);
      if (!this.drag) return;
      if (this.drag.kind === 'pan') {
        const dx = px - this.drag.px;
        const dy = py - this.drag.py;
        if (Math.hypot(dx, dy) > 4) this.drag.moved = true;
        if (this.drag.moved) {
          this.follow = false;
          this.fire('follow', false);
          this.center = {
            x: this.drag.center.x - dx / this.scale,
            y: this.drag.center.y + dy / this.scale,
          };
        }
      } else if (this.drag.kind === 'point') {
        const d = this.drag;
        if (!d.moved && Math.hypot(px - d.px, py - d.py) < 4) return;
        d.moved = true;
        const p = round({ x: world.x + d.offset.x, y: world.y + d.offset.y });
        const points = this[d.list];
        points[d.index] = { ...points[d.index], ...p };
        d.at = p;
        this.fire('dragging', d.list, d.index, p);
      } else if (this.drag.kind === 'rect') {
        this.rect.b = world;
        this.fire('rect', this.rect, false);
      }
    });
    const finish = (e, cancelled) => {
      const drag = this.drag;
      this.drag = null;
      if (!drag) return;
      const [px, py] = pos(e);
      const world = this.toWorld(px, py);
      this.updateCursor(px, py);
      if (drag.kind === 'point') {
        if (drag.moved && !cancelled) {
          this.fire('moved', drag.list, drag.index, drag.at);
        } else if (drag.moved) {
          const points = this[drag.list];
          points[drag.index] = { ...points[drag.index], ...drag.from };
          this.fire('drag-cancel', drag.list, drag.index);
        } else if (this.tool === 'goto') {
          this.fire('goto', round(world));
        }
      } else if (drag.kind === 'rect') {
        this.rect.b = world;
        this.fire('rect', this.rect, true);
      } else if (!drag.moved && !cancelled) {
        if (this.tool === 'add') this.fire('add', round(world));
        else if (this.tool === 'goto') this.fire('goto', round(world));
      }
    };
    c.addEventListener('pointerup', (e) => finish(e, false));
    c.addEventListener('pointercancel', (e) => finish(e, true));
    // Esc durante o arrasto devolve o ponto para onde estava.
    window.addEventListener('keydown', (e) => {
      if (e.code !== 'Escape' || !this.drag || this.drag.kind !== 'point') return;
      const d = this.drag;
      this.drag = null;
      const points = this[d.list];
      points[d.index] = { ...points[d.index], ...d.from };
      this.fire('drag-cancel', d.list, d.index);
    });
    c.addEventListener('pointerleave', () => { this.hover = null; this.fire('hover', null); });
  }

  updateCursor(px, py) {
    let cursor = this.tool === 'pan' ? 'grab' : 'crosshair';
    if (this.drag && this.drag.kind === 'point') cursor = 'grabbing';
    else if (this.drag && this.drag.kind === 'pan' && this.drag.moved) cursor = 'grabbing';
    else if (!this.drag && this.draggableAt(px, py)) cursor = 'grab';
    this.canvas.style.cursor = cursor;
  }

  draw() {
    const ctx = this.ctx;
    ctx.setTransform(this.ratio, 0, 0, this.ratio, 0, 0);
    const styles = getComputedStyle(document.documentElement);
    ctx.fillStyle = styles.getPropertyValue('--map-bg').trim() || '#1b1e24';
    ctx.fillRect(0, 0, this.width, this.height);
    if (this.follow) this.centerOnRobot();
    this.drawGrid(styles);
    this.drawMap();
    this.drawScan();
    this.drawPlan();
    this.drawRoute(this.preview, true);
    this.drawRoute(this.waypoints, false);
    this.drawRect();
    this.drawRobot();
    this.drawProbe();
    this.drawScaleBar(styles);
  }

  drawGrid(styles) {
    const ctx = this.ctx;
    const step = this.scale > 25 ? 1 : this.scale > 6 ? 5 : 10;
    const tl = this.toWorld(0, 0);
    const br = this.toWorld(this.width, this.height);
    ctx.lineWidth = 1;
    ctx.strokeStyle = styles.getPropertyValue('--map-grid').trim() || '#2a2e36';
    ctx.beginPath();
    for (let x = Math.floor(tl.x / step) * step; x <= br.x; x += step) {
      const [sx] = this.toScreen(x, 0);
      ctx.moveTo(sx, 0); ctx.lineTo(sx, this.height);
    }
    for (let y = Math.floor(br.y / step) * step; y <= tl.y; y += step) {
      const [, sy] = this.toScreen(0, y);
      ctx.moveTo(0, sy); ctx.lineTo(this.width, sy);
    }
    ctx.stroke();
    // Eixos do frame (a partida do robô).
    const [ox, oy] = this.toScreen(0, 0);
    ctx.lineWidth = 2;
    ctx.strokeStyle = 'rgba(229,72,77,0.8)';
    ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(ox + this.scale, oy); ctx.stroke();
    ctx.strokeStyle = 'rgba(47,191,79,0.8)';
    ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(ox, oy - this.scale); ctx.stroke();
  }

  drawMap() {
    if (!this.map || !this.map.image.complete) return;
    const { info, image } = this.map;
    const ctx = this.ctx;
    const [x, y, yaw] = info.origin;
    const [sx, sy] = this.toScreen(x, y);
    ctx.save();
    ctx.translate(sx, sy);
    ctx.rotate(-yaw);
    ctx.globalAlpha = 0.9;
    ctx.imageSmoothingEnabled = false;
    // A imagem tem o y máximo na primeira linha: canto de baixo na origem.
    const w = info.width * info.resolution * this.scale;
    const h = info.height * info.resolution * this.scale;
    ctx.drawImage(image, 0, -h, w, h);
    ctx.restore();
  }

  drawScan() {
    const scan = this.state && this.state.scan;
    if (!scan || !scan.length) return;
    const ctx = this.ctx;
    ctx.fillStyle = '#ff5a5f';
    const size = Math.max(2, Math.min(4, this.scale * 0.05));
    for (let i = 0; i < scan.length; i += 2) {
      const [sx, sy] = this.toScreen(scan[i], scan[i + 1]);
      ctx.fillRect(sx - size / 2, sy - size / 2, size, size);
    }
  }

  drawPlan() {
    const plan = this.state && this.state.plan;
    if (!plan || plan.length < 4) return;
    const ctx = this.ctx;
    ctx.strokeStyle = '#4fc3f7';
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    for (let i = 0; i < plan.length; i += 2) {
      const [sx, sy] = this.toScreen(plan[i], plan[i + 1]);
      if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
    }
    ctx.stroke();
  }

  drawRoute(points, preview) {
    if (!points.length) return;
    const ctx = this.ctx;
    ctx.save();
    ctx.lineWidth = 2;
    ctx.strokeStyle = preview ? 'rgba(255,255,255,0.55)' : 'rgba(255,255,255,0.75)';
    if (preview) ctx.setLineDash([6, 5]);
    ctx.beginPath();
    points.forEach((p, i) => {
      const [sx, sy] = this.toScreen(p.x, p.y);
      if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
    });
    ctx.stroke();
    ctx.setLineDash([]);
    points.forEach((p, i) => {
      const [sx, sy] = this.toScreen(p.x, p.y);
      if (p.check && p.check.suggestion) {
        const [gx, gy] = this.toScreen(p.check.suggestion[0], p.check.suggestion[1]);
        ctx.strokeStyle = STATUS_COLORS.invalido;
        ctx.setLineDash([3, 3]);
        ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(gx, gy); ctx.stroke();
        ctx.setLineDash([]);
        ctx.beginPath(); ctx.arc(gx, gy, 6, 0, 2 * Math.PI); ctx.stroke();
      }
      const bad = p.check && !['ok', 'desconhecido'].includes(p.check.status);
      let color;
      if (preview) color = bad ? STATUS_COLORS.invalido : '#cfd3da';
      else if (bad && (p.status === 'pendente' || !p.status)) color = STATUS_COLORS.invalido;
      else color = STATUS_COLORS[p.status] || STATUS_COLORS.pendente;
      ctx.fillStyle = color;
      ctx.beginPath(); ctx.arc(sx, sy, preview ? 5 : 8, 0, 2 * Math.PI); ctx.fill();
      if (preview) {
        // Contorno escuro: a prévia também aparece sobre o mapa claro.
        ctx.strokeStyle = '#3a3f47';
        ctx.lineWidth = 1.5;
        ctx.stroke();
      }
      if (bad) {
        ctx.strokeStyle = STATUS_COLORS.invalido;
        ctx.lineWidth = 2;
        ctx.beginPath(); ctx.arc(sx, sy, preview ? 9 : 12, 0, 2 * Math.PI); ctx.stroke();
      }
      if (!preview) {
        ctx.fillStyle = '#0d0f12';
        ctx.font = 'bold 10px system-ui, sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(String(i + 1), sx, sy + 0.5);
      }
    });
    ctx.restore();
  }

  // Ponto sendo arrastado: a pegada do robô no lugar (verde: cabe e chega;
  // roxo: não; cinza: área não mapeada ou ainda conferindo) e o motivo.
  drawProbe() {
    const d = this.drag;
    if (!d || d.kind !== 'point' || !d.moved) return;
    const p = this[d.list][d.index];
    if (!p) return;
    const probe = this.probe;
    const fresh = probe && Math.hypot(probe.x - p.x, probe.y - p.y) < 0.02;
    let color = '#8a8f98';
    let text = 'conferindo…';
    if (probe && fresh) {
      if (probe.status === 'ok') { color = STATUS_COLORS.atingido; text = probe.text || 'o robô cabe e chega aqui'; }
      else if (probe.status === 'desconhecido') { text = probe.text || 'área não mapeada'; }
      else { color = STATUS_COLORS.invalido; text = probe.text || probe.status; }
    }
    const ctx = this.ctx;
    const [sx, sy] = this.toScreen(p.x, p.y);
    ctx.save();
    // Pegada do robô (1,2 x 1,12 m) olhando na direção de saída.
    const heading = this.dragHeading(d);
    ctx.translate(sx, sy);
    ctx.rotate(-heading);
    const l = ROBOT_LENGTH * this.scale;
    const w = ROBOT_WIDTH * this.scale;
    ctx.globalAlpha = 0.35;
    ctx.fillStyle = color;
    ctx.fillRect(-l / 2, -w / 2, l, w);
    ctx.globalAlpha = 1;
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.strokeRect(-l / 2, -w / 2, l, w);
    ctx.restore();
    if (probe && fresh && probe.suggestion) {
      const [gx, gy] = this.toScreen(probe.suggestion[0], probe.suggestion[1]);
      ctx.save();
      ctx.strokeStyle = STATUS_COLORS.atingido;
      ctx.setLineDash([3, 3]);
      ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(gx, gy); ctx.stroke();
      ctx.setLineDash([]);
      ctx.beginPath(); ctx.arc(gx, gy, 6, 0, 2 * Math.PI); ctx.stroke();
      ctx.restore();
    }
    // Etiqueta com o motivo, ao lado do ponto.
    ctx.save();
    ctx.font = '12px system-ui, sans-serif';
    const label = `${p.x.toFixed(2)}, ${p.y.toFixed(2)} · ${text}`;
    const tw = ctx.measureText(label).width;
    let lx = sx + 16;
    let ly = sy - 26;
    if (lx + tw + 12 > this.width) lx = sx - 16 - tw - 12;
    if (ly < 4) ly = sy + 14;
    ctx.fillStyle = 'rgba(13,15,18,0.85)';
    ctx.fillRect(lx, ly, tw + 12, 20);
    ctx.strokeStyle = color;
    ctx.lineWidth = 1;
    ctx.strokeRect(lx + 0.5, ly + 0.5, tw + 11, 19);
    ctx.fillStyle = '#e6e8eb';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'middle';
    ctx.fillText(label, lx + 6, ly + 10.5);
    ctx.restore();
  }

  // Direção de saída do ponto arrastado, como o waypoint_mission completa.
  dragHeading(d) {
    const points = this[d.list];
    const p = points[d.index];
    if (p.yaw !== null && p.yaw !== undefined) return p.yaw;
    const next = points[d.index + 1];
    if (next) return Math.atan2(next.y - p.y, next.x - p.x);
    const prev = points[d.index - 1] || (this.state && this.state.pose
      ? { x: this.state.pose[0], y: this.state.pose[1] } : { x: 0, y: 0 });
    return Math.atan2(p.y - prev.y, p.x - prev.x);
  }

  drawRect() {
    if (!this.rect) return;
    const ctx = this.ctx;
    const [ax, ay] = this.toScreen(this.rect.a.x, this.rect.a.y);
    const [bx, by] = this.toScreen(this.rect.b.x, this.rect.b.y);
    ctx.save();
    ctx.strokeStyle = '#f0a020';
    ctx.fillStyle = 'rgba(240,160,32,0.08)';
    ctx.setLineDash([8, 6]);
    ctx.lineWidth = 1.5;
    ctx.fillRect(Math.min(ax, bx), Math.min(ay, by), Math.abs(bx - ax), Math.abs(by - ay));
    ctx.strokeRect(Math.min(ax, bx), Math.min(ay, by), Math.abs(bx - ax), Math.abs(by - ay));
    ctx.setLineDash([]);
    ctx.fillStyle = '#f0a020';
    ctx.beginPath(); ctx.arc(ax, ay, 5, 0, 2 * Math.PI); ctx.fill();
    ctx.restore();
  }

  drawRobot() {
    const pose = this.state && this.state.pose;
    if (!pose) return;
    const ctx = this.ctx;
    const [sx, sy] = this.toScreen(pose[0], pose[1]);
    ctx.save();
    ctx.translate(sx, sy);
    ctx.rotate(-pose[2]);
    const l = ROBOT_LENGTH * this.scale;
    const w = ROBOT_WIDTH * this.scale;
    const source = this.state.source;
    ctx.fillStyle = source === 'teleop' ? 'rgba(240,160,32,0.55)' : 'rgba(42,127,255,0.45)';
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.5;
    ctx.fillRect(-l / 2, -w / 2, l, w);
    ctx.strokeRect(-l / 2, -w / 2, l, w);
    // Seta da frente (mínimo de 14 px para aparecer de longe).
    const a = Math.max(14, l * 0.6);
    ctx.fillStyle = '#ffffff';
    ctx.beginPath();
    ctx.moveTo(a, 0);
    ctx.lineTo(a * 0.35, -a * 0.3);
    ctx.lineTo(a * 0.35, a * 0.3);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  drawScaleBar(styles) {
    const ctx = this.ctx;
    const meters = [0.5, 1, 2, 5, 10, 20, 50].find((m) => m * this.scale > 60) || 100;
    const px = meters * this.scale;
    const x = 16;
    const y = this.height - 18;
    ctx.strokeStyle = styles.getPropertyValue('--text').trim() || '#e6e8eb';
    ctx.fillStyle = ctx.strokeStyle;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(x, y - 5); ctx.lineTo(x, y); ctx.lineTo(x + px, y); ctx.lineTo(x + px, y - 5);
    ctx.stroke();
    ctx.font = '11px system-ui, sans-serif';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'bottom';
    ctx.fillText(`${meters} m`, x + 4, y - 4);
  }
}

function round(p) {
  return { x: Math.round(p.x * 100) / 100, y: Math.round(p.y * 100) / 100 };
}
