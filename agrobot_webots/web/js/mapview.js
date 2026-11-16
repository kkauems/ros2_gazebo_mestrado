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

  waypointAt(px, py) {
    for (let i = this.waypoints.length - 1; i >= 0; i -= 1) {
      const [sx, sy] = this.toScreen(this.waypoints[i].x, this.waypoints[i].y);
      if (Math.hypot(sx - px, sy - py) < 12) return i;
    }
    return -1;
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
      const i = this.waypointAt(px, py);
      if (i >= 0) this.fire('remove', i);
    });
    c.addEventListener('pointerdown', (e) => {
      if (e.button !== 0) return;
      c.setPointerCapture(e.pointerId);
      const [px, py] = pos(e);
      const world = this.toWorld(px, py);
      const index = this.tool === 'add' ? this.waypointAt(px, py) : -1;
      if (index >= 0) {
        this.drag = { kind: 'waypoint', index, moved: false };
      } else if (this.tool === 'sweep') {
        this.rect = { a: world, b: world };
        this.drag = { kind: 'rect' };
      } else {
        this.drag = {
          kind: 'pan', px, py, center: { ...this.center }, moved: false, world,
        };
      }
    });
    c.addEventListener('pointermove', (e) => {
      const [px, py] = pos(e);
      const world = this.toWorld(px, py);
      this.hover = world;
      this.fire('hover', world);
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
      } else if (this.drag.kind === 'waypoint') {
        this.drag.moved = true;
        this.waypoints[this.drag.index] = { ...this.waypoints[this.drag.index], ...round(world) };
      } else if (this.drag.kind === 'rect') {
        this.rect.b = world;
        this.fire('rect', this.rect, false);
      }
    });
    c.addEventListener('pointerup', (e) => {
      const drag = this.drag;
      this.drag = null;
      if (!drag) return;
      const [px, py] = pos(e);
      const world = this.toWorld(px, py);
      if (drag.kind === 'waypoint') {
        if (drag.moved) this.fire('move', drag.index, round(world));
      } else if (drag.kind === 'rect') {
        this.rect.b = world;
        this.fire('rect', this.rect, true);
      } else if (!drag.moved) {
        if (this.tool === 'add') this.fire('add', round(world));
        else if (this.tool === 'goto') this.fire('goto', round(world));
      }
    });
    c.addEventListener('pointerleave', () => { this.hover = null; this.fire('hover', null); });
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
      if (!preview && p.check && p.check.suggestion) {
        const [gx, gy] = this.toScreen(p.check.suggestion[0], p.check.suggestion[1]);
        ctx.strokeStyle = STATUS_COLORS.invalido;
        ctx.setLineDash([3, 3]);
        ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(gx, gy); ctx.stroke();
        ctx.setLineDash([]);
        ctx.beginPath(); ctx.arc(gx, gy, 6, 0, 2 * Math.PI); ctx.stroke();
      }
      const bad = !preview && p.check && !['ok', 'desconhecido'].includes(p.check.status);
      const color = preview ? '#cfd3da' : (bad && p.status === 'pendente'
        ? STATUS_COLORS.invalido : STATUS_COLORS[p.status] || STATUS_COLORS.pendente);
      ctx.fillStyle = color;
      ctx.beginPath(); ctx.arc(sx, sy, preview ? 4 : 8, 0, 2 * Math.PI); ctx.fill();
      if (bad) {
        ctx.strokeStyle = STATUS_COLORS.invalido;
        ctx.lineWidth = 2;
        ctx.beginPath(); ctx.arc(sx, sy, 12, 0, 2 * Math.PI); ctx.stroke();
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
