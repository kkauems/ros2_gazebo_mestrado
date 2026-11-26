// Interface web do operador do Agrobot.
//
// Fala com o ROS pelo rosbridge:
//   /operator/state, /operator/map, /operator/*/compressed   (operator_bridge)
//   /waypoint_mission/status, /command e /probe               (waypoint_mission)
//   /cmd_vel_teleop                                          (cmd_vel_mux)
//   /data_camera/enable                                      (driver)

import { Ros } from './rosbridge.js';
import { MapView, STATUS_COLORS } from './mapview.js';
import { generateSweep, routeLength } from './sweep.js';

const $ = (id) => document.getElementById(id);

const SPEEDS = { slow: { v: 0.3, w: 0.6 }, fast: { v: 0.5, w: 1.0 } };
const STATUS_LABEL = {
  pendente: 'pendente', atual: 'indo', atingido: 'atingido', perdido: 'perdido',
  invalido: 'inválido', cancelado: 'cancelado',
};

let ros;
let mission = null;        // último /waypoint_mission/status
let state = null;          // último /operator/state
let lastMessage = '';
let lastDataImage = 0;     // hora (ms) da última imagem da câmera de aquisição
const map = new MapView($('map'));
window.agrobotMap = map;   // para depurar pelo console do navegador

// Conexão ------------------------------------------------------------------

async function start() {
  let port = 9090;
  try {
    const config = await (await fetch('api/config')).json();
    port = config.rosbridge_port || port;
  } catch (error) { /* página aberta do disco: porta padrão */ }
  const host = location.hostname || 'localhost';
  ros = new Ros(`ws://${host}:${port}`);
  ros.onConnection((connected) => {
    const chip = $('conn');
    chip.textContent = connected ? 'conectado' : 'desconectado';
    chip.className = `chip ${connected ? 'ok' : 'bad'}`;
    if (!connected) toast('Sem conexão com o robô (rosbridge). Tentando de novo…', 'error');
  });
  ros.subscribe('/operator/state', 'std_msgs/msg/String', (m) => onState(JSON.parse(m.data)),
    { throttle_rate: 150, queue_length: 1 });
  ros.subscribe('/operator/map', 'std_msgs/msg/String', (m) => onMap(JSON.parse(m.data)),
    { queue_length: 1 });
  ros.subscribe('/waypoint_mission/status', 'std_msgs/msg/String',
    (m) => onMission(JSON.parse(m.data)), { queue_length: 1 });
  ros.subscribe('/waypoint_mission/probe', 'std_msgs/msg/String',
    (m) => onProbe(JSON.parse(m.data)), { queue_length: 1 });
  updateCameraSubscriptions();
  setInterval(teleopTick, 100);
}

function command(op, extra = {}) {
  if (!ros || !ros.connected) {
    toast('Sem conexão com o robô.', 'error');
    return;
  }
  ros.publish('/waypoint_mission/command', 'std_msgs/msg/String',
    { data: JSON.stringify({ op, ...extra }) });
}

let toastTimer = null;
function toast(text, level = 'info') {
  const el = $('toast');
  el.textContent = text;
  el.className = `toast ${level}`;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.classList.add('hidden'), 5000);
}

// Estado do robô ---------------------------------------------------------

function onState(s) {
  if (!state && s.pose) map.center = { x: s.pose[0], y: s.pose[1] };
  state = s;
  map.state = s;
  const pose = s.pose;
  $('pose').textContent = pose
    ? `${s.frame} x ${pose[0].toFixed(2)}  y ${pose[1].toFixed(2)}  θ ${(pose[2] * 180 / Math.PI).toFixed(0)}°`
    : `sem TF ${s.frame} → base_link`;
  $('speed').textContent = s.v === null ? 'v —' : `v ${s.v.toFixed(2)} m/s  ω ${s.w.toFixed(2)} rad/s`;
  const labels = { teleop: 'operador', nav: 'Nav2', none: 'parado' };
  const source = $('chip-source');
  source.textContent = `comando: ${labels[s.source] || s.source}`;
  source.className = `chip ${s.source === 'teleop' ? 'warn' : s.source === 'nav' ? 'ok' : ''}`;
  $('chip-slip').classList.toggle('hidden', !s.slip);
  // Camada de segurança (collision_monitor com o LiDAR de segurança): vale
  // também para o joystick desta página.
  const safetyLabels = {
    parada: 'PARADA DE SEGURANÇA', lentidao: 'segurança: devagar',
    aproximacao: 'segurança: aproximação', limite: 'segurança: limite',
  };
  const safety = $('chip-safety');
  safety.classList.toggle('hidden', !s.safety);
  safety.textContent = safetyLabels[s.safety] || '';
  safety.className = `chip ${s.safety ? '' : 'hidden'} ${s.safety === 'parada' ? 'bad' : 'warn'}`;
  // A câmera de aquisição faz ~1 foto por segundo de simulação; com a
  // simulação lenta, até 10 s entre imagens ainda é "ligada".
  const live = Date.now() - lastDataImage < 10000;
  $('data-cam-state').textContent = live ? '(ligada)' : '(sem imagem)';
}

function onMap(m) {
  const image = new Image();
  image.onload = () => map.setMap(m, image);
  image.src = `data:image/png;base64,${m.png}`;
}

// Missão --------------------------------------------------------------------

function onMission(m) {
  const first = mission === null;
  mission = m;
  $('chip-world').textContent = `${m.world || 'mundo ?'}${m.map ? ` · mapa ${m.map}` : ''} · ${m.frame}`;
  map.locked = m.running;
  if (!map.drag || map.drag.kind !== 'point' || map.drag.list !== 'waypoints') {
    map.waypoints = m.waypoints;
  }
  const msg = $('mission-msg');
  msg.textContent = m.message || (m.running ? `Missão ${m.case} em andamento.` : 'Pronto.');
  msg.className = `msg ${m.level || ''}`;
  if ((m.level === 'error' || m.level === 'warn') && m.message !== lastMessage) {
    toast(m.message, m.level);
  }
  lastMessage = m.message;
  const busy = m.running;
  $('btn-start').disabled = busy || !m.waypoints.length;
  $('btn-start').textContent = m.starting ? 'Esperando o Nav2…' : 'Iniciar missão';
  $('btn-cancel').disabled = !busy;
  for (const id of ['btn-undo', 'btn-clear', 'btn-load', 'btn-sw-use']) {
    $(id).disabled = busy || (id === 'btn-sw-use' && !map.preview.length);
  }
  $('btn-save').disabled = !m.editable || !m.edited || !m.waypoints.length;
  $('btn-save-as').disabled = !m.waypoints.length;
  $('list-file').textContent = m.case_file
    ? `Arquivo: ${m.case_file}${m.edited ? ' · alterada, não gravada' : ''}`
      + (m.editable ? '' : ' (caso de estudo do projeto: use Salvar como)')
    : (m.waypoints.length ? 'Lista nova, ainda sem arquivo.' : '');
  // Com o formulário de edição aberto, a lista não é redesenhada (perderia
  // o que está sendo digitado); redesenha quando ele fecha.
  if (editing === null) renderWaypoints(m);
  renderCases(m);
  if (first && m.waypoints.length) map.fitWaypoints();
}

function renderWaypoints(m) {
  $('wp-count').textContent = m.waypoints.length
    ? `(${m.waypoints.length}, ${routeLength(m.waypoints).toFixed(0)} m · ${m.case})` : '';
  const list = $('wp-list');
  list.replaceChildren();
  m.waypoints.forEach((wp, i) => {
    const li = document.createElement('li');
    const bad = wp.check && !['ok', 'desconhecido'].includes(wp.check.status);
    const color = bad && wp.status === 'pendente' ? STATUS_COLORS.invalido
      : STATUS_COLORS[wp.status] || STATUS_COLORS.pendente;
    const dot = document.createElement('span');
    dot.className = 'dot';
    dot.style.background = color;
    const body = document.createElement('div');
    const yaw = wp.yaw === null ? '' : `  θ ${(wp.yaw * 180 / Math.PI).toFixed(0)}°`;
    body.innerHTML = `<div>${i + 1}. ${escapeHtml(wp.name)} <small>${STATUS_LABEL[wp.status] || wp.status}</small></div>
      <div class="coords">x ${wp.x.toFixed(2)}  y ${wp.y.toFixed(2)}${yaw}</div>`;
    if (bad || (wp.check && wp.check.status === 'desconhecido' && wp.check.text)) {
      const problem = document.createElement('div');
      problem.className = bad ? 'problem' : 'problem unknown';
      problem.textContent = wp.check.text || wp.check.status;
      body.appendChild(problem);
    }
    const actions = document.createElement('div');
    actions.className = 'actions';
    if (!m.running) {
      const last = m.waypoints.length - 1;
      for (const [label, title, enabled, fn] of [
        ['✎', 'Editar nome, posição e orientação', true, () => openEditor(i)],
        ['↑', 'Subir na ordem', i > 0, () => editList((pts) => swap(pts, i, i - 1))],
        ['↓', 'Descer na ordem', i < last, () => editList((pts) => swap(pts, i, i + 1))],
        ['+', 'Inserir um ponto depois deste', true, () => editList((pts) => insertAfter(pts, i))],
      ]) {
        const button = document.createElement('button');
        button.textContent = label;
        button.title = title;
        button.disabled = !enabled;
        button.onclick = fn;
        actions.appendChild(button);
      }
    }
    if (wp.check && wp.check.suggestion && !m.running) {
      const fix = document.createElement('button');
      fix.textContent = 'usar ponto livre';
      fix.title = `Mover para (${wp.check.suggestion[0].toFixed(2)}, ${wp.check.suggestion[1].toFixed(2)})`;
      fix.onclick = () => editList((pts) => {
        pts[i] = { ...pts[i], x: wp.check.suggestion[0], y: wp.check.suggestion[1] };
      });
      actions.appendChild(fix);
    }
    if (!m.running) {
      const remove = document.createElement('button');
      remove.textContent = '×';
      remove.title = 'Apagar';
      remove.onclick = () => editList((pts) => pts.splice(i, 1));
      actions.appendChild(remove);
    }
    body.classList.add('clickable');
    body.title = 'Centralizar no mapa';
    body.onclick = () => {
      map.follow = false;
      $('btn-follow').classList.remove('on');
      map.center = { x: wp.x, y: wp.y };
    };
    li.append(dot, body, actions);
    list.appendChild(li);
  });
}

function swap(points, a, b) {
  [points[a], points[b]] = [points[b], points[a]];
}

// Novo ponto no meio do caminho até o próximo (ou 1 m adiante, no último).
function insertAfter(points, i) {
  const p = points[i];
  const q = points[i + 1];
  let x;
  let y;
  if (q) {
    x = (p.x + q.x) / 2;
    y = (p.y + q.y) / 2;
  } else {
    const prev = points[i - 1];
    const heading = prev ? Math.atan2(p.y - prev.y, p.x - prev.x) : 0;
    x = p.x + Math.cos(heading);
    y = p.y + Math.sin(heading);
  }
  const used = new Set(points.map((w) => w.name));
  let name = `${p.name}_a`;
  for (let k = 1; used.has(name); k += 1) name = `${p.name}_${k}`;
  points.splice(i + 1, 0, { name, x: Math.round(x * 100) / 100, y: Math.round(y * 100) / 100, yaw: null });
}

// Formulário de edição de um waypoint, dentro da lista.
let editing = null;

function openEditor(i) {
  const wp = mission.waypoints[i];
  editing = i;
  const li = $('wp-list').children[i];
  const form = document.createElement('form');
  form.className = 'editor';
  const yaw = wp.yaw === null ? '' : (wp.yaw * 180 / Math.PI).toFixed(1);
  form.innerHTML = `
    <label>Nome <input name="name" value="${escapeHtml(wp.name)}"></label>
    <div class="grid3">
      <label>x (m) <input name="x" type="number" step="0.05" value="${wp.x}"></label>
      <label>y (m) <input name="y" type="number" step="0.05" value="${wp.y}"></label>
      <label>orientação (°) <input name="yaw" type="number" step="5" value="${yaw}"
        placeholder="auto"></label>
    </div>
    <p class="hint">Orientação vazia: o robô chega olhando para o próximo ponto.
      0° é +x, 90° é +y.</p>
    <div class="row"><button class="primary" type="submit">Aplicar</button>
      <button type="button" data-cancel>Cancelar</button></div>`;
  form.onsubmit = (e) => {
    e.preventDefault();
    const data = new FormData(form);
    const x = parseFloat(data.get('x'));
    const y = parseFloat(data.get('y'));
    const yawText = String(data.get('yaw')).trim();
    const yawDeg = parseFloat(yawText);
    if (!Number.isFinite(x) || !Number.isFinite(y) || (yawText && !Number.isFinite(yawDeg))) {
      toast('Valores inválidos.', 'warn');
      return;
    }
    closeEditor();
    editList((pts) => {
      pts[i] = {
        name: String(data.get('name')).trim() || pts[i].name,
        x,
        y,
        yaw: yawText ? yawDeg * Math.PI / 180 : null,
      };
    });
  };
  form.querySelector('[data-cancel]').onclick = () => {
    closeEditor();
    if (mission) renderWaypoints(mission);
  };
  li.replaceChildren(form);
  li.classList.add('editing');
  form.querySelector('input[name=x]').focus();
}

function closeEditor() {
  editing = null;
}

function renderCases(m) {
  const select = $('case-select');
  const current = select.value;
  const key = JSON.stringify(m.cases);
  if (select.dataset.key === key) return;
  select.dataset.key = key;
  select.replaceChildren();
  if (!m.cases.length) {
    select.appendChild(new Option('nenhuma para este mundo', ''));
  }
  for (const c of m.cases) {
    const label = `${c.name} (${c.waypoints} pontos${c.saved ? ', gravada' : ''})`;
    select.appendChild(new Option(label, c.name));
  }
  if (current) select.value = current;
}

function escapeHtml(text) {
  return String(text).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
}

// Lista atual da missão, editada por fn e mandada inteira ao waypoint_mission.
function editList(fn) {
  if (!mission) return;
  if (mission.running) {
    toast('Missão em andamento: cancele antes de mudar a lista.', 'warn');
    return;
  }
  const points = mission.waypoints.map((w) => ({ name: w.name, x: w.x, y: w.y, yaw: w.yaw }));
  fn(points);
  // Os nomes automáticos (wp1, wp2...) acompanham a nova ordem.
  points.forEach((p, i) => { if (/^wp\d+$/.test(p.name)) p.name = `wp${i + 1}`; });
  const name = mission.case === 'rviz' ? 'operador' : mission.case;
  command('set', { name, waypoints: points });
}

map.on('add', (p) => editList((pts) => pts.push({ name: `wp${pts.length + 1}`, x: p.x, y: p.y, yaw: null })));
// Arrastar um ponto pelo mapa (waypoint da lista ou da prévia da varredura):
// enquanto arrasta, a página pergunta ao waypoint_mission (op "probe") se o
// robô cabe e chega ali; uma pergunta por vez, sempre com a posição mais nova.
const probe = { id: 0, waiting: false, sentAt: 0, pending: null };

function probeRequest(list, i, p) {
  const points = list === 'preview' ? map.preview : map.waypoints;
  const prev = points[i - 1];
  const next = points[i + 1];
  const yaw = points[i].yaw === undefined ? null : points[i].yaw;
  return {
    x: p.x, y: p.y, yaw,
    prev: prev ? [prev.x, prev.y] : null,
    next: next ? [next.x, next.y] : null,
  };
}

function sendProbe(request) {
  if (!ros || !ros.connected) return;
  probe.id += 1;
  probe.waiting = true;
  probe.sentAt = Date.now();
  probe.pending = null;
  ros.publish('/waypoint_mission/command', 'std_msgs/msg/String',
    { data: JSON.stringify({ op: 'probe', id: probe.id, ...request }) });
}

function onProbe(reply) {
  if (reply.id !== probe.id) return;
  probe.waiting = false;
  map.probe = reply;
  if (probe.pending) sendProbe(probe.pending);
}

map.on('dragging', (list, i, p) => {
  const request = probeRequest(list, i, p);
  // Sem resposta em 2 s (nó ocupado ou fora do ar): pergunta de novo.
  if (probe.waiting && Date.now() - probe.sentAt < 2000) probe.pending = request;
  else sendProbe(request);
});
map.on('moved', (list, i, p) => {
  const last = map.probe && Math.hypot(map.probe.x - p.x, map.probe.y - p.y) < 0.02 ? map.probe : null;
  map.probe = null;
  probe.pending = null;
  if (list === 'waypoints') {
    editList((pts) => { pts[i] = { ...pts[i], x: p.x, y: p.y }; });
    if (last && !['ok', 'desconhecido'].includes(last.status)) toast(`Waypoint ${i + 1}: ${last.text}.`, 'warn');
    return;
  }
  // Prévia: o ponto fica onde foi solto, com a última checagem.
  map.preview[i] = { ...map.preview[i], x: p.x, y: p.y, check: last || undefined };
  sweepEdited = true;
  describeSweep();
});
map.on('drag-cancel', () => {
  map.probe = null;
  probe.pending = null;
  if (mission && map.drag === null) map.waypoints = mission.waypoints;
});
map.on('remove-preview', (i) => {
  map.preview.splice(i, 1);
  sweepEdited = true;
  describeSweep();
});
map.on('remove', (i) => editList((pts) => pts.splice(i, 1)));
map.on('goto', (p) => {
  if (mission && mission.running) {
    toast('Missão em andamento: cancele antes de mandar o robô a outro ponto.', 'warn');
    return;
  }
  command('set', { name: 'ir_ate', waypoints: [{ name: 'alvo', x: p.x, y: p.y }] });
  command('start');
});
map.on('hover', (p) => {
  $('cursor').textContent = p ? `cursor ${p.x.toFixed(2)}, ${p.y.toFixed(2)}` : '';
});
map.on('follow', (on) => $('btn-follow').classList.toggle('on', on));

$('btn-start').onclick = () => command('start');
$('btn-cancel').onclick = () => command('cancel');
$('btn-undo').onclick = () => command('undo');
$('btn-clear').onclick = () => {
  if (confirm('Apagar todos os waypoints da lista?')) command('clear');
};
$('btn-load').onclick = () => {
  const name = $('case-select').value;
  if (name) command('load', { case: name });
};
$('btn-save').onclick = () => command('save', { overwrite: true });
$('btn-save-as').onclick = () => {
  const name = $('save-name').value.trim();
  if (!name) {
    toast('Escreva um nome para a missão.', 'warn');
    $('save-name').focus();
    return;
  }
  command('save', { name, overwrite: $('save-overwrite').checked });
};

for (const button of $('tools').querySelectorAll('button')) {
  button.onclick = () => {
    $('tools').querySelectorAll('button').forEach((b) => b.classList.toggle('on', b === button));
    map.tool = button.dataset.tool;
    if (map.tool === 'sweep') showTab('sweep');
  };
}
$('btn-follow').onclick = () => {
  map.follow = !map.follow;
  $('btn-follow').classList.toggle('on', map.follow);
};
$('btn-fit').onclick = () => map.fitWaypoints();

// Varredura -------------------------------------------------------------

let sweepRect = null;
let sweepEdited = false;   // algum ponto da prévia foi arrastado ou apagado

function updateSweep() {
  map.preview = [];
  sweepEdited = false;
  const info = $('sw-info');
  info.className = 'msg';
  if (!sweepRect) {
    info.textContent = 'Nenhum retângulo desenhado.';
  } else {
    try {
      map.preview = generateSweep(sweepRect.a, sweepRect.b, {
        spacing: parseFloat($('sw-spacing').value),
        axis: $('sw-axis').value,
        maxLeg: parseFloat($('sw-leg').value) || 10,
      });
      describeSweep();
    } catch (error) {
      info.textContent = error.message;
      info.className = 'msg warn';
    }
  }
  $('btn-sw-use').disabled = !map.preview.length || (mission && mission.running);
}

function describeSweep() {
  const info = $('sw-info');
  const lanes = new Set(map.preview.map((p) => p.name.split('_')[0])).size;
  const bad = map.preview.filter((p) => p.check && !['ok', 'desconhecido'].includes(p.check.status)).length;
  info.textContent = `${lanes} passadas, ${map.preview.length} pontos, `
    + `${routeLength(map.preview).toFixed(0)} m de rota (fora a ida até o começo).`
    + (sweepEdited ? ' Pontos ajustados à mão (mudar as opções acima refaz a varredura).' : '')
    + (bad ? ` ${bad} ponto(s) em roxo: o robô não cabe ou não chega.` : '');
  info.className = bad ? 'msg warn' : 'msg';
  $('btn-sw-use').disabled = !map.preview.length || (mission && mission.running);
}

map.on('rect', (rect, done) => {
  sweepRect = { a: { ...rect.a }, b: { ...rect.b } };
  updateSweep();
  if (done) showTab('sweep');
});
for (const id of ['sw-spacing', 'sw-axis', 'sw-leg']) $(id).addEventListener('input', updateSweep);
$('btn-sw-use').onclick = () => {
  if (!map.preview.length) return;
  command('set', {
    name: 'varredura',
    waypoints: map.preview.map((p) => ({ name: p.name, x: p.x, y: p.y, yaw: p.yaw })),
  });
  sweepRect = null;
  map.rect = null;
  updateSweep();
  showTab('mission');
  toast('Varredura carregada na missão: confira os pontos e inicie.');
};
$('btn-sw-clear').onclick = () => {
  sweepRect = null;
  map.rect = null;
  updateSweep();
};

// Teleoperação -------------------------------------------------------------

const joy = { x: 0, y: 0, active: false };
const keys = new Set();
let hold = false;
let zerosToSend = 0;

function teleopTick() {
  if (!ros || !ros.connected) return;
  const speed = SPEEDS[document.querySelector('input[name=speed]:checked').value];
  let v = 0;
  let w = 0;
  let active = false;
  if (hold) {
    active = true;
  } else if (joy.active) {
    v = -joy.y * speed.v;
    w = -joy.x * speed.w;
    active = true;
  } else if (keys.size) {
    if (keys.has('up')) v += speed.v;
    if (keys.has('down')) v -= speed.v;
    if (keys.has('left')) w += speed.w;
    if (keys.has('right')) w -= speed.w;
    active = true;
  }
  if (active) {
    zerosToSend = 3;   // ao soltar, alguns zeros param o robô na hora
  } else if (zerosToSend > 0) {
    zerosToSend -= 1;
  } else {
    return;            // não comanda: o Nav2 volta em 0,5 s
  }
  ros.publish('/cmd_vel_teleop', 'geometry_msgs/msg/Twist', {
    linear: { x: v, y: 0, z: 0 }, angular: { x: 0, y: 0, z: w },
  });
}

function setHold(on) {
  hold = on;
  const button = $('btn-stop');
  button.classList.toggle('on', on);
  button.textContent = on ? 'SOLTAR' : 'PARAR';
  if (on) toast('Robô seguro parado: o Nav2 não comanda até você soltar.', 'warn');
}
$('btn-stop').onclick = () => setHold(!hold);

const KEYMAP = {
  ArrowUp: 'up', KeyW: 'up', ArrowDown: 'down', KeyS: 'down',
  ArrowLeft: 'left', KeyA: 'left', ArrowRight: 'right', KeyD: 'right',
};
window.addEventListener('keydown', (e) => {
  if (['INPUT', 'SELECT', 'TEXTAREA'].includes(document.activeElement.tagName)) return;
  if (e.code === 'Space') {
    e.preventDefault();
    if (!e.repeat) setHold(!hold);
    return;
  }
  const key = KEYMAP[e.code];
  if (key) {
    e.preventDefault();
    keys.add(key);
  }
});
window.addEventListener('keyup', (e) => keys.delete(KEYMAP[e.code]));
window.addEventListener('blur', () => { keys.clear(); joy.active = false; });
document.addEventListener('visibilitychange', () => {
  if (document.hidden) { keys.clear(); joy.active = false; }
});

(function setupJoystick() {
  const canvas = $('joy');
  const ctx = canvas.getContext('2d');
  const r = canvas.width / 2;
  const knob = 34;
  const draw = () => {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = 'rgba(127,127,127,0.15)';
    ctx.strokeStyle = 'rgba(127,127,127,0.6)';
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(r, r, r - 2, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(r, 10); ctx.lineTo(r, 2 * r - 10); ctx.moveTo(10, r); ctx.lineTo(2 * r - 10, r); ctx.stroke();
    ctx.fillStyle = joy.active ? '#f0a020' : '#2a7fff';
    ctx.beginPath();
    ctx.arc(r + joy.x * (r - knob), r + joy.y * (r - knob), knob, 0, 2 * Math.PI);
    ctx.fill();
  };
  const move = (e) => {
    const box = canvas.getBoundingClientRect();
    let x = (e.clientX - box.left - r) / (r - knob);
    let y = (e.clientY - box.top - r) / (r - knob);
    const n = Math.hypot(x, y);
    if (n > 1) { x /= n; y /= n; }
    joy.x = Math.abs(x) < 0.08 ? 0 : x;
    joy.y = Math.abs(y) < 0.08 ? 0 : y;
    draw();
  };
  canvas.addEventListener('pointerdown', (e) => {
    canvas.setPointerCapture(e.pointerId);
    joy.active = true;
    move(e);
  });
  canvas.addEventListener('pointermove', (e) => { if (joy.active) move(e); });
  const release = () => { joy.active = false; joy.x = 0; joy.y = 0; draw(); };
  canvas.addEventListener('pointerup', release);
  canvas.addEventListener('pointercancel', release);
  draw();
}());

// Câmeras ---------------------------------------------------------------------

const cameraSubs = new Map();

function updateCameraSubscriptions() {
  if (!ros) return;
  const tab = currentTab;
  const wanted = {
    vslam_camera: tab === 'drive' || tab === 'cameras',
    vslam_depth: tab === 'cameras',
    data_camera: tab === 'cameras',
  };
  const targets = {
    vslam_camera: () => [$('drive-cam'), $('cam-color')],
    vslam_depth: () => [$('cam-depth')],
    data_camera: () => [$('cam-data')],
  };
  for (const [key, on] of Object.entries(wanted)) {
    const sub = cameraSubs.get(key);
    if (on && !sub) {
      const unsubscribe = ros.subscribe(`/operator/${key}/compressed`,
        'sensor_msgs/msg/CompressedImage', (m) => {
          const src = `data:image/jpeg;base64,${m.data}`;
          if (key === 'data_camera') lastDataImage = Date.now();
          targets[key]().forEach((img) => { img.src = src; });
        }, { queue_length: 1 });
      cameraSubs.set(key, unsubscribe);
    } else if (!on && sub) {
      sub();
      cameraSubs.delete(key);
    }
  }
}

async function setDataCamera(on) {
  try {
    const result = await ros.callService('/data_camera/enable', 'std_srvs/srv/SetBool', { data: on });
    toast(result.message || (on ? 'Câmera ligada.' : 'Câmera desligada.'));
  } catch (error) {
    toast(`Câmera de aquisição: ${error.message}. Ela só existe com data_camera:=true.`, 'warn');
  }
}
$('btn-datacam-on').onclick = () => setDataCamera(true);
$('btn-datacam-off').onclick = () => setDataCamera(false);

// Dados gravados ------------------------------------------------------------

let dataDir = '';

async function loadData(dir) {
  dataDir = dir;
  $('data-path').textContent = `dados/${dir}`;
  const list = $('data-list');
  const thumbs = $('data-thumbs');
  list.replaceChildren();
  thumbs.replaceChildren();
  let listing;
  try {
    const response = await fetch(`api/dados?dir=${encodeURIComponent(dir)}`);
    listing = await response.json();
    if (!response.ok) throw new Error(listing.error);
  } catch (error) {
    list.innerHTML = '<li>Pasta vazia ou servidor fora do ar.</li>';
    return;
  }
  const images = [];
  for (const entry of listing.entries) {
    const li = document.createElement('li');
    const path = dir ? `${dir}/${entry.name}` : entry.name;
    const link = document.createElement('a');
    link.textContent = entry.dir ? `📁 ${entry.name}` : entry.name;
    if (entry.dir) {
      link.href = '#';
      link.onclick = (e) => { e.preventDefault(); loadData(path); };
    } else {
      link.href = `dados/${path.split('/').map(encodeURIComponent).join('/')}`;
      link.target = '_blank';
      if (/\.(png|jpe?g)$/i.test(entry.name)) images.push(link.href);
    }
    const info = document.createElement('small');
    const date = new Date(entry.mtime * 1000).toLocaleString('pt-BR', { dateStyle: 'short', timeStyle: 'short' });
    info.textContent = entry.dir ? date : `${formatSize(entry.size)} · ${date}`;
    li.append(link, info);
    list.appendChild(li);
  }
  if (listing.total > listing.entries.length) {
    list.insertAdjacentHTML('beforeend', `<li><small>… e mais ${listing.total - listing.entries.length}</small></li>`);
  }
  // Miniaturas das fotos (as mais novas; cada PNG tem 2 a 3 MB).
  images.slice(0, 12).forEach((src) => {
    const img = document.createElement('img');
    img.loading = 'lazy';
    img.src = src;
    img.onclick = () => window.open(src, '_blank');
    thumbs.appendChild(img);
  });
}

function formatSize(bytes) {
  if (bytes > 1e6) return `${(bytes / 1e6).toFixed(1)} MB`;
  if (bytes > 1e3) return `${(bytes / 1e3).toFixed(0)} kB`;
  return `${bytes} B`;
}

$('btn-data-up').onclick = () => loadData(dataDir.split('/').slice(0, -1).join('/'));
$('btn-data-refresh').onclick = () => loadData(dataDir);

// Abas ----------------------------------------------------------------------------

let currentTab = 'mission';

function showTab(name) {
  currentTab = name;
  for (const button of $('tabs').querySelectorAll('button')) {
    button.classList.toggle('on', button.dataset.tab === name);
  }
  for (const panel of document.querySelectorAll('.panel')) {
    panel.classList.toggle('hidden', panel.id !== `tab-${name}`);
  }
  updateCameraSubscriptions();
  if (name === 'data') loadData(dataDir);
}

for (const button of $('tabs').querySelectorAll('button')) {
  button.onclick = () => showTab(button.dataset.tab);
}

start();
