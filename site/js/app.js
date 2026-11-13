// Interface: painel lateral (apresentação, nós, próximos passos,
// resultados), grafo dos nós e controles da cena 3D.

(function () {
  const $ = (s, r) => (r || document).querySelector(s);
  const $$ = (s, r) => Array.from((r || document).querySelectorAll(s));
  const esc = (t) => String(t).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
  const NO = Object.fromEntries(window.NOS.map(n => [n.id, n]));
  const CAMADAS = [
    ['rotulos', 'Rótulos'], ['lidar', 'Raios do LiDAR'], ['rgbd', 'Visão da RGB-D'],
    ['camdados', 'Visão da câmera de aquisição'], ['tf', 'Frames (TF)'],
    ['trilhas', 'Trilhas (real × estimada)'], ['fantasma', 'Pose estimada'],
    ['waypoints', 'Waypoints'], ['caminho', 'Caminho do Nav2'], ['costmap', 'Costmap local'],
  ];
  const estado = { passo: 0, no: null, aba: 'apresentacao' };

  // ------------------------------------------------------------------ abas
  function abrirAba(aba) {
    estado.aba = aba;
    $$('.aba').forEach(b => b.classList.toggle('ativa', b.dataset.aba === aba));
    $$('.conteudo-aba').forEach(c => { c.hidden = c.id !== 'aba-' + aba; });
  }

  function abrirVista(v) {
    $$('.vista-btn').forEach(b => b.classList.toggle('ativa', b.dataset.vista === v));
    $('#cena3d').hidden = v !== '3d';
    $('#grafo').hidden = v !== 'grafo';
    $('#controles3d').hidden = v !== '3d';
  }

  // ----------------------------------------------------------- apresentação
  function mostrarPasso(i) {
    const R = window.ROTEIRO;
    estado.passo = Math.max(0, Math.min(R.length - 1, i));
    const p = R[estado.passo];
    $('#passo-num').textContent = `${estado.passo + 1} / ${R.length}`;
    $('#passo-titulo').textContent = p.titulo;
    $('#passo-texto').innerHTML = p.texto;
    $$('#passo-pontos button').forEach((b, k) => b.classList.toggle('ativo', k === estado.passo));
    $('#passo-ant').disabled = estado.passo === 0;
    $('#passo-prox').disabled = estado.passo === R.length - 1;
    if (p.mundo) { Cena.definirMundo(p.mundo); $('#sel-mundo').value = p.mundo; }
    if (p.metodo) { Cena.definirMetodo(p.metodo); $('#sel-metodo').value = p.metodo; }
    atualizarMetodoVisivel();
    Cena.definirCamadas(p.camadas || []);
    sincronizarCamadas();
    Cena.destacar(p.destaque || []);
    Cena.irPara(p.camera || 'geral');
    Cena.tocar(!p.parado);
    atualizarPlay();
    marcarNoGrafo(p.no || null);
    estado.no = p.no || null;
  }

  function montarApresentacao() {
    const pontos = $('#passo-pontos');
    window.ROTEIRO.forEach((p, k) => {
      const b = document.createElement('button');
      b.title = p.titulo;
      b.setAttribute('aria-label', `Passo ${k + 1}: ${p.titulo}`);
      b.onclick = () => mostrarPasso(k);
      pontos.appendChild(b);
    });
    $('#passo-ant').onclick = () => mostrarPasso(estado.passo - 1);
    $('#passo-prox').onclick = () => mostrarPasso(estado.passo + 1);
    $('#passo-abrir').onclick = () => {
      const p = window.ROTEIRO[estado.passo];
      if (p.aba) abrirAba(p.aba);
      else if (p.no) { abrirAba('nos'); mostrarNo(p.no); }
    };
  }

  // -------------------------------------------------------------------- nós
  function lista(itens) {
    return '<ul>' + itens.map(t => `<li><code>${esc(t)}</code></li>`).join('') + '</ul>';
  }

  function mostrarNo(id) {
    const n = NO[id];
    estado.no = id;
    const G = window.GRUPOS[n.grupo];
    const partes = n.partes.map(p => `<button class="tag-parte" data-parte="${p}">${window.PARTES[p]}</button>`).join(' ');
    $('#nos-detalhe').innerHTML = `
      <button class="voltar" id="nos-voltar">← todos os nós</button>
      <div class="grupo-rotulo" style="--c:${G.cor}">${G.nome}</div>
      <h2>${n.nome}</h2>
      <p class="meta"><b>Pacote:</b> ${n.pacote}<br><b>Sobe quando:</b> <code>${esc(n.quando)}</code></p>
      <p>${n.faz}</p>
      <div class="es"><div><h4>Entra</h4>${lista(n.entradas)}</div><div><h4>Sai</h4>${lista(n.saidas)}</div></div>
      <h4>Por quê</h4><p>${n.porque}</p>
      ${partes ? `<h4>Onde age no robô</h4><p>${partes}</p>` : ''}
      <p class="arquivo"><b>Arquivo:</b> <code>${esc(n.arquivo)}</code></p>`;
    $('#nos-detalhe').hidden = false;
    $('#nos-lista').hidden = true;
    $('#nos-voltar').onclick = fecharNo;
    $$('#nos-detalhe .tag-parte').forEach(b => { b.onclick = () => mostrarParte(b.dataset.parte); });
    Cena.destacar(n.partes);
    const cam = new Set(Cena.camadas());
    n.camadas.forEach(c => cam.add(c));
    cam.add('rotulos');
    Cena.definirCamadas([...cam]);
    if (n.metodo && Cena.mundo() === 'plantacao') { Cena.definirMetodo(n.metodo); $('#sel-metodo').value = n.metodo; }
    sincronizarCamadas();
    marcarNoGrafo(id);
  }

  function fecharNo() {
    estado.no = null;
    $('#nos-detalhe').hidden = true;
    $('#nos-lista').hidden = false;
    Cena.destacar([]);
    marcarNoGrafo(null);
  }

  function mostrarParte(parte) {
    abrirAba('nos');
    const usam = window.NOS.filter(n => n.partes.includes(parte) && n.id !== 'webots');
    $('#nos-detalhe').innerHTML = `
      <button class="voltar" id="nos-voltar">← todos os nós</button>
      <div class="grupo-rotulo" style="--c:#ffb627">Parte do robô</div>
      <h2>${window.PARTES[parte]}</h2>
      <p>${DESCRICAO_PARTE[parte]}</p>
      <h4>Nós que usam esta parte</h4>
      <div class="lista-nos">${usam.map(botaoNo).join('')}</div>`;
    $('#nos-detalhe').hidden = false;
    $('#nos-lista').hidden = true;
    $('#nos-voltar').onclick = fecharNo;
    ligarBotoesNo($('#nos-detalhe'));
    Cena.destacar([parte]);
    const cam = new Set(Cena.camadas());
    cam.add('rotulos');
    if (['lidar', 'rgbd', 'camdados'].includes(parte)) cam.add(parte);
    Cena.definirCamadas([...cam]);
    sincronizarCamadas();
    marcarNoGrafo(null, usam.map(n => n.id));
  }

  const DESCRICAO_PARTE = {
    chassi: 'Quadro de alumínio do CAD (malha <code>chassis.stl</code>), 37 kg no modelo. Colisão em caixa de 1,2 × 0,8 × 0,35 m. O <code>base_link</code> fica no centro, 0,24 m acima do chão.',
    rodas: 'Quatro rodas de 0,22 m de raio com 16 garras, bitola de 0,96 m, tração skid-steer. Os encoders dão a odometria das rodas (<code>/wheel/odom</code>), que escorrega nas curvas: por isso o yaw vem da IMU.',
    lidar: 'LiDAR de um plano, 0,65 m à frente e ~13 cm do chão: 172°, 300 raios, 0,1 a 8 m. Baixo para ver caixas e pedras de 0,2 m; mais baixo que isso o plano batia no chão quando o robô inclinava.',
    rgbd: 'Câmera RGB-D como a Intel RealSense D455: 0,45 m à frente, ~0,61 m do chão, 15° para baixo, 424 × 240, 87°, profundidade de 0,5 a 6 m, ~10 Hz. É a entrada do VSLAM.',
    camdados: 'Câmera RGB de aquisição: no centro do robô, ~0,90 m do chão, olhando para a esquerda. 1280 × 960, 90°, uma foto a cada ~1 s. Cada foto cobre ~1,8 m de fileira por 1,35 m de altura. Não entra na localização.',
    imu: 'IMU (InertialUnit, Gyro e Accelerometer do Webots) 0,35 m acima do base_link. Dá o yaw e a velocidade de giro do EKF. Na simulação não tem ruído nem deriva, o que favorece o EKF.',
  };

  function botaoNo(n) {
    const G = window.GRUPOS[n.grupo];
    return `<button class="item-no" data-no="${n.id}" style="--c:${G.cor}"><span class="bolinha"></span><span><b>${n.nome}</b><small>${n.faz.replace(/<[^>]+>/g, '').split('. ')[0]}.</small></span></button>`;
  }

  function ligarBotoesNo(raiz) {
    $$('[data-no]', raiz).forEach(b => { b.onclick = () => { abrirAba('nos'); mostrarNo(b.dataset.no); }; });
  }

  function montarNos() {
    let html = '<p class="dica">Clique num nó (aqui, no grafo ou numa parte do robô no 3D) para ver o que ele faz e onde age.</p>';
    for (const [g, G] of Object.entries(window.GRUPOS)) {
      const nos = window.NOS.filter(n => n.grupo === g);
      if (!nos.length) continue;
      html += `<h3 class="grupo" style="--c:${G.cor}">${G.nome}</h3><div class="lista-nos">${nos.map(botaoNo).join('')}</div>`;
    }
    $('#nos-lista').innerHTML = html;
    ligarBotoesNo($('#nos-lista'));
  }

  // ----------------------------------------------------------------- grafo
  const W = 166, H = 44;
  function caminhoLigacao(a, b) {
    const ax = a.x, ay = a.y, bx = b.x, by = b.y;
    if (a.id === 'mux' && b.id === 'driver') {
      return `M ${ax + W} ${ay + H / 2} H ${ax + W + 12} V 648 H ${bx - 16} V ${by + 30} H ${bx}`;
    }
    if (ax === bx) {
      if (Math.abs(by - ay) <= 100) {
        return by > ay ? `M ${ax + W / 2} ${ay + H} V ${by}` : `M ${ax + W / 2} ${ay} V ${by + H}`;
      }
      const lado = ax - 14 - (by - ay) / 40;
      return `M ${ax} ${ay + H / 2 + 6} H ${lado} V ${by + H / 2} H ${bx}`;
    }
    if (bx < ax) {
      const sx = ax + W / 2, sy = ay + H, ex = bx + W, ey = by + H / 2;
      return `M ${sx} ${sy} C ${sx} ${ey}, ${ex + 60} ${ey}, ${ex} ${ey}`;
    }
    const sx = ax + W, sy = ay + H / 2, ex = bx, ey = by + H / 2, m = (ex - sx) / 2;
    return `M ${sx} ${sy} C ${sx + m} ${sy}, ${ex - m} ${ey}, ${ex} ${ey}`;
  }

  function montarGrafo() {
    const ns = 'http://www.w3.org/2000/svg';
    const svg = $('#grafo svg');
    let html = `<defs><marker id="seta" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="context-stroke"/></marker></defs>`;
    html += '<g class="colunas">';
    for (const [x, t] of [[20, 'Webots'], [210, 'Ponte'], [400, 'Pré-processamento'], [590, 'Localização'], [780, 'Nav2 e missão'], [970, 'Comando']]) {
      html += `<text x="${x + W / 2}" y="18" text-anchor="middle">${t}</text>`;
    }
    html += '</g><g class="ligacoes">';
    window.LIGACOES.forEach(([de, para, topico, tracejada], i) => {
      const d = caminhoLigacao(NO[de], NO[para]);
      html += `<path id="lig-${i}" class="lig${tracejada ? ' tracejada' : ''}" data-de="${de}" data-para="${para}" d="${d}" marker-end="url(#seta)"/>`;
    });
    html += '</g><g class="rotulos-lig">';
    window.LIGACOES.forEach(([de, para, topico], i) => {
      html += `<text class="rot-lig" data-de="${de}" data-para="${para}" dy="-4"><textPath href="#lig-${i}" startOffset="50%" text-anchor="middle">${esc(topico)}</textPath></text>`;
    });
    html += '</g><g class="nos">';
    for (const n of window.NOS) {
      const G = window.GRUPOS[n.grupo];
      html += `<g class="no${n.grupo === 'legado' ? ' legado' : ''}" data-no="${n.id}" transform="translate(${n.x},${n.y})" tabindex="0" role="button" aria-label="${esc(n.nome)}">
        <rect width="${W}" height="${H}" rx="8" style="--c:${G.cor}"/>
        <rect width="5" height="${H}" rx="2" fill="${G.cor}"/>
        <text x="${W / 2 + 2}" y="19" text-anchor="middle" class="nome">${n.nome}</text>
        <text x="${W / 2 + 2}" y="34" text-anchor="middle" class="sub">${esc(n.quando.split(' (')[0].slice(0, 30))}</text>
      </g>`;
    }
    html += '</g>';
    svg.innerHTML = html;
    $$('#grafo .no').forEach(g => {
      const abrir = () => { abrirAba('nos'); mostrarNo(g.dataset.no); };
      g.addEventListener('click', abrir);
      g.addEventListener('keydown', e => { if (e.key === 'Enter') abrir(); });
      g.addEventListener('mouseenter', () => marcarNoGrafo(g.dataset.no, null, true));
      g.addEventListener('mouseleave', () => marcarNoGrafo(estado.no));
    });
    svg.setAttribute('xmlns', ns);
  }

  function marcarNoGrafo(id, varios, hover) {
    const alvo = new Set(varios || (id ? [id] : []));
    const ativo = alvo.size > 0;
    $('#grafo').classList.toggle('com-selecao', ativo);
    $$('#grafo .no').forEach(g => {
      g.classList.toggle('sel', alvo.has(g.dataset.no));
    });
    const vizinhos = new Set(alvo);
    $$('#grafo .lig, #grafo .rot-lig').forEach(p => {
      const on = alvo.has(p.dataset.de) || alvo.has(p.dataset.para);
      p.classList.toggle('on', on);
      if (on) { vizinhos.add(p.dataset.de); vizinhos.add(p.dataset.para); }
    });
    $$('#grafo .no').forEach(g => g.classList.toggle('viz', vizinhos.has(g.dataset.no)));
  }

  // ----------------------------------------------------- próximos passos
  function montarFuturo() {
    const rot = { feito: 'feito', agora: 'em andamento', proximo: 'próximo', futuro: 'futuro' };
    let html = '<p class="dica">O que já funciona e o que ainda falta, a partir do manual e dos planejamentos do projeto.</p>';
    for (const bloco of window.FUTURO) {
      html += `<section class="bloco ${bloco.estado}"><h3><span class="selo ${bloco.estado}">${rot[bloco.estado]}</span>${bloco.titulo}</h3><ul>`;
      for (const [t, nota] of bloco.itens) {
        html += `<li><span class="check"></span><span>${esc(t)}${nota ? `<small>${esc(nota)}</small>` : ''}</span></li>`;
      }
      html += '</ul></section>';
    }
    $('#aba-futuro').innerHTML = html;
  }

  // ------------------------------------------------------------ resultados
  function montarResultados() {
    const max = 3.0;
    let html = `<p class="dica">Passo 3: 3 rodadas de cada caso com cada método, sensores sem ruído (Webots na nuvem, sem GPU). <b>ATE</b> = erro médio da posição estimada ao longo de todo o percurso.</p>
      <table class="res"><thead><tr><th>Método</th><th>No alvo</th><th>ATE (m)</th><th>Erro máx.</th></tr></thead><tbody>`;
    for (const [caso, m, , alvo, ate, dp, emax] of window.RESULTADOS) {
      if (caso) html += `<tr class="caso"><td colspan="4">${caso}</td></tr>`;
      const M = window.METODOS[m];
      const larg = Math.min(100, ate / max * 100);
      html += `<tr><td><span class="bolinha" style="--c:var(${M.cor})"></span>${M.nome}</td><td>${alvo}</td>
        <td><div class="barra"><span style="width:${larg}%;--c:var(${M.cor})"></span><b>${ate.toFixed(2).replace('.', ',')}</b><small>± ${dp.toFixed(2).replace('.', ',')}</small></div></td>
        <td>${emax.toFixed(2).replace('.', ',')}</td></tr>`;
    }
    html += `</tbody></table>
      <h4>Como ler</h4>
      <ul class="leitura">
        <li>Na plantação, o VSLAM corta o erro do EKF pela metade e chega a todos os waypoints do caso 2.</li>
        <li>O SLAM por LiDAR se perde nas fileiras iguais: com o yaw errado, a entrelinha seguinte aparece torta no mapa.</li>
        <li>No caso 3 a maior parte das falhas veio da navegação na cabeceira leste e da patinagem, não da localização.</li>
        <li>"No alvo" usa a pose verdadeira: o Nav2 pode achar que chegou estando na entrelinha errada.</li>
      </ul>
      <p class="dica">No 3D (plantação), troque o método para ver a rodada real de cada um: a trilha verde é a pose verdadeira, a colorida é a estimada.</p>`;
    $('#aba-resultados').innerHTML = html;
  }

  // ------------------------------------------------------- controles 3D
  function montarControles() {
    const box = $('#camadas');
    box.innerHTML = CAMADAS.map(([id, nome]) => `<label><input type="checkbox" data-camada="${id}"> ${nome}</label>`).join('');
    $$('input', box).forEach(i => { i.onchange = () => Cena.alternarCamada(i.dataset.camada, i.checked); });
    $('#sel-mundo').onchange = (e) => { Cena.definirMundo(e.target.value); atualizarMetodoVisivel(); };
    $('#sel-metodo').onchange = (e) => Cena.definirMetodo(e.target.value);
    $('#btn-play').onclick = () => { Cena.tocar(!Cena.tocando()); atualizarPlay(); };
    $('#sel-vel').onchange = (e) => Cena.velocidade(Number(e.target.value));
    $('#tempo').oninput = (e) => Cena.irParaTempo(Number(e.target.value) / 1000 * estado.fim);
    $$('[data-camera]').forEach(b => { b.onclick = () => Cena.irPara(b.dataset.camera); });
    $('#btn-camadas').onclick = () => $('#camadas').classList.toggle('aberto');
  }

  function atualizarMetodoVisivel() {
    $('#campo-metodo').hidden = Cena.mundo() !== 'plantacao';
    $('#nota-olival').hidden = Cena.mundo() !== 'olival';
  }

  function sincronizarCamadas() {
    const c = Cena.camadas();
    $$('#camadas input').forEach(i => { i.checked = c.has(i.dataset.camada); });
  }

  function atualizarPlay() {
    const on = Cena.tocando();
    $('#btn-play').textContent = on ? '❚❚' : '▶';
    $('#btn-play').setAttribute('aria-label', on ? 'Pausar' : 'Tocar');
  }

  function fmt(t) {
    const s = Math.max(0, Math.floor(t));
    return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`;
  }

  function aoAtualizar(info) {
    estado.fim = info.fim;
    if (document.activeElement !== $('#tempo')) $('#tempo').value = Math.min(1000, info.t / info.fim * 1000);
    $('#t-texto').textContent = `${fmt(Math.min(info.t, info.fim))} / ${fmt(info.fim)}`;
    $('#wp-texto').textContent = `waypoint ${Math.min(info.wp, info.nWp)}/${info.nWp}: ${info.nomeWp}`;
    $('#erro-texto').textContent = info.erro === null ? '' : `erro agora: ${info.erro.toFixed(2).replace('.', ',')} m`;
  }

  // ---------------------------------------------------------------- início
  function iniciar() {
    if (!window.THREE || !window.THREE.OrbitControls) {
      $('#cena3d').innerHTML = '<p class="erro">Não consegui carregar o three.js (pasta vendor/).</p>';
      return;
    }
    Cena.iniciar($('#cena3d'), { aoClicarParte: mostrarParte, aoAtualizar });
    montarControles();
    montarApresentacao();
    montarNos();
    montarGrafo();
    montarFuturo();
    montarResultados();
    $$('.aba').forEach(b => { b.onclick = () => abrirAba(b.dataset.aba); });
    $$('.vista-btn').forEach(b => { b.onclick = () => abrirVista(b.dataset.vista); });
    $('#btn-tela').onclick = () => {
      if (document.fullscreenElement) document.exitFullscreen();
      else document.documentElement.requestFullscreen().catch(() => {});
    };
    $('#btn-painel').onclick = () => document.body.classList.toggle('sem-painel');
    document.addEventListener('keydown', (e) => {
      if (e.target.matches('input, select, textarea')) return;
      if (e.key === 'ArrowRight' || e.key === 'PageDown') { abrirAba('apresentacao'); mostrarPasso(estado.passo + 1); e.preventDefault(); }
      if (e.key === 'ArrowLeft' || e.key === 'PageUp') { abrirAba('apresentacao'); mostrarPasso(estado.passo - 1); e.preventDefault(); }
      if (e.key === ' ') { Cena.tocar(!Cena.tocando()); atualizarPlay(); e.preventDefault(); }
      if (e.key === 'g') abrirVista($('#grafo').hidden ? 'grafo' : '3d');
      if (e.key === 'p') document.body.classList.toggle('sem-painel');
    });
    abrirAba('apresentacao');
    abrirVista('3d');
    mostrarPasso(0);
  }

  window.addEventListener('DOMContentLoaded', iniciar);
})();
