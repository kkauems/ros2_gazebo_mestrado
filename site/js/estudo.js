// Página de estudo: índice automático, filtro, seção atual, tema e equações.
(function () {
  const $ = (s, r = document) => r.querySelector(s);
  const $$ = (s, r = document) => [...r.querySelectorAll(s)];

  function guardar(chave, valor) {
    try { localStorage.setItem(chave, valor); } catch (e) { /* sem armazenamento */ }
  }
  function ler(chave) {
    try { return localStorage.getItem(chave); } catch (e) { return null; }
  }

  function aplicarTema(tema) {
    if (tema === 'claro') document.documentElement.dataset.tema = 'claro';
    else delete document.documentElement.dataset.tema;
  }

  function montarIndice() {
    const lista = $('#indice-lista');
    const itens = [];
    $$('#texto section[id]').forEach(sec => {
      const h2 = $('h2', sec);
      if (!h2) return;
      itens.push({ id: sec.id, texto: h2.textContent, nivel: 2 });
      $$('h3[id]', sec).forEach(h3 => itens.push({ id: h3.id, texto: h3.textContent, nivel: 3 }));
    });
    lista.innerHTML = itens.map(i =>
      `<li class="n${i.nivel}"><a href="#${i.id}" data-alvo="${i.id}">${i.texto}</a></li>`).join('');
    lista.addEventListener('click', e => {
      if (e.target.closest('a')) document.body.classList.remove('indice-aberto');
    });
    return itens;
  }

  function filtrar(termo) {
    const t = termo.trim().toLowerCase();
    $$('#indice-lista li').forEach(li => {
      const alvo = document.getElementById($('a', li).dataset.alvo);
      if (!t) { li.hidden = false; return; }
      // Procura no título e no texto da seção (até a próxima de mesmo nível).
      let texto = li.textContent.toLowerCase();
      if (alvo) texto += ' ' + (alvo.tagName === 'SECTION' ? alvo.textContent : textoAteProximo(alvo)).toLowerCase();
      li.hidden = !texto.includes(t);
    });
  }

  function textoAteProximo(h3) {
    let s = '';
    for (let n = h3.nextElementSibling; n && n.tagName !== 'H3'; n = n.nextElementSibling) s += ' ' + n.textContent;
    return s;
  }

  function seguirSecao(itens) {
    const links = new Map($$('#indice-lista a').map(a => [a.dataset.alvo, a]));
    const alvos = itens.map(i => document.getElementById(i.id)).filter(Boolean);
    let atual = null;
    function marcar() {
      const topo = 90;
      let escolhido = alvos[0];
      for (const el of alvos) {
        if (el.getBoundingClientRect().top - topo <= 0) escolhido = el;
        else break;
      }
      if (escolhido && escolhido.id !== atual) {
        if (atual && links.get(atual)) links.get(atual).classList.remove('atual');
        atual = escolhido.id;
        const a = links.get(atual);
        if (a) {
          a.classList.add('atual');
          const nav = $('#indice');
          const r = a.getBoundingClientRect(), rn = nav.getBoundingClientRect();
          if (r.top < rn.top || r.bottom > rn.bottom) a.scrollIntoView({ block: 'nearest' });
        }
      }
    }
    let pendente = false;
    window.addEventListener('scroll', () => {
      if (pendente) return;
      pendente = true;
      requestAnimationFrame(() => { pendente = false; marcar(); });
    }, { passive: true });
    marcar();
  }

  function abrirPerguntaDoLink() {
    // Um link direto para dentro de uma pergunta da banca a abre.
    const alvo = location.hash && document.getElementById(location.hash.slice(1));
    const det = alvo && alvo.closest('details');
    if (det) det.open = true;
  }

  function equacoes() {
    if (!window.renderMathInElement) return;
    window.renderMathInElement($('#texto'), {
      delimiters: [
        { left: '\\[', right: '\\]', display: true },
        { left: '\\(', right: '\\)', display: false },
      ],
      ignoredTags: ['script', 'noscript', 'style', 'textarea', 'pre', 'code', 'svg'],
      throwOnError: false,
    });
  }

  function iniciar() {
    aplicarTema(ler('estudo-tema'));
    $('#btn-tema').onclick = () => {
      const novo = document.documentElement.dataset.tema === 'claro' ? 'escuro' : 'claro';
      aplicarTema(novo);
      guardar('estudo-tema', novo);
    };
    $('#btn-indice').onclick = () => document.body.classList.toggle('indice-aberto');
    equacoes();
    const itens = montarIndice();
    $('#filtro').addEventListener('input', e => filtrar(e.target.value));
    seguirSecao(itens);
    abrirPerguntaDoLink();
    // A posição do link (#no-ekf) muda depois que as equações são desenhadas.
    if (location.hash) {
      const alvo = document.getElementById(location.hash.slice(1));
      if (alvo) alvo.scrollIntoView();
    }
  }

  // Aplica o tema antes de desenhar, para não piscar.
  aplicarTema(ler('estudo-tema'));
  document.addEventListener('DOMContentLoaded', iniciar);
})();
