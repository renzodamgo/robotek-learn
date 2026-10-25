"use strict";
// Página de los alumnos: unirse con código + apodo y jugar.

const app = document.getElementById("app");
const chipsBar = document.getElementById("chips");
const chipNick = document.getElementById("chip-nick");
const chipTotal = document.getElementById("chip-total");
const chipRank = document.getElementById("chip-rank");
const STORE = "reto.session";

let session = null;
let stream = null;
let render = null;

function loadSession() { try { return JSON.parse(localStorage.getItem(STORE)); } catch { return null; } }
function saveSession(s) { try { localStorage.setItem(STORE, JSON.stringify(s)); } catch { /* modo privado */ } }
function clearSession() { try { localStorage.removeItem(STORE); } catch { /* nada */ } }

function stopStream() { if (stream) { stream.close(); stream = null; } clearTimer(); }

function updateChips(me) {
  if (!me) return;
  chipsBar.hidden = false;
  chipNick.textContent = me.nick;
  chipTotal.textContent = "Total: " + fmt(me.total);
  chipRank.textContent = `puesto ${me.rank} de ${me.n}`;
}

// ------------------------------------------------------------ unirse
function showJoin(message) {
  stopStream();
  render = null;
  chipsBar.hidden = true;
  const params = new URLSearchParams(location.search);
  const saved = loadSession();
  const code = h("input", { id: "code", class: "code", type: "text", inputmode: "numeric", pattern: "[0-9]*", maxlength: 4, autocomplete: "off", "aria-describedby": "err", value: params.get("code") || (saved && saved.code) || "" });
  const nick = h("input", { id: "nick", type: "text", maxlength: 16, autocomplete: "off", autocapitalize: "words", "aria-describedby": "err", value: (saved && saved.nick) || "" });
  const err = h("p", { id: "err", class: "error", role: "alert" }, message || "");
  const btn = h("button", { class: "btn wide big", type: "submit" }, "Entrar");
  const form = h("form", { class: "card", novalidate: true },
    h("div", { class: "field" }, h("label", { for: "code" }, "Código de la sala"), code),
    h("div", { class: "field" }, h("label", { for: "nick" }, "Tu apodo (hasta 16 letras)"), nick),
    btn, err);
  form.addEventListener("submit", async (ev) => {
    ev.preventDefault();
    err.textContent = "";
    const c = code.value.trim();
    const n = nick.value.trim();
    if (!/^\d{4}$/.test(c)) { err.textContent = "El código tiene 4 números."; code.focus(); return; }
    if (!n) { err.textContent = "Escribe un apodo."; nick.focus(); return; }
    btn.disabled = true;
    try {
      const prev = loadSession();
      const token = prev && prev.code === c && prev.nick.toLowerCase() === n.toLowerCase() ? prev.token : undefined;
      const r = await api("/api/join", { code: c, nick: n, token });
      if (!r.ok) { err.textContent = r.data.error || "No se pudo entrar."; return; }
      session = { code: r.data.code, nick: r.data.nick, token: r.data.token };
      saveSession(session);
      start();
    } catch {
      err.textContent = "No hay conexión con el servidor.";
    } finally {
      btn.disabled = false;
    }
  });
  mount(app,
    h("div", { class: "brand" }, h("span", { class: "dot" }), h("b", null, "Reto en vivo")),
    h("h1", null, "¡A jugar!"),
    h("p", { class: "muted" }, "Pide el código al profesor. Está en la pantalla grande."),
    form);
  (code.value ? nick : code).focus();
}

// ------------------------------------------------------------ partida
function start() {
  stopStream();
  render = makeRenderer(app, { key: keyOf, build });
  const url = `/events?room=${encodeURIComponent(session.code)}&token=${encodeURIComponent(session.token)}`;
  stream = connectEvents(url, onState, onFatal);
}

async function onFatal() {
  // El servidor rechazó la conexión: ¿se acabó la sala o hay que volver a entrar?
  stopStream();
  try {
    const r = await api("/api/join", { code: session.code, nick: session.nick, token: session.token });
    if (r.ok) { session.token = r.data.token; saveSession(session); start(); return; }
    if (r.status === 404) { clearSession(); showJoin("La sala ya no existe. Pide un código nuevo."); return; }
    showJoin(r.data.error || "No se pudo volver a entrar.");
  } catch {
    setConnected(false);
    await sleep(3000);
    onFatal();
  }
}

function onState(s) {
  updateChips(s.me);
  if (s.phase === "ended") { clearSession(); stopStream(); setConnected(true); }
  render(s);
}

function keyOf(s) {
  switch (s.phase) {
    case "question": return `q${s.qid}:${s.answered}`;
    case "reveal": return `r${s.qid}`;
    case "summary": return `s${s.step.step}`;
    default: return s.phase;
  }
}

function build(s) {
  switch (s.phase) {
    case "lobby": return buildLobby(s);
    case "question": return buildQuestion(s);
    case "reveal": return buildReveal(s);
    case "summary": return buildSummary(s);
    case "final": return buildFinal(s);
    default: return buildEnded();
  }
}

function leaveBtn(label) {
  return h("button", { class: "btn ghost", type: "button", onclick: () => { clearSession(); session = null; showJoin(); } }, label);
}

function buildLobby(s) {
  const count = h("strong", null, s.nPlayers);
  return {
    el: h("div", null,
      h("h1", null, "¡Estás dentro!"),
      h("div", { class: "card" },
        h("p", null, "Espera a que el profesor empiece el juego."),
        h("p", { class: "muted" }, "Jugadores en la sala: ", count)),
      leaveBtn("Salir de la sala")),
    update(n) { count.textContent = n.nPlayers; },
    announce: "Estás dentro. Espera a que empiece el juego.",
  };
}

function qHeader(s) {
  return h("div", { class: "qmeta" }, `Paso ${s.step.step}: ${s.step.title} · Pregunta ${s.qNum} de ${s.qTotal}`);
}

function buildQuestion(s) {
  const status = h("p", { class: "muted", role: "status" }, s.answered ? "Respuesta enviada. Espera a los demás." : "Elige una opción.");
  let sent = s.answered;
  const buttons = [];
  const opts = h("div", { class: "opts" }, s.options.map((t, i) => {
    const b = optionEl(i, t, "button", (e) => {
      if (s.answered) {
        e.disabled = true;
        e.classList.add(i === s.myChoice ? "picked" : "dim");
      } else {
        e.addEventListener("click", () => choose(i));
      }
    });
    buttons.push(b);
    return b;
  }));

  async function choose(i) {
    if (sent || clock.now() > s.startedAt + s.durationMs) return;
    sent = true;
    buttons.forEach((b, k) => { b.disabled = true; b.classList.add(k === i ? "picked" : "dim"); });
    status.textContent = "Enviando...";
    try {
      const r = await api("/api/answer", { room: session.code, token: session.token, qid: s.qid, choice: i });
      if (r.ok) status.textContent = "Respuesta enviada. Espera a los demás.";
      else status.textContent = r.data.error || "No se pudo enviar.";
    } catch {
      status.textContent = "No se pudo enviar. Revisa tu conexión.";
      sent = false;
      buttons.forEach((b) => { b.disabled = false; b.classList.remove("picked", "dim"); });
    }
  }
  function expire() {
    if (sent) return;
    buttons.forEach((b) => { b.disabled = true; b.classList.add("dim"); });
    status.textContent = "Se acabó el tiempo.";
    sent = true;
  }
  const timer = timerEl(s.startedAt, s.durationMs, expire);
  return {
    el: h("div", null, qHeader(s), timer, h("h2", { class: "qtext" }, s.q), opts, status),
    announce: `Pregunta ${s.qNum} de ${s.qTotal}. ${s.q}. Tienes ${Math.round(s.durationMs / 1000)} segundos.`,
  };
}

function buildReveal(s) {
  const r = s.result;
  let cls, title, sub, say;
  if (r.choice == null) { cls = "bad"; title = "Sin respuesta"; sub = "0 puntos"; say = "No respondiste. 0 puntos."; }
  else if (r.correct) {
    cls = "good"; title = "✓ ¡Correcto!";
    sub = `+${fmt(r.gain)} puntos` + (r.bonus ? ` (incluye racha +${r.bonus})` : "");
    say = `Correcto. Ganaste ${r.gain} puntos.`;
  } else { cls = "bad"; title = "✗ Incorrecto"; sub = "0 puntos"; say = "Incorrecto. 0 puntos."; }
  const opts = h("div", { class: "opts" }, s.options.map((t, i) =>
    optionEl(i, t, "div", (e) => {
      if (i === s.answer) markOption(e, "right");
      else if (i === r.choice) markOption(e, "wrong");
      else e.classList.add("dim");
      if (i === r.choice) e.classList.add("picked");
    })));
  return {
    el: h("div", null,
      qHeader(s),
      h("div", { class: "result " + cls }, h("div", { class: "big" }, title), h("div", null, sub)),
      h("h2", { class: "qtext" }, s.q),
      opts,
      h("div", { class: "why" }, h("p", null, s.why), s.ref ? h("p", { class: "ref" }, "Repasa: " + s.ref) : null),
      h("h3", null, "Qué eligió la clase"),
      chartEl(s.options, s.counts, s.answer),
      h("p", { class: "muted" }, "Espera a que el profesor siga.")),
    announce: say + " " + s.why,
  };
}

function buildSummary(s) {
  const me = { nick: s.me.nick, rank: s.me.rank, total: s.me.total, stepPts: s.stepPoints };
  const inTop = s.top5.some((l) => l.nick === s.me.nick);
  return {
    el: h("div", null,
      h("h1", null, `Paso ${s.step.step} terminado`),
      h("div", { class: "card" },
        h("p", { class: "qtext" }, `Paso ${s.step.step}: tus puntos en estas ${s.qTotal} preguntas: ${fmt(s.stepPoints)}. Acumulado: ${fmt(s.me.total)}.`),
        h("p", null, `Tu puesto: ${s.me.rank} de ${s.me.n}.`)),
      h("h3", null, "Los 5 primeros"),
      rankList(s.top5, { me: s.me.nick, step: true }),
      inTop ? null : [h("p", { class: "muted" }, "Tu lugar:"), rankList([me], { me: s.me.nick, step: true })],
      h("p", { class: "muted" }, "Espera a que el profesor siga.")),
    announce: `Paso ${s.step.step} terminado. Hiciste ${s.stepPoints} puntos. Estás en el puesto ${s.me.rank} de ${s.me.n}.`,
  };
}

function buildFinal(s) {
  return {
    el: h("div", null,
      h("h1", null, "¡Se acabó el juego!"),
      h("div", { class: "card" }, h("p", { class: "qtext" }, `Terminaste en el puesto ${s.me.rank} de ${s.me.n} con ${fmt(s.me.total)} puntos.`)),
      podiumEl(s.podium),
      h("h3", null, "Clasificación final"),
      rankList(s.board, { me: s.me.nick }),
      leaveBtn("Salir")),
    announce: `Se acabó el juego. Terminaste en el puesto ${s.me.rank} de ${s.me.n}.`,
  };
}

function buildEnded() {
  return {
    el: h("div", null,
      h("h1", null, "La sala se cerró"),
      h("p", null, "Gracias por jugar."),
      leaveBtn("Entrar a otra sala")),
    announce: "La sala se cerró.",
  };
}

// ------------------------------------------------------------ arranque
(async function init() {
  session = loadSession();
  if (!session || !session.code || !session.token) { showJoin(); return; }
  try {
    const r = await api("/api/join", { code: session.code, nick: session.nick, token: session.token });
    if (r.ok) { session.token = r.data.token; saveSession(session); start(); return; }
    clearSession();
    session = null;
    showJoin(r.status === 404 ? "La sala anterior ya no existe." : r.data.error);
  } catch {
    showJoin("No hay conexión con el servidor.");
  }
})();
