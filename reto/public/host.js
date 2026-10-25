"use strict";
// Consola del profesor. También es la vista para el proyector.

const app = document.getElementById("app");
const KEY = new URLSearchParams(location.search).get("key") || "";
let info = null;
let room = null;
let stream = null;
let render = null;
const state = { showAll: false };

async function getInfo() {
  const r = await fetch("/api/host/info", { headers: { "x-host-key": KEY } });
  if (!r.ok) throw new Error(r.status === 401 ? "Clave incorrecta. Usa la dirección completa que sale en la terminal." : "No se pudo hablar con el servidor.");
  info = await r.json();
  return info;
}
function hostDo(action, extra) { return api("/api/host", { room, action, ...extra }, KEY); }
function stopStream() { if (stream) { stream.close(); stream = null; } clearTimer(); }

function roomInfo(s) { return `Sala ${s.code} · ${s.nPlayers} ${s.nPlayers === 1 ? "jugador" : "jugadores"}`; }
function topBar(s) {
  return h("div", { class: "hostbar" },
    h("div", { class: "muted", id: "roominfo" }, roomInfo(s)),
    h("button", { class: "btn danger", type: "button", onclick: endRoom }, "Terminar sala"));
}
async function endRoom() {
  if (!confirm("¿Terminar la sala? Se pierden los puntos (descarga el CSV antes).")) return;
  await hostDo("end");
}
function actionBtn(label, action, opts = {}) {
  const err = h("span", { class: "error", role: "alert" });
  const b = h("button", { class: "btn " + (opts.cls || "") + " big", type: "button" }, label);
  b.addEventListener("click", async () => {
    b.disabled = true;
    err.textContent = "";
    try {
      const r = await hostDo(action);
      if (!r.ok) { err.textContent = r.data.error || "No se pudo."; b.disabled = false; }
    } catch { err.textContent = "Sin conexión."; b.disabled = false; }
  });
  return { b, err, el: h("div", { class: "row" }, b, err) };
}

// ------------------------------------------------------------ preparar sala
function showSetup(message) {
  stopStream();
  room = null;
  const boxes = info.steps.map((st) => {
    const cb = h("input", { type: "checkbox", value: st.step, checked: true });
    return { cb, el: h("label", null, cb, h("span", null, h("strong", null, `Paso ${st.step}: ${st.title}`), h("span", { class: "muted" }, ` (${st.count} ${st.count === 1 ? "pregunta" : "preguntas"})`))) };
  });
  const err = h("p", { class: "error", role: "alert" }, message || "");
  const create = h("button", { class: "btn big", type: "button" }, "Crear sala");
  create.addEventListener("click", async () => {
    const steps = boxes.filter((b) => b.cb.checked).map((b) => Number(b.cb.value));
    if (!steps.length) { err.textContent = "Elige al menos un paso."; return; }
    create.disabled = true;
    try {
      const r = await api("/api/rooms", { steps }, KEY);
      if (!r.ok) { err.textContent = r.data.error || "No se pudo crear la sala."; return; }
      await getInfo();
      startRoom(r.data.code);
    } catch { err.textContent = "Sin conexión con el servidor."; }
    finally { create.disabled = false; }
  });
  const setAll = (v) => boxes.forEach((b) => { b.cb.checked = v; });
  mount(app,
    h("div", { class: "brand" }, h("span", { class: "dot" }), h("b", null, "Reto en vivo · Profesor")),
    h("h1", null, "Crear una sala"),
    info.errors.length ? h("div", { class: "card" }, h("p", { class: "error" }, "Problemas con el archivo de preguntas:"), h("ul", null, info.errors.map((e) => h("li", null, e)))) : null,
    h("p", { class: "muted" }, `Elige los pasos que entran en el juego. Cada pregunta dura ${Math.round(info.durationMs / 1000)} segundos.`),
    h("div", { class: "row" },
      h("button", { class: "btn ghost", type: "button", onclick: () => setAll(true) }, "Todos"),
      h("button", { class: "btn ghost", type: "button", onclick: () => setAll(false) }, "Ninguno")),
    h("div", { class: "steps" }, boxes.map((b) => b.el)),
    create, err);
}

// ------------------------------------------------------------ sala
function startRoom(code) {
  stopStream();
  room = code;
  state.showAll = false;
  render = makeRenderer(app, { key: keyOf, build });
  stream = connectEvents(`/events?room=${encodeURIComponent(code)}&key=${encodeURIComponent(KEY)}`, (s) => {
    if (s.phase === "ended") { stopStream(); getInfo().then(() => showSetup("La sala se cerró.")).catch(() => {}); return; }
    render(s);
    const ri = document.getElementById("roominfo");
    if (ri) ri.textContent = roomInfo(s);
  }, async () => {
    stopStream();
    try { await getInfo(); showSetup("La sala ya no existe."); } catch { setConnected(false); }
  });
}

function keyOf(s) {
  switch (s.phase) {
    case "question": return `q${s.qid}`;
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
    default: return { el: h("p", null, "Sala cerrada.") };
  }
}

function joinUrls() {
  const set = new Set(info.studentUrls);
  if (!["localhost", "127.0.0.1", "[::1]"].includes(location.hostname)) set.add(location.origin + "/");
  return [...set];
}

function playerChips(list) {
  return list.map((p) => h("span", { class: "pchip" + (p.online ? "" : " off") }, p.nick));
}

function buildLobby(s) {
  const chips = h("div", { class: "pchips", "aria-live": "off" }, playerChips(s.players));
  const count = h("h2", null, `Jugadores (${s.nPlayers} de ${s.maxPlayers})`);
  const start = actionBtn("Empezar", "start");
  start.b.disabled = s.nPlayers < 1;
  const urls = joinUrls();
  return {
    el: h("div", null,
      topBar(s),
      h("h1", null, "Entra desde tu teléfono o tu computadora"),
      h("div", { class: "urls" }, urls.length ? urls.map((u) => h("div", null, h("code", null, u))) : h("div", { class: "muted" }, "No encontré una red. Conecta el Mac a Wi-Fi o abre un túnel (mira la terminal).")),
      h("p", { class: "muted" }, "y escribe este código:"),
      h("div", { class: "bigcode", "aria-label": "Código de la sala: " + s.code.split("").join(" ") }, s.code),
      count, chips,
      start.el),
    update(n) {
      chips.replaceChildren(...playerChips(n.players));
      count.textContent = `Jugadores (${n.nPlayers} de ${n.maxPlayers})`;
      start.b.disabled = n.nPlayers < 1;
    },
    announce: "Sala lista. El código es " + s.code.split("").join(" "),
  };
}

function head(s) {
  return h("div", { class: "qmeta" }, `Paso ${s.step.step}: ${s.step.title} · Pregunta ${s.qNum} de ${s.qTotal}`);
}

function buildQuestion(s) {
  const counter = h("div", { class: "counter", role: "status" });
  const side = h("div");
  const paint = (n) => {
    counter.textContent = `${n.answered} de ${n.nPlayers} han respondido`;
    side.replaceChildren(h("h3", null, "Puestos ahora"), rankList(n.top5));
  };
  paint(s);
  const skip = actionBtn("Saltar", "skip", { cls: "ghost" });
  const opts = h("div", { class: "opts" }, s.options.map((t, i) => optionEl(i, t, "div")));
  return {
    el: h("div", null,
      topBar(s),
      head(s),
      h("div", { class: "hostgrid two" },
        h("div", null, timerEl(s.startedAt, s.durationMs), h("h2", { class: "qtext" }, s.q), opts,
          h("div", { class: "row" }, counter, h("span", { class: "grow" }), skip.el)),
        side)),
    update: paint,
    announce: `Pregunta ${s.qNum} de ${s.qTotal}. ${s.q}`,
  };
}

function boardTable(lines, opts = {}) {
  const shown = opts.limit && !state.showAll ? lines.slice(0, opts.limit) : lines;
  return h("table", { class: "board" },
    h("thead", null, h("tr", null,
      h("th", null, "Puesto"), h("th", null, "Jugador"),
      opts.gain ? h("th", { class: "num" }, "Esta pregunta") : null,
      opts.step ? h("th", { class: "num" }, "Este paso") : null,
      h("th", { class: "num" }, "Total"))),
    h("tbody", null, shown.map((l) => h("tr", null,
      h("td", null, l.rank + "º"), h("td", null, l.nick),
      opts.gain ? h("td", { class: "num gain" }, l.gain ? "+" + fmt(l.gain) : "0") : null,
      opts.step ? h("td", { class: "num" }, fmt(l.stepPts)) : null,
      h("td", { class: "num" }, fmt(l.total))))));
}

function buildReveal(s) {
  const last = s.qNum >= s.qTotal;
  const next = actionBtn(last ? "Ver resumen del paso" : "Siguiente pregunta", "next");
  const boardBox = h("div");
  const paintBoard = (n) => {
    const more = n.board.length > 10
      ? h("button", { class: "btn ghost", type: "button", onclick: () => { state.showAll = !state.showAll; paintBoard(n); } }, state.showAll ? "Mostrar solo 10" : `Mostrar todos (${n.board.length})`)
      : null;
    mount(boardBox, h("h3", null, `Respondieron ${n.answered} de ${n.nPlayers}`), boardTable(n.board, { gain: true, limit: 10 }), more);
  };
  paintBoard(s);
  const opts = h("div", { class: "opts" }, s.options.map((t, i) =>
    optionEl(i, t, "div", (e) => { if (i === s.answer) markOption(e, "right"); else e.classList.add("dim"); })));
  return {
    el: h("div", null,
      topBar(s),
      head(s),
      h("div", { class: "hostgrid two" },
        h("div", null,
          h("h2", { class: "qtext" }, s.q), opts,
          h("div", { class: "why" }, h("p", null, s.why), s.ref ? h("p", { class: "ref" }, "Repasa: " + s.ref) : null),
          chartEl(s.options, s.counts, s.answer),
          next.el),
        boardBox)),
    update(n) { paintBoard(n); },
    announce: `Respuesta correcta: ${LETTERS[s.answer]}. ${s.why}`,
  };
}

function buildSummary(s) {
  const lastStep = s.step.pos >= s.nSteps;
  const next = actionBtn(lastStep ? "Ver podio final" : "Siguiente paso", "next");
  return {
    el: h("div", null,
      topBar(s),
      h("h1", null, `Paso ${s.step.step} terminado: ${s.step.title}`),
      h("p", { class: "muted" }, `Los 5 primeros después de ${s.qTotal} preguntas de este paso.`),
      boardTable(s.top5, { step: true }),
      next.el),
    announce: `Paso ${s.step.step} terminado. Va primero ${s.top5[0] ? s.top5[0].nick : "nadie"}.`,
  };
}

function buildFinal(s) {
  const csv = h("a", { class: "btn big", href: `/api/host/csv?room=${encodeURIComponent(s.code)}&key=${encodeURIComponent(KEY)}`, download: `reto-${s.code}.csv` }, "Descargar resultados (CSV)");
  return {
    el: h("div", null,
      topBar(s),
      h("h1", null, "¡Podio final!"),
      podiumEl(s.podium),
      h("div", { class: "row" }, csv),
      h("h2", null, "Clasificación completa"),
      boardTable(s.board)),
    announce: `Juego terminado. Ganó ${s.podium[0] ? s.podium[0].nick : "nadie"}.`,
  };
}

// ------------------------------------------------------------ arranque
(async function init() {
  if (!KEY) { mount(app, h("h1", null, "Falta la clave"), h("p", null, "Abre la dirección completa que sale en la terminal (termina en ?key=...).")); return; }
  try {
    await getInfo();
  } catch (e) {
    mount(app, h("h1", null, "No se puede abrir"), h("p", { class: "error" }, e.message));
    return;
  }
  if (info.rooms.length) {
    const latest = info.rooms.reduce((a, b) => (b.createdAt > a.createdAt ? b : a));
    startRoom(latest.code);
  } else {
    showSetup();
  }
})();
