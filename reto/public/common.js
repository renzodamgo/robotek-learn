"use strict";
// Utilidades compartidas por la página de alumnos y la del profesor.
// Regla: nada de HTML armado a mano. Todo texto entra con textContent, así un apodo
// como <b>hola</b> se ve tal cual y no se ejecuta.

const LETTERS = ["A", "B", "C", "D"];
const reduceMotion = window.matchMedia("(prefers-reduced-motion: reduce)");

function h(tag, attrs, ...kids) {
  const e = document.createElement(tag);
  if (attrs) {
    for (const [k, v] of Object.entries(attrs)) {
      if (v == null || v === false) continue;
      if (k === "class") e.className = v;
      else if (k.startsWith("on") && typeof v === "function") e.addEventListener(k.slice(2), v);
      else e.setAttribute(k, v === true ? "" : String(v));
    }
  }
  for (const kid of kids.flat()) {
    if (kid == null || kid === false) continue;
    e.append(kid.nodeType ? kid : String(kid));
  }
  return e;
}
function mount(root, ...kids) { root.replaceChildren(...kids.flat().filter(Boolean)); }
const fmt = (n) => Number(n).toLocaleString("es");
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

// ---- reloj: el servidor manda serverNow en cada evento
const clock = {
  offset: 0,
  sync(serverNow) { this.offset = serverNow - Date.now(); },
  now() { return Date.now() + this.offset; },
};

// ---- avisos para lectores de pantalla (poca frecuencia)
function announce(text) {
  const live = document.getElementById("live");
  if (!live) return;
  live.textContent = "";
  setTimeout(() => { live.textContent = text; }, 60);
}

function setConnected(ok) {
  const c = document.getElementById("conn");
  if (c) c.hidden = ok;
}

// ---- API
async function api(path, body, key) {
  const headers = { "content-type": "application/json" };
  if (key) headers["x-host-key"] = key;
  const r = await fetch(path, { method: "POST", headers, body: JSON.stringify(body) });
  let data = {};
  try { data = await r.json(); } catch { /* sin cuerpo */ }
  return { ok: r.ok, status: r.status, data };
}

// ---- SSE con reconexión automática (la hace el navegador)
function connectEvents(url, onState, onFatal) {
  const es = new EventSource(url);
  es.onopen = () => setConnected(true);
  es.onmessage = (ev) => {
    setConnected(true);
    try { onState(JSON.parse(ev.data)); } catch (e) { console.error(e); }
  };
  es.onerror = () => {
    setConnected(false);
    if (es.readyState === EventSource.CLOSED) onFatal();
  };
  return { close() { es.close(); } };
}

// ---- formas y opciones A B C D (forma + letra, no solo color)
const SVGNS = "http://www.w3.org/2000/svg";
function shapeSvg(i) {
  const svg = document.createElementNS(SVGNS, "svg");
  svg.setAttribute("viewBox", "0 0 24 24");
  svg.setAttribute("class", "shape");
  svg.setAttribute("aria-hidden", "true");
  let s;
  if (i === 0) { s = document.createElementNS(SVGNS, "polygon"); s.setAttribute("points", "12,3 22,21 2,21"); }
  else if (i === 1) { s = document.createElementNS(SVGNS, "polygon"); s.setAttribute("points", "12,1 23,12 12,23 1,12"); }
  else if (i === 2) { s = document.createElementNS(SVGNS, "circle"); s.setAttribute("cx", "12"); s.setAttribute("cy", "12"); s.setAttribute("r", "10"); }
  else { s = document.createElementNS(SVGNS, "rect"); s.setAttribute("x", "3"); s.setAttribute("y", "3"); s.setAttribute("width", "18"); s.setAttribute("height", "18"); s.setAttribute("rx", "2"); }
  svg.append(s);
  return svg;
}
// tag: "button" para alumnos (se puede pulsar) o "div" para el proyector.
function optionEl(i, text, tag, extra) {
  const e = h(tag, { class: "opt", "data-i": i, type: tag === "button" ? "button" : null, "aria-label": `Opción ${LETTERS[i]}: ${text}` },
    shapeSvg(i),
    h("span", { class: "letter", "aria-hidden": "true" }, LETTERS[i]),
    h("span", { class: "label" }, text),
    h("span", { class: "mark", "aria-hidden": "true" }),
  );
  if (extra) extra(e);
  return e;
}
function markOption(e, kind) {
  const mark = e.querySelector(".mark");
  if (kind === "right") { e.classList.add("right"); mark.textContent = "✓"; }
  else if (kind === "wrong") { e.classList.add("wrong"); mark.textContent = "✗"; }
}

// ---- gráfica de barras por opción
function chartEl(options, counts, answer) {
  const max = Math.max(1, ...counts);
  return h("div", { class: "chart", role: "img", "aria-label": "Respuestas: " + options.map((_, i) => `${LETTERS[i]} ${counts[i]}`).join(", ") },
    options.map((_, i) => {
      const fill = h("div", { class: "fill" });
      fill.style.width = Math.round((counts[i] / max) * 100) + "%";
      return h("div", { class: "bar" + (i === answer ? " right" : "") },
        h("span", null, LETTERS[i] + (i === answer ? " ✓" : "")),
        h("div", { class: "track" }, fill),
        h("span", { class: "n" }, counts[i]),
      );
    }));
}

// ---- lista de puestos
function rankList(lines, opts = {}) {
  return h("ol", { class: "rank" }, lines.map((l) =>
    h("li", { class: l.nick === opts.me ? "me" : null },
      h("span", { class: "pos" }, l.rank + "º"),
      h("span", { class: "nm" }, l.nick),
      h("span", { class: "pts" },
        fmt(l.total),
        opts.gain && l.gain ? h("span", { class: "gain" }, "+" + fmt(l.gain)) : null,
        opts.step ? h("span", { class: "muted" }, ` (paso: ${fmt(l.stepPts)})`) : null,
      ),
    )));
}

function podiumEl(lines) {
  const order = [lines[1], lines[0], lines[2]];
  return h("div", { class: "podium" }, order.map((l, k) => {
    const place = [2, 1, 3][k];
    if (!l) return h("div", { class: "spot p" + place });
    return h("div", { class: "spot p" + place },
      h("div", { class: "name" }, l.nick),
      h("div", { class: "block" }, h("span", null, place), h("small", null, fmt(l.total) + " pts")));
  }));
}

// ---- cuenta atrás suave (usa el reloj del servidor)
let activeTimer = null;
function timerEl(startedAt, durationMs, onZero) {
  const num = h("span", { class: "timer-num" }, Math.ceil(durationMs / 1000) + " s");
  const bar = h("div", { class: "timer-bar" });
  const root = h("div", { class: "timer", "aria-hidden": "true" },
    h("div", { class: "timer-top" }, h("span", null, "Tiempo"), num),
    h("div", { class: "timer-track" }, bar));
  activeTimer = { root, num, bar, startedAt, durationMs, onZero, zeroDone: false, said: {} };
  paintTimer();
  return root;
}
function clearTimer() { activeTimer = null; }
function paintTimer() {
  const t = activeTimer;
  if (!t) return;
  const left = Math.max(0, t.startedAt + t.durationMs - clock.now());
  const secs = Math.ceil(left / 1000);
  const frac = reduceMotion.matches ? secs * 1000 / t.durationMs : left / t.durationMs;
  t.bar.style.transform = `scaleX(${Math.min(1, frac)})`;
  const txt = secs + " s";
  if (t.num.textContent !== txt) t.num.textContent = txt;
  t.root.classList.toggle("low", left <= 5000);
  for (const mark of [10, 5]) {
    if (secs <= mark && !t.said[mark] && t.durationMs > mark * 1000 && left > 0) {
      t.said[mark] = true;
      announce(`Quedan ${mark} segundos`);
    }
  }
  if (left <= 0 && !t.zeroDone) { t.zeroDone = true; if (t.onZero) t.onZero(); }
}
function tickLoop() {
  paintTimer();
  if (reduceMotion.matches) setTimeout(tickLoop, 250);
  else requestAnimationFrame(tickLoop);
}
tickLoop();

// ---- pantalla "viva": reconstruye solo si cambia algo
function makeRenderer(root, builders) {
  let lastSig = "";
  let lastKey = "";
  let cur = null;
  return function render(s) {
    clock.sync(s.serverNow);
    const sig = JSON.stringify({ ...s, serverNow: 0 });
    if (sig === lastSig) return;
    lastSig = sig;
    const key = builders.key(s);
    if (key !== lastKey || !cur || !cur.update) {
      const changed = key !== lastKey;
      lastKey = key;
      clearTimer();
      cur = builders.build(s);
      mount(root, cur.el);
      if (changed && cur.announce) announce(cur.announce);
    } else {
      cur.update(s);
    }
  };
}
