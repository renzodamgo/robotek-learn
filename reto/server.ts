// Reto en vivo: un quiz estilo Kahoot para la clase.
// Sin dependencias: Bun.serve + SSE + HTML/JS/CSS a mano.
// Todo el estado vive en memoria. Nunca se ejecuta nada que llegue de un cliente.
import { timingSafeEqual } from "node:crypto";
import { networkInterfaces } from "node:os";
import { join, resolve } from "node:path";

// ---------------------------------------------------------------- configuracion
const PORT = Number(process.env.PORT ?? 3000);
const QUESTIONS_PATH = process.env.RETO_QUESTIONS
  ? resolve(process.env.RETO_QUESTIONS)
  : join(import.meta.dir, "questions.json");
const PUBLIC_DIR = join(import.meta.dir, "public");
const DURATION_MS = Number(process.env.RETO_DURATION_MS ?? 30000); // solo se cambia en pruebas
const GRACE_MS = 400;
const MAX_PLAYERS = 60;
const MAX_ROOMS = 20;
const EXPIRE_MS = Number(process.env.RETO_EXPIRE_MS ?? 6 * 3600 * 1000);
const SWEEP_MS = Number(process.env.RETO_SWEEP_MS ?? 60000);
const KEEPALIVE_MS = 15000;
const MAX_BODY = 4096;

const KEY_ALPHABET = "abcdefghjkmnpqrstuvwxyz23456789";
function randomKey(n = 6): string {
  const bytes = crypto.getRandomValues(new Uint8Array(n));
  return [...bytes].map((b) => KEY_ALPHABET[b % KEY_ALPHABET.length]).join("");
}
const HOST_KEY = randomKey(6);

// ---------------------------------------------------------------- tipos
type Question = { q: string; options: string[]; answer: number; why: string; ref: string };
type Step = { step: number; title: string; questions: Question[] };
type Phase = "lobby" | "question" | "reveal" | "summary" | "final" | "ended";

interface LastResult {
  qid: number;
  choice: number | null;
  correct: boolean;
  gain: number;
  base: number;
  bonus: number;
  ms: number | null;
}
interface Player {
  token: string;
  nick: string;
  fold: string;
  joinedAt: number;
  score: number;
  streak: number;
  correct: number;
  stepPts: Record<number, number>;
  conns: number;
  lastSeen: number;
  last?: LastResult;
}
interface Client {
  kind: "host" | "player";
  player?: Player;
  ctl: ReadableStreamDefaultController<Uint8Array>;
  closed: boolean;
  released: boolean;
}
interface Room {
  code: string;
  steps: Step[];
  players: Map<string, Player>; // por token
  byFold: Map<string, Player>;
  phase: Phase;
  stepPos: number;
  qPos: number;
  qid: number;
  startedAt: number;
  durationMs: number;
  answers: Map<Player, { choice: number; at: number }>;
  counts: number[];
  timer: ReturnType<typeof setTimeout> | null;
  clients: Set<Client>;
  lastActivity: number;
  createdAt: number;
}

const rooms = new Map<string, Room>();
const encoder = new TextEncoder();

// ---------------------------------------------------------------- preguntas
function validateQuestions(data: any): { steps: Step[]; errors: string[] } {
  const errors: string[] = [];
  const steps: Step[] = [];
  if (!data || data.version !== 1 || !Array.isArray(data.steps)) {
    return { steps, errors: ['El archivo debe tener {"version":1,"steps":[...]}'] };
  }
  const seen = new Set<number>();
  for (const s of data.steps) {
    const where = `paso ${s?.step}`;
    if (!Number.isInteger(s?.step) || s.step < 0 || s.step > 99) { errors.push(`${where}: "step" no es un número válido`); continue; }
    if (seen.has(s.step)) { errors.push(`${where}: repetido`); continue; }
    seen.add(s.step);
    if (typeof s.title !== "string" || !s.title.trim()) errors.push(`${where}: falta "title"`);
    if (!Array.isArray(s.questions) || s.questions.length === 0) { errors.push(`${where}: no tiene preguntas`); continue; }
    const qs: Question[] = [];
    s.questions.forEach((q: any, i: number) => {
      const w = `${where}, pregunta ${i + 1}`;
      let ok = true;
      if (typeof q?.q !== "string" || !q.q.trim()) { errors.push(`${w}: falta "q"`); ok = false; }
      if (!Array.isArray(q?.options) || q.options.length < 2 || q.options.length > 4 || q.options.some((o: any) => typeof o !== "string" || !o.trim())) {
        errors.push(`${w}: "options" debe tener de 2 a 4 textos`); ok = false;
      } else if (!Number.isInteger(q.answer) || q.answer < 0 || q.answer >= q.options.length) {
        errors.push(`${w}: "answer" debe ser el índice (desde 0) de una opción`); ok = false;
      }
      if (ok) qs.push({ q: q.q, options: q.options, answer: q.answer, why: String(q.why ?? ""), ref: String(q.ref ?? "") });
    });
    if (qs.length) steps.push({ step: s.step, title: String(s.title ?? ""), questions: qs });
  }
  steps.sort((a, b) => a.step - b.step);
  return { steps, errors };
}

async function loadQuestions(): Promise<{ steps: Step[]; errors: string[] }> {
  try {
    const text = await Bun.file(QUESTIONS_PATH).text();
    return validateQuestions(JSON.parse(text));
  } catch (e: any) {
    return { steps: [], errors: [`No se pudo leer ${QUESTIONS_PATH}: ${e?.message ?? e}`] };
  }
}

// ---------------------------------------------------------------- apodos
// Lista corta de palabras groseras, guardada en ROT13 para que el codigo no las muestre.
const rot13 = (s: string) => s.replace(/[a-z]/g, (c) => String.fromCharCode(((c.charCodeAt(0) - 97 + 13) % 26) + 97));
// Coinciden en cualquier parte del apodo (palabras largas, casi sin falsos positivos).
const BAD_PARTS = ["shpx", "fuvg", "ovgpu", "phag", "avtt", "sntt", "juber", "zvreq", "craqrw", "pnoeba", "znevpba", "puvatnq", "uvgyre", "tvyvcbyy", "uvwhrchgn"].map(rot13);
// Solo coinciden si son una palabra entera (cortas: evitan falsos positivos).
const BAD_WORDS = new Set(["chgn", "chgb", "chgnf", "chgbf", "iretn", "phyb", "cvwn", "qvpx", "pbpx", "fyhg", "encr", "cbea", "mbeen", "znevpn", "wbqre", "crar", "anmv", "grgnf"].map(rot13));

const fold = (s: string) => s.normalize("NFD").replace(/\p{M}/gu, "").toLowerCase();
const LEET: Record<string, string> = { "0": "o", "1": "i", "3": "e", "4": "a", "5": "s", "7": "t", "8": "b", "@": "a", $: "s", "!": "i", "+": "t" };
function simplify(s: string): string {
  return fold(s).replace(/[0134578@$!+]/g, (c) => LEET[c]).replace(/(.)\1{2,}/g, "$1");
}
function isRude(nick: string): boolean {
  const simple = simplify(nick);
  const squashed = simple.replace(/[^a-z]/g, "");
  if (BAD_PARTS.some((w) => squashed.includes(w))) return true;
  if (BAD_WORDS.has(squashed)) return true;
  return simple.split(/[^a-z]+/).some((t) => BAD_WORDS.has(t));
}

// Letras, numeros, espacios, emojis y unos pocos signos. Sin < > & " ' ni control.
const NICK_OK = /^[\p{L}\p{M}\p{N} _.\-!?¡¿*+#@~()\p{Extended_Pictographic}\p{Emoji_Modifier}\p{Regional_Indicator}\u200d\ufe0f]+$/u;
function cleanNick(raw: unknown): { nick?: string; error?: string } {
  if (typeof raw !== "string") return { error: "Escribe un apodo." };
  const nick = raw.normalize("NFC").replace(/\s+/g, " ").trim();
  const len = [...nick].length;
  if (len < 1) return { error: "Escribe un apodo." };
  if (len > 16) return { error: "El apodo puede tener hasta 16 letras." };
  if (!NICK_OK.test(nick)) return { error: "Usa solo letras, números, espacios y emojis." };
  if (isRude(nick)) return { error: "Ese apodo no vale. Elige otro." };
  return { nick };
}

// ---------------------------------------------------------------- limites por IP
const hits = new Map<string, number[]>();
function allow(kind: string, ip: string, limit: number, windowMs = 60000): boolean {
  const k = `${kind}|${ip}`;
  const now = Date.now();
  const arr = (hits.get(k) ?? []).filter((t) => now - t < windowMs);
  if (arr.length >= limit) { hits.set(k, arr); return false; }
  arr.push(now);
  hits.set(k, arr);
  return true;
}
const isLoopback = (a: string) => a === "127.0.0.1" || a === "::1" || a === "::ffff:127.0.0.1";
function clientIp(req: Request, srv: { requestIP(r: Request): { address: string } | null }): string {
  const peer = srv.requestIP(req)?.address ?? "unknown";
  if (isLoopback(peer)) {
    // Detras de un tunel local (cloudflared, tailscale funnel) todo llega desde 127.0.0.1.
    const fwd = req.headers.get("cf-connecting-ip") ?? req.headers.get("x-forwarded-for");
    if (fwd) return fwd.split(",")[0].trim().slice(0, 64);
  }
  return peer;
}

// ---------------------------------------------------------------- utilidades
const json = (data: unknown, status = 200, extra: Record<string, string> = {}) =>
  new Response(JSON.stringify(data), { status, headers: { "content-type": "application/json; charset=utf-8", "cache-control": "no-store", ...extra } });
const fail = (status: number, error: string) => json({ error }, status);

function sameKey(given: string | null | undefined): boolean {
  if (!given) return false;
  const a = Buffer.from(given);
  const b = Buffer.from(HOST_KEY);
  return a.length === b.length && timingSafeEqual(a, b);
}
function hostKeyOf(req: Request, url: URL): string | null {
  return req.headers.get("x-host-key") ?? url.searchParams.get("key");
}
function randomToken(): string {
  return Buffer.from(crypto.getRandomValues(new Uint8Array(18))).toString("base64url");
}
function newCode(): string {
  for (let i = 0; i < 200; i++) {
    const c = String(1000 + Math.floor(Math.random() * 9000));
    if (!rooms.has(c)) return c;
  }
  throw new Error("sin codigos libres");
}
async function readJson(req: Request): Promise<any | null> {
  const len = Number(req.headers.get("content-length") ?? 0);
  if (len > MAX_BODY) return null;
  try {
    const text = await req.text();
    if (text.length > MAX_BODY) return null;
    const v = JSON.parse(text);
    return v && typeof v === "object" ? v : null;
  } catch { return null; }
}
const touch = (room: Room) => { room.lastActivity = Date.now(); };
const SECURITY_HEADERS: Record<string, string> = {
  "content-security-policy": "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data:; connect-src 'self'; base-uri 'none'; form-action 'self'; frame-ancestors 'none'",
  "x-content-type-options": "nosniff",
  "referrer-policy": "no-referrer",
  "x-frame-options": "DENY",
};

// ---------------------------------------------------------------- puntajes y vistas
interface Entry { player: Player; rank: number; total: number }
function ranking(room: Room): Entry[] {
  const list = [...room.players.values()].sort((a, b) => b.score - a.score || a.joinedAt - b.joinedAt);
  const out: Entry[] = [];
  list.forEach((p, i) => {
    const rank = i > 0 && out[i - 1].total === p.score ? out[i - 1].rank : i + 1;
    out.push({ player: p, rank, total: p.score });
  });
  return out;
}
const currentStep = (room: Room) => room.steps[room.stepPos];
const currentQuestion = (room: Room) => currentStep(room).questions[room.qPos];

function lineOf(room: Room, e: Entry) {
  const p = e.player;
  const step = room.steps[room.stepPos];
  return {
    nick: p.nick,
    rank: e.rank,
    total: e.total,
    gain: room.phase === "reveal" && p.last?.qid === room.qid ? p.last.gain : 0,
    stepPts: step ? p.stepPts[step.step] ?? 0 : 0,
    streak: p.streak,
  };
}

function baseView(room: Room) {
  const v: Record<string, unknown> = {
    serverNow: Date.now(),
    code: room.code,
    phase: room.phase,
    nPlayers: room.players.size,
    nSteps: room.steps.length,
  };
  if (room.phase === "question" || room.phase === "reveal" || room.phase === "summary") {
    const step = currentStep(room);
    v.step = { step: step.step, title: step.title, pos: room.stepPos + 1 };
    v.qNum = room.qPos + 1;
    v.qTotal = step.questions.length;
  }
  if (room.phase === "question" || room.phase === "reveal") {
    const q = currentQuestion(room);
    v.qid = room.qid;
    v.startedAt = room.startedAt;
    v.durationMs = room.durationMs;
    v.q = q.q;
    v.options = q.options;
  }
  if (room.phase === "reveal") {
    const q = currentQuestion(room);
    v.answer = q.answer; // solo despues de cerrar la pregunta
    v.why = q.why;
    v.ref = q.ref;
    v.counts = room.counts;
  }
  return v;
}

function viewFor(room: Room, client: Client, rk: Entry[]) {
  const v = baseView(room);
  const lines = rk.map((e) => lineOf(room, e));
  if (client.kind === "host") {
    v.role = "host";
    v.players = [...room.players.values()].sort((a, b) => a.joinedAt - b.joinedAt).map((p) => ({ nick: p.nick, online: p.conns > 0 }));
    v.maxPlayers = MAX_PLAYERS;
    if (room.phase === "question" || room.phase === "reveal") v.answered = room.answers.size;
    v.board = room.phase === "lobby" ? [] : lines;
    v.top5 = lines.slice(0, 5);
    if (room.phase === "final") v.podium = lines.slice(0, 3);
    return v;
  }
  const p = client.player!;
  const me = rk.find((e) => e.player === p)!;
  v.role = "player";
  v.me = { nick: p.nick, total: p.score, rank: me.rank, n: rk.length, streak: p.streak };
  if (room.phase === "question") {
    const a = room.answers.get(p);
    v.answered = !!a;
    v.myChoice = a ? a.choice : null;
  }
  if (room.phase === "reveal") {
    v.result = p.last?.qid === room.qid ? p.last : { qid: room.qid, choice: null, correct: false, gain: 0, base: 0, bonus: 0, ms: null };
  }
  if (room.phase === "summary") {
    const step = currentStep(room);
    v.stepPoints = p.stepPts[step.step] ?? 0;
    v.top5 = lines.slice(0, 5);
  }
  if (room.phase === "final") {
    v.podium = lines.slice(0, 3);
    v.board = lines.map((l) => ({ nick: l.nick, rank: l.rank, total: l.total }));
  }
  return v;
}

function send(client: Client, view: unknown) {
  if (client.closed) return;
  try {
    client.ctl.enqueue(encoder.encode(`data: ${JSON.stringify(view)}\n\n`));
  } catch {
    client.closed = true;
  }
}
// Quita un cliente SSE una sola vez y descuenta su conexion.
function release(room: Room, client: Client) {
  if (client.released) return;
  client.released = true;
  client.closed = true;
  room.clients.delete(client);
  const p = client.player;
  if (p) { p.conns = Math.max(0, p.conns - 1); p.lastSeen = Date.now(); }
}
function broadcast(room: Room, only?: (c: Client) => boolean) {
  const rk = ranking(room);
  for (const c of room.clients) {
    if (c.closed) { release(room, c); continue; }
    if (only && !only(c)) continue;
    send(c, viewFor(room, c, rk));
  }
}

// ---------------------------------------------------------------- flujo del juego
function startQuestion(room: Room) {
  room.phase = "question";
  room.qid += 1;
  room.startedAt = Date.now();
  room.durationMs = DURATION_MS;
  room.answers = new Map();
  room.counts = currentQuestion(room).options.map(() => 0);
  if (room.timer) clearTimeout(room.timer);
  room.timer = setTimeout(() => endQuestion(room), room.durationMs + GRACE_MS);
  touch(room);
  broadcast(room);
}

function endQuestion(room: Room) {
  if (room.phase !== "question") return;
  if (room.timer) { clearTimeout(room.timer); room.timer = null; }
  const q = currentQuestion(room);
  const step = currentStep(room);
  for (const p of room.players.values()) {
    const a = room.answers.get(p);
    let res: LastResult;
    if (a) room.counts[a.choice] += 1;
    if (a && a.choice === q.answer) {
      const ms = Math.min(Math.max(a.at - room.startedAt, 0), room.durationMs);
      const base = Math.round(1000 - 500 * (ms / room.durationMs));
      p.streak += 1;
      p.correct += 1;
      const bonus = p.streak >= 3 ? Math.min(100 * (p.streak - 2), 300) : 0;
      res = { qid: room.qid, choice: a.choice, correct: true, gain: base + bonus, base, bonus, ms };
    } else {
      p.streak = 0;
      const ms = a ? Math.min(Math.max(a.at - room.startedAt, 0), room.durationMs) : null;
      res = { qid: room.qid, choice: a ? a.choice : null, correct: false, gain: 0, base: 0, bonus: 0, ms };
    }
    p.score += res.gain;
    p.stepPts[step.step] = (p.stepPts[step.step] ?? 0) + res.gain;
    p.last = res;
  }
  room.phase = "reveal";
  touch(room);
  broadcast(room);
}

function connectedPlayers(room: Room): Player[] {
  const now = Date.now();
  return [...room.players.values()].filter((p) => p.conns > 0 || now - p.lastSeen < 8000);
}

function hostAction(room: Room, action: string, body: any): { ok: boolean; error?: string } {
  touch(room);
  switch (action) {
    case "start":
      if (room.phase !== "lobby") return { ok: false, error: "La sala ya empezó." };
      if (room.players.size < 1) return { ok: false, error: "Todavía no hay jugadores." };
      room.stepPos = 0; room.qPos = 0;
      startQuestion(room);
      return { ok: true };
    case "skip":
      if (room.phase !== "question") return { ok: false, error: "No hay pregunta en curso." };
      endQuestion(room);
      return { ok: true };
    case "next":
      if (room.phase === "reveal") {
        if (room.qPos + 1 < currentStep(room).questions.length) { room.qPos += 1; startQuestion(room); }
        else { room.phase = "summary"; broadcast(room); }
        return { ok: true };
      }
      if (room.phase === "summary") {
        if (room.stepPos + 1 < room.steps.length) { room.stepPos += 1; room.qPos = 0; startQuestion(room); }
        else { room.phase = "final"; broadcast(room); }
        return { ok: true };
      }
      return { ok: false, error: "Ahora no se puede avanzar." };
    case "kick": {
      const nick = typeof body?.nick === "string" ? body.nick : "";
      const p = room.byFold.get(fold(nick));
      if (!p) return { ok: false, error: "No existe ese jugador." };
      room.players.delete(p.token);
      room.byFold.delete(p.fold);
      room.answers.delete(p);
      for (const c of [...room.clients]) if (c.player === p) { try { c.ctl.close(); } catch {} release(room, c); }
      broadcast(room);
      if (room.phase === "question" && room.players.size > 0 && connectedPlayers(room).every((q) => room.answers.has(q))) endQuestion(room);
      return { ok: true };
    }
    case "end":
      closeRoom(room);
      return { ok: true };
    default:
      return { ok: false, error: "Acción desconocida." };
  }
}

function closeRoom(room: Room) {
  if (room.timer) clearTimeout(room.timer);
  room.phase = "ended";
  broadcast(room);
  for (const c of room.clients) { c.closed = true; try { c.ctl.close(); } catch {} }
  room.clients.clear();
  rooms.delete(room.code);
}

// ---------------------------------------------------------------- CSV
function csvCell(v: string | number): string {
  let s = String(v);
  if (/^[=+\-@\t\r]/.test(s)) s = "'" + s; // evita formulas en Excel/Sheets
  return /[",\n\r]/.test(s) ? `"${s.replace(/"/g, '""')}"` : s;
}
function buildCsv(room: Room): string {
  const head = ["Puesto", "Apodo", "Total", "Correctas", ...room.steps.map((s) => `Paso ${s.step}`)];
  const rows = ranking(room).map((e) => [e.rank, e.player.nick, e.total, e.player.correct, ...room.steps.map((s) => e.player.stepPts[s.step] ?? 0)]);
  return "\uFEFF" + [head, ...rows].map((r) => r.map(csvCell).join(",")).join("\r\n") + "\r\n";
}

// ---------------------------------------------------------------- SSE
function openStream(room: Room, kind: "host" | "player", player?: Player): Response {
  let client: Client;
  const stream = new ReadableStream<Uint8Array>({
    start(ctl) {
      client = { kind, player, ctl, closed: false, released: false };
      room.clients.add(client);
      if (player) { player.conns += 1; player.lastSeen = Date.now(); }
      ctl.enqueue(encoder.encode("retry: 2000\n\n"));
      touch(room);
      send(client, viewFor(room, client, ranking(room)));
      if (player) broadcast(room, (c) => c.kind === "host");
    },
    cancel() {
      const was = client.released;
      release(room, client);
      if (player && !was) broadcast(room, (c) => c.kind === "host");
    },
  });
  return new Response(stream, {
    headers: {
      "content-type": "text/event-stream; charset=utf-8",
      "cache-control": "no-cache, no-transform",
      connection: "keep-alive",
      "x-accel-buffering": "no",
      ...SECURITY_HEADERS,
    },
  });
}

setInterval(() => {
  for (const room of rooms.values()) {
    for (const c of room.clients) {
      if (c.closed) { release(room, c); continue; }
      try { c.ctl.enqueue(encoder.encode(": ka\n\n")); } catch { release(room, c); }
    }
  }
}, KEEPALIVE_MS);

setInterval(() => {
  const now = Date.now();
  for (const room of [...rooms.values()]) if (now - room.lastActivity > EXPIRE_MS) closeRoom(room);
  for (const [k, arr] of hits) {
    const fresh = arr.filter((t) => now - t < 60000);
    if (fresh.length) hits.set(k, fresh); else hits.delete(k);
  }
}, SWEEP_MS);

// ---------------------------------------------------------------- HTTP
async function serveStatic(name: string): Promise<Response> {
  if (!/^[A-Za-z0-9._-]+$/.test(name) || name.startsWith(".")) return new Response("No encontrado", { status: 404 });
  const file = Bun.file(join(PUBLIC_DIR, name));
  if (!(await file.exists())) return new Response("No encontrado", { status: 404 });
  return new Response(file, { headers: { "cache-control": "no-cache", ...SECURITY_HEADERS } });
}

async function handle(req: Request, srv: any): Promise<Response> {
  const url = new URL(req.url);
  const path = url.pathname;
  const ip = clientIp(req, srv);
  const method = req.method;

  // ---- paginas
  if (method === "GET" || method === "HEAD") {
    if (path === "/") return serveStatic("index.html");
    if (path === "/host") {
      if (!authHost(req, url, ip)) {
        return new Response("Falta la clave del profesor (mira la terminal donde corre el servidor).", { status: 401, headers: { "content-type": "text/plain; charset=utf-8", ...SECURITY_HEADERS } });
      }
      return serveStatic("host.html");
    }
    if (path === "/events") return handleEvents(req, url, srv, ip);
    const roomMatch = path.match(/^\/api\/room\/(\d{4})$/);
    if (roomMatch) {
      if (!allow("lookup", ip, 120)) return fail(429, "Demasiadas consultas. Espera un momento.");
      const room = rooms.get(roomMatch[1]);
      if (!room) return fail(404, "No existe esa sala.");
      return json({ exists: true, phase: room.phase, players: room.players.size, full: room.players.size >= MAX_PLAYERS });
    }
    if (path === "/api/host/info") {
      if (!authHost(req, url, ip)) return fail(401, "Clave del profesor incorrecta.");
      const { steps, errors } = await loadQuestions();
      return json({
        steps: steps.map((s) => ({ step: s.step, title: s.title, count: s.questions.length })),
        errors,
        studentUrls: studentUrls(srv.port),
        rooms: [...rooms.values()].map((r) => ({ code: r.code, phase: r.phase, players: r.players.size, createdAt: r.createdAt })),
        durationMs: DURATION_MS,
      });
    }
    if (path === "/api/host/csv") {
      if (!authHost(req, url, ip)) return fail(401, "Clave del profesor incorrecta.");
      const room = rooms.get(url.searchParams.get("room") ?? "");
      if (!room) return fail(404, "No existe esa sala.");
      return new Response(buildCsv(room), {
        headers: {
          "content-type": "text/csv; charset=utf-8",
          "content-disposition": `attachment; filename="reto-${room.code}.csv"`,
          "cache-control": "no-store",
        },
      });
    }
    return serveStatic(path.slice(1));
  }

  if (method !== "POST") return fail(405, "Método no permitido.");

  // ---- acciones
  if (path === "/api/rooms") {
    if (!authHost(req, url, ip)) return fail(401, "Clave del profesor incorrecta.");
    const body = await readJson(req);
    if (!body) return fail(400, "Petición no válida.");
    if (rooms.size >= MAX_ROOMS) return fail(429, "Hay demasiadas salas abiertas.");
    const { steps, errors } = await loadQuestions();
    if (!steps.length) return fail(500, `El archivo de preguntas no sirve. ${errors.join(" ")}`);
    const wanted: number[] = Array.isArray(body.steps) ? body.steps.map(Number) : steps.map((s) => s.step);
    const chosen = steps.filter((s) => wanted.includes(s.step));
    if (!chosen.length) return fail(400, "Elige al menos un paso.");
    const room: Room = {
      code: newCode(), steps: chosen, players: new Map(), byFold: new Map(), phase: "lobby",
      stepPos: 0, qPos: 0, qid: 0, startedAt: 0, durationMs: DURATION_MS, answers: new Map(), counts: [],
      timer: null, clients: new Set(), lastActivity: Date.now(), createdAt: Date.now(),
    };
    rooms.set(room.code, room);
    return json({ code: room.code, steps: chosen.map((s) => s.step), warnings: errors });
  }

  if (path === "/api/host") {
    if (!authHost(req, url, ip)) return fail(401, "Clave del profesor incorrecta.");
    const body = await readJson(req);
    if (!body) return fail(400, "Petición no válida.");
    const room = rooms.get(String(body.room ?? ""));
    if (!room) return fail(404, "No existe esa sala.");
    const r = hostAction(room, String(body.action ?? ""), body);
    return r.ok ? json({ ok: true }) : fail(409, r.error ?? "No se pudo.");
  }

  if (path === "/api/join") {
    if (!allow("join", ip, 40)) return fail(429, "Demasiados intentos. Espera un minuto.");
    const body = await readJson(req);
    if (!body) return fail(400, "Petición no válida.");
    const room = rooms.get(String(body.code ?? "").trim());
    if (!room) return fail(404, "No existe esa sala. Revisa el código.");
    const c = cleanNick(body.nick);
    if (!c.nick) return fail(400, c.error!);
    const key = fold(c.nick);
    const existing = room.byFold.get(key);
    const token = typeof body.token === "string" ? body.token : "";
    if (existing) {
      if (token && token === existing.token) {
        existing.lastSeen = Date.now();
        touch(room);
        return json({ token: existing.token, nick: existing.nick, code: room.code, restored: true });
      }
      return fail(409, "Ese apodo ya está en uso en esta sala.");
    }
    if (room.players.size >= MAX_PLAYERS) return fail(403, "La sala está llena (60 jugadores).");
    const p: Player = {
      token: randomToken(), nick: c.nick, fold: key, joinedAt: Date.now(), score: 0, streak: 0, correct: 0,
      stepPts: {}, conns: 0, lastSeen: Date.now(),
    };
    room.players.set(p.token, p);
    room.byFold.set(key, p);
    touch(room);
    broadcast(room, (cl) => cl.kind === "host" || room.phase === "lobby");
    return json({ token: p.token, nick: p.nick, code: room.code, restored: false });
  }

  if (path === "/api/answer") {
    if (!allow("answer", ip, 120)) return fail(429, "Demasiadas respuestas seguidas.");
    const at = Date.now();
    const body = await readJson(req);
    if (!body) return fail(400, "Petición no válida.");
    const room = rooms.get(String(body.room ?? ""));
    if (!room) return fail(404, "No existe esa sala.");
    const p = room.players.get(String(body.token ?? ""));
    if (!p) return fail(401, "Jugador no reconocido.");
    if (room.phase !== "question" || body.qid !== room.qid) return fail(409, "Llegaste tarde: la pregunta ya terminó.");
    if (at > room.startedAt + room.durationMs + GRACE_MS) return fail(409, "Llegaste tarde: se acabó el tiempo.");
    const choice = body.choice;
    const q = currentQuestion(room);
    if (!Number.isInteger(choice) || choice < 0 || choice >= q.options.length) return fail(400, "Opción no válida.");
    p.lastSeen = at;
    if (room.answers.has(p)) return json({ ok: true, accepted: false });
    room.answers.set(p, { choice, at });
    touch(room);
    if (connectedPlayers(room).every((x) => room.answers.has(x))) endQuestion(room);
    else broadcast(room, (c) => c.kind === "host" || c.player === p);
    return json({ ok: true, accepted: true });
  }

  return fail(404, "No encontrado.");
}

function authHost(req: Request, url: URL, ip: string): boolean {
  if (!allow("hostkey", ip, 300)) return false;
  const ok = sameKey(hostKeyOf(req, url));
  if (!ok && !allow("badkey", ip, 10)) return false; // frena adivinar la clave
  return ok;
}

function handleEvents(req: Request, url: URL, srv: any, ip: string): Response {
  if (!allow("events", ip, 120)) return fail(429, "Demasiadas conexiones.");
  const room = rooms.get(url.searchParams.get("room") ?? "");
  if (!room) return fail(404, "No existe esa sala.");
  srv.timeout(req, 0);
  if (url.searchParams.has("key")) {
    if (!authHost(req, url, ip)) return fail(401, "Clave del profesor incorrecta.");
    return openStream(room, "host");
  }
  const p = room.players.get(url.searchParams.get("token") ?? "");
  if (!p) return fail(401, "Jugador no reconocido.");
  return openStream(room, "player", p);
}

// ---------------------------------------------------------------- arranque
function studentUrls(port: number): string[] {
  const out: string[] = [];
  for (const addrs of Object.values(networkInterfaces())) {
    for (const a of addrs ?? []) if (a.family === "IPv4" && !a.internal) out.push(`http://${a.address}:${port}/`);
  }
  return out;
}

const server = Bun.serve({
  port: PORT,
  hostname: "0.0.0.0",
  idleTimeout: 120,
  fetch: (req, srv) => handle(req, srv).catch((e) => {
    console.error("Error:", e);
    return fail(500, "Error interno.");
  }),
});

const loaded = await loadQuestions();
const nQ = loaded.steps.reduce((n, s) => n + s.questions.length, 0);
console.log("");
console.log("  RETO EN VIVO");
console.log("  ============");
console.log("");
console.log("  Profesor (guarda esta dirección, tiene la clave secreta):");
console.log(`    http://localhost:${server.port}/host?key=${HOST_KEY}`);
console.log("");
console.log("  Alumnos (misma red Wi-Fi):");
const urls = studentUrls(server.port);
if (urls.length) for (const u of urls) console.log(`    ${u}`);
else console.log("    (no encontré ninguna red; conéctate a una o usa un túnel)");
console.log("");
if (loaded.steps.length) console.log(`  Preguntas: ${nQ} en ${loaded.steps.length} pasos (${QUESTIONS_PATH})`);
for (const e of loaded.errors) console.log(`  AVISO preguntas: ${e}`);
if (!loaded.steps.length) console.log("  No hay preguntas válidas: no podrás crear salas hasta arreglar el archivo.");
console.log(`  Tiempo por pregunta: ${DURATION_MS / 1000} s. Máximo ${MAX_PLAYERS} jugadores por sala.`);
console.log(`
  Ayuda
  - Si los alumnos no pueden abrir la dirección, puede que el Wi-Fi tenga "aislamiento
    de clientes" (pasa en muchos colegios): los aparatos no se ven entre sí. Solución
    fácil: crea un punto de acceso (hotspot) en tu teléfono y conecta ahí tu Mac y a
    los alumnos.
  - Para abrirlo desde fuera de la red, sin crear cuenta (túnel rápido):
        cloudflared tunnel --url http://localhost:${server.port}
    Te da una dirección https://algo.trycloudflare.com para los alumnos.
    Otra opción: Tailscale Funnel (tailscale funnel ${server.port}).
    No compartas con los alumnos la dirección del profesor (la que lleva ?key=).
  - Todo vive en memoria. Si paras el servidor (Ctrl+C), las salas se pierden.
`);

function shutdown() { server.stop(true); process.exit(0); }
process.on("SIGINT", shutdown);
process.on("SIGTERM", shutdown);
