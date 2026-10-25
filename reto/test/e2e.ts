// Prueba de extremo a extremo del servidor del reto.
// Uso: bun reto/test/e2e.ts
// Arranca el servidor en un puerto libre con las preguntas de ejemplo, simula
// jugadores con fetch + SSE y comprueba reglas, puntos y seguridad.
import { join } from "node:path";
import { readdirSync, readFileSync } from "node:fs";

const SAMPLE = join(import.meta.dir, "questions.sample.json");
const PUBLIC = join(import.meta.dir, "..", "public");
const DURATION = 3000;
const GRACE = 400;
const rot13 = (s: string) => s.replace(/[a-z]/g, (c) => String.fromCharCode(((c.charCodeAt(0) - 97 + 13) % 26) + 97));
const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));

let passed = 0;
const failures: string[] = [];
function check(name: string, ok: boolean, detail?: unknown) {
  if (ok) { passed++; console.log(`  ok   ${name}`); }
  else { failures.push(name); console.log(`  FAIL ${name}${detail !== undefined ? "  -> " + JSON.stringify(detail) : ""}`); }
}
const section = (t: string) => console.log(`\n${t}`);

// ------------------------------------------------------------ servidor
interface Srv { port: number; key: string; out: string; base: string; stop(): Promise<void> }
async function startServer(env: Record<string, string> = {}): Promise<Srv> {
  const proc = Bun.spawn(["bun", join(import.meta.dir, "..", "server.ts")], {
    env: { ...process.env, PORT: "0", RETO_QUESTIONS: SAMPLE, RETO_DURATION_MS: String(DURATION), ...env },
    stdout: "pipe", stderr: "pipe",
  });
  let out = "";
  const reader = proc.stdout.getReader();
  const dec = new TextDecoder();
  const t0 = Date.now();
  while (Date.now() - t0 < 8000 && !/Ayuda[\s\S]*Ctrl\+C/.test(out)) {
    const { value, done } = await reader.read();
    if (done) break;
    out += dec.decode(value);
  }
  reader.releaseLock();
  const m = out.match(/http:\/\/localhost:(\d+)\/host\?key=([a-z0-9]+)/);
  if (!m) { proc.kill(); throw new Error("no pude leer la dirección del profesor:\n" + out); }
  return {
    port: Number(m[1]), key: m[2], out, base: `http://127.0.0.1:${m[1]}`,
    async stop() { proc.kill(); await proc.exited; },
  };
}

// ------------------------------------------------------------ cliente SSE
class Sse {
  last: any = null;
  events: any[] = [];
  comments = 0;
  private waiters: Array<() => void> = [];
  private ctl = new AbortController();
  status = 0;
  ended = false;
  constructor(public url: string) { this.run(); }
  private async run() {
    try {
      const r = await fetch(this.url, { signal: this.ctl.signal });
      this.status = r.status;
      if (!r.body) return;
      const reader = r.body.getReader();
      const dec = new TextDecoder();
      let buf = "";
      for (;;) {
        const { value, done } = await reader.read();
        if (done) break;
        buf += dec.decode(value, { stream: true });
        let i: number;
        while ((i = buf.indexOf("\n\n")) >= 0) {
          const block = buf.slice(0, i);
          buf = buf.slice(i + 2);
          for (const line of block.split("\n")) {
            if (line.startsWith("data: ")) { this.last = JSON.parse(line.slice(6)); this.events.push(this.last); }
            else if (line.startsWith(":")) this.comments++;
          }
          this.waiters.splice(0).forEach((w) => w());
        }
      }
    } catch { /* abortado */ }
    this.ended = true;
    this.waiters.splice(0).forEach((w) => w());
  }
  async waitFor(pred: (s: any) => boolean, ms = 8000): Promise<any> {
    const t0 = Date.now();
    for (;;) {
      if (this.last && pred(this.last)) return this.last;
      const left = ms - (Date.now() - t0);
      if (left <= 0 || this.ended) throw new Error("tiempo agotado esperando un estado; último: " + JSON.stringify(this.last)?.slice(0, 300));
      await new Promise<void>((res) => { this.waiters.push(res); setTimeout(res, Math.min(left, 200)); });
    }
  }
  close() { this.ctl.abort(); }
}

// ------------------------------------------------------------ llamadas
let S: Srv;
const post = (path: string, body: unknown, headers: Record<string, string> = {}) =>
  fetch(S.base + path, { method: "POST", headers: { "content-type": "application/json", ...headers }, body: JSON.stringify(body) });
const hostPost = (body: any) => post("/api/host", body, { "x-host-key": S.key });
let ipCounter = 0;
const freshIp = () => ({ "x-forwarded-for": `10.9.${Math.floor(++ipCounter / 250)}.${ipCounter % 250}` });
const joinReq = (code: string, nick: string, token?: string, ip = true) => post("/api/join", { code, nick, token }, ip ? freshIp() : {});
async function newRoom(steps?: number[]): Promise<string> {
  const r = await post("/api/rooms", steps ? { steps } : {}, { "x-host-key": S.key });
  return (await r.json()).code;
}

const ANS = [2, 1, 0, 2, 3, 0]; // respuestas correctas del archivo de ejemplo
const wrong = (i: number) => (ANS[i] + 1) % 4;

// ------------------------------------------------------------ pruebas
async function main() {
  S = await startServer();
  console.log(`Servidor de prueba en el puerto ${S.port}`);

  section("Arranque y clave del profesor");
  check("imprime la dirección del profesor con clave de 6 caracteres", /localhost:\d+\/host\?key=[a-z0-9]{6}\b/.test(S.out), S.key);
  check("imprime direcciones de alumnos (IPv4 no interna) o avisa", /http:\/\/\d+\.\d+\.\d+\.\d+:\d+\//.test(S.out) || /no encontré ninguna red/.test(S.out));
  check("imprime la ayuda (aislamiento de clientes, cloudflared, Tailscale)", /aislamiento/.test(S.out) && /cloudflared tunnel --url/.test(S.out) && /Tailscale/.test(S.out));
  const unauth = async (name: string, p: Promise<Response>) => check(name + " -> 401", (await p).status === 401);
  await unauth("GET /host sin clave", fetch(S.base + "/host"));
  await unauth("GET /host con clave mala", fetch(S.base + "/host?key=zzzzzz"));
  await unauth("POST /api/rooms sin clave", post("/api/rooms", {}));
  await unauth("POST /api/host sin clave", post("/api/host", { room: "1234", action: "start" }));
  await unauth("GET /api/host/info sin clave", fetch(S.base + "/api/host/info"));
  await unauth("GET csv sin clave", fetch(S.base + "/api/host/csv?room=1234"));
  check("GET /host con la clave -> 200", (await fetch(S.base + `/host?key=${S.key}`)).status === 200);
  check("el servidor ve las preguntas de ejemplo", (await (await fetch(S.base + "/api/host/info", { headers: { "x-host-key": S.key } })).json()).steps.length === 2);

  section("Sala y búsqueda por código");
  check("crear sala con pasos inválidos -> 400", (await post("/api/rooms", { steps: [99] }, { "x-host-key": S.key })).status === 400);
  const code = await newRoom([1, 2]);
  check("código de 4 dígitos", /^\d{4}$/.test(code), code);
  const look = await fetch(`${S.base}/api/room/${code}`);
  check("GET /api/room/CODE existe", look.status === 200 && (await look.json()).phase === "lobby");
  check("código que no existe -> 404", (await fetch(`${S.base}/api/room/0000`)).status === 404);
  check("unirse a un código que no existe -> 404", (await joinReq("0000", "Zed")).status === 404);

  section("Apodos");
  const roomB = await newRoom([1]);
  const j = async (nick: string, room = roomB) => joinReq(room, nick);
  let r = await j("  Dani  ");
  const dj = await r.json();
  check("el apodo se recorta (trim)", r.status === 200 && dj.nick === "Dani", dj);
  check("apodo repetido (otra mayúscula) -> 409", (await j("dani")).status === 409);
  check("apodo repetido (con tilde) -> 409", (await j("Dáni")).status === 409);
  check("mismo apodo en otra sala vale", (await j("Dani", await newRoom([1]))).status === 200);
  check("etiqueta HTML rechazada -> 400", (await j("<script>alert(1)</script>")).status === 400);
  check("símbolo < o & rechazado -> 400", (await j("a<b")).status === 400 && (await j("Ana & Co")).status === 400);
  check("comillas rechazadas -> 400", (await j(`x" onerror="y`)).status === 400);
  check("apodo vacío -> 400", (await j("   ")).status === 400);
  check("17 caracteres -> 400", (await j("x".repeat(17))).status === 400);
  check("16 caracteres vale", (await j("y".repeat(16))).status === 200);
  check("palabra grosera -> 400", (await j(rot13("zvreqn"))).status === 400);
  check("grosera con números (leet) -> 400", (await j(rot13("zvreqn").replace("i", "1").replace("a", "4"))).status === 400);
  check("grosera con espacios -> 400", (await j(rot13("zvreqn").split("").join(" "))).status === 400);
  check("nombres normales con letras parecidas valen (Penélope, Computadora)", (await j("Penélope")).status === 200 && (await j("Computadora")).status === 200);
  check("emoji vale", (await j("🦊 Zorro")).status === 200);
  const csvRoom = await newRoom([1]);
  await joinReq(csvRoom, "@fórmula");
  const csvOdd = await (await fetch(`${S.base}/api/host/csv?room=${csvRoom}`, { headers: { "x-host-key": S.key } })).text();
  check("CSV neutraliza apodos que empiezan con @ (fórmulas)", csvOdd.includes("'@fórmula"), csvOdd);

  section("Capacidad y límite de intentos");
  const roomC = await newRoom([1]);
  let okCount = 0;
  for (let i = 1; i <= 60; i++) if ((await joinReq(roomC, "P" + i)).status === 200) okCount++;
  check("entran 60 jugadores", okCount === 60, okCount);
  check("el 61 se rechaza -> 403", (await joinReq(roomC, "P61")).status === 403);
  const flood = { "x-forwarded-for": "7.7.7.7" };
  const codes: number[] = [];
  for (let i = 0; i < 50; i++) codes.push((await post("/api/join", { code: "0000", nick: "Spam" + i }, flood)).status);
  check("límite por IP en intentos de unirse (llega un 429)", codes.includes(429) && codes[0] === 404, codes.slice(0, 3).concat(codes.slice(-2)));
  check("otra IP no se ve afectada", (await joinReq(roomC, "Otro")).status === 403 /* sala llena, no 429 */);

  section("Partida completa con 3 jugadores");
  const names = ["Ana", "Beto", "Cami"];
  const tokens: Record<string, string> = {};
  for (const n of names) {
    const res = await joinReq(code, n);
    const body = await res.json();
    tokens[n] = body.token;
  }
  check("tres jugadores unidos con token", names.every((n) => tokens[n]?.length >= 16));
  check("apodo repetido sin token -> 409", (await joinReq(code, "ana")).status === 409);
  check("apodo repetido con token ajeno -> 409", (await joinReq(code, "Ana", tokens.Beto)).status === 409);

  const sse = (n: string) => new Sse(`${S.base}/events?room=${code}&token=${tokens[n]}`);
  const host = new Sse(`${S.base}/events?room=${code}&key=${S.key}`);
  const pl: Record<string, Sse> = { Ana: sse("Ana"), Beto: sse("Beto"), Cami: sse("Cami") };
  const lobby = await host.waitFor((s) => s.players?.length === 3);
  check("el profesor ve a los 3 jugadores en vivo", lobby.players.length === 3 && lobby.phase === "lobby");
  check("SSE rechaza token malo y clave mala", (await fetch(`${S.base}/events?room=${code}&token=nope`)).status === 401 && (await fetch(`${S.base}/events?room=${code}&key=nope00`)).status === 401);
  check("SSE rechaza sala inexistente", (await fetch(`${S.base}/events?room=0000&token=x`)).status === 404);
  check("el alumno no recibe la lista del profesor", !("players" in pl.Ana.last) && pl.Ana.last.role === "player");

  check("empezar sin clave -> 401", (await post("/api/host", { room: code, action: "start" })).status === 401);
  check("no se puede avanzar durante el lobby -> 409", (await hostPost({ room: code, action: "next" })).status === 409);
  check("empezar con clave -> 200", (await hostPost({ room: code, action: "start" })).status === 200);
  check("empezar dos veces -> 409", (await hostPost({ room: code, action: "start" })).status === 409);

  type Rec = { gain: number; base: number; bonus: number; ms: number | null; correct: boolean; total: number };
  const rec: Record<string, Rec[]> = { Ana: [], Beto: [], Cami: [] };
  const answer = (n: string, qid: number, choice: number) =>
    post("/api/answer", { room: code, token: tokens[n], qid, choice });
  const expectStreak: Record<string, number> = { Ana: 0, Beto: 0, Cami: 0 };

  async function waitQuestion(qid: number) {
    const evs = await Promise.all([host, ...Object.values(pl)].map((c) => c.waitFor((s) => s.phase === "question" && s.qid === qid)));
    return evs[0];
  }
  async function waitReveal(qid: number) {
    await Promise.all([host, ...Object.values(pl)].map((c) => c.waitFor((s) => s.phase === "reveal" && s.qid === qid)));
    for (const n of names) {
      const e = pl[n].last;
      rec[n].push({ ...e.result, total: e.me.total });
    }
    return host.last;
  }
  function checkScoring(qIdx: number) {
    for (const n of names) {
      const x = rec[n][qIdx];
      if (x.correct) {
        expectStreak[n] += 1;
        const expBonus = expectStreak[n] >= 3 ? Math.min(100 * (expectStreak[n] - 2), 300) : 0;
        const expBase = Math.round(1000 - 500 * ((x.ms as number) / DURATION));
        check(`P${qIdx + 1} ${n}: base = 1000 - 500*t/duración (${x.ms} ms -> ${expBase}) y bonus de racha ${expBonus}`, x.base === expBase && x.bonus === expBonus && x.gain === expBase + expBonus, x);
      } else {
        expectStreak[n] = 0;
        check(`P${qIdx + 1} ${n}: sin acierto -> 0 puntos y racha a cero`, x.gain === 0 && x.bonus === 0, x);
      }
    }
  }

  // --- P1: Ana acierta rápido (y manda una segunda respuesta), Beto acierta tarde, Cami falla
  let q = await waitQuestion(1);
  const qEvents = [host, ...Object.values(pl)].flatMap((c) => c.events.filter((e) => e.phase === "question"));
  check("la pregunta trae texto y 4 opciones", typeof q.q === "string" && q.options.length === 4);
  check("el evento de pregunta NO trae la respuesta ni la explicación", qEvents.every((e) => !/"answer"|"why"|"ref"|"counts"/.test(JSON.stringify(e))));
  check("el evento trae startedAt, durationMs y serverNow", q.startedAt > 0 && q.durationMs === DURATION && Math.abs(q.serverNow - Date.now()) < 2000);
  const t1 = Date.now();
  check("Ana responde bien", (await answer("Ana", 1, ANS[0])).status === 200);
  const hs = await host.waitFor((s) => s.qid === 1 && s.answered === 1);
  check("el contador del profesor sube a 1 de 3 y la pregunta sigue abierta", hs.phase === "question" && hs.nPlayers === 3);
  const dbl = await answer("Ana", 1, wrong(0));
  check("segunda respuesta de Ana ignorada (accepted:false)", dbl.status === 200 && (await dbl.json()).accepted === false);
  await sleep(900);
  await answer("Beto", 1, ANS[0]);
  await answer("Cami", 1, wrong(0));
  let rv = await waitReveal(1);
  check("termina antes de tiempo cuando todos respondieron", Date.now() - t1 < DURATION - 500, Date.now() - t1);
  check("el reveal trae la respuesta, la explicación y la gráfica", rv.answer === ANS[0] && rv.why.length > 3 && rv.counts[ANS[0]] === 2 && rv.counts[wrong(0)] === 1, rv.counts);
  check("la segunda respuesta de Ana no cambió la gráfica ni sus puntos", rec.Ana[0].correct && rec.Ana[0].choice === ANS[0]);
  checkScoring(0);
  check("Beto contestó más tarde que Ana y ganó menos", rec.Beto[0].ms! > 700 && rec.Beto[0].base < rec.Ana[0].base, [rec.Ana[0].ms, rec.Beto[0].ms]);
  check("puestos tras P1: Ana 1, Beto 2, Cami 3", pl.Ana.last.me.rank === 1 && pl.Beto.last.me.rank === 2 && pl.Cami.last.me.rank === 3);
  check("el profesor ve a cada jugador con puntos ganados, total y puesto", rv.board.length === 3 && rv.board.every((l: any) => "gain" in l && "total" in l && "rank" in l));
  check("avanzar desde el reveal sin clave -> 401", (await post("/api/host", { room: code, action: "next" })).status === 401);
  await hostPost({ room: code, action: "next" });

  // --- P2: todos aciertan; Beto se desconecta y vuelve
  q = await waitQuestion(2);
  await Promise.all(names.map((n) => answer(n, 2, ANS[1])));
  await waitReveal(2);
  checkScoring(1);
  await hostPost({ room: code, action: "next" });

  // --- P3: Ana y Cami aciertan (Ana: racha 3), Beto falla. Antes: reconexión de Beto.
  const totalBeforeLeave = pl.Beto.last.me.total;
  pl.Beto.close();
  await host.waitFor((s) => s.players.find((p: any) => p.nick === "Beto")?.online === false);
  check("el profesor ve a Beto desconectado", true);
  check("volver a entrar con otro apodo y su token crea otro jugador (no roba el puesto)", (await joinReq(code, "Beto2", tokens.Beto)).status === 200);
  const reJoin = await joinReq(code, "beto", tokens.Beto);
  const reBody = await reJoin.json();
  check("reconectar con apodo + token restaura al mismo jugador", reJoin.status === 200 && reBody.restored === true && reBody.token === tokens.Beto, reBody);
  check("sin el token no se puede reclamar el apodo -> 409", (await joinReq(code, "Beto")).status === 409);
  pl.Beto = sse("Beto");
  const back = await pl.Beto.waitFor((s) => s.me?.nick === "Beto");
  check("tras reconectar conserva sus puntos", back.me.total === totalBeforeLeave && totalBeforeLeave > 0, [back.me.total, totalBeforeLeave]);
  await hostPost({ room: code, action: "kick", nick: "Beto2" });
  await host.waitFor((s) => s.players.length === 3);
  check("el profesor puede expulsar a un jugador", host.last.players.length === 3);
  q = await waitQuestion(3);
  await answer("Ana", 3, ANS[2]);
  await answer("Beto", 3, wrong(2));
  await answer("Cami", 3, ANS[2]);
  await waitReveal(3);
  checkScoring(2);
  check("Ana tiene +100 de racha en la tercera seguida", rec.Ana[2].bonus === 100);
  await hostPost({ room: code, action: "next" });

  // --- P4: solo Ana responde; el profesor salta; Beto llega tarde
  q = await waitQuestion(4);
  await answer("Ana", 4, ANS[3]);
  check("'Saltar' sin clave -> 401", (await post("/api/host", { room: code, action: "skip" })).status === 401);
  check("'Saltar' con clave cierra la pregunta", (await hostPost({ room: code, action: "skip" })).status === 200);
  await waitReveal(4);
  checkScoring(3);
  const tooLate = await answer("Beto", 4, ANS[3]);
  check("respuesta después del cierre -> 409", tooLate.status === 409);
  check("sin respuesta = 0 puntos (Beto y Cami)", rec.Beto[3].choice === null && rec.Cami[3].choice === null);
  check("Ana +200 en la cuarta seguida", rec.Ana[3].bonus === 200);
  await hostPost({ room: code, action: "next" });

  // --- P5: Ana y Beto aciertan; Cami contesta dentro del margen de gracia (400 ms)
  q = await waitQuestion(5);
  await answer("Ana", 5, ANS[4]);
  await answer("Beto", 5, ANS[4]);
  const wait = q.startedAt + DURATION + 150 - Date.now();
  await sleep(Math.max(0, wait));
  const grace = await answer("Cami", 5, ANS[4]);
  check("respuesta dentro de los 400 ms de gracia se acepta", grace.status === 200 && (await grace.json()).accepted === true);
  await waitReveal(5);
  checkScoring(4);
  check("con respuesta en la gracia el tiempo se limita a la duración (base 500)", rec.Cami[4].base === 500, rec.Cami[4]);
  check("Ana +300 en la quinta seguida (tope)", rec.Ana[4].bonus === 300);
  check("respuesta con una pregunta anterior (qid viejo) -> 409", (await answer("Ana", 4, ANS[3])).status === 409);
  check("opción fuera de rango o texto -> 400 o 409", [(await post("/api/answer", { room: code, token: tokens.Ana, qid: 5, choice: 9 })).status].every((s) => s === 400 || s === 409));

  // --- Resumen del paso 1
  await hostPost({ room: code, action: "next" });
  const sumHost = await host.waitFor((s) => s.phase === "summary");
  const sumAna = await pl.Ana.waitFor((s) => s.phase === "summary");
  const sumCami = await pl.Cami.waitFor((s) => s.phase === "summary");
  const stepSum = (n: string) => rec[n].slice(0, 5).reduce((a, x) => a + x.gain, 0);
  check("resumen del paso: puntos de las 5 preguntas de cada alumno", sumAna.stepPoints === stepSum("Ana") && sumCami.stepPoints === stepSum("Cami"), [sumAna.stepPoints, stepSum("Ana")]);
  check("el acumulado del resumen coincide con la suma de ganancias", sumAna.me.total === stepSum("Ana") && sumAna.qTotal === 5);
  check("resumen con top 5 (aquí 3 jugadores) y puesto propio", sumAna.top5.length === 3 && sumAna.me.rank === 1 && sumHost.top5[0].nick === "Ana");
  const order = [...names].sort((a, b) => stepSum(b) - stepSum(a));
  check("el orden del ranking sigue los puntos", sumHost.top5.map((l: any) => l.nick).join() === order.join(), [sumHost.top5.map((l: any) => l.nick), order]);

  // --- Paso 2 y final
  await hostPost({ room: code, action: "next" });
  q = await waitQuestion(6);
  check("el paso 2 empieza con su título", q.step.step === 2 && q.qNum === 1 && q.qTotal === 1);
  await Promise.all(names.map((n) => answer(n, 6, ANS[5])));
  await waitReveal(6);
  checkScoring(5);
  check("la racha de Ana sigue en el paso 2 con tope de +300", rec.Ana[5].bonus === 300);
  await hostPost({ room: code, action: "next" }); // resumen paso 2
  await host.waitFor((s) => s.phase === "summary" && s.step.step === 2);
  await hostPost({ room: code, action: "next" }); // final
  const fin = await host.waitFor((s) => s.phase === "final");
  const finAna = await pl.Ana.waitFor((s) => s.phase === "final");
  const totals: Record<string, number> = {};
  for (const n of names) totals[n] = rec[n].reduce((a, x) => a + x.gain, 0);
  const finalOrder = [...names].sort((a, b) => totals[b] - totals[a]);
  check("podio: los 3 primeros en orden de puntos", fin.podium.map((l: any) => l.nick).join() === finalOrder.join() && fin.podium[0].total === totals[finalOrder[0]], fin.podium);
  check("clasificación completa en el alumno, con su puesto", finAna.board.length === 3 && finAna.me.rank === 1 && finAna.me.total === totals.Ana);

  section("CSV");
  const csvRes = await fetch(`${S.base}/api/host/csv?room=${code}&key=${S.key}`);
  const csv = (await csvRes.text()).replace(/^﻿/, "").trim().split("\r\n");
  check("CSV: tipo y descarga", csvRes.headers.get("content-type")!.startsWith("text/csv") && /attachment/.test(csvRes.headers.get("content-disposition") ?? ""));
  check("CSV: cabecera", csv[0] === "Puesto,Apodo,Total,Correctas,Paso 1,Paso 2", csv[0]);
  check("CSV: una fila por jugador en orden de puesto con sus totales", csv.length === 4 && finalOrder.every((n, i) => csv[i + 1].split(",")[1] === n && Number(csv[i + 1].split(",")[2]) === totals[n]), csv);
  check("CSV: puntos por paso y correctas de Ana", csv[1] === `1,Ana,${totals.Ana},6,${stepSum("Ana")},${rec.Ana[5].gain}`, csv[1]);

  section("Cerrar sala");
  await hostPost({ room: code, action: "end" });
  const ended = await pl.Ana.waitFor((s) => s.phase === "ended");
  check("al terminar la sala los alumnos reciben 'ended'", ended.phase === "ended");
  check("la sala ya no existe", (await fetch(`${S.base}/api/room/${code}`)).status === 404);

  section("Estáticos y seguridad web");
  const idx = await fetch(S.base + "/");
  check("GET / sirve la página de alumnos con CSP y nosniff", idx.status === 200 && /default-src 'self'/.test(idx.headers.get("content-security-policy") ?? "") && idx.headers.get("x-content-type-options") === "nosniff");
  check("no se sirve nada fuera de public/", (await fetch(S.base + "/..%2fserver.ts")).status === 404 && (await fetch(S.base + "/server.ts")).status === 404 && (await fetch(S.base + "/.env")).status === 404);
  const jsFiles = readdirSync(PUBLIC).filter((f) => /\.(js|html)$/.test(f));
  const inner = jsFiles.filter((f) => /innerHTML|outerHTML|insertAdjacentHTML|document\.write|eval\(/.test(readFileSync(join(PUBLIC, f), "utf8")));
  check("el código del navegador nunca usa innerHTML ni eval", inner.length === 0, inner);
  const inlineScripts = jsFiles.filter((f) => f.endsWith(".html") && /<script(?![^>]*src=)/.test(readFileSync(join(PUBLIC, f), "utf8")));
  check("sin scripts en línea (compatible con la CSP)", inlineScripts.length === 0);
  const dashes = [...jsFiles, "style.css"].filter((f) => /[\u2013\u2014]/.test(readFileSync(join(PUBLIC, f), "utf8")));
  check("sin rayas largas (em/en dash) en la interfaz", dashes.length === 0, dashes);

  [host, ...Object.values(pl)].forEach((c) => c.close());
  await S.stop();

  section("Caducidad de salas (servidor aparte con RETO_EXPIRE_MS=800)");
  S = await startServer({ RETO_EXPIRE_MS: "800", RETO_SWEEP_MS: "200" });
  const rExp = await newRoom();
  check("la sala existe al crearla", (await fetch(`${S.base}/api/room/${rExp}`)).status === 200);
  await sleep(1500);
  check("la sala caduca sin actividad", (await fetch(`${S.base}/api/room/${rExp}`)).status === 404);
  await S.stop();

  section("Archivo de preguntas inválido");
  const bad = join(import.meta.dir, ".bad-questions.tmp.json");
  await Bun.write(bad, JSON.stringify({ version: 1, steps: [{ step: 1, title: "x", questions: [{ q: "a", options: ["1", "2"], answer: 5, why: "", ref: "" }] }] }));
  S = await startServer({ RETO_QUESTIONS: bad });
  const badRes = await post("/api/rooms", {}, { "x-host-key": S.key });
  check("sin preguntas válidas no se crea la sala (500 con mensaje)", badRes.status === 500 && /answer/.test((await badRes.json()).error));
  await S.stop();
  try { (await import("node:fs")).unlinkSync(bad); } catch { /* ya borrado */ }
}

try {
  await main();
} catch (e: any) {
  failures.push("excepción: " + (e?.message ?? e));
  console.log("\nEXCEPCIÓN:", e?.stack ?? e);
}
console.log(`\n${passed} comprobaciones correctas, ${failures.length} fallos`);
if (failures.length) { console.log(failures.map((f) => " - " + f).join("\n")); process.exit(1); }
process.exit(0);
