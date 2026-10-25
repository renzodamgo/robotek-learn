# Reto en vivo

Un quiz en vivo, al estilo Kahoot, para repasar el curso en clase. El profesor
abre una sala en su computadora y los alumnos entran desde el teléfono o la
computadora con un código de 4 números y un apodo. Cada pregunta dura 30
segundos; contestar rápido da más puntos y acertar varias seguidas da bono.

No necesita instalar nada más que [Bun](https://bun.sh). No usa dependencias
de npm: todo es `Bun.serve`, SSE y HTML, CSS y JavaScript simples.

## Cómo se usa

```sh
bun reto/server.ts            # puerto 3000
PORT=4000 bun reto/server.ts  # otro puerto
```

Al arrancar, la terminal muestra:

- La dirección del **profesor**: `http://localhost:3000/host?key=XXXXXX`.
  La `key` es una clave secreta de 6 caracteres que cambia cada vez que se
  arranca el servidor. Sin ella nadie puede crear salas ni controlar el juego.
  No la compartas con los alumnos.
- Las direcciones de los **alumnos**: `http://192.168.x.x:3000/` (una por cada
  red del Mac).

Pasos en clase:

1. Abre la dirección del profesor en el navegador (conviene que sea la que se
   proyecta en la pantalla grande).
2. Elige los pasos del curso que entran (por defecto todos) y pulsa **Crear sala**.
   Sale el código en letras muy grandes.
3. Los alumnos abren la dirección de alumnos, escriben el código y su apodo.
   Los ves entrar en vivo.
4. Pulsa **Empezar**. Para cada pregunta: se ve el tiempo, cuántos han
   respondido y los puestos. Termina cuando todos respondieron, cuando se acaba
   el tiempo o cuando pulsas **Saltar**.
5. Al terminar cada pregunta sale la respuesta correcta, una frase que explica
   por qué, una gráfica de lo que eligió la clase y los puntos de cada uno.
   Pulsa **Siguiente pregunta**.
6. Tras la última pregunta de un paso sale el resumen del paso y luego
   **Siguiente paso**. Al final hay podio, clasificación completa y el botón
   **Descargar resultados (CSV)**.
7. **Terminar sala** cierra la sala en cualquier momento.

Si recargas la página del profesor, vuelve a la sala abierta. Si un alumno
recarga o pierde la conexión, entra solo otra vez y conserva sus puntos (el
navegador guarda un token). Si cierra la pestaña, puede volver con el mismo
código y el mismo apodo desde el mismo aparato.

## Puntos

- Respuesta mala o sin respuesta: 0.
- Respuesta buena: `1000 - 500 * (tiempo usado / tiempo total)`, redondeado.
  Es decir, de 1000 (al instante) a 500 (en el último segundo).
- Racha: +100 por cada acierto seguido desde el tercero, con tope de +300
  (tercero +100, cuarto +200, quinto en adelante +300). La racha sigue entre
  pasos y se rompe con un fallo o una pregunta sin responder.
- Empates: mismo puesto.

El servidor mide el tiempo con su propio reloj. Acepta solo la primera
respuesta de cada jugador, hasta 400 ms después del final (margen por la
conexión). La respuesta correcta no viaja a los teléfonos hasta que termina la
pregunta.

## Conectar a los alumnos

**Misma red Wi-Fi.** Es lo más simple, pero muchos Wi-Fi de colegios tienen
*aislamiento de clientes*: los aparatos no se ven entre sí y los alumnos no
podrán abrir la dirección. Si pasa, **usa el punto de acceso (hotspot) de tu
teléfono**: conecta tu Mac y a los alumnos a esa red.

**Por internet con un túnel** (sirve cuando no hay red común). Con el servidor
corriendo, en otra terminal:

```sh
cloudflared tunnel --url http://localhost:3000
```

Los túneles rápidos de Cloudflare no necesitan cuenta. Te da una dirección
`https://algo.trycloudflare.com`: esa es la de los alumnos. Otra opción es
Tailscale Funnel (`tailscale funnel 3000`). Para el profesor sigue valiendo
`http://localhost:3000/host?key=...`. Con un túnel el servidor toma la IP real
del alumno de la cabecera `cf-connecting-ip` o `x-forwarded-for` (solo si la
petición llega desde el propio Mac).

Los túneles rápidos de Cloudflare no dejan pasar el stream en vivo (SSE): llegan
las cabeceras pero nunca los datos. Por eso, si en 5 segundos no llega nada por
`/events`, el navegador pasa solo a pedir el estado a `/api/state` cada 0,6 s.
Las preguntas pueden aparecer hasta medio segundo tarde, pero el tiempo de cada
respuesta lo sigue midiendo el servidor.

## Límites

- Todo vive **en memoria**. Si reinicias el servidor se pierden las salas y los
  puntos. Descarga el CSV antes de cerrar.
- Máximo 60 jugadores por sala y 20 salas a la vez.
- Una sala se borra sola tras 6 horas sin actividad.
- Hay un límite sencillo por IP al unirse y al responder, para frenar abusos.
- El servidor no ejecuta nada que le manden los clientes. Los apodos solo
  admiten letras, números, espacios, emojis y unos pocos signos, y la página
  nunca arma HTML con ellos.

## Privacidad

Solo se guarda el apodo (hasta 16 caracteres) y los puntos, y solo en la
memoria del servidor mientras dura la sala. No hay cuentas, ni cookies, ni
base de datos, ni se escribe nada en disco. En el teléfono del alumno el
navegador guarda en `localStorage` el código, el apodo y un token para poder
volver a entrar. El CSV lo genera el profesor y lo guarda donde quiera.
Hay una lista corta de palabras groseras que no se aceptan como apodo; el
profesor puede además expulsar a un jugador desde la API (`kick`) si hiciera
falta.

## Archivo de preguntas

Por defecto `reto/questions.json`. Con otra ruta:

```sh
RETO_QUESTIONS=reto/test/questions.sample.json bun reto/server.ts
```

El archivo se lee cada vez que se crea una sala, así que puedes corregir una
pregunta sin reiniciar (las salas ya creadas conservan su copia). Formato:

```json
{
  "version": 1,
  "steps": [
    {
      "step": 1,
      "title": "El coche",
      "questions": [
        {
          "q": "¿Qué parte gira las ruedas de delante?",
          "options": ["La batería", "El servo", "El ventilador", "La cámara"],
          "answer": 1,
          "why": "El servo mueve las ruedas de delante.",
          "ref": "Paso 1, sección Dirección"
        }
      ]
    }
  ]
}
```

- `answer` es la posición de la opción correcta, **empezando en 0** (0 es la A).
- Cada pregunta lleva de 2 a 4 opciones. Se pintan como A, B, C y D, con forma
  y color, para que se distingan sin ver colores.
- `why` es la frase que se muestra al revelar. `ref` es dónde repasarlo.
- Para añadir preguntas, agrégalas a la lista `questions` de su paso; para un
  paso nuevo, añade otro objeto a `steps` con su `step` y su `title`.
- Si algo está mal escrito, el servidor avisa en la terminal y al crear la sala.

Opcional para pruebas: `RETO_DURATION_MS=5000` cambia el tiempo por pregunta.

## Pruebas

```sh
bun reto/test/e2e.ts
```

Arranca un servidor en un puerto libre con `reto/test/questions.sample.json`,
simula tres jugadores y una partida completa, y comprueba reglas, puntos,
rachas, reconexión, clave del profesor, que la respuesta no se filtre antes de
tiempo, apodos, CSV y caducidad de salas.
