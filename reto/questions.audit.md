# Auditoría del banco de preguntas (questions.json)

Fuente: las páginas `paso-1-el-coche.md` a `paso-9-puntuar-y-medir.md` en
`Portfolio-Astro/src/content/learn/steps/open-strategy/` (se cita `pN:línea`) y el código de referencia del tag
`step-9` (extraído en `/tmp/q9`, se cita `archivo.py`). Las comprobaciones con código se corrieron con RoboStack
(`PYTHONPATH=/tmp/q9/src/my_open:/tmp/q9/src/robotek_course`). Id de pregunta: `S<paso>Q<n>`.

## Paso 1, El coche
- S1Q1: p1:35 "el radio no se mueve ... depende del ángulo y de nada más"; p1:44 `radio = L / tan(δ)`; `model.py::turn_radius` no recibe la velocidad.
- S1Q2: p1:73 "Su tope son 17,6° (REAL_STEER_LIMIT = 0.30718)". Comando: `degrees(0.30718)` = 17.6001. Distractores 25° (límite del simulador, p1:73), 30° y 11,5° no aparecen como tope.
- S1Q3: p1:44 y p1:33. Comando: `0.1174/0.255` = 0.4604; `turn_radius(0.25)` = 0.45978 (p1:33 dice 0,46 m). Distractores: `0.1174*0.255` = 0.030, `0.255/0.1174` = 2.17, y el error de coma 4,6.
- S1Q4: p1:219 fila 0,30: modelo 0,380, medido 0,43, +13 %; p1:221 "el simulador recorta el ángulo cerca del tope del servo (0,307 rad), desde unos 0,278 rad". Comando: `turn_radius(0.3)` = 0.3795.
- S1Q5: p1:50-57 "el coche ... recibe un Twist ... La conversión es la primera fórmula"; "El simulador hace lo contrario" (de ahí el distractor invertido). `model.py::yaw_rate` = `speed * tan(steer) / WHEELBASE`.

## Paso 2, El rumbo
- S2Q1: p2:25-31 `yaw = yaw + gyro * dt`, "Es una integral"; `heading.py::update` hace `self.yaw += (gyro - self.bias) * dt`.
- S2Q2: p2:37 "Con el coche parado, la lectura es el sesgo (más ruido), así que basta el promedio de unas cuantas lecturas ... con N lecturas el ruido del promedio baja unas √N veces. Con 30 lecturas (0,6 s a 50 por segundo)"; p2:119 "El sesgo es su promedio"; `heading.py::calibrate` = `sum(samples) / len(samples)`.
- S2Q3: p2:35 "unos 0,006 rad/s ... 10 s ... 0,06 rad (3,4°)". Cálculo: 0.006*20 = 0.12 rad = 6.875° (comando). Distractores: valor de 10 s, error de coma (1,2) y de coma por el otro lado (0,012).
- S2Q4: p2:62 "3 rad y -3 rad ... a solo 0,28 rad"; p2:66 y p2:242 "el volante gira al tope a la izquierda (el camino largo)". Comando: `clamp(0.8*(3-(-3)), -STEER_LIMIT, STEER_LIMIT)` = +0.30718 (izquierda); `heading_steer(3,-3)` con wrap = -0.2265 (derecha, poco).
- S2Q5: p2:57 y p2:60 "KP = 0,8 ... Si el error es de 0,1 rad el volante gira 0,08 rad"; p2:168 "cuántos radianes de volante das por cada radián de error". Comando: `heading_steer(0.1, 0)` = 0.08.

## Paso 3, Ver el campo
- S3Q1: p3:41-50 "`perceive` devuelve una `Scene` con cuatro números ... left, right, front_axis, island_end"; p3:50 "Un número que no se ve vale inf".
- S3Q2: p3:34 "Rayos perdidos 4,3 % al azar ... llegan como `inf` (infinito: «no volvió nada»)". El rango es 0,12 a 12 m (p3:29): el distractor "midió justo 12 m" es falso porque inf es "no volvió nada".
- S3Q3: p3:36 "0,128 m delante de base_link"; p3:62 "1,5 - 0,128 * cos(0,3) = 1,378: la pared está a 1,5 m"; p3:235 "la pared de enfrente a 1,372 m" (coche recto). Comando: `1.5-0.128` = 1.372.
- S3Q4: p3:54-56 "63 rayos ... un rayo ... 9 m ... mediana 0,255 m y promedio 0,396 m"; p3:459. Comando: con `np.mean` en `perceive`, `left` = 0.39595; con mediana 0.25534.
- S3Q5: p3:60-62 "una pared vista en diagonal parece más lejos ... mide a lo largo de la pista ... range * cos(ángulo respecto a la pista)"; `perception.py::front_axis`.

## Paso 4, Seguir la isla
- S4Q1: p4:27-33 "En vez de mover el volante según el hueco, mueves la meta del rumbo"; p4:232 resumen; `lane.py::lane_steer` llama a `heading_steer(goal + lateral_bias(...), yaw)`.
- S4Q2 (reescrita): p4:48 `LIMIT` 0,4 rad (23°); p4:52 "Si el hueco fuera de 5 m ... 0,3 * 4,75 = 1,4 rad (82°): para eso está LIMIT"; p4:222. Comando: `lateral_bias(5.0)` = 0.4. La pregunta no usa números difíciles; solo la explicación (why) cita 0,4 rad y 82°. Distractores: 0,25 m es GAP, 0,6 m/s es una velocidad de otro paso, los rayos perdidos son del LiDAR.
- S4Q3: p4:44-48 (GAP 0,25, GAIN 0,3) y p4:52 (ejemplo análogo con 0,45 m). Comando: `lateral_bias(0.55)` = 0.09. Distractor 0,165 = 0,3*0,55 (olvida restar GAP); 0,30 es el error en metros.
- S4Q4: p4:72 fila GAIN 0: "0,456 m ... nunca corrige" a los 14 s; p4:73 fila 0,3: 0,287 m; p4:221 Rómpelo 1.
- S4Q5: p4:50 "`sign` vale +1 si la isla está a la izquierda ... y -1 si está a la derecha"; p4:116; `lane.py::lateral_bias` docstring; p4:184 "con la isla a la derecha todo es el espejo".

## Paso 5, La esquina
- S5Q1: p5:25 "Girar a la izquierda 90 grados es sumar π/2 a goal"; `mission.py::_run` hace `self.goal = wrap(self.goal + sign * math.pi / 2)`.
- S5Q2: p5:43 "se suma `steer_ff = sign * STEER_LIMIT` ... No hace falta esperar a que el error crezca"; los distractores son las otras dos piezas (amortiguamiento p5:45, aflojado p5:43).
- S5Q3: p5:43 "a menos de `EXIT_ANGLE = 0.25` rad de la meta, el feed-forward se multiplica por distancia restante / 0.25"; `arc.py`: `scale = min(1, max(0, sign*wrap(goal-yaw)) / EXIT_ANGLE)`. Cálculo: 0.10/0.25 = 0.4.
- S5Q4: p5:88 fila "+ feed-forward + aflojado" (sin amortiguamiento), servo lento 10,6; p5:95 "10,6 a 0,2 grados"; p5:101. Comando con `run_arc` (servo_delay 0.15, servo_tau 0.10): sin amortiguamiento 10.6°, versión final 0.2°; servo real 0.8°/0.4°. Distractor 17,5 es otra celda (feed-forward equivocado).
- S5Q5: p5:37 "`ramp(ref, goal, rate, dt)` mueve `ref` hacia `goal` como mucho `rate * dt` por tick, por el camino corto"; `arc.py::ramp`.

## Paso 6, Cuándo girar
- S6Q1: p6:23-25 "Girar a destiempo es el error más caro ... Una sola medida no distingue bien ... un rayo suelto del LiDAR puede engañar".
- S6Q2: p6:31 `TURN_FRONT = 1.19`; los distractores son `ARM_DISTANCE` 0,25 (p6:30), `LOOKAHEAD` 0,35 y `ISLAND_OPEN` 1,0 (p6:32); `turn_gate.py`.
- S6Q3: p6:40 "El límite pasa a `min(LOOKAHEAD, inner - END_CLEARANCE)` ... Con la isla a 0,30 m el límite baja a 0,22 m". Comando: `min(0.35, 0.40-0.08)` = 0.32. Con `TurnGate` real: isla a 0,30 y final a 0,30 no cuenta (False, False, False); isla a 0,40 y final a 0,30 sí cuenta (False, True, True).
- S6Q4: p6:48 "50 Hz ... 10 escaneos por segundo ... el mismo escaneo llega 5 ticks seguidos"; p6:327 Rómpelo 1. Comando: con `if True:` en lugar de `scene is not self._scene`, el mismo `Scene` repetido da `[False, True, True]` (confirma con un solo escaneo); con el código real da `[False, False, False, False, False]`.
- S6Q5: p6:332 "Gira una vez y no vuelve a girar. La puerta no se rearma"; p6:44 y `turn_gate.py`: `seen_again = isfinite(inner) and inner < ISLAND_OPEN and travelled >= ARM_DISTANCE`. Comando: tras `corner_started()` `armed` es False y `update` devuelve False.

## Paso 7, La misión
- S7Q1: p7:44-54 "SETUP ... RUN ... STOP"; "El nombre que ve el resto del sistema (label: FOLLOW, TURN, FINISH, DONE...) se calcula a partir de ellos"; `mission.py::label`.
- S7Q2: p7:19 "avanza despacio hasta que la pared de enfrente está como estaba en Start"; p7:94; `mission.py::_finish` (`FINISH_SPEED` 0,15; frena si `front_axis <= _start_front + FINISH_LEAD`).
- S7Q3 (reescrita): p7:97 "El comando 1,28 m/s no es la velocidad real: con el motor medido del coche, 1,28 llega a 0,60 m/s (`real_speed`)"; `mission.py` `CRUISE_SPEED = 1.28  # m/s commanded (the car really reaches 0.6)`. Comando: `real_speed(1.28)` = 0.6 y `real_speed(0.6)` = 0.42 (p1:255 "el coche real llega a 0,42 m/s con esa orden", de ahí el distractor 0,42). Distractor 0,15 es `FINISH_SPEED`.
- S7Q4: p7:454 "`LAST_CORNER_SPEED = 1.28` | las 12 pruebas pasan | 18 de 36 válidas; las otras 18 dan 27 puntos: el coche no consigue parar dentro de la sección de salida".
- S7Q5: p7:95 "El frenado se enclava (`_braking`). Si un rayo con ruido hace que la pared parezca más lejos, el motor no se reinicia"; `mission.py::_finish` comentario "latched".

## Paso 8, La dirección
- S8Q1: p8:31-33 "Azul más larga que naranja: vuelta antihoraria (ccw), voto +1"; `direction.py`: `vote = 1 if blue_length > orange_length else -1`.
- S8Q2: p8:82 "la misión espera hasta 15 s ... Si no llega, se detiene (DIRECTION_FAULT) en vez de apostar a un lado. Parar es mejor que adivinar"; `mission.py::_setup`.
- S8Q3: p8:51 "|300 - 60| / 300 = 0,80"; `direction.py`: `confidence = abs(blue-orange)/max(longest, 1.0)`. Cálculo: 120/200 = 0.6 (comando); 80 px >= `MIN_LENGTH` 40 y 0,6 >= 0,25, vota -1 (comando: `[0,0,0,0,-1]` tras 5 cuadros). Distractores: 80/200 = 0,40 (divide por la línea corta del otro lado), 120/80 = 1,50 (divide por la más corta) y 0,28 (inventado).
- S8Q4: p8:253 "con 1, la decisión salta a -1 en ese cuadro (+1, +1, +1, -1, +1, +1, +1); con 5 se queda en 0". Comando: `CONFIRMATIONS=1` da `[1,1,1,-1,1,1,1]`; con 5 da `[0,0,0,0,0,0,0]`.
- S8Q5 (reescrita): p8:49 `CONFIRMATIONS` 5; p8:53 "Una imagen que no vota, o que vota otra cosa, reinicia la cuenta"; p8:55-65 tabla: 3 votos +1, la imagen 4 no vota (cuenta a 0), y decide en la imagen 9, o sea tras 5 votos +1 más. Comando: `DirectionVote` con 3x(300,60), (20,5) y 6x(300,60) da `[0,0,0,0,0,0,0,0,1,1]`: decide con el 5.º +1 después del cuadro sin voto.

## Paso 9, Puntuar y medir
- S9Q1: p9:35 "1.3 Parada | 3 | parar con todo el coche dentro de la sección de salida, quieto 15 s"; p9:37 "El `DONE` de la estrategia tampoco da puntos: la parada solo cuenta si la pose real confirma ...".
- S9Q2: p9:106 "`square100` ccw da 32,58 s en el lab y 34,80 s en Gazebo. No se ha investigado de dónde sale esa diferencia"; p9:102 lo que el lab no modela; p9:108 "el lab filtra ideas; Gazebo decide"; p9:112 "No compares tiempos entre Gazebo y el lab". Cálculo: 34,80-32,58 = 2,22 (se usa como distractor porque la guía dice que no se comparan).
- S9Q3 (reescrita): p9:67 "Un intento de 27 puntos en 30 s pierde contra uno de 30 puntos en 40 s" (ejemplo de la propia página, sin números inventados); regla "primero los puntos, después el tiempo".
- S9Q4: p9:41-49 ronda válida (ningún contacto, margen 1 cm, 25°, parada 30 s); p9:87 "`Válida: no (no_contacts)`: tocó un muro"; p9:49 "el resumen distingue «Puntos» de «Válida»".
- S9Q5: p9:185 "un cambio por fila, con el valor anterior y el nuevo (`TURN_FRONT 1.19 a 1.10`) ... repite tres rondas ... descarta un cambio que baje puntos o validez, aunque sea más rápido"; p9:178 columna «Dónde» (lab o Gazebo) porque sus tiempos no se comparan.

## Notas
- Ninguna pregunta se descartó. Se corrigieron durante la revisión: opciones correctas demasiado largas (15 casos), una frase de más de 20 palabras, y dos distractores que podían confundirse con hechos del curso (CONFIRMATIONS del paso 8 en una pregunta del paso 6).
- Límites: S3Q3 usa la geometría de la página (pared a 1,5 m del centro) y no un escaneo concreto; S5Q4 vale para servo lento con feed-forward y aflojado activos (fila de la tabla).
