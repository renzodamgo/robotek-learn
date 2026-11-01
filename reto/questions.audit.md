# Auditoría del banco de preguntas (questions.json)

Fuente: las páginas `paso-0-instalar-y-preparar.md`, `paso-1-el-coche.md` a `paso-9-puntuar-y-medir.md` en
`Portfolio-Astro/src/content/learn/steps/open-strategy/` (se cita `pN:línea`) y el código de referencia del tag
`step-9` (extraído en `/tmp/q9`, se cita `archivo.py`). Las comprobaciones con código se corrieron con RoboStack
(`PYTHONPATH=/tmp/q9/src/my_open:/tmp/q9/src/robotek_course`). Id de pregunta: `S<paso>Q<n>`.

## Paso 0, Instalar y preparar
- S0Q1 (reescrita, 1 nov 2026): escenario de terminal nueva. p0 'Seis palabras': «Si una orden de ROS «no existe», casi siempre falta un `source`»; 'Resumen': «Cada terminal nueva necesita su `source`». Distractores: recompilar (ya compiló), reinstalar, rosdep ('Si algo falla': no hace falta).
- S0Q2 (reescrita, 1 nov 2026): p0 'Seis palabras' (nodos y tópicos, `/cmd_vel_raw`) y 'Conduce con el teclado': «El programa publica en `/cmd_vel_raw`, el mismo tópico que usará tu estrategia». Distractores: install/, colcon, el USB del LiDAR.
- S0Q3: p0:264 "La ronda puntúa casi 0 aunque el código esté bien ... Casi siempre quedan procesos de una ejecución anterior"; `clean_sim.sh`. Distractor rosdep: p0:263 dice que no hace falta; 6 fotogramas es la cámara (p0:172).
- S0Q4 (reescrita, 1 nov 2026): p0 macOS: «compila con `colcon build` a secas ... `install/` es una copia y tendrás que recompilar tu paquete con `colcon build --packages-select my_open` cada vez que lo edites». Distractores: caché de Python, git commit, Gazebo.
- S0Q5: p0:253 "Git ignora `src/my_open` en la rama `main`: es tu código ... una rama `solutions` con ... una etiqueta `step-N` por cada paso". Distractor: my_open no existe en solutions solamente; colcon copia (no mueve) a install/.

## Paso 1, El coche
- S1Q1: p1:35 "el radio no se mueve ... depende del ángulo y de nada más"; p1:44 `radio = L / tan(δ)`; `model.py::turn_radius` no recibe la velocidad.
- S1Q2 (reescrita, 1 nov 2026): p1 'Rómpelo' 1: «a 0,1 rad el radio pasa de 1,170 m a 1,174 m, pero con el volante a tope el radio mínimo pasa de 0,370 m a 0,382 m, un 3 % más». El tope 17,6° queda en S1Q4 y en el why de otras.
- S1Q3: p1:44 y p1:33. Comando: `0.1174/0.255` = 0.4604; `turn_radius(0.25)` = 0.45978 (p1:33 dice 0,46 m). Distractores: `0.1174*0.255` = 0.030, `0.255/0.1174` = 2.17, y el error de coma 4,6.
- S1Q4: p1:219 fila 0,30: modelo 0,380, medido 0,43, +13 %; p1:221 "el simulador recorta el ángulo cerca del tope del servo (0,307 rad), desde unos 0,278 rad". Comando: `turn_radius(0.3)` = 0.3795.
- S1Q5: p1:50-57 "el coche ... recibe un Twist ... La conversión es la primera fórmula"; "El simulador hace lo contrario" (de ahí el distractor invertido). `model.py::yaw_rate` = `speed * tan(steer) / WHEELBASE`.

## Paso 2, El rumbo
- S2Q1: p2:25-31 `yaw = yaw + gyro * dt`, "Es una integral"; `heading.py::update` hace `self.yaw += (gyro - self.bias) * dt`.
- S2Q2 (reescrita, 1 nov 2026): pregunta el porqué del promedio. p2 'El sesgo del giroscopio': sesgo constante, ruido que cambia en cada lectura, «con N lecturas el ruido del promedio baja unas √N veces ... más de cinco veces menos». Distractor clave: «el sesgo cambia en cada lectura» (es el ruido el que cambia).
- S2Q3 (reescrita, 1 nov 2026): p2 'Por qué el coche no se da cuenta': «La ley de control mantiene el rumbo estimado en 0, así que el coche corrige un error que no existe y deja que crezca el real ... nada dentro del coche te avisa». En el paso 2 no se usa el LiDAR (distractor).
- S2Q4: p2:62 "3 rad y -3 rad ... a solo 0,28 rad"; p2:66 y p2:242 "el volante gira al tope a la izquierda (el camino largo)". Comando: `clamp(0.8*(3-(-3)), -STEER_LIMIT, STEER_LIMIT)` = +0.30718 (izquierda); `heading_steer(3,-3)` con wrap = -0.2265 (derecha, poco).
- S2Q5: p2:57 y p2:60 "KP = 0,8 ... Si el error es de 0,1 rad el volante gira 0,08 rad"; p2:168 "cuántos radianes de volante das por cada radián de error". Comando: `heading_steer(0.1, 0)` = 0.08.

## Paso 3, Ver el campo
- S3Q1 (reescrita, 1 nov 2026): p3 'El final de la isla' (recta ajustada, se detiene con más de 2 rayos seguidos fuera, `WALL_TOLERANCE` 3 cm) y 'Explora el escaneo' (15 % de pérdidas: 70 de 500 fallan; 30 mm: 386 de 500; `left` sigue estable); 'Resumen': «`island_end` es el número frágil: depende de una recta ajustada a pocos rayos». Distractor: left es la mediana, pero island_end no usa el promedio.
- S3Q2: p3:34 "Rayos perdidos 4,3 % al azar ... llegan como `inf` (infinito: «no volvió nada»)". El rango es 0,12 a 12 m (p3:29): el distractor "midió justo 12 m" es falso porque inf es "no volvió nada".
- S3Q3: p3:36 "0,128 m delante de base_link"; p3:62 "1,5 - 0,128 * cos(0,3) = 1,378: la pared está a 1,5 m"; p3:235 "la pared de enfrente a 1,372 m" (coche recto). Comando: `1.5-0.128` = 1.372.
- S3Q4: p3:54-56 "63 rayos ... un rayo ... 9 m ... mediana 0,255 m y promedio 0,396 m"; p3:459. Comando: con `np.mean` en `perceive`, `left` = 0.39595; con mediana 0.25534.
- S3Q5: p3:60-62 "una pared vista en diagonal parece más lejos ... mide a lo largo de la pista ... range * cos(ángulo respecto a la pista)"; `perception.py::front_axis`.

## Paso 4, Seguir la isla
- S4Q1 (reescrita, 1 nov 2026): pregunta el porqué. p4 'Corregir el rumbo, no el hueco': «El volante cambia el rumbo y el rumbo cambia la posición: son dos «sumas» seguidas ... Con la meta del rumbo, solo queda una». La guía avisa que es una explicación de diseño; lo medido es que el banco no oscila.
- S4Q2 (reescrita): p4:48 `LIMIT` 0,4 rad (23°); p4:52 "Si el hueco fuera de 5 m ... 0,3 * 4,75 = 1,4 rad (82°): para eso está LIMIT"; p4:222. Comando: `lateral_bias(5.0)` = 0.4. La pregunta no usa números difíciles; solo la explicación (why) cita 0,4 rad y 82°. Distractores: 0,25 m es GAP, 0,6 m/s es una velocidad de otro paso, los rayos perdidos son del LiDAR.
- S4Q3: p4:44-48 (GAP 0,25, GAIN 0,3) y p4:52 (ejemplo análogo con 0,45 m). Comando: `lateral_bias(0.55)` = 0.09. Distractor 0,165 = 0,3*0,55 (olvida restar GAP); 0,30 es el error en metros.
- S4Q4: p4:72 fila GAIN 0: "0,456 m ... nunca corrige" a los 14 s; p4:73 fila 0,3: 0,287 m; p4:221 Rómpelo 1.
- S4Q5 (reescrita, 1 nov 2026): p4 'Cambia GAIN y mide': GAIN 3,0 en el mini 0,250 m a los 5 s; en el Open lab 5 de 6 válidas, «una inválida porque el coche rozó la isla»; «es un valor probado, no el mejor». Distractores: no oscila (mínimo 0,249), LIMIT corta el desvío, Gazebo no midió otras ganancias. `sign` sigue en el why de S4Q3 y en el paso 5.

## Paso 5, La esquina
- S5Q1: p5:25 "Girar a la izquierda 90 grados es sumar π/2 a goal"; `mission.py::_run` hace `self.goal = wrap(self.goal + sign * math.pi / 2)`.
- S5Q2: p5:43 "se suma `steer_ff = sign * STEER_LIMIT` ... No hace falta esperar a que el error crezca"; los distractores son las otras dos piezas (amortiguamiento p5:45, aflojado p5:43).
- S5Q3: p5:43 "a menos de `EXIT_ANGLE = 0.25` rad de la meta, el feed-forward se multiplica por distancia restante / 0.25"; `arc.py`: `scale = min(1, max(0, sign*wrap(goal-yaw)) / EXIT_ANGLE)`. Cálculo: 0.10/0.25 = 0.4.
- S5Q4 (reescrita, 1 nov 2026): p5 'Qué enseña cada término': «El amortiguamiento arregla el servo lento (10,6 a 0,2 grados), pero no el feed-forward equivocado (17,3)»; «El aflojado arregla el feed-forward equivocado». Distractores: lo que arregla el aflojado, la puerta (paso 6) y la calibración (paso 2).
- S5Q5 (reescrita, 1 nov 2026): p5 tabla, fila «solo rampa», servo lento 10,7°; «La rampa sola no evita el sobrepaso del servo lento»; 'Una rampa en vez de un salto': «Sirve para que la esquina y la recta usen la misma ley, y para saber qué velocidad de giro se espera».

## Paso 6, Cuándo girar
- S6Q1: p6:23-25 "Girar a destiempo es el error más caro ... Una sola medida no distingue bien ... un rayo suelto del LiDAR puede engañar".
- S6Q2 (reescrita, 1 nov 2026): p6 'Sin offset en la esquina': «Cuando la isla termina ... esa ley pediría girar hacia donde no hay nada. Por eso, si `inner` no es finito o es mayor que `ISLAND_OPEN`, se usa `GAP`»; p3 (left salta de 0,5 a unos 2,5 m). Distractores: rearmar la puerta (pide ver la isla), frenar, rayos perdidos.
- S6Q3: p6:40 "El límite pasa a `min(LOOKAHEAD, inner - END_CLEARANCE)` ... Con la isla a 0,30 m el límite baja a 0,22 m". Comando: `min(0.35, 0.40-0.08)` = 0.32. Con `TurnGate` real: isla a 0,30 y final a 0,30 no cuenta (False, False, False); isla a 0,40 y final a 0,30 sí cuenta (False, True, True).
- S6Q4: p6:48 "50 Hz ... 10 escaneos por segundo ... el mismo escaneo llega 5 ticks seguidos"; p6:327 Rómpelo 1. Comando: con `if True:` en lugar de `scene is not self._scene`, el mismo `Scene` repetido da `[False, True, True]` (confirma con un solo escaneo); con el código real da `[False, False, False, False, False]`.
- S6Q5: p6:332 "Gira una vez y no vuelve a girar. La puerta no se rearma"; p6:44 y `turn_gate.py`: `seen_again = isfinite(inner) and inner < ISLAND_OPEN and travelled >= ARM_DISTANCE`. Comando: tras `corner_started()` `armed` es False y `update` devuelve False.

## Paso 7, La misión
- S7Q1 (reescrita, 1 nov 2026): p7 'Tres estados y dos datos': «Dentro de `RUN` no hay más estados. Solo hay dos datos: `completed` ... y `arc` ... El nombre (`label`: `FOLLOW`, `TURN`, `FINISH`, `DONE`...) se calcula a partir de ellos». La pose real solo la usa el juez (p7 'El árbitro').
- S7Q2: p7:19 "avanza despacio hasta que la pared de enfrente está como estaba en Start"; p7:94; `mission.py::_finish` (`FINISH_SPEED` 0,15; frena si `front_axis <= _start_front + FINISH_LEAD`).
- S7Q3 (reescrita, 1 nov 2026): p7 'La lógica es Python puro; el nodo es una envoltura': «Tu estrategia corre en tres sitios ... En el nodo no hay decisiones: si algo falla en Gazebo, casi siempre puedes reproducirlo en una prueba». El dato 1,28 a 0,60 m/s sigue en el why de S1 y en la guía.
- S7Q4: p7:454 "`LAST_CORNER_SPEED = 1.28` | las 12 pruebas pasan | 18 de 36 válidas; las otras 18 dan 27 puntos: el coche no consigue parar dentro de la sección de salida".
- S7Q5: p7:95 "El frenado se enclava (`_braking`). Si un rayo con ruido hace que la pared parezca más lejos, el motor no se reinicia"; `mission.py::_finish` comentario "latched".

## Paso 8, La dirección
- S8Q1 (reescrita, 1 nov 2026): p8 'Pruébalo en el simulador': «El argumento `direction` de `round.launch.py` solo hace que el coche aparezca mirando a un lado u otro; el nodo no lo recibe, así que lo decide la cámara»; 'Dos líneas de colores' (azul +1, naranja -1).
- S8Q2: p8:82 "la misión espera hasta 15 s ... Si no llega, se detiene (DIRECTION_FAULT) en vez de apostar a un lado. Parar es mejor que adivinar"; `mission.py::_setup`.
- S8Q3: p8:51 "|300 - 60| / 300 = 0,80"; `direction.py`: `confidence = abs(blue-orange)/max(longest, 1.0)`. Cálculo: 120/200 = 0.6 (comando); 80 px >= `MIN_LENGTH` 40 y 0,6 >= 0,25, vota -1 (comando: `[0,0,0,0,-1]` tras 5 cuadros). Distractores: 80/200 = 0,40 (divide por la línea corta del otro lado), 120/80 = 1,50 (divide por la más corta) y 0,28 (inventado).
- S8Q4: p8:253 "con 1, la decisión salta a -1 en ese cuadro (+1, +1, +1, -1, +1, +1, +1); con 5 se queda en 0". Comando: `CONFIRMATIONS=1` da `[1,1,1,-1,1,1,1]`; con 5 da `[0,0,0,0,0,0,0]`.
- S8Q5 (reescrita): p8:49 `CONFIRMATIONS` 5; p8:53 "Una imagen que no vota, o que vota otra cosa, reinicia la cuenta"; p8:55-65 tabla: 3 votos +1, la imagen 4 no vota (cuenta a 0), y decide en la imagen 9, o sea tras 5 votos +1 más. Comando: `DirectionVote` con 3x(300,60), (20,5) y 6x(300,60) da `[0,0,0,0,0,0,0,0,1,1]`: decide con el 5.º +1 después del cuadro sin voto.

## Paso 9, Puntuar y medir
- S9Q1: p9:35 "1.3 Parada | 3 | parar con todo el coche dentro de la sección de salida, quieto 15 s"; p9:37 "El `DONE` de la estrategia tampoco da puntos: la parada solo cuenta si la pose real confirma ...".
- S9Q2: p9:106 "`square100` ccw da 32,58 s en el lab y 34,80 s en Gazebo. No se ha investigado de dónde sale esa diferencia"; p9:102 lo que el lab no modela; p9:108 "el lab filtra ideas; Gazebo decide"; p9:112 "No compares tiempos entre Gazebo y el lab". Cálculo: 34,80-32,58 = 2,22 (se usa como distractor porque la guía dice que no se comparan).
- S9Q3 (reescrita): p9:67 "Un intento de 27 puntos en 30 s pierde contra uno de 30 puntos en 40 s" (ejemplo de la propia página, sin números inventados); regla "primero los puntos, después el tiempo".
- S9Q4: p9:41-49 ronda válida (ningún contacto, margen 1 cm, 25°, parada 30 s); p9:87 "`Válida: no (no_contacts)`: tocó un muro"; p9:49 "el resumen distingue «Puntos» de «Válida»".
- S9Q5 (reescrita, 1 nov 2026): escenario con dos cambios a la vez. p9 'Paso 9.3: anotar': «un cambio por fila, con el valor anterior y el nuevo ... repite tres rondas antes de concluir ... descarta un cambio que baje puntos o validez»; 'Rondas, tiempo y orden': se compara el tiempo simulado, no el de pared.

## Notas
- 1 nov 2026: se reescribieron 16 preguntas que solo pedían recordar un nombre o una constante (S0Q1, S0Q2, S0Q4, S1Q2, S2Q2, S2Q3, S3Q1, S4Q1, S4Q5, S5Q4, S5Q5, S6Q2, S7Q1, S7Q3, S8Q1, S9Q5). Ahora preguntan por qué o qué pasa si, con la misma posición de la respuesta correcta. Las referencias nuevas citan la sección de la guía y no la línea, porque las páginas ganaron un vídeo al principio y las líneas se movieron.
- Ninguna pregunta se descartó. Se corrigieron durante la revisión: opciones correctas demasiado largas (15 casos), una frase de más de 20 palabras, y dos distractores que podían confundirse con hechos del curso (CONFIRMATIONS del paso 8 en una pregunta del paso 6).
- Límites: S3Q3 usa la geometría de la página (pared a 1,5 m del centro) y no un escaneo concreto; S5Q4 vale para servo lento con feed-forward y aflojado activos (fila de la tabla).
