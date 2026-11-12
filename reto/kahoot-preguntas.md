# Preguntas del Reto (el Kahoot del curso)

50 preguntas en 10 pasos de la guía del Open. La respuesta correcta va en negrita y con ✅. Cada pregunta trae la explicación que se muestra al revelar y dónde repasarlo.

Este archivo se genera desde `reto/questions.json` con `python3 reto/questions_to_md.py`. No lo edites a mano: cambia el JSON, pasa `python3 reto/validate_questions.py` y vuelve a generarlo.

## Paso 0. Instalar y preparar

### 0.1. Abres una terminal nueva y ros2 run my_open drive no encuentra el paquete, aunque ya compilaste. ¿Qué falta?

- A. Volver a compilar con colcon build
- B. Instalar ROS 2 y Gazebo otra vez
- C. Correr rosdep install en src
- **D. Hacer source install/setup.bash** ✅

Por qué: Cada terminal nueva necesita su source (en macOS, tools/macos_env.sh). Si una orden de ROS «no existe», casi siempre falta un source.

Repasar: paso 0, 'Seis palabras que vas a leer' y 'Resumen'

### 0.2. keyboard_drive y tu estrategia mueven el mismo coche sin conocerse. ¿Cómo es posible?

- **A. Publican en el mismo tópico, /cmd_vel_raw** ✅
- B. Las dos escriben órdenes en la carpeta install/
- C. colcon las une en un solo programa
- D. Las dos leen el cable USB del LiDAR

Por qué: Los nodos se pasan mensajes por tópicos con nombre. Quien publica en /cmd_vel_raw pide velocidades al coche.

Repasar: paso 0, 'Seis palabras que vas a leer' y 'Conduce con el teclado'

### 0.3. Tu ronda puntúa casi 0 aunque el código esté bien. ¿Qué es lo más probable?

- A. El LiDAR del simulador está roto
- **B. Quedan procesos de una ronda anterior** ✅
- C. Falta correr rosdep install antes de compilar
- D. La cámara va a 6 fotogramas

Por qué: Un gz sim o nodos de una ronda cortada se mezclan con la nueva. Ciérralos con clean_sim.sh antes de lanzar otra.

Repasar: paso 0, 'Si algo falla'

### 0.4. Compilaste con colcon build, sin --symlink-install. Editas drive.py y la ronda sigue igual que antes. ¿Por qué?

- A. Python sigue usando la versión vieja en caché
- B. Falta hacer git commit de tus cambios
- **C. install/ es una copia: hay que recompilar** ✅
- D. Gazebo carga el código solo al arrancar

Por qué: Sin --symlink-install, install/ guarda una copia de tus archivos. Recompila con colcon build --packages-select my_open tras cada cambio.

Repasar: paso 0, 'Ubuntu 24.04' y 'macOS (RoboStack)'

### 0.5. ¿Por qué git no ve tu carpeta src/my_open en la rama main?

- A. Porque ROS la borra al compilar
- B. Porque solo existe en la rama solutions
- C. Porque colcon la mueve a install/
- **D. Porque main la ignora: es tu código** ✅

Por qué: Git ignora src/my_open en main porque es tu código. La referencia de cada paso está en la rama solutions, con etiquetas step-N.

Repasar: paso 0, 'Tu código y la referencia'

## Paso 1. El coche

### 1.1. Un coche gira con el volante fijo. Si duplica su velocidad, ¿qué pasa con el radio del círculo?

- A. Se hace el doble de grande y la curva se abre
- B. Se reduce a la mitad porque va más rápido
- **C. Se queda igual; solo cambia lo que tarda** ✅
- D. Se hace cuatro veces más pequeño al ir veloz

Por qué: El radio solo depende del ángulo del volante. La velocidad solo cambia el tiempo que tarda la vuelta.

Repasar: paso 1, 'La idea'

### 1.2. En turn_radius cambias math.tan(steer) por steer. ¿Cuándo se nota el error?

- **A. Sobre todo con el volante a tope: un 3 %** ✅
- B. Siempre: el radio sale el doble de grande
- C. Solo cuando el coche va muy rápido
- D. Nunca: tan(δ) y δ son lo mismo

Por qué: Con ángulos pequeños tan(δ) se parece a δ: a 0,1 rad casi no cambia. Con el volante a tope el radio mínimo pasa de 0,370 a 0,382 m.

Repasar: paso 1, 'Rómpelo'

### 1.3. L = 0,1174 m es la distancia entre ejes. Con tan(0,25) = 0,255, ¿qué radio de giro sale con el volante a 0,25 rad?

- A. 0,03 m
- B. 2,17 m
- C. 4,6 m
- **D. 0,46 m** ✅

Por qué: Radio = L / tan(δ) = 0,1174 / 0,255 = 0,46 m. Es el círculo que usa la referencia.

Repasar: paso 1, 'La idea' y 'Prueba esto' (paso 1)

### 1.4. Pones el volante a 0,30 rad y el radio medido sale 0,43 m, no 0,38 m. ¿Qué pasa?

- A. La fórmula L / tan(δ) falla con ángulos pequeños
- **B. Cerca del tope, el simulador recorta el ángulo** ✅
- C. El coche va demasiado lento para girar con precisión
- D. El giroscopio acumula un sesgo muy grande en la curva

Por qué: El servo llega a 0,307 rad y desde unos 0,278 rad el simulador recorta el ángulo. El radio sale un 13 % mayor.

Repasar: paso 1, 'Pruébalo en el simulador' (tabla)

### 1.5. angular.z es la velocidad de giro que recibe el coche. ¿Qué hace cmd.angular.z = speed * math.tan(steer) / WHEELBASE?

- A. Pasa la velocidad de giro a ángulo de volante
- B. Calcula el radio del círculo que dibuja el coche
- **C. Pasa el ángulo del volante a velocidad de giro** ✅
- D. Suma lo que ha girado el coche hasta ahora

Por qué: El coche recibe una velocidad de giro, no un ángulo. Esa línea pasa de uno a otro con la primera fórmula.

Repasar: paso 1, 'Por qué la estrategia lo necesita'

## Paso 2. El rumbo

### 2.1. El giroscopio dice a qué velocidad gira el coche (rad/s). ¿Cómo se obtiene el rumbo con esas lecturas?

- A. Leyendo directamente el último valor del giroscopio
- B. Restando cada lectura a la anterior
- C. Promediando las últimas 30 lecturas
- **D. Sumando cada lectura multiplicada por el tiempo dt** ✅

Por qué: El rumbo es una suma: yaw = yaw + gyro * dt. Es como el cuentakilómetros, que suma velocidad por tiempo.

Repasar: paso 2, 'El rumbo es una suma'

### 2.2. Para medir el sesgo se promedian unas 30 lecturas con el coche quieto. ¿Por qué no basta una sola?

- A. El sesgo cambia mucho de una lectura a la otra
- **B. Una lectura trae ruido; el promedio, menos** ✅
- C. La primera lectura siempre vale cero
- D. Hacen falta 30 para sumar el giro total

Por qué: El sesgo es constante, pero cada lectura trae ruido. Con N lecturas el ruido del promedio baja unas √N veces: con 30, más de cinco veces.

Repasar: paso 2, 'El sesgo del giroscopio'

### 2.3. Sin calibrar, el coche acaba torcido 3,4° tras 10 s, pero su rumbo estimado marca 0. ¿Por qué no lo corrige?

- **A. Corrige el rumbo estimado, que no ve el error** ✅
- B. El volante no tiene recorrido para corregir 3,4°
- C. El LiDAR le confirma que va recto por el pasillo
- D. El giroscopio se pone a cero cada 10 segundos

Por qué: La ley mantiene el rumbo estimado en 0 y deja crecer el real. Nada dentro del coche avisa: por eso hay que calibrar.

Repasar: paso 2, 'Por qué el coche no se da cuenta'

### 2.4. Meta 3 rad y coche en -3 rad. wrap lleva el error a un valor entre -π y π. ¿Qué hace el volante sin wrap?

- A. Gira un poco a la derecha, por el camino más corto
- B. Se queda recto, porque el error es cero
- **C. Gira al tope a la izquierda, por el camino largo** ✅
- D. Gira al tope a la derecha, por el camino largo

Por qué: Sin wrap el error es 3 - (-3) = 6 rad y el volante va al tope. Con wrap el error es 0,28 rad y gira poco.

Repasar: paso 2, 'Rómpelo' y 'Volver al rumbo'

### 2.5. En heading_steer, el volante es KP * (goal - yaw) con KP = 0,8. ¿Qué significa KP?

- A. El rumbo máximo que puede tener el coche, en radianes
- B. Los metros que avanza el coche en cada instante
- **C. Radianes de volante por cada radián de error** ✅
- D. La velocidad de giro del giroscopio en rad/s

Por qué: KP dice cuánto volante das por cada radián de error. Con 0,1 rad de error, el volante gira 0,08 rad.

Repasar: paso 2, 'Volver al rumbo: una ley proporcional'

## Paso 3. Ver el campo

### 3.1. El LiDAR pierde un tramo de rayos seguidos sobre la pared de la isla. ¿Qué hace island_wall con ellos?

- A. Termina la pared en el primer rayo perdido
- **B. No los cuenta: un rayo perdido no vio nada** ✅
- C. Los cambia por la mediana de sus rayos vecinos
- D. Devuelve inf en los dos números de la pared

Por qué: Solo acaban la pared más de 2 rayos seguidos con retorno que ven más allá. Con 15 % de rayos perdidos, 0 de 500 escaneos fallan.

Repasar: paso 3, 'El final de la isla y el rumbo respecto a su pared' y 'Explora el escaneo'

### 3.2. En un escaneo, algunos rayos llegan como inf. ¿Qué significa inf?

- A. La pared está pegada al coche, a distancia cero
- B. El rayo midió justo 12 m, el máximo del LiDAR
- C. El LiDAR falló y el coche debe parar ya
- **D. Ese rayo no volvió: no vio ninguna pared** ✅

Por qué: inf es infinito: no volvió nada. Cerca del 4,3 % de los rayos se pierde al azar, y el código debe tolerarlo.

Repasar: paso 3, 'Qué entrega el LiDAR'

### 3.3. La pared de enfrente está a 1,5 m del centro del coche. El LiDAR va 0,128 m delante del centro. Coche recto: ¿qué mide?

- A. 1,628 m
- B. 1,5 m
- **C. 1,372 m** ✅
- D. 0,128 m

Por qué: El LiDAR ya está 0,128 m más cerca de la pared: 1,5 - 0,128 = 1,372 m. Las distancias se miden desde el LiDAR.

Repasar: paso 3, 'Qué entrega el LiDAR' y 'A lo largo de la pista'

### 3.4. De 63 rayos a la izquierda, uno se cuela por un hueco y mide 9 m. La pared está a 0,25 m. ¿Qué da el promedio?

- **A. Unos 0,40 m: se arrastra hacia el rayo** ✅
- B. 0,25 m: ignora al rayo, igual que la mediana
- C. 9 m: se queda con el valor del rayo salvaje
- D. Unos 0,14 m: se queda más cerca que la pared

Por qué: El promedio se arrastra hacia el rayo salvaje y da 0,396 m. La mediana ni lo nota y da 0,255 m.

Repasar: paso 3, 'Mediana, no promedio'

### 3.5. front_axis multiplica cada distancia por un coseno. ¿Para qué?

- A. Para ignorar los rayos perdidos, los que llegan como inf
- B. Para pasar las distancias de metros a centímetros
- C. Para quitar el ruido de 8 mm que tiene cada rayo
- **D. Para medir a lo largo de la pista, aunque vaya torcido** ✅

Por qué: Si el coche va torcido, una pared en diagonal parece más lejos. Con el coseno, front_axis mide a lo largo de la pista.

Repasar: paso 3, 'A lo largo de la pista'

## Paso 4. Seguir la isla

### 4.1. ¿Por qué la ley de carril desplaza la meta del rumbo en vez de mover el volante según el hueco?

- **A. Así hay una sola suma y el coche no zigzaguea** ✅
- B. Porque el volante no puede girar más de 17,6°
- C. Porque el LiDAR no mide bien el hueco
- D. Así el coche va más rápido en las rectas

Por qué: El volante cambia el rumbo y el rumbo la posición: dos sumas seguidas zigzaguean fácil. Con la meta queda una y el hueco baja sin pasarse.

Repasar: paso 4, 'Corregir el rumbo, no el hueco'

### 4.2. Una lectura mala dice que la isla está muy lejos. ¿Para qué sirve LIMIT?

- A. Para que el coche no se acerque a menos de 0,25 m
- B. Para que el coche no pase de 0,6 m/s en las rectas
- **C. Para que el desvío del rumbo no pase de un tope** ✅
- D. Para que el LiDAR no pierda rayos al azar en el escaneo

Por qué: LIMIT corta el desvío a 0,4 rad. Sin él, una lectura de 5 m pediría 1,4 rad (82°) de golpe.

Repasar: paso 4, 'Corregir el rumbo, no el hueco' y 'Rómpelo'

### 4.3. GAP es 0,25 m y GAIN 0,3 rad por metro. El coche está a 0,55 m de la isla. ¿Qué desvío pide la ley?

- A. 0,165 rad hacia la isla
- **B. 0,09 rad hacia la isla** ✅
- C. 0,09 rad alejándose de la isla
- D. 0,30 rad hacia la isla

Por qué: Error = 0,55 - 0,25 = 0,30 m. Desvío = 0,3 * 0,30 = 0,09 rad, hacia la isla porque está más lejos de lo debido.

Repasar: paso 4, 'Corregir el rumbo, no el hueco' (ejemplo 0,45 m)

### 4.4. Pones GAIN = 0 y el coche sale a 0,45 m de la isla. ¿Qué hueco tiene a los 14 s?

- A. Unos 0,287 m, la bajada lenta del GAIN de 0,3
- B. Unos 0,25 m: llega justo al hueco buscado
- C. Unos 0,00 m: choca con la isla poco a poco
- **D. Sigue en unos 0,456 m: nunca corrige** ✅

Por qué: Con GAIN = 0 el desvío es siempre 0 y el coche sigue como salió. Con GAIN 0,3 serían 0,287 m.

Repasar: paso 4, 'Cambia GAIN y mide' y 'Rómpelo'

### 4.5. En el mini, GAIN = 3,0 llega al hueco casi al instante. ¿Por qué la referencia usa 0,3?

- A. Con 3,0 el coche oscila y nunca llega
- **B. Lenta pero segura: con 3,0 rozó la isla** ✅
- C. Con 3,0 el volante pasa del límite de 25°
- D. Gazebo demostró que 0,3 es la mejor

Por qué: En el Open lab, GAIN 3,0 dio 5 de 6 rondas válidas: una rozó la isla. 0,3 es un valor probado y seguro, no el mejor medido.

Repasar: paso 4, 'Cambia GAIN y mide'

## Paso 5. La esquina

### 5.1. En la ley de rumbo, ¿cómo se pide una esquina de 90° a la izquierda?

- A. Restando π/2 (90°) a goal, la meta del rumbo
- B. Sumando π/2 (90°) a yaw, el rumbo que mide el giroscopio
- **C. Sumando π/2 (90°) a goal, la meta del rumbo** ✅
- D. Cambiando el hueco GAP de 0,25 m a 0,50 m

Por qué: Girar a la izquierda 90 grados es sumar π/2 a goal. yaw se mide con el giroscopio y no se toca.

Repasar: paso 5, 'Girar es mover la meta'

### 5.2. TURN_TOLERANCE = 0,3 rad. Al coche le faltan 0,5 rad para la meta de la esquina. ¿Qué volante pide arc_steer?

- **A. El tope, hacia el lado de la meta** ✅
- B. 0,40 rad: KP por el error, sin recortar
- C. Cero: la esquina ya terminó y va recto
- D. El tope, hacia el lado contrario a la meta

Por qué: Lejos de la meta el volante va a tope hacia ella. El tope se suelta cuando faltan 0,3 rad (17°) o menos.

Repasar: paso 5, 'Girar es mover la meta' y 'Lo difícil es soltar a tiempo'

### 5.3. KP = 0,8 y DAMPING = 0,1. Faltan 0,2 rad para la meta y el coche gira hacia ella a 1,0 rad/s. ¿Qué volante pide damped_steer?

- A. 0,16 rad
- B. 0,26 rad
- C. 0,307 rad
- **D. 0,06 rad** ✅

Por qué: Volante = KP * error - DAMPING * gyro = 0,8 * 0,2 - 0,1 * 1,0 = 0,06 rad. Cuanto más rápido gira el coche, menos volante pide.

Repasar: paso 5, 'Un freno para terminar: el amortiguamiento' y paso 5.1

### 5.4. Con un servo lento, soltar el tope a 17° de la meta se pasa 6,0° aun con amortiguamiento. ¿Qué lo deja en 0°?

- A. Soltar más tarde, a 7° de la meta
- **B. Soltar antes, a 29° de la meta** ✅
- C. Quitar el amortiguamiento de la ley
- D. Subir la velocidad de esquina a 1,28 m/s

Por qué: Un servo lento tarda más en enderezar las ruedas. Soltando a 29° (0,50 rad) con amortiguamiento el sobrepaso es 0,0°.

Repasar: paso 5, 'Cuándo soltar (medido en el laboratorio de Python)'

### 5.5. Con el servo real sueltas el tope a 3° de la meta en vez de a 17°. ¿Qué pasa en la esquina de 90°?

- **A. Se pasa 12°: el servo tarda en enderezar** ✅
- B. Llega justo: el tope lleva hasta la meta
- C. Se queda corto y nunca llega a los 90°
- D. Gira hacia el lado contrario de la esquina

Por qué: El servo tarda unas tres décimas en enderezar y el coche sigue girando. Soltando a 3° llega a 102°; a 17° no se pasa.

Repasar: paso 5, 'Lo difícil es soltar a tiempo' y 'Cuándo soltar'

## Paso 6. Cuándo girar

### 6.1. ¿Por qué el coche gira solo cuando se cumplen varias condiciones a la vez y no con una sola medida?

- A. Para que el coche gire más rápido en cada esquina
- **B. Una sola medida puede engañar, como un rayo suelto** ✅
- C. Porque el LiDAR solo entrega un número por cada escaneo
- D. Porque el giroscopio no mide la pared de enfrente

Por qué: Girar a destiempo es el error más caro: pronto se come la isla, tarde choca. Un rayo suelto o la pared cerca no bastan.

Repasar: paso 6, 'Girar a destiempo es el error más caro'

### 6.2. Al acabar la isla, left salta a unos 2,5 m. ¿Por qué el nodo usa entonces GAP como distancia a la isla?

- A. Para poder girar otra vez al instante
- B. Para frenar antes de entrar en la esquina
- C. Para que el LiDAR no cuente rayos perdidos
- **D. Para no girar hacia donde ya no hay isla** ✅

Por qué: Con la isla terminada, la ley de carril pediría acercarse a una pared que no existe. Con GAP el error es cero y no hay desvío.

Repasar: paso 6, 'Sin offset en la esquina'

### 6.3. LOOKAHEAD es 0,35 m y END_CLEARANCE 0,08 m. La isla está a 0,40 m del coche. ¿Hasta qué distancia cuenta el final de la isla?

- **A. 0,32 m** ✅
- B. 0,35 m
- C. 0,27 m
- D. 0,48 m

Por qué: El límite es min(0,35; 0,40 - 0,08) = min(0,35; 0,32) = 0,32 m. Con la isla a 0,30 m bajaría a 0,22 m.

Repasar: paso 6, 'Dos detalles de la condición 3' y paso 6.5

### 6.4. Cuentas el final de la isla en cada tick (50 por segundo) y no por escaneo (10 por segundo). ¿Qué pasa?

- A. La puerta nunca confirma el final y no gira
- B. El coche gira a la derecha en vez de a la izquierda
- **C. Un solo escaneo basta para confirmar y gira antes** ✅
- D. Nada cambia: el resultado es el mismo

Por qué: El mismo escaneo llega 5 ticks seguidos. Contado cada vez, 'dos escaneos seguidos' se cumple con uno solo.

Repasar: paso 6, 'Un escaneo cuenta una vez' y 'Rómpelo'

### 6.5. Al empezar la recta la pared de enfrente está a 2,40 m. Luego mide 2,10 m y después, con ruido, 2,13 m. ¿Qué guarda StraightDistance?

- A. 0,27 m
- **B. 0,30 m** ✅
- C. 2,13 m
- D. 0,03 m

Por qué: La distancia recorrida es lo que se acercó la pared: 2,40 - 2,10 = 0,30 m. Nunca baja: el coche no fue marcha atrás.

Repasar: paso 6, 'La distancia recorrida: la pared de enfrente, no un encoder' y paso 6.6

## Paso 7. La misión

### 7.1. Dentro de RUN no hay más estados. ¿Cómo sabe la misión si sigue la pared, gira o termina?

- A. Con estados FOLLOW, TURN y FINISH
- B. Con la pose real que guarda el árbitro
- C. Con los segundos pasados desde Start
- **D. Con dos datos: completed y arc** ✅

Por qué: completed cuenta las esquinas hechas y arc dice si hay una en curso. FOLLOW, TURN y FINISH son solo nombres calculados con esos dos datos.

Repasar: paso 7, 'Tres estados y dos datos'

### 7.2. TURN_FRONT = 1,19 m, BRAKE_MARGIN = 0,15 m y BRAKE_GAIN = 2,0. La pared de enfrente está a 1,44 m. ¿Qué velocidad pide la recta?

- **A. 1,20 m/s** ✅
- B. 1,28 m/s
- C. 1,00 m/s
- D. 0,20 m/s

Por qué: gap = 1,44 - 1,19 - 0,15 = 0,10 m. Velocidad = min(1,28; 1,0 + 2,0 * 0,10) = 1,20 m/s. Así frena antes de cada esquina.

Repasar: paso 7, 'Frenar antes de cada esquina' y paso 7.5

### 7.3. ¿Por qué toda la lógica de la misión está en mission.py, sin ROS, y drive.py solo reenvía?

- A. Los nodos de ROS no pueden guardar variables
- B. Así el código va más rápido en el coche
- **C. Así corre en pytest, en el lab y en Gazebo** ✅
- D. Gazebo no acepta lógica escrita en Python

Por qué: La misma misión corre en el nodo, en el Open lab y en las pruebas. Si algo falla en Gazebo, casi siempre lo reproduces con escenas a mano.

Repasar: paso 7, 'La lógica es Python puro; el nodo es una envoltura'

### 7.4. Pones WALL_HEADING_GAIN = 0 y el giroscopio lee un 6 % de menos. ¿Qué ves en el laboratorio (36 rondas)?

- A. Las 36 rondas siguen válidas con 30 puntos
- **B. 0 de 36 válidas: 33 chocan en una recta** ✅
- C. 34 válidas; las otras dos paran fuera de la salida
- D. 18 válidas; las otras 18 sacan 27 puntos

Por qué: Con cada escaneo la pared de la isla corrige 0,3 de la diferencia con el giroscopio. Sin eso, los 5° por esquina se acumulan.

Repasar: paso 7, 'El giroscopio se queda corto: la pared de la isla lo corrige' y 'Rómpelo'

### 7.5. ¿Por qué _braking se queda en True para siempre una vez que empieza a frenar?

- **A. Para que un rayo con ruido no reinicie el motor** ✅
- B. Para que el coche no vuelva a calibrar el giroscopio
- C. Para guardar la distancia de la vuelta 3
- D. Para que la cámara no cambie el sentido

Por qué: Si un rayo con ruido hace que la pared parezca más lejos, sin el enclavamiento el motor arrancaría otra vez.

Repasar: paso 7, 'El final de la vuelta' y paso 7.7

## Paso 8. La dirección

### 8.1. Lanzas la ronda con direction:=cw. ¿Cómo sabe tu nodo hacia dónde dar las vueltas?

- A. Lee el argumento direction del lanzador
- B. Prueba un sentido y cambia si choca
- **C. Lo decide la cámara con las dos líneas** ✅
- D. Siempre va en sentido antihorario

Por qué: direction solo cambia hacia dónde aparece mirando el coche. El nodo no lo recibe: azul más larga vota +1 (ccw) y naranja más larga -1 (cw).

Repasar: paso 8, 'Dos líneas de colores' y 'Pruébalo en el simulador'

### 8.2. Si la misión no recibe ningún sentido de la cámara en 15 s, ¿qué hace?

- **A. Se detiene (DIRECTION_FAULT) en vez de apostar** ✅
- B. Elige antihorario por defecto y arranca la ronda
- C. Elige un sentido al azar y arranca la ronda
- D. Sigue esperando sin límite de tiempo hasta que vote

Por qué: Parar es mejor que adivinar. Sin voto en 15 s, la misión se detiene con DIRECTION_FAULT.

Repasar: paso 8, 'Cómo usa la misión el sentido'

### 8.3. Una imagen mide azul 80 px y naranja 200 px. La confianza es |azul - naranja| / la línea más larga. ¿Cuánto vale?

- A. 0,40
- B. 1,50
- C. 0,28
- **D. 0,60** ✅

Por qué: |80 - 200| / 200 = 0,60. Pasa el mínimo de 0,25 y la naranja es más larga, así que vota -1.

Repasar: paso 8, 'Una imagen puede mentir'

### 8.4. Con CONFIRMATIONS = 1, un solo cuadro con la naranja más larga aparece entre cuadros con la azul más larga. ¿Qué pasa?

- A. La decisión sigue en +1: un cuadro no cambia nada
- **B. La decisión salta a -1 en ese cuadro** ✅
- C. La decisión se queda en 0 y nunca llega a decidir
- D. La misión se detiene con DIRECTION_FAULT

Por qué: Con 1, un cuadro basta para decidir y un reflejo puede empezar la ronda al revés. Con 5, un cuadro malo reinicia la cuenta.

Repasar: paso 8, 'Rómpelo'

### 8.5. Para decidir hacen falta 5 votos seguidos iguales. Llegan 3 votos +1, una imagen sin voto y luego más +1. ¿Cuántos +1 hacen falta?

- A. 2
- B. 3
- **C. 5** ✅
- D. 8

Por qué: La imagen sin voto reinicia la cuenta a 0. Hacen falta 5 votos seguidos otra vez (en la tabla decide en la imagen 9).

Repasar: paso 8, 'Una imagen puede mentir' (tabla de ejemplo)

## Paso 9. Puntuar y medir

### 9.1. La estrategia termina y marca DONE. ¿Cuándo cuentan los 3 puntos de la parada para el juez?

- A. Siempre que la estrategia diga DONE, sin comprobar nada
- **B. Si todo el coche para dentro de la sección de salida** ✅
- C. Si el coche para en cualquier sección del tapete
- D. Si el coche para antes de que acabe la vuelta 3

Por qué: El DONE de la estrategia no da puntos. El juez mira la pose real: todo el coche en la sección de salida y quieto 15 s.

Repasar: paso 9, 'Qué puntúa el juez'

### 9.2. En square100 ccw, el Open lab da 33,78 s y Gazebo 35,56 s. ¿Qué conclusión es correcta?

- A. Tu estrategia es 1,78 s más lenta en Gazebo que en el lab
- B. El lab es más fiable porque corre mucho más rápido
- C. Gazebo mide mal, así que hay que fiarse del lab
- **D. No se comparan: son dos modelos distintos** ✅

Por qué: El lab no modela el deslizamiento de las ruedas ni parte de la dinámica de Gazebo. Filtra ideas; Gazebo decide.

Repasar: paso 9, 'El Open lab: filtro rápido, no autoridad'

### 9.3. Intento A: 27 puntos en 30 s. Intento B: 30 puntos en 40 s. ¿Cuál es mejor según la guía?

- A. A: es 10 s más rápido y eso decide
- B. Empatan: puntos y tiempo pesan igual
- **C. B: más puntos, aunque tarde más** ✅
- D. A: 27 puntos bastan si va más rápido

Por qué: Primero los puntos, después el tiempo. Un intento de 27 puntos en 30 s pierde contra uno de 30 puntos en 40 s.

Repasar: paso 9, 'Rondas, tiempo y orden'

### 9.4. Una ronda saca 30 puntos pero el sensor de contacto detecta que el coche tocó un muro. ¿Qué resumen sale?

- **A. 30 de 30, pero Válida: no (no_contacts)** ✅
- B. 27 de 30, porque el golpe resta 3 puntos
- C. 30 de 30 y Válida: sí, el contacto no cuenta
- D. La ronda se borra y no imprime nada

Por qué: Los puntos son la regla. 'Válida' pide además cero contactos, 1 cm de margen, volante hasta 25° y parada observada 30 s.

Repasar: paso 9, 'Qué es una ronda válida' y 'Cómo leer lo que imprime score'

### 9.5. Cambias TURN_FRONT y CRUISE_SPEED a la vez y la ronda tarda más. ¿Qué falla en tu método?

- A. Faltó correr la ronda solo en el lab
- B. Faltó comparar el tiempo de pared
- C. Nada: dos cambios ahorran rondas
- **D. No sabes cuál de los dos lo empeoró** ✅

Por qué: Un cambio por fila, con valor anterior y nuevo, y tres rondas antes de concluir. Si baja puntos o validez, se descarta.

Repasar: paso 9, 'Paso 9.3: anotar'
