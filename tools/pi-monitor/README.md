# Raspberry Pi Bar (PiMonitor): salud del Raspberry Pi del coche en la barra de menús del Mac

App de SwiftUI para la barra de menús. Cada 2 s pregunta al Pi por SSH cómo está y lo
muestra; guarda un historial para ver cuándo se cayó el Wi-Fi o se reinició el Pi.

```bash
tools/pi-monitor/build-app.sh --install  # compila, crea build/Raspberry Pi Bar.app y lo copia a /Applications
open "/Applications/Raspberry Pi Bar.app"
```

Hace falta Xcode (o sus Command Line Tools) y macOS 14 o más nuevo. La primera vez, macOS pregunta si
PiMonitor puede buscar dispositivos en la red local: hay que permitirlo (Ajustes del Sistema →
Privacidad y seguridad → Red local). Para que arranque solo: Ajustes del Sistema → General →
Ítems de inicio → añadir `Raspberry Pi Bar.app`.

El icono es `icon/raspberry-pi-bar.svg` (una frambuesa con barras); `build-app.sh` lo pasa a `AppIcon.icns` con
`icon/svg2png.swift` e `iconutil`. Sin `--install` la app queda solo en `build/`.

## Qué muestra

| Sección | Datos | De dónde salen (en el Pi) |
|---|---|---|
| Barra de menús | batería y temperatura de la CPU; un triángulo si hay bajo voltaje o batería baja, `wifi.slash` sin conexión | |
| Energía | batería de la placa RRC Lite, entrada de 5 V del Pi, voltaje del núcleo, `get_throttled` explicado (rojo = ahora, naranja = desde el arranque) | `/rrc/battery`, `vcgencmd pmic_read_adc EXT5V_V`, `measure_volts`, `get_throttled` |
| CPU y GPU | uso y frecuencia de cada núcleo, temperatura, carga media, temperatura y relojes de la VideoCore (`arm`, `core`, `v3d`) | `/proc/stat`, `cpufreq`, `thermal_zone0`, `vcgencmd` |
| Memoria y disco | RAM, swap y `/` | `/proc/meminfo`, `df` |
| Wi-Fi | señal (dBm), calidad, red, bitrate, ping desde el Mac | `/proc/net/wireless`, `iw dev wlan0 link`, `ping` |
| ROS | procesos ROS en marcha (cada 2 s); cada 30 s, nodos y frecuencia de `/scan`, `/robotek/imu` y `/rrc/battery` | `ps`; un `rclpy` de 5 s |
| Historial | gráficas de los últimos 30 min, franja verde/roja de conexión de las últimas 2 h y eventos | |

**Eventos** (también en `~/Library/Application Support/PiMonitor/events.jsonl`): sin conexión
(y si el ping todavía responde), vuelta en línea con cuánto tiempo estuvo caído, **reinicio**
(cambia el `boot_id` del Pi: muestra a qué hora arrancó), avisos nuevos de `get_throttled`,
batería por debajo de 6,8 V, señal Wi-Fi débil (≤ −75 dBm), IP nueva y el stack ROS que arranca o se
detiene. Cada muestra queda en `samples-AAAA-MM-DD.csv` en la misma carpeta (botón «Abrir
registros»): sirve para ver qué pasaba (batería, 5 V, temperatura, señal) justo antes de una caída.

La batería solo se puede leer con el driver de la placa (`rrc_lite_driver`) en marcha.

## Cómo carga al Pi

- No instala nada en el Pi: manda un script corto de `sh` por la entrada estándar (`ssh … sh -s`),
  que solo lee `/proc`, `/sys`, `vcgencmd`, `iw` y `ps`.
- Una sola conexión SSH abierta (`ControlMaster`): cada consulta es un canal nuevo sobre ella, sin
  repetir el saludo de SSH.
- La sonda de ROS (`zsh -ic`, para cargar ROS y `ROS_DOMAIN_ID` como en una sesión normal) arranca
  un proceso `rclpy` unos 5 s cada 30 s, y solo si hay procesos ROS en marcha.
- Los scripts están en [Sources/PiMonitor/Scripts.swift](Sources/PiMonitor/Scripts.swift).

## Encontrar el Pi si cambió de IP

Prueba primero la última IP conocida, `192.168.8.153` y `192.168.8.242`. Tras 3 fallos seguidos
(y cada 20 s mientras siga sin conexión) prueba el puerto 22 en toda la `192.168.8.0/24` y en la
red /24 del Mac. Solo acepta una dirección si su llave ED25519 es la del Pi del coche
(`SHA256:NEHaEUFIi9CALGWtaSNaSTT46r0xAvXM1qDEDUWsrGg`). Luego conecta con
`HostKeyAlias=192.168.8.242`, el nombre con el que está la llave en `~/.ssh/known_hosts`. El botón
«Buscar el Pi» fuerza la búsqueda.

Requisito: entrar por SSH al Pi con llave, sin contraseña (`ssh-copy-id revine@<ip>`).

## Otro coche u otro usuario

```bash
defaults write org.robotek.PiMonitor user pi
defaults write org.robotek.PiMonitor hostKeyAlias 192.168.1.50
defaults write org.robotek.PiMonitor fingerprint SHA256:...   # ssh-keyscan -t ed25519 IP | ssh-keygen -lf -
defaults write org.robotek.PiMonitor subnets 192.168.1          # redes /24 a explorar, separadas por comas
defaults write org.robotek.PiMonitor host 192.168.1.50          # IP con la que empezar
```

Lo mismo sirve como argumentos para una prueba, sin guardarlos:
`open build/PiMonitor.app --args -host 10.0.0.5 -user yo -hostKeyAlias 10.0.0.5 -openWindow YES`
(`-openWindow YES` abre el panel también como ventana normal).
