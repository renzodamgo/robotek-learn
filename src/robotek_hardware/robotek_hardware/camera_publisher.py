#!/usr/bin/env python3
"""Publica la webcam USB en el topic /usb_cam/image_raw (ROS 2).

Si la camara no esta (o se desconecta) el nodo NO se cae: avisa con un mensaje claro y reintenta cada
`reopen_retry` s hasta que aparezca. Con `device:=''` (por defecto) busca la camara solo: la primera
/dev/v4l/by-id/*-video-index0 y, si no hay, /dev/video0.

Escrito para ESTA camara, medida con `v4l2-ctl --list-ctrls` y
`--list-formats-ext`:
  * Solo entrega YUYV 640x480 (no tiene MJPG). No se pide MJPG: fallaria en
    silencio y el driver caeria a YUYV igual.
  * NO tiene control de 'gain'. La luz se regula con la exposicion.
  * Tambien publica /usb_cam/image_raw/compressed (JPEG, calidad jpeg_quality)
    para verla en RViz por Wi-Fi; solo comprime si alguien esta suscrito.
  * Comparte el USB2 con el lidar. Por eso 15 fps y buffer de 1 cuadro, para no
    robarle ancho de banda al sensor que evita los choques.

Por defecto la camara queda en AUTO (auto-exposicion + auto-balance de blancos):
es lo que mejor se ve a simple vista. Si estas calibrando colores en HSV y
necesitas que el brillo y el matiz NO cambien solos entre cuadros, arranca con:

    ros2 run <paquete> camera_publisher --ros-args -p manual:=true

OJO -- los controles de la camara PERSISTEN entre ejecuciones. Si una corrida
anterior la dejo en manual y oscura, abrir un nodo "sin tocar nada" hereda ese
estado y se ve horrible sin razon aparente. Por eso este nodo SIEMPRE fija de
forma explicita el modo que quiere (auto o manual): asi arranca igual siempre.
"""

import glob
import os
import shutil
import subprocess
import time

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # Por id de USB, no por /dev/videoN: la Pi 5 tiene ~19 nodos de video propios (codec,
        # pisp_be) y el numero de la camara USB puede cambiar con el orden de arranque.
        # index0 = nodo de captura; index1 es el de metadatos. Vacio = buscarla sola.
        self.device_param = str(self.declare_parameter('device', '').value)
        self.device = self._buscar_dispositivo()
        self.width = int(self.declare_parameter('width', 640).value)
        self.height = int(self.declare_parameter('height', 480).value)
        self.fps = int(self.declare_parameter('fps', 15).value)
        self.manual = bool(self.declare_parameter('manual', False).value)
        # Estos solo se usan si manual:=true.
        self.exposure = int(
            self.declare_parameter('exposure_absolute', 120).value)  # 100us: 120 = 12ms
        self.wb_temp = int(
            self.declare_parameter('white_balance_temperature', 4000).value)
        self.brightness = int(self.declare_parameter('brightness', 32).value)  # -64..64
        self.jpeg_quality = int(self.declare_parameter('jpeg_quality', 50).value)  # 1..100

        # Si la camara se desconecta el descriptor abierto queda muerto: tras reopen_after fallos
        # seguidos se reabre sola, reintentando cada reopen_retry s (0 = desactivado).
        self.reopen_after = int(self.declare_parameter('reopen_after', 30).value)
        self.reopen_retry = float(self.declare_parameter('reopen_retry', 1.0).value)
        self._ultimo_reintento = 0.0

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, 'usb_cam/image_raw', 10)
        self.pub_jpeg = self.create_publisher(
            CompressedImage, 'usb_cam/image_raw/compressed', 10)
        self._fallos = 0
        self.cap = None
        self._abrir()
        self.timer = self.create_timer(1.0 / max(1, self.fps), self._publicar)

    def _buscar_dispositivo(self):
        """El dispositivo pedido; si no, la primera camara por id de USB; si no, /dev/video0."""
        if self.device_param:
            return self.device_param
        por_id = sorted(glob.glob('/dev/v4l/by-id/*-video-index0'))
        return por_id[0] if por_id else '/dev/video0'

    def _abrir(self):
        """Intenta abrir la camara. Sin camara: un aviso claro (cada reopen_retry s) y se sigue intentando."""
        self.device = self._buscar_dispositivo()
        if not os.path.exists(self.device):
            self.get_logger().warn(
                f'Camara no encontrada ({self.device} no existe): conectala; reintento cada '
                f'{self.reopen_retry:g} s. Comprueba con: ls /dev/v4l/by-id/ y v4l2-ctl --list-devices',
                throttle_duration_sec=10.0)
            return False
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().warn(
                f'No se pudo abrir la camara {self.device} (permisos? el usuario debe estar en el grupo '
                f'"video"; o la usa otro programa); reintento cada {self.reopen_retry:g} s',
                throttle_duration_sec=10.0)
            cap.release()
            return False
        self.cap = cap
        self._configurar_captura()
        self._configurar_imagen()
        self.get_logger().info(
            f'Camara {self.device} '
            f'{int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
            f'{int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} @ '
            f'{self.cap.get(cv2.CAP_PROP_FPS):.0f} fps, '
            f'modo={"manual" if self.manual else "auto"}')
        return True

    def _configurar_captura(self):
        """Formato, resolucion, fps y buffer. Verifica cada set: uno que falla
        tiene que verse en el log, no adivinarse."""
        ajustes = [
            (cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'), 'FOURCC'),
            (cv2.CAP_PROP_FRAME_WIDTH, self.width, 'WIDTH'),
            (cv2.CAP_PROP_FRAME_HEIGHT, self.height, 'HEIGHT'),
            (cv2.CAP_PROP_FPS, self.fps, 'FPS'),
            (cv2.CAP_PROP_BUFFERSIZE, 1, 'BUFFERSIZE'),  # siempre el cuadro mas fresco
        ]
        for prop, valor, nombre in ajustes:
            if not self.cap.set(prop, valor):
                self.get_logger().warn(f'La camara rechazo {nombre}={valor}')

    def _configurar_imagen(self):
        """Fija exposicion y balance por NOMBRE con v4l2-ctl.

        cap.set(CAP_PROP_EXPOSURE) es ambiguo entre builds de OpenCV (a veces
        100us, a veces log2 de segundos) y falla en silencio. v4l2-ctl escribe
        el control por nombre y se puede releer para confirmar.
        """
        if shutil.which('v4l2-ctl') is None:
            self.get_logger().warn(
                'v4l2-ctl no instalado: la camara queda como la dejo la ultima '
                'corrida (puede verse oscura o con colores raros).')
            return

        if self.manual:
            # El ORDEN importa: primero el modo, despues el valor. Con
            # auto_exposure en automatico, exposure_time_absolute esta inactive
            # y el valor se descarta sin avisar.
            ctrls = [
                ('auto_exposure', 1),                 # 1 = Manual
                ('exposure_time_absolute', self.exposure),
                ('white_balance_automatic', 0),
                ('white_balance_temperature', self.wb_temp),
                ('brightness', self.brightness),
            ]
        else:
            # AUTO: lo que mejor se ve. Reset EXPLICITO por si una corrida previa
            # dejo la camara en manual/oscura (los controles persisten).
            ctrls = [
                ('auto_exposure', 3),                 # 3 = Aperture Priority (auto)
                ('white_balance_automatic', 1),
                ('brightness', 0),                    # borra cualquier offset viejo
            ]

        for nombre, valor in ctrls:
            try:
                subprocess.run(
                    ['v4l2-ctl', '-d', self.device, '--set-ctrl', f'{nombre}={valor}'],
                    check=True, capture_output=True, timeout=2.0)
            except Exception as e:                    # noqa: BLE001
                self.get_logger().warn(f'No se pudo fijar {nombre}={valor}: {e}')

        # Releer solo lo que se fijo para confirmar que quedo aplicado.
        nombres = ','.join(n for n, _ in ctrls)
        try:
            r = subprocess.run(
                ['v4l2-ctl', '-d', self.device, '--get-ctrl', nombres],
                check=True, capture_output=True, text=True, timeout=2.0)
            self.get_logger().info(
                'Controles -> ' + r.stdout.strip().replace('\n', ' | '))
        except Exception as e:                        # noqa: BLE001
            self.get_logger().warn(f'No se pudo verificar los controles: {e}')

    def _reabrir(self):
        """Libera el descriptor muerto (o ausente) y reabre la camara, como mucho una vez cada reopen_retry s."""
        ahora = time.monotonic()
        if ahora - self._ultimo_reintento < self.reopen_retry:
            return
        self._ultimo_reintento = ahora
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        if self._abrir():
            self.get_logger().info(f'Camara reabierta tras {self._fallos} fallos seguidos')

    def _publicar(self):
        if self.cap is None:
            self._fallos += 1
            self._reabrir()
            return
        ret, frame = self.cap.read()
        if not ret:
            # Un warn por cuadro perdido inunda el log; avisar cada 30.
            if self._fallos % 30 == 0:
                self.get_logger().warn(f'Fallo al capturar cuadro (x{self._fallos + 1})')
            self._fallos += 1
            if self.reopen_after > 0 and self._fallos >= self.reopen_after:
                self._reabrir()
            return
        self._fallos = 0
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.pub.publish(msg)

        # JPEG solo si alguien lo pide (p. ej. RViz en otra PC): no gasta CPU en carrera.
        if self.pub_jpeg.get_subscription_count() > 0:
            ok, jpg = cv2.imencode(
                '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if ok:
                comp = CompressedImage()
                comp.header = msg.header
                comp.format = 'bgr8; jpeg compressed bgr8'
                comp.data = jpg.tobytes()
                self.pub_jpeg.publish(comp)

    def destroy_node(self):
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except Exception:                             # noqa: BLE001
            pass


if __name__ == '__main__':
    main()
