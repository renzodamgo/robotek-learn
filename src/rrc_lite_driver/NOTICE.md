# Aviso de procedencia

Código basado en Hiwonder MentorPi (https://github.com/Hiwonder/MentorPi, commit fb6d9969). Incluido con el permiso de
Hiwonder, confirmado por el autor del repositorio. El código original no declara licencia; los cambios del equipo
Robotek están bajo MIT.

Cambios del equipo respecto a MentorPi (`driver/ros_robot_controller`): nombres (`rrc_lite_driver`, nodo `rrc_driver`,
tópicos `/rrc/...`), el servo de dirección (`SetAckerServoState` en `/rrc/acker_servo/set_state`, servo PWM 2),
reconexión del puerto serie, puerto serie como parámetro (`port`, por defecto `/dev/rrc_lite` o `/dev/ttyACM0`) y
reintentos con un mensaje claro cuando la placa no está enchufada. Solo se incluye lo que necesita el coche real.
