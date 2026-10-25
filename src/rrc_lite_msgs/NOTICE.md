# Aviso de procedencia

Código basado en Hiwonder MentorPi (https://github.com/Hiwonder/MentorPi, commit fb6d9969). Incluido con el permiso de
Hiwonder, confirmado por el autor del repositorio. El código original no declara licencia; los cambios del equipo
Robotek están bajo MIT.

Cambios del equipo respecto a MentorPi (`driver/ros_robot_controller_msgs`): el paquete se llama `rrc_lite_msgs` y
tiene un mensaje más, `SetAckerServoState` (`float64 duration`, `uint16 position` en microsegundos).
