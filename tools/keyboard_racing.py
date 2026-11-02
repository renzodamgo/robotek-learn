#!/usr/bin/env python3
r"""Drive the real car like a racing game, from a small window on your computer (not over ssh).

    # on the car:      ros2 launch robotek_course car.launch.py teleop:=true
    # on the computer: bash tools/keyboard_racing.sh            (or this script with ROS and the car's network set)
    # on Windows:      powershell -ExecutionPolicy Bypass -File tools\keyboard_racing.ps1 -Speed 0.4

Hold the arrows (or W A S D) and combine them: up = forward, down = back, left/right = steer while held. Releasing
everything stops the car and centres the wheels. If the window loses focus or closes, the car stops. It publishes
/cmd_vel_raw like a strategy (linear.x = speed, angular.z = speed * tan(steer) / wheelbase), at 20 Hz, so base_adapter's
watchdog, caps and wall guard still apply.

A terminal only sends characters, so it cannot tell that two keys are held at once; a window gets key press and key
release events, which is what a racing game needs. The key repeat of the system (press/release pairs while a key is
held) is filtered: a release only counts if no press of the same key follows within RELEASE_DELAY.
"""
import math
import sys
import tkinter as tk

import rclpy
from geometry_msgs.msg import Twist

SPEED = 0.2          # m/s: 1 rps on the team's encoder motor (less did not move the car)
STEER = 15.0         # deg of wheel angle while left/right is held
WHEELBASE = 0.1174   # m (robot.yaml)
RELEASE_DELAY = 80   # ms
KEYS = {'Up': 'up', 'w': 'up', 'W': 'up', 'Down': 'down', 's': 'down', 'S': 'down',
        'Left': 'left', 'a': 'left', 'A': 'left', 'Right': 'right', 'd': 'right', 'D': 'right'}


def main():
    speed = float(sys.argv[1]) if len(sys.argv) > 1 else SPEED
    steer_deg = float(sys.argv[2]) if len(sys.argv) > 2 else STEER
    rclpy.init()
    node = rclpy.create_node('keyboard_racing')
    pub = node.create_publisher(Twist, '/cmd_vel_raw', 10)
    held = set()
    pending = {}

    root = tk.Tk()
    root.title('Robotek: conducir')
    label = tk.Label(root, font=('Helvetica', 16), justify='left', padx=20, pady=20)
    label.pack()
    tk.Label(root, text='Mantén ↑ ↓ ← → (o W A S D) y combínalos. Suelta todo para parar.\n'
                        'Si la ventana pierde el foco, el coche se para.', padx=20, pady=10).pack()

    def command():
        v = speed * ((1 if 'up' in held else 0) - (1 if 'down' in held else 0))
        steer = math.radians(steer_deg) * ((1 if 'left' in held else 0) - (1 if 'right' in held else 0))
        return v, steer

    def publish():
        v, steer = command()
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(v * math.tan(steer) / WHEELBASE)
        pub.publish(cmd)
        label.config(text=f'velocidad {v:+.2f} m/s\ndirección {math.degrees(steer):+.0f}°  '
                          f'({"izquierda" if steer > 0 else "derecha" if steer < 0 else "recta"})')
        rclpy.spin_once(node, timeout_sec=0)
        root.after(50, publish)

    def press(event):
        key = KEYS.get(event.keysym)
        if key:
            if key in pending:
                root.after_cancel(pending.pop(key))
            held.add(key)

    def release(event):
        key = KEYS.get(event.keysym)
        if key:
            def really():
                pending.pop(key, None)
                held.discard(key)
            pending[key] = root.after(RELEASE_DELAY, really)

    def stop(_=None):
        held.clear()

    def close():
        held.clear()
        for _ in range(5):
            pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
        root.destroy()

    root.bind('<KeyPress>', press)
    root.bind('<KeyRelease>', release)
    root.bind('<FocusOut>', stop)
    root.bind('<space>', stop)
    root.protocol('WM_DELETE_WINDOW', close)
    root.after(50, publish)
    # Started from another program the window can open behind the others: bring it to the front once.
    root.lift()
    root.attributes('-topmost', True)
    root.after(800, lambda: root.attributes('-topmost', False))
    root.focus_force()
    try:
        root.mainloop()
    except KeyboardInterrupt:
        close()


if __name__ == '__main__':
    main()
