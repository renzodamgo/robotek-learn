// Scripts that run on the Pi. They are sent over ssh each time (nothing is installed there).
// Keep them read-only and cheap: the fast probe runs every ~2 s.

enum Scripts {
    /// Fast probe: /proc, /sys, vcgencmd, Wi-Fi and the ROS processes. One `sh -s` per tick.
    /// Output: one "KEY value..." line per item; the app parses it in Probe.swift.
    static let fast = #"""
    echo "HOST $(hostname)"
    echo "BOOT $(cat /proc/sys/kernel/random/boot_id)"
    echo "UP $(cut -d' ' -f1 /proc/uptime)"
    echo "LOAD $(cat /proc/loadavg)"
    grep '^cpu' /proc/stat | sed 's/^/STAT /'
    echo "TEMP $(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null)"
    for f in /sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_cur_freq; do
      [ -r "$f" ] && echo "FREQ $(cat "$f")"
    done
    awk '/^(MemTotal|MemAvailable|SwapTotal|SwapFree):/{print "MEM",$1,$2}' /proc/meminfo
    df -Pk / | awk 'NR==2{print "DISK",$2,$3,$4}'
    if command -v vcgencmd >/dev/null 2>&1; then
      echo "THR $(vcgencmd get_throttled 2>&1)"
      echo "VTEMP $(vcgencmd measure_temp 2>&1)"
      for c in arm core v3d; do echo "CLK $c $(vcgencmd measure_clock $c 2>&1)"; done
      echo "VOLT $(vcgencmd measure_volts core 2>&1)"
      echo "EXT5V $(vcgencmd pmic_read_adc EXT5V_V 2>&1)"
    fi
    awk 'NR>2{sub(":","",$1); print "WIRELESS",$1,$3,$4}' /proc/net/wireless 2>/dev/null
    W=$(awk 'NR>2{sub(":","",$1); print $1; exit}' /proc/net/wireless 2>/dev/null)
    if [ -n "$W" ] && command -v iw >/dev/null 2>&1; then
      iw dev "$W" link 2>/dev/null | sed 's/^/IW /'
    fi
    echo "IP $(hostname -I 2>/dev/null)"
    ps -eo etimes=,args= | grep -E '[_]_node:=|[r]os2 launch|[r]os-args' | cut -c1-300 | sed 's/^/PROC /'
    echo "END"
    """#

    /// ROS probe (every ~30 s, only while ROS processes are up): one rclpy process that lists the
    /// nodes, reads /rrc/battery (UInt16, mV, ~0.5-1 Hz) and measures the /scan and /robotek/imu
    /// rates for a few seconds. Prints one JSON line.
    static let ros = #"""
    import json, time
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message

    WATCH = ['/rrc/battery', '/scan', '/robotek/imu']
    rclpy.init()
    node = Node('pi_monitor_probe')
    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    spin(1.2)                                   # discovery
    types = dict(node.get_topic_names_and_types())
    stats = {t: {'n': 0, 'first': None, 'last': None} for t in WATCH}
    battery = []
    subs = []
    for topic in WATCH:
        if topic not in types:
            continue
        def callback(msg, topic=topic):
            s = stats[topic]
            now = time.monotonic()
            s['n'] += 1
            s['first'] = s['first'] or now
            s['last'] = now
            if topic == '/rrc/battery':
                battery.append(int(msg.data))
        subs.append(node.create_subscription(get_message(types[topic][0]), topic, callback,
                                             qos_profile_sensor_data))
    spin(3.5)
    rates = {}
    for topic, s in stats.items():
        if topic not in types:
            rates[topic] = None
        elif s['n'] >= 2 and s['last'] > s['first']:
            rates[topic] = round((s['n'] - 1) / (s['last'] - s['first']), 1)
        else:
            rates[topic] = 0.0
    nodes = sorted((ns.rstrip('/') + '/' + name) for name, ns in node.get_node_names_and_namespaces()
                   if name != 'pi_monitor_probe')
    print('ROSJSON ' + json.dumps({'nodes': nodes, 'rates': rates,
                                   'battery_mv': battery[-1] if battery else None,
                                   'topics': len(types)}), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    """#
}
