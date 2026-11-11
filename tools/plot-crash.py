#!/usr/bin/env python3
"""Plot a crash-<time>.json written by the Open v5 crash detector (detect_crash: true).

    python3 tools/plot-crash.py crash-181500.json [-o crash.png]

Left: speed, steering, gyro, wall distances and valid LiDAR rays over the last seconds (0 = the crash stop).
Right: the last scans before the stop, top view in the car's frame (x forward, y left).
Also prints (and puts on the figure) a cause checklist: LiDAR scans lost, stale sensors, control loop lag, car not
moving, car not turning while the servo was ordered to, and what the strategy was doing. --text: only the checklist.
"""
import argparse
import json
import math



def column(rows, key):
    return [math.nan if r.get(key) is None else r[key] for r in rows]


def _runs(times, flags):
    """[(start, end)] of consecutive True flags."""
    runs, start = [], None
    for x, flag in zip(times, flags):
        if flag and start is None:
            start = x
        if not flag and start is not None:
            runs.append((start, x))
            start = None
    if start is not None:
        runs.append((start, times[-1]))
    return runs


def diagnose(data):
    """Lines of a checklist of the usual causes, times in s from the crash stop (negative = before)."""
    t0, ticks, scans = data['crash_time'], data['ticks'], data['scans']
    params = data.get('params', {})
    min_rays = params.get('crash_min_rays', 120)
    out = [f"motivo de la parada: {data['reason']}"]
    st = [s['t'] - t0 for s in scans]
    # Impact ~ the first scan of the last blind run.
    onset = 0.0
    for x, s in zip(reversed(st), reversed(scans)):
        if s['valid'] >= params.get('crash_clear_rays', 180):
            break
        if s['valid'] < min_rays:
            onset = x
    out.append(f'impacto aprox. (empieza el LiDAR ciego): {onset:+.2f} s')
    # 1. LiDAR scans lost (no message): the LD19 sends one every ~0.1 s.
    gaps = [(a, b) for a, b in zip(st, st[1:]) if b - a > 0.2]
    before = [g for g in gaps if g[1] <= onset + 0.05]
    if before:
        worst = max(before, key=lambda g: g[1] - g[0])
        last = before[-1]
        out.append(f'LiDAR: {len(before)} huecos sin scan (> 0,2 s) antes del impacto; el mayor {worst[1] - worst[0]:.2f} s '
                   f'en {worst[0]:+.2f}; el último {last[1] - last[0]:.2f} s en {last[0]:+.2f}'
                   + ('  <-- en el último segundo' if last[1] > onset - 1.0 else ''))
    else:
        out.append('LiDAR: sin huecos de scans antes del impacto (todos llegaron)')
    weak = [x for x, s in zip(st, scans) if s['valid'] < min_rays and x < onset - 0.05]
    if weak:
        out.append(f'LiDAR: {len(weak)} scans con < {min_rays} rayos antes del impacto (primero en {weak[0]:+.2f})')
    tt = [r['t'] - t0 for r in ticks]
    # 2. Stale sensors as the node saw them (it brakes at 0.35 s of LiDAR, 0.2 s of IMU).
    for key, limit, name in (('scan_age', params.get('scan_max_age', 0.35), 'LiDAR'),
                             ('imu_age', params.get('imu_max_age', 0.2), 'IMU (placa)')):
        ages = [(x, r.get(key)) for x, r in zip(tt, ticks) if r.get(key) is not None and x <= onset]
        if ages:
            worst = max(ages, key=lambda a: a[1])
            runs = _runs([a[0] for a in ages], [a[1] > limit for a in ages])
            out.append(f'{name}: edad máx. {worst[1]:.2f} s en {worst[0]:+.2f}; viejo (> {limit} s, frena) '
                       + (', '.join(f'{a:+.2f}..{b:+.2f}' for a, b in runs) if runs else 'nunca'))
    # 3. Control loop lag (50 Hz expected).
    lags = [(a, b - a) for a, b in zip(tt, tt[1:]) if b - a > 0.08 and b <= onset]
    out.append('lazo de control: ' + (f'{len(lags)} ticks tardíos, máx. {max(l for _, l in lags):.2f} s en '
                                       f'{max(lags, key=lambda g: g[1])[0]:+.2f}' if lags else 'a tiempo'))
    # 4. Car not moving while asked to: the walls do not move (motor, board or stuck).
    still = []
    for i, (x, r) in enumerate(zip(tt, ticks)):
        if x > onset or (r.get('v_mission') or 0) < 0.3:
            continue
        window = [q for y, q in zip(tt, ticks) if x - 0.5 <= y <= x]
        if x - tt[0] < 0.5 or len(window) < 10:
            continue
        spans = []
        for key in ('left', 'right', 'front_axis'):
            vals = [q[key] for q in window if q.get(key) is not None]
            spans.append(max(vals) - min(vals) if len(vals) > 5 else math.inf)
        if max(spans) < 0.03:
            still.append(x)
    out.append('movimiento: ' + (f'paredes quietas 0,5 s con velocidad pedida desde {still[0]:+.2f} s '
                                 '(motor/placa parados o atascado)' if still else 'se movía hasta el impacto'))
    # 5. Steering ordered but the car did not turn (servo frozen?): gyro vs steering sign, 0.3 s windows.
    bad = []
    for x, r in zip(tt, ticks):
        steer = r.get('steer') or 0.0
        if x > onset - 0.1 or abs(steer) < 0.2 or (r.get('v_mission') or 0) < 0.3:
            continue
        window = [q for y, q in zip(tt, ticks) if x - 0.3 <= y <= x]
        if all(abs(q.get('steer') or 0) >= 0.2 and (q['steer'] > 0) == (steer > 0) for q in window):
            mean = sum(q['gyro'] for q in window) / len(window)
            if mean * math.copysign(1, steer) < 0.15:
                bad.append(x)
    out.append('dirección: ' + (f'giro pedido sin giro del coche desde {bad[0]:+.2f} s (¿servo congelado o '
                                f'patinando?)' if bad else 'el coche giró hacia donde pedía el servo'))
    # 6. What the strategy was doing.
    states = [(x, r['state'], r.get('corners')) for x, r in zip(tt, ticks)]
    at = next((s for s in reversed(states) if s[0] <= onset), states[-1] if states else None)
    if at:
        turn = next((x for x, s, _ in reversed(states) if s == 'TURN' and x <= onset), None)
        line = f'estrategia en el impacto: {at[1]} (giros {at[2]})'
        if turn is not None:
            start = turn
            for x, s, _ in reversed(states):
                if x <= turn and s == 'TURN':
                    start = x
                elif x < start:
                    break
            row = next(r for x, r in zip(tt, ticks) if x >= start)
            line += f'; último TURN empezó en {start:+.2f} s con front_axis ' + (f'{row["front_axis"]:.2f} m' if row.get('front_axis') is not None else '?')
        out.append(line)
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('file')
    parser.add_argument('-o', '--output', help='PNG instead of a window')
    parser.add_argument('--text', action='store_true', help='only the cause checklist, no plot')
    args = parser.parse_args()
    data = json.load(open(args.file))
    report = diagnose(data)
    print('\n'.join(report))
    if args.text:
        return
    import matplotlib.pyplot as plt
    t0, ticks, scans = data['crash_time'], data['ticks'], data['scans']
    t = [r['t'] - t0 for r in ticks]
    fig = plt.figure(figsize=(15, 11))
    grid = fig.add_gridspec(5, 2, width_ratios=(2, 1), height_ratios=(1, 1, 1, 1, 0.9))
    axes = [fig.add_subplot(grid[i, 0]) for i in range(4)]
    for ax in axes[1:]:
        ax.sharex(axes[0])
    axes[0].plot(t, column(ticks, 'v_mission'), label='v pedida (misión)')
    axes[0].plot(t, column(ticks, 'v_cmd'), label='v enviada')
    axes[0].set_ylabel('m/s')
    axes[1].plot(t, [math.degrees(v) for v in column(ticks, 'steer')], label='dirección (°)')
    axes[1].plot(t, [math.degrees(v) for v in column(ticks, 'gyro')], label='giroscopio (°/s)')
    axes[1].set_ylabel('° / °/s')
    for key in ('left', 'right', 'front_axis', 'island_end'):
        axes[2].plot(t, column(ticks, key), label=key)
    axes[2].set_ylabel('m')
    axes[2].set_ylim(0, 3.2)
    axes[3].plot([s['t'] - t0 for s in scans], [s['valid'] for s in scans], '.-', label='rayos válidos')
    axes[3].axhline(data['params'].get('crash_min_rays', 120), color='r', ls=':', label='umbral')
    ages = column(ticks, 'scan_age')
    ax_age = axes[3].twinx()
    ax_age.plot(t, ages, color='gray', alpha=0.6, label='edad del scan (s)')
    ax_age.set_ylabel('s')
    axes[3].set_ylabel('rayos')
    axes[3].set_xlabel('s (0 = parada por choque)')
    # State changes as shaded text along the top plot.
    previous = None
    for row, x in zip(ticks, t):
        if row['state'] != previous:
            for ax in axes:
                ax.axvline(x, color='k', alpha=0.15)
            axes[0].text(x, axes[0].get_ylim()[1], f" {row['state']} {row['corners']}", fontsize=7, va='top')
            previous = row['state']
    for ax in axes:
        ax.axvline(0, color='r')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, loc='upper left')
    # Last scans, newest darkest.
    # Scans that never arrived: red bands on the rays plot.
    st = [x['t'] - t0 for x in scans]
    for a, b in zip(st, st[1:]):
        if b - a > 0.2:
            axes[3].axvspan(a, b, color='r', alpha=0.15)
    text = fig.add_subplot(grid[4, :])
    text.axis('off')
    text.text(0, 1, '\n'.join(report), va='top', family='monospace', fontsize=8)
    ax = fig.add_subplot(grid[:4, 1])
    last = scans[-6:]
    for i, s in enumerate(last):
        xs, ys = [], []
        for k, r in enumerate(s['ranges']):
            if r is not None and s['range_min'] < r < s['range_max']:
                a = s['angle_min'] + k * s['angle_increment']
                xs.append(r * math.cos(a))
                ys.append(r * math.sin(a))
        ax.scatter(xs, ys, s=3, alpha=min(1.0, 0.2 + 0.8 * (i + 1) / len(last)),
                   label=f"{s['t'] - t0:+.2f} s, {s['valid']} rayos")
    ax.add_patch(plt.Rectangle((-0.17, -0.09), 0.26, 0.18, fill=False, color='r'))
    ax.set_aspect('equal')
    ax.set_xlim(-1.5, 3.0)
    ax.set_ylim(-1.5, 1.5)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7)
    ax.set_title('últimos scans (x adelante, y izquierda)')
    fig.suptitle(f"{args.file}: {data['reason']} | sentido {data.get('direction')}")
    fig.tight_layout()
    if args.output:
        fig.savefig(args.output, dpi=110)
    else:
        plt.show()


if __name__ == '__main__':
    main()
