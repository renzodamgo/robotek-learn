#!/usr/bin/env python3
"""Plot a crash-<time>.json written by the Open v5 crash detector (detect_crash: true).

    python3 tools/plot-crash.py crash-181500.json [-o crash.png]

Left: speed, steering, gyro, wall distances and valid LiDAR rays over the last seconds (0 = the crash stop).
Right: the last scans before the stop, top view in the car's frame (x forward, y left).
"""
import argparse
import json
import math

import matplotlib.pyplot as plt


def column(rows, key):
    return [math.nan if r.get(key) is None else r[key] for r in rows]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('file')
    parser.add_argument('-o', '--output', help='PNG instead of a window')
    args = parser.parse_args()
    data = json.load(open(args.file))
    t0, ticks, scans = data['crash_time'], data['ticks'], data['scans']
    t = [r['t'] - t0 for r in ticks]
    fig = plt.figure(figsize=(15, 9))
    grid = fig.add_gridspec(4, 2, width_ratios=(2, 1))
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
    ax = fig.add_subplot(grid[:, 1])
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
