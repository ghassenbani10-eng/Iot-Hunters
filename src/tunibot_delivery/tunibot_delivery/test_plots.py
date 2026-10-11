"""Figures for the scenario results (used by scenario_runner and make_test_report)."""
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402

STATE_COLORS = {
    'DOCKED': '#2e7d32', 'CHARGING': '#66bb6a', 'UNDOCKING': '#8d6e63', 'TO_PICKUP': '#1e88e5',
    'LOADING': '#90caf9', 'TO_TABLE': '#6a1b9a', 'SERVING': '#ce93d8', 'IDLE': '#bdbdbd',
    'RETURNING_TO_DOCK': '#fb8c00', 'DOCKING': '#ffcc80', 'EMERGENCY_STOP': '#d32f2f', 'ERROR': '#b71c1c',
}


def load_map():
    """Static map image + extent (map frame) from the package share."""
    d = os.path.join(get_package_share_directory('tunibot_delivery'), 'maps')
    meta = {}
    for line in open(os.path.join(d, 'restaurant_map.yaml')):
        if ':' in line:
            k, v = line.split(':', 1)
            meta[k.strip()] = v.strip()
    raw = open(os.path.join(d, meta['image']), 'rb').read()
    parts, i = [], 0
    while len(parts) < 4:
        while raw[i:i + 1].isspace():
            i += 1
        if raw[i:i + 1] == b'#':
            while raw[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while not raw[j:j + 1].isspace():
            j += 1
        parts.append(raw[i:j])
        i = j
    w, h = int(parts[1]), int(parts[2])
    img = np.frombuffer(raw[i + 1:i + 1 + w * h], dtype=np.uint8).reshape(h, w)
    res = float(meta['resolution'])
    ox, oy = [float(v) for v in meta['origin'].strip('[]').split(',')[:2]]
    return img, [ox, ox + w * res, oy, oy + h * res]


def plot_scenario(out_dir, sid, result, samples, events):
    if not samples:
        return
    fig = plt.figure(figsize=(13, 6.5))
    ax = fig.add_axes([0.03, 0.08, 0.5, 0.85])
    try:
        img, ext = load_map()
        ax.imshow(img, cmap='gray', origin='upper', extent=ext, vmin=0, vmax=255)
    except (OSError, KeyError, ValueError):
        pass
    xs = np.array([s['x'] for s in samples])
    ys = np.array([s['y'] for s in samples])
    people = sorted({k for s in samples for k in s if k.startswith('p_')})
    for k in people:
        pts = [tuple(map(float, s[k].split(';'))) for s in samples if s.get(k)]
        if pts:
            p = np.array(pts)
            ax.plot(p[:, 0], p[:, 1], '-', color='#e57373', lw=1, alpha=0.7)
            ax.plot(p[-1, 0], p[-1, 1], 'o', color='#c62828', ms=5)
    states = [s['state'] for s in samples]
    for st, col in STATE_COLORS.items():
        m = np.array([s == st for s in states])
        if m.any():
            ax.scatter(xs[m], ys[m], s=3, color=col, label=st)
    mc = result.get('min_clearance_at')
    if mc:
        ax.plot(*mc['robot'], marker='*', color='red', ms=14, mec='k')
        ax.annotate('min clearance %.2f m' % result['min_clearance_m'], mc['robot'], xytext=(8, 8),
                    textcoords='offset points', fontsize=8, color='red')
    pad = 2.0
    ax.set_xlim(min(xs.min(), 0) - pad, max(xs.max(), 0) + pad)
    ax.set_ylim(ys.min() - pad, ys.max() + pad)
    ax.set_aspect('equal')
    ax.legend(loc='lower left', fontsize=6, markerscale=3, ncol=2)
    ax.set_title('%s - %s: %s' % (sid.upper(), result.get('title', ''), 'PASS' if result.get('passed') else 'FAIL'),
                 fontsize=9)

    t = np.array([s['t'] for s in samples])
    a1 = fig.add_axes([0.6, 0.56, 0.37, 0.36])
    soc = np.array([s['soc'] if isinstance(s['soc'], (int, float)) else np.nan for s in samples], dtype=float)
    a1.plot(t, soc, 'g-', lw=1.5)
    a1.set_ylabel('battery SoC (%)')
    a1.set_ylim(0, 105)
    a1.grid(alpha=0.3)
    for i in range(len(t) - 1):
        col = STATE_COLORS.get(states[i])
        if col and (i == 0 or states[i] != states[i - 1]):
            j = i
            while j < len(t) - 1 and states[j] == states[i]:
                j += 1
            a1.axvspan(t[i], t[j], color=col, alpha=0.15, lw=0)
    for e in events:
        if e['kind'] in ('battery', 'failure', 'recovery', 'estop'):
            a1.axvline(e.get('t_test', 0), color='r', lw=0.6, alpha=0.5)
    a2 = fig.add_axes([0.6, 0.1, 0.37, 0.36], sharex=a1)
    clr = np.array([s['clearance'] if s['clearance'] is not None else np.nan for s in samples], dtype=float)
    a2.plot(t, np.clip(clr, None, 4.0), 'b-', lw=1)
    a2.axhline(0.0, color='r', lw=1)
    a2.set_ylabel('robot-person clearance (m)')
    a2.set_xlabel('time (s, simulation)')
    a2.set_ylim(-0.1, 4.0)
    a2.grid(alpha=0.3)
    fig.savefig(os.path.join(out_dir, sid + '.png'), dpi=90)
    plt.close(fig)
