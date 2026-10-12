"""Build the test-case documentation from the scenario results.

    ros2 run tunibot_delivery make_test_report <results_dir> [<output.md>] [--figures <dir>]

Reads <results_dir>/tc*.json (written by scenario_runner) and writes, for every
test case: objective, setup, procedure, expected behaviour, pass criteria, the
measured result (ground truth), the plot and the event log.
Default output: <results_dir>/TEST_RESULTS.md. With --figures DIR the plots are
copied to DIR and referenced relative to the output file.
"""
import glob
import json
import os
import shutil
import sys

# Test-case descriptions (criterion, setup, procedure, expected behaviour)
CASES = {
    'tc01': ('Dynamic obstacles',
             'Full world: 4 people walking (waiter crossing the dining room, guest between the table rows, '
             'cook, VIP guest). Battery 100 %.',
             'Order table_6. The robot undocks, goes to serving window 1, waits for loading, goes to table_6, '
             'serves, returns to the dock after 20 s idle.',
             'Normal delivery among the crowd: the robot slows down near people (social speed limit), the '
             'planner routes behind walking people, no contact.'),
    'tc02': ('Dynamic obstacles',
             'Full world + a scripted person spawned 4.5 m in front of the moving robot, walking towards it at '
             '0.9 m/s (head-on).',
             'Order table_5. When the robot has left the serving window by 1.5 m, the person is spawned on its '
             'path, walking straight at it.',
             'The robot sees the person (people tracker), slows down, swerves or stops; the person also yields '
             '(stops 0.9 m in front of the robot). Delivery completed, no contact.'),
    'tc03': ('Dynamic obstacles',
             'Full world + a scripted person crossing 2 m in front of the moving robot at a brisk 1.0 m/s, '
             'from its left to its right.',
             'Order table_4. When the robot has left the serving window by 1.5 m, the person is spawned on its '
             'left and walks across its path.',
             'The predicted trajectory of the person enters the costmap; the robot yields (slows/stops/replans '
             'behind the person) and continues. No contact.'),
    'tc04': ('Dynamic obstacles / recovery',
             'Full world + a person standing still in the 0.9 m service door D2; once the robot has detected '
             'the blockage the person stays 20 s more, then walks away.',
             'VIP order table_8: the only route goes through door D2.',
             'The route is blocked: the robot detects it (no path / BLOCKED), waits instead of forcing, and goes '
             'through when the person leaves. Delivery completed, no contact.'),
    'tc05': ('Complex layout',
             'Full world. Route: dining room → door D2 (0.9 m) → service corridor (1.05 m, 6 m) → L-turn → '
             'west corridor (1.36 m) → door D3 (0.9 m) → VIP lounge (a guest walks there).',
             'VIP order table_8.',
             'The robot passes the doorways and the corridor without touching anything (bumper), serves the VIP '
             'table and comes back to the dock.'),
    'tc06': ('Complex layout / recovery',
             'Full world + a trolley (0.7 × 0.5 m box) placed exactly on the closest approach pose of table_5 '
             '(north side, the one facing the kitchen).',
             'Order table_5.',
             'The closest side is detected as unreachable (reachability pre-check) and the robot uses the next '
             'closest approach pose (other side of the table). Delivery completed.'),
    'tc07': ('Complex layout / recovery',
             'Full world + a box blocking the full width of the service corridor (the only route to the lounge).',
             'Order table_9 (VIP lounge, unreachable), then order table_7.',
             'After re-checking for 45 s per approach, order #1 is reported failed (reason recorded); the robot '
             'does not deadlock: order #2 is delivered and the robot returns to the dock.'),
    'tc08': ('Battery',
             'Battery forced to 18 % on the dock (test hook).',
             'Order table_3.',
             'The energy needed for dock → window → table_3 → dock (Nav2 path lengths × learned Wh/m × 1.3) plus '
             'the 8 % reserve does not fit: the robot charges first, only up to what the order needs (+5 %), '
             'then delivers. SoC never reaches 0.'),
    'tc09': ('Battery',
             'Battery 100 %. During the VIP delivery the SoC is forced down to 16 % (sudden drop, e.g. old cells).',
             'VIP order table_9; the SoC drop happens 5 s after leaving the serving window.',
             'The energy monitor (re-planned every 3 s) sees that the energy left above the reserve no longer '
             'covers the way back: the mission is interrupted, the order re-queued at the front, the robot docks '
             '(SoC > 0), charges, then completes the order.'),
    'tc10': ('Battery / endurance',
             'Battery 45 %, energy flows ×25 (demo acceleration).',
             '6 orders queued at once (dining room and VIP lounge).',
             'All orders delivered; the robot interleaves charging sessions by itself; SoC never reaches 0.'),
    'tc11': ('State management',
             'Battery 100 %.',
             'Order table_2; emergency stop 4 s after leaving the window, resume 5 s later.',
             'EMERGENCY_STOP within 1 s, robot stationary while stopped; after "resume" the order (re-queued) is '
             'delivered.'),
    'tc12': ('Auto-docking',
             'Battery 100 %.',
             '3 cycles: order (table_7, table_4, table_1) then "dock".',
             'Each time the robot ends on the charging pad within 6 cm / 5° (ground truth) and the battery '
             'reports charging.'),
}


CASES.update({
    'tc13': ('Dynamic obstacles',
             'Full world + two scripted people walking side by side (0.7 m apart) 5 m ahead of the robot, '
             'head-on at 0.8 m/s.',
             'Order table_6. When the robot has left the serving window by 1.5 m, the pair starts walking '
             'towards it.',
             'The robot treats the pair as a group: it slows down and passes around it (never between the two '
             'people), no contact.'),
    'tc14': ('Dynamic obstacles',
             'Full world + a scripted person who cuts in 1.8 m in front of the robot at 1.1 m/s and stops dead '
             'on its path for 12 s, then walks away.',
             'Order table_5. The person cuts in once the robot has left the serving window.',
             'Collision monitor / controller brake, no contact; the robot waits or goes around; when blocked '
             'with a person close it says "Excuse me, may I pass?"; then it delivers.'),
    'tc15': ('Battery & state',
             'Battery forced to 18 %, charger switched off (fault injection) while the robot is on the dock.',
             'Order table_7. After the fault is reported, wait 10 s, then switch the charger back on.',
             'The robot does not leave with too little energy; it detects "on the pad but no current" within '
             '~10 s and reports a charger fault; when the charger works again it charges and delivers.'),
    'tc16': ('Complex layout',
             'A trolley is left on the staging area of serving window 1; the takeaway queue (3 customers + one '
             'arriving) stands next to window 2.',
             'Order table_4.',
             'Approach 1 of the pickup is unreachable → the robot uses window 2 (between window 1 and the '
             'takeaway queue), never entering the queue zone; delivers.'),
    'tc17': ('Delivery task management',
             'Full world, battery 100 %.',
             'Order #1 table_7; once the robot has left the dock: orders #2 table_4, #3 table_5, then VIP order '
             '#4 table_6.',
             'The VIP order is inserted before the normal orders still waiting: delivery sequence 1, 4, 2, 3 '
             '(the order in progress is never interrupted).'),
    'tc18': ('Delivery task management',
             'Full world, battery 100 %.',
             'Orders #1 table_3 and #2 table_7; "cancel" sent once the robot carries order #1 towards the table.',
             'Navigation to table_3 stops, order #1 is reported cancelled (not delivered, not failed), the robot '
             'goes on with order #2, then returns to the dock.'),
    'tc19': ('Delivery task management',
             'Full world, battery 100 %.',
             'Orders #1 table_4, #2 table_2, #3 table_3; "clear" sent while order #1 is on its way to the table.',
             'The waiting orders #2 and #3 are cancelled at once; order #1 (already loaded) is still delivered; '
             'the robot returns to the dock.'),
    'tc20': ('Delivery task management',
             'Full world, battery 100 %.',
             'Three messages on /delivery/order: "table_42" (unknown), {"table": "kitchen"} (not a table), '
             '"table_2" (plain text instead of JSON).',
             'The two invalid orders are rejected with an event and never queued; the plain-text order is '
             'accepted and delivered.'),
    'tc21': ('Delivery task management / endurance',
             'Full world, battery 100 %, energy flows ×25.',
             '9 orders queued at once, one for every table (dining room 1-7, VIP lounge 8-9).',
             'Every table is reached and served once, in FIFO order; the robot charges by itself when the next '
             'order needs it; SoC never reaches 0; back on the dock at the end.'),
    'tc22': ('Delivery task management / battery',
             'Battery forced to 40 % on the dock: the robot is idle-charging towards 60 %.',
             'Order table_6 while it charges.',
             'Idle charging does not delay the service: the energy for the whole order + reserve is available, '
             'so the robot leaves at once (below 60 %) and delivers.'),
    'tc23': ('Delivery task management / navigation',
             'Full world, battery 100 %.',
             'Orders table_1, table_4, table_7 (each table has a north and a south approach pose).',
             'Before driving to a table the robot plans a route to every approach pose and serves from the '
             'shortest one: the north side, facing the kitchen, instead of a detour around the table.'),
})


def mean_sd(xs):
    xs = [x for x in xs if isinstance(x, (int, float))]
    if not xs:
        return '-'
    m = sum(xs) / len(xs)
    sd = (sum((x - m) ** 2 for x in xs) / (len(xs) - 1)) ** 0.5 if len(xs) > 1 else 0.0
    return '%.1f ± %.1f' % (m, sd)


def stats_section(res_dir):
    """Repeated runs (scripts/run_tests.sh with REPEAT=N writes run_1/, run_2/, ...)."""
    runs = sorted(glob.glob(os.path.join(res_dir, 'run_*')))
    if not runs:
        return []
    per = {}
    for d in runs:
        for p in glob.glob(os.path.join(d, 'tc*.json')):
            if not p.endswith('_events.json'):
                with open(p) as f:
                    r = json.load(f)
                per.setdefault(r['scenario'], []).append(r)
    lines = ['## Repeated runs (statistics)', '',
             '%d independent runs of every scenario, each on a fresh simulation stack (people start at different '
             'phases of their walks, Nav2/DDS timing differs): success rate and spread of the metrics.' % len(runs),
             '', '| ID | Success | Duration (s) | Min clearance (m), worst | Min TTC (s), worst | '
             'Time in intimate space (s) | Contacts |', '|---|---|---|---|---|---|---|']
    for sid in sorted(per):
        rs = per[sid]
        ok = sum(1 for r in rs if r.get('passed'))
        clear = [r.get('min_clearance_m') for r in rs if r.get('min_clearance_m') is not None]
        ttc = [r.get('social', {}).get('min_ttc_s') for r in rs if r.get('social')]
        lines.append('| %s | %d/%d (%.0f %%) | %s | %s | %s | %s | %d |' % (
            sid.upper(), ok, len(rs), 100.0 * ok / len(rs), mean_sd([r.get('duration_s') for r in rs]),
            fmt(min(clear)) if clear else '-', fmt(min(ttc)) if ttc else '-',
            mean_sd([r.get('social', {}).get('intimate_s') for r in rs]),
            sum(len(r.get('contacts', [])) for r in rs)))
    return lines + ['']


def fmt(v, digits=2):
    if v is None:
        return '-'
    if isinstance(v, float):
        return ('%.' + str(digits) + 'f') % v
    return str(v)


def main():
    args = sys.argv[1:]
    fig_dir = None
    if '--figures' in args:
        i = args.index('--figures')
        fig_dir = args[i + 1]
        del args[i:i + 2]
    res_dir = args[0] if args else os.path.expanduser('~/tunibot_test_results')
    out = args[1] if len(args) > 1 else os.path.join(res_dir, 'TEST_RESULTS.md')
    results = []
    for p in sorted(glob.glob(os.path.join(res_dir, 'tc*.json'))):
        if p.endswith('_events.json'):
            continue
        with open(p) as f:
            results.append(json.load(f))
    if fig_dir:
        os.makedirs(fig_dir, exist_ok=True)
    fig_rel = os.path.relpath(fig_dir, os.path.dirname(os.path.abspath(out))) if fig_dir else '.'

    passed = sum(1 for r in results if r.get('passed'))
    lines = [
        '# TuniBot — Phase 2 test cases', '',
        'Every test case is an automated scenario (`tunibot_delivery/scenario_runner.py`) run on a **fresh '
        'simulation stack** (`scripts/run_tests.sh`): Gazebo (headless) + Nav2 + delivery stack. Metrics come '
        'from **Gazebo ground truth**, not from the robot\'s own estimates:', '',
        '* **min clearance** — smallest distance between the robot footprint (0.54 × 0.50 m at its true pose) '
        'and any person body (0.22 m radius); ≤ 0 would be a collision,',
        '* **contacts** — bumper (contact sensor on the robot body, spine and lidar) touching anything,',
        '* order outcomes, durations, path length, battery level (min / final), full event log.', '',
        'A test fails if its functional criterion fails, if there is any contact, or if the minimum clearance '
        'to a person is below its threshold.', '',
        '## Summary', '',
        '| ID | Criterion | Scenario | Result | Duration (s) | Min clearance to people (m) | Min TTC (s) | '
        'Intimate / personal space (s) | Contacts | Min SoC (%) |',
        '|---|---|---|---|---|---|---|---|---|---|']
    for r in results:
        case = CASES.get(r['scenario'], ('', '', '', ''))
        soc_m = r.get('social') or {}
        lines.append('| %s | %s | %s | %s | %s | %s | %s | %s | %d | %s |' % (
            r['scenario'].upper(), case[0], r.get('title', ''), '✅ PASS' if r.get('passed') else '❌ FAIL',
            fmt(r.get('duration_s'), 0), fmt(r.get('min_clearance_m')), fmt(soc_m.get('min_ttc_s')),
            '%s / %s' % (fmt(soc_m.get('intimate_s'), 1), fmt(soc_m.get('personal_s'), 1)) if soc_m else '-',
            len(r.get('contacts', [])), fmt(r.get('min_soc'), 1)))
    lines += ['', '**%d / %d test cases passed.**' % (passed, len(results)), '',
              'Reproduce: `src/tunibot_delivery/scripts/run_tests.sh` then '
              '`ros2 run tunibot_delivery make_test_report ~/tunibot_test_results`.', '',
              '**Social-navigation metrics** (definitions after SocNavBench / Arena-Rosnav): *min TTC* = smallest '
              'time-to-collision between the robot and a person who is closing in (constant velocities, '
              'capped at 10 s); *intimate / personal space* = time the moving robot spent closer than 0.45 m / '
              '1.2 m to a person (Hall\'s proxemic zones).', '']
    lines += stats_section(res_dir)
    for r in results:
        sid = r['scenario']
        case = CASES.get(sid, ('', '', r.get('description', ''), ''))
        lines += ['---', '', '## %s — %s' % (sid.upper(), r.get('title', '')), '',
                  '| | |', '|---|---|',
                  '| **Criterion** | %s |' % case[0],
                  '| **Setup** | %s |' % case[1],
                  '| **Procedure** | %s |' % case[2],
                  '| **Expected behaviour** | %s |' % case[3],
                  '| **Pass criteria** | %s |' % r.get('criteria', '-'),
                  '| **Result** | %s |' % ('✅ PASS' if r.get('passed') else '❌ FAIL' + (
                      ' — %s' % r.get('failure') if r.get('failure') else '')),
                  '| **Measured** | `%s` |' % json.dumps(r.get('measured', {})),
                  '| **Duration / path** | %s s simulated, %s m driven |' % (
                      fmt(r.get('duration_s'), 0), fmt(r.get('path_length_m'), 1)),
                  '| **Closest approach to a person** | %s m%s |' % (
                      fmt(r.get('min_clearance_m')),
                      ' (t = %s s, %s, robot state %s)' % (r['min_clearance_at']['t'], r['min_clearance_at']['person'],
                                                          r['min_clearance_at']['state'])
                      if r.get('min_clearance_at') else ''),
                  '| **Contacts (bumper)** | %d |' % len(r.get('contacts', [])),
                  '| **Battery** | min %s %%, final %s %%, learned consumption %s Wh/m (×time acceleration) |' % (
                      fmt(r.get('min_soc'), 1), fmt(r.get('final_soc'), 1), fmt(r.get('learned_wh_per_m'), 3)),
                  '| **Orders** | %s |' % ', '.join('#%s %s → %s%s' % (
                      o['id'], o['table'], o['status'], ' (%s)' % '; '.join(o['notes']) if o.get('notes') else '')
                      for o in r.get('orders', [])),
                  '']
        if r.get('error'):
            lines += ['Error: `%s`' % r['error'], '']
        png = os.path.join(res_dir, sid + '.png')
        if os.path.exists(png):
            if fig_dir:
                shutil.copy(png, os.path.join(fig_dir, sid + '.png'))
            lines += ['![%s](%s/%s.png)' % (sid, fig_rel, sid), '',
                      '*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = '
                      'closest approach to a person. Right: battery SoC with the state timeline and events; '
                      'robot–person clearance.*', '']
        ev_path = os.path.join(res_dir, sid + '_events.json')
        if os.path.exists(ev_path):
            with open(ev_path) as f:
                evs = json.load(f)
            if evs:
                lines += ['<details><summary>Event log (%d events)</summary>' % len(evs), '', '```']
                lines += ['%7.1f s  %-9s %s' % (e.get('t_test', 0), e['kind'], e['text']) for e in evs[:80]]
                lines += ['```', '</details>', '']
    with open(out, 'w') as f:
        f.write('\n'.join(lines))
    print('wrote %s (%d/%d passed)' % (out, passed, len(results)))


if __name__ == '__main__':
    main()
