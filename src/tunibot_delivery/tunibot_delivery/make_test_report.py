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
             'Full world + a person standing still in the 0.9 m service door D2; once the robot is 3.5 m away '
             'the person stays 20 s more, then walks away.',
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
             'Full world + a trolley (0.7 × 0.5 m box) placed exactly on the first approach pose of table_5.',
             'Order table_5.',
             'The first approach is detected as unreachable (reachability pre-check) and the robot uses the second '
             'approach pose (other side of the table). Delivery completed.'),
    'tc07': ('Complex layout / recovery',
             'Full world + a box blocking the full width of the service corridor (the only route to the lounge).',
             'Order table_9 (VIP lounge, unreachable), then order table_7.',
             'After re-checking for 30 s per approach, order #1 is reported failed (reason recorded); the robot '
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
        '| ID | Criterion | Scenario | Result | Duration (s) | Min clearance to people (m) | Contacts | Min SoC (%) |',
        '|---|---|---|---|---|---|---|---|']
    for r in results:
        case = CASES.get(r['scenario'], ('', '', '', ''))
        lines.append('| %s | %s | %s | %s | %s | %s | %d | %s |' % (
            r['scenario'].upper(), case[0], r.get('title', ''), '✅ PASS' if r.get('passed') else '❌ FAIL',
            fmt(r.get('duration_s'), 0), fmt(r.get('min_clearance_m')), len(r.get('contacts', [])),
            fmt(r.get('min_soc'), 1)))
    lines += ['', '**%d / %d test cases passed.**' % (passed, len(results)), '',
              'Reproduce: `src/tunibot_delivery/scripts/run_tests.sh` then '
              '`ros2 run tunibot_delivery make_test_report ~/tunibot_test_results`.', '']
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
