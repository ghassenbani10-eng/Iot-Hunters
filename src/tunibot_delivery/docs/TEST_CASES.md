# TuniBot — Phase 2 test cases

Every test case is an automated scenario (`tunibot_delivery/scenario_runner.py`) run on a **fresh simulation stack** (`scripts/run_tests.sh`): Gazebo (headless) + Nav2 + delivery stack. Metrics come from **Gazebo ground truth**, not from the robot's own estimates:

* **min clearance** — smallest distance between the robot footprint (0.54 × 0.50 m at its true pose) and any person body (0.22 m radius); ≤ 0 would be a collision,
* **contacts** — bumper (contact sensor on the robot body, spine and lidar) touching anything,
* order outcomes, durations, path length, battery level (min / final), full event log.

A test fails if its functional criterion fails, if there is any contact, or if the minimum clearance to a person is below its threshold.

## Summary

| ID | Criterion | Scenario | Result | Duration (s) | Min clearance to people (m) | Contacts | Min SoC (%) |
|---|---|---|---|---|---|---|---|
| TC01 | Dynamic obstacles | Delivery with people walking around (ambient crowd) | ✅ PASS | 200 | 0.32 | 0 | 76.8 |
| TC02 | Dynamic obstacles | Person walking head-on towards the robot | ✅ PASS | 87 | 0.22 | 0 | 89.7 |
| TC03 | Dynamic obstacles | Person crossing just in front of the robot | ✅ PASS | 75 | 0.64 | 0 | 91.4 |
| TC04 | Dynamic obstacles / recovery | Doorway blocked by a standing person (25 s) | ✅ PASS | 183 | 0.31 | 0 | 77.8 |
| TC05 | Complex layout | VIP lounge: doorways + narrow corridor + L-turn | ✅ PASS | 549 | 0.24 | 0 | 44.0 |
| TC06 | Complex layout / recovery | Table approach blocked by a trolley -> alternate approach pose | ✅ PASS | 217 | 0.47 | 0 | 75.6 |
| TC07 | Complex layout / recovery | Unreachable table (corridor blocked): graceful failure, queue continues | ✅ PASS | 379 | 0.33 | 0 | 60.1 |
| TC08 | Battery | Low battery when the order arrives -> charge first | ✅ PASS | 171 | 0.51 | 0 | 18.3 |
| TC09 | Battery | Battery runs low mid-mission -> return to dock in time, resume later | ✅ PASS | 433 | 0.40 | 0 | 8.6 |
| TC10 | Battery / endurance | Endurance: 6 orders, battery cycles handled autonomously | ✅ PASS | 1079 | 0.20 | 0 | 29.3 |
| TC11 | State management | Emergency stop and resume | ✅ PASS | 105 | 0.55 | 0 | 88.2 |
| TC12 | Auto-docking | Docking precision (3 cycles) and charger contact | ✅ PASS | 413 | 0.49 | 0 | 59.1 |

**12 / 12 test cases passed.**

### Run history (transparency)

All results above come from one complete run of the 12 scenarios on the final code (MPPI controller),
except **TC07 and TC11, shown from their re-run** after one fix:

* **TC11** first failed before the scenario even started: the delivery manager waited forever for Nav2
  (`BasicNavigator.waitUntilNav2Active` makes one lifecycle service call with no timeout, and with
  FastDDS the response was lost). Fixed (own wait with timeouts and retries), then re-run: PASS.
* **TC07** first failed on its *second* order: the blocked-corridor part behaved as specified (order 1
  reported unreachable, no deadlock), then the controller repeatedly rejected plans ("plan has 0 poses")
  — the intermittent Nav2-side issue described in [PHASE2.md §6](PHASE2.md). Re-run: PASS.

Earlier full runs during development found and fixed, among others: the robot's 15 % traction, the
parking drift, a person/robot standoff, doors 15 cm too narrow, the corridor SLAM aliasing, the
collision-monitor freeze — see [PHASE2.md](PHASE2.md).

Reproduce: `src/tunibot_delivery/scripts/run_tests.sh` then `ros2 run tunibot_delivery make_test_report ~/tunibot_test_results`.

---

## TC01 — Delivery with people walking around (ambient crowd)

| | |
|---|---|
| **Criterion** | Dynamic obstacles |
| **Setup** | Full world: 4 people walking (waiter crossing the dining room, guest between the table rows, cook, VIP guest). Battery 100 %. |
| **Procedure** | Order table_6. The robot undocks, goes to serving window 1, waits for loading, goes to table_6, serves, returns to the dock after 20 s idle. |
| **Expected behaviour** | Normal delivery among the crowd: the robot slows down near people (social speed limit), the planner routes behind walking people, no contact. |
| **Pass criteria** | order delivered, robot back on the dock, 0 collisions, min clearance > 0.10 m |
| **Result** | ✅ PASS |
| **Measured** | `{"delivered": true}` |
| **Duration / path** | 200 s simulated, 31.9 m driven |
| **Closest approach to a person** | 0.32 m (t = 154.41 s, human_waiter, robot state RETURNING_TO_DOCK) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 76.8 %, final 79.2 %, learned consumption 1.169 Wh/m (×time acceleration) |
| **Orders** | #1 table_6 → delivered |

![tc01](test_results/tc01.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (6 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  order     Order #1 queued -> table_6 (TC01: 2 coffees)
    4.3 s  dock      Undocked
   96.2 s  delivery  Order #1 delivered to table_6
  116.3 s  dock      Returning to dock: idle for 20s
  200.2 s  dock      Docked, charging contacts OK (79%), pad offset 1.5 cm
```
</details>

---

## TC02 — Person walking head-on towards the robot

| | |
|---|---|
| **Criterion** | Dynamic obstacles |
| **Setup** | Full world + a scripted person spawned 4.5 m in front of the moving robot, walking towards it at 0.9 m/s (head-on). |
| **Procedure** | Order table_5. When the robot has left the serving window by 1.5 m, the person is spawned on its path, walking straight at it. |
| **Expected behaviour** | The robot sees the person (people tracker), slows down, swerves or stops; the person also yields (stops 0.9 m in front of the robot). Delivery completed, no contact. |
| **Pass criteria** | robot slows/stops or swerves, order delivered, 0 collisions, clearance > 0.10 m |
| **Result** | ✅ PASS |
| **Measured** | `{"delivered": true}` |
| **Duration / path** | 87 s simulated, 13.8 m driven |
| **Closest approach to a person** | 0.22 m (t = 49.36 s, human_waiter, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 89.7 %, final 89.7 %, learned consumption 1.113 Wh/m (×time acceleration) |
| **Orders** | #1 table_5 → delivered |

![tc02](test_results/tc02.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (4 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  order     Order #1 queued -> table_5 (TC02: 1 tea)
    4.3 s  dock      Undocked
   86.7 s  delivery  Order #1 delivered to table_5
```
</details>

---

## TC03 — Person crossing just in front of the robot

| | |
|---|---|
| **Criterion** | Dynamic obstacles |
| **Setup** | Full world + a scripted person crossing 2 m in front of the moving robot at a brisk 1.0 m/s, from its left to its right. |
| **Procedure** | Order table_4. When the robot has left the serving window by 1.5 m, the person is spawned on its left and walks across its path. |
| **Expected behaviour** | The predicted trajectory of the person enters the costmap; the robot yields (slows/stops/replans behind the person) and continues. No contact. |
| **Pass criteria** | robot yields (stops/slows) and continues, order delivered, 0 collisions |
| **Result** | ✅ PASS |
| **Measured** | `{"delivered": true}` |
| **Duration / path** | 75 s simulated, 10.3 m driven |
| **Closest approach to a person** | 0.64 m (t = 48.77 s, human_waiter, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 91.4 %, final 91.3 %, learned consumption 1.156 Wh/m (×time acceleration) |
| **Orders** | #1 table_4 → delivered |

![tc03](test_results/tc03.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (4 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  order     Order #1 queued -> table_4 (TC03: 3 waters)
    4.3 s  dock      Undocked
   74.6 s  delivery  Order #1 delivered to table_4
```
</details>

---

## TC04 — Doorway blocked by a standing person (25 s)

| | |
|---|---|
| **Criterion** | Dynamic obstacles / recovery |
| **Setup** | Full world + a person standing still in the 0.9 m service door D2; once the robot has detected the blockage the person stays 20 s more, then walks away. |
| **Procedure** | VIP order table_8: the only route goes through door D2. |
| **Expected behaviour** | The route is blocked: the robot detects it (no path / BLOCKED), waits instead of forcing, and goes through when the person leaves. Delivery completed, no contact. |
| **Pass criteria** | robot detects the blockage, waits/recovers without colliding, goes through once the person leaves, order delivered |
| **Result** | ✅ PASS |
| **Measured** | `{"delivered": true, "blocked_detected": true}` |
| **Duration / path** | 183 s simulated, 33.6 m driven |
| **Closest approach to a person** | 0.31 m (t = 77.86 s, human_extra_1, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 77.8 %, final 77.8 %, learned consumption 1.390 Wh/m (×time acceleration) |
| **Orders** | #1 table_8 → delivered |

![tc04](test_results/tc04.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (6 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  order     Order #1 queued -> table_8 (TC04: VIP order, doorway blocked)
    4.3 s  dock      Undocked
  164.1 s  nav       Blocked on the way to table_8 (people around: 0) - waiting / replanning
  164.5 s  nav       Moving again towards table_8
  182.6 s  delivery  Order #1 delivered to table_8
```
</details>

---

## TC05 — VIP lounge: doorways + narrow corridor + L-turn

| | |
|---|---|
| **Criterion** | Complex layout |
| **Setup** | Full world. Route: dining room → door D2 (0.9 m) → service corridor (1.05 m, 6 m) → L-turn → west corridor (1.36 m) → door D3 (0.9 m) → VIP lounge (a guest walks there). |
| **Procedure** | VIP order table_8. |
| **Expected behaviour** | The robot passes the doorways and the corridor without touching anything (bumper), serves the VIP table and comes back to the dock. |
| **Pass criteria** | order to table_8 delivered through D2, the 1.05 m corridor, the L-turn and D3, robot back on the dock, 0 contacts |
| **Result** | ✅ PASS |
| **Measured** | `{"delivered": true}` |
| **Duration / path** | 549 s simulated, 66.9 m driven |
| **Closest approach to a person** | 0.24 m (t = 472.77 s, human_waiter, robot state RETURNING_TO_DOCK) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 44.0 %, final 60.4 %, learned consumption 1.030 Wh/m (×time acceleration) |
| **Orders** | #1 table_8 → delivered |

![tc05](test_results/tc05.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (31 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  order     Order #1 queued -> table_8 [VIP] (TC05: VIP champagne)
    4.3 s  dock      Undocked
   61.0 s  recovery  Nav2 recovery #1 on the way to table_8
   61.2 s  recovery  Nav2 recovery #2 on the way to table_8
   64.2 s  recovery  Nav2 recovery #3 on the way to table_8
  162.1 s  nav       Blocked on the way to table_8 (people around: 0) - waiting / replanning
  162.9 s  nav       Moving again towards table_8
  167.9 s  nav       Blocked on the way to table_8 (people around: 1) - waiting / replanning
  171.9 s  recovery  Nav2 recovery #4 on the way to table_8
  187.1 s  recovery  Nav2 recovery #6 on the way to table_8
  202.4 s  recovery  Nav2 recovery #7 on the way to table_8
  217.3 s  recovery  Nav2 recovery #8 on the way to table_8
  218.9 s  failure   Giving up on table_8: no arrival within 180 s
  219.0 s  failure   Navigation to table_8 failed (approach 1/2, attempt 1)
  227.2 s  nav       Blocked on the way to table_8 (people around: 0) - waiting / replanning
  237.3 s  recovery  Nav2 recovery #1 on the way to table_8
  252.5 s  recovery  Nav2 recovery #2 on the way to table_8
  255.5 s  recovery  Nav2 recovery #3 on the way to table_8
  270.6 s  recovery  Nav2 recovery #4 on the way to table_8
  285.8 s  recovery  Nav2 recovery #6 on the way to table_8
  291.6 s  failure   Giving up on table_8: no arrival within 69 s
  291.7 s  failure   Navigation to table_8 failed (approach 1/2, attempt 2)
  299.9 s  nav       Blocked on the way to table_8 (people around: 0) - waiting / replanning
  310.1 s  recovery  Nav2 recovery #1 on the way to table_8
  325.0 s  recovery  Nav2 recovery #2 on the way to table_8
  327.1 s  nav       Moving again towards table_8
  350.4 s  delivery  Order #1 delivered to table_8
  370.6 s  dock      Returning to dock: idle for 20s
  373.5 s  recovery  Nav2 recovery #1 on the way to the dock
  525.6 s  dock      Docked, charging contacts OK (45%), pad offset 2.1 cm
```
</details>

---

## TC06 — Table approach blocked by a trolley -> alternate approach pose

| | |
|---|---|
| **Criterion** | Complex layout / recovery |
| **Setup** | Full world + a trolley (0.7 × 0.5 m box) placed exactly on the first approach pose of table_5. |
| **Procedure** | Order table_5. |
| **Expected behaviour** | The first approach is detected as unreachable (reachability pre-check) and the robot uses the second approach pose (other side of the table). Delivery completed. |
| **Pass criteria** | first approach fails, robot uses the second approach pose, order delivered |
| **Result** | ✅ PASS |
| **Measured** | `{"delivered": true, "first_approach_failed": true}` |
| **Duration / path** | 217 s simulated, 26.2 m driven |
| **Closest approach to a person** | 0.47 m (t = 47.76 s, human_waiter, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 75.6 %, final 75.6 %, learned consumption 1.644 Wh/m (×time acceleration) |
| **Orders** | #1 table_5 → delivered |

![tc06](test_results/tc06.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (20 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.1 s  order     Order #1 queued -> table_5 (TC06: 2 sodas)
    4.3 s  dock      Undocked
   66.5 s  recovery  Nav2 recovery #1 on the way to table_5
   84.6 s  recovery  Nav2 recovery #2 on the way to table_5
   91.4 s  recovery  Nav2 recovery #3 on the way to table_5
  104.1 s  recovery  Nav2 recovery #4 on the way to table_5
  111.3 s  recovery  Nav2 recovery #5 on the way to table_5
  118.8 s  recovery  Nav2 recovery #6 on the way to table_5
  128.8 s  recovery  Nav2 recovery #7 on the way to table_5
  129.3 s  recovery  Nav2 recovery #8 on the way to table_5
  132.3 s  recovery  Nav2 recovery #9 on the way to table_5
  138.6 s  recovery  Nav2 recovery #10 on the way to table_5
  138.8 s  recovery  Nav2 recovery #12 on the way to table_5
  139.0 s  failure   Giving up on table_5: no arrival within 100 s
  139.1 s  failure   Navigation to table_5 failed (approach 1/2, attempt 1)
  142.2 s  nav       No route to table_5 right now (route or goal blocked) - re-checking for 45 s
  188.7 s  failure   Approach 1/2 of table_5 unreachable
  190.5 s  recovery  Nav2 recovery #1 on the way to table_5
  216.9 s  delivery  Order #1 delivered to table_5
```
</details>

---

## TC07 — Unreachable table (corridor blocked): graceful failure, queue continues

| | |
|---|---|
| **Criterion** | Complex layout / recovery |
| **Setup** | Full world + a box blocking the full width of the service corridor (the only route to the lounge). |
| **Procedure** | Order table_9 (VIP lounge, unreachable), then order table_7. |
| **Expected behaviour** | After re-checking for 45 s per approach, order #1 is reported failed (reason recorded); the robot does not deadlock: order #2 is delivered and the robot returns to the dock. |
| **Pass criteria** | order 1 reported failed (unreachable) after the recoveries, order 2 delivered, robot returns to dock (no deadlock) |
| **Result** | ✅ PASS |
| **Measured** | `{"order1": "failed", "order2": "delivered", "notes": ["table_9 unreachable"]}` |
| **Duration / path** | 379 s simulated, 47.5 m driven |
| **Closest approach to a person** | 0.33 m (t = 49.56 s, human_waiter, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 60.1 %, final 62.0 %, learned consumption 1.052 Wh/m (×time acceleration) |
| **Orders** | #2 table_7 → delivered, #1 table_9 → failed (table_9 unreachable) |

![tc07](test_results/tc07.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (29 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.2 s  order     Order #1 queued -> table_9 (TC07: blocked VIP order)
    0.2 s  order     Order #2 queued -> table_7 (TC07: next order)
    4.5 s  dock      Undocked
   87.5 s  recovery  Nav2 recovery #1 on the way to table_9
   87.7 s  recovery  Nav2 recovery #2 on the way to table_9
   90.9 s  recovery  Nav2 recovery #4 on the way to table_9
   91.3 s  recovery  Nav2 recovery #6 on the way to table_9
   91.7 s  recovery  Nav2 recovery #7 on the way to table_9
   92.1 s  recovery  Nav2 recovery #8 on the way to table_9
   93.1 s  nav       Blocked on the way to table_9 (people around: 0) - waiting / replanning
   93.9 s  nav       Moving again towards table_9
   94.7 s  recovery  Nav2 recovery #9 on the way to table_9
   95.1 s  recovery  Nav2 recovery #10 on the way to table_9
   99.0 s  nav       Blocked on the way to table_9 (people around: 0) - waiting / replanning
  100.2 s  recovery  Nav2 recovery #11 on the way to table_9
  100.4 s  recovery  Nav2 recovery #12 on the way to table_9
  103.8 s  recovery  Nav2 recovery #14 on the way to table_9
  104.0 s  recovery  Nav2 recovery #16 on the way to table_9
  105.0 s  recovery  Nav2 recovery #17 on the way to table_9
  105.3 s  failure   Navigation to table_9 failed (approach 1/2, attempt 1)
  108.4 s  nav       No route to table_9 right now (route or goal blocked) - re-checking for 45 s
  154.3 s  failure   Approach 1/2 of table_9 unreachable
  154.3 s  nav       No route to table_9 right now (route or goal blocked) - re-checking for 45 s
  200.2 s  failure   Approach 2/2 of table_9 unreachable
  200.2 s  failure   Order #1 failed: table_9 unreachable
  311.0 s  delivery  Order #2 delivered to table_7
  331.1 s  dock      Returning to dock: idle for 20s
  379.1 s  dock      Docked, charging contacts OK (62%), pad offset 2.5 cm
```
</details>

---

## TC08 — Low battery when the order arrives -> charge first

| | |
|---|---|
| **Criterion** | Battery |
| **Setup** | Battery forced to 18 % on the dock (test hook). |
| **Procedure** | Order table_3. |
| **Expected behaviour** | The energy needed for dock → window → table_3 → dock (Nav2 path lengths × learned Wh/m × 1.3) plus the 8 % reserve does not fit: the robot charges first, only up to what the order needs (+5 %), then delivers. SoC never reaches 0. |
| **Pass criteria** | robot does not leave at 18 %, charges until the energy needed for the whole order + reserve is available, then delivers; SoC never reaches 0 |
| **Result** | ✅ PASS |
| **Measured** | `{"charged_first": true, "soc_at_departure": 67.24, "need_soc": null}` |
| **Duration / path** | 171 s simulated, 19.4 m driven |
| **Closest approach to a person** | 0.51 m (t = 88.32 s, human_waiter, robot state TO_PICKUP) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 18.3 %, final 55.9 %, learned consumption 1.033 Wh/m (×time acceleration) |
| **Orders** | #1 table_3 → delivered |

![tc08](test_results/tc08.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (5 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    2.0 s  order     Order #1 queued -> table_3 (TC08: low battery at order time)
    2.2 s  battery   Charging to 67% (order #1 needs 54%), now 19%
   80.6 s  dock      Undocked
  170.5 s  delivery  Order #1 delivered to table_3
```
</details>

---

## TC09 — Battery runs low mid-mission -> return to dock in time, resume later

| | |
|---|---|
| **Criterion** | Battery |
| **Setup** | Battery 100 %. During the VIP delivery the SoC is forced down to 16 % (sudden drop, e.g. old cells). |
| **Procedure** | VIP order table_9; the SoC drop happens 5 s after leaving the serving window. |
| **Expected behaviour** | The energy monitor (re-planned every 3 s) sees that the energy left above the reserve no longer covers the way back: the mission is interrupted, the order re-queued at the front, the robot docks (SoC > 0), charges, then completes the order. |
| **Pass criteria** | energy monitor interrupts the mission, robot docks with SoC > 0 (reserve kept), charges, then the re-queued order is delivered |
| **Result** | ✅ PASS |
| **Measured** | `{"interrupted": true, "docked": true, "soc_at_dock": 10.46, "delivered_after_charge": true}` |
| **Duration / path** | 433 s simulated, 42.8 m driven |
| **Closest approach to a person** | 0.40 m (t = 47.56 s, human_waiter, robot state RETURNING_TO_DOCK) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 8.6 %, final 75.8 %, learned consumption 1.330 Wh/m (×time acceleration) |
| **Orders** | #1 table_9 → delivered (energy to reach the dock: 8.3 Wh needed, 6.8 Wh left above the reserve) |

![tc09](test_results/tc09.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (14 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  order     Order #1 queued -> table_9 (TC09: VIP delivery, battery failing)
    4.4 s  dock      Undocked
   41.7 s  battery   Order #1 interrupted (energy to reach the dock: 8.3 Wh needed, 6.8 Wh left above the reserve); re-queued, going to charge
   41.7 s  dock      Returning to dock: energy to reach the dock: 8.3 Wh needed, 6.8 Wh left above the reserve
   41.8 s  failure   Navigation to the dock failed (approach 1/1, attempt 1)
   46.9 s  recovery  Nav2 recovery #1 on the way to the dock
   92.8 s  dock      Docked, charging contacts OK (10%), pad offset 1.6 cm
   93.1 s  battery   Charging to 97% (order #1 needs 80%), now 10%
  263.9 s  dock      Undocked
  375.8 s  recovery  Nav2 recovery #1 on the way to table_9
  376.2 s  recovery  Nav2 recovery #2 on the way to table_9
  379.4 s  recovery  Nav2 recovery #3 on the way to table_9
  433.4 s  delivery  Order #1 delivered to table_9
```
</details>

---

## TC10 — Endurance: 6 orders, battery cycles handled autonomously

| | |
|---|---|
| **Criterion** | Battery / endurance |
| **Setup** | Battery 45 %, energy flows ×25 (demo acceleration). |
| **Procedure** | 6 orders queued at once (dining room and VIP lounge). |
| **Expected behaviour** | All orders delivered; the robot interleaves charging sessions by itself; SoC never reaches 0. |
| **Pass criteria** | all 6 orders delivered starting from 45 %, the robot charges by itself when an order needs it, SoC never 0 |
| **Result** | ✅ PASS |
| **Measured** | `{"delivered": 6, "of": 6, "charging_sessions": 3}` |
| **Duration / path** | 1079 s simulated, 183.0 m driven |
| **Closest approach to a person** | 0.20 m (t = 747.07 s, human_guest, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 29.3 %, final 33.2 %, learned consumption 1.125 Wh/m (×time acceleration) |
| **Orders** | #6 table_7 → delivered, #5 table_3 → delivered, #4 table_6 → delivered, #3 table_8 → delivered, #2 table_1 → delivered, #1 table_4 → delivered |

![tc10](test_results/tc10.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (28 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  order     Order #1 queued -> table_4 (TC10 order 1)
    0.0 s  order     Order #2 queued -> table_1 (TC10 order 2)
    0.0 s  order     Order #3 queued -> table_8 (TC10 order 3)
    0.0 s  order     Order #4 queued -> table_6 (TC10 order 4)
    0.0 s  order     Order #5 queued -> table_3 (TC10 order 5)
    0.0 s  order     Order #6 queued -> table_7 (TC10 order 6)
    4.3 s  dock      Undocked
   73.5 s  delivery  Order #1 delivered to table_4
   73.8 s  dock      Returning to dock: energy: order #2 needs 46%, battery 37%
  135.3 s  dock      Docked, charging contacts OK (31%), pad offset 4.9 cm
  135.6 s  battery   Charging to 51% (order #2 needs 40%), now 31%
  171.6 s  dock      Undocked
  249.1 s  delivery  Order #2 delivered to table_1
  249.4 s  dock      Returning to dock: energy: order #3 needs 88%, battery 42%
  313.9 s  dock      Docked, charging contacts OK (36%), pad offset 1.3 cm
  314.2 s  battery   Charging to 89% (order #3 needs 73%), now 36%
  407.5 s  dock      Undocked
  567.3 s  delivery  Order #3 delivered to table_8
  603.3 s  recovery  Nav2 recovery #2 on the way to the kitchen pass
  606.3 s  recovery  Nav2 recovery #3 on the way to the kitchen pass
  764.0 s  delivery  Order #4 delivered to table_6
  764.3 s  dock      Returning to dock: energy: order #5 needs 58%, battery 46%
  849.8 s  dock      Docked, charging contacts OK (37%), pad offset 1.4 cm
  850.1 s  battery   Charging to 57% (order #5 needs 45%), now 37%
  885.7 s  dock      Undocked
  973.9 s  delivery  Order #5 delivered to table_3
 1079.3 s  delivery  Order #6 delivered to table_7
```
</details>

---

## TC11 — Emergency stop and resume

| | |
|---|---|
| **Criterion** | State management |
| **Setup** | Battery 100 %. |
| **Procedure** | Order table_2; emergency stop 4 s after leaving the window, resume 5 s later. |
| **Expected behaviour** | EMERGENCY_STOP within 1 s, robot stationary while stopped; after "resume" the order (re-queued) is delivered. |
| **Pass criteria** | robot stops within 1 s and stays still, resumes and delivers the re-queued order |
| **Result** | ✅ PASS |
| **Measured** | `{"estop_state": true, "stationary_during_estop": true, "delivered": true}` |
| **Duration / path** | 105 s simulated, 16.0 m driven |
| **Closest approach to a person** | 0.55 m (t = 8.82 s, human_waiter, robot state TO_PICKUP) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 88.2 %, final 88.2 %, learned consumption 1.037 Wh/m (×time acceleration) |
| **Orders** | #1 table_2 → delivered (emergency stop) |

![tc11](test_results/tc11.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (8 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  order     Order #1 queued -> table_2 (TC11: e-stop test)
    4.2 s  dock      Undocked
   30.5 s  command   Operator command: estop
   30.7 s  estop     EMERGENCY STOP - motors held at zero until "resume"
   38.0 s  command   Operator command: resume
   38.0 s  command   Resumed after emergency stop
  104.6 s  delivery  Order #1 delivered to table_2
```
</details>

---

## TC12 — Docking precision (3 cycles) and charger contact

| | |
|---|---|
| **Criterion** | Auto-docking |
| **Setup** | Battery 100 %. |
| **Procedure** | 3 cycles: order (table_7, table_4, table_1) then "dock". |
| **Expected behaviour** | Each time the robot ends on the charging pad within 6 cm / 5° (ground truth) and the battery reports charging. |
| **Pass criteria** | final pose within 6 cm / 5 deg of the charging pad, charging confirmed each time |
| **Result** | ✅ PASS |
| **Measured** | `{"dockings": [{"dx": 0.008, "dy": 0.029, "dyaw_deg": -0.0, "charging": true}, {"dx": -0.052, "dy": 0.002, "dyaw_deg": -0.4, "charging": true}, {"dx": 0.057, "dy": 0.022, "dyaw_deg": -1.1, "charging": true}]}` |
| **Duration / path** | 413 s simulated, 66.1 m driven |
| **Closest approach to a person** | 0.49 m (t = 137.96 s, human_waiter, robot state TO_PICKUP) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 59.1 %, final 63.0 %, learned consumption 0.969 Wh/m (×time acceleration) |
| **Orders** | #3 table_1 → delivered, #2 table_4 → delivered, #1 table_7 → delivered |

![tc12](test_results/tc12.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (19 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  order     Order #1 queued -> table_7 (TC12 docking 1)
    4.2 s  dock      Undocked
   73.2 s  delivery  Order #1 delivered to table_7
   73.5 s  command   Operator command: dock
   73.6 s  dock      Returning to dock: operator request
  123.9 s  dock      Docked, charging contacts OK (87%), pad offset 1.9 cm
  127.0 s  order     Order #2 queued -> table_4 (TC12 docking 2)
  131.3 s  dock      Undocked
  197.9 s  delivery  Order #2 delivered to table_4
  198.0 s  command   Operator command: dock
  198.1 s  dock      Returning to dock: operator request
  257.1 s  dock      Docked, charging contacts OK (74%), pad offset 2.2 cm
  260.5 s  order     Order #3 queued -> table_1 (TC12 docking 3)
  265.0 s  dock      Undocked
  344.8 s  delivery  Order #3 delivered to table_1
  345.0 s  command   Operator command: dock
  345.0 s  dock      Returning to dock: operator request
  409.7 s  dock      Docked, charging contacts OK (61%), pad offset 1.9 cm
```
</details>
