# TuniBot — Phase 2 test cases

Every test case is an automated scenario (`tunibot_delivery/scenario_runner.py`) run on a **fresh simulation stack** (`scripts/run_tests.sh`): Gazebo (headless) + Nav2 + delivery stack. Metrics come from **Gazebo ground truth**, not from the robot's own estimates:

* **min clearance** — smallest distance between the robot footprint (0.54 × 0.50 m at its true pose) and any person body (0.22 m radius); ≤ 0 would be a collision,
* **contacts** — bumper (contact sensor on the robot body, spine and lidar) touching anything,
* order outcomes, durations, path length, battery level (min / final), full event log.

A test fails if its functional criterion fails, if there is any contact, or if the minimum clearance to a person is below its threshold.

## Summary

| ID | Criterion | Scenario | Result | Duration (s) | Min clearance to people (m) | Min TTC (s) | Intimate / personal space (s) | Contacts | Min SoC (%) |
|---|---|---|---|---|---|---|---|---|---|
| TC01 | Dynamic obstacles | Delivery with people walking around (ambient crowd) | ✅ PASS | 201 | 0.23 | 1.06 | 11.1 / 67.6 | 0 | 77.1 |
| TC02 | Dynamic obstacles | Person walking head-on towards the robot | ✅ PASS | 81 | 0.34 | 0.71 | 1.4 / 13.5 | 0 | 90.5 |
| TC03 | Dynamic obstacles | Person crossing just in front of the robot | ✅ PASS | 72 | 0.46 | 0.82 | 0.0 / 14.2 | 0 | 91.7 |
| TC04 | Dynamic obstacles / recovery | Doorway blocked by a standing person (25 s) | ✅ PASS | 169 | 0.42 | 1.59 | 1.0 / 42.7 | 0 | 79.5 |
| TC05 | Complex layout | VIP lounge: doorways + narrow corridor + L-turn | ✅ PASS | 348 | 0.26 | 1.74 | 3.1 / 82.9 | 0 | 58.6 |
| TC06 | Complex layout / recovery | Table approach blocked by a trolley -> alternate approach pose | ✅ PASS | 225 | 0.47 | 1.01 | 0.0 / 54.1 | 0 | 75.7 |
| TC07 | Complex layout / recovery | Unreachable table (corridor blocked): graceful failure, queue continues | ✅ PASS | 430 | 0.13 | 0.29 | 7.4 / 69.3 | 0 | 56.4 |
| TC08 | Battery | Low battery when the order arrives -> charge first | ✅ PASS | 155 | 0.15 | 1.09 | 0.2 / 8.4 | 0 | 18.3 |
| TC09 | Battery | Battery runs low mid-mission -> return to dock in time, resume later | ✅ PASS | 493 | 0.61 | 2.96 | 0.0 / 60.1 | 0 | 9.0 |
| TC10 | Battery / endurance | Endurance: 6 orders, battery cycles handled autonomously | ✅ PASS | 1112 | 0.22 | 1.02 | 6.0 / 168.4 | 0 | 23.3 |
| TC11 | State management | Emergency stop and resume | ✅ PASS | 102 | 0.95 | 9.62 | 0.0 / 4.6 | 0 | 88.6 |
| TC12 | Auto-docking | Docking precision (3 cycles) and charger contact | ✅ PASS | 397 | 0.38 | 8.10 | 1.2 / 14.5 | 0 | 65.2 |
| TC13 | Dynamic obstacles | Group of two people walking head-on | ✅ PASS | 108 | 0.13 | 0.35 | 7.7 / 50.4 | 0 | 87.0 |
| TC14 | Dynamic obstacles | Person stops suddenly in front of the robot | ✅ PASS | 82 | 0.38 | 0.83 | 4.1 / 22.2 | 0 | 90.3 |
| TC15 | Battery & state | Charger fault on the dock: detection and recovery | ✅ PASS | 109 | 0.60 | 1.74 | 0.0 / 5.0 | 0 | 16.4 |
| TC16 | Complex layout | Serving window 1 blocked -> window 2 (beside the takeaway queue) | ✅ PASS | 122 | 0.31 | 7.10 | 1.8 / 14.2 | 0 | 87.9 |
| TC17 | Delivery task management | VIP order jumps the queue | ✅ PASS | 348 | 0.19 | 0.74 | 11.2 / 93.9 | 0 | 58.9 |
| TC18 | Delivery task management | Order cancelled during the delivery, queue continues | ✅ PASS | 172 | 0.65 | 5.65 | 0.0 / 8.0 | 0 | 81.1 |
| TC19 | Delivery task management | Queue cleared by the operator during a delivery | ✅ PASS | 159 | 0.32 | 3.01 | 2.1 / 19.4 | 0 | 82.3 |
| TC20 | Delivery task management | Order validation: unknown tables rejected, plain-text order served | ✅ PASS | 177 | 0.62 | 2.93 | 0.0 / 7.8 | 0 | 80.0 |
| TC21 | Delivery task management / endurance | Full service: every table (1-9) served once | ✅ PASS | 1711 | 0.14 | 0.81 | 17.7 / 324.7 | 0 | 25.0 |
| TC22 | Delivery task management / battery | Order arrives while charging -> leaves without waiting for a full charge | ✅ PASS | 116 | 0.16 | 2.59 | 6.3 / 35.3 | 0 | 40.3 |
| TC23 | Delivery task management / navigation | Tables served from the closest side (shortest route) | ✅ PASS | 309 | 0.36 | 0.98 | 1.0 / 22.5 | 0 | 65.0 |

**23 / 23 test cases passed.**

Reproduce: `src/tunibot_delivery/scripts/run_tests.sh` then `ros2 run tunibot_delivery make_test_report ~/tunibot_test_results`.

**Social-navigation metrics** (definitions after SocNavBench / Arena-Rosnav): *min TTC* = smallest time-to-collision between the robot and a person who is closing in (constant velocities, capped at 10 s); *intimate / personal space* = time the moving robot spent closer than 0.45 m / 1.2 m to a person (Hall's proxemic zones).

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
| **Duration / path** | 201 s simulated, 31.6 m driven |
| **Closest approach to a person** | 0.23 m (t = 112.52 s, human_wc, robot state RETURNING_TO_DOCK) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 77.1 %, final 81.0 %, learned consumption 1.212 Wh/m (×time acceleration) |
| **Orders** | #1 table_6 → delivered |

![tc01](test_results/tc01.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (8 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.596 m behind the lidar, at 0.0 deg
    0.0 s  order     Order #1 queued -> table_6 (TC01: 2 coffees)
    4.3 s  dock      Undocked
   41.1 s  nav       Serving table_6 from side 2 of 2 (12.7 m, shortest planned route)
   83.0 s  hri       Robot says: "Hello! Your order has arrived (TC01: 2 coffees). Please take it from the tray."
   91.1 s  delivery  Order #1 delivered to table_6
  111.2 s  dock      Returning to dock: idle for 20s
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
| **Duration / path** | 81 s simulated, 11.7 m driven |
| **Closest approach to a person** | 0.34 m (t = 55.94 s, human_waiter, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 90.5 %, final 90.5 %, learned consumption 1.189 Wh/m (×time acceleration) |
| **Orders** | #1 table_5 → delivered |

![tc02](test_results/tc02.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (7 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.596 m behind the lidar, at 0.1 deg
    0.0 s  order     Order #1 queued -> table_5 (TC02: 1 tea)
    4.3 s  dock      Undocked
   39.6 s  nav       Serving table_5 from side 2 of 2 (8.2 m, shortest planned route)
   72.3 s  hri       Robot says: "Hello! Your order has arrived (TC02: 1 tea). Please take it from the tray."
   80.5 s  delivery  Order #1 delivered to table_5
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
| **Duration / path** | 72 s simulated, 8.4 m driven |
| **Closest approach to a person** | 0.46 m (t = 46.46 s, human_extra_1, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 91.7 %, final 91.7 %, learned consumption 1.403 Wh/m (×time acceleration) |
| **Orders** | #1 table_4 → delivered |

![tc03](test_results/tc03.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (7 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.600 m behind the lidar, at -0.1 deg
    0.0 s  order     Order #1 queued -> table_4 (TC03: 3 waters)
    4.4 s  dock      Undocked
   39.5 s  nav       Serving table_4 from side 2 of 2 (5.2 m, shortest planned route)
   63.3 s  hri       Robot says: "Hello! Your order has arrived (TC03: 3 waters). Please take it from the tray."
   71.5 s  delivery  Order #1 delivered to table_4
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
| **Duration / path** | 169 s simulated, 33.7 m driven |
| **Closest approach to a person** | 0.42 m (t = 50.77 s, human_waiter, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 79.5 %, final 79.5 %, learned consumption 1.374 Wh/m (×time acceleration) |
| **Orders** | #1 table_8 → delivered |

![tc04](test_results/tc04.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (7 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.595 m behind the lidar, at 0.0 deg
    0.0 s  order     Order #1 queued -> table_8 (TC04: VIP order, doorway blocked)
    4.5 s  dock      Undocked
   39.0 s  nav       Serving table_8 from side 1 of 4 (31.5 m, shortest planned route)
  160.7 s  hri       Robot says: "Hello! Your order has arrived (TC04: VIP order, doorway blocked). Please take it from the tray."
  168.7 s  delivery  Order #1 delivered to table_8
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
| **Duration / path** | 348 s simulated, 67.6 m driven |
| **Closest approach to a person** | 0.26 m (t = 85.95 s, human_takeaway, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 58.6 %, final 61.0 %, learned consumption 1.175 Wh/m (×time acceleration) |
| **Orders** | #1 table_8 → delivered |

![tc05](test_results/tc05.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (11 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.596 m behind the lidar, at 0.0 deg
    0.0 s  order     Order #1 queued -> table_8 [VIP] (TC05: VIP champagne)
    4.5 s  dock      Undocked
   39.8 s  nav       Serving table_8 from side 1 of 4 (31.4 m, shortest planned route)
   57.8 s  recovery  Nav2 recovery #1 on the way to table_8
  116.7 s  recovery  Nav2 recovery #2 on the way to table_8
  161.9 s  hri       Robot says: "Hello! Your order has arrived (TC05: VIP champagne). Please take it from the tray."
  169.9 s  delivery  Order #1 delivered to table_8
  190.1 s  dock      Returning to dock: idle for 20s
  347.5 s  dock      Docked, charging contacts OK (61%), pad offset 1.7 cm (lidar)
```
</details>

---

## TC06 — Table approach blocked by a trolley -> alternate approach pose

| | |
|---|---|
| **Criterion** | Complex layout / recovery |
| **Setup** | Full world + a trolley (0.7 × 0.5 m box) placed exactly on the closest approach pose of table_5 (north side, the one facing the kitchen). |
| **Procedure** | Order table_5. |
| **Expected behaviour** | The closest side is detected as unreachable (reachability pre-check) and the robot uses the next closest approach pose (other side of the table). Delivery completed. |
| **Pass criteria** | closest approach fails, robot uses the other side of the table, order delivered |
| **Result** | ✅ PASS |
| **Measured** | `{"delivered": true, "first_approach_failed": true}` |
| **Duration / path** | 225 s simulated, 20.4 m driven |
| **Closest approach to a person** | 0.47 m (t = 50.65 s, human_waiter, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 75.7 %, final 75.7 %, learned consumption 1.535 Wh/m (×time acceleration) |
| **Orders** | #1 table_5 → delivered |

![tc06](test_results/tc06.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (27 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.598 m behind the lidar, at -0.0 deg
    0.2 s  order     Order #1 queued -> table_5 (TC06: 2 sodas)
    4.5 s  dock      Undocked
   40.4 s  nav       Serving table_5 from side 2 of 2 (8.3 m, shortest planned route)
   73.9 s  recovery  Nav2 recovery #1 on the way to table_5
   78.1 s  recovery  Nav2 recovery #2 on the way to table_5
   79.7 s  recovery  Nav2 recovery #3 on the way to table_5
   82.8 s  recovery  Nav2 recovery #4 on the way to table_5
   83.4 s  nav       Blocked on the way to table_5 (people around: 1) - waiting / replanning
   84.2 s  nav       Moving again towards table_5
   91.2 s  recovery  Nav2 recovery #5 on the way to table_5
  118.1 s  nav       Blocked on the way to table_5 (people around: 4) - waiting / replanning
  118.1 s  hri       Robot says: "Excuse me, may I pass? Thank you!"
  125.3 s  nav       Moving again towards table_5
  129.0 s  recovery  Nav2 recovery #7 on the way to table_5
  135.8 s  failure   Giving up on table_5: no arrival within 93 s
  135.9 s  failure   Navigation to table_5 failed (approach 1/2, attempt 1)
  139.0 s  nav       No route to table_5 right now (route or goal blocked) - re-checking for 45 s
  184.0 s  failure   Approach 1/2 of table_5 unreachable
  188.4 s  recovery  Nav2 recovery #1 on the way to table_5
  188.8 s  recovery  Nav2 recovery #3 on the way to table_5
  193.7 s  nav       Blocked on the way to table_5 (people around: 2) - waiting / replanning
  193.7 s  hri       Robot says: "Excuse me, may I pass? Thank you!"
  195.0 s  nav       Moving again towards table_5
  216.5 s  hri       Robot says: "Hello! Your order has arrived (TC06: 2 sodas). Please take it from the tray."
  224.6 s  delivery  Order #1 delivered to table_5
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
| **Duration / path** | 430 s simulated, 41.3 m driven |
| **Closest approach to a person** | 0.13 m (t = 205.67 s, human_takeaway, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 56.4 %, final 60.6 %, learned consumption 1.201 Wh/m (×time acceleration) |
| **Orders** | #2 table_7 → delivered, #1 table_9 → failed (table_9 unreachable) |

![tc07](test_results/tc07.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (36 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.598 m behind the lidar, at -0.0 deg
    0.3 s  order     Order #1 queued -> table_9 (TC07: blocked VIP order)
    0.3 s  order     Order #2 queued -> table_7 (TC07: next order)
    4.7 s  dock      Undocked
   40.3 s  nav       Serving table_9 from side 3 of 3 (32.9 m, shortest planned route)
   87.4 s  recovery  Nav2 recovery #2 on the way to table_9
   90.5 s  recovery  Nav2 recovery #3 on the way to table_9
   90.7 s  recovery  Nav2 recovery #4 on the way to table_9
   90.9 s  recovery  Nav2 recovery #6 on the way to table_9
   91.3 s  recovery  Nav2 recovery #7 on the way to table_9
   91.9 s  recovery  Nav2 recovery #8 on the way to table_9
   92.1 s  nav       Blocked on the way to table_9 (people around: 0) - waiting / replanning
   94.2 s  nav       Moving again towards table_9
   94.4 s  recovery  Nav2 recovery #9 on the way to table_9
   95.7 s  recovery  Nav2 recovery #10 on the way to table_9
   96.1 s  recovery  Nav2 recovery #11 on the way to table_9
  101.2 s  recovery  Nav2 recovery #12 on the way to table_9
  102.9 s  recovery  Nav2 recovery #13 on the way to table_9
  103.3 s  recovery  Nav2 recovery #14 on the way to table_9
  106.4 s  recovery  Nav2 recovery #16 on the way to table_9
  106.6 s  recovery  Nav2 recovery #18 on the way to table_9
  107.2 s  recovery  Nav2 recovery #19 on the way to table_9
  107.6 s  failure   Navigation to table_9 failed (approach 1/3, attempt 1)
  110.7 s  nav       No route to table_9 right now (route or goal blocked) - re-checking for 45 s
  156.3 s  failure   Approach 1/3 of table_9 unreachable
  156.3 s  nav       No route to table_9 right now (route or goal blocked) - re-checking for 45 s
  201.9 s  failure   Approach 2/3 of table_9 unreachable
  201.9 s  nav       No route to table_9 right now (route or goal blocked) - re-checking for 45 s
  247.6 s  failure   Approach 3/3 of table_9 unreachable
  247.6 s  failure   Order #1 failed: table_9 unreachable
  338.1 s  nav       Serving table_7 from side 2 of 2 (2.9 m, shortest planned route)
  346.2 s  hri       Robot says: "Hello! Your order has arrived (TC07: next order). Please take it from the tray."
  354.3 s  delivery  Order #2 delivered to table_7
  374.4 s  dock      Returning to dock: idle for 20s
  429.6 s  dock      Docked, charging contacts OK (60%), pad offset 2.5 cm (lidar)
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
| **Measured** | `{"charged_first": true, "soc_at_departure": 57.34, "need_soc": null}` |
| **Duration / path** | 155 s simulated, 17.2 m driven |
| **Closest approach to a person** | 0.15 m (t = 154.64 s, human_guest, robot state SERVING) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 18.3 %, final 46.1 %, learned consumption 1.010 Wh/m (×time acceleration) |
| **Orders** | #1 table_3 → delivered |

![tc08](test_results/tc08.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (9 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.600 m behind the lidar, at 0.0 deg
    2.0 s  order     Order #1 queued -> table_3 (TC08: low battery at order time)
    2.3 s  battery   Charging to 57% (order #1 needs 45%), now 19%
   65.3 s  dock      Undocked
  100.0 s  nav       Serving table_3 from side 2 of 2 (14.0 m, shortest planned route)
  127.3 s  recovery  Nav2 recovery #1 on the way to table_3
  146.9 s  hri       Robot says: "Hello! Your order has arrived (TC08: low battery at order time). Please take it from the tray."
  155.0 s  delivery  Order #1 delivered to table_3
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
| **Measured** | `{"interrupted": true, "docked": true, "soc_at_dock": 11.5, "delivered_after_charge": true}` |
| **Duration / path** | 493 s simulated, 43.4 m driven |
| **Closest approach to a person** | 0.61 m (t = 359.51 s, human_waiter, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 9.0 %, final 77.7 %, learned consumption 1.315 Wh/m (×time acceleration) |
| **Orders** | #1 table_9 → delivered (energy to reach the dock: 9.0 Wh needed, 6.8 Wh left above the reserve) |

![tc09](test_results/tc09.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (13 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.598 m behind the lidar, at -0.0 deg
    0.0 s  order     Order #1 queued -> table_9 (TC09: VIP delivery, battery failing)
    4.4 s  dock      Undocked
   38.9 s  nav       Serving table_9 from side 3 of 3 (33.1 m, shortest planned route)
   41.9 s  battery   Order #1 interrupted (energy to reach the dock: 9.0 Wh needed, 6.8 Wh left above the reserve); re-queued, going to charge
   41.9 s  dock      Returning to dock: energy to reach the dock: 9.0 Wh needed, 6.8 Wh left above the reserve
   92.2 s  dock      Docked, charging contacts OK (11%), pad offset 1.4 cm (lidar)
   92.6 s  battery   Charging to 100% (order #1 needs 82%), now 12%
  317.4 s  dock      Undocked
  354.1 s  nav       Serving table_9 from side 3 of 3 (33.1 m, shortest planned route)
  484.9 s  hri       Robot says: "Hello! Your order has arrived (TC09: VIP delivery, battery failing). Please take it from the tray."
  493.1 s  delivery  Order #1 delivered to table_9
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
| **Measured** | `{"delivered": 6, "of": 6, "charging_sessions": 2}` |
| **Duration / path** | 1112 s simulated, 165.5 m driven |
| **Closest approach to a person** | 0.22 m (t = 246.92 s, human_waiter, robot state RETURNING_TO_DOCK) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 23.3 %, final 23.2 %, learned consumption 1.166 Wh/m (×time acceleration) |
| **Orders** | #6 table_7 → delivered, #5 table_3 → delivered, #4 table_6 → delivered, #3 table_8 → delivered, #2 table_1 → delivered, #1 table_4 → delivered |

![tc10](test_results/tc10.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (49 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.594 m behind the lidar, at 0.1 deg
    0.0 s  order     Order #1 queued -> table_4 (TC10 order 1)
    0.0 s  order     Order #2 queued -> table_1 (TC10 order 2)
    0.0 s  order     Order #3 queued -> table_8 (TC10 order 3)
    0.0 s  order     Order #4 queued -> table_6 (TC10 order 4)
    0.0 s  order     Order #5 queued -> table_3 (TC10 order 5)
    0.0 s  order     Order #6 queued -> table_7 (TC10 order 6)
    4.3 s  dock      Undocked
   39.8 s  nav       Serving table_4 from side 2 of 2 (5.2 m, shortest planned route)
   61.1 s  hri       Robot says: "Hello! Your order has arrived (TC10 order 1). Please take it from the tray."
   69.3 s  delivery  Order #1 delivered to table_4
   69.5 s  dock      Returning to dock: energy: order #2 needs 38%, battery 37%
  130.9 s  dock      Docked, charging contacts OK (34%), pad offset 3.1 cm (lidar)
  131.1 s  battery   Charging to 44% (order #2 needs 34%), now 34%
  151.4 s  dock      Undocked
  186.2 s  nav       Serving table_1 from side 2 of 2 (8.5 m, shortest planned route)
  219.9 s  hri       Robot says: "Hello! Your order has arrived (TC10 order 2). Please take it from the tray."
  228.0 s  delivery  Order #2 delivered to table_1
  228.5 s  dock      Returning to dock: energy: order #3 needs 86%, battery 35%
  301.6 s  dock      Docked, charging contacts OK (31%), pad offset 0.8 cm (lidar)
  302.1 s  battery   Charging to 97% (order #3 needs 80%), now 32%
  443.7 s  dock      Undocked
  479.0 s  nav       Serving table_8 from side 1 of 4 (31.6 m, shortest planned route)
  489.5 s  recovery  Nav2 recovery #1 on the way to table_8
  561.9 s  recovery  Nav2 recovery #2 on the way to table_8
  562.7 s  recovery  Nav2 recovery #3 on the way to table_8
  564.1 s  recovery  Nav2 recovery #4 on the way to table_8
  597.5 s  hri       Robot says: "Hello! Your order has arrived (TC10 order 3). Please take it from the tray."
  605.5 s  delivery  Order #3 delivered to table_8
  752.0 s  nav       Serving table_6 from side 2 of 2 (12.9 m, shortest planned route)
  799.3 s  hri       Robot says: "Hello! Your order has arrived (TC10 order 4). Please take it from the tray."
  807.3 s  delivery  Order #4 delivered to table_6
  807.7 s  dock      Returning to dock: energy: order #5 needs 57%, battery 52%
  860.9 s  nav       Blocked on the way to the dock (people around: 1) - waiting / replanning
  860.9 s  hri       Robot says: "Excuse me, may I pass? Thank you!"
  869.4 s  recovery  Nav2 recovery #1 on the way to the dock
  884.6 s  recovery  Nav2 recovery #2 on the way to the dock
  887.7 s  recovery  Nav2 recovery #3 on the way to the dock
  888.6 s  nav       Moving again towards the dock
  931.5 s  dock      Docked, charging contacts OK (44%), pad offset 0.9 cm (lidar)
  936.1 s  dock      Undocked
  972.2 s  nav       Serving table_3 from side 2 of 2 (14.0 m, shortest planned route)
 1014.4 s  hri       Robot says: "Hello! Your order has arrived (TC10 order 5). Please take it from the tray."
 1022.5 s  delivery  Order #5 delivered to table_3
 1094.8 s  nav       Serving table_7 from side 2 of 2 (3.0 m, shortest planned route)
 1103.5 s  hri       Robot says: "Hello! Your order has arrived (TC10 order 6). Please take it from the tray."
 1111.5 s  delivery  Order #6 delivered to table_7
 1111.7 s  dock      Returning to dock: battery low (23%)
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
| **Duration / path** | 102 s simulated, 13.8 m driven |
| **Closest approach to a person** | 0.95 m (t = 60.52 s, human_waiter, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 88.6 %, final 88.5 %, learned consumption 1.081 Wh/m (×time acceleration) |
| **Orders** | #1 table_2 → delivered (emergency stop) |

![tc11](test_results/tc11.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (12 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.595 m behind the lidar, at 0.1 deg
    0.0 s  order     Order #1 queued -> table_2 (TC11: e-stop test)
    4.3 s  dock      Undocked
   30.5 s  command   Operator command: estop
   30.7 s  nav       Serving table_2 from side 2 of 2 (10.1 m, shortest planned route)
   30.7 s  estop     EMERGENCY STOP - motors held at zero until "resume"
   38.0 s  command   Operator command: resume
   38.0 s  command   Resumed after emergency stop
   62.2 s  nav       Serving table_2 from side 2 of 2 (10.6 m, shortest planned route)
   93.8 s  hri       Robot says: "Hello! Your order has arrived (TC11: e-stop test). Please take it from the tray."
  101.9 s  delivery  Order #1 delivered to table_2
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
| **Measured** | `{"dockings": [{"dx": 0.011, "dy": 0.012, "dyaw_deg": 0.0, "charging": true}, {"dx": 0.011, "dy": -0.023, "dyaw_deg": 0.4, "charging": true}, {"dx": -0.005, "dy": -0.003, "dyaw_deg": -0.2, "charging": true}]}` |
| **Duration / path** | 397 s simulated, 48.9 m driven |
| **Closest approach to a person** | 0.38 m (t = 195.97 s, human_waiter, robot state RETURNING_TO_DOCK) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 65.2 %, final 70.9 %, learned consumption 1.171 Wh/m (×time acceleration) |
| **Orders** | #3 table_1 → delivered, #2 table_4 → delivered, #1 table_7 → delivered |

![tc12](test_results/tc12.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (26 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.596 m behind the lidar, at 0.1 deg
    0.0 s  order     Order #1 queued -> table_7 (TC12 docking 1)
    4.2 s  dock      Undocked
   39.1 s  nav       Serving table_7 from side 2 of 2 (3.0 m, shortest planned route)
   47.5 s  hri       Robot says: "Hello! Your order has arrived (TC12 docking 1). Please take it from the tray."
   55.6 s  delivery  Order #1 delivered to table_7
   56.0 s  command   Operator command: dock
   56.1 s  dock      Returning to dock: operator request
  113.1 s  dock      Docked, charging contacts OK (90%), pad offset 0.8 cm (lidar)
  115.6 s  order     Order #2 queued -> table_4 (TC12 docking 2)
  119.9 s  dock      Undocked
  154.2 s  nav       Serving table_4 from side 2 of 2 (5.3 m, shortest planned route)
  172.0 s  hri       Robot says: "Hello! Your order has arrived (TC12 docking 2). Please take it from the tray."
  180.2 s  delivery  Order #2 delivered to table_4
  180.5 s  command   Operator command: dock
  180.5 s  dock      Returning to dock: operator request
  242.3 s  dock      Docked, charging contacts OK (81%), pad offset 1.7 cm (lidar)
  244.6 s  order     Order #3 queued -> table_1 (TC12 docking 3)
  248.9 s  dock      Undocked
  283.1 s  nav       Serving table_1 from side 2 of 2 (8.4 m, shortest planned route)
  313.5 s  hri       Robot says: "Hello! Your order has arrived (TC12 docking 3). Please take it from the tray."
  321.6 s  delivery  Order #3 delivered to table_1
  322.0 s  command   Operator command: dock
  322.0 s  dock      Returning to dock: operator request
  395.4 s  dock      Docked, charging contacts OK (69%), pad offset 0.4 cm (lidar)
```
</details>

---

## TC13 — Group of two people walking head-on

| | |
|---|---|
| **Criterion** | Dynamic obstacles |
| **Setup** | Full world + two scripted people walking side by side (0.7 m apart) 5 m ahead of the robot, head-on at 0.8 m/s. |
| **Procedure** | Order table_6. When the robot has left the serving window by 1.5 m, the pair starts walking towards it. |
| **Expected behaviour** | The robot treats the pair as a group: it slows down and passes around it (never between the two people), no contact. |
| **Pass criteria** | robot gives way around the group (never between the two people), order delivered, 0 collisions, clearance > 0.10 m |
| **Result** | ✅ PASS |
| **Measured** | `{"delivered": true}` |
| **Duration / path** | 108 s simulated, 17.5 m driven |
| **Closest approach to a person** | 0.13 m (t = 56.12 s, human_waiter, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 87.0 %, final 87.0 %, learned consumption 1.363 Wh/m (×time acceleration) |
| **Orders** | #1 table_6 → delivered |

![tc13](test_results/tc13.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (7 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.595 m behind the lidar, at 0.1 deg
    0.0 s  order     Order #1 queued -> table_6 (TC13: group)
    4.3 s  dock      Undocked
   37.9 s  nav       Serving table_6 from side 2 of 2 (12.4 m, shortest planned route)
   99.5 s  hri       Robot says: "Hello! Your order has arrived (TC13: group). Please take it from the tray."
  107.7 s  delivery  Order #1 delivered to table_6
```
</details>

---

## TC14 — Person stops suddenly in front of the robot

| | |
|---|---|
| **Criterion** | Dynamic obstacles |
| **Setup** | Full world + a scripted person who cuts in 1.8 m in front of the robot at 1.1 m/s and stops dead on its path for 12 s, then walks away. |
| **Procedure** | Order table_5. The person cuts in once the robot has left the serving window. |
| **Expected behaviour** | Collision monitor / controller brake, no contact; the robot waits or goes around; when blocked with a person close it says "Excuse me, may I pass?"; then it delivers. |
| **Pass criteria** | robot brakes without contact, waits or goes around (and says "excuse me" if it is blocked), then delivers; 0 collisions, clearance > 0.05 m |
| **Result** | ✅ PASS |
| **Measured** | `{"delivered": true, "blocked_or_excuse_me": false}` |
| **Duration / path** | 82 s simulated, 11.7 m driven |
| **Closest approach to a person** | 0.38 m (t = 55.94 s, human_waiter, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 90.3 %, final 90.3 %, learned consumption 1.227 Wh/m (×time acceleration) |
| **Orders** | #1 table_5 → delivered |

![tc14](test_results/tc14.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (7 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.599 m behind the lidar, at -0.1 deg
    0.0 s  order     Order #1 queued -> table_5 (TC14: sudden stop)
    4.2 s  dock      Undocked
   38.5 s  nav       Serving table_5 from side 2 of 2 (8.3 m, shortest planned route)
   74.2 s  hri       Robot says: "Hello! Your order has arrived (TC14: sudden stop). Please take it from the tray."
   82.3 s  delivery  Order #1 delivered to table_5
```
</details>

---

## TC15 — Charger fault on the dock: detection and recovery

| | |
|---|---|
| **Criterion** | Battery & state |
| **Setup** | Battery forced to 18 %, charger switched off (fault injection) while the robot is on the dock. |
| **Procedure** | Order table_7. After the fault is reported, wait 10 s, then switch the charger back on. |
| **Expected behaviour** | The robot does not leave with too little energy; it detects "on the pad but no current" within ~10 s and reports a charger fault; when the charger works again it charges and delivers. |
| **Pass criteria** | charger off at 18 %: robot detects "on the pad but no current" within ~10 s, reports it, does not leave with too little energy; after the repair it charges and delivers |
| **Result** | ✅ PASS |
| **Measured** | `{"fault_detected": true, "left_without_energy": false, "resumed": true, "delivered": true}` |
| **Duration / path** | 109 s simulated, 6.1 m driven |
| **Closest approach to a person** | 0.60 m (t = 101.3 s, human_waiter, robot state SERVING) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 16.4 %, final 27.6 %, learned consumption 1.294 Wh/m (×time acceleration) |
| **Orders** | #1 table_7 → delivered |

![tc15](test_results/tc15.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (10 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.599 m behind the lidar, at 0.0 deg
    2.0 s  order     Order #1 queued -> table_7 (TC15: charger fault)
    2.2 s  battery   Charging to 34% (order #1 needs 25%), now 18%
   12.3 s  error     Charger fault: on the pad but no charging current for 10 s - staff notified, orders wait until it is fixed
   22.9 s  battery   Charger OK again, charging resumed (16%)
   54.1 s  dock      Undocked
   90.8 s  nav       Serving table_7 from side 2 of 2 (3.0 m, shortest planned route)
  100.5 s  hri       Robot says: "Hello! Your order has arrived (TC15: charger fault). Please take it from the tray."
  108.6 s  delivery  Order #1 delivered to table_7
```
</details>

---

## TC16 — Serving window 1 blocked -> window 2 (beside the takeaway queue)

| | |
|---|---|
| **Criterion** | Complex layout |
| **Setup** | A trolley is left on the staging area of serving window 1; the takeaway queue (3 customers + one arriving) stands next to window 2. |
| **Procedure** | Order table_4. |
| **Expected behaviour** | Approach 1 of the pickup is unreachable → the robot uses window 2 (between window 1 and the takeaway queue), never entering the queue zone; delivers. |
| **Pass criteria** | pickup at window 2 without entering the takeaway queue zone, order delivered, 0 collisions with the queueing customers |
| **Result** | ✅ PASS |
| **Measured** | `{"delivered": true, "window_2_used": true}` |
| **Duration / path** | 122 s simulated, 10.0 m driven |
| **Closest approach to a person** | 0.31 m (t = 61.24 s, human_waiter, robot state TO_PICKUP) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 87.9 %, final 87.8 %, learned consumption 1.324 Wh/m (×time acceleration) |
| **Orders** | #1 table_4 → delivered |

![tc16](test_results/tc16.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (9 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.596 m behind the lidar, at 0.1 deg
    0.1 s  order     Order #1 queued -> table_4 (TC16: window 1 blocked)
    4.3 s  dock      Undocked
    4.3 s  nav       No route to the kitchen pass right now (route or goal blocked) - re-checking for 45 s
   51.0 s  failure   Approach 1/2 of the kitchen pass unreachable
   97.7 s  nav       Serving table_4 from side 2 of 2 (4.1 m, shortest planned route)
  113.5 s  hri       Robot says: "Hello! Your order has arrived (TC16: window 1 blocked). Please take it from the tray."
  121.6 s  delivery  Order #1 delivered to table_4
```
</details>

---

## TC17 — VIP order jumps the queue

| | |
|---|---|
| **Criterion** | Delivery task management |
| **Setup** | Full world, battery 100 %. |
| **Procedure** | Order #1 table_7; once the robot has left the dock: orders #2 table_4, #3 table_5, then VIP order #4 table_6. |
| **Expected behaviour** | The VIP order is inserted before the normal orders still waiting: delivery sequence 1, 4, 2, 3 (the order in progress is never interrupted). |
| **Pass criteria** | VIP order #4 (sent last) is served right after the order in progress: delivery sequence 1, 4, 2, 3; all 4 orders delivered |
| **Result** | ✅ PASS |
| **Measured** | `{"queue_after_vip": [4, 2, 3], "delivery_sequence": [1, 4, 2, 3], "statuses": {"1": "delivered", "2": "delivered", "3": "delivered", "4": "delivered"}}` |
| **Duration / path** | 348 s simulated, 55.4 m driven |
| **Closest approach to a person** | 0.19 m (t = 243.32 s, human_waiter, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 58.9 %, final 58.8 %, learned consumption 1.267 Wh/m (×time acceleration) |
| **Orders** | #3 table_5 → delivered, #2 table_4 → delivered, #4 table_6 → delivered, #1 table_7 → delivered |

![tc17](test_results/tc17.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (20 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.596 m behind the lidar, at 0.1 deg
    0.0 s  order     Order #1 queued -> table_7 (TC17 order 1)
    4.3 s  dock      Undocked
    4.5 s  order     Order #2 queued -> table_4 (TC17 order 2)
    4.5 s  order     Order #3 queued -> table_5 (TC17 order 3)
    4.5 s  order     Order #4 queued -> table_6 [VIP] (TC17 VIP order 4)
   39.6 s  nav       Serving table_7 from side 2 of 2 (3.0 m, shortest planned route)
   48.0 s  hri       Robot says: "Hello! Your order has arrived (TC17 order 1). Please take it from the tray."
   56.0 s  delivery  Order #1 delivered to table_7
   96.5 s  nav       Serving table_6 from side 2 of 2 (12.7 m, shortest planned route)
  146.2 s  hri       Robot says: "Hello! Your order has arrived (TC17 VIP order 4). Please take it from the tray."
  154.3 s  delivery  Order #4 delivered to table_6
  231.0 s  nav       Serving table_4 from side 2 of 2 (5.2 m, shortest planned route)
  236.8 s  recovery  Nav2 recovery #1 on the way to table_4
  254.5 s  hri       Robot says: "Hello! Your order has arrived (TC17 order 2). Please take it from the tray."
  262.6 s  delivery  Order #2 delivered to table_4
  308.1 s  nav       Serving table_5 from side 2 of 2 (8.6 m, shortest planned route)
  340.1 s  hri       Robot says: "Hello! Your order has arrived (TC17 order 3). Please take it from the tray."
  348.2 s  delivery  Order #3 delivered to table_5
```
</details>

---

## TC18 — Order cancelled during the delivery, queue continues

| | |
|---|---|
| **Criterion** | Delivery task management |
| **Setup** | Full world, battery 100 %. |
| **Procedure** | Orders #1 table_3 and #2 table_7; "cancel" sent once the robot carries order #1 towards the table. |
| **Expected behaviour** | Navigation to table_3 stops, order #1 is reported cancelled (not delivered, not failed), the robot goes on with order #2, then returns to the dock. |
| **Pass criteria** | order #1 cancelled within a few seconds of the "cancel" command (never reported delivered), order #2 delivered, robot back on the dock |
| **Result** | ✅ PASS |
| **Measured** | `{"cancelled_quickly": true, "statuses": {"1": "cancelled", "2": "delivered"}, "delivery_sequence": [2]}` |
| **Duration / path** | 172 s simulated, 14.5 m driven |
| **Closest approach to a person** | 0.65 m (t = 104.22 s, human_waiter, robot state IDLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 81.1 %, final 83.6 %, learned consumption 1.217 Wh/m (×time acceleration) |
| **Orders** | #2 table_7 → delivered, #1 table_3 → cancelled (cancelled by operator) |

![tc18](test_results/tc18.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (13 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.597 m behind the lidar, at 0.1 deg
    0.0 s  order     Order #1 queued -> table_3 (TC18: order cancelled by the customer)
    0.0 s  order     Order #2 queued -> table_7 (TC18: next order)
    4.3 s  dock      Undocked
   39.6 s  nav       Serving table_3 from side 2 of 2 (14.0 m, shortest planned route)
   43.6 s  command   Operator command: cancel
   43.6 s  failure   Order #1 cancelled: cancelled by operator
   82.4 s  nav       Serving table_7 from side 2 of 2 (2.9 m, shortest planned route)
   91.2 s  hri       Robot says: "Hello! Your order has arrived (TC18: next order). Please take it from the tray."
   99.3 s  delivery  Order #2 delivered to table_7
  119.4 s  dock      Returning to dock: idle for 20s
  172.0 s  dock      Docked, charging contacts OK (83%), pad offset 2.6 cm (lidar)
```
</details>

---

## TC19 — Queue cleared by the operator during a delivery

| | |
|---|---|
| **Criterion** | Delivery task management |
| **Setup** | Full world, battery 100 %. |
| **Procedure** | Orders #1 table_4, #2 table_2, #3 table_3; "clear" sent while order #1 is on its way to the table. |
| **Expected behaviour** | The waiting orders #2 and #3 are cancelled at once; order #1 (already loaded) is still delivered; the robot returns to the dock. |
| **Pass criteria** | after "clear" the queue is empty, orders #2 and #3 are cancelled, the order in progress (#1) is still delivered, then the robot returns to the dock |
| **Result** | ✅ PASS |
| **Measured** | `{"queue_empty_after_clear": true, "statuses": {"1": "delivered", "2": "cancelled", "3": "cancelled"}}` |
| **Duration / path** | 159 s simulated, 16.4 m driven |
| **Closest approach to a person** | 0.32 m (t = 99.28 s, human_waiter, robot state RETURNING_TO_DOCK) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 82.3 %, final 84.7 %, learned consumption 1.359 Wh/m (×time acceleration) |
| **Orders** | #1 table_4 → delivered, #3 table_3 → cancelled, #2 table_2 → cancelled |

![tc19](test_results/tc19.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (13 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.596 m behind the lidar, at 0.1 deg
    0.0 s  order     Order #1 queued -> table_4 (TC19 order 1)
    0.0 s  order     Order #2 queued -> table_2 (TC19 order 2)
    0.0 s  order     Order #3 queued -> table_3 (TC19 order 3)
    4.2 s  dock      Undocked
   28.0 s  command   Operator command: clear
   40.2 s  nav       Serving table_4 from side 2 of 2 (5.2 m, shortest planned route)
   61.6 s  hri       Robot says: "Hello! Your order has arrived (TC19 order 1). Please take it from the tray."
   69.7 s  delivery  Order #1 delivered to table_4
   89.8 s  dock      Returning to dock: idle for 20s
  101.1 s  recovery  Nav2 recovery #1 on the way to the dock
  159.2 s  dock      Docked, charging contacts OK (84%), pad offset 14.0 cm (AMCL)
```
</details>

---

## TC20 — Order validation: unknown tables rejected, plain-text order served

| | |
|---|---|
| **Criterion** | Delivery task management |
| **Setup** | Full world, battery 100 %. |
| **Procedure** | Three messages on /delivery/order: "table_42" (unknown), {"table": "kitchen"} (not a table), "table_2" (plain text instead of JSON). |
| **Expected behaviour** | The two invalid orders are rejected with an event and never queued; the plain-text order is accepted and delivered. |
| **Pass criteria** | "table_42" and "kitchen" rejected with an event (never queued), the plain-text order "table_2" delivered, robot back on the dock |
| **Result** | ✅ PASS |
| **Measured** | `{"rejected": 2, "orders_created": 1, "statuses": {"1": "delivered"}}` |
| **Duration / path** | 177 s simulated, 25.0 m driven |
| **Closest approach to a person** | 0.62 m (t = 51.49 s, human_waiter, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 80.0 %, final 83.1 %, learned consumption 1.069 Wh/m (×time acceleration) |
| **Orders** | #1 table_2 → delivered |

![tc20](test_results/tc20.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (11 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.597 m behind the lidar, at -0.0 deg
    0.0 s  order     Rejected order for unknown table "table_42"
    0.0 s  order     Rejected order for unknown table "kitchen"
    0.0 s  order     Order #1 queued -> table_2 ()
    4.3 s  dock      Undocked
   40.0 s  nav       Serving table_2 from side 2 of 2 (9.8 m, shortest planned route)
   74.7 s  hri       Robot says: "Hello! Your order has arrived. Please take it from the tray."
   82.9 s  delivery  Order #1 delivered to table_2
  103.0 s  dock      Returning to dock: idle for 20s
  177.1 s  dock      Docked, charging contacts OK (83%), pad offset 2.6 cm (lidar)
```
</details>

---

## TC21 — Full service: every table (1-9) served once

| | |
|---|---|
| **Criterion** | Delivery task management / endurance |
| **Setup** | Full world, battery 100 %, energy flows ×25. |
| **Procedure** | 9 orders queued at once, one for every table (dining room 1-7, VIP lounge 8-9). |
| **Expected behaviour** | Every table is reached and served once, in FIFO order; the robot charges by itself when the next order needs it; SoC never reaches 0; back on the dock at the end. |
| **Pass criteria** | the 9 orders delivered in FIFO order (each table reached, VIP lounge included), charging handled autonomously, SoC never 0, robot back on the dock |
| **Result** | ✅ PASS |
| **Measured** | `{"delivered": 9, "of": 9, "delivery_sequence": [1, 2, 3, 4, 5, 6, 7, 8, 9], "charging_sessions": 3}` |
| **Duration / path** | 1711 s simulated, 274.5 m driven |
| **Closest approach to a person** | 0.14 m (t = 667.32 s, human_guest, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 25.0 %, final 50.7 %, learned consumption 1.206 Wh/m (×time acceleration) |
| **Orders** | #9 table_9 → delivered, #8 table_8 → delivered, #7 table_7 → delivered, #6 table_6 → delivered, #5 table_5 → delivered, #4 table_4 → delivered, #3 table_3 → delivered, #2 table_2 → delivered, #1 table_1 → delivered |

![tc21](test_results/tc21.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (71 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.595 m behind the lidar, at 0.2 deg
    0.0 s  order     Order #1 queued -> table_1 (TC21 table_1)
    0.0 s  order     Order #2 queued -> table_2 (TC21 table_2)
    0.0 s  order     Order #3 queued -> table_3 (TC21 table_3)
    0.0 s  order     Order #4 queued -> table_4 (TC21 table_4)
    0.0 s  order     Order #5 queued -> table_5 (TC21 table_5)
    0.0 s  order     Order #6 queued -> table_6 (TC21 table_6)
    0.0 s  order     Order #7 queued -> table_7 (TC21 table_7)
    0.0 s  order     Order #8 queued -> table_8 (TC21 table_8)
    0.0 s  order     Order #9 queued -> table_9 (TC21 table_9)
    4.3 s  dock      Undocked
   39.0 s  nav       Serving table_1 from side 2 of 2 (8.4 m, shortest planned route)
   70.6 s  hri       Robot says: "Hello! Your order has arrived (TC21 table_1). Please take it from the tray."
   78.7 s  delivery  Order #1 delivered to table_1
  137.3 s  nav       Serving table_2 from side 2 of 2 (9.8 m, shortest planned route)
  171.8 s  hri       Robot says: "Hello! Your order has arrived (TC21 table_2). Please take it from the tray."
  179.9 s  delivery  Order #2 delivered to table_2
  245.2 s  nav       Serving table_3 from side 2 of 2 (14.0 m, shortest planned route)
  292.6 s  hri       Robot says: "Hello! Your order has arrived (TC21 table_3). Please take it from the tray."
  300.6 s  delivery  Order #3 delivered to table_3
  373.1 s  nav       Serving table_4 from side 2 of 2 (5.0 m, shortest planned route)
  393.6 s  hri       Robot says: "Hello! Your order has arrived (TC21 table_4). Please take it from the tray."
  401.8 s  delivery  Order #4 delivered to table_4
  446.1 s  nav       Serving table_5 from side 2 of 2 (8.6 m, shortest planned route)
  477.9 s  hri       Robot says: "Hello! Your order has arrived (TC21 table_5). Please take it from the tray."
  485.9 s  delivery  Order #5 delivered to table_5
  486.2 s  dock      Returning to dock: energy: order #6 needs 50%, battery 42%
  567.6 s  dock      Docked, charging contacts OK (36%), pad offset 0.8 cm (lidar)
  568.0 s  battery   Charging to 53% (order #6 needs 41%), now 37%
  598.4 s  dock      Undocked
  634.6 s  nav       Serving table_6 from side 2 of 2 (12.7 m, shortest planned route)
  679.7 s  hri       Robot says: "Hello! Your order has arrived (TC21 table_6). Please take it from the tray."
  687.7 s  delivery  Order #6 delivered to table_6
  733.1 s  recovery  Nav2 recovery #1 on the way to the kitchen pass
  763.2 s  nav       Serving table_7 from side 2 of 2 (3.0 m, shortest planned route)
  766.2 s  recovery  Nav2 recovery #1 on the way to table_7
  774.0 s  hri       Robot says: "Hello! Your order has arrived (TC21 table_7). Please take it from the tray."
  782.2 s  delivery  Order #7 delivered to table_7
  782.8 s  dock      Returning to dock: energy: order #8 needs 79%, battery 31%
  840.4 s  dock      Docked, charging contacts OK (30%), pad offset 2.6 cm (lidar)
  841.1 s  battery   Charging to 94% (order #8 needs 78%), now 30%
  962.9 s  dock      Undocked
  998.4 s  nav       Serving table_8 from side 1 of 4 (31.6 m, shortest planned route)
 1116.2 s  hri       Robot says: "Hello! Your order has arrived (TC21 table_8). Please take it from the tray."
 1124.4 s  delivery  Order #8 delivered to table_8
 1125.0 s  dock      Returning to dock: energy: order #9 needs 109%, battery 74%
 1285.9 s  dock      Docked, charging contacts OK (59%), pad offset 5.9 cm (AMCL)
 1286.4 s  battery   Charging to 90% (order #9 needs 74%), now 59%
 1343.8 s  dock      Undocked
 1378.4 s  nav       Serving table_9 from side 3 of 3 (33.0 m, shortest planned route)
 1452.1 s  recovery  Nav2 recovery #1 on the way to table_9
 1452.8 s  recovery  Nav2 recovery #2 on the way to table_9
 1455.8 s  recovery  Nav2 recovery #3 on the way to table_9
 1462.4 s  recovery  Nav2 recovery #5 on the way to table_9
 1465.5 s  recovery  Nav2 recovery #6 on the way to table_9
 1467.1 s  recovery  Nav2 recovery #9 on the way to table_9
 1467.3 s  recovery  Nav2 recovery #10 on the way to table_9
 1505.7 s  hri       Robot says: "Hello! Your order has arrived (TC21 table_9). Please take it from the tray."
 1513.8 s  delivery  Order #9 delivered to table_9
 1533.8 s  dock      Returning to dock: idle for 20s
 1549.7 s  recovery  Nav2 recovery #1 on the way to the dock
 1555.3 s  recovery  Nav2 recovery #2 on the way to the dock
 1555.5 s  recovery  Nav2 recovery #3 on the way to the dock
 1560.1 s  recovery  Nav2 recovery #4 on the way to the dock
 1560.3 s  recovery  Nav2 recovery #5 on the way to the dock
 1565.7 s  nav       Blocked on the way to the dock (people around: 0) - waiting / replanning
 1565.9 s  nav       Moving again towards the dock
 1569.4 s  recovery  Nav2 recovery #6 on the way to the dock
 1575.4 s  recovery  Nav2 recovery #7 on the way to the dock
 1602.1 s  recovery  Nav2 recovery #8 on the way to the dock
```
</details>

---

## TC22 — Order arrives while charging -> leaves without waiting for a full charge

| | |
|---|---|
| **Criterion** | Delivery task management / battery |
| **Setup** | Battery forced to 40 % on the dock: the robot is idle-charging towards 60 %. |
| **Procedure** | Order table_6 while it charges. |
| **Expected behaviour** | Idle charging does not delay the service: the energy for the whole order + reserve is available, so the robot leaves at once (below 60 %) and delivers. |
| **Pass criteria** | robot charging at 40 %: the order interrupts idle charging (no wait until 60 %) because the energy for the order + reserve is available; order delivered |
| **Result** | ✅ PASS |
| **Measured** | `{"was_charging": true, "soc_at_departure": 53.43, "need_soc": null, "delivered": true}` |
| **Duration / path** | 116 s simulated, 16.4 m driven |
| **Closest approach to a person** | 0.16 m (t = 106.64 s, human_wc, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 40.3 %, final 41.9 %, learned consumption 1.292 Wh/m (×time acceleration) |
| **Orders** | #1 table_6 → delivered |

![tc22](test_results/tc22.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (9 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.598 m behind the lidar, at 0.1 deg
    1.0 s  order     Order #1 queued -> table_6 (TC22: order during charging)
    1.1 s  battery   Charging to 53% (order #1 needs 42%), now 41%
   25.0 s  dock      Undocked
   60.0 s  nav       Serving table_6 from side 2 of 2 (12.3 m, shortest planned route)
   87.0 s  recovery  Nav2 recovery #1 on the way to table_6
  108.4 s  hri       Robot says: "Hello! Your order has arrived (TC22: order during charging). Please take it from the tray."
  116.4 s  delivery  Order #1 delivered to table_6
```
</details>

---

## TC23 — Tables served from the closest side (shortest route)

| | |
|---|---|
| **Criterion** | Delivery task management / navigation |
| **Setup** | Full world, battery 100 %. |
| **Procedure** | Orders table_1, table_4, table_7 (each table has a north and a south approach pose). |
| **Expected behaviour** | Before driving to a table the robot plans a route to every approach pose and serves from the shortest one: the north side, facing the kitchen, instead of a detour around the table. |
| **Pass criteria** | tables 1, 4 and 7 served from their north side (towards the kitchen), no detour around the table, all 3 orders delivered |
| **Result** | ✅ PASS |
| **Measured** | `{"sides_used": {"table_1": 2, "table_4": 2, "table_7": 2}, "statuses": {"1": "delivered", "2": "delivered", "3": "delivered"}, "path_length_m": 39.1}` |
| **Duration / path** | 309 s simulated, 39.1 m driven |
| **Closest approach to a person** | 0.36 m (t = 147.77 s, human_waiter, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 65.0 %, final 69.2 %, learned consumption 1.167 Wh/m (×time acceleration) |
| **Orders** | #3 table_7 → delivered, #2 table_4 → delivered, #1 table_1 → delivered |

![tc23](test_results/tc23.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (16 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.595 m behind the lidar, at 0.1 deg
    0.0 s  order     Order #1 queued -> table_1 (TC23 table_1)
    0.0 s  order     Order #2 queued -> table_4 (TC23 table_4)
    0.0 s  order     Order #3 queued -> table_7 (TC23 table_7)
    4.4 s  dock      Undocked
   38.7 s  nav       Serving table_1 from side 2 of 2 (8.5 m, shortest planned route)
   69.8 s  hri       Robot says: "Hello! Your order has arrived (TC23 table_1). Please take it from the tray."
   77.9 s  delivery  Order #1 delivered to table_1
  137.1 s  nav       Serving table_4 from side 2 of 2 (5.1 m, shortest planned route)
  161.9 s  hri       Robot says: "Hello! Your order has arrived (TC23 table_4). Please take it from the tray."
  169.9 s  delivery  Order #2 delivered to table_4
  217.0 s  nav       Serving table_7 from side 2 of 2 (3.0 m, shortest planned route)
  225.5 s  hri       Robot says: "Hello! Your order has arrived (TC23 table_7). Please take it from the tray."
  233.6 s  delivery  Order #3 delivered to table_7
  253.7 s  dock      Returning to dock: idle for 20s
```
</details>
