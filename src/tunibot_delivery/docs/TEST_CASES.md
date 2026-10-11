# TuniBot — Phase 2 test cases

Every test case is an automated scenario (`tunibot_delivery/scenario_runner.py`) run on a **fresh simulation stack** (`scripts/run_tests.sh`): Gazebo (headless) + Nav2 + delivery stack. Metrics come from **Gazebo ground truth**, not from the robot's own estimates:

* **min clearance** — smallest distance between the robot footprint (0.54 × 0.50 m at its true pose) and any person body (0.22 m radius); ≤ 0 would be a collision,
* **contacts** — bumper (contact sensor on the robot body, spine and lidar) touching anything,
* order outcomes, durations, path length, battery level (min / final), full event log.

A test fails if its functional criterion fails, if there is any contact, or if the minimum clearance to a person is below its threshold.

## Summary

| ID | Criterion | Scenario | Result | Duration (s) | Min clearance to people (m) | Min TTC (s) | Intimate / personal space (s) | Contacts | Min SoC (%) |
|---|---|---|---|---|---|---|---|---|---|
| TC01 | Dynamic obstacles | Delivery with people walking around (ambient crowd) | ✅ PASS | 212 | 0.63 | 2.78 | 0.0 / 9.4 | 0 | 75.6 |
| TC02 | Dynamic obstacles | Person walking head-on towards the robot | ✅ PASS | 91 | 0.48 | 0.65 | 0.0 / 12.4 | 0 | 89.2 |
| TC03 | Dynamic obstacles | Person crossing just in front of the robot | ✅ PASS | 89 | 0.56 | 0.83 | 0.0 / 25.7 | 0 | 89.6 |
| TC04 | Dynamic obstacles / recovery | Doorway blocked by a standing person (25 s) | ✅ PASS | 173 | 0.41 | 4.50 | 3.2 / 42.4 | 0 | 78.9 |
| TC05 | Complex layout | VIP lounge: doorways + narrow corridor + L-turn | ✅ PASS | 606 | 0.06 | 0.36 | 10.2 / 107.1 | 0 | 32.6 |
| TC06 | Complex layout / recovery | Table approach blocked by a trolley -> alternate approach pose | ✅ PASS | 232 | 0.56 | 3.24 | 0.0 / 26.8 | 0 | 73.4 |
| TC07 | Complex layout / recovery | Unreachable table (corridor blocked): graceful failure, queue continues | ✅ PASS | 424 | 0.24 | 0.98 | 5.9 / 64.7 | 0 | 54.6 |
| TC08 | Battery | Low battery when the order arrives -> charge first | ✅ PASS | 182 | 1.27 | 6.13 | 0.0 / 0.0 | 0 | 18.3 |
| TC09 | Battery | Battery runs low mid-mission -> return to dock in time, resume later | ✅ PASS | 524 | 0.34 | 3.85 | 1.7 / 43.6 | 0 | 9.1 |
| TC10 | Battery / endurance | Endurance: 6 orders, battery cycles handled autonomously | ✅ PASS | 1393 | 0.08 | 0.20 | 14.6 / 191.4 | 0 | 22.8 |
| TC11 | State management | Emergency stop and resume | ✅ PASS | 115 | 0.93 | 9.86 | 0.0 / 4.1 | 0 | 86.8 |
| TC12 | Auto-docking | Docking precision (3 cycles) and charger contact | ✅ PASS | 500 | 0.44 | 0.98 | 0.3 / 19.7 | 0 | 52.6 |
| TC13 | Dynamic obstacles | Group of two people walking head-on | ✅ PASS | 100 | 0.47 | 0.69 | 0.0 / 10.4 | 0 | 88.0 |
| TC14 | Dynamic obstacles | Person stops suddenly in front of the robot | ✅ PASS | 91 | 0.38 | 0.76 | 8.3 / 27.1 | 0 | 89.2 |
| TC15 | Battery & state | Charger fault on the dock: detection and recovery | ✅ PASS | 148 | 1.01 | 7.21 | 0.0 / 2.6 | 0 | 16.4 |
| TC16 | Complex layout | Serving window 1 blocked -> window 2 (beside the takeaway queue) | ✅ PASS | 134 | 0.48 | 6.16 | 0.0 / 18.8 | 0 | 86.3 |

**16 / 16 test cases passed.**

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
| **Duration / path** | 212 s simulated, 36.8 m driven |
| **Closest approach to a person** | 0.63 m (t = 51.42 s, human_waiter, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 75.6 %, final 79.8 %, learned consumption 1.035 Wh/m (×time acceleration) |
| **Orders** | #1 table_6 → delivered |

![tc01](test_results/tc01.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (8 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.597 m behind the lidar, at -0.0 deg
    0.0 s  order     Order #1 queued -> table_6 (TC01: 2 coffees)
    4.4 s  dock      Undocked
   91.9 s  hri       Robot says: "Hello! Your order has arrived (TC01: 2 coffees). Please take it from the tray."
  100.0 s  delivery  Order #1 delivered to table_6
  120.2 s  dock      Returning to dock: idle for 20s
  211.5 s  dock      Docked, charging contacts OK (80%), pad offset 2.2 cm (lidar)
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
| **Duration / path** | 91 s simulated, 14.7 m driven |
| **Closest approach to a person** | 0.48 m (t = 52.92 s, human_extra_1, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 89.2 %, final 89.2 %, learned consumption 1.178 Wh/m (×time acceleration) |
| **Orders** | #1 table_5 → delivered |

![tc02](test_results/tc02.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (6 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.598 m behind the lidar, at -0.0 deg
    0.0 s  order     Order #1 queued -> table_5 (TC02: 1 tea)
    4.2 s  dock      Undocked
   82.6 s  hri       Robot says: "Hello! Your order has arrived (TC02: 1 tea). Please take it from the tray."
   90.7 s  delivery  Order #1 delivered to table_5
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
| **Duration / path** | 89 s simulated, 11.6 m driven |
| **Closest approach to a person** | 0.56 m (t = 66.89 s, human_extra_1, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 89.6 %, final 89.5 %, learned consumption 1.458 Wh/m (×time acceleration) |
| **Orders** | #1 table_4 → delivered |

![tc03](test_results/tc03.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (6 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.594 m behind the lidar, at 0.1 deg
    0.0 s  order     Order #1 queued -> table_4 (TC03: 3 waters)
    4.3 s  dock      Undocked
   80.9 s  hri       Robot says: "Hello! Your order has arrived (TC03: 3 waters). Please take it from the tray."
   89.0 s  delivery  Order #1 delivered to table_4
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
| **Duration / path** | 173 s simulated, 33.7 m driven |
| **Closest approach to a person** | 0.41 m (t = 80.54 s, human_extra_1, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 78.9 %, final 78.8 %, learned consumption 1.355 Wh/m (×time acceleration) |
| **Orders** | #1 table_8 → delivered |

![tc04](test_results/tc04.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (7 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.594 m behind the lidar, at 0.1 deg
    0.0 s  order     Order #1 queued -> table_8 (TC04: VIP order, doorway blocked)
    4.3 s  dock      Undocked
   60.0 s  recovery  Nav2 recovery #1 on the way to table_8
  165.0 s  hri       Robot says: "Hello! Your order has arrived (TC04: VIP order, doorway blocked). Please take it from the tray."
  173.0 s  delivery  Order #1 delivered to table_8
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
| **Duration / path** | 606 s simulated, 70.9 m driven |
| **Closest approach to a person** | 0.06 m (t = 518.98 s, human_takeaway, robot state RETURNING_TO_DOCK) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 32.6 %, final 36.2 %, learned consumption 1.293 Wh/m (×time acceleration) |
| **Orders** | #1 table_8 → delivered |

![tc05](test_results/tc05.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (47 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.601 m behind the lidar, at -0.1 deg
    0.0 s  order     Order #1 queued -> table_8 [VIP] (TC05: VIP champagne)
    4.2 s  dock      Undocked
   60.1 s  recovery  Nav2 recovery #1 on the way to table_8
   60.5 s  recovery  Nav2 recovery #2 on the way to table_8
   63.6 s  recovery  Nav2 recovery #3 on the way to table_8
  131.0 s  recovery  Nav2 recovery #4 on the way to table_8
  173.2 s  hri       Robot says: "Hello! Your order has arrived (TC05: VIP champagne). Please take it from the tray."
  181.3 s  delivery  Order #1 delivered to table_8
  201.4 s  dock      Returning to dock: idle for 20s
  220.4 s  nav       Blocked on the way to the dock (people around: 1) - waiting / replanning
  228.5 s  recovery  Nav2 recovery #1 on the way to the dock
  243.6 s  recovery  Nav2 recovery #3 on the way to the dock
  258.6 s  recovery  Nav2 recovery #4 on the way to the dock
  261.1 s  hri       Robot says: "Excuse me, may I pass? Thank you!"
  267.5 s  recovery  Nav2 recovery #5 on the way to the dock
  269.3 s  hri       Robot says: "Excuse me, may I pass? Thank you!"
  273.9 s  recovery  Nav2 recovery #6 on the way to the dock
  278.5 s  nav       Moving again towards the dock
  288.6 s  nav       Blocked on the way to the dock (people around: 1) - waiting / replanning
  294.9 s  recovery  Nav2 recovery #7 on the way to the dock
  310.0 s  recovery  Nav2 recovery #8 on the way to the dock
  315.0 s  hri       Robot says: "Excuse me, may I pass? Thank you!"
  323.2 s  hri       Robot says: "Excuse me, may I pass? Thank you!"
  330.3 s  recovery  Nav2 recovery #9 on the way to the dock
  345.2 s  recovery  Nav2 recovery #10 on the way to the dock
  348.3 s  recovery  Nav2 recovery #11 on the way to the dock
  363.6 s  recovery  Nav2 recovery #12 on the way to the dock
  369.0 s  hri       Robot says: "Excuse me, may I pass? Thank you!"
  377.1 s  hri       Robot says: "Excuse me, may I pass? Thank you!"
  378.5 s  recovery  Nav2 recovery #14 on the way to the dock
  393.8 s  recovery  Nav2 recovery #15 on the way to the dock
  396.9 s  failure   Giving up on the dock: no arrival within 190 s
  397.0 s  failure   Navigation to the dock failed (approach 1/1, attempt 1)
  405.3 s  nav       Blocked on the way to the dock (people around: 1) - waiting / replanning
  415.3 s  recovery  Nav2 recovery #1 on the way to the dock
  422.7 s  hri       Robot says: "Excuse me, may I pass? Thank you!"
  430.4 s  recovery  Nav2 recovery #2 on the way to the dock
  430.8 s  hri       Robot says: "Excuse me, may I pass? Thank you!"
  433.9 s  nav       Moving again towards the dock
  438.9 s  nav       Blocked on the way to the dock (people around: 1) - waiting / replanning
  443.0 s  nav       Moving again towards the dock
  447.4 s  recovery  Nav2 recovery #3 on the way to the dock
  463.5 s  recovery  Nav2 recovery #4 on the way to the dock
  489.0 s  recovery  Nav2 recovery #5 on the way to the dock
  606.4 s  dock      Docked, charging contacts OK (36%), pad offset 3.7 cm (lidar)
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
| **Duration / path** | 232 s simulated, 30.5 m driven |
| **Closest approach to a person** | 0.56 m (t = 123.65 s, human_guest, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 73.4 %, final 73.3 %, learned consumption 1.402 Wh/m (×time acceleration) |
| **Orders** | #1 table_5 → delivered |

![tc06](test_results/tc06.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (18 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.600 m behind the lidar, at -0.1 deg
    0.1 s  order     Order #1 queued -> table_5 (TC06: 2 sodas)
    4.4 s  dock      Undocked
   64.1 s  recovery  Nav2 recovery #1 on the way to table_5
   84.3 s  recovery  Nav2 recovery #2 on the way to table_5
   86.4 s  recovery  Nav2 recovery #3 on the way to table_5
  105.7 s  recovery  Nav2 recovery #4 on the way to table_5
  113.6 s  recovery  Nav2 recovery #5 on the way to table_5
  127.8 s  recovery  Nav2 recovery #6 on the way to table_5
  135.5 s  recovery  Nav2 recovery #7 on the way to table_5
  146.3 s  failure   Giving up on table_5: no arrival within 107 s
  146.4 s  failure   Navigation to table_5 failed (approach 1/2, attempt 1)
  149.5 s  nav       No route to table_5 right now (route or goal blocked) - re-checking for 45 s
  196.2 s  failure   Approach 1/2 of table_5 unreachable
  196.7 s  recovery  Nav2 recovery #1 on the way to table_5
  223.4 s  hri       Robot says: "Hello! Your order has arrived (TC06: 2 sodas). Please take it from the tray."
  231.5 s  delivery  Order #1 delivered to table_5
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
| **Duration / path** | 424 s simulated, 52.1 m driven |
| **Closest approach to a person** | 0.24 m (t = 86.14 s, human_takeaway, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 54.6 %, final 57.5 %, learned consumption 1.351 Wh/m (×time acceleration) |
| **Orders** | #2 table_7 → delivered, #1 table_9 → failed (table_9 unreachable) |

![tc07](test_results/tc07.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (32 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.600 m behind the lidar, at -0.0 deg
    0.2 s  order     Order #1 queued -> table_9 (TC07: blocked VIP order)
    0.2 s  order     Order #2 queued -> table_7 (TC07: next order)
    4.5 s  dock      Undocked
   87.0 s  recovery  Nav2 recovery #1 on the way to table_9
   87.3 s  recovery  Nav2 recovery #2 on the way to table_9
   90.5 s  recovery  Nav2 recovery #4 on the way to table_9
   90.6 s  recovery  Nav2 recovery #6 on the way to table_9
   91.2 s  recovery  Nav2 recovery #7 on the way to table_9
   91.6 s  recovery  Nav2 recovery #8 on the way to table_9
   92.4 s  nav       Blocked on the way to table_9 (people around: 0) - waiting / replanning
   93.0 s  nav       Moving again towards table_9
   94.3 s  recovery  Nav2 recovery #9 on the way to table_9
   94.5 s  recovery  Nav2 recovery #10 on the way to table_9
   99.6 s  recovery  Nav2 recovery #11 on the way to table_9
   99.8 s  recovery  Nav2 recovery #12 on the way to table_9
  100.0 s  nav       Blocked on the way to table_9 (people around: 0) - waiting / replanning
  102.9 s  recovery  Nav2 recovery #14 on the way to table_9
  103.1 s  recovery  Nav2 recovery #16 on the way to table_9
  103.7 s  recovery  Nav2 recovery #17 on the way to table_9
  104.0 s  failure   Navigation to table_9 failed (approach 1/2, attempt 1)
  107.1 s  nav       No route to table_9 right now (route or goal blocked) - re-checking for 45 s
  152.7 s  failure   Approach 1/2 of table_9 unreachable
  152.7 s  nav       No route to table_9 right now (route or goal blocked) - re-checking for 45 s
  198.4 s  failure   Approach 2/2 of table_9 unreachable
  198.4 s  failure   Order #1 failed: table_9 unreachable
  223.9 s  recovery  Nav2 recovery #1 on the way to the kitchen pass
  314.4 s  hri       Robot says: "Hello! Your order has arrived (TC07: next order). Please take it from the tray."
  322.6 s  delivery  Order #2 delivered to table_7
  342.7 s  dock      Returning to dock: idle for 20s
  423.9 s  dock      Docked, charging contacts OK (57%), pad offset 1.8 cm (lidar)
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
| **Measured** | `{"charged_first": true, "soc_at_departure": 66.92, "need_soc": null}` |
| **Duration / path** | 182 s simulated, 21.3 m driven |
| **Closest approach to a person** | 1.27 m (t = 104.21 s, human_expo, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 18.3 %, final 54.1 %, learned consumption 1.074 Wh/m (×time acceleration) |
| **Orders** | #1 table_3 → delivered |

![tc08](test_results/tc08.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (7 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.599 m behind the lidar, at -0.0 deg
    2.0 s  order     Order #1 queued -> table_3 (TC08: low battery at order time)
    2.3 s  battery   Charging to 66% (order #1 needs 53%), now 19%
   80.1 s  dock      Undocked
  173.8 s  hri       Robot says: "Hello! Your order has arrived (TC08: low battery at order time). Please take it from the tray."
  182.0 s  delivery  Order #1 delivered to table_3
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
| **Measured** | `{"interrupted": true, "docked": true, "soc_at_dock": 11.09, "delivered_after_charge": true}` |
| **Duration / path** | 524 s simulated, 42.5 m driven |
| **Closest approach to a person** | 0.34 m (t = 395.4 s, human_guest, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 9.1 %, final 76.2 %, learned consumption 1.313 Wh/m (×time acceleration) |
| **Orders** | #1 table_9 → delivered (energy to reach the dock: 7.4 Wh needed, 6.9 Wh left above the reserve) |

![tc09](test_results/tc09.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (17 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.596 m behind the lidar, at 0.1 deg
    0.0 s  order     Order #1 queued -> table_9 (TC09: VIP delivery, battery failing)
    4.3 s  dock      Undocked
   42.1 s  battery   Order #1 interrupted (energy to reach the dock: 7.4 Wh needed, 6.9 Wh left above the reserve); re-queued, going to charge
   42.1 s  dock      Returning to dock: energy to reach the dock: 7.4 Wh needed, 6.9 Wh left above the reserve
   91.2 s  dock      Docked, charging contacts OK (11%), pad offset 2.3 cm (AMCL)
   91.5 s  battery   Charging to 100% (order #1 needs 83%), now 11%
  327.5 s  dock      Undocked
  448.9 s  recovery  Nav2 recovery #1 on the way to table_9
  479.3 s  nav       Blocked on the way to table_9 (people around: 0) - waiting / replanning
  488.3 s  recovery  Nav2 recovery #2 on the way to table_9
  489.6 s  recovery  Nav2 recovery #3 on the way to table_9
  492.8 s  recovery  Nav2 recovery #4 on the way to table_9
  496.5 s  nav       Moving again towards table_9
  515.4 s  hri       Robot says: "Hello! Your order has arrived (TC09: VIP delivery, battery failing). Please take it from the tray."
  523.5 s  delivery  Order #1 delivered to table_9
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
| **Duration / path** | 1393 s simulated, 199.1 m driven |
| **Closest approach to a person** | 0.08 m (t = 1271.45 s, human_guest, robot state TO_PICKUP) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 22.8 %, final 31.8 %, learned consumption 1.258 Wh/m (×time acceleration) |
| **Orders** | #6 table_7 → delivered, #5 table_3 → delivered, #4 table_6 → delivered, #3 table_8 → delivered, #2 table_1 → delivered, #1 table_4 → delivered |

![tc10](test_results/tc10.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (56 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.595 m behind the lidar, at 0.1 deg
    0.0 s  order     Order #1 queued -> table_4 (TC10 order 1)
    0.0 s  order     Order #2 queued -> table_1 (TC10 order 2)
    0.0 s  order     Order #3 queued -> table_8 (TC10 order 3)
    0.0 s  order     Order #4 queued -> table_6 (TC10 order 4)
    0.0 s  order     Order #5 queued -> table_3 (TC10 order 5)
    0.0 s  order     Order #6 queued -> table_7 (TC10 order 6)
    4.3 s  dock      Undocked
   67.6 s  nav       Blocked on the way to table_4 (people around: 0) - waiting / replanning
   76.5 s  recovery  Nav2 recovery #1 on the way to table_4
   91.6 s  recovery  Nav2 recovery #2 on the way to table_4
   94.7 s  recovery  Nav2 recovery #3 on the way to table_4
  109.9 s  recovery  Nav2 recovery #4 on the way to table_4
  111.2 s  nav       Moving again towards table_4
  120.3 s  hri       Robot says: "Hello! Your order has arrived (TC10 order 1). Please take it from the tray."
  128.4 s  delivery  Order #1 delivered to table_4
  128.7 s  dock      Returning to dock: energy: order #2 needs 51%, battery 31%
  202.5 s  dock      Docked, charging contacts OK (26%), pad offset 1.3 cm (lidar)
  202.8 s  battery   Charging to 56% (order #2 needs 45%), now 26%
  255.3 s  dock      Undocked
  331.6 s  hri       Robot says: "Hello! Your order has arrived (TC10 order 2). Please take it from the tray."
  339.8 s  delivery  Order #2 delivered to table_1
  340.1 s  dock      Returning to dock: energy: order #3 needs 95%, battery 47%
  430.9 s  dock      Docked, charging contacts OK (39%), pad offset 2.6 cm (lidar)
  431.2 s  battery   Charging to 97% (order #3 needs 80%), now 40%
  562.9 s  dock      Undocked
  610.6 s  recovery  Nav2 recovery #1 on the way to table_8
  623.9 s  recovery  Nav2 recovery #2 on the way to table_8
  624.0 s  recovery  Nav2 recovery #4 on the way to table_8
  625.4 s  recovery  Nav2 recovery #5 on the way to table_8
  625.8 s  recovery  Nav2 recovery #6 on the way to table_8
  719.2 s  hri       Robot says: "Hello! Your order has arrived (TC10 order 3). Please take it from the tray."
  727.3 s  delivery  Order #3 delivered to table_8
  727.6 s  dock      Returning to dock: energy: order #4 needs 79%, battery 77%
  778.7 s  recovery  Nav2 recovery #1 on the way to the dock
  784.7 s  recovery  Nav2 recovery #2 on the way to the dock
  784.9 s  recovery  Nav2 recovery #3 on the way to the dock
  789.1 s  nav       Blocked on the way to the dock (people around: 0) - waiting / replanning
  790.7 s  nav       Moving again towards the dock
  888.8 s  dock      Docked, charging contacts OK (60%), pad offset 8.2 cm (AMCL)
  893.2 s  dock      Undocked
  977.1 s  hri       Robot says: "Hello! Your order has arrived (TC10 order 4). Please take it from the tray."
  985.2 s  delivery  Order #4 delivered to table_6
  985.5 s  dock      Returning to dock: energy: order #5 needs 63%, battery 49%
 1038.4 s  recovery  Nav2 recovery #1 on the way to the dock
 1092.7 s  dock      Docked, charging contacts OK (39%), pad offset 2.0 cm (lidar)
 1093.1 s  battery   Charging to 61% (order #5 needs 49%), now 39%
 1132.6 s  dock      Undocked
 1168.3 s  nav       No route to table_3 right now (route or goal blocked) - re-checking for 45 s
 1214.3 s  failure   Approach 1/2 of table_3 unreachable
 1258.7 s  hri       Robot says: "Hello! Your order has arrived (TC10 order 5). Please take it from the tray."
 1266.8 s  delivery  Order #5 delivered to table_3
 1317.6 s  recovery  Nav2 recovery #1 on the way to the kitchen pass
 1385.1 s  hri       Robot says: "Hello! Your order has arrived (TC10 order 6). Please take it from the tray."
 1393.1 s  delivery  Order #6 delivered to table_7
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
| **Duration / path** | 115 s simulated, 18.0 m driven |
| **Closest approach to a person** | 0.93 m (t = 60.72 s, human_waiter, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 86.8 %, final 86.8 %, learned consumption 1.119 Wh/m (×time acceleration) |
| **Orders** | #1 table_2 → delivered (emergency stop) |

![tc11](test_results/tc11.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (10 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.595 m behind the lidar, at 0.1 deg
    0.0 s  order     Order #1 queued -> table_2 (TC11: e-stop test)
    4.3 s  dock      Undocked
   31.4 s  command   Operator command: estop
   31.6 s  estop     EMERGENCY STOP - motors held at zero until "resume"
   39.1 s  command   Operator command: resume
   39.1 s  command   Resumed after emergency stop
  107.0 s  hri       Robot says: "Hello! Your order has arrived (TC11: e-stop test). Please take it from the tray."
  115.2 s  delivery  Order #1 delivered to table_2
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
| **Measured** | `{"dockings": [{"dx": 0.009, "dy": 0.013, "dyaw_deg": -0.1, "charging": true}, {"dx": 0.006, "dy": -0.015, "dyaw_deg": -0.6, "charging": true}, {"dx": 0.017, "dy": -0.036, "dyaw_deg": -1.5, "charging": true}]}` |
| **Duration / path** | 500 s simulated, 76.8 m driven |
| **Closest approach to a person** | 0.44 m (t = 109.62 s, human_waiter, robot state RETURNING_TO_DOCK) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 52.6 %, final 62.4 %, learned consumption 1.092 Wh/m (×time acceleration) |
| **Orders** | #3 table_1 → delivered, #2 table_4 → delivered, #1 table_7 → delivered |

![tc12](test_results/tc12.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (23 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.598 m behind the lidar, at -0.0 deg
    0.0 s  order     Order #1 queued -> table_7 (TC12 docking 1)
    4.2 s  dock      Undocked
   70.5 s  hri       Robot says: "Hello! Your order has arrived (TC12 docking 1). Please take it from the tray."
   78.6 s  delivery  Order #1 delivered to table_7
   79.0 s  command   Operator command: dock
   79.1 s  dock      Returning to dock: operator request
  156.9 s  dock      Docked, charging contacts OK (84%), pad offset 2.9 cm (AMCL)
  160.5 s  order     Order #2 queued -> table_4 (TC12 docking 2)
  164.7 s  dock      Undocked
  230.3 s  hri       Robot says: "Hello! Your order has arrived (TC12 docking 2). Please take it from the tray."
  238.3 s  delivery  Order #2 delivered to table_4
  238.5 s  command   Operator command: dock
  238.5 s  dock      Returning to dock: operator request
  308.2 s  dock      Docked, charging contacts OK (71%), pad offset 0.7 cm (lidar)
  311.5 s  order     Order #3 queued -> table_1 (TC12 docking 3)
  315.8 s  dock      Undocked
  394.9 s  hri       Robot says: "Hello! Your order has arrived (TC12 docking 3). Please take it from the tray."
  403.0 s  delivery  Order #3 delivered to table_1
  403.5 s  command   Operator command: dock
  403.5 s  dock      Returning to dock: operator request
  489.0 s  dock      Docked, charging contacts OK (55%), pad offset 2.5 cm (lidar)
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
| **Duration / path** | 100 s simulated, 18.6 m driven |
| **Closest approach to a person** | 0.47 m (t = 47.77 s, human_extra_2, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 88.0 %, final 88.0 %, learned consumption 1.082 Wh/m (×time acceleration) |
| **Orders** | #1 table_6 → delivered |

![tc13](test_results/tc13.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (6 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.597 m behind the lidar, at 0.0 deg
    0.0 s  order     Order #1 queued -> table_6 (TC13: group)
    4.2 s  dock      Undocked
   91.2 s  hri       Robot says: "Hello! Your order has arrived (TC13: group). Please take it from the tray."
   99.4 s  delivery  Order #1 delivered to table_6
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
| **Duration / path** | 91 s simulated, 14.5 m driven |
| **Closest approach to a person** | 0.38 m (t = 62.22 s, human_extra_1, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 89.2 %, final 89.2 %, learned consumption 1.149 Wh/m (×time acceleration) |
| **Orders** | #1 table_5 → delivered |

![tc14](test_results/tc14.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (7 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.594 m behind the lidar, at 0.2 deg
    0.0 s  order     Order #1 queued -> table_5 (TC14: sudden stop)
    4.3 s  dock      Undocked
   44.6 s  recovery  Nav2 recovery #1 on the way to table_5
   82.6 s  hri       Robot says: "Hello! Your order has arrived (TC14: sudden stop). Please take it from the tray."
   90.7 s  delivery  Order #1 delivered to table_5
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
| **Duration / path** | 148 s simulated, 11.8 m driven |
| **Closest approach to a person** | 1.01 m (t = 104.17 s, human_waiter, robot state TO_TABLE) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 16.4 %, final 36.8 %, learned consumption 1.232 Wh/m (×time acceleration) |
| **Orders** | #1 table_7 → delivered |

![tc15](test_results/tc15.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (9 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.600 m behind the lidar, at -0.0 deg
    2.0 s  order     Order #1 queued -> table_7 (TC15: charger fault)
    2.2 s  battery   Charging to 45% (order #1 needs 35%), now 18%
   12.2 s  error     Charger fault: on the pad but no charging current for 10 s - staff notified, orders wait until it is fixed
   22.8 s  battery   Charger OK again, charging resumed (16%)
   72.8 s  dock      Undocked
  140.0 s  hri       Robot says: "Hello! Your order has arrived (TC15: charger fault). Please take it from the tray."
  148.1 s  delivery  Order #1 delivered to table_7
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
| **Duration / path** | 134 s simulated, 13.3 m driven |
| **Closest approach to a person** | 0.48 m (t = 58.56 s, human_waiter, robot state TO_PICKUP) |
| **Contacts (bumper)** | 0 |
| **Battery** | min 86.3 %, final 86.2 %, learned consumption 1.295 Wh/m (×time acceleration) |
| **Orders** | #1 table_4 → delivered |

![tc16](test_results/tc16.png)

*Left: ground-truth trajectory over the SLAM map, coloured by state; people (red); red star = closest approach to a person. Right: battery SoC with the state timeline and events; robot–person clearance.*

<details><summary>Event log (8 events)</summary>

```
    0.0 s  system    Nav2 active, robot on the dock
    0.0 s  dock      Docking reference learned from the lidar: wall 1.602 m behind the lidar, at -0.2 deg
    0.2 s  order     Order #1 queued -> table_4 (TC16: window 1 blocked)
    4.4 s  dock      Undocked
    4.4 s  nav       No route to the kitchen pass right now (route or goal blocked) - re-checking for 45 s
   50.7 s  failure   Approach 1/2 of the kitchen pass unreachable
  125.5 s  hri       Robot says: "Hello! Your order has arrived (TC16: window 1 blocked). Please take it from the tray."
  133.5 s  delivery  Order #1 delivered to table_4
```
</details>
