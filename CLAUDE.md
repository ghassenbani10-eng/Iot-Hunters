# CSTAM-TUNIBOT — Phase 1 (deadline: 1 Oct 2026, late = -5 pts)

Challenge spec: `/home/bani/CSTAM-BOOK.pdf` pages 10-12 (TuniBot "Autonomous Service Robot for Indoor Delivery").
Phase 1 (50 pts): SLAM 15 · Autonomous navigation 15 · Delivery task management 10 · Auto-docking 10.
Deliverables: source repo (ROS 2 ws, nodes, UI backend), high-level architecture diagram, short video of movement + mapping.

## Stack
ROS 2 Humble, Gazebo Classic, Cartographer, Nav2, Python (ament_python). Package: `src/tunibot_delivery` (see its README.md).
User environment: Ubuntu, no passwordless sudo — the user must run `sudo` commands themselves.

## State (2026-10-11)
- Phase 1 + Phase 2 complete, merged to `main` and pushed (GitHub: ghassenbani10-eng/Iot-Hunters).
  Phase 2 doc: src/tunibot_delivery/docs/PHASE2.md · tests: docs/TEST_CASES.md (12/12) · video: docs/media/.
- Layout (user requirement): staff-only kitchen serving through 2 windows, dock in the dining-room corner,
  VIP lounge via 0.9 m doors + 1.05 m corridor. Frames: robot spawns at world (-9.0, 2.2): map = world + (9.0, -2.2).
- World is generated: scripts/build_world.py (then make_keepout_mask.py). Map: auto_mapping.launch.py.
- Controller: MPPI by default (navigation.launch.py controller:=mppi|rpp).
- Tests: scripts/run_tests.sh [tcNN ...] (fresh headless stack each, ROS_DOMAIN_ID 77) → make_test_report.
- Video: demo_recorder node (Gazebo cameras + live map → MP4), run with a scenario_runner scenario.
- Known open issue: rare Nav2 controller stale-TF / "plan has 0 poses" (PHASE2.md §6).
- Next: Phase 3 (UI/UX & documentation: technical report) and Phase 4 (pitch); bonus items in CSTAM-BOOK p.11.
