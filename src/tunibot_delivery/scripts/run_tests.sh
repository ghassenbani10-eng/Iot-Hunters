#!/usr/bin/env bash
# Run the Phase 2 test scenarios, each on a fresh simulation stack (headless).
#
#   src/tunibot_delivery/scripts/run_tests.sh                 # all scenarios
#   src/tunibot_delivery/scripts/run_tests.sh tc02 tc09       # a selection
#   GUI=true RESULTS=~/results src/tunibot_delivery/scripts/run_tests.sh tc03
#   CONTROLLER=rpp src/tunibot_delivery/scripts/run_tests.sh tc05   # rpp | mppi (default)
#   REPEAT=5 src/tunibot_delivery/scripts/run_tests.sh tc02 tc03     # statistics: $RESULTS/run_1..5/
#
# Results: $RESULTS/<scenario>.{json,csv,png} + _events.json, logs in $RESULTS/logs.
# Then: ros2 run tunibot_delivery make_test_report $RESULTS
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
RESULTS="${RESULTS:-$HOME/tunibot_test_results}"
GUI="${GUI:-false}"
CONTROLLER="${CONTROLLER:-mppi}"
SCENARIOS=("$@")
[ ${#SCENARIOS[@]} -eq 0 ] && SCENARIOS=(tc01 tc02 tc03 tc04 tc05 tc06 tc07 tc08 tc09 tc10 tc11 tc12 tc13 tc14 tc15 tc16 tc17 tc18 tc19 tc20 tc21 tc22 tc23 tc24)
mkdir -p "$RESULTS/logs"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
source "$WS/src/tunibot_delivery/scripts/nvidia_gl.sh"   # GPU rendering in distrobox (NVIDIA host)
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"   # isolate the tests from other ROS graphs
[ -f /opt/ros/humble/lib/librmw_cyclonedds_cpp.so ] && export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

stop_stack() {
  # anchored patterns: only processes started from ROS / this workspace
  for sig in INT KILL; do
    pkill -$sig -x gzserver; pkill -$sig -x gzclient
    pkill -$sig -f '^/usr/bin/python3 /opt/ros/humble/bin/ros2 (launch|run)'
    pkill -$sig -f '^/opt/ros/humble/lib/'
    pkill -$sig -f '^/usr/bin/python3 /opt/ros/humble/lib/'
    pkill -$sig -f "^/usr/bin/python3 $WS/install/"
    pkill -$sig -f "^$WS/install/"
    [ $sig = INT ] && sleep 4
  done
  sleep 1
}

REPEAT="${REPEAT:-1}"
OUT_BASE="$RESULTS"
for rep in $(seq 1 "$REPEAT"); do
RESULTS="$OUT_BASE"; [ "$REPEAT" -gt 1 ] && RESULTS="$OUT_BASE/run_$rep" && mkdir -p "$RESULTS/logs"
for sc in "${SCENARIOS[@]}"; do
  echo "================ $sc (run $rep/$REPEAT) ================"
  stop_stack >/dev/null 2>&1
  ACC=25.0; SOC=100.0
  case "$sc" in
    tc10) ACC=25.0 ;;     # endurance (starts at 45 %, see scenario)
  esac
  L="$RESULTS/logs/$sc"
  ros2 launch tunibot_delivery sim.launch.py gui:=$GUI > "$L-sim.log" 2>&1 &
  sleep 15
  ros2 launch tunibot_delivery navigation.launch.py rviz:=$GUI controller:=$CONTROLLER > "$L-nav.log" 2>&1 &
  sleep 12
  ros2 launch tunibot_delivery delivery.launch.py time_acceleration:=$ACC initial_soc:=$SOC port:=8080 > "$L-delivery.log" 2>&1 &
  timeout 4200 ros2 run tunibot_delivery scenario_runner --ros-args -p use_sim_time:=true \
      -p scenario:=$sc -p out_dir:="$RESULTS" 2>&1 | tee "$L-runner.log" | grep -E 'RESULT|EVENT|CONTACT|passed'
done
done
RESULTS="$OUT_BASE"
stop_stack >/dev/null 2>&1
echo "Results in $RESULTS"
