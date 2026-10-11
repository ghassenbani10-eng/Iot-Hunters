"""Social navigation metrics of the scenario runner."""
import pytest

pytest.importorskip('gazebo_msgs')
from tunibot_delivery.scenario_runner import social_metrics  # noqa: E402


def run(person_vx, clearance):
    s = []
    for i in range(50):
        t = i * 0.1
        s.append({'t': t, 'x': 0.5 * t, 'y': 0.0, 'yaw': 0.0, 'clearance': clearance,
                  'p_a': '%.3f;0.0' % (5.0 + person_vx * t)})
    return social_metrics(s)


def test_head_on_ttc():
    m = run(-0.5, 2.0)        # closing at 1 m/s from 5 m, radii 0.59 m -> TTC ~ 4.4 s at start
    assert 0.0 <= m['min_ttc_s'] < 4.5
    assert m['intimate_s'] == 0.0 and m['personal_s'] == 0.0
    assert m['path_irregularity_rad_per_m'] == 0.0


def test_receding_person_has_no_ttc():
    m = run(+1.0, 0.3)        # walking away faster than the robot
    assert m['min_ttc_s'] == 10.0
    assert m['intimate_s'] > 4.0 and m['personal_s'] > 4.0
