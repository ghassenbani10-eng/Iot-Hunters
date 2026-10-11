"""Battery model: open-circuit voltage curve."""
import pytest

rclpy = pytest.importorskip('rclpy')
from tunibot_delivery.battery_simulator import OCV_TABLE, ocv_cell  # noqa: E402


def test_ocv_monotonic_and_bounded():
    prev = 0.0
    for i in range(101):
        v = ocv_cell(i / 100)
        assert v >= prev
        prev = v
    assert ocv_cell(-1.0) == OCV_TABLE[0][1]
    assert ocv_cell(2.0) == OCV_TABLE[-1][1]


def test_ocv_interpolates_table():
    for s, v in OCV_TABLE:
        assert abs(ocv_cell(s) - v) < 1e-9
    assert 3.74 < ocv_cell(0.55) < 3.81
