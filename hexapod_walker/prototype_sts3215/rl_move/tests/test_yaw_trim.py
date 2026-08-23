"""Unit tests for `rl_move.sim.yaw_trim.update_trim` (pure numeric, no sim)."""
from rl_move.sim.yaw_trim import update_trim


def test_zero_command_is_a_noop():
    # No turn commanded: whatever measured noise exists, the scale must
    # not move (stop/heading segments must never be trimmed).
    assert update_trim(1.0, measured_wz=0.37, cmd_wz=0.0) == 1.0
    assert update_trim(0.73, measured_wz=-0.1, cmd_wz=1e-9) == 0.73


def test_perfect_tracking_is_a_noop():
    scale = update_trim(1.0, measured_wz=0.2, cmd_wz=0.2)
    assert scale == 1.0


def test_overshoot_reduces_scale():
    # measured 33% faster than commanded -> next-period scale must drop
    # below 1.0 so the next commanded omega is smaller.
    scale = update_trim(1.0, measured_wz=0.2 * 1.33, cmd_wz=0.2)
    assert scale < 1.0


def test_undershoot_raises_scale():
    scale = update_trim(1.0, measured_wz=0.2 * 0.7, cmd_wz=0.2)
    assert scale > 1.0


def test_sign_consistent_for_negative_commands():
    # A CCW turn (cmd_wz<0) overshooting (more negative measured) must
    # also trim the scale down, exactly mirroring the positive case.
    pos = update_trim(1.0, measured_wz=0.2 * 1.33, cmd_wz=0.2)
    neg = update_trim(1.0, measured_wz=-0.2 * 1.33, cmd_wz=-0.2)
    assert abs(pos - neg) < 1e-12


def test_clamped_within_bounds():
    lo_scale = update_trim(0.45, measured_wz=10.0, cmd_wz=0.2,
                           kp=5.0, lo=0.4, hi=1.6)
    assert lo_scale == 0.4
    hi_scale = update_trim(1.55, measured_wz=-10.0, cmd_wz=0.2,
                           kp=5.0, lo=0.4, hi=1.6)
    assert hi_scale == 1.6


def test_converges_from_synthetic_gain_error():
    # Synthetic plant: whatever omega we command, the body actually
    # achieves `actual_gain * commanded` (mimics the mu0.8 overshoot:
    # actual_gain ~1.33 measured on the cpg track 08-23). Closing the
    # loop over repeated windows should drive the EFFECTIVE output
    # (cmd_wz * trim_scale * actual_gain) back toward cmd_wz.
    cmd_wz = 0.2
    actual_gain = 1.33
    scale = 1.0
    for _ in range(20):
        commanded_this_period = cmd_wz * scale
        measured = commanded_this_period * actual_gain
        scale = update_trim(scale, measured, cmd_wz)
    effective = cmd_wz * scale * actual_gain
    assert abs(effective / cmd_wz - 1.0) < 0.05


def test_converges_from_synthetic_undershoot():
    cmd_wz = -0.2
    actual_gain = 0.75
    scale = 1.0
    for _ in range(20):
        commanded_this_period = cmd_wz * scale
        measured = commanded_this_period * actual_gain
        scale = update_trim(scale, measured, cmd_wz)
    effective = cmd_wz * scale * actual_gain
    assert abs(effective / cmd_wz - 1.0) < 0.05
