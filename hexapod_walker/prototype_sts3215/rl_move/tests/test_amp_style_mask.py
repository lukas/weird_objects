"""Tests for the AMP obs_style feature mask (--amp-style-mask-dims).

The mask blinds the discriminator to named feature channels (e.g. dim
38 = base_angular_velocity z / yaw rate) by zeroing them on BOTH the
real (MotionLibrary, before mean/std fit) and fake (wrapper,
apply_style_mask) sides. Default = empty = bit-exact legacy.
"""
import numpy as np
import pytest

from rl_move.sim.amp_discriminator import (DEFAULT_LIBRARY, MotionLibrary,
                                           apply_style_mask)

pytestmark = pytest.mark.skipif(
    not DEFAULT_LIBRARY.exists(), reason="motion library npz not present")


def test_default_off_bit_exact():
    raw = np.asarray(np.load(DEFAULT_LIBRARY, allow_pickle=True)["obs_style"],
                     dtype=np.float32)
    lib = MotionLibrary()
    assert lib.mask_dims == ()
    np.testing.assert_array_equal(lib.obs_style, raw)


def test_masked_library_zeros_dim_and_guards_std():
    lib0 = MotionLibrary()
    lib = MotionLibrary(mask_dims=(38,))
    assert lib.mask_dims == (38,)
    assert np.all(lib.obs_style[:, 38] == 0.0)
    assert lib.mean[38] == 0.0
    assert lib.std[38] == 1.0  # constant-dim guard
    # every other column untouched
    keep = [i for i in range(lib.obs_style.shape[1]) if i != 38]
    np.testing.assert_array_equal(lib.obs_style[:, keep],
                                  lib0.obs_style[:, keep])


def test_masked_transitions_carry_zero():
    lib = MotionLibrary(mask_dims=(36, 37, 38))
    s_t, s_t1 = lib.sample_real_transitions(64, np.random.default_rng(0))
    for s in (s_t, s_t1):
        assert np.all(np.asarray(s)[:, 36:39] == 0.0)


def test_apply_style_mask_empty_is_noop():
    x = np.random.default_rng(1).normal(size=(5, 60)).astype(np.float32)
    ref = x.copy()
    out = apply_style_mask(x, ())
    np.testing.assert_array_equal(out, ref)
    out2 = apply_style_mask(x, None)
    np.testing.assert_array_equal(out2, ref)


def test_apply_style_mask_zeros_named_dims_only():
    x = np.random.default_rng(2).normal(size=(5, 60)).astype(np.float32)
    ref = x.copy()
    apply_style_mask(x, (38,))
    assert np.all(x[:, 38] == 0.0)
    keep = [i for i in range(60) if i != 38]
    np.testing.assert_array_equal(x[:, keep], ref[:, keep])


def test_out_of_range_mask_raises():
    with pytest.raises(ValueError):
        MotionLibrary(mask_dims=(60,))
    with pytest.raises(ValueError):
        MotionLibrary(mask_dims=(-1,))
