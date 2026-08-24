import unittest
from unittest import mock
from rl_move.sim.mjx_sharded_vec_env import (_shm_layout, _shm_layout_bytes,
                                              _check_shm_budget)


class TestShmBudget(unittest.TestCase):
    def _layout(self, B, n_obs, dr=False, seq=False):
        dr_shapes = {"body_mass": (26,)} if dr else None
        return _shm_layout(B, 18, n_obs, 25, 24, 36, tag="t",
                            dr_shapes=dr_shapes, seq=seq)

    def test_bytes_scale_linearly_in_B(self):
        l1 = self._layout(100, 1152)
        l2 = self._layout(200, 1152)
        self.assertAlmostEqual(_shm_layout_bytes(l2) / _shm_layout_bytes(l1),
                                2.0, places=3)

    def test_bytes_scale_with_history(self):
        small = self._layout(100, 1152)
        big = self._layout(100, 4608)
        self.assertGreater(_shm_layout_bytes(big), _shm_layout_bytes(small))

    def test_passes_when_under_budget(self):
        layout = self._layout(10, 72)
        with mock.patch("shutil.disk_usage",
                        return_value=(64_000_000, 0, 64_000_000)):
            _check_shm_budget(layout, 10)  # should not raise

    def test_raises_with_actionable_message_when_over_budget(self):
        layout = self._layout(3072, 4608, dr=True, seq=True)
        with mock.patch("shutil.disk_usage",
                        return_value=(65_536_000, 0, 65_536_000)):
            with self.assertRaises(RuntimeError) as ctx:
                _check_shm_budget(layout, 3072)
        msg = str(ctx.exception)
        self.assertIn("shm", msg.lower())
        self.assertIn("--n-envs", msg)

    def test_skips_when_disk_usage_unavailable(self):
        layout = self._layout(3072, 4608, dr=True, seq=True)
        with mock.patch("shutil.disk_usage", side_effect=OSError("nope")):
            _check_shm_budget(layout, 3072)  # should not raise


if __name__ == "__main__":
    unittest.main()
