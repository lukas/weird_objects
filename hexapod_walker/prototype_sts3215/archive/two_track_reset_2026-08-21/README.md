# Two-track reset — 2026-08-21

The operator reset the campaign to exactly two goals:

1. `joystick` — RL from the scripted programmatic gait to joystick
   control (60 s command-following in sim, zero falls, teacher-band
   slip).
2. `amp` — the from-scratch AMP program (`rl_docs/AMP_LOCOMOTION.md`,
   no Isaac Lab).

Everything here was active process not aligned to those goals:

- `rl_docs/` — retired topic docs (dynrep, multitask, turn, rise,
  takeoff, recover deploy, wishlist, agent handoff, dated reports).
- `tracks/` — the seven pre-reset track dirs (hw, arch, nobc, quad,
  turn, multitask, dynrep) with their final STATUS.md files.
- `make_rl_review_bundle.sh` — the external-review bundle generator.

Facts these docs established that still matter were folded into
`CURRENT_TRUTHS.md` and the two new track docs. Nothing here is
deleted from git history; consult for historical questions only.
