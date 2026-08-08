# Video review protocol

Numbers lie by omission — the tripod stance, the raise collapse, and the
skating "walk" all scored fine on scalars and were only caught on video.
Every gate decision (promote a checkpoint, pass a DR rung, pick the next
experiment) requires a video review recorded against this checklist.

## Artifacts

- **Training (W&B `video/rollout`):** each entry is now a multi-episode
  *reel* — `--video-episodes` (default 4) episodes cycling through every
  active goal mode, one continuous MP4 with per-episode title cards and
  telemetry overlay. Watch first and latest to see the arc of the run.
- **Gate eval (`eval_checkpoint.py` output dir):**
  - `contact_sheet.png` — one film-strip row per mode (first deterministic
    episode each). The unit of review: one image answers "what is this
    checkpoint actually doing."
  - `<mode>_<det|sto>_<k>.mp4` + `.png` strip — full videos for anything
    the sheet makes suspicious.

## Workflow (per checkpoint)

1. Open `contact_sheet.png`. Walk the checklist below for each row.
2. For any row that fails or looks odd, watch that mode's full MP4.
3. Record the verdict (pass / fail + one line per anomaly) in the round
   notes and the W&B run notes. A checkpoint that scores well but LOOKS
   wrong is a failed checkpoint — fix the metric before training on it.

## Per-mode checklist

**All modes**
- [ ] No jitter/vibration at rest (physics chatter = dead-band regression).
- [ ] All six feet flat on the pads; no foot floating or clipping ground.
- [ ] No servo pinned at a joint limit for the whole episode.
- [ ] Overlay telemetry consistent with the motion seen (refs vs act).

**hold / lean / track**
- [ ] All six feet loaded (watch for the 3-leg tripod: a foot that never
      touches down for the entire episode).
- [ ] Attitude follows the ref with no oscillation or slow drift.

**raise**
- [ ] Lift is smooth and small (10–30 mm), no overshoot-then-sag.
- [ ] Feet stay planted — the lift comes from legs, not a lunge.

**rise (check each start kind: flat / bridge / crouch)**
- [ ] Flat start: legs curl under body BEFORE the push — no stilt pose
      (knees near-straight lifting body tall on extended legs).
- [ ] No belly-slam or bounce at any stage.
- [ ] Final pose is the plant stance, held quietly to episode end
      (not "parked low" 20+ mm short).

**lower**
- [ ] Controlled descent — body does not free-fall the last third.
- [ ] Ends flat and still, legs extended, no residual fighting.

**walk**
- [ ] Feet actually lift and swing (skating = feet never break contact
      while the body drags — the classic failure).
- [ ] Motion direction matches the commanded velocity vector.
- [ ] No leg doing double duty (one leg swinging every cycle while
      another never moves).
- [ ] Body height roughly constant; no progressive sag over the episode.

**Current/thermal (report.json, alongside the video)**
- [ ] `cur_max_a` peaks < ~2.7 A and `hot_s` (time > 1.5 A) not
      concentrated on one servo; `cur_leg_imbalance` ≲ 1.5.
