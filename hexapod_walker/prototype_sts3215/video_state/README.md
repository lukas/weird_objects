# video_state — vision checks for bench sessions

Two tools that work off the bench camera (`bench_blast_*` sessions):

1. **`robot_in_frame.py`** — fast classical-CV classifier: is the robot
   fully inside the camera frame? Run it over a session's `camera.mp4`
   (or point the bench script at it live) to catch bad framing before a
   whole run is filmed with the robot half out of shot.
2. **`train_state.py` / `infer_state.py`** — a small CNN (0.55M params)
   that estimates robot state from a single video frame: body roll,
   pitch, and all 18 joint angles. Labels come from the encoder/IMU
   traces, so this is a vision-only *independent* estimate of the state
   the robot believes it is in — useful when diagnosing tips.

Everything runs in the repo `.venv` (cv2 + torch already there).

## Robot-in-frame classifier

```sh
python robot_in_frame.py <session>/camera.mp4 [--annotate out.mp4] [--jsonl out.jsonl]
```

Prints %full / %partial / %not_visible plus the time spans of any bad
stretches, and exits 1 if more than `--fail-threshold` (default 5%) of
frames are not fully in frame. ~700 fps on the Mac, no ML.

How it works (`detect.py`): the robot is the only object in the scene
that is a dense mix of red AND blue printed parts. HSV-mask red and
blue, dilate both, and score connected regions of red∧blue overlap —
battery wires (red only) and floor-sheet tape (blue only) don't score.
The bbox comes from the red|blue pixels around the winning overlap blob;
"partial" means the bbox touches the frame edge.

Validated on all 9 sessions of 2026-08-11: flags the operator blocking
the lens and the robot walking to the frame edge; no false alarms from
the wires / tape measure / potting-soil bag.

## State model

```sh
python build_dataset.py   # video frames + synced trace labels -> data/state_dataset.npz
python gen_synth.py       # MuJoCo-rendered synthetic pretraining set (~3 min)
python train_state.py --synth data/synth_dataset.npz   # ~5 min on MPS
python infer_state.py --walk 193306/tip1-r1 --out /tmp/state_timeline.png
```

Synthetic pretraining (`gen_synth.py`): the MuJoCo model is posed from
harvested trace rows (65%) and uniform joint-range samples (35%), colored
with a randomized red/blue palette matching the real print, rendered
offscreen, and composited via segmentation mask onto random patches of
real bench video. Pretrain on 12k of those, then fine-tune on the real
frames at 0.3x LR. Known compositing gaps: no cast shadows, no wiring,
background perspective is arbitrary.

Dataset: frames from every session that has BOTH `camera.mp4` and
per-tick trace CSVs, using the exact `camera.t0_unix` ↔ `t_start_unix`
sync. Walk sync is refined at most ±0.35 s (must stay under half a gait
period — wider search aliases on the periodic gait and corrupts the
sync). Stand episodes are anchored on the "learned stand up starting"
event + 2.4 s (measured safe-zero acquisition lag) and gated on motion
correlation; badly-correlated stands are dropped rather than trained on.
As of 2026-08-12: 2364 frames, 19 walks + 2 stands.

### Honest accuracy (2026-08-11 data, whole-session 193306 holdout)

MAE in degrees on the held-out session, best config per stage:

| target | v1 (real only) | final (synth pretrain + stands + bbox aux) |
|---|---|---|
| roll | 7.1° | 6.7° |
| pitch | 2.8° | 3.2° |
| coxa | 15.2° | 13.3° |
| femur | 14.5° | 13.8° |
| knee | 18.4° | 16.0° |
| all | 14.9° | 13.4° |

Timelines (infer_state.py) are the right way to judge it: the model
detects tips (roll rises with the IMU), tracks pitch, and follows
coxa/femur/knee direction while underestimating magnitude and lagging
the fastest transients (15 fps blur + residual sync error live exactly
there). It is a coarse gross-pose estimate, not an encoder replacement —
the encoders are already logged and synced to the video; the value is an
independent cross-check (wrong-logical-zero detection, tip diagnosis
when the trace is suspect).

Config notes (all measured on this val set, differences of ~1° are
within noise): synthetic pretraining with real-only fine-tune helps;
mixing synthetic frames INTO the fine-tune hurts (16.1° overall);
160 px input, real-only norm stats, and 2x knee loss weight were each
tried and did not measurably beat this recipe. Error analysis: knee
error is NOT dominated by sync noise (static frames score worse than
moving ones — it's pose-coverage, mostly the post-trip collapse poses).

To improve: more videoed sessions (data is the bottleneck — 21 usable
episodes, one bench, ~2 camera placements), 30+ fps capture, shadows and
wiring in the synthetic compositing, or per-frame render-and-compare
optimization against the MuJoCo model instead of a feed-forward net.

Split: `--val-session <substring>` holds out every episode whose name
contains the substring (default `193306` = whole-session holdout; use
e.g. `tip1-r1` for a per-session walk holdout).
