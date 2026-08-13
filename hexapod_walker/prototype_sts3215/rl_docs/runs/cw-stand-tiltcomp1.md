# cw-stand-tiltcomp1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-13T05:09:41+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-footlow2-tip1

**wandb_id**: mnccyhdw

**hardware_ready**: False

**hypothesis**: Teach the standing robot to actively LEVEL itself when its body is tilted, instead of contentedly holding the lean — the deployed stance candidate stands on the real robot with a persistent ~8deg lean, and the last attempt to train this (cw-stand-footlow2-tip1, tipped spawns with the tilt-BLIND anchor) taught exactly the opposite: hold the same pose while leaning. This arm is tip1's identical recipe plus ONE change: the new tip-aware hold anchor (train.bc_anchor_tilt_comp=1.0, snapshot 1efc816) replaces the constant settled-pose target with the IK pose that counter-rotates the measured lean (soft deadband 1.5deg, cap 6deg = measured action-space expressibility boundary) — a proportional posture-feedback teacher, so the tipped spawns now supervise LEVELING instead of tolerance. Prediction-if-true: on the forced-8deg-tip matched-parent probe the policy both holds height AND levels (tip1 held height 12/12 but settled 0/12 with tail 7.2deg; parent hard1 settles 11/12 but misses height), with nominal untipped retention staying at hard1's clean band. Prediction-if-false: nominal retention breaks or a foot parks again — meaning ANY extra hold-mode gradient pressure on this lineage reopens the park regardless of the teacher's correctness, implicating the anchored-hold equilibrium itself, and the hardware lean fix must move off the tipped-spawn exposure route entirely (operator design discussion). Strongest alternative: the policy levels on the probe but via a new cheat (e.g. narrowing its support polygon) — caught by the valid_plant/footprint spec and video.

**gate**: PASS if (1) forced-8deg-tip probe (dr-scale 0.0, dr.tipped_start_prob=1.0, deg=8,8, hold mode, n=12 det + 12 sto, MATCHED frozen-parent footlow2-hard1 rerun under the identical injection+seeds): roll settled/recovered >=10/12 det with tail med <=3deg (tip1: 0/12, tail 7.2deg) AND valid_plant >=9/12 det (parent's own probe baseline was 5/12-and-below); (2) zero park: all six feet duty >=0.5 in every probe and nominal episode; (3) nominal untipped retention (dr-scale 0, standard draw): hold det 6/6 valid_plant with all-six duty >=0.9 and level tail <=1deg, rise+lower det+sto >=10/12 each, drag/roll_tail not worse than hard1's band, no flag-leg on video. FAIL if tilt-tolerance reproduces (settled <9/12 or nominal hold ends tilted >3deg) OR any retention park/fall regression vs hard1 — then the tipped-exposure route is closed even with a correct teacher; consequence: no dose retry, escalate the lean to an operator design discussion.

**verdict**: FAIL — clause 1+2 fire: even with a CORRECT tip-aware teacher (anchor counter-rotates measured lean), 2M of 50% tipped spawns reproduces tip1's tilt-tolerance. Matched forced-8deg probe (frozen hard1 baseline on file, seed 0): child holds height (valid_plant det 12/12, h_err 0.9mm vs parent 0/12) but NEVER levels — roll_class leaning 12/12 det+sto, tail med 5.75deg (bar <=3, parent recovers to 1.45 in 11/12), and parks a foot every episode (min duty 0.01-0.03 det). Residual lean sits at the 6deg comp cap: the policy exploits comp authority to satisfy height spec while staying tilted. Nominal retention milder than tip1 (no falls, hold det 6/6 tail 0.4deg) but degraded vs hard1: hold det min-duty 0.69 vs 0.95/bar 0.9, slip 0.597 vs 0.136. Per pre-registered consequence: tipped-exposure training route CLOSED even with a correct teacher — two teachers (tilt-blind, tilt-aware) converged on hold-height-stay-tilted, implicating the incentive (nothing prices residual lean in hold income), not teacher quality. No dose retry; hardware ~8deg lean escalates to an operator design discussion.

