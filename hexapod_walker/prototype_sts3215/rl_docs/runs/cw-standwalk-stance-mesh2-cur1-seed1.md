# cw-standwalk-stance-mesh2-cur1-seed1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T05:58:06+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-standwalk-stance-mesh1-rr1

**wandb_id**: h8nqxuk2

**hypothesis**: Seed 1 of the cw-standwalk-stance-mesh2-cur1 3-seed batch (see that entry for the full plain-English rationale + probe evidence): mesh/100Hz from-scratch stance recipe with the probe-chosen near-saturation current price (k_current_hot=1.0 @ 2.0 A) + termination horizon cost (term3 cap 60) that flips the measured ground-grind exploit negative in every mode while honest behavior keeps 90%+ of its return. Distinct-seed robustness arm; same predictions as seed 0.

**gate**: Read jointly with cw-standwalk-stance-mesh2-cur1 (seed 0) at 20M: stance panel rise/hold/lower n>=12 det+sto DR-0 + own-DR(0.2), zero falls/tips, rise valid plant >=5/6, lower posture-strict >=5/6, hold quiet 6/6. 2-3/3 healthy = recipe robust; 0-1/3 = named fork, not more seeds.

**verdict**: Second seed of the cur1 realigned-pricing batch, identical joint reading to cw-standwalk-stance-mesh2-cur1 (this cycle): pricing kills the current-grind (no more uniform over_current-everywhere signature) but the policy still fails every stance mode honestly. All 36 DR-0 + 36 own-DR episodes 0/6 across hold/rise/lower det+sto -- hold TERM tilt_roll, rise TERM tilt_pitch, lower TERM over_current (three distinct failure modes, matching cur1's mixed signature). Reward flat the whole 20M run (quarters -318/-533/-311/-297, dips then returns to ~where it started -- no sustained rise). Video (contact sheet) matches cur1: hold starts planted then slides/lists sideways (drag 267-428mm), rise topples onto its side within the first couple seconds instead of pushing up, lower ends dragged/splayed. This is flat-reward+flat-task+wrong-video = genuine FAIL, not misalignment (the reward-rising grind signature from mesh1 is gone). JOINT READ (per the pre-registered gate): 0/2 of this cycle's two seeds are healthy, which already caps the 3-seed batch at <=1/3 regardless of cw-standwalk-stance-mesh2-cur1-seed2's own result (concurrent/uncredited) -- the '0-1/3 = escalate the named fork, not more seeds' branch is live now. Do not launch a same-recipe seed3 or seed4; the next lever is teacher signal, a longer from-scratch budget, or a hold-only-first split (see cur1's verdict for the full fork list). Evidence: logs/ckpt_eval/cw_standwalk_stance_mesh2_cur1_seed1_{gate,owncfg}/, W&B h8nqxuk2.

