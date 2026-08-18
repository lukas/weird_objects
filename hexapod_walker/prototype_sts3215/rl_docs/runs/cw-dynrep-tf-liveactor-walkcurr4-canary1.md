# cw-dynrep-tf-liveactor-walkcurr4-canary1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-18T15:47:18+00:00

**pod**: hexapod-mjx-train-4

**steps**: 4000000

**parent**: cw-dep-bcgait1-hard1

**hypothesis**: Let the walking robot keep learning its physics intuition while it practices: instead of freezing the dynamics transformer that conditions its actor and critic, this canary trains that transformer LIVE on the walking the robot is actually doing (with 25% retained-corpus rehearsal and guarded boundary snapshots), while warm-starting the proven tall-gait champion's own actor AND critic across the history-stack widening. Operator-ordered (fb 20260818T153944Z) after every frozen-transformer walkcurr4 arm failed one layer deep: canB re-crouched, canC stood tall but would not walk, bridge2's fresh critic never reached usable EV in 2M. The bet: a transplanted proven critic + a live-updating predictive representation gives a usable value signal from step 0, so the imported gait survives B0 and starts climbing the sustained-joystick ladder. Prediction-if-true: pre-PPO B0 cert passes (zero falls, prog>=0.5); through 4M the source B0 is retained with zero falls, the curriculum promotes past B0, the actor transformer residual/gate goes nonzero, online heldout prediction improves on the pretrained encoder, and at least one guarded snapshot is ACCEPTED. Prediction-if-false: every snapshot is rejected (dead live-learning) or falls/rollbacks recur despite the live model - pointing at the ladder or live-replay distribution, not rollback mechanics.

**gate**: PRE-PPO fail-closed: cert-at-init B0 zero falls AND cmd_prog_frac>=0.5. IN-RUN fail-closed (trainer-enforced): CPU physics/Torch, encoder hash mismatch, action mismatch vs pred-gate-action-kl semantics, empty live replay, or absent predictor updates. 4M PASS requires ALL (operator bars, fb 20260818T153944Z): (1) source B0 retained ZERO falls across all cert rounds; (2) curriculum progresses (>=1 promotion past B0); (3) nonzero actor transformer residual; (4) online heldout prediction IMPROVES vs start-of-run pretrained reference; (5) >=1 guarded snapshot ACCEPTED (versions/rejections logged). PASS => operator-ordered 40M successor with this recipe. FAIL => NO 40M; name the failed bar.

**verdict**: INFRA CRASH at boot, not a code/behavior verdict: all 24 sharded-env host workers died with SIGBUS (exitcode -7) in mjx_sharded_vec_env shared-memory setup because hexapod-mjx-train-4 has the 64M Docker-default /dev/shm (pods 5/7/9/11 have the intended 4G tmpfs; bridge2 ran this same hist16 family fine on train-11's 4G). The predictive-live/hist16 stack's shm buffers exceed 64M. No PPO step ran. Retrying once per DEAD protocol as -r1 on train-11 (4G shm) after code sync. PLACEMENT CONSTRAINT: do not place hist16/predictive-live sharded-env runs on 64M-shm pods (train-0/4/6/8 measured 64M today).

**failed_reason**: process died; log tail:
xitcode=-7
  worker 15: exitcode=-7
  worker 16: exitcode=-7
  worker 17: exitcode=-7
  worker 18: exitcode=-7
  worker 19: exitcode=-7
  worker 20: exitcode=-7
  worker 21: exitcode=-7
  worker 22: exitcode=-7
  worker 23: exitcode=-7
[1;34mwandb[0m: 
[1;34mwandb[0m: 🚀 View run [33mcw-dynrep-tf-liveactor-walkcurr4-canary1[0m at: [34mhttps://wandb.ai/l2k2/hexapod-balance/runs/nocabfni[0m
[1;34mwandb[0m: Find logs at: [1;35mwandb/run-20260818_154757-nocabfni/logs[0m
[canary] regression auto-stop armed (stop after 3 consecutive full-group failures)
/usr/local/lib/python3.11/multiprocessing/resource_tracker.py:254: UserWarning: resource_tracker: There appear to be 39 leaked shared_memory objects to clean up at shutdown
  warnings.warn('resource_tracker: There appear to be %d '


