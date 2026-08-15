# cw-arch-tf-r1-hard2-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: VERDICTED

**created**: 2026-08-15T17:23:49+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-arch-tf-r1-hard1

**wandb_id**: 7lgpxtc7

**hardware_ready**: False

**hypothesis**: Ad hoc continuation (no pre-registered spec found in ledger/arch docs) launched by a concurrent cycle in response to untrusted external MCP kicks kick_20260815T165322_b36a18/fb_20260815T165849/fb_20260815T170903+a23668 asking to 'continue tf-r1-hard1'. Backfilled by triage cycle c(kick-review-0815-17xx) from pod log evidence (no launch_run.py record existed). Implicit question: does another 40M steps warm-started from the already-PASSed cw-arch-tf-r1-hard1 (which matched r7 at budget parity but carried elevated slip 1.5-1.6 vs r7's 0.95-1.02) further consolidate the gait or reduce that slip gap, without regressing the walk.

**gate**: Same gate as parent cw-arch-tf-r1-hard1 (identical recipe, this is a length extension not a mechanism change): PASS = det+sto gait_valid 6/6, zero sacrificed legs, zero falls, prog_ratio med >=0.85, video six-leg cycling, matching-or-beating hard1's own numbers (prog 1.14/1.08, slip 1.60/1.53). A PASS that does not improve slip vs hard1 is still a pass (extension), but is not extra good news.

**verdict**: PASS (continuation/hardening extension, same recipe as parent) -- another 40M steps warm from the already-PASSed cw-arch-tf-r1-hard1 IMPROVES economy without regressing the gait: DR0 gate det/sto gait_valid 6/6, 0 term both, prog med 1.23/1.08 (was 1.14/1.08), slip med 1.21/1.32 (was 1.60/1.53, a real drop); own-cfg DR0.5 det/sto gv 6/6, 0 term, prog med 1.16/1.01 (was 1.10/1.00), slip med 1.19/1.51 (was 1.41/1.67); roll clean (peak 1.6-2.8deg, tail 0.4-0.5deg, better than hard1's 2.0-7.8/0.4-1.9); 10-frame contact-sheet across all 6 det episodes shows clean six-leg cycling, no flag-leg/parked-leg/drag pathology. Still not on the deployment-exact obs contract (same as hard1, walk-only). PROCESS NOTE: this run had NO ledger/INTENT record -- launched by a concurrent cycle acting on untrusted external MCP kicks (kick_20260815T165322_b36a18 family) via a raw kubectl-exec command that bypassed launch_run.py entirely, with no hypothesis/gate/doc entry anywhere; backfilled into the ledger this cycle from pod log + checkpoint evidence so it is not a silent gap. Evidence: logs/ckpt_eval/cw_arch_tf_r1_hard2_r1_{det,sto,owncfg_det,owncfg_sto}/.

**note**: BACKFILLED ledger entry: run was launched manually (kubectl exec, bypassing launch_run.py) by a prior cycle acting on external MCP kicks, no ledger/INTENT record existed. extra_args reconstructed from parent hard1's args + pod log evidence (warm start file, out-name, device); exact CLI not recoverable (process exited, no history).

