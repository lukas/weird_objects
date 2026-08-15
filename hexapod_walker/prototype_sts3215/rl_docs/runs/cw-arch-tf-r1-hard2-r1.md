# cw-arch-tf-r1-hard2-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-15T17:23:49+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-arch-tf-r1-hard1

**wandb_id**: 7lgpxtc7

**hardware_ready**: False

**hypothesis**: Ad hoc continuation (no pre-registered spec found in ledger/arch docs) launched by a concurrent cycle in response to untrusted external MCP kicks kick_20260815T165322_b36a18/fb_20260815T165849/fb_20260815T170903+a23668 asking to 'continue tf-r1-hard1'. Backfilled by triage cycle c(kick-review-0815-17xx) from pod log evidence (no launch_run.py record existed). Implicit question: does another 40M steps warm-started from the already-PASSed cw-arch-tf-r1-hard1 (which matched r7 at budget parity but carried elevated slip 1.5-1.6 vs r7's 0.95-1.02) further consolidate the gait or reduce that slip gap, without regressing the walk.

**gate**: Same gate as parent cw-arch-tf-r1-hard1 (identical recipe, this is a length extension not a mechanism change): PASS = det+sto gait_valid 6/6, zero sacrificed legs, zero falls, prog_ratio med >=0.85, video six-leg cycling, matching-or-beating hard1's own numbers (prog 1.14/1.08, slip 1.60/1.53). A PASS that does not improve slip vs hard1 is still a pass (extension), but is not extra good news.

**verdict**: PASS -- the extra 40M-step continuation (ad hoc launch, ledger backfilled) CONSOLIDATES the causal-transformer walk: DR0 gate det+sto gait_valid 6/6 both, zero sacrificed legs, zero falls across all 24 gate + 24 own-cfg(DR0.5) episodes (48/48), roll clean/recovered every episode (peak 1.4-8.0deg, tail 0.2-1.4deg, all settled), height error <=8mm. prog_ratio med 1.23 det / 1.09 sto (matches hard1's 1.14/1.08, >=0.85 bar) and slip/m med IMPROVED 1.60->1.22 det / 1.53->1.32 sto vs hard1's own numbers -- closing (not closed) the gap to the hist16-MLP champion r7's 0.95-1.02. Own-cfg DR0.5 slip is worse (1.19 det/1.51 sto, similar to hard1) so the DR0-only slip gain does not fully transfer to on-DR training conditions -- real but partial consolidation, not a new mechanism result. Video (12 det+12 sto frame strips, both DR passes) shows six feet cycling through changing contact patterns, no flag leg, no parked/static gait, consistent height. This is a length-extension PASS on an already-PASSed line (hard1), not a new architecture finding.

**note**: BACKFILLED ledger entry: run was launched manually (kubectl exec, bypassing launch_run.py) by a prior cycle acting on external MCP kicks, no ledger/INTENT record existed. extra_args reconstructed from parent hard1's args + pod log evidence (warm start file, out-name, device); exact CLI not recoverable (process exited, no history).

