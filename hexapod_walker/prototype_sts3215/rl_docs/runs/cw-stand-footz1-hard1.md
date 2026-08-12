# cw-stand-footz1-hard1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-12T09:36:06+00:00

**pod**: hexapod-mjx-train-0

**steps**: 10000000

**parent**: cw-stand-footz1-r1

**wandb_id**: nd3u9txh

**hardware_ready**: False

**hypothesis**: Teach the standing robot to plant all six feet AND hold that for longer, harder training -- does the just-landed foot-height BC anchor (which fixed the invisible one-foot park in 2M steps of discovery) keep working, and does the small rise regression clear, when hardened to 10M steps the same way bc1->bc1-hard1 and holdbc1->holdbc1-hard1 both consolidated cleanly on this identical discovery-then-hardening pattern? One variable vs the discovery run: step count only (2M->10M), same recipe, same warm start (holdbc1_hard1), same foot-z anchor.

**gate**: PASS if det hold: every foot duty >=0.5 in all 6 episodes (no regression from discovery) AND hold det+sto valid_plant >=10/12 (matching or beating discovery's 10/12, no new park) AND det rise >=6/6 valid_plant with no new falls (recovering the flat-start miss) AND det lower >= matched-parent probe (4/6 baseline) with no NEW outrigger pattern beyond the inherited one AND drag_m/roll_tail quoted vs both the frozen parent and the footz1-r1 discovery run (no worsening beyond noise). FAIL if any foot parks (duty <0.5 det hold) or any of the above regresses vs the discovery run; record train/bc_anchor_footz_loss trajectory either way (does it converge further with more steps).

**verdict**: Hold WINS decisively but lower REGRESSES against the run's own gate -- net FAIL, hard1 (holdbc1_hard1) stays deployed. Det+sto hold: ALL SIX feet duty 0.92-0.99 in every one of 12 episodes (matches/beats footz1-r1 discovery, first clean six-foot hold survives hardening), valid_plant clean, video-confirmed level quiet stand. Det rise unchanged at 5/6 (same pre-existing flat-start height miss, zero falls, honest crouch-to-stand) -- no new regression, but also didn't recover to the gate's required 6/6. Det+sto LOWER collapsed to 0/12 (both passes) -- REGRESSED from the discovery run's 4/6-matching-parent baseline. Video + per-foot duty_cycle confirm this is the SAME three-leg outrigger/flag-leg cheat this lineage has always carried (legs idx0/2/4 hovering, idx1/3/5 planted -- exact same leg indices as footz1-r1's inherited pattern) but now MORE severe under the 10M hardening budget: worst-foot clearance up to 147-170mm (was ~100mm max at discovery) and 0/12 pass rate (was ~4/6). KNOWN exploit, no forensics needed -- one-line stop for that clause. Root cause: this lineage (footz1, warm from holdbc1-hard1) never received the lower-mode BC anchor that a SIBLING branch (cw-stand-anchormix1-r1) already used to solve lower cleanly (6/6) -- the two fixes (foot-z anchor for hold, lower-anchor+stratified for lower) have never been combined. Follow-up cw-stand-footlow1 (discovery, warm from this checkpoint + adds train.bc_anchor_lower/stratified/state_aligned/lookahead) queued+launched this cycle to test the combination.

