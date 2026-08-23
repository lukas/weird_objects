# cw-amp-m3-pushcur2-noamp-repeat3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-23T01:24:38+00:00

**pod**: hexapod-mjx-train-1

**steps**: 6000000

**parent**: cw-amp-m3-pushcur1-noamp-repeat2

**wandb_id**: s7monu60

**hypothesis**: Plain English: the walker that found two shoves nearly free now takes up to THREE per episode — is the third shove learnable from the rung-2 prior, where the flat jump from one to three learned nothing (repeat3-r1: reward and terms flat all 6M, 3/12 topples)? Stage 2 of the count curriculum: identical to pushcur1-noamp-repeat2 except dr.ext_push_repeat_max=3, warm from repeat2's own checkpoint (PASS at 2/12 topples but terms started AND stayed low — rung 2 was inherited, so THIS rung is the real test of whether count staging works at all). Prediction-if-true: terms start high and FALL, gate at repeat_max=3 beats repeat3-r1's 3/12 within the <=1/6 det + <=2/6 sto bar — count is stageable and M3's count axis closes at 3. Prediction-if-false: terms flat here too despite the rung-2 prior — the third shove lands mid-recovery and dose staging cannot reach mid-recovery balance; joint with style05-r3b1530-r1's count-x-force plateau this locks in the recovery-mechanism lever for M3. Strongest alternative: rung 3 is also nearly free (terms low-flat, gate clean) — then repeat3-r1's failure was its cold jump, and shove density/episode length is the real ceiling variable.

**gate**: Hardening stage 2 on the count axis (6M, DR-0, 10-25N, repeat_max=3, warm from pushcur1-noamp-repeat2). PASS = own-cfg gate gait_valid >=5/6 det+sto, zero sacrificed legs, det prog med >=0.9, topples <=1/6 det AND <=2/6 sto, video shows surviving three shoves in-episode; on PASS the count axis closes at 3 and the M3 spec records staged-count as the recipe. INFORMATIVE-plateau = topples above bar with terms flat over last 2M => count not stageable past 2; recovery-state mechanism is the named lever. INFORMATIVE-free = terms low-flat AND gate clean => density/episode-length is the ceiling variable, queue a density probe. FAIL = collapse/statue/NaN.

**verdict**: Count rung 3 is CLEAN: repeat_max=3 (three 10-25N shoves/ep) warm from the rung-2 ckpt lands 0/6 det + 1/6 sto topples (bar <=1/6 det, <=2/6 sto), gait_valid 12/12, zero sacrificed, det prog med 1.17/slip 2.97 — the M3 count axis CLOSES at 3 with staged-count as the recipe, and staging beats the cold jump outright (repeat3-r1 flat 1->3 sat at 3/12 with nothing learning; warm chain = 1/12). Strips watched: upright six-leg cycling det, the single sto knockdown is a genuine full roll topple. HOWEVER the pre-named strongest-alternative fired AGAIN: tilt terms started low and stayed low-flat (~19-21/window all 6M, reward flat after Q1) — rung 3 was largely FREE from the rung-2 prior, i.e. count-at-mild-force is not the hard direction; r1's failure was its cold jump. Open variable per the gate's own free-branch: shove DENSITY/mid-recovery landings. Density stress probe launched eval-side (repeat_max=6 at 15s, same ckpt, train-1, out logs/ckpt_eval/cw_amp_m3_pushcur2_noamp_repeat3_dense6) — if that stays clean, count/density is closed entirely and force (n2040 chain) is M3's only open axis; if it breaks, it's direct evidence for the recovery-mechanism lever.

