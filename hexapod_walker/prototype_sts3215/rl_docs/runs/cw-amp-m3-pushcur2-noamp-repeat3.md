# cw-amp-m3-pushcur2-noamp-repeat3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T01:24:38+00:00

**pod**: hexapod-mjx-train-1

**steps**: 6000000

**parent**: cw-amp-m3-pushcur1-noamp-repeat2

**wandb_id**: s7monu60

**hypothesis**: Plain English: the walker that found two shoves nearly free now takes up to THREE per episode — is the third shove learnable from the rung-2 prior, where the flat jump from one to three learned nothing (repeat3-r1: reward and terms flat all 6M, 3/12 topples)? Stage 2 of the count curriculum: identical to pushcur1-noamp-repeat2 except dr.ext_push_repeat_max=3, warm from repeat2's own checkpoint (PASS at 2/12 topples but terms started AND stayed low — rung 2 was inherited, so THIS rung is the real test of whether count staging works at all). Prediction-if-true: terms start high and FALL, gate at repeat_max=3 beats repeat3-r1's 3/12 within the <=1/6 det + <=2/6 sto bar — count is stageable and M3's count axis closes at 3. Prediction-if-false: terms flat here too despite the rung-2 prior — the third shove lands mid-recovery and dose staging cannot reach mid-recovery balance; joint with style05-r3b1530-r1's count-x-force plateau this locks in the recovery-mechanism lever for M3. Strongest alternative: rung 3 is also nearly free (terms low-flat, gate clean) — then repeat3-r1's failure was its cold jump, and shove density/episode length is the real ceiling variable.

**gate**: Hardening stage 2 on the count axis (6M, DR-0, 10-25N, repeat_max=3, warm from pushcur1-noamp-repeat2). PASS = own-cfg gate gait_valid >=5/6 det+sto, zero sacrificed legs, det prog med >=0.9, topples <=1/6 det AND <=2/6 sto, video shows surviving three shoves in-episode; on PASS the count axis closes at 3 and the M3 spec records staged-count as the recipe. INFORMATIVE-plateau = topples above bar with terms flat over last 2M => count not stageable past 2; recovery-state mechanism is the named lever. INFORMATIVE-free = terms low-flat AND gate clean => density/episode-length is the ceiling variable, queue a density probe. FAIL = collapse/statue/NaN.

