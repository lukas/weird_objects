# cw-amp-m3-pushhard2-style05-r3n2040

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-23T00:42:53+00:00

**pod**: hexapod-mjx-train-2

**steps**: 6000000

**parent**: cw-amp-m3-pushhard1-style05-repeat3

**hypothesis**: Plain English: the walker that already survives three 10-25N shoves per episode is now hit with the same up-to-three shoves at 20-40N — can the count-hardened, style-kept walker absorb roughly double the per-shove force, or is ~25N near the recoverable ceiling for this build? Completes the M3 escalation 2x2 (count x force) from the count arm's own PASSing checkpoint. The force-only sibling n2040 (single shove 20-40N off the pushacq1 ckpt) is unread as of launch — this arm answers the combined dose on the strongest substrate regardless, and any disagreement between the two reads names which axis transfers. Prediction-if-true: tilt-terms spike at start (both axes at max) then fall over 6M; DR-0 own-cfg gate holds gait_valid >=5/6 with topples <=2/6 det + <=3/6 sto. Prediction-if-false: terms high AND flat with reward flat — repeated 40N-class shoves are beyond recoverable at this morphology/stance; that names a physical dose ceiling for the M3 spec (informative, not a lineage kill). Strongest alternatives: (a) survival via crouch-statue — watch height band + det prog med; (b) the thinning discriminator (style_reward 0.109 at repeat3 end) finally vetoes the wilder recovery transients — watch style_reward_mean vs the 0.1 bar.

**gate**: Hardening (6M, DR-0, dr.ext_push_repeat_max=3 AND dr.ext_push_n=20-40N). PASS = DR-0 own-cfg gate gait_valid >=5/6 det+sto, zero sacrificed legs, det prog med >=0.9, topples <=2/6 det AND <=3/6 sto, style_reward_mean >0.1 at end, majority of surviving episodes roll_class=recovered. INFORMATIVE-ceiling = topples above bar but training tilt-terms still falling at cutoff => continue per 08-21 ruling. FAIL-statue = det prog med <0.6 or crouch fingerprint (height band exit). FAIL = collapse/NaN.

**refused_reason**: hexapod-mjx-train-2 code marker c96540f45785701f8fbb71aedc9b175437a8d1c0 != local HEAD 07d79522e11398fbb316420fb0efdc6d7d589005. Sync first: snapshot.sh --sync hexapod-mjx-train-2 (and snapshot/commit before that if the tree is dirty).

