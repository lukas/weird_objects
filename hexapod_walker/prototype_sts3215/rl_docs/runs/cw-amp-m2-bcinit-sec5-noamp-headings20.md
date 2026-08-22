# cw-amp-m2-bcinit-sec5-noamp-headings20

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-22T21:26:48+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-noamp

**wandb_id**: 6pcol9je

**hypothesis**: Plain English: task-only twin of style05-headings20 -- same question (does BC-init walking survive being asked to turn, goal.walk_heading_max_rad 0.0->0.4363 rad / 25deg, stage 1 of the operator's untried forward-only->small-set->full->irregular staged curriculum fb_20260822T003132) with ZERO AMP style, continuing from the noamp checkpoint (--init-from-source). Isolates whether AMP style is protective when the exploration problem returns under heading diversity (paired read: if noamp collapses first while style05 keeps walking, style earns its first real functional -- not just cosmetic -- benefit).

**gate**: Discovery continuation (2M, judged on det video + DR-0 gate harness at the new heading range, read JOINTLY with style05-headings20 twin). INFORMATIVE-PASS = gait_valid stays >=5/6 det+sto, no new sacrificed legs, height_err stays near 18-31mm (not climbing to 59-85mm crouch). FAIL-collapse = gait degrades toward statue/drag under turning demand. Joint read: style-vs-noamp delta under heading stress is the batch's primary measurement.

**verdict**: Task-only twin of style05-headings20 -- also SURVIVES heading-diversity stage 1 (25deg) cleanly. DR-0 gate: gait_valid 6/6 det+sto, zero sacrificed legs, det prog med 1.10 (flat vs forward-only baseline's 1.09), slip med 2.45 (up from 2.13, ~15% cost, within allowance); sto prog med 0.79 (up from 0.52), slip 4.36 (down from 5.38). env/height_err_mm 14-24mm the whole run, no crouch. Still slightly WORSE than the style05 twin on every axis under heading stress too (det prog 1.10 vs 1.30, slip 2.45 vs 2.17) -- the modest style edge persists and widens a little under the harder turning task. Video clean six-leg cycling, no new pathology. hardware-ready: no (2M continuation, DR-0). Next: stage 2 launched paired with the style05 twin.

