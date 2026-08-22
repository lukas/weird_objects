# cw-amp-m2-bcinit-sec5-noamp-headings20

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-22T21:26:48+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-noamp

**wandb_id**: 6pcol9je

**hypothesis**: Plain English: task-only twin of style05-headings20 -- same question (does BC-init walking survive being asked to turn, goal.walk_heading_max_rad 0.0->0.4363 rad / 25deg, stage 1 of the operator's untried forward-only->small-set->full->irregular staged curriculum fb_20260822T003132) with ZERO AMP style, continuing from the noamp checkpoint (--init-from-source). Isolates whether AMP style is protective when the exploration problem returns under heading diversity (paired read: if noamp collapses first while style05 keeps walking, style earns its first real functional -- not just cosmetic -- benefit).

**gate**: Discovery continuation (2M, judged on det video + DR-0 gate harness at the new heading range, read JOINTLY with style05-headings20 twin). INFORMATIVE-PASS = gait_valid stays >=5/6 det+sto, no new sacrificed legs, height_err stays near 18-31mm (not climbing to 59-85mm crouch). FAIL-collapse = gait degrades toward statue/drag under turning demand. Joint read: style-vs-noamp delta under heading stress is the batch's primary measurement.

