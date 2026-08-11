# cw-arch-gru-long1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-11T13:34:42+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-arch-gru-r2

**wandb_id**: evuh0eak

**hypothesis**: GRU rung, hardening budget: r1/r2 (2M discovery each) proved all skills emerge in one recurrent policy but see-saw at tiny budgets (walk-heavy diet bought walk err 0.060 and lower 2/2 at the cost of hold 2.5->6.9deg). 10x the budget with a balanced diet (walk 0.60, rise/lower 0.15 each, hold 0.10) gives every skill more absolute data than either discovery run gave it — expect the see-saw to resolve: walk err well below 0.05 AND stance precision recovering toward r1 levels, in one policy. If the see-saw persists at 24M cumulative, the answer is capacity (gru-hidden 256) not steps.

**gate**: PASS if final det eval shows walk err <= 0.045 m/s AND hold <= 3deg AND rise completions in >= 2 start kinds AND lower >= 1/2 — i.e. the unified policy beats both discovery runs on their own strengths simultaneously. Compare against stance champion (hold 0.8deg) and walk champion for context in triage.

**verdict**: INVALID EVIDENCE, not retried: launched (by a concurrent cycle) citing r2's video-log line 'walk:ok' as proof rise/walk/lower all work; the actual harness/video on r2 (this cycle's triage) shows the SAME leg-sacrifice/paddle cheat as r1 -- the cited evidence was training-log-only, never checked against harness. It also 10x'd the step budget on an unchanged diet shape after 2 identical-class misses (r1, r2), which RESEARCH_RULES rules out (change the hypothesis, never the step count). Separately died on its own (W&B global_step regression) before burning real compute. Do not relaunch this spec; GRU rung stays frozen off the blocker list pending a real recipe change.

**failed_reason**: W&B global_step not advancing (131072 -> 1024)

