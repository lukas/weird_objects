# risewalk-single2-s7

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: done

**created**: 2026-08-15T22:32:46+00:00

**pod**: hexapod-mjx-train-6

**steps**: 4500000

**parent**: none

**wandb_id**: r3nbkwhb

**hardware_ready**: False

**hypothesis**: GRU-encoder rise-to-walk A/B/C benchmark, seed 7 -- same design as risewalk-single2-s5 (one seed per pod).

**gate**: Same pre-registered risewalk gate as risewalk-single2-s5.

**verdict**: LEDGER-HYGIENE CLOSE (08-19 idle-drain cycle), no fresh evaluation of this run: ledger status was stale RUNNING since 08-15 -- W&B shows every phase of the relaunched cohort (rw_rise_A/B/C_s5/6/7, per-attempt names) state=finished on 08-15 evening, and /proc on train-4/5/6 confirms no train_ppo_transfer process remains (only leftover pod_memwatch.sh watchers, killed fleet-wide this cycle). The GRU-encoder condition-A/B/C science was absorbed into the dynrep line conclusion (dynrep/STATUS: scratch PPO best in every dynrep cohort; line stopped by pre-registered gate + operator order). Recorded only to clear stale drain-queue noise; no new science claimed.

**note**: Script-owned cohort (pod_risewalk.sh); registered post-hoc per operator order 20260815T221231Z; wandb_id = current phase run rw_rise_C_s7; verified live via check_cohort this cycle.

