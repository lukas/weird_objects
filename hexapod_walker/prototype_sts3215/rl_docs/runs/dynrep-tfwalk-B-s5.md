# dynrep-tfwalk-B-s5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-15T22:31:35+00:00

**pod**: hexapod-mjx-train-7

**steps**: 1000000

**parent**: cw-dynrep-tf-state2-recovered1

**wandb_id**: f086dlfd

**hardware_ready**: False

**hypothesis**: Frozen-encoder arm of the operator-ordered (20260815T221231Z) Transformer walking/heading transfer test: the first G1/G1.1+G3-passing Transformer dynrep encoder (cw-dynrep-tf-state2-recovered1, md5 9df48f68) is frozen and PPO learns commanded-velocity walking through its latent z; tests whether the pretrained body-dynamics representation alone speeds/cleans learning vs scratch (A). No older/GRU encoder substituted per order.

**gate**: Judged as a matched triple with A/C at 1M steps: beats A on walk steps-to-threshold or final return outside eval noise, with gait-quality columns no worse; loses to A = the representation does not transfer frozen.

**note**: Script-owned cohort (pod_tfwalk.sh); G1/G1.1 re-verified this cycle on the exact v5_mjx_fresh test split (logs/ckpt_eval/cw_dynrep_tf_state2_recovered1/eval_g1_test_order_20260815T2219.json); W&B attempt name dynrep-tfwalk-B-s5.0815-2221Z.

