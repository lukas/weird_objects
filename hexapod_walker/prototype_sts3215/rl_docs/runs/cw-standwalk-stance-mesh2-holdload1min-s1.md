# cw-standwalk-stance-mesh2-holdload1min-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T08:05:53+00:00

**pod**: hexapod-mjx-train-1

**steps**: 6000000

**parent**: cw-standwalk-stance-mesh2-holdload1min

**wandb_id**: iphg85z4

**hypothesis**: Seed twin of holdload1min (seed 1): same question -- does gating hold income on measured per-foot load (min-over-feet) produce a six-foot quiet hold from scratch on mesh/100Hz? holdonly1's stilt basin discovery may be seed-lucky; two seeds tell us whether the load-gated recipe finds six-foot stance reliably or lands in yet another basin. Predictions as holdload1min.

**gate**: Same as holdload1min: hold panel at 6M, DR-0 det+sto n=6+6, >=10/12 survive with zero OC/tilt terms, six-foot stance (valid_plant or all-leg duty>=0.9), cur_p95<=1.0A.

**verdict**: Result: FAIL at the 6M hold gate -- the load-min income gate does not produce a six-foot hold from scratch; PPO finds a BELLY-FLOP FREEZE basin instead. Evidence: DR-0 gate hold det+sto 12/12 SURVIVE the full 15s with zero terminations and roll_tail 0.0, but 0/12 ok -- h_err_end=70mm (body on the ground), all-feet duty <=0.27, plant_margin=never-planted sentinel, cur_p95 ~0.5A, return -802..-985/ep; video (hold_det_0) shows it dropping from the planted start onto its belly by t=3s, legs folded up, frozen thereafter. Why it matters: the reward IS aligned (probe: honest six-foot hold earns +1471.6/ep vs this policy's -802; belly-freeze is deeply unprofitable) yet training reward DECLINED all run (-94/-406/-618/-627) -- PPO starts every hold episode INSIDE the paying plant basin and still walks downhill out of it into the safest do-nothing attractor. That is an exploration/optimization failure, not a pricing failure: term_cost never fires (it survives), current_hot barely fires (0.5A), and the sparse min-over-feet income gives no gradient from belly back to plant. What's next: single-lever optimization arms -- (a) DR-0 isolation (is flat DR=0.2 from step 0 what knocks the random-init policy out of the plant basin?) and (b) entropy retune (does more sustained exploration keep contact with the paying basin?) -- launched this cycle as holdload1min-dr0/-ent4. Scratch-pair joint read: seed 0 pending its owner's triage; this seed's basin is belly-freeze, warm sibling's is knife-edge collapse.

