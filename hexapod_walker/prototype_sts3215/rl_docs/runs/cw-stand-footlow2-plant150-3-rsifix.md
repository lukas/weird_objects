# cw-stand-footlow2-plant150-3-rsifix

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-22T17:26:57+00:00

**pod**: hexapod-mjx-train-5

**steps**: 10000000

**parent**: cw-stand-footlow2-plant150-2c-heightfix

**wandb_id**: np32kmx1

**hypothesis**: Plain English: fix the RSI reward's target height so it agrees with the real gate target, then see if RSI-start rises actually improve. cw-stand-footlow2-plant150-2c-heightfix's RSI-start rise episodes stayed pinned at 22-29mm height error through 10M extra steps, completely unmoved -- root-caused as a genuine reward misalignment, not an unlearnable motion: the RSI mechanism's height-schedule rewrite (sim_env.py) anchored the episode's absolute target to rise_ref_belly2plant.npz's OWN stale recorded final height (110.96mm, extracted pre-tibia-150), ~21mm below the real goal.rise_height_mm=[128,136] window the eval actually grades against -- almost exactly matching the observed 22-29mm gap. Fixed: the RSI schedule now anchors to the episode's own live-cfg target (already correct post-tibia-150) and uses the reference only for fractional progress at the spawn point (2 new regression tests, full bank re-run clean). This run continues training from the SAME checkpoint with ONLY that mechanism fix live (identical cfg/recipe otherwise) -- if RSI genuinely just needed an aligned reward, height error should now fall well below the stale 20-29mm band; if it stays pinned even with the target fixed, the defect is elsewhere (e.g. the reference's intermediate joint poses themselves being unreachable/asymmetric at tibia-150, per the reverted extract_rise_ref.py --blend-mode ik finding).

**gate**: PASS if RSI-start rise (det+sto combined) reaches >=3/5 success (height_err_end_mm <=15mm, i.e. crossing the PLANT_SPEC window) with bridge/crouch/flat staying at or above this lineage's own baseline (det>=3/6, sto>=4/6) and no new falls/roll blowups. PARTIAL if RSI height_err falls decisively (median >=10mm improvement from the 22-29mm band) but doesn't yet cross 15mm -- read as 'fix worked, needs more budget or a slightly bigger RSI ramp allowance,' not a re-verdict of the mechanism. FAIL if RSI height_err stays statistically unchanged from the 22-29mm band despite the fix -- redirect to auditing the reference file's own intermediate joint poses (extract_rise_ref.py re-extraction) as the next lever, not further reward tuning.

**verdict**: THE RSI REWARD-TARGET FIX WORKS — RSI-start rise went from 0/5 (pinned 22-29mm error through 10M steps) to 5/5 clean, exactly the pre-registered PASS branch. Plain English: last cycle found RSI episodes were being TRAINED toward a height target ~21mm below what the eval grades (the stale npz's recorded final height); this run re-trained the same checkpoint with ONLY that mechanism fix, and the previously-unlearnable RSI starts now succeed every episode. Evidence: gate report det rsi 3/3 (height err 0.8/10.7/11.7mm, all <=15mm), sto rsi 2/2 (3.2/1.2mm); bridge/crouch/flat 7/7 (parent baseline det 3/6, sto 4/6 — now 12/12 rise successes overall); zero terminations; training reward rose then held (141.9->236.5) — reward and eval now AGREE, confirming the misalignment diagnosis end-to-end. Videos watched (det rsi strips): sprawled mid-ramp spawn -> symmetric rise -> level planted stance held to episode end. Honest caveats: 2 of 3 det RSI episodes end 'leaning' (roll 8.9-9.0deg, height err 10.7-11.7mm — inside the bar but at the window edge); and the prestaged SESSION composition (this stance + vref1 walk) shows a rise-segment FELL over_current at stand 163.7mm — that harness likely carries the stale pre-tibia-150 height cfg (the exact copy-paste trap CURRENT_TRUTHS documents; corrected cfg was proven decisive for this lineage twice), so the session read needs a corrected-cfg rerun before it means anything. Hardware-ready: stance-half candidate, pending the corrected-cfg session gate. Next: re-run the tibia-150 interactive session gate with corrected heights pairing this checkpoint with the promoted walk half (cw-dep-bcgait1-plant150-1) to close the MEASURED-PLANT GATE BREAK's stance half.

