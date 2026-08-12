# cw-stand-transdrag1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-12T01:00:02+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-holdbc1-hard1

**wandb_id**: 3k274z6v

**hypothesis**: Operator 08-11 night: the deployed stand specialist scrapes its feet across the floor during stand/sit - measured tonight by the new harness drag field: hold drags 221mm per 15s episode (while PASSING 3/3), lower drags 371mm, rise 291mm (DR0 det, /tmp smoke; the metric is slip_m_total which existed but was never surfaced or judged for stance modes). Until tonight NOTHING priced a loaded foot sliding outside walk mode. DISCOVERY (2M, warm from holdbc1-hard1 itself, its exact recipe): reward.k_drag_trans=400 (bank operating point) charges loaded foot-XY travel beyond a per-episode allowance - rise keeps a 0.55m free budget covering its inherent 463mm curl slide (measured), hold/lower charge from the first excess mm. Bank-verified tonight: scraper goes net-negative, quiet stand and honest rise reference byte-identical. If-true: hold drag collapses toward 0 and lower drag halves with rise/hold success intact - the charge rides into every future stand arm. If-false (success erodes or drag persists): pricing closes AGAIN for the stand line and the lever is BC-anchor supervision of lower ticks, not price.

**gate**: Own-cfg det+sto @15s vs the BINDING matched frozen parent (identical cfg/seed): hold 6/6 strict-valid with drag_m med <=0.05 (parent measures 0.221); lower success >= parent AND drag_m med <=0.20 (parent 0.371); rise flat+crouch success >= parent (the 0.55m allowance must make the charge invisible to the honest curl); roll_settled 12/12, no new termination class; frames watched det+sto

