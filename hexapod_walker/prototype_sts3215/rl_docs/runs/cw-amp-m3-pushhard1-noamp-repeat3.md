# cw-amp-m3-pushhard1-noamp-repeat3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-23T00:12:31+00:00

**pod**: hexapod-mjx-train-1

**steps**: 6000000

**parent**: cw-amp-m3-pushacq1-noamp

**hypothesis**: Plain English: pushacq1-noamp just passed the single-push acquisition gate cleanly (0/12 terminations, PASSED better than the style05 twin's 1/6+0/6) -- the concurrent cycle's own pre-registered rule says 'if noamp reads clearly better, escalation respecs onto the noamp line are cheap', and M3's own bar explicitly wants REPEATED pushes per episode, which the single-push mechanism never tested. Mirrors the already-running cw-amp-m3-pushhard1-style05-repeat3 arm exactly (dr.ext_push_repeat_max=3, up to 3 non-overlapping 10-25N shoves/episode spaced 1-3s apart, horizon 13s of the 15s episode, per-shove dose unchanged) but warm-started from the BETTER (noamp) single-push champion instead. Prediction-if-true: tilt terminations rise briefly (3x exposure) then fall back toward pushacq1-noamp's near-zero floor over the 6M budget, and the DR-0 own-cfg gate under multi-push episodes holds gait_valid >=5/6 with topples <=1/6 det, <=2/6 sto. Prediction-if-false: terminations stay high/flat all run -- recovery capacity (not exposure count) is the binding constraint, matching whatever the style05 twin reads. Strongest alternative: noamp's cleaner single-push floor makes 3-push survival come almost for free (terms barely rise) -- would say push COUNT is trivial once single-push is truly solved, and per-shove FORCE is the real M3 frontier.

**gate**: Acquisition (6M, DR-0, repeat_max=3, dose/shove unchanged). Joint read vs cw-amp-m3-pushhard1-style05-repeat3 (same lever, style-kept twin): PASS = DR-0 own-cfg gate under repeated pushes shows gait_valid >=5/6 det+sto, topples <=1/6 det AND <=2/6 sto, zero sacrificed legs, det prog med >=0.9, video shows genuine multi-shove survival (not just surviving the first hit then coasting on a truncated episode). INFORMATIVE-plateau = topples no better than the single-push acquisition floor with training tilt-terms flat over the last 2M => push COUNT needs its own dedicated budget/curriculum, not a flat respec. FAIL = collapse/statue/numerical blowup, or terminations scale up roughly linearly with push count (no learned multi-push recovery at all).

**refused_reason**: hexapod-mjx-train-1 already runs cw-amp-m3-pushhard1-style05-n2040 — GPU pods host exactly one run; pick a free GPU pod.

