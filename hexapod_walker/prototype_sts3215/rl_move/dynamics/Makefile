# dynamics (dynrep) — run from anywhere: make -C rl_move/dynamics <target>
HERE  := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
PROTO := $(abspath $(HERE)/../..)
PY    := $(abspath $(PROTO)/../..)/.venv/bin/python

help:                    ## this help
	@grep -E '^[a-z-]+:.*##' $(MAKEFILE_LIST) | awk -F':.*## ' '{printf "  %-14s %s\n", $$1, $$2}'

smoke:                   ## tiny end-to-end run: collect 12 eps -> train 300 steps -> eval
	cd $(PROTO) && $(PY) -m rl_move.dynamics.collect \
	    --out rl_move/dynamics/datasets/smoke --episodes 12 \
	    --episode-seconds 10 --walk-episode-seconds 8 --seed 0
	cd $(PROTO) && $(PY) -m rl_move.dynamics.train \
	    --data rl_move/dynamics/datasets/smoke --name dyn_smoke \
	    --steps 300 --val-every 150 --val-windows 512 --warmup 50 \
	    --allow-excess-window-reuse
	cd $(PROTO) && $(PY) -m rl_move.dynamics.eval_model \
	    --ckpt rl_move/dynamics/models/dyn_smoke.pt \
	    --data rl_move/dynamics/datasets/smoke \
	    --val-windows 1024 --linear-rows 4096

collect:                 ## v1 dataset (400 episodes; append-safe, vary --seed via SEED=)
	cd $(PROTO) && $(PY) -m rl_move.dynamics.collect \
	    --out rl_move/dynamics/datasets/v1 --episodes 400 --seed $(or $(SEED),0)

train:                   ## pretrain dyn_v1 on datasets/v1
	cd $(PROTO) && $(PY) -m rl_move.dynamics.train \
	    --data rl_move/dynamics/datasets/v1 --name dyn_v1 --steps 20000

eval:                    ## gate G1 eval of dyn_v1 (+ latent dump)
	cd $(PROTO) && $(PY) -m rl_move.dynamics.eval_model \
	    --ckpt rl_move/dynamics/models/dyn_v1.pt \
	    --data rl_move/dynamics/datasets/v1 --dump-latents

.PHONY: help smoke collect train eval
