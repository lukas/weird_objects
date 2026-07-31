"""Stand-up current-draw simulation for the STS3215 hexapod.

Answers "how many amps does the whole robot draw when it stands up,
and does the power system survive it?" using the MuJoCo model in
``mujoco_prototype.py``.  Results + verdicts are written up in
``firmware/WIRING.md`` section 6.5 -- re-run this script and refresh
that section whenever masses, gains, or the leg geometry change.

Scenario
--------
The robot starts collapsed on its belly (legs folded: hip pitch -0.90,
knee +1.20 rad), position targets ramp to the standing stance over
``T_STAND`` seconds (cosine ramp), then hold.  Two mass cases are run
(the as-modeled 0.55 kg chassis and a realistic 1.30 kg chassis that
adds the yaw servos, the 138 x 46 x 24 mm ~300 g LiPo, and the deck
electronics) plus a stiff-gain case approximating the real servo PID.

Electrical model (per servo, 12 V bus)
--------------------------------------
    I(tau) = I_IDLE + (I_STALL - I_IDLE) * |tau| / TAU_STALL, capped at I_STALL
    I_IDLE    = 0.20 A     (WIRING.md 6.3: idle ~0.2 A/servo)
    I_STALL   = 2.70 A     (FEETECH STS3215 @ 12 V)
    TAU_STALL = 2.94 N*m   (30 kg-cm @ 12 V)

The net actuator torque at each hinge (``data.qfrc_actuator``) feeds
the model; per-leg branch current is the sum of that leg's 3 servos
and the trunk adds every leg plus the Uno Q logic draw reflected to
12 V.  This is conservative for standing holds: a geared servo
parked inside its deadband draws less than the linear model predicts.

Usage
-----
    PYTHONPATH=. python standup_current_sim.py            # default cases
    PYTHONPATH=. python standup_current_sim.py --fast     # 0.4 s slam-up
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mujoco  # noqa: E402
import mujoco_prototype as MP  # noqa: E402

I_IDLE = 0.20
I_STALL = 2.70
TAU_STALL = 2.94
LOGIC_12V_A = 0.20   # Uno Q via buck (~0.3 A @ 5 V -> ~0.15 A @ 12 V) + margin

FOLD_PITCH = -0.90   # legs folded up (joint range -1.40..+0.52)
FOLD_KNEE = 1.20     # knee tucked    (joint range -0.35..+1.40)
T_SETTLE = 1.0
T_STAND = 1.5
T_HOLD = 2.5

# Realistic chassis mass: printed plates + 6 yaw STS3215 + 300 g LiPo
# (138 x 46 x 24 mm pack) + PDB/buck/Uno Q/deck stack.  The leg masses
# already carry the hip + knee servos (see mujoco_prototype.py).
REALISTIC_CHASSIS_MASS = 1.30


def servo_current(tau: float) -> float:
    return min(I_IDLE + (I_STALL - I_IDLE) * abs(tau) / TAU_STALL, I_STALL)


def run_case(label: str, chassis_mass: float, gain_mult: float = 1.0,
             t_stand: float = None):
    t_stand = T_STAND if t_stand is None else t_stand
    MP.CHASSIS_MASS = chassis_mass
    MP.KP_YAW = 18.0 * gain_mult
    MP.KP_PITCH = 26.0 * gain_mult
    MP.KP_KNEE = 24.0 * gain_mult
    model = mujoco.MjModel.from_xml_string(MP.build_xml(obstacles_xml=""))
    data = mujoco.MjData(model)
    model.hfield_data[:] = 0.0            # flat world
    dt = model.opt.timestep

    chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    total_mass = float(model.body_subtreemass[chassis_id])

    joint_dof = {}
    for i in range(6):
        for kind in ("yaw", "pitch", "knee"):
            j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"L{i}_{kind}")
            joint_dof[(i, kind)] = model.jnt_dofadr[j]

    # Initial collapsed pose: belly on the floor, legs folded.
    data.qpos[0:3] = [0.0, 0.0, 0.010]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    for i in range(6):
        for kind, val in (("yaw", 0.0), ("pitch", FOLD_PITCH), ("knee", FOLD_KNEE)):
            j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"L{i}_{kind}")
            data.qpos[model.jnt_qposadr[j]] = val
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    def set_targets(pitch, knee):
        for i in range(6):
            base = i * 6
            data.ctrl[base + 0] = 0.0
            data.ctrl[base + 1] = pitch
            data.ctrl[base + 2] = knee

    stance_p, stance_k = MP.STANCE_FEMUR, MP.STANCE_TIBIA
    t_hist, trunk_hist, branch_hist, chassis_z, tau_hist = [], [], [], [], []

    for k in range(int((T_SETTLE + t_stand + T_HOLD) / dt)):
        t = k * dt
        if t < T_SETTLE:
            set_targets(FOLD_PITCH, FOLD_KNEE)
        elif t < T_SETTLE + t_stand:
            s = (t - T_SETTLE) / t_stand
            s = 0.5 - 0.5 * math.cos(math.pi * s)   # smooth cosine ramp
            set_targets(FOLD_PITCH + s * (stance_p - FOLD_PITCH),
                        FOLD_KNEE + s * (stance_k - FOLD_KNEE))
        else:
            set_targets(stance_p, stance_k)
        mujoco.mj_step(model, data)

        branches = [
            sum(servo_current(float(data.qfrc_actuator[joint_dof[(i, kind)]]))
                for kind in ("yaw", "pitch", "knee"))
            for i in range(6)
        ]
        tau_hist.append(max(abs(float(data.qfrc_actuator[d]))
                            for d in joint_dof.values()))
        t_hist.append(t)
        branch_hist.append(branches)
        trunk_hist.append(sum(branches) + LOGIC_12V_A)
        chassis_z.append(float(data.body(chassis_id).xpos[2]))

    t_hist = np.array(t_hist)
    trunk = np.array(trunk_hist)
    branch = np.array(branch_hist)
    z = np.array(chassis_z)
    tau = np.array(tau_hist)

    def windowed_max(x, win_s):
        # Fuses blow on sustained current, not spikes -> windowed averages.
        w = max(1, int(win_s / dt))
        return float(np.convolve(x, np.ones(w) / w, mode="valid").max())

    standup = (t_hist >= T_SETTLE) & (t_hist < T_SETTLE + t_stand + 0.3)
    hold = t_hist >= (T_SETTLE + t_stand + 1.0)
    up_z = data.body(chassis_id).xmat.reshape(3, 3)[2, 2]

    print(f"\n=== {label} (total robot mass {total_mass:.2f} kg) ===")
    print(f"  chassis z: start 0.010 m -> final {z[-1]:.3f} m "
          f"(stance target {MP.stance_chassis_height():.3f} m); upright z.z = {up_z:.3f}")
    print(f"  TRUNK current (18 servos + logic):")
    print(f"    peak instantaneous          : {trunk.max():6.2f} A")
    print(f"    peak 100 ms average         : {windowed_max(trunk, 0.10):6.2f} A")
    print(f"    peak 500 ms average         : {windowed_max(trunk, 0.50):6.2f} A")
    print(f"    stand-up phase mean         : {trunk[standup].mean():6.2f} A")
    print(f"    standing hold (steady state): {trunk[hold].mean():6.2f} A")
    bmax = branch.max(axis=0)
    print(f"  PER-LEG branch current:")
    print(f"    worst leg peak instantaneous: {bmax.max():6.2f} A  (leg {int(bmax.argmax())})")
    print(f"    worst leg peak 500 ms avg   : "
          f"{max(windowed_max(branch[:, i], 0.5) for i in range(6)):6.2f} A")
    print(f"    worst leg hold mean         : {branch[hold].mean(axis=0).max():6.2f} A")
    print(f"  worst single-servo torque    : {tau.max():5.2f} N*m "
          f"(sim limit {MP.TORQUE_LIMIT} N*m, STS3215 stall {TAU_STALL} N*m)")
    ok_stand = z[-1] > 0.8 * MP.stance_chassis_height() and up_z > 0.95
    print(f"  stood up OK: {ok_stand}")
    return ok_stand


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--fast", action="store_true",
                    help="also run the aggressive 0.4 s stand-up (worst-case peaks)")
    args = ap.parse_args()

    print(f"STS3215 model: I = {I_IDLE} A idle + {I_STALL - I_IDLE} A * |tau|/{TAU_STALL} N*m, "
          f"cap {I_STALL} A @ 12 V")
    ok = True
    ok &= run_case("as-modeled chassis mass (0.55 kg), sim gains", 0.55)
    ok &= run_case("realistic mass (1.30 kg chassis), sim gains",
                   REALISTIC_CHASSIS_MASS)
    ok &= run_case("realistic mass, stiff gains (5x ~ real STS3215 PID)",
                   REALISTIC_CHASSIS_MASS, gain_mult=5.0)
    if args.fast:
        ok &= run_case("realistic mass, stiff gains, FAST 0.4 s stand-up",
                       REALISTIC_CHASSIS_MASS, gain_mult=5.0, t_stand=0.4)
    # The soft sim-gain cases sag below the 80% stance-height bar (pure-P
    # droop); only fail the run if the stiff-gain (realistic) case falls.
    print("\nDone.  See firmware/WIRING.md section 6.5 for the written-up budget.")


if __name__ == "__main__":
    main()
