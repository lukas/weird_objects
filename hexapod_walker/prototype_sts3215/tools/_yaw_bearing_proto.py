"""Scratch: prototype the yaw turntable hub + 6706 bearing integration.

Frame: coxa-local, origin = yaw axis at disc-horn TOP (z=0). +Z up,
+X outboard, +Y = hip-pitch axis.  Disc horn occupies z[-5,0], the
servo output face is at z=-5.  The 6706-2RS (30x37x4) sits z[-5,-1]
around the horn: outer race in a chassis tower, inner race clamped to
the rotating hub.
"""
import numpy as np
import pyvista as pv
import hexapod_prototype as HP

pv.global_theme.allow_empty_mesh = True

_cyl = HP._cyl
_box = HP._box
_union = HP._union
_diff = HP._diff

# --- 6706-2RS thin-section bearing -------------------------------------
YAW_BRG_ID = 30.0
YAW_BRG_OD = 37.0
YAW_BRG_W = 4.0
YAW_BRG_INNER_OD = 32.0   # approx inner-race OD (rotating band r15..16)
YAW_BRG_OUTER_ID = 35.0   # approx outer-race ID (stationary band r17.5..18.5)

HORN_TOP_Z = 0.0
HORN_BOT_Z = -HP.HORN_STACK_H          # -5  (servo output face)
BRG_TOP_Z = -1.0
BRG_BOT_Z = BRG_TOP_Z - YAW_BRG_W      # -5

PAD_T = HP.PEDESTAL_CAP_T              # 4
PAD_OD = 44.0
HORN_CLR = HP.DISC_HORN_OD / 2.0 + 1.0  # r11 inner skirt bore (clear horn)


def make_bearing_visual():
    inner = _diff(_cyl(YAW_BRG_INNER_OD / 2.0, YAW_BRG_W),
                  _cyl(YAW_BRG_ID / 2.0, YAW_BRG_W * 2))
    outer = _diff(_cyl(YAW_BRG_OD / 2.0, YAW_BRG_W),
                  _cyl(YAW_BRG_OUTER_ID / 2.0, YAW_BRG_W * 2))
    brg = _union(inner, outer)
    brg.apply_translation([0, 0, BRG_BOT_Z + YAW_BRG_W / 2.0])
    return brg


def make_hub():
    # Top pad: bolts DOWN onto the yaw disc horn (z[0,PAD_T]).
    pad = _cyl(PAD_OD / 2.0, PAD_T)
    pad.apply_translation([0, 0, PAD_T / 2.0])

    # Inner-race clamp flange (z[BRG_TOP_Z,0]) sits on the inner race top;
    # OD stays <= inner-race OD so it never touches the stationary outer
    # race.  Annular (bore clears the horn).
    flange = _diff(_cyl(YAW_BRG_INNER_OD / 2.0, -BRG_TOP_Z),
                   _cyl(HORN_CLR, -BRG_TOP_Z * 3))
    flange.apply_translation([0, 0, BRG_TOP_Z / 2.0])

    # Boss into the inner-race bore (z[BRG_BOT_Z,BRG_TOP_Z]); annular.
    boss = _diff(_cyl(YAW_BRG_ID / 2.0 - 0.1, YAW_BRG_W),
                 _cyl(HORN_CLR, YAW_BRG_W * 3))
    boss.apply_translation([0, 0, BRG_BOT_Z + YAW_BRG_W / 2.0])

    hub = _union(pad, flange, boss)

    # Cut the disc-horn mating pattern through the pad (4x M3 PCD14 +
    # centre + collar recess).
    cuts = []
    r = HP.DISC_HORN_BOLT_PCD / 2.0
    for t in HP.DISC_HORN_BOLT_ANGLES_RAD:
        h = _cyl(HP.DISC_HORN_BOLT_OD / 2.0, PAD_T * 4)
        h.apply_translation([r * np.cos(t), r * np.sin(t), PAD_T])
        cuts.append(h)
    ctr = _cyl(HP.HORN_CENTRE_OD / 2.0, PAD_T * 4)
    ctr.apply_translation([0, 0, PAD_T])
    cuts.append(ctr)
    collar = _cyl(HP.DISC_HORN_COLLAR_OD / 2.0, HP.DISC_HORN_COLLAR_DEPTH + 0.1)
    collar.apply_translation([0, 0, HP.DISC_HORN_COLLAR_DEPTH / 2.0 - 0.05])
    cuts.append(collar)
    return _diff(hub, *cuts)


if __name__ == "__main__":
    hub = make_hub()
    brg = make_bearing_visual()
    horn = HP.make_disc_horn()
    horn.apply_translation([0, 0, HORN_BOT_Z])   # seat at z[-5,0]
    print("hub bounds", np.round(hub.bounds, 2).tolist())
    print("brg bounds", np.round(brg.bounds, 2).tolist())

    pl = pv.Plotter(off_screen=True, window_size=(1200, 1000))
    pl.set_background("white")
    pl.add_mesh(pv.wrap(hub), color="#9467bd", opacity=0.55)
    pl.add_mesh(pv.wrap(brg), color="#c0a000")
    pl.add_mesh(pv.wrap(horn), color="silver")
    pl.view_xz()
    pl.screenshot("/tmp/hub_proto_side.png")
    pl.view_vector((0.6, -1, 0.4))
    pl.screenshot("/tmp/hub_proto_iso.png")
    print("rendered")
