# euler_conventions.py: translating between common conventions for
# Euler angles

from scipy.spatial.transform import Rotation as R

class RPY():
    """Euler angles as Roll/Pitch/Yaw, intrinsic rotations with respect to
       North and horizon."""
    def __init__(self, roll=None, pitch=None, yaw=None, *, r=None):
        """With three arguments, construct a rotation matrix from roll, pitch
           and yaw angles, in degrees; with a single argument, treat
           it as a complete rotation matrix."""
        if r is None:
            self.r = R.from_euler("ZYX", (yaw, pitch, roll), degrees=True)
        else:
            self.r = r
        self.yaw, self.pitch, self.roll = self.r.as_euler("ZYX", degrees=True)
        # First angle belongs to [-180, 180] degrees (both inclusive)
        # Second angle belongs to [-90, 90] degrees if all axes are different (like xyz)
        # Third angle belongs to [-180, 180] degrees (both inclusive)
        # https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.as_euler.html
    def __str__(self):
        return f"r:{self.roll:6.1f} p:{self.pitch:5.1f} y:{self.yaw:6.1f}"

def test_RPY():
    assert str(RPY( 35,  25, 115)) == "r:  35.0 p: 25.0 y: 115.0"
    assert str(RPY( 10,  20,  30)) == "r:  10.0 p: 20.0 y:  30.0"
    assert str(RPY(-10, -20, -30)) == "r: -10.0 p:-20.0 y: -30.0"
    assert str(RPY(180,  20,   0)) == "r: 180.0 p: 20.0 y:   0.0"
    # SciPy normalizes the representation:
    assert str(RPY(  0, 100,   0)) == "r: 180.0 p: 80.0 y: 180.0"

class AzEl():
    """Euler angles as Azimuth/Elevation, extrinsic rotations with respect
       to North and horizon."""
    def __init__(self, azimuth=None, elevation=None, *, r=None):
        if r is None:
            self.r = R.from_euler("ZYX", (azimuth, elevation, 0), degrees=True)
        else:
            self.r = r
        self.azimuth, self.elevation, _ = self.r.as_euler("ZYX", degrees=True)
    def __str__(self):
        return f"a:{self.azimuth:6.1f} e:{self.elevation:5.1f}"
def test_AzEl():
    assert str(AzEl(  20,  30)) == "a:  20.0 e: 30.0"
    assert str(AzEl( 120,  30)) == "a: 120.0 e: 30.0"
    assert str(AzEl(-120, -30)) == "a:-120.0 e:-30.0"

class PT():
    """Euler angles as Pan/Tilt, intrinsic rotations with respect to
       base."""
    def __init__(self, pan=None, tilt=None, *, r=None):
        if r is None:
            self.r = R.from_euler("ZYX", (pan, tilt, 0), degrees=True)
        else:
            self.r = r
        self.pan, self.tilt, _ = self.r.as_euler("ZYX", degrees=True)
    def __str__(self):
        return f"p:{self.pan:6.1f} t:{self.tilt:5.1f}"
def test_PT():
    assert str(PT(  20,  30)) == "p:  20.0 t: 30.0"
    assert str(PT( 120,  30)) == "p: 120.0 t: 30.0"
    assert str(PT(-120, -30)) == "p:-120.0 t:-30.0"

def test_composition():
    # find azimuth/elevation of camera from base attitude and pan/tilt of
    # turret

    def compose(base_att, turret_pt):
        return AzEl(r=(base_att.r * turret_pt.r))

    # simple addition on one axis
    att = RPY(0, 30, 0)
    pt = PT(0, 20)
    assert str(compose(att, pt)) == "a:   0.0 e: 50.0"

    # rotating the base rotates the turret
    att = RPY(0, 0, 90)
    assert str(compose(att, pt)) == "a:  90.0 e: 20.0"

    # yawing the base right and pitching it up adds turret tilt to base pitch
    att = RPY(0, 30, 90)
    assert str(compose(att, pt)) == "a:  90.0 e: 50.0"

    # yawing the base right and rolling it left decreases azimuth and elevation
    att = RPY(-30, 0, 90)
    assert str(compose(att, pt)) == "a:  79.7 e: 17.2"

def test_inv_composition():
    # find necessary pan/tilt for turret from base attitude and target
    # azimuth/elevation

    def compose(base_att, target_azel):
        return PT(r=(base_att.r.inv() * target_azel.r))

    # simple subtraction on one axis
    att = RPY(0, 30, 0)
    azel = AzEl(0, 20)
    assert str(compose(att, azel)) == "p:   0.0 t:-10.0"

    # yawing the base requires unyawing the turret but has no effect on tilt
    att = RPY(0, 0, 90)
    assert str(compose(att, azel)) == "p: -90.0 t: 20.0"

    # yawing the base right and rollig it right subtracts base roll from target
    # elevation
    att = RPY(30, 0, 90)
    assert str(compose(att, azel)) == "p: -90.0 t:-10.0"

    # yawing the base right and pitching it up requires unyawing, but
    # less than 90 degrees, and tilt less than 20; as pitch increases
    # past 45 degrees, pan becomes responsible for elevation and tilt
    # for azimuth
    att = RPY(0, 30, 90)
    assert str(compose(att, azel)) == "p: -79.7 t: 17.2"
