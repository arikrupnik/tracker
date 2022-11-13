# euler_conventions.py: translating between common conventions for
# Euler angles

from scipy.spatial.transform import Rotation as R

class RPY():
    """Euler angles as Roll/Pitch/Yaw, extrinsic rotations with respect to
       North and horizon."""
    def __init__(self, roll=None, pitch=None, yaw=None, *, r=None):
        """With three arguments, construct a rotation matrix from roll, pitch
           and yaw angles, in degrees; with a single argument, treat
           it as a complete rotation matrix."""
        if r is None:
            self.r = R.from_euler("zyx", (yaw, pitch, roll), degrees=True)
            #self.r = R.from_euler("xyz", (roll, pitch, yaw), degrees=True)  # this moves the camera correctly but tests fail
        else:
            self.r = r
        self.yaw, self.pitch, self.roll = self.r.as_euler("zyx", degrees=True)
        # First angle belongs to [-180, 180] degrees (both inclusive)
        # Second angle belongs to [-90, 90] degrees if all axes are different (like xyz)
        # Third angle belongs to [-180, 180] degrees (both inclusive)
        # https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.as_euler.html
    def __str__(self):
        return f"r:{self.roll:6.1f} p:{self.pitch:5.1f} y:{self.yaw:6.1f}"

def test_RPY():
    assert str(RPY(r=R.from_euler("zyx", (10, 20, 30), degrees=True))) == \
                                      "r:  30.0 p: 20.0 y:  10.0"
    assert str(RPY( 35,  25, 115)) == "r:  35.0 p: 25.0 y: 115.0"
    assert str(RPY( 10,  20,  30)) == "r:  10.0 p: 20.0 y:  30.0"
    assert str(RPY(-10, -20, -30)) == "r: -10.0 p:-20.0 y: -30.0"
    assert str(RPY(180,  20,   0)) == "r: 180.0 p: 20.0 y:   0.0"
    # SciPy normalizes the representation:
    assert str(RPY(  0, 100,   0)) == "r: 180.0 p: 80.0 y:-180.0"

class AzEl():
    """Euler angles as Azimuth/Elevation, extrinsic rotations with respect
       to North and horizon."""
    def __init__(self, azimuth=None, elevation=None, *, r=None):
        if r is None:
            self.r = R.from_euler("zyx", (azimuth, elevation, 0), degrees=True)
        else:
            self.r = r
        self.azimuth, self.elevation, _ = self.r.as_euler("zyx", degrees=True)
    def __str__(self):
        return f"a:{self.azimuth:6.1f} e:{self.elevation:5.1f}"
def test_AzEl():
    assert str(AzEl(r=R.from_euler("zyx", (20, 30, 0), degrees=True))) == \
                                 "a:  20.0 e: 30.0"
    assert str(AzEl(  20,  30)) == "a:  20.0 e: 30.0"
    assert str(AzEl( 120,  30)) == "a: 120.0 e: 30.0"
    assert str(AzEl(-120, -30)) == "a:-120.0 e:-30.0"

class PT():
    """Euler angles as Pan/Tilt, intrisic rotations with respect to
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
    # Pan/Tilt is intrinsic, relative to base. If I feed it an
    # extrinsic argument, SciPy takes care of the translation.
    assert str(PT(r=R.from_euler("zyx", (20, 30, 0), degrees=True))) == \
                                 "p:  22.8 t: 28.0"
    assert str(PT(  20,  30)) == "p:  20.0 t: 30.0"
    assert str(PT( 120,  30)) == "p: 120.0 t: 30.0"
    assert str(PT(-120, -30)) == "p:-120.0 t:-30.0"

