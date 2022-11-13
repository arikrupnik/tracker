
import threading, queue
import time
import math
from scipy.spatial.transform import Rotation as R
import pymap3d
import pytest

import euler_conventions

import serial
import pymavlink
from pymavlink import mavutil
import visca

# from pymavlink.dialects.v20 import ardupilotmega as mavlink2

class Location():
    def __init__(self, lat, lon, alt):
        self.lat = lat
        self.lon = lon
        self.alt = alt
    def __str__(self):
        return f"{self.lat:11.7f} {self.lon:12.7f} {self.alt:6.1f}"

class Tracker():

    def __init__(self, camera):
        self.att         = None  # attitude of the base relative to
                                 # horizon and North
        self.target_azel = None  # target azimuth and elevatoin,
                                 # relative to horizon and North
        # I keep the two attributes above as `euler_conventions'
        # objects rather than rotation matrices to retain information
        # about their intents. Additionally, displaying original
        # `azimuth' or `roll' can be useful in debugging or UI.
        self.loc = None

        self.camera = camera

    def attitude(self, att=None):
        if att is None:
            return self.att
        else:
            self.att = att
            if self.target_azel is None:
                # Until I receive a target azimuth and elevation from
                # upstream, I want the camera to point in the
                # direction the IMU points at bootup. This conditional
                # is true only once per instantiation of this class.
                self.target_azel = euler_conventions.AzEl(r=att.r)
            gimbal_rm = self.target_azel.r * att.r.inv()
            self.camera.point(gimbal_rm)

    def location(self, loc=None):
        if loc is None:
            return self.loc
        else:
            self.loc = loc

class MAVLinkIMUAdapter():
    # useful now: ATTITUDE(radians; aeronautical frame (right-handed, Z-down, X-front, Y-right))
    # # While above seems to imply ZXY, "The Euler angles follow the convention of a 3-2-1 intrinsic Tait-Bryan rotation sequence."
    # useful now: GLOBAL_POSITION_INT(degE7/mm; GPS-frame (right-handed, Z-up))
    # useful now: GPS_RAW_INT: has GPS quality, fix type
    # useful later: HEARTBEAT(.type==5->MAV_TYPE_ANTENNA_TRACKER) POWER_STATUS SYS_STATUS(includes power) SCALED_PRESSURE STATUSTEXT MCU_STATUS(#11039, comes up as BAD_DATA, has MCU temp)
    # other: BAD_DATA EKF_STATUS_REPORT NAV_CONTROLLER_OUTPUT RC_CHANNELS SERVO_OUTPUT_RAW RAW_IMU SCALED_IMU2 AHRS(Status of DCM attitude estimator) AHRS2(Status of secondary AHRS filter) PARAM_VALUE MEMINFO SYSTEM_TIME TIMESYNC
    # declination = degrees(mavutil.mavfile_global.param('COMPASS_DEC', 0))

    def __init__(self, serial_path, serial_baud, att_callback, loc_callback):

        imu_link = mavutil.mavlink_connection(serial_path, serial_baud)

        self.att_callback = att_callback
        self.loc_callback = loc_callback

        def mavlink_listener():
            while True:
                p = imu_link.recv_match(blocking=True)
                t = p.get_type()
                if t == "ATTITUDE":
                    roll, pitch, yaw = map(math.degrees, (p.roll, p.pitch, p.yaw))
                    self.att_callback(euler_conventions.RPY(roll, pitch, yaw))
                if t == "GLOBAL_POSITION_INT":
                    lat, lon = map(lambda n: n/10**7, (p.lat, p.lon))
                    alt = p.alt/10**3
                    self.loc_callback(Location(lat, lon, alt))

        threading.Thread(target=mavlink_listener, name="MAVLink", daemon=True).start()

class GimbalCamera():
    """Abstraction of a camera on a gimbal."""
    def point(self, rotation_matrix, fov=None, focus_distance=None):
        raise NotImplementedError()

class VISCAAdapter():
    """Concrete implementation of `GimbalCamera', using a VISCA camera as
       a backend."""
    def __init__(self, serial_path):
        self.net = visca.Network(serial_path)
        self.rotation_matrix = None
        def att_listener():
            c = self.net.cameras[0]
            while True:
                if self.rotation_matrix is not None:
                    pt = euler_conventions.PT(r=self.rotation_matrix)
                    pan = pt.pan
                    tilt = pt.tilt
                    self.net.send_packet(c.PanTiltDriveAbsolutePosition(pan, tilt))
                    self.rotation_matrix = None
                else:
                    time.sleep(0.01)
        threading.Thread(target=att_listener, name="VISCA", daemon=True).start()
    def point(self, rotation_matrix, fov=None, focus_distance=None):
        self.rotation_matrix = rotation_matrix

if __name__ == "__main__":
    camera = VISCAAdapter("/dev/ttyUSB0")
    tracker = Tracker(camera)
    imu_adapter = MAVLinkIMUAdapter("/dev/ttyACM0", 115200,
                                    tracker.attitude, tracker.location)

    while True:
        time.sleep(0.5)
        #print(f"a: {tracker.attitude()}  l:{tracker.location()}")
