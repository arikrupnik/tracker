
import threading, queue
import time
import math
from scipy.spatial.transform import Rotation as R
import pymap3d.aer
import pytest

import euler_conventions

import serial
import pymavlink
from pymavlink import mavutil
import visca

# from pymavlink.dialects.v20 import ardupilotmega as mavlink2

class Location():
    def __init__(self, lat, lon, alt):
        self.lat = lat  # degrees
        self.lon = lon  # degrees
        self.alt = alt  # meters
    def __str__(self):
        return f"{self.lat:11.7f} {self.lon:12.7f} {self.alt:6.1f}"

def ptz_callback(pt, fov=None, focus_distance=None):
    """Callback on the camera. Tracker constructor takes a function with
       this signature as an argument and calls it when it receives
       valid information about its own location and attitude and the
       location of its target"""
    raise NotImplementedError()

def att_callback(rpy):
    """Callback on the tracker. IMU calls a function with this signature
       when it has an update on the attitude of the base."""
    raise NotImplementedError()

def loc_callback(location):
    """Callback on the tracker. IMU calls a function with this signature
       when it has an update on the location of the base. Telemetry
       listener calls a function with this signature when it has an
       update on the location of the target."""
    raise NotImplementedError()

class Tracker():

    def __init__(self, ptz_callback):
        self.att        = None  # attitude of the base relative to
                                # horizon and North
        self.loc        = None  # location of base
        self.target_loc = None  # where we want to look

        self.ptz_callback = ptz_callback

    def _callback_if(self):
        if self.att and self.loc and self.target_loc:
            az, el, distance = pymap3d.aer.geodetic2aer(
                self.target_loc.lat, self.target_loc.lon, self.target_loc.alt,
                self.loc.lat, self.loc.lon, self.loc.alt)
            target_azel = euler_conventions.AzEl(az, el)
            pt = euler_conventions.necessary_pt(self.att, target_azel)
            print(self.loc, self.target_loc, pt, distance)
            self.ptz_callback(pt, None, distance)

    def attitude(self, att):
        self.att = att
        self._callback_if()

    def location(self, loc):
        self.loc = loc
        self._callback_if()

    def target_location(self, loc):
        self.target_loc = loc
        self._callback_if()

def test_sanity():
    pt = None
    fov = None
    distance = None
    def c(arg_pt, arg_fov=None, arg_distance=None):
        nonlocal pt, fov, distance
        pt = arg_pt
        fov = arg_fov
        distance = arg_distance
    t = Tracker(c)
    t.attitude(euler_conventions.RPY(0, 0, 0))  # pointing North
    t.location(Location(0, 0, 0))               # mid-Atlantic
    t.target_location(Location(1, 1, 0))        # one degree North and East
    # very crude approximation of the shape as a square 1 arc-degree, or 60 NM, on the side
    assert pt.pan == pytest.approx(45, rel=0.005)
    assert distance == pytest.approx(1852 * 60 * math.sqrt(2), rel=0.005)
    assert pt.tilt == pytest.approx(-0.707, rel=0.005)  # slightly below curvature of the Earth
    t.attitude(euler_conventions.RPY(0, 0, 90))
    assert pt.pan == pytest.approx(-45, rel=0.005)


class MAVLinkIMUAdapter():
    # useful now: ATTITUDE(radians; ZYX, intrinsic)
    # useful now: GLOBAL_POSITION_INT(degE7/mm; GPS-frame (right-handed, Z-up))
    # useful now: GPS_RAW_INT: has GPS quality, fix type
    # useful later: HEARTBEAT(.type==5->mavlink.MAV_TYPE_ANTENNA_TRACKER) POWER_STATUS SYS_STATUS(includes power) SCALED_PRESSURE STATUSTEXT MCU_STATUS(has MCU temp)
    # other: BAD_DATA EKF_STATUS_REPORT NAV_CONTROLLER_OUTPUT RC_CHANNELS SERVO_OUTPUT_RAW RAW_IMU SCALED_IMU2 AHRS(Status of DCM attitude estimator) AHRS2(Status of secondary AHRS filter) PARAM_VALUE MEMINFO SYSTEM_TIME TIMESYNC

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

        threading.Thread(target=mavlink_listener, name="MAVLink-IMU", daemon=True).start()

class MAVLinkTelemetryAdapter():

    def __init__(self, serial_path, serial_baud, loc_callback):
        telemetry_link = mavutil.mavlink_connection(serial_path, serial_baud)

        self.loc_callback = loc_callback

        def mavlink_listener():
            while True:
                p = telemetry_link.recv_match(blocking=True)
                t = p.get_type()
                if t == "GLOBAL_POSITION_INT":
                    lat, lon = map(lambda n: n/10**7, (p.lat, p.lon))
                    alt = p.alt/10**3
                    self.loc_callback(Location(lat, lon, alt))

        threading.Thread(target=mavlink_listener, name="MAVLink-Telemetry", daemon=True).start()


class VISCAAdapter():
    """Concrete implementation of `GimbalCamera', using a VISCA camera as
       a backend."""
    def __init__(self, serial_path):
        self.net = visca.Network(serial_path)
        self.pt = None
        def pt_listener():
            c = self.net.cameras[0]
            while True:
                if self.pt is not None:
                    pt = self.pt
                    self.pt = None
                    pan = pt.pan
                    tilt = pt.tilt
                    self.net.send_packet(c.PanTiltDriveAbsolutePosition(pan, tilt))
                else:
                    time.sleep(0.01)
        threading.Thread(target=pt_listener, name="VISCA", daemon=True).start()
    def ptz(self, pt, fov=None, focus_distance=None):
        self.pt = pt

if __name__ == "__main__":
    camera = VISCAAdapter("/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0")
    tracker = Tracker(camera.ptz)
    imu_adapter = MAVLinkIMUAdapter("/dev/serial/by-id/usb-ArduPilot_MatekH743_1D0024000651303230373534-if00",
                                    115200, tracker.attitude, tracker.location)
    telemetry = MAVLinkTelemetryAdapter("/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_D307DA85-if00-port0",
                                        57600, tracker.target_location)

    try:
        while True:
            time.sleep(0.5)
            #print(f"a: {tracker.attitude()}  l:{tracker.location()}")
    except KeyboardInterrupt:
        print()
