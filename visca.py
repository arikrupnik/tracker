
import threading, queue
import serial
import sys
import logging
import enum
import struct
import types, numbers
import pytest


class Network:

    def __init__(self, port_name):
        self.port = serial.Serial(port_name,
                                  baudrate = 9600,
                                  bytesize = serial.EIGHTBITS,
                                  parity   = serial.PARITY_NONE,
                                  stopbits = serial.STOPBITS_ONE)
        self.cameras = []
        self.response_packets_q = queue.Queue()

        def listener():
            buffer = b""
            while True:
                buffer += self.port.read(1)
                if buffer[-1]==0xFF:
                    packet = DevicePacket(buffer)
                    logging.debug(f"dev:{buffer.hex().ljust(8)}  | {packet}")
                    self.response_packets_q.put(packet)
                    buffer = b""
        threading.Thread(target=listener, daemon=True).start()

        # "Before starting control of the [cameras], be sure to send
        # the Address command and the IF_Clear command using the
        # broadcast function."
        self.send_packet(IfClear(BROADCAST_ADDR))
        devices = self.send_packet(AddressSet())
        if not devices:
            raise NetworkResponseError("0 cameras responding on network")
        for device in devices:
            dt = self.send_packet(CAMDeviceTypeInq(device.address))[0]
            camera_class = Camera.find_model(dt.vendor, dt.model)
            if camera_class:
                self.cameras.append(camera_class(device.address))
            else:
                # warn unknown camera
                pass

    def camera(self, address):
        """Camera with network address `address'. If no camera exists with
           this address, raises `CallerInputError'. Valid addresses
           are 1 through 7."""
        for c in self.cameras:
            if c.address == address:
                return c
        raise CallerInputError(f"no camera in network with address {address}")

    def send_packet(self, packet):
        logging.debug(f"ctl:{packet.bytes.hex()}")
        self.port.write(packet.bytes)
        self.port.flush()
        return packet.process_responses(self.response_packets_q)



CONTROLLER_ADDR = 0  # "The address of the controller is fixed at 0"
BROADCAST_ADDR  = 8

class ControllerPacket():

    class Type(enum.Enum):
        COMMAND = 0x01
        INQUIRY = 0x09

    class Category(enum.Enum):
        INTERFACE = 0x00
        CAMERA    = 0x04
        PAN_TILT  = 0x06

    def __init__(self, recver_addr, message, expected_responses=[]):
        if not (1 <= recver_addr <= 8):
            raise CallerInputError(f"address {recver_addr} out of range")
        header_byte = 0x80 | recver_addr
        self.bytes = bytes([header_byte]) + message + bytes([0xFF])
        self.expected_responses = expected_responses

    def process_responses(self, response_packets_q):
        responses = []
        for r in self.expected_responses:
            try:
                p = response_packets_q.get(timeout=3)
                if p.response_class != r:
                    raise NetworkResponseError(f"expected {r}, received {p}")
                responses.append(p)
            except queue.Empty:
                raise NetworkTimeout(f"timeout waiting for {r}") from None
        return responses

def test_ControllerPacket():
    assert ControllerPacket(1, b"").bytes == bytes([0x81, 0xFF])
    assert ControllerPacket(2, b"").bytes == bytes([0x82, 0xFF])
    assert ControllerPacket(8, b"").bytes == bytes([0x88, 0xFF])
    assert ControllerPacket(1, bytes.fromhex("00010203")).bytes == \
        bytes([0x81, 0x00, 0x01, 0x02, 0x03, 0xFF])
    with pytest.raises(CallerInputError):
        ControllerPacket(0, b"")
    with pytest.raises(CallerInputError):
        ControllerPacket(9, b"")

"""VISCA uses 0xFF as packet terminator. This means the byte 0xFF
cannot appear in a payload. Where arbitrary data may appear in the
payload, the protocol breaks up each byte into two, e.g., 0x7F becomes
0x070F. This function takes the number of 4-bit nibbles as an
argument. Some VISCA commands specify an odd number of nibbles,
discarding the high bits of the high byte."""
def escape_bytes(data, nibbles_to_escape):
    result = b""
    for n in range(nibbles_to_escape):
        nibble = (data >> (n*4)) & 0xF
        result = bytes([nibble]) + result
    return result
def test_escape_bytes():
    assert escape_bytes(0x00,       2) == bytes.fromhex("0000")
    assert escape_bytes(0x00,       4) == bytes.fromhex("00000000")
    assert escape_bytes(0x00,       8) == bytes.fromhex("0000000000000000")
    assert escape_bytes(0x1234,     4) == bytes.fromhex("01020304")
    assert escape_bytes(0x12345678, 8) == bytes.fromhex("0102030405060708")

class CommandPacket(ControllerPacket):
    def __init__(self, recver_addr, category, message):
        super().__init__(recver_addr,
                         bytes([ControllerPacket.Type.COMMAND.value, category.value]) + message,
                         [DevicePacket.ResponseClass.ACK,
                          DevicePacket.ResponseClass.COMPLETION])
def test_CommandPacket():
    assert CommandPacket(1, ControllerPacket.Category.PAN_TILT, b"").bytes == \
        bytes([0x81, 0x01, 0x06, 0xFF])

class InquiryPacket(ControllerPacket):
    def __init__(self, recver_addr, category, message):
        super().__init__(recver_addr,
                         bytes([ControllerPacket.Type.INQUIRY.value, category.value]) + message,
                         [DevicePacket.ResponseClass.COMPLETION])
def test_InquiryPacket():
    assert InquiryPacket(1, ControllerPacket.Category.PAN_TILT, b"").bytes == \
        bytes([0x81, 0x09, 0x06, 0xFF])

# this packet is a special case, neither a command nor inquiry,
# inherits directly from ControllerPacket
class AddressSet(ControllerPacket):
    def __init__(self):
        super().__init__(BROADCAST_ADDR,
                         bytes([0x30, 0x01]),
                         [DevicePacket.ResponseClass.NETWORK])
    def process_responses(self, response_packets_q):
        devices = []
        for i in range(7):
            try:
                p = response_packets_q.get(timeout=0.2)
                if p.response_class == DevicePacket.ResponseClass.NETWORK and \
                   p.recver_addr == BROADCAST_ADDR:
                    # consider raising if len(payload) != 1
                    p.address = p.payload[0] - 1
                    p.str_addendum = types.MethodType(lambda self: f" addr: {self.address}", p)
                    logging.debug(f"              | {p}")
                else:
                    raise NetworkResponseError(
                        f"expecting AddressSet response, have {p}")
                devices.append(p)
            except queue.Empty:
                break
        return devices
def testAddressSet():
    p = AddressSet()
    assert p.bytes == bytes([0x88, 0x30, 0x01, 0xFF])
    # test process_responses()
    response_packets_q = queue.Queue()
    response_packets_q.put(DevicePacket(bytes.fromhex("883002ff")))
    response_packets_q.put(DevicePacket(bytes.fromhex("883004ff")))
    packets = p.process_responses(response_packets_q)
    assert packets[0].address == 1
    assert packets[1].address == 3


# command packets
class IfClear(CommandPacket):
    def __init__(self, recver_addr):
        super().__init__(recver_addr, ControllerPacket.Category.INTERFACE, bytes([0x01]))
        # unusually for command packets, the response to IfClear
        # is only a completion in case of individual addressing or
        # rebroadcast in case of controller broadcast
        self.expected_responses = []
    def process_responses(self, response_packets_q):
        if self.bytes[0] ==  BROADCAST_ADDR << 4 | BROADCAST_ADDR:
            rebroadcasts = []
            for i in range(7):
                try:
                    p = response_packets_q.get(timeout=0.2)
                    if p.response_class == DevicePacket.ResponseClass.REBREOADCAST:
                        rebroadcasts.append(p)
                    else:
                        raise NetworkResponseError(
                            f"expecting IfClear rebroadcast, have {p}")
                except queue.Empty:
                    break
            return rebroadcasts
        else:
            self.expected_responses = [DevicePacket.ResponseClass.COMPLETION]
            return super().process_responses(response_packets_q)
def testIfClear():
    assert IfClear(1).bytes == bytes([0x81, 0x01, 0x00, 0x01, 0xFF])
    assert not IfClear(1).expected_responses
    assert IfClear(BROADCAST_ADDR).bytes == bytes([0x88, 0x01, 0x00, 0x01, 0xFF])

class PanTiltDriveAbsolutePosition(CommandPacket):
    def __init__(self, address, pan_pos, tilt_pos, pan_speed, tilt_speed):
        # 0x01: nudge
        # 0x02: AbsolutePosition
        # 0x03: RelativePosition
        # 0x04: Home
        # 0x05: Reset
        super().__init__(address, ControllerPacket.Category.PAN_TILT,
                         bytes([0x02, pan_speed, tilt_speed]) + \
                         escape_bytes(pan_pos, 4) + escape_bytes(tilt_pos, 4))
def test_PanTiltDriveAbsolutePosition():
    assert PanTiltDriveAbsolutePosition(1, 0, -360, 0x18, 0x14).bytes == \
        bytes.fromhex("810106021814000000000f0e0908ff")
    assert PanTiltDriveAbsolutePosition(1, 0,  360, 0x18, 0x14).bytes == \
        bytes.fromhex("8101060218140000000000010608ff")

# inquiry packets
class CAMDeviceTypeInq(InquiryPacket):
    def __init__(self, recver_addr):
        super().__init__(recver_addr, ControllerPacket.Category.INTERFACE, bytes([0x02]))
    def process_responses(self, response_packets_q):
        responses = super().process_responses(response_packets_q)
        r = responses[0]
        try:
            r.vendor, r.model, r.rom_ver, r.num_sockets = \
                struct.unpack(">HHHB", r.payload)
        except struct.error:
            raise NetworkResponseParseError(
                f"CAMDeviceTypeInq response payload length {len(r.payload)} instead of 7")

        return responses
def test_CAMDeviceTypeInq():
    assert CAMDeviceTypeInq(1).bytes == bytes.fromhex("81090002FF")


"""Packets from VISCA devices (cameras) on the network."""
class DevicePacket():

    class ResponseClass(enum.Enum):
        NETWORK_CHANGE = 0x38
        NETWORK    = 3
        ACK        = 4
        COMPLETION = 5
        ERROR      = 6
        REBREOADCAST = 0x8801  # not in spec, speial case for devices
                               # rebroadcasting broadcast commands

    class ErrorType(enum.Enum):
        MESSAGE_LENGTH_ERROR   = 0x01
        SYNTAX_ERROR           = 0x02
        COMMAND_BUFFER_FULL    = 0x03
        COMMAND_CANCELED       = 0x04
        NO_SOCKET              = 0x05
        COMMAND_NOT_EXECUTABLE = 0x41

    def __init__(self, buffer):
        if len(buffer) < 3 or len(buffer) > 16:
            raise NetworkResponseParseError(
                f"invalid packet length ({len(buffer)}) 0x{buffer.hex()} must be between 3 and 16")
        if buffer[-1] != 0xFF:
            raise NetworkResponseParseError(
                f"missing terminator 0xFF in 0x{buffer.hex()}")

        header = buffer[0]
        if header >> 7 != 1:
            raise NetworkResponseParseError(f"invalid device header {header:x}")
        self.sender_addr = (header >> 4) & 0b0111
        self.recver_addr = (header >> 0) & 0b1111

        if self.recver_addr not in [CONTROLLER_ADDR, BROADCAST_ADDR]:
            raise NetworkResponseParseError(
                f"invalid receiver address: {self.recver_addr:x} in device packet {buffer.hex()}")

        if buffer[1] == 0x38:
            # special case for Network Change packet:
            self.response_class = self.ResponseClass(buffer[1])
            self.socket = None
        elif buffer == bytes.fromhex("88010001FF"):
            # special case for devices rebroadcasting IF_Clear
            self.response_class = self.ResponseClass.REBREOADCAST
            self.socket = None
        else:
            response_class = buffer[1]>>4
            self.socket = buffer[1] & 0xF
            try:
                self.response_class = self.ResponseClass(response_class)
                self.error_type = None
            except ValueError:
                raise NetworkResponseParseError(
                    f"invalid response class {hex(response_class)} in 0x{buffer.hex()}")

        self.payload = buffer[2:-1]
        # Special case for errors. Most response payloads are specific
        # to requests; errors can occur in response to any request. I
        # handle them centrally here.
        if self.response_class == self.ResponseClass.ERROR:
            # consider raising if len(payload) != 1
            error_type = self.payload[0]
            try:
                self.error_type = self.ErrorType(error_type)
            except ValueError:
                raise NetworkResponseParseError(f"invalid error code 0x{hex(error_type)}")
            self.str_addendum = types.MethodType(lambda self: f" {self.error_type}", self)

    def __str__(self):
        s = f"fr:{self.sender_addr} to:{self.recver_addr} s:{self.socket} {self.response_class}"
        sa = self.str_addendum()
        if sa:
            s += sa
        elif self.payload:
            s += f" pl:0x{self.payload.hex()}"
        return s

    # hook for monkey-patching __str__() with interpretation of payload
    def str_addendum(self):
        return ""

def test_DevicePacket():
    with pytest.raises(NetworkResponseParseError):
        # too short
        DevicePacket(b"")
    with pytest.raises(NetworkResponseParseError):
        # too long
        DevicePacket(bytes.fromhex("0000"+"0000"+"0000"+"0000"+"0000"+"0000"+"0000"+"0000"+"00"))
    with pytest.raises(NetworkResponseParseError):
        # missing terminator
        DevicePacket(bytes.fromhex("0000"+"0000"+"0000"+"0000"+"0000"+"0000"+"0000"+"0000"))
    p = DevicePacket(bytes.fromhex("9038FF"))
    assert p.sender_addr == 1
    assert p.response_class == DevicePacket.ResponseClass.NETWORK_CHANGE
    assert p.socket == None
    p = DevicePacket(bytes.fromhex("883002FF"))
    assert p.sender_addr == 0
    assert p.recver_addr == BROADCAST_ADDR
    assert p.response_class == DevicePacket.ResponseClass.NETWORK
    assert p.socket == 0
    p = DevicePacket(bytes.fromhex("9041FF"))
    assert p.sender_addr == 1
    assert p.recver_addr == 0
    assert p.response_class == DevicePacket.ResponseClass.ACK
    assert p.socket == 1
    p = DevicePacket(bytes.fromhex("9050FF"))
    assert p.sender_addr == 1
    assert p.recver_addr == 0
    assert p.response_class == DevicePacket.ResponseClass.COMPLETION
    assert p.socket == 0
    # invalid response classes: 2, 7
    with pytest.raises(NetworkResponseParseError):
        p = DevicePacket(bytes.fromhex("9021FF"))
    with pytest.raises(NetworkResponseParseError):
        p = DevicePacket(bytes.fromhex("9071FF"))


"Caller passes out-of-range values into a local function"
class CallerInputError(ValueError):
    pass
"Cannot parse response bytes from network device"
class NetworkResponseParseError(RuntimeError):
    pass
"Well-formed packet from network device makes no sense in sequence of packets"
class NetworkResponseError(RuntimeError):
    pass
"Network device fails to respond within timeout"
class NetworkTimeout(RuntimeError):
    pass



"""VISCA camera abstraction. Subclasses keep information about hex
limits to pan, tilt, zoom and focus for specific models as well as
mappings to real-world values for these. Instances keep the camera's
network address."""
class Camera():
    @classmethod
    def _find_model(cls, vendor, model):
        for sc in cls.__subclasses__():
            if sc.vendor == vendor and sc.model == model:
                return sc
            else:
                ssc = sc.find_model(vendor, model)
                if ssc:
                    return ssc
        return None

    @classmethod
    def find_model(cls, vendor, model):
        try:
            return list(
                filter(
                    lambda c: c.vendor==vendor and c.model==model,
                    descendant_classes(cls)))[0]
        except IndexError:
            return None
    
    def __init__(self, address):
        self.address = address

    """Construct an absolute Pan/Tilt drive command packet, translating
       degrees to native steps according to the camera's model and
       using the address specific to this instance."""
    def PanTiltDriveAbsolutePosition(self, pan_degrees, tilt_degrees):
        return PanTiltDriveAbsolutePosition(self.address,
                                            self.pan_degrees.map_to_int(pan_degrees,
                                                                        self._pan_native),
                                            self.tilt_degrees.map_to_int(tilt_degrees,
                                                                         self._tilt_native),
                                            self._pan_speed_native.max,
                                            self._tilt_speed_native.max)

def descendant_classes(cls):
    subclasses = []
    for subclass in cls.__subclasses__():
        subclasses.append(subclass)
        subclasses.extend(descendant_classes(subclass))
    return subclasses

def test_find_model():
    assert Camera.find_model(-1, -1) == None
    assert Camera.find_model(0x0001, 0x0402) == EVI_D30
    assert Camera.find_model(0x0001, 0x040D) == EVI_D100
    # confirm we're finding all descendant camera classes
    D90 = Camera.find_model(0x0001, 0x050D)
    assert issubclass(D90, Camera)     # subclass,
    assert Camera not in D90.__bases__ # but indirect subclass


def test_all_camera_subclasses_valid():
    """verify that all subclasses of Camera have the minimum required
       class attributes. The base abstract class leaves it up to
       concrete classes to define these. This tests that they do."""
    for c in descendant_classes(Camera):
        assert c.vendor
        assert c.model
        assert c._pan_native
        assert c.pan_degrees
        assert c._pan_speed_native
        assert c.pan_speed_degrees_s
        assert c._tilt_native
        assert c.tilt_degrees
        assert c._tilt_speed_native
        assert c.tilt_speed_degrees_s

def signed_from_hex(hex):
    """Signed number from hex string. Python has no concept of fixed-width
       integers, and consequently no concept of signed integers. Most
       documentation for VISCA cameras specifies protocol limits,
       e.g., min and max values pan the camea acceps, as signed hex
       numbers. I want to use the same hex in code as it appears in
       documentation to avoid manual conversion errors. As a quick,
       some documentation specifies odd numbers of digits, e.g.,
       0xF000. This function interprets a hex string up to 16
       characters long (64 bits) as a signed integer."""
    padding_char = "f" if int(hex[0], 16) >> 3 else "0"
    str64 = hex.rjust(16, padding_char)
    return struct.unpack(">q", bytes.fromhex(str64))[0]
def test_signed_from_hex():
    assert signed_from_hex("1") ==   1
    assert signed_from_hex("0") ==   0
    assert signed_from_hex("F") ==  -1
    assert signed_from_hex("FED4") ==  -300
    assert signed_from_hex("012C") ==   300
    assert signed_from_hex("FA60") == -1440
    assert signed_from_hex("05A0") ==  1440
    # struct can handle up to 64 bits
    assert signed_from_hex("FFFFFFFFFFFFFFFF") == -1
    # more bits raises
    with pytest.raises(struct.error):
        signed_from_hex("FFFFFFFFFFFFFFFFFF")
    # odd number of characters (past 16) raises a different exception:
    with pytest.raises(ValueError):
        signed_from_hex("FFFFFFFFFFFFFFFFF")

"""Min-Middle-Max: range with mapping capability. On the protocol
side, cameras normally specify integer hex ranges for pan, tilt, zoom
and focus limits. In outward-facing documentation, cameras specify
these ranges in degrees and meters. This class encapsulates the
arithmetic for mapping one set of values to the other."""
class MMM():
    def __init__(self, min, middle, max):
        self.min, self.middle, self.max = map(
            float, map(
                lambda m: m if isinstance(m, numbers.Number) else signed_from_hex(m),
                (min, middle, max)))

    """Maps value in domain to value in range."""
    @classmethod
    def _linear_map(cls, d_min, d_max, d_value, r_min, r_max):
        return  r_min + ((d_value - d_min) * (r_max - r_min) / (d_max - d_min))

    """Map a value in this domain to a float value in other range"""
    def map_to(domain, d_value, range):
        if d_value < domain.middle:
            return domain._linear_map(domain.min,domain.middle,
                                      d_value,
                                      range.min,range.middle)
        else:
            return domain._linear_map(domain.middle,domain.max,
                                      d_value,
                                      range.middle,range.max)

    """Map a value in this domain to other range and round to nearest
       int. Useful for constructing VISCA messages that use integer
       values."""
    def map_to_int(self, self_value, other):
        return int(round(self.map_to(self_value, other)))

def test_linear_map():
    assert MMM._linear_map(0,10, 0, -1,0) == pytest.approx(-1.0)
    assert MMM._linear_map(0,10, 2, -1,0) == pytest.approx(-0.8)
    assert MMM._linear_map(0,10, 9, -1,0) == pytest.approx(-0.1)
    assert MMM._linear_map(0,10,10, -1,0) == pytest.approx( 0.0)
    # extrapolates past nominal range
    assert MMM._linear_map(0,10,-1, -1,0) == pytest.approx(-1.1)
    assert MMM._linear_map(0,10,11, -1,0) == pytest.approx( 0.1)

def test_map_to():
    # simple case from EVI-D100 documentation
    d100_pan_native = MMM("FA60", 0, "05A0")
    d100_pan_degrees = MMM(-100, 0, 100)
    assert d100_pan_degrees.map_to    (-100, d100_pan_native) == -1440
    assert d100_pan_degrees.map_to_int(-100, d100_pan_native) == -1440
    assert d100_pan_degrees.map_to    (   0, d100_pan_native) ==     0
    assert d100_pan_degrees.map_to_int(   0, d100_pan_native) ==     0
    assert d100_pan_degrees.map_to    ( 100, d100_pan_native) ==  1440
    assert d100_pan_degrees.map_to_int( 100, d100_pan_native) ==  1440

    # different ranges top and bottom
    native_mmm = MMM(-300, 0, 900)
    degrees_mmm = MMM(-30, 0,  90)
    assert degrees_mmm.map_to(-30, native_mmm) == -300
    assert degrees_mmm.map_to(-15, native_mmm) == -150
    assert degrees_mmm.map_to(  0, native_mmm) ==    0
    assert degrees_mmm.map_to( 45, native_mmm) ==  450
    assert degrees_mmm.map_to( 90, native_mmm) ==  900
    
    # different /ratios/ top and bottom
    native_mmm = MMM(-300, 0, 300)
    degrees_mmm = MMM(-30, 0,  90)
    assert degrees_mmm.map_to(-30, native_mmm) == -300
    assert degrees_mmm.map_to(-15, native_mmm) == -150
    assert degrees_mmm.map_to(  0, native_mmm) ==    0
    assert degrees_mmm.map_to( 45, native_mmm) ==  150
    assert degrees_mmm.map_to( 90, native_mmm) ==  300

    # different zeroes
    native_mmm = MMM(0, 100, 200)
    degrees_mmm = MMM(-30, 0, 30)
    assert degrees_mmm.map_to(-30, native_mmm) ==   0
    assert degrees_mmm.map_to(-15, native_mmm) ==  50
    assert degrees_mmm.map_to(  0, native_mmm) == 100
    assert degrees_mmm.map_to( 15, native_mmm) == 150
    assert degrees_mmm.map_to( 30, native_mmm) == 200

"""Min-Max range."""
class MM():
    def __init__(self, min, max):
        self.min = min
        self.max = max

class EVI_D30(Camera):
    vendor = 0x0001
    model  = 0x0402
    _pan_native = MMM("FC90", 0, "0370")
    pan_degrees = MMM(-100, 0, 100)
    _pan_speed_native = MM(0x01, 0x18)
    pan_speed_degrees_s = MM(3.3, 80.0)
    _tilt_native = MMM("FED4", 0, "012C")
    tilt_degrees = MMM(-25, 0, 25)
    _tilt_speed_native = MM(0x01, 0x14)
    tilt_speed_degrees_s = MM(2.5, 50.0)
def test_EVI_D30():
    e = EVI_D30
    assert e.pan_degrees.max-e.pan_degrees.min == 200
    assert e.tilt_degrees.max-e.tilt_degrees.min == 50

class EVI_D70(Camera):
    vendor = 0x0001
    model  = 0x040E
    _pan_native = MMM("F725", 0, "08DB")
    pan_degrees = MMM(-170, 0, 170)
    _pan_speed_native = MM(0x01, 0x18)
    pan_speed_degrees_s = MM(3.3, 80.0)
    _tilt_native = MMM("FE70", 0, "04B0")
    tilt_degrees = MMM(-30, 0, 90)
    _tilt_speed_native = MM(0x01, 0x17)
    tilt_speed_degrees_s = MM(2.5, 50.0)

class EVI_D80(Camera):
    vendor = 0x0001
    model  = 0x050C
    _pan_native = MMM("E1E5", 0, "1E1B")
    pan_degrees = MMM(-170, 0, 170)
    _pan_speed_native = MM(0x01, 0x18)
    pan_speed_degrees_s = MM(3.3, 80.0)
    _tilt_native = MMM("FC75", 0, "0FF0")
    tilt_degrees = MMM(-20, 0, 90)
    _tilt_speed_native = MM(0x01, 0x17)
    tilt_speed_degrees_s = MM(2.5, 50.0)

class EVI_D90(EVI_D80):
    # same as D80 but with 28x zoom instead of 18x for D80
    # TODO: update class attributes when I add zoom control
    model  = 0x050D
    
class EVI_D100(Camera):
    vendor = 0x0001
    model  = 0x040D
    _pan_native = MMM("FA60", 0, "05A0")
    pan_degrees = MMM(-100, 0, 100)
    _pan_speed_native = MM(0x01, 0x18)
    pan_speed_degrees_s = MM(2.0, 300)
    _tilt_native = MMM("FE98", 0, "0168")
    tilt_degrees = MMM(-25, 0, 25)
    _tilt_speed_native = MM(0x01, 0x14)
    tilt_speed_degrees_s = MM(2.0, 125)
def test_EVI_D100():
    e = EVI_D100
    assert e.pan_degrees.max-e.pan_degrees.min == 200
    assert e.tilt_degrees.max-e.tilt_degrees.min == 50


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    net = Network("/dev/ttyUSB0")
    logging.info(f" camera(s) on network: {list(net.cameras)}")
    c = net.cameras[0]
    net.send_packet(c.PanTiltDriveAbsolutePosition(   0,   0))
    net.send_packet(c.PanTiltDriveAbsolutePosition(   0, -25))
    net.send_packet(c.PanTiltDriveAbsolutePosition(   0,  25))
    net.send_packet(c.PanTiltDriveAbsolutePosition(   0,   0))
    net.send_packet(c.PanTiltDriveAbsolutePosition(   0, -10))
    net.send_packet(c.PanTiltDriveAbsolutePosition(   0,  10))
    net.send_packet(c.PanTiltDriveAbsolutePosition(   0,   0))
    net.send_packet(c.PanTiltDriveAbsolutePosition( -10,   0))
    net.send_packet(c.PanTiltDriveAbsolutePosition(  10,   0))
    net.send_packet(c.PanTiltDriveAbsolutePosition(   0,   0))

