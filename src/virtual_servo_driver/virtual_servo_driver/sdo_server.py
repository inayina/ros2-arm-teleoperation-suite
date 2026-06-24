"""Minimal CANopen SDO server backed by a small object dictionary."""

import struct


class ObjectDictionary:
    def __init__(self):
        self._od = {
            (0x6040, 0x00): 0,        # controlword
            (0x6041, 0x00): 0x0040,   # statusword (Switch On Disabled)
            (0x6060, 0x00): 10,       # mode of operation: cyclic sync torque
            (0x6071, 0x00): 0,        # target torque
            (0x6072, 0x00): 87000,    # max torque (milli-rated)
            (0x6064, 0x00): 0,        # actual position
            (0x606C, 0x00): 0,        # actual velocity
            (0x6077, 0x00): 0,        # actual torque
            (0x1017, 0x00): 1000,     # heartbeat producer time (ms)
            (0x6085, 0x00): 100000,   # quick-stop deceleration
        }

    def read(self, index: int, subindex: int = 0x00):
        return self._od.get((index, subindex))

    def write(self, index: int, value, subindex: int = 0x00) -> bool:
        self._od[(index, subindex)] = value
        return True

    def set_statusword(self, value: int):
        self._od[(0x6041, 0x00)] = value & 0xFFFF

    def set_feedback(self, position_counts: int, velocity: int, torque: int):
        self._od[(0x6064, 0x00)] = position_counts
        self._od[(0x606C, 0x00)] = velocity
        self._od[(0x6077, 0x00)] = torque


class SdoServer:
    """Handle expedited SDO download/upload requests."""

    def __init__(self, node_id: int, od: ObjectDictionary | None = None,
                 on_controlword=None, on_max_torque=None):
        self.node_id = node_id
        self.od = od or ObjectDictionary()
        self.on_controlword = on_controlword
        self.on_max_torque = on_max_torque

    def handle(self, data: bytes) -> bytes | None:
        if len(data) < 4:
            return self._abort(0x05040001)

        cmd = data[0]
        index = data[1] | (data[2] << 8)
        sub = data[3]

        if cmd == 0x40:
            val = self.od.read(index, sub)
            if val is None:
                return self._abort(0x06020000)
            return self._upload_response(index, sub, val)

        if cmd in (0x2F, 0x2B, 0x23):
            size = {0x2F: 1, 0x2B: 2, 0x23: 4}[cmd]
            raw = int.from_bytes(data[4:4 + size], "little", signed=False)
            if index == 0x6040 and self.on_controlword:
                self.on_controlword(raw & 0xFFFF)
            if index == 0x6072 and self.on_max_torque:
                self.on_max_torque(raw)
            self.od.write(index, raw, sub)
            return bytes([0x60, data[1], data[2], data[3], 0, 0, 0, 0])

        return self._abort(0x05040001)

    def _upload_response(self, index: int, sub: int, value: int) -> bytes:
        if (index, sub) in ((0x6064, 0x00), (0x606C, 0x00)):
            payload = struct.pack("<I", value & 0xFFFFFFFF)
            return bytes([0x43, index & 0xFF, (index >> 8) & 0xFF, sub]) + payload
        if value < 0:
            payload = struct.pack("<h", value)
            return bytes([0x4B, index & 0xFF, (index >> 8) & 0xFF, sub]) + payload + b"\x00\x00"
        if value <= 0xFF:
            return bytes([0x4F, index & 0xFF, (index >> 8) & 0xFF, sub, value & 0xFF, 0, 0, 0])
        payload = struct.pack("<H", value & 0xFFFF)
        return bytes([0x4B, index & 0xFF, (index >> 8) & 0xFF, sub]) + payload + b"\x00\x00"

    def _abort(self, code: int) -> bytes:
        return struct.pack("<BIII", 0x80, 0, 0, code)[:8]
