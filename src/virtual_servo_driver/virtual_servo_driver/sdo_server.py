"""Minimal CANopen SDO server backed by a small object dictionary.

Scaffold supports expedited SDO read/write of the DS402 objects we use.
M2 task: wire this to real SDO frames (0x600/0x580 + node_id) on the bus.
"""


class ObjectDictionary:
    def __init__(self):
        # (index, subindex) -> value
        self._od = {
            (0x6040, 0x00): 0,        # controlword
            (0x6041, 0x00): 0x0040,   # statusword (Switch On Disabled)
            (0x6060, 0x00): 10,       # mode of operation: cyclic sync torque
            (0x6071, 0x00): 0,        # target torque
            (0x6064, 0x00): 0,        # actual position
            (0x606C, 0x00): 0,        # actual velocity
            (0x6077, 0x00): 0,        # actual torque
            (0x6085, 0x00): 100000,   # quick-stop deceleration
        }

    def read(self, index: int, subindex: int = 0x00):
        return self._od.get((index, subindex))

    def write(self, index: int, value, subindex: int = 0x00) -> bool:
        self._od[(index, subindex)] = value
        return True
