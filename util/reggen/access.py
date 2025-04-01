# Copyright lowRISC contributors (OpenTitan project).
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
"""Enumerated types for fields
Generated by validation, used by backends
"""

from enum import Enum

from reggen.lib import check_str


class JsonEnum(Enum):

    def for_json(x) -> str:
        return str(x)


class SwWrAccess(JsonEnum):
    WR = 1
    NONE = 2


class SwRdAccess(JsonEnum):
    RD = 1
    RC = 2  # Special handling for port
    NONE = 3


class SwAccess(JsonEnum):
    RO = 1
    RW = 2
    WO = 3
    W1C = 4
    W1S = 5
    W0C = 6
    RC = 7
    R0W1C = 8
    NONE = 9


class HwAccess(JsonEnum):
    HRO = 1
    HRW = 2
    HWO = 3
    NONE = 4  # No access allowed


# swaccess permitted values
# text description, access enum, wr access enum, rd access enum, ok in window
# yapf: disable
SWACCESS_PERMITTED = {
    'none':  ("No access",                                              # noqa: E241
              SwAccess.NONE, SwWrAccess.NONE, SwRdAccess.NONE, False),  # noqa: E241
    'ro':    ("Read Only",                                              # noqa: E241
              SwAccess.RO,   SwWrAccess.NONE, SwRdAccess.RD,   True),   # noqa: E241
    'rc':    ("Read Only, reading clears",                              # noqa: E241
              SwAccess.RC,   SwWrAccess.WR,   SwRdAccess.RC,   False),  # noqa: E241
    'rw':    ("Read/Write",                                             # noqa: E241
              SwAccess.RW,   SwWrAccess.WR,   SwRdAccess.RD,   True),   # noqa: E241
    'r0w1c': ("Read zero, Write with 1 clears",                         # noqa: E241
              SwAccess.W1C,  SwWrAccess.WR,   SwRdAccess.NONE, False),  # noqa: E241
    'rw1s':  ("Read, Write with 1 sets",                                # noqa: E241
              SwAccess.W1S,  SwWrAccess.WR,   SwRdAccess.RD,   False),  # noqa: E241
    'rw1c':  ("Read, Write with 1 clears",                              # noqa: E241
              SwAccess.W1C,  SwWrAccess.WR,   SwRdAccess.RD,   False),  # noqa: E241
    'rw0c':  ("Read, Write with 0 clears",                              # noqa: E241
              SwAccess.W0C,  SwWrAccess.WR,   SwRdAccess.RD,   False),  # noqa: E241
    'wo':    ("Write Only",                                             # noqa: E241
              SwAccess.WO,   SwWrAccess.WR,   SwRdAccess.NONE, True)    # noqa: E241
}
# yapf: enable

# hwaccess permitted values
HWACCESS_PERMITTED = {
    'hro': ("Read Only", HwAccess.HRO),
    'hrw': ("Read/Write", HwAccess.HRW),
    'hwo': ("Write Only", HwAccess.HWO),
    'none': ("No Access Needed", HwAccess.NONE)
}


class SWAccess:

    def __init__(self, where: str, raw: object, is_mubi: bool = False):
        self.key = check_str(raw, 'swaccess for {}'.format(where))
        self.is_mubi = is_mubi
        try:
            self.value = SWACCESS_PERMITTED[self.key]
        except KeyError:
            raise ValueError(
                f'Unknown swaccess key, {self.key}, for {where}.') from None

    def dv_rights(self) -> str:
        '''Return a UVM access string as used by uvm_field::set_access().'''
        if self.key == 'r0w1c':
            return 'W1C'
        elif self.key in ['rw1s', 'rw1c', 'rw0c', 'r0w1c', 'rc'
                          ] and self.is_mubi:
            # The native UVM implementations for these access modes are
            # incompatible with Mubi's. We hence have to use regular RW access
            # at the lowest level for the UVM RAL implementation, and model the
            # custom mubi behavior in the uvm_base_reg_field abstraction of
            # uvm_base_reg_field.
            return 'RW'
        return str(self.value[1].name)

    def dv_mubi_rights(self) -> str:
        '''Return a UVM access string as used by uvm_base_reg_field abstraction.
        '''
        if not self.is_mubi:
            return 'NONE'
        elif self.key == 'r0w1c':
            return 'W1C'

        return str(self.value[1].name)

    def swrd(self) -> SwRdAccess:
        return self.value[3]

    def allows_read(self) -> bool:
        return self.value[3] != SwRdAccess.NONE

    def allows_write(self) -> bool:
        return self.value[2] == SwWrAccess.WR

    def needs_we(self) -> bool:
        '''Should the register for this field have a write-enable signal?

        This is almost the same as allows_write(), but doesn't return true for
        RC registers, which should use a read-enable signal (connected to their
        prim_subreg's we port).

        '''
        return self.value[1] != SwAccess.RC and self.allows_write()

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, SWAccess):
            return NotImplemented
        return (self.key == other.key) and (self.value == other.value)

    def __str__(self) -> str:
        return '{}: {}'.format(self.key, self.value)


class HWAccess:

    def __init__(self, where: str, raw: object):
        self.key = check_str(raw, 'hwaccess for {}'.format(where))
        try:
            self.value = HWACCESS_PERMITTED[self.key]
        except KeyError:
            raise ValueError(
                f'Unknown hwaccess key, {self.key}, for {where}.') from None

    def allows_read(self) -> bool:
        return self.key in ['hro', 'hrw']

    def allows_write(self) -> bool:
        return self.key in ['hrw', 'hwo']

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, HWAccess):
            return NotImplemented
        return (self.key == other.key) and (self.value == other.value)

    def __str__(self) -> str:
        return '{}: {}'.format(self.key, self.value)
