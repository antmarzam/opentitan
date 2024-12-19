# Copyright lowRISC contributors (OpenTitan project).
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

"""Rules to describe OpenTitan HW"""

def opentitan_ip(name, hjson):
    """
    Return a structure describing an IP. This can be given to opentitan_top.

    Arguments:
    - name: name of ip in lower case.
    - hjson: label of the IP's hjson path, you MUST NOT use a relative label.
    - alias: label of an alias file.
    """
    return struct(
        name = name,
        hjson = hjson,
    )

def opentitan_top(name, hjson, top_lib, top_ld, ips):
    """
    Return a structure describing a top.

    Arguments:
    - name: name of top in lower case.
    - hjson: label of the top's hjson path (generated by topgen), you MUST NOT
             use a relative label.
    - top_lib: same but for the top's library.
    - top_ld: same but for the top's linker script.
    - ips: array of ips, the entries must be built by opentitan_ip().
    """
    return struct(
        name = name,
        hjson = hjson,
        top_lib = top_lib,
        top_ld = top_ld,
        ips = ips,
    )
