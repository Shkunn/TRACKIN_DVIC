#!/usr/bin/env python
#
# Serial port enumeration. Console tool and backend selection.
#
# This file is part of pySerial. https://github.com/pyserial/pyserial
# (C) 2011-2015 Chris Liechti <cliechti@gmx.net>
#
# SPDX-License-Identifier:    BSD-3-Clause

"""
This module will provide a function called comports that returns an
iterable (generator or list) that will enumerate available com ports. Note that
on some systems non-existent ports may be listed.

Additionally a grep function is supplied that can be used to search for ports
based on their descriptions or hardware ID.
"""

from __future__ import absolute_import

import sys
import os
import re

# chose an implementation, depending on os
#~ if sys.platform == 'cli':
#~ else:
if os.name == 'nt':  # sys.platform == 'win32':
    from serial.tools.list_ports_windows import comports
elif os.name == 'posix':
    from serial.tools.list_ports_posix import comports
#~ elif os.name == 'java':
else:
    raise ImportError("Sorry: no implementation for your platform ('{}') available".format(os.name))

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def get_usb():

    hits = 0
    # get iteraror w/ or w/o filter
    iterator = sorted(comports(include_links=True))
    # list them
    for n, (port, desc, hwid) in enumerate(iterator, 1):
        return f"{port}"



# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# test
"""
if __name__ == '__main__':
    get_usb()"""
