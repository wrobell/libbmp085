#
# bmp085 - BMP085 pressure sensor library
#
# Copyright (C) 2013 by Artur Wroblewski <wrobell@pld-linux.org>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

import ctypes as ct

class Sensor(object):
    """
    BMP085 sensor communication interface.
    """
    def __init__(self):
        """
        Create pressure sensor communication interface.
        """
        self._lib = ct.CDLL('libbmp085.so.0')
        self._lib.bmp085_init()


    def read_pressure(self):
        """
        Read temperature from sensor.
        """
        return self._lib.bmp085_read_pressure()


    def read_temp(self):
        """
        Read temperature from sensor.
        """
        return self._lib.bmp085_read_temp()

# vim: sw=4:et:ai
