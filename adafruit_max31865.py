# The MIT License (MIT)
#
# Copyright (c) 2017 Tony DiCola for Adafruit Industries
# Copyright (c) 2018 Esteban A. Bosse H.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""
`adafruit_max31865`
====================================================

Python3 module for the MAX31865 platinum RTD temperature sensor in BeagleBone and PocketBeagle.  See
examples/simpletest.py for an example of the usage.

* Author(s): Tony DiCola, Esteban A. Bosse H.

Implementation Notes
--------------------

**Hardware:**

* Adafruit `Universal Thermocouple Amplifier MAX31856 Breakout
  <https://www.adafruit.com/product/3263>`_ (Product ID: 3263)

* Adafruit `PT100 RTD Temperature Sensor Amplifier - MAX31865
  <https://www.adafruit.com/product/3328>`_ (Product ID: 3328)

* Adafruit `PT1000 RTD Temperature Sensor Amplifier - MAX31865
  <https://www.adafruit.com/product/3648>`_ (Product ID: 3648)

"""
import math
import time
import logging


_MAX31865_CONFIG_REG          = 0x00
_MAX31865_CONFIG_BIAS         = 0x80
_MAX31865_CONFIG_MODEAUTO     = 0x40
_MAX31865_CONFIG_MODEOFF      = 0x00
_MAX31865_CONFIG_1SHOT        = 0x20
_MAX31865_CONFIG_3WIRE        = 0x10
_MAX31865_CONFIG_24WIRE       = 0x00
_MAX31865_CONFIG_FAULTSTAT    = 0x02
_MAX31865_CONFIG_FILT50HZ     = 0x01
_MAX31865_CONFIG_FILT60HZ     = 0x00
_MAX31865_RTDMSB_REG          = 0x01
_MAX31865_RTDLSB_REG          = 0x02
_MAX31865_HFAULTMSB_REG       = 0x03
_MAX31865_HFAULTLSB_REG       = 0x04
_MAX31865_LFAULTMSB_REG       = 0x05
_MAX31865_LFAULTLSB_REG       = 0x06
_MAX31865_FAULTSTAT_REG       = 0x07
_MAX31865_FAULT_HIGHTHRESH    = 0x80
_MAX31865_FAULT_LOWTHRESH     = 0x40
_MAX31865_FAULT_REFINLOW      = 0x20
_MAX31865_FAULT_REFINHIGH     = 0x10
_MAX31865_FAULT_RTDINLOW      = 0x08
_MAX31865_FAULT_OVUV          = 0x04
_RTD_A = 3.9083e-3
_RTD_B = -5.775e-7

logging.basicConfig(level=logging.DEBUG)


class MAX31865:
    def __init__(self, spi, rtd_nominal=100, ref_resistor=430.0, wires=2):
        self.rtd_nominal = rtd_nominal
        self.ref_resistor = ref_resistor
        self._spi = spi

        # Set wire config register based on the number of wires specified.
        if wires not in (2, 3, 4):
            raise ValueError('Wires must be a value of 2, 3, or 4!')
        config = self._read_u8(_MAX31865_CONFIG_REG)
        if wires == 3:
            config |= _MAX31865_CONFIG_3WIRE
        else:
            # 2 or 4 wire
            config &= ~_MAX31865_CONFIG_3WIRE
        self._write_u8(_MAX31865_CONFIG_REG, config)

        # Default to no bias and no auto conversion.
        self.bias = False
        self.auto_convert = False

    def _read_u8(self, address):
        value = self._spi.xfer2([address & 0x7F, 0])
        return value[1]

    def _read_u16(self, address):
        value = self._spi.xfer2([address & 0x7F, 0, 0])
        return (value[1] << 8) | value[2]

    def _write_u8(self, address, val):
        self._spi.xfer2([(address | 0x80) & 0xFF, val & 0xFF])

    @property
    def bias(self):
        """The state of the sensor's bias (True/False)."""
        return bool(self._read_u8(_MAX31865_CONFIG_REG) & _MAX31865_CONFIG_BIAS)

    @bias.setter
    def bias(self, val):
        config = self._read_u8(_MAX31865_CONFIG_REG)
        if val:
            config |= _MAX31865_CONFIG_BIAS  # Enable bias.
        else:
            config &= ~_MAX31865_CONFIG_BIAS  # Disable bias.
        self._write_u8(_MAX31865_CONFIG_REG, config)

    @property
    def auto_convert(self):
        """The state of the sensor's automatic conversion
        mode (True/False).
        """
        return bool(self._read_u8(_MAX31865_CONFIG_REG) & _MAX31865_CONFIG_MODEAUTO)

    @auto_convert.setter
    def auto_convert(self, val):
        config = self._read_u8(_MAX31865_CONFIG_REG)
        if val:
            config |= _MAX31865_CONFIG_MODEAUTO   # Enable auto convert.
        else:
            config &= ~_MAX31865_CONFIG_MODEAUTO  # Disable auto convert.
        self._write_u8(_MAX31865_CONFIG_REG, config)

    def fault(self):
        """The fault state of the sensor.  Use ``clear_faults()`` to clear the
        fault state.  Returns a 6-tuple of boolean values which indicate if any
        faults are present:

        - HIGHTHRESH
        - LOWTHRESH
        - REFINLOW
        - REFINHIGH
        - RTDINLOW
        - OVUV
        """
        faults = self._read_u8(_MAX31865_FAULTSTAT_REG)
        highthresh = bool(faults & _MAX31865_FAULT_HIGHTHRESH)
        lowthresh  = bool(faults & _MAX31865_FAULT_LOWTHRESH)
        refinlow   = bool(faults & _MAX31865_FAULT_REFINLOW)
        refinhigh  = bool(faults & _MAX31865_FAULT_REFINHIGH)
        rtdinlow   = bool(faults & _MAX31865_FAULT_RTDINLOW)
        ovuv       = bool(faults & _MAX31865_FAULT_OVUV)
        return (highthresh, lowthresh, refinlow, refinhigh, rtdinlow, ovuv)

    def clear_faults(self):
        """Clear any fault state previously detected by the sensor."""
        config = self._read_u8(_MAX31865_CONFIG_REG)
        config &= ~0x2C
        config |= _MAX31865_CONFIG_FAULTSTAT
        self._write_u8(_MAX31865_CONFIG_REG, config)

    def read_rtd(self):
        """Perform a raw reading of the thermocouple and return its 15-bit
        value.  You'll need to manually convert this to temperature using the
        nominal value of the resistance-to-digital conversion and some math.  If you just want
        temperature use the temperature property instead.
        """
        self.clear_faults()
        self.bias = True
        time.sleep(0.01)
        config = self._read_u8(_MAX31865_CONFIG_REG)
        config |= _MAX31865_CONFIG_1SHOT
        self._write_u8(_MAX31865_CONFIG_REG, config)
        time.sleep(0.065)
        rtd = self._read_u16(_MAX31865_RTDMSB_REG)
        # Remove fault bit.
        rtd >>= 1
        return rtd

    def get_resistance(self):
        """Read the resistance of the RTD and return its value in Ohms."""
        resistance = self.read_rtd()
        resistance /= 32768
        resistance *= self.ref_resistor
        logging.info("Resistance value(ohms): {0:0.3f}".format(resistance))
        return resistance

    def get_temperature(self):
        def convert_resistance_to_temperature_with_sub_zero_convertion(resistance):
            rpoly = resistance
            temperature = -242.02
            temperature += 2.2228 * rpoly
            rpoly *= resistance  # square
            temperature += 2.5859e-3 * rpoly
            rpoly *= resistance  # ^3
            temperature -= 4.8260e-6 * rpoly
            rpoly *= resistance  # ^4
            temperature -= 2.8183e-8 * rpoly
            rpoly *= resistance  # ^5
            temperature += 1.5243e-10 * rpoly
            return temperature

        def convert_resistance_to_temperature_with_over_zero_convertion(resistance):
            Z1 = -_RTD_A
            Z2 = _RTD_A * _RTD_A - (4 * _RTD_B)
            Z3 = (4 * _RTD_B) / self.rtd_nominal
            Z4 = 2 * _RTD_B
            temperature = Z2 + (Z3 * resistance)
            temperature = (math.sqrt(temperature) + Z1) / Z4
            return temperature

        resistance = self.get_resistance()
        temperature = convert_resistance_to_temperature_with_over_zero_convertion(resistance)
        if temperature <= 0:
            temperature = convert_resistance_to_temperature_with_sub_zero_convertion(resistance)
        return temperature 
  
