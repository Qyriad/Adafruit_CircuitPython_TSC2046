# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2023 Qyriad for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_tsc2046`
================================================================================

CircuitPython library for the TI TSC2046 gyroscope


* Author(s): Qyriad

Implementation Notes
--------------------

**Hardware:**

.. todo:: Add links to any specific hardware product page(s), or category page(s).
  Use unordered list & hyperlink rST inline format: "* `Link Text <url>`_"

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

.. todo:: Uncomment or remove the Bus Device and/or the Register library dependencies
  based on the library's use of either.

 * Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

# imports
from typing import Optional
import math
import digitalio
import busio
from adafruit_bus_device.spi_device import SPIDevice
from micropython import const


__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/Qyriad/Adafruit_CircuitPython_TSC2046.git"

# Use 2kHz as a reasonable default frequency.
SPI_DEFAULT_FREQ_HZ = const(2_000_000)

INTERNAL_VREF = const(2.5)

VBAT_MULTIPLIER = const(4)

class Addr:

    class Ser:
        """ Addresses in single-ended reference mode. """
        TEMP0 = 0b000
        Y_POS = 0b001
        VBAT = 0b010
        Z1_POS = 0b011
        Z2_POS = 0b100
        X_POS = 0b101
        AUX = 0b110
        TEMP1 = 0b111

    class Dfr:
        """ Addresses in differential reference mode. """
        Y_POS = 0b001
        Z1_POS = 0b011
        Z2_POS = 0b100
        X_POS = 0b101


class CommandBits:

    def __init__(self):

        self.addr = 0
        self.use_8_bit_conv = False
        self.single_ended_ref = False
        self.enable_internal_vref = False
        self.enable_or_idle_adc = False

    def to_bytearray(self) -> bytearray:

        self.addr = self.addr & 0b111

        byte = (
            1 << 7 | # START bit, always 1.
            self.addr << 4 |
            int(self.use_8_bit_conv) << 3 |
            int(self.single_ended_ref) << 2 |
            int(self.enable_internal_vref) << 1 |
            int(self.enable_or_idle_adc)
        )

        return bytearray([byte])


class TSPoint:

    """
    The type of :py:attr:`TSC2046.touched_point`, which represents a
    touchscreen point. See the individual fields for more information.
    """

    def __init__(self, x: int, y: int, z: float):

        self.x: int = x
        """
        The full scale raw X coordinate from the touchscreen.

        If the touchscreen is not being touched, this value is meaningless.
        """

        self.y: int = y
        """
        The full scale raw Y coordinate from the touchscreen.

        If the touchscreen is not being touched, this value is meaningless.
        """

        self.z: float = z
        """
        The resistance measurement that corresponds to the pressure currently
        exerted on the touchscreen. The *higher* the pressure, the *lower*
        this resistance value will be. Unlike the X and Y coordinates, this
        value is not in an arbitrary unit of a full scale, but is a physical
        measurement, in ohms (Ω).
        """

    def __repr__(self):
        return f"X={self.x}, Y={self.y}, Z={self.z}"


class TSC2046:
    """
    General class for interacting with the TI TSC2046 touchscreen.
    You probably want :py:attr:`touched_point`.
    """

    def __init__(self, spi: busio.SPI, cs: digitalio.DigitalInOut, x_resistance=400, baudrate=SPI_DEFAULT_FREQ_HZ):

        self.vref: Optional[float] = None
        """
        The voltage (in volts) connected to the TSC2046's VRef pin, if any.

        `None` indicates that nothing is connected to the TSC2046's VRef pin,
        which is also the default.

        Connecting VRef to a voltage higher than 2.5V increases the accuracy of
        **non**-touchscreen reads (temperature, battery voltage, and auxilary
        voltage), and also directly determines the maximum voltage that can be
        measured by :py:attr:`auxiliary_voltage`. It has no effect on
        touchscreen coordinate reads.

        The TSC2046's VRef pin should either be connected to the same supply as
        the TSC2046's Vin pin, or not connected at all (Vin should be connected
        to a 5V or 3.3V supply from your CircuitPython board). If you do not
        connect VRef, leave this as `None`.
        """

        # NOTE(Qyriad): In my testing, the absolute most delicate touch where
        # the X and Y coordinate values were not completely nonsensical had
        # R_TOUCH at about 5.7kΩ (and even then the X and Y coordinates were
        # still fairly off), and every complete false positive was above
        # 100kΩ. To be on the safe side we'll use 100kΩ as the default threshold.
        self.touched_threshold: float = 100_000
        """
        The threshold for :py:attr:`is_touched`.

        This should be the resistance value (technically in ohms) to use as
        the threshold for :py:meth:`TSC2046.is_touched`. Any pressure
        readings that are higher than this value are considered "not
        touching" (remember that the pressure readings get LOWER as the
        physical pressure increases, see :py:attr:`TSPoint.z`). Also note
        that regardless of this threshold value, resistances of 0 and
        nonfinite numbers (like infinity) are always considered not
        touching.

        Defaults to :const:`100000` for 100,000kΩ.
        """

        self.interrupts_enabled: bool = True
        """
        Enables or disables interrupts that fire when the touchscreen is
        touched. When an interrupt fires, the ``IRQ`` pin on the TSC2046 is
        pulled LOW. That pin can be connected to an interrupt-enabled pin to
        run code when the TSC2046 detects a touch.

        .. todo:: Document CircuitPython interrupt stuff.

        Defaults to `True`.
        """


        # Regarding SPI mode, timing diagrams on the datasheet show DCLK idling
        # LOW, which means the leading edge is a rising edge, which means
        # CPOL (polarity) = 0.
        # For DOUT (MISO/CIPO), the datasheet says "data are shifted on the
        # falling edge of DCLK", and for DIN it says "data are latched on the
        # rising edge of DCLK", which means the OUT side changes on the
        # trailing edge of the clock, and the IN side changes on/after the
        # leading edge of the clock, which means CPHA (phase) = 1.
        self.spi_device = SPIDevice(spi, cs, baudrate=baudrate, polarity=0, phase=0)
        self._x_resistance = x_resistance

    def _effective_vref(self):
        if self.vref is not None:
            return self.vref

        return INTERNAL_VREF


    def _read_coord(self, channel_select):

        cmd = CommandBits()

        cmd.addr = channel_select

        # Use 12-bit conversions.
        cmd.use_8_bit_conv = False

        # Differential reference mode is only available for touchscreen reads,
        # but is more accurate. Since touchscreen coordinates are what we're
        # trying to read here, let's make use of that increased accuracy by
        # leaving this LOW to disable single-ended reference mode (and thus
        # enable differential reference mode).
        cmd.single_ended_ref = False

        # NOTE(Qyriad): The datasheet says that PD0 = 1 disables interrupts,
        # however in my testing PENIRQ' goes low when the touchscreen is
        # touched even if PO0 = 1, and the only case where PENIRQ' does not
        # respond to touches is when *both* PD0 and PD1 are both.
        if self.interrupts_enabled:
            # We're using differential reference mode, so VREF (internal or
            # external) doesn't matter at all, so let's just leave it off.
            cmd.enable_internal_vref = False

            # Interrupts are *only* disabled when *both* PD1 and PD0 are HIGH.
            # In this block, interrupts are enabled, so we can leave this LOW
            # and have the ADC power down between conversions to save power.
            cmd.enable_or_idle_adc = False
        else:
            # We're using differential reference mode, so VREF (internal or
            # external) doesn't matter at all. However, this value *does*
            # affect whether or not interrupts are enabled. If we explicitly
            # want to disable interrupts, we have to set this bit HIGH, which
            # consumes more power.
            cmd.enable_internal_vref = True

            # We want the ADC on, and also this value has to be HIGH to disable
            # interrupts.
            cmd.enable_or_idle_adc = True

        spi_cmd = cmd.to_bytearray()

        # We're reading a 12-bit value with the most significant BIT first.
        # Therefore, the first 1-byte read will read the 8 most-significant
        # bits, and the next 1-byte read will read the 4 least-significant
        # bits.

        out_data = bytearray(2)
        with self.spi_device as spi:
            spi.write(spi_cmd)
            spi.readinto(out_data)

        out_int = int.from_bytes(out_data, "big")

        # We can read the two bytes, concat'd to a 16-bit value, then drop
        # the bottom 3 bits and mask away the top bit.
        return (out_int >> 3) & 0xFFF

    def _read_extra(self, channel_select):

        cmd = CommandBits()

        cmd.addr = channel_select

        # Use 12-bit conversions.
        cmd.use_8_bit_conv = False

        # Differential reference mode is more accurate, but is only available
        # for touchscreen coordinate reads. Since we're reading the "extras"
        # (temperature, VBAT, etc), we keep this HIGH to use single-ended
        # reference mode.
        cmd.single_ended_ref = True

        # If the user connected an external VREF, turn the internal one off.
        cmd.enable_internal_vref = (self.vref is not None)

        # Leaving the ADC off has no downsides, except that the PENIRQ' output
        # used to trigger interrupts is also disabled if this bit is HIGH.
        # Since that's the only functionality of this bit that we care about,
        # we'll make it directly correspond to the user's IRQ settings.
        cmd.enable_or_idle_adc = not self.interrupts_enabled

        spi_cmd = cmd.to_bytearray()

        # We're reading a 12-bit value with the most significant BIT first.
        # Therefore, the first 1-byte read will read the 8 most-significant
        # bits, and the next 1-byte read will read the 4 least-significant
        # bits.

        out_data = bytearray(2)
        with self.spi_device as spi:
            spi.write(spi_cmd)
            spi.readinto(out_data)

        out_int = int.from_bytes(out_data, "big")

        # We can read the two bytes, conat'd to a 16-bit value, then drop
        # the bottom 3 bits and mask away the top bit.
        return (out_int >> 3) & 0xFFF

    def _read_temperature_k(self):
        # There are two ways to measure temperature on this chip.
        # The first is to Already Know what the ADC value of TEMP0 is at 25°C
        # for this particular chip, and then take a reading.
        # We don't want to make the user do more measurements than they have
        # to, so thankfully the second method eschews that limitation, in
        # favor of needing to measure two different values.
        # It then gives us a formula to calculate the temperature in Kelvin
        # given the difference of two voltages.
        #
        # The formula is:
        # T = (ELEMENTARY_CHARGE * ΔV) / (BOLTZMANN_CONST * ln(91))
        # Now, the elementary charge constant and Boltzmann's constant are both
        # *incredibly* tiny (Boltsmann's constant is 1.3807e-23) -- we probably
        # don't want to do that kind of math on this microcontroller.
        #
        # Thankfully, that formula simplifies to:
        # T = 2572.52 K/V (kelvins per volt), or
        # T = 2.57257 K/mV (kelvins per mimivolt).

        temp0 = self._read_extra(Addr.Ser.TEMP0)
        temp1 = self._read_extra(Addr.Ser.TEMP1)

        # temp0 and temp1 are given as a ratio of the reference voltage
        # and the full ADC scale.
        # In other words, the V_temp0 = (temp0 * V_REF) / (2 ** ADC_SIZE)
        # Which in our case means V_temp0 = (temp0 * effective_vref) / 4096.
        # We want the change in voltage accross those two readings, and in
        # millivolts, so:
        vref = self._effective_vref()
        delta_millivolts = (((temp1 - temp0) * vref) / 4096) * 1000

        # Now apply that simplified formula:
        temperature_kelvin = delta_millivolts * 2.573

        return temperature_kelvin

    def _is_point_touched(self, point: TSPoint):
        # If the resistance is not infinity, NaN, some other non-finite number,
        # or 0, then the touchscreen is probably not being touched.
        return math.isfinite(point.z) and point.z != 0 and point.z < self.touched_threshold

    @property
    def temperature(self) -> float:
        return self._read_temperature_k() - 273

    @property
    def temperature_c(self) -> float:
        """
        Equivalent to :py:meth:`TSC2046.temperature`; can be used for clarity.
        """
        return self.temperature

    @property
    def temperature_f(self) -> float:
        celsius = self.temperature_c
        return (9 / 5) * celsius + 32

    @property
    def battery_voltage(self):
        # According to the datasheet, the battery voltage readings are divided
        # down by 4 to simplifiy the logic in the chip, which means we have to
        # multiply it back up again.
        raw_vbat = self._read_extra(Addr.Ser.VBAT)

        # V_BAT = ADC_VALUE * 4 * effective_vref / (2 ** ADC_SIZE)
        vref = self._effective_vref()
        return (raw_vbat * VBAT_MULTIPLIER * vref) / 4096

    @property
    def auxiliary_voltage(self):

        raw_vaux = self._read_extra(Addr.Ser.AUX)

        # V_AUX = (ADC_VALUE * effective_vref) / (2 ** ADC_SIZE)
        return (raw_vaux * self._effective_vref()) / 4096


    @property
    def is_touched(self) -> bool:
        """
        Determines if the touchscreen is currently being touched.

        You can also change the threshold used to determine if the touchscreen
        is being touched by setting :py:attr:`TSC2046.touched_threshold`.

        .. seealso:: :py:meth:`touched_point`

        :returns: `True` if the touchscreen is being touched, `False` if it is
            not.
        :rtype: bool
        """

        return self._is_point_touched(self._get_point())

    @property
    def touched_point(self) -> Optional[TSPoint]:
        """
        The X, Y, and Z coordinates of the current touch on the touchscreen, or
        `None` if the touchscreen is not being touched.
        """

        point = self._get_point()
        if self._is_point_touched(point):
            return point
        else:
            return None

    def _get_point(self) -> TSPoint:
        """
        Gets the coordinates of the current touch on the touchscreen.
        Use :py:meth:`TSC2046.is_touched` to determine if the touchscreen is
        being touched in the first place.

        .. seealso:: :py:attr:`touched_point`

        :returns: The X, Y, and Z (pressure) coordinates.
        :rtype: TSPoint
        """

        x = self._read_coord(Addr.Dfr.X_POS)
        y = self._read_coord(Addr.Dfr.Y_POS)
        z1 = self._read_coord(Addr.Dfr.Z1_POS)
        z2 = self._read_coord(Addr.Dfr.Z2_POS)

        # If we would divide by 0, consider R_TOUCH to be zero.
        if z1 == 0:
            r_touch = 0
        else:
            # The datasheet gives two ways to calculate pressure. We're going
            # to use the one that requires the least information from the user:
            #
            # R_TOUCH = R_X_PLATE * (X_POSITION / 4096) * (Z_2 / Z_1 -1)
            # So this requires knowing the X-Plate resistance, which thankfully
            # we got from the user in self.__init__().
            r_touch = self._x_resistance * (x / 4096) * (z2 / z1 - 1)

        return TSPoint(x, y, r_touch)
