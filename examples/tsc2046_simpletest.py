# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2023 Qyriad for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense

"""
Simpletest example that shows how to get touchscreen coordinates, temperature
readings, battery voltages, and auxiliary voltages.
"""

import time
import board
import digitalio

import adafruit_tsc2046

# The TSC2046 communicates over SPI, so we need to grab an SPI bus to
# communicate over.
spi = board.SPI()

# As well as the SPI chip select pin to use for this touchscreen.
# This pin can be any digital pin; just make sure whichever pin you pick is
# connected to the "CS" pin on your TSC2046 breakout.
# This example uses pin D5.
cs_pin = digitalio.DigitalInOut(board.D5)

# Create a TSC2046 object, which we'll use to communicate with the touchscreen
# from now on.
touchscreen = adafruit_tsc2046.TSC2046(spi, cs_pin)

while True:

    point = touchscreen.touched_point

    # If the touchscreen isn't being touched at all, then `point` will be None.
    if point is not None:
        # We get X and Y as coordinates, but Z (pressure) is a physical
        # measurement in resistance. This resistance *decreases* as the physical
        # pressure *increases*.
        print(f"Touched: ({point.x}, {point.z}), with {point.z} omhs of pressure")

    bat_voltage = touchscreen.battery_voltage
    aux_voltage = touchscreen.auxiliary_voltage
    temp_c = touchscreen.temperature_c

    print(f"Battery: {bat_voltage:.2f}V")
    print(f"Aux: {aux_voltage:.2f}V")
    print(f"Temperature: {temp_c:.2f}°C")

    time.sleep(0.5)
