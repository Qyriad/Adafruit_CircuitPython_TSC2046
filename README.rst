Introduction
============


.. image:: https://readthedocs.org/projects/adafruit-circuitpython-tsc2046/badge/?version=latest
    :target: https://docs.circuitpython.org/projects/tsc2046/en/latest/
    :alt: Documentation Status


.. image:: https://raw.githubusercontent.com/adafruit/Adafruit_CircuitPython_Bundle/main/badges/adafruit_discord.svg
    :target: https://adafru.it/discord
    :alt: Discord


.. image:: https://github.com/Qyriad/Adafruit_CircuitPython_TSC2046/workflows/Build%20CI/badge.svg
    :target: https://github.com/Qyriad/Adafruit_CircuitPython_TSC2046/actions
    :alt: Build Status


.. image:: https://img.shields.io/badge/code%20style-black-000000.svg
    :target: https://github.com/psf/black
    :alt: Code Style: Black

CircuitPython library for the TI TSC2046 resistive touchscreen controller.


Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Adafruit's Bus Device library <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://circuitpython.org/libraries>`_
or individual libraries can be installed using
`circup <https://github.com/adafruit/circup>`_.

This library is for the TI TSC2046 resistive touchscreen controller and its
`breakout board <https://www.adafruit.com/product/5767>`_.

`Purchase one from the Adafruit shop <http://www.adafruit.com/products/5767>`_

Installing from PyPI
=====================
.. note:: This library is not available on PyPI yet. Install documentation is included
   as a standard element. Stay tuned for PyPI availability!

On supported GNU/Linux systems like the Raspberry Pi, you can install the driver locally `from
PyPI <https://pypi.org/project/adafruit-circuitpython-tsc2046/>`_.
To install for current user:

.. code-block:: shell

    pip3 install adafruit-circuitpython-tsc2046

To install system-wide (this may be required in some cases):

.. code-block:: shell

    sudo pip3 install adafruit-circuitpython-tsc2046

To install in a virtual environment in your current project:

.. code-block:: shell

    mkdir project-name && cd project-name
    python3 -m venv .venv
    source .env/bin/activate
    pip3 install adafruit-circuitpython-tsc2046

Installing to a Connected CircuitPython Device with Circup
==========================================================

Make sure that you have ``circup`` installed in your Python environment.
Install it with the following command if necessary:

.. code-block:: shell

    pip3 install circup

With ``circup`` installed and your CircuitPython device connected use the
following command to install:

.. code-block:: shell

    circup install adafruit_tsc2046

Or the following command to update an existing version:

.. code-block:: shell

    circup update

Usage Example
=============

.. code-block:: python

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
            print(f"Touched: ({point.x}, {point.y}), with {point.z} omhs of pressure")

        bat_voltage = touchscreen.battery_voltage
        aux_voltage = touchscreen.auxiliary_voltage
        temp_c = touchscreen.temperature_c

        print(f"Battery: {bat_voltage:.2f}V")
        print(f"Aux: {aux_voltage:.2f}V")
        print(f"Temperature: {temp_c:.2f}°C")

        time.sleep(0.5)



Documentation
=============
API documentation for this library can be found on `Read the Docs <https://docs.circuitpython.org/projects/tsc2046/en/latest/>`_.

For information on building library documentation, please check out
`this guide <https://learn.adafruit.com/creating-and-sharing-a-circuitpython-library/sharing-our-docs-on-readthedocs#sphinx-5-1>`_.

Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/Qyriad/Adafruit_CircuitPython_TSC2046/blob/HEAD/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.
