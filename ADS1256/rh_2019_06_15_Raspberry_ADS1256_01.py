#!/usr/bin/python
# -*- coding: utf-8 -*-
"""PiPyADC:
Hardware: Waveshare ADS1256 board interfaced to the Raspberry Pi 3
Ulrich Lukas 2017-03-10
wm           2019-06-16

wm: Thanks to Ulrich! (https://github.com/ul-gh/PiPyADC)
Use his library to do measurements with the 'Waveshare high precision AD/DA Board'.
(I did not succeed to use this board with other libraries).
In this slightly modified program only one channel is used for AD conversion.
Value is printed. That's all.
"""
import sys
import os
from ADS1256_definitions import *
from pipyadc import ADS1256

if not os.path.exists("/dev/spidev0.1"):
    raise IOError("Error: No SPI device. Check settings in /boot/config.txt")

### START EXAMPLE ###
################################################################################
###  STEP 0: CONFIGURE CHANNELS AND USE DEFAULT OPTIONS FROM CONFIG FILE: ###
#
# For channel code values (bitmask) definitions, see ADS1256_definitions.py.
# The values representing the negative and positive input pins connected to
# the ADS1256 hardware multiplexer must be bitwise OR-ed to form eight-bit
# values, which will later be sent to the ADS1256 MUX register. The register
# can be explicitly read and set via ADS1256.mux property, but here we define
# a list of differential channels to be input to the ADS1256.read_sequence()
# method which reads all of them one after another.
#
# ==> Each channel in this context represents a differential pair of physical
# input pins of the ADS1256 input multiplexer.
#
# ==> For single-ended measurements, simply select AINCOM as the negative input.
#
# AINCOM does not have to be connected to AGND (0V), but it is if the jumper
# on the Waveshare board is set.
#
# Input pin for the potentiometer on the Waveshare Precision ADC board:
POTI = POS_AIN0|NEG_AINCOM
# Light dependant resistor of the same board:
LDR  = POS_AIN1|NEG_AINCOM
# The other external input screw terminals of the Waveshare board:
EXT2, EXT3, EXT4 = POS_AIN2|NEG_AINCOM, POS_AIN3|NEG_AINCOM, POS_AIN4|NEG_AINCOM
EXT5, EXT6, EXT7 = POS_AIN5|NEG_AINCOM, POS_AIN6|NEG_AINCOM, POS_AIN7|NEG_AINCOM

# You can connect any pin as well to the positive as to the negative ADC input.
# The following reads the voltage of the potentiometer with negative polarity.
# The ADC reading should be identical to that of the POTI channel, but negative.
POTI_INVERTED = POS_AINCOM|NEG_AIN0

# For fun, connect both ADC inputs to the same physical input pin.
# The ADC should always read a value close to zero for this.
SHORT_CIRCUIT = POS_AIN0|NEG_AIN0

# Specify here an arbitrary length list (tuple) of arbitrary input channel pair
# eight-bit code values to scan sequentially from index 0 to last.
# Eight channels fit on the screen nicely for this example..
# CH_SEQUENCE = (POTI, LDR, EXT2, EXT3, EXT4, EXT7, POTI_INVERTED, SHORT_CIRCUIT)
# Use only AIN4/AIN5.
# >POS_AIN2|NEG_AINCOM< is not necessary, but program runs into error, if list
# has only one element.
CH_SEQUENCE = (NEG_AIN4|POS_AIN5, POS_AIN2|NEG_AINCOM)
# CH_SEQUENCE = (NEG_AIN4|POS_AIN5, None)
################################################################################

def do_measurement():
    ### STEP 1: Initialise ADC object using default configuration:
    # (Note1: See ADS1256_default_config.py, see ADS1256 datasheet)
    # (Note2: Input buffer on means limited voltage range 0V...3V for 5V supply)
    ads = ADS1256()

    ### STEP 2: Gain and offset self-calibration:
    ads.cal_self()

    while True:
        ### STEP 3: Get data:
        raw_channels = ads.read_sequence(CH_SEQUENCE)
        # voltages     = [i * ads.v_per_digit for i in raw_channels]

        ### STEP 4: DONE. Have fun!
        # sys.stdout.write( ", ".join(["{: 8d}".format(i) for i in raw_channels]) + "\n")
        sys.stdout.write( "{: d}".format(raw_channels[0]) + "\n")
        # nice_output(raw_channels, voltages)
### END EXAMPLE ###
#############################################################################

# Start data acquisition
try:
    # print("\033[2J\033[H") # Clear screen
    # print(__doc__)
    # print("\nPress CTRL-C to exit.")
    do_measurement()

except (KeyboardInterrupt):
    print("\n"*8 + "User exit.\n")
    sys.exit(0)
