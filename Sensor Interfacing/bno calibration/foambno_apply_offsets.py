# Identical to bno_apply_offsets, but for 0x29 instead of 0x28

# This program applies the calibration offsets to the bno055.
# A word of warning: as soon as a program that applies offsets to the
# bno055 closes, the sensor INSTANTLY reverts to its factory resets.

import time
import json
import board
import adafruit_bno055

i2c = board.I2C()
bno = adafruit_bno055.BNO055_I2C(i2c, address=0x29)
bno.mode = adafruit_bno055.CONFIG_MODE #allow to be configured now

# first, load json file with calibration info
try:
	with open("foambno_offsets.json", "r") as f:
		config = json.load(f) # dict of all the data
except FileNotFoundError:
	print("Error: JSON file not found")
	exit()
	
# next, obtain offsets and apply to bno
try:
	bno.offsets_accelerometer = tuple(config["offset_a"])
	bno.offsets_gyroscope = tuple(config["offset_g"])
	bno.offsets_magnetometer = tuple(config["offset_m"])
	bno.radius_accelerometer = config["radius_a"]
	bno.radius_magnetometer = config["radius_m"]
	
	bno.mode = adafruit_bno055.IMUPLUS_MODE
	# explicitly put back into standard mode
	

except OSError:
	print(OSError)
	print("I2C error")
	exit()
