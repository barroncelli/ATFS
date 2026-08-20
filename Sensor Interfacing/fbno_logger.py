# a replica of bno_logger, but with the calibration presets for the
# foam-padded BNO055, and logs to different file

import time
import board
import busio
import adafruit_bno055
import csv
import os
import sys
import subprocess
import datetime
import json


dt = .01  # sensing period

i2c = busio.I2C(board.SCL, board.SDA)

# call the test_sensor func to ensure i2c is behaving
result = (subprocess.run(["python3", "elementary testing/test_bno.py"],
		  capture_output = True, text=True))
		  
test_sensor_output = result.stdout

# one known fault of the following is if certain pins, like the Vin pin,
# fall out, the mpl temporarily has enough charge to give back the
# success signal 

if not ("Success! Sensor detected.") in test_sensor_output:
	print(test_sensor_output)
	sys.exit(1)

bno = adafruit_bno055.BNO055_I2C(i2c, address=0x29)
bno.mode = adafruit_bno055.CONFIG_MODE #allow to be configured now

# first, load json file with calibration info
with open("bno calibration/foambno_offsets.json", "r") as f:
		config = json.load(f) # dict of all the data
bno.offsets_accelerometer = tuple(config["offset_a"])
bno.offsets_gyroscope = tuple(config["offset_g"])
bno.offsets_magnetometer = tuple(config["offset_m"])
bno.radius_accelerometer = config["radius_a"]
bno.radius_magnetometer = config["radius_m"]
bno.mode = adafruit_bno055.NDOF_MODE # collect accel, mag, gyro data

timestamp = time.strftime("%Y%m%d-%H%M%S")

folder = "fbno captures"
filename = f"fbno_log_{timestamp}.csv"
filepath = os.path.join(folder,filename)
file_exists = os.path.isfile(filepath)
print(f"Logging foam-padded accelerometer/gyro/heading data to {filename}. Ctrl+C to stop.")



with open(filepath, mode='a', newline = '') as csvfile:
	fieldnames = ['timestamp', 'accel_x', 'accel_y', 'accel_z',
				'gyro_x', 'gyro_y', 'gyro_z', 
				'mag_x', 'mag_y', 'mag_z']
	writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
	
	# headers at top of csv:
	if not file_exists:
		writer.writeheader()
		
	try:
		while True:
			a = bno.acceleration # returns 3-vector
			ax = a[0]
			ay = a[1]
			az = a[2]
			
			g = bno.gyro
			gx = g[0]
			gy = g[1]
			gz = g[2]
			
			m = bno.magnetic
			mx = m[0]
			my = m[1]
			mz = m[2]
			
			cur = datetime.datetime.now()
			t = time.strftime("%Y-%m-%d %H:%M:%S") + f".{cur.microsecond//1000:03d}"
			
			
			# sometimes, due to i2c glitches, the program can't format None-type data
			if a[0] is not None and g[0] is not None and m[0] is not None:
				writer.writerow({
					'timestamp': t,
					'accel_x': f"{ax:.2f}",
					'accel_y': f"{ay:.2f}",
					'accel_z': f"{az:.2f}",
					'gyro_x': f"{gx:.2f}",
					'gyro_y': f"{gy:.2f}",
					'gyro_z': f"{gz:.2f}",
					'mag_x': f"{mx:.2f}",
					'mag_y': f"{my:.2f}",
					'mag_z': f"{mz:.2f}",
				})
			
				print(f"{t} | A_x: {ax:.2f}m/s/s | A_y: {ay:.2f}m/s/s "
				f"| A_z: {az:.2f}m/s/s | G_x: {gx:.2f}rad/s | G_y: {gy:.2f}rad/s "
				f"| G_z: {gz:.2f}rad/s | M_x: {mx:.2f}mT | M_y: {my:.2f}mT "
				f"| M_z: {mz:.2f}mT")
			
				csvfile.flush()
				time.sleep(dt)

	except KeyboardInterrupt:
			print("\nLogging stopped.")
	
	
