# Identical to calibrate_bno055, but for address 0x29, not 0x28

import time
import json
import board
import adafruit_bno055

i2c = board.I2C()
bno = adafruit_bno055.BNO055_I2C(i2c, address=0x29)

print("hold sensor still at 6 stable positions")

try:
	while True:
		sys_cal, gyro_cal, accel_cal, mag_cal = bno.calibration_status
		print(f"\r[STATUS] Sys: {sys_cal}/3 | Gyro: {gyro_cal}/3 | Accel: {accel_cal}/3 | Mag: {mag_cal}/3", end="")
		if sys_cal == 3 and accel_cal == 3 and gyro_cal == 3 and mag_cal == 3:
			print("\n Accelerometer and gyro calibrated")
			print("Switching to config mode to read offsets")
			
			calibrations = {
				"offset_a": list(bno.offsets_accelerometer),
				"offset_g": list(bno.offsets_gyroscope),
				"offset_m": list(bno.offsets_magnetometer),
				"radius_a": bno.radius_accelerometer,
				"radius_m": bno.radius_magnetometer
			}
			
			
			with open("foambno_offsets.json", "w") as f:
				json.dump(calibrations, f, indent=4)

			print("Calibration saved successfully to 'foambno_offsets.json'")
			break
		time.sleep(0.2)

except KeyboardInterrupt:
	print("\nCalibration cancelled")
