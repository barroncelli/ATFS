import board
import busio
import adafruit_bno055
import time

i2c = busio.I2C(board.SCL, board.SDA)
bno = adafruit_bno055.BNO055_I2C(i2c)
bno.mode = adafruit_bno055.NDOF_MODE  # Full sensor fusion

time.sleep(1)

for i in range(10):
	x,y,z = bno.acceleration
	a,b,c = bno.gyro
	print(f"Accel: x = {x:.2f}, y = {y:.2f}, z = {z:.2f}, Gyro: x = {a:.2f}, y = {b:.2f}, z = {c:.2f}")
	time.sleep(0.1)
