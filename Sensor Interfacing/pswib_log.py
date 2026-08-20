# pswib_log.py logs time-indexed power, speed, wind, imu, 
# and barometric data to CSV
# This file is an accumulation of work from the following files:
#
# To prevent obscene latency, ensure I2c clock is maxed out at 400khz
#
# reedspeed_logger.py
# bno_logger.py
# double_bme_diff_wind.py
# ble_pwr_logger.py
#
#
# datafields: 
#
# no sensor: timestamp
# speed sensor: speed, time elapsed since last speed update
# 		-> will care more about extrapolated speed, this can be logged
# handlebar-mounted BME680: pressure
# framebag-mounted BME680: pressure
#		-> these two will also measure temp+hum and give us:
#          differential pressure, computed windspeed
# bottom bracket-mounted BNO055: XYZ acceleration, XYZ gyro (GYRO NOW BROKEN)
# top tube-mounted foam-cushioned BNO055: XYZ acceleration, XYZ gyro
# powermeter: instantaneous power
#
# Note: some of these readings need be less frequent than others.
#
# Logging humidity or extrapolated speed at same fs 
# as acceleration data is not useful.


### ADD: rho, last update bme1, last update bme2, last update bno, power, speed
### also update speed for multiple magnets

import csv
import time
import os
import board
import busio
import sys
import subprocess
import numpy as np
import datetime
from smbus2 import SMBus #to i2cset the pca i2c multiplexer & open channels
#from adafruit_extended_bus import ExtendedI2C
#i2c = ExtendedI2C(3)

#IMU:
import adafruit_bno055
import json
# reedspeed:
from gpiozero import Button
# Baro:
import adafruit_bme680
# Powermeter:
import asyncio
import struct
from bleak import BleakClient
import threading

# bools for whether to activate/use bno
run_bno= False

# i2c addresses:
mux = 0x70

# bno i2c: 0x28
bno_port = 0x00
# bag i2c: 0x76
bag_port = 0x01
# bme i2c: 0x77
bme_port = 0x10
# fbno i2c: 0x29
fbno_port = 0x00
bus = SMBus(1)


dt = .01


# speed sensor GPIO pin:
gpio_s = 26
circum = 2.096	# inflated tire circumference in m
min_speed = .4469 # below this in m/s, we want speed readout=0

address = "EF:79:3C:65:6E:8E" # address for my own powermeter
# if you want a quick scare to see how many bluetooth devices are 
# running around you, you can run :
# sudo hcitool lescan
# and watch all the devices pop up
uuid = "00002a63-0000-1000-8000-00805F9B34FB"


i2c = busio.I2C(board.SCL, board.SDA) 
i2c_lock = threading.Lock()

timestamp = time.strftime("%Y%m%d-%H%M%S")
folder = "/home/bcelli/Projects/RAVENS/Sensor Interfacing/full captures"
filename = f"full_log_{timestamp}.csv"
filepath = os.path.join(folder,filename)
file_exists = os.path.isfile(filepath)

# BNO055 IMU code:

bno = None
with open("/home/bcelli/Projects/RAVENS/Sensor Interfacing/bno calibration/bno055_offsets.json", "r") as f:
		configs = json.load(f) # dict of all the bno offsets
# coordinate rotation matrix, computed in RAVENS/Sensor Processing/coordinate_conv.py

R = np.array([[ 0.99931267,  0.00313223,  0.03693735],
 [ 0.,          0.99642389, -0.0844951],
 [-0.03706991,  0.08443702,  0.99573903]]
)

last_a = np.array([0,0,9.71]) # coords for my stationary bno
last_g = np.array([0,0,0])
imulock = threading.Lock()
last_imu_ping = time.monotonic()
a = np.array([0, 0, 0]) # placeholder, to be accessed in "since ping" logic
g = np.array([0, 0, 0])
def imu_ping():
	global bno, last_a, last_g, last_imu_ping, a, g
	while True: 
		try: # prevent i2c issue from derailing whole file
			while bno is None:
				with i2c_lock: 
					bus.write_byte(mux, bno_port)	
					try:	
						bno = adafruit_bno055.BNO055_I2C(i2c, address=0x28) 
						bno.mode = adafruit_bno055.CONFIG_MODE #allow to be configured now
					except (OSError, RuntimeError, ValueError) as e:
						print(f"BNO not found: {e}")
						bno = None
				if bno is None:
					time.sleep(1) # error connecting, don't try again immediately
					continue
				time.sleep(.02) # allow time to switch mode
				with i2c_lock:
					bus.write_byte(mux, bno_port)
					bno.offsets_accelerometer = tuple(configs["offset_a"])
					bno.offsets_gyroscope = tuple(configs["offset_g"])
					bno.offsets_magnetometer = tuple(configs["offset_m"])
					bno.radius_accelerometer = configs["radius_a"]
					bno.radius_magnetometer = configs["radius_m"]
					bno.mode = adafruit_bno055.IMUPLUS_MODE # collect accel, gyro data
				print("Low BNO connected.")
				time.sleep(.01) # switch mode again
			if i2c_lock.acquire(blocking=False): # prevent i2c from getting stopped up
				bus.write_byte(mux, bno_port)
				try:
					this_acc = bno.acceleration
					# miniscule chance of missing update since bno logs many digits
					# of precision:
					if this_acc[0] != a[0] and this_acc[0] is not None: # only update "since"
						last_imu_ping = time.monotonic() # w/ new valid data
					g = bno.gyro
					a = this_acc
				finally:
					i2c_lock.release()
			if  a[0] is not None and g[0] is not None:
			# this occasional glitchy misread as None 
			# puts a stop to the entire program
				a_mapped = R@a
				g_mapped = R@g
				with imulock:
					last_a = a_mapped
					last_g = g_mapped
			time.sleep(.001)
		except (OSError, RuntimeError) as e:
			t = time.strftime('%H:%M:%S')
			print(f"{t} BNO055 error: {e}.")
			bno = None # signal to reconnect

if run_bno:	
	imu_thread = threading.Thread(target=imu_ping, daemon=True)
	imu_thread.start()

# end low IMU code

# begin foam-cushioned IMU code
# I will refer to this as foambno and imuf
fbno = None
with open("/home/bcelli/Projects/RAVENS/Sensor Interfacing/bno calibration/foambno_offsets.json", "r") as ff:
		fconfigs = json.load(ff) # dict of all the bno offsets
		
# coordinate rotation matrix, computed in RAVENS/Sensor Processing/coordinate_conv.py

Rf = np.array([[ 0.99197295, -0.04167364,  0.11938579],
 [ 0.,         -0.9441327,  -0.32956553],
 [ 0.12645023,  0.32692009, -0.93655411]]) 

last_af = np.array([0,0,9.73]) ## mapped coords for my stationary bno
last_gf = np.array([0,0,0])
imuflock = threading.Lock()
last_imuf_ping = time.monotonic()
af = np.array([0, 0, 0]) # placeholder, to be accessed in "since ping" logic
gf = np.array([0, 0, 0])
def imuf_ping():
	global fbno, last_af, last_gf, last_imuf_ping, af, gf
	while True: 
		try: # prevent i2c issue from derailing whole file
			while fbno is None:
				with i2c_lock:
					bus.write_byte(mux, fbno_port) 
					try:	
						fbno = adafruit_bno055.BNO055_I2C(i2c, address=0x29) #ADDR pin pulled-up 
						fbno.mode = adafruit_bno055.CONFIG_MODE #allow to be configured now
					except (OSError, RuntimeError, ValueError) as e:
						print(f"Foam BNO not found: {e}")
						fbno = None
				if fbno is None:
					time.sleep(1) #sensor down, wait & try again
					continue
				time.sleep(.02) # time to switch modes
				with i2c_lock: 
					bus.write_byte(mux, fbno_port)
					fbno.offsets_accelerometer = tuple(fconfigs["offset_a"])
					fbno.offsets_gyroscope = tuple(fconfigs["offset_g"])
					fbno.offsets_magnetometer = tuple(fconfigs["offset_m"])
					fbno.radius_accelerometer = fconfigs["radius_a"]
					fbno.radius_magnetometer = fconfigs["radius_m"]
					fbno.mode = adafruit_bno055.IMUPLUS_MODE # collect accel, gyro data
				print("Foam BNO connected.")
				time.sleep(.01)
			if i2c_lock.acquire(blocking=False):
				bus.write_byte(mux, fbno_port)
				try:
					this_accf = fbno.acceleration
					# miniscule chance of missing update since bno logs many digits
					# of precision:
					if this_accf[0] != af[0] and this_accf[0] is not None: # only update "since"
						last_imuf_ping = time.monotonic() # w/ new valid data
					gf = fbno.gyro
					af = this_accf
				finally:
					i2c_lock.release()
		
				##
				## troubleshoot:
			#if af[0] is None or gf[0] is None:
			#	print("fbno is None")
				##
				##
				
			if  af[0] is not None and gf[0] is not None:
			# this occasional glitchy misread as None 
			# puts a stop to the entire program
				af_mapped = Rf@af
				gf_mapped = Rf@gf
				with imuflock:
					last_af = af_mapped
					last_gf = gf_mapped
			time.sleep(.001)
		except (OSError, RuntimeError) as e:
			t = time.strftime('%H:%M:%S')
			print(f"{t} Foam BNO error: {e}.")
			fbno = None # signal to reconnect
			
if run_bno:
	imuf_thread = threading.Thread(target=imuf_ping, daemon=True)
	imuf_thread.start()
# end second imu code

# begin BME680 code

bme = None
last_pbme = 0 # used to determine whether ready for 
pbme_raw = 0 # initialize as 0 so it remains = to last_pbme
last_tbme = 0 # for robustness if bme can't connect
last_hbme = 0
bmelock = threading.Lock()
bme_last_ping = time.monotonic()
def bme_ping():
	global bme, last_pbme, pbme_raw, last_tbme, last_hbme, bme_last_ping
	counter = 0
	while True:
		try:
			while bme is None:
				with i2c_lock:
					bus.write_byte(mux, bme_port)
					try:
						bme = adafruit_bme680.Adafruit_BME680_I2C(i2c, address=0x77)
						bme.refresh_rate = 10
						bme.filter_size = 0 # adjust the internal low-passing
						# to balance consistency and hysteresis in pressure data
						bme.pressure_oversample=16
						bme._run_gas = False # we don't need gas analytics

						last_pbme = bme.pressure
						last_tbme = bme.temperature
						last_hbme = bme.humidity
						print("Connected to pitot BME.")
					except (OSError, RuntimeError, ValueError) as e:
						print(f"Could not connect to pitot bme: {e}")
				if bme is None:
					time.sleep(1)
					continue
			if i2c_lock.acquire(blocking=False):
				bus.write_byte(mux, bme_port)
				try:
					pbme_raw = float(bme.pressure) # pitot pressure
				finally:
					i2c_lock.release()
			with bmelock:
				if pbme_raw != last_pbme:
					bme_last_ping = time.monotonic()
				counter += 1
				last_pbme = pbme_raw
				if counter % 10 == 0: # decrease latency by sampling less
					if i2c_lock.acquire(blocking=False):
						bus.write_byte(mux, bme_port)
						try:
							last_tbme = float(bme.temperature)
							last_hbme = float(bme.humidity)
						finally:
							i2c_lock.release()
			time.sleep(.001)
		except (OSError, RuntimeError) as e:
			t = time.strftime('%H:%M:%S')
			print(f"{t} BME680 pitot error: {e}.")
			
bme_thread = threading.Thread(target=bme_ping, daemon=True)
bme_thread.start()
			
	
#second pressure sensor in bag for wind calcs 
bag_bme = None

last_bag = 0
pbag_raw = 0
baglock = threading.Lock()
bag_last_ping = time.monotonic()
def bag_ping():
	global bag_bme, pbag_raw, last_bag, bag_last_ping
	while True:
		try:
			while bag_bme is None:
				with i2c_lock:
					bus.write_byte(mux, bag_port)
					try:
						bag_bme = adafruit_bme680.Adafruit_BME680_I2C(i2c, address=0x76)
						bag_bme.refresh_rate = 10
						bag_bme.filter_size = 0
						bag_bme.pressure_oversample=16
						bag_bme._run_gas = False
						print("Connected to bag BME.")
					except (OSError, RuntimeError, ValueError) as e:
						print(f"Could not connect to bag bme: {e}")
				if bag_bme is None:
					time.sleep(1)
					continue
			if i2c_lock.acquire(blocking=False):
				bus.write_byte(mux, bag_port)
				try:
					pbag_raw = float(bag_bme.pressure) # environmental pressure
				finally:
					i2c_lock.release()
			with baglock:
						if last_bag != pbag_raw: # catches each new ping
							bag_last_ping = time.monotonic()
						last_bag = pbag_raw
			time.sleep(.001)
		except (OSError, RuntimeError, ValueError) as e:
			t = time.strftime('%H:%M:%S')
			print(f"{t} BME680 (bag) error: {e}.")
bag_thread = threading.Thread(target=bag_ping, daemon=True)
bag_thread.start()
	
def bme_calibrate():
	global delta_bme
	time.sleep(3) # allow other processes to boot before calibration
	print("BME offset calibration underway...")
	# number of samples to calibrate the offsets between sensors:
	calibrate_size = 50
	pbme_logs = []
	pbag_logs = []
	for i in range(calibrate_size):
		print(" ", end="", flush=True)
	print("|")
	while len(pbme_logs) < calibrate_size:
		with bmelock:
			pbme_val = last_pbme
		with baglock:
			pbag_val = last_bag
		if pbme_val > 0 and pbag_val > 0:
			pbme_logs.append(pbme_val)
			pbag_logs.append(pbag_val)
			print(".", end="", flush=True)
		time.sleep(.01)
	pbme_avg = np.mean(pbme_logs)
	pbag_avg = np.mean(pbag_logs)
	delta_bme = pbme_avg-pbag_avg
	print("\nCalibration complete.")
	
bme_calibrate()

#
# speed sensor code:
#
# allow RPi to read gpio pin:
os.environ['GPIOZERO_PIN_FACTOR'] = 'lgpio'
class reedswitch:
	def __init__(self, gpioPin, circumference, minspeed):
		self.switch = Button(gpioPin, pull_up=True, bounce_time=.01)
		# can't be triggered twice within .01s
		self.circumference = circumference
		self.dist_per_dt = circumference / 4 ## 4 MAGNETS PER WHEEL HERE
		self.recent = time.monotonic()
		self._speed = 0.0 # float speed
		self.switch.when_pressed = self.closed_alert
		self.threshold = minspeed #.4469m/s = 1mph, below this readout we want 0
		
	@property
	def speed(self):
		now = time.monotonic()
		d_t = now - self.recent
		d_t_threshold = self.circumference/self.threshold
		if d_t > d_t_threshold:
			self._speed = 0.0 # if sensor hasn't triggered in a while
		return self._speed
	@property
	def elapsed_since_ping(self):
            return time.monotonic()-self.recent
	def closed_alert(self):
		now = time.monotonic()
		d_t = now - self.recent
		d_t_threshold = self.circumference/self.threshold
		self._speed = self.dist_per_dt/d_t
		self.recent = now #back to next cycle
instance = reedswitch(gpio_s, circum, min_speed)
#
# end of speed code
#


# powermeter code:
#
power = 0 # before call is made from main csv logging loop
powerlock = threading.Lock()
last_pwr_ping = time.monotonic()
pwm = None # to be modified in BLE_manage
print("Connecting to Power Meter. Spin pedals to wake it up.")
def BLE_ping(pwm, data):
	global power, last_pwr_ping
	new_power = struct.unpack('<h', data[2:4])[0] # '<' to read backwards, 
	last_pwr_ping = time.monotonic()				# H = hex b/c 2 bytes or 16 bit
	with powerlock:
		power = new_power
		
async def BLE_manage(): # should run regardless of BLE ping timing
	global pwm
	while True:	
		if pwm is None or not pwm.is_connected:
			pwm = BleakClient(address)
		try:
			async with pwm:
				await pwm.start_notify(uuid, BLE_ping)
				print("Power meter connected!")
				while pwm.is_connected:
					await asyncio.sleep(1) #run indefinitely
				# when while fails, it goes to top and retries
		except Exception as e:
			print(type(e).__name__, e)
			print("Spin pedals to wake up meter.")
			await asyncio.sleep(5)
			continue

def BLE_wrapper():
	loop = asyncio.new_event_loop()
	asyncio.set_event_loop(loop)
	loop.run_until_complete(BLE_manage())
	
# begin async thread and kill with rest of program:
ble_thread = threading.Thread(target=BLE_wrapper, daemon=True)
ble_thread.start()

#while pwm is None or not pwm.is_connected: # wait for connection
#	time.sleep(.1)


#
#
# end power code


print(f"Logging power, speed, baro, wind, IMU data to {filename}. Ctrl+C to stop.")


def main():
	counter = 0
	with open(filepath, mode='a', newline = '') as csvfile:

		fieldnames =  ['timestamp', 'pressure_bag', 'pressure_bme', 
					'diff_pressure', 'v_wind_est', 'speed', 
					'accel_x', 'accel_y', 'accel_z',
					'gyro_x', 'gyro_y', 'gyro_z', 
					'faccel_x', 'faccel_y', 'faccel_z',
					'fgyro_x', 'fgyro_y', 'fgyro_z',
					'power', 'rho', 'since_imu',
					'since_imuf', 'since_bag', 'since_bme',
				   'since_speed', 'since_pwr']
		writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
		
		# headers at top of csv:
		if not file_exists:
			writer.writeheader()
			
		next_sample = time.monotonic()
		start = time.time()
		start_mono = time.monotonic()
		try:
			while True:
				now = time.monotonic()
				sample_timestamp = next_sample # this is what we index data to
				if now < next_sample: # began early
					time.sleep(next_sample-now)
				next_sample += dt
				#
				# wind code:
				#
				with bmelock:
					pbme = last_pbme
					temp = last_tbme
					hum = last_hbme
				
				with baglock:
					pbag = last_bag

				since_bag = time.monotonic() - bag_last_ping 
				since_bme = time.monotonic() - bme_last_ping
				pbag_normd = pbag + delta_bme # pbme and pbag are offset by some amount found above
				dp = 100 * (pbme-pbag_normd) # hPa to Pa
				
				hum = hum/100 # convert from pct to decimal
				
				pascal = pbag*100 # environment pressure in pascal
				
				#hum = humidity pct from BME
				#temp = temp in celsius from BME
				#pascal = pressure in pascals from bag_BME
				
				pvapor = hum*610.78*np.exp((17.27*temp)/(temp+237.3))
				ctk = temp+273.15
				rho = (pascal-pvapor)/(287.058*ctk) + (pvapor)/(461.5*ctk)

				# By Bernoulli's eqn, v = sqrt((2*dp)/rho), units m/s
				
				vwind = np.sqrt((2*abs(dp))/abs(rho)) # rho should always be >0
				# but in first entry due to uninitialized vals can be <0
				if dp < 0:
					vwind = -vwind #tailwind condition
				
				#
				# end wind code
				#
				if run_bno:
					since_imu = time.monotonic() - last_imu_ping
					with imulock:
						ax = last_a[0]
						ay = last_a[1]
						az = last_a[2]
						gx = last_g[0]
						gy = last_g[1]
						gz = last_g[2]
					
					since_imuf = time.monotonic() - last_imuf_ping
					with imuflock:
						afx = last_af[0]
						afy = last_af[1]
						afz = last_af[2]
						gfx = last_gf[0]
						gfy = last_gf[1]
						gfz = last_gf[2]
				else:
					ax=ay=az=gx=gy=gz=afx=afy=afz=gfx=gfy=gfz=0
					since_imu = 0
					since_imuf = 0
				
				# reedspeed code:
				speed_now = instance.speed
				elapsed = instance.elapsed_since_ping
				# reedspeed done
				
				# power code:
				with powerlock:
					Pow = power
				since_pwr = time.monotonic() - last_pwr_ping
				# power done
				
				
				timestamp = start + (sample_timestamp-start_mono)
				t = datetime.datetime.fromtimestamp(timestamp).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
				writer.writerow({
					'timestamp': t,
					'pressure_bag': f"{pbag:.2f}",
					'pressure_bme': f"{pbme:.2f}",
					'diff_pressure': f"{dp:.2f}",
					'v_wind_est': f"{vwind:.2f}",
					'speed': f"{speed_now:.2f}",
					'accel_x': f"{ax:.2f}",
					'accel_y': f"{ay:.2f}",
					'accel_z': f"{az:.2f}",
					'gyro_x': f"{gx:.2f}",
					'gyro_y': f"{gy:.2f}",
					'gyro_z': f"{gz:.2f}",
					'faccel_x': f"{afx:.2f}",
					'faccel_y': f"{afy:.2f}",
					'faccel_z': f"{afz:.2f}",
					'fgyro_x': f"{gfx:.2f}",
					'fgyro_y': f"{gfy:.2f}",
					'fgyro_z': f"{gfz:.2f}",
					'power': Pow, # no rounding needed
					'rho': f"{rho:.4f}", # rho is <2 so long decimal
					'since_imu': f"{since_imu:.2f}",
					'since_imuf' : f"{since_imuf:.2f}",
					'since_bag': f"{since_bag:.2f}",
					'since_bme': f"{since_bme:.2f}",
					'since_speed': f"{elapsed:.2f}",
					'since_pwr': f"{since_pwr:.2f}"
				})
				
				if counter % 100 == 0:
					csvfile.flush()
				counter += 1
				

				
				
		except KeyboardInterrupt:
			return

if __name__ == "__main__":
	main()
