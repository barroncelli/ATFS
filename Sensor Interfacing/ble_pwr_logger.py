# Logs real-time power from Stages SPM1 powermeter over 
# Bluetooth Low-Energy

import asyncio
import struct
import time
import datetime
from bleak import BleakClient
import csv
import sys
import os

address = "EF:79:3C:65:6E:8E" # address for my own powermeter
# if you want a quick scare to see how many bluetooth devices are 
# running around you, you can run :
# sudo hcitool lescan
# and watch all the devices pop up


# Bluetooth base uuid:
# 0000XXXX-0000-1000-8000-00805F9B34FB
# powermeter uuid = 2a63
uuid = "00002a63-0000-1000-8000-00805F9B34FB"


timestamp = time.strftime("%Y%m%d-%H%M%S")
folder = "pwm captures"
filename = f"pwm_log_{timestamp}.csv"
filepath = os.path.join(folder,filename)
file_exists = os.path.isfile(filepath)
print(f"Logging power data to {filename}. Ctrl+C to stop.")


# take in data and  parse out 2-byte power, which is stored backwards 
def BLE_ping(pwm, data):
	power =  struct.unpack('<h', data[2:4])[0] # '<' to read backwards, 
										# H = hex b/c 2 bytes or 16 bit
	
	cur = datetime.datetime.now()
	t = time.strftime("%Y-%m-%d %H:%M:%S") + f".{cur.microsecond//1000:03d}"
	
	writer.writerow({
		'timestamp': t,
		'power': power
	})	
	
	
			
			
async def main(): # should run regardless of device ping timing
	
	with open(filepath, mode='a', newline = '') as csvfile:
		fieldnames = ['timestamp', 'power']
		global writer # we need to access in BLE_ping
		writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
		writer.writeheader()
		print("Connecting...")
		pwm = None
		
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
				
		
if __name__ == "__main__":
	try:
		asyncio.run(main()) 
	except KeyboardInterrupt:
		print("\nLogging stopped.")

	
	




