import time
import threading
import signal
import glob
import picamera
import serial
import datetime as dt
import copy
import csv
import io
from PIL import Image
import os

OVERALL_LATENCY = 0.050 #seconds Latency of the camera encoding
FOOTAGE_LENGTH = 60 #Length in seconds of each rush
TGPS_TRPI = []  #tuple with GPS time and RaspPi time at the same instant.
RECORDING_PATH = "/media/footage" #Path where to shot footage
SERIAL_PORT = '/dev/ttyS0'
time_lock = threading.Lock()

#   lock.acquire()
#   g += 1
#  lock.release()

class TGPS_TRPI():
	'''
	This class is use as a communication between the thread to sync time of GPU ans GPS
	'''
	def __init__(self):
		self.TGps = {} #time of the GPS
		self.TRpi_for_GPS = 0 #Second Time of the RPi taken at the same moment as TGps
		self.TRpi_for_GPU = 0	#Second Time of the RPi taken at the same moment as TGPU
		self.TGPU = 0	#MicroSecond Time of the GPU
		self.temporary_convert = {"year": 0, "month": 0, "day": 0, "hour": 0, "minute": 0, "second": 0, "usecond": 0}
		
	def time_sync(self, frame_timestamp):
		'''
		This function take a time stamp (GPU clock) and return the time in GPS frame)
		'''
		if frame_timestamp is None:
			print("Frame dropped")			
			return None
		else:
			frame_timestampRPI = self.TRpi_for_GPU + frame_timestamp / 1e6 - self.TGPU / 1e6
			delta_timestamp2GPS = frame_timestampRPI - self.TRpi_for_GPS
			#print("Delta : ", self.addTime(self.TGps, self.micros2time(delta_timestamp2GPS)))
			return self.addTime(self.TGps, self.micros2time(delta_timestamp2GPS))
	
	def micros2time(self, second):
		'''
		This fonction convert an amount of second into a structure as self.temporary_convert 
		I haven't done for year month and day because i'm lazy and we work under a second
		'''
		self.temporary_convert["usecond"] = int(second * 1e6) % 1e6
		self.temporary_convert["second"] = int(second) % 60
		self.temporary_convert["minute"] = int(second / 60) % 60
		self.temporary_convert["hour"] = int(second / 3600) % 24
			
		return self.temporary_convert

	def addTime(self, time1, time2):
		'''
		This fonction allow to add two time structure
		'''
		returned_time = {"year": 0, "month": 0, "day": 0, "hour": 0, "minute": 0, "second": 0, "usecond": 0}
		returned_time["year"] = time1["year"] + time2["year"]
		returned_time["month"] = time1["month"] + time2["month"]
		returned_time["day"] = time1["day"] + time2["day"]
		returned_time["hour"] = time1["hour"] + time2["hour"]
		returned_time["minute"] = time1["minute"] + time2["minute"]
		returned_time["second"] = time1["second"] + time2["second"]
		returned_time["usecond"] = time1["usecond"] + time2["usecond"]

		if returned_time["usecond"] >= 1e6:
			returned_time["second"] = returned_time["second"] + int(returned_time["usecond"] / 1e6)
			returned_time["usecond"] = returned_time["usecond"] % 1e6

		if returned_time["second"] >= 60:
			returned_time["minute"] = returned_time["minute"] + int(returned_time["second"] / 60)
			returned_time["second"] = returned_time["second"] % 60
		
		#if returned_time["minute"] > 60:
		#	returned_time["hour"] = returned_time["hour"] + int(returned_time["minute"] / 60)
		#	returned_time["minute"] = returned_time["minute"] % 60

		#if returned_time["hour"] > 24:
		#	returned_time["day"] = returned_time["day"] + int(returned_time["hour"] / 24)
		#	returned_time["hour"] = returned_time["hour"] % 24
		
		return returned_time

#Create the structure to share time
great_timeStruct = TGPS_TRPI()

#----------------------------------------------------------
#----------------------------------------------------------
#----------------------------------------------------------

class TheDoctor(threading.Thread):
 	'''
	This Thread is dedicated to listen on Serial port the Time coming from the ucontroller
	'''
	def __init__(self):
		threading.Thread.__init__(self)
 
		# The shutdown_flag is a threading.Event object that
		# indicates whether the thread should be terminated.
		self.shutdown_flag = threading.Event()
 
		self.time = {"year": 0, "month": 0, "day": 0, "hour": 0, "minute": 0, "second": 0, "usecond": 0}
		
		#Initiate the serial communication
		global SERIAL_PORT
		
		#Init SerialPort
		try:
			self.ser = serial.Serial(SERIAL_PORT, 115200, timeout=12) #'/dev/ttyS0' or '/dev/ttyAMA0'
			self.ser.open()
			print("The port "+str(self.ser.port)+" is available")

		except serial.serialutil.SerialException:
			print("The port "+str(self.ser.port)+" is already in use : rebooting...")
			self.ser.close()
			self.ser.open()

		print("Reading data income")
 
	def run(self):
		global great_timeStruct

		while not self.shutdown_flag.is_set():
			first_shot = time.time()
			response = self.ser.readline()
			print("response", response)
			self.parser(response)

			#Sync of GPS Time and RPi Time
			time_lock.acquire()
			great_timeStruct.TGps = copy.deepcopy(self.time)
			great_timeStruct.TRpi_for_GPS = (time.time() + first_shot) / 2
			time_lock.release()
 
		self.destruct()
		print('Thread serial stopped')
	
	def destruct(self):
		self.ser.close()

	def parser(self, timestring):
		'''
		This function parse the incoming time data from GPS
		'''
		timestring = timestring.replace('\n','')
		timestring = timestring.replace('\r','')
		timestring = timestring.split(":")
		self.time['year'] = int(timestring[0])
		self.time['month'] = int(timestring[1])
		self.time['day'] = int(timestring[2])
		self.time['hour'] = int(timestring[3])
		self.time['minute'] = int(timestring[4])
		self.time['second'] = int(timestring[5])
		self.time['usecond'] = int(timestring[6])
		print(self.time)

#----------------------------------------------------------
#----------------------------------------------------------
#----------------------------------------------------------

class RecorderStamper(threading.Thread):
 	'''
	This class open the camera and sync the time
	'''
	def __init__(self, width=1280, height=720, framerate=30):
		threading.Thread.__init__(self)
 
		# The shutdown_flag is a threading.Event object that
		# indicates whether the thread should be terminated.
		self.shutdown_flag = threading.Event()
 
		#Defining Camera and settings
		self.camera = picamera.PiCamera()
		self.camera.resolution = (width, height)
		self.framerate = framerate
		self.camera.framerate = framerate
		self.camera.clock_mode = 'raw'

		self.csvsheet = None
		self.output = None
		self.frame_index = 0
		self.prev_frame_index = 0
		self.folderName = ""

		#Defining a lock for picture saving
		self.picture_lock = threading.Lock()

		self.actual_GPS_time = {}
		
		#StartingCamera
		self.camera.start_preview()

		#Init background marking
		self.camera.annotate_background = picamera.Color('black')
		self.camera.annotate_text_size = 16

		self.wait_time = 1/framerate
		time.sleep(12.)
		print("Start recording frame")
 
	def run(self):
		global RECORDING_PATH
		global FOOTAGE_LENGTH
		global great_timeStruct
		
		#Sync GPU and RPI Clock for the first time
		self.syncRPiGPU()
		
		#Wait until we get some time data from GPS
		while not great_timeStruct.TGps:
			#Just wait for serial data to come.
			time.sleep(0.01)

		#Ask the instant now in GPS frame to create file name
		time_now_GPSbase = self.gpsTimeNow(self.camera.timestamp)


		#Name The file (Video and CSV)
		self.csvsheet, self.csvwriter = self.open_csv_sheet(self.time2string(time_now_GPSbase, YMDHMSuS = [1,1,1,1,1,1,0]))

		#Begin streaming
		self.camera.start_recording(self, format='mjpeg')
		
		print('Thread camera started')
		prev_frame_index = 0
 
		while not self.shutdown_flag.is_set():
			n = 0
			sync_done = False	

			if n % self.framerate == 0 and not sync_done:
				#Sync GPU and CPU every second (n % framerate)
				self.syncRPiGPU()
				sync_done = True
				self.camera.wait_recording(0.0001)

			else:
				self.camera.wait_recording(0.0001)
				sync_done = False
				continue

		self.destruct()
		print('Thread camera stopped')

	def open_csv_sheet(self, title):
		if self.csvsheet:
			self.csvsheet.close()

		csvsheet = open(title, mode='w')
		csvwriter = csv.writer(csvsheet, delimiter='-', quotechar='"', quoting=csv.QUOTE_MINIMAL)
 		csvwriter.writerow(['index', 'year', 'month', 'day', 'hour', 'minute', 'second', 'usecond'])
		return csvsheet, csvwriter
	
	def gpsTimeNow(self, TSinGPUframe):
		global great_timeStruct
		time_lock.acquire()
		time_frame_GPSbase = great_timeStruct.time_sync(TSinGPUframe)
		time_lock.release()
		return time_frame_GPSbase
		

	def syncRPiGPU(self):
		global great_timeStruct
		time_lock.acquire()
		first_shot = time.time()
		great_timeStruct.TGPU = self.camera.timestamp
		great_timeStruct.TRpi_for_GPU = (time.time() + first_shot) / 2
		time_lock.release()

	def time2string(self, time, YMDHMSuS = [1,1,1,1,1,1,1]):
		'''
		This fonction convert time dict into a string Year, Month, Day, Hour, Minute, Second, uSecond for 
		the order of YMDHMSuS if YMDHMSuS[3] = 0 then Hour won be in the string
		'''
		keys = ["year", "month", "day", "hour", "minute", "second", "usecond"]
		string = ""
		if time is None:
			return "Dropped Frame"

		for i in range(len(YMDHMSuS)):
			if YMDHMSuS[i]:
				string = string+ str(time[keys[i]]) +"-"
			else:
				continue
		return string[:-1]
			
	
	def destruct(self):
		self.camera.stop_recording()
		self.camera.close()

	def write(self, buf):
		global RECORDING_PATH
		global FOOTAGE_LENGTH

		if buf.startswith(b'\xff\xd8'):

      # Start of new frame; close the old one (if any) and
      # open a new output
			if self.output:
				self.output.close()
									
			with self.picture_lock:
				if self.frame_index % (self.framerate * FOOTAGE_LENGTH) == 0:
					self.folderName = RECORDING_PATH + "/" + self.time2string(self.gpsTimeNow(self.camera.timestamp), YMDHMSuS = [1,1,1,1,1,1,0])
					self.createFolder(self.folderName)
					self.csvsheet, self.csvwriter = self.open_csv_sheet(self.folderName + '/time_record.csv')
					print("create folder")

				self.frame_index = self.camera.frame.index
				self.output = io.open(self.folderName +'/'+ str(self.frame_index)+'.jpg', 'wb')
		 		self.output.write(buf)
				self.prev_frame_index = self.frame_index
				self.actual_GPS_time = self.gpsTimeNow(self.camera.frame.timestamp)

				if not self.actual_GPS_time is None:
					self.csvwriter.writerow([str(self.frame_index), str(self.actual_GPS_time["year"]), str(self.actual_GPS_time["month"]), str(self.actual_GPS_time["day"]), str(self.actual_GPS_time["hour"]), str(self.actual_GPS_time["minute"]), str(int(self.actual_GPS_time["second"])), str(self.actual_GPS_time["usecond"])])
				else:
					self.csvwriter.writerow(["None"])

	def createFolder(self, directory):
		try:
			if not os.path.exists(directory):
				os.makedirs(directory)
		except OSError:
			print ('Error: Creating directory. ' + directory)
 
class ServiceExit(Exception):
	"""
	Custom exception which is used to trigger the clean exit
	of all running threads and the main program.
	"""
	pass
 
 
def service_shutdown(signum, frame):
	print('Caught signal %d' % signum)
	raise ServiceExit
 
 
def main():
 
	# Register the signal handlers
	signal.signal(signal.SIGTERM, service_shutdown)
	signal.signal(signal.SIGINT, service_shutdown)
 
	print('Starting main program')
 
	# Start the job threads
	try:
		threadTime = TheDoctor()
		threadCamera = RecorderStamper()
		threadTime.start()
		threadCamera.start()
 
		# Keep the main thread running, otherwise signals are ignored.
		while True:
			time.sleep(1)
 
	except ServiceExit:
		# Terminate the running threads.
		# Set the shutdown flag on each thread to trigger a clean shutdown of each thread.
		threadTime.shutdown_flag.set()
		threadCamera.shutdown_flag.set()
		# Wait for the threads to close...
		threadTime.join()
		threadCamera.join()
 
	print('Exiting main program')
 
 
if __name__ == '__main__':
	main()
