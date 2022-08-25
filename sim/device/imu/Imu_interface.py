#!usr/bin/env python
"""
Module for getting data from the 3DM-GX4-25. Need to have mscl.py and _mscl.so (generated after compiling mscl source) in the same folder for things to work.
This "modified_imu" just adds calibration to all 9 measurements (instead of 3 in imu.py)
"""

__author__ 	= "Josh Hooks/Jeffrey Yu"
__email__ 	= "c.jeffyu@gmail.com"
__date__	= "May 25, 2017"

import sys
#sys.path.append("../../dependencies/Python")
import mscl
import numpy as np
import time


class imu:
	def __init__(self, sample_rate=1000):
		self.FREQ = sample_rate
		self.data = {'scaledAccelX': 0.0, 'scaledAccelY': 0.0, 'scaledAccelZ': 0.0,
					 'scaledGyroX': 0.0, 'scaledGyroY': 0.0, 'scaledGyroZ': 0.0,
					 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
		#self.acc_bias = np.zeros(3, float)
		self.acc_bias = np.zeros(9, float)

		try:
			#Set up an inertia node
			COM_PORT = "/dev/ttyACM0"
			self.connection = mscl.Connection.Serial(COM_PORT, 921600)
			self.node = mscl.InertialNode(self.connection)

			#Build up channels
			chs = mscl.MipChannels()
			chs.append(mscl.MipChannel(mscl.MipTypes.CH_FIELD_SENSOR_SCALED_ACCEL_VEC, mscl.SampleRate.Hertz(self.FREQ)))
			chs.append(mscl.MipChannel(mscl.MipTypes.CH_FIELD_SENSOR_SCALED_GYRO_VEC, mscl.SampleRate.Hertz(self.FREQ)))
			chs.append(mscl.MipChannel(mscl.MipTypes.CH_FIELD_SENSOR_EULER_ANGLES, mscl.SampleRate.Hertz(self.FREQ)))	# Seems to be strictly better than ScaledAccelVec

			#Set active channels
			self.node.setActiveChannelFields(mscl.MipTypes.CLASS_AHRS_IMU, chs)
			#enableDataStream starts to sample, node.resume() does the same thing?
			self.node.enableDataStream(mscl.MipTypes.CLASS_AHRS_IMU)
			self.node.setToIdle()
		except mscl.Error as e:
			print("Error:", e)

	def calibBias(self):
		# Get bias for 3 acceleration channels
		run_time = 10.0  #Calibration time
		t_dt = np.linspace(0, run_time, int(run_time * self.FREQ + 1))
		# estm_acc = np.zeros((3, len(t_dt)), float)
		estm_acc = np.zeros((9, len(t_dt)), float)
		self.node.resume()
		t0 = time.perf_counter()
		for i in range(len(t_dt) - 1):
			q = np.array(self.getData())
			# estm_acc[:, i] = q[0:3]
			estm_acc[:, i] = q[:] # Written by JZ
			while (time.perf_counter() - t0 < t_dt[i + 1]):
				pass
		self.node.setToIdle()
		avg = np.average(estm_acc, 1)
		print("IMU Bias: %s" % (avg))
		self.acc_bias = avg

	def update(self):
		packets = self.node.getDataPackets(1)
		#packets = self.node.getDataPackets(5, 1)
		for packet in packets:
			# get all of the points in the packet
			points = packet.data()
			for dataPoint in points:
				self.data[dataPoint.channelName()] = dataPoint.as_float() #as_float() returns the dataPoint value as a 4-byte float
		#Channel names are:
		# scaledAccelX
		# scaledAccelY
		# scaledAccelZ
		# scaledGyroX
		# scaledGyroY
		# scaledGyroZ
		# roll
		# pitch
		# yaw

	def getData(self):
		self.update()
		p = self.data['pitch'] - self.acc_bias[6] #Rotation along y axis
		r = self.data['roll'] - self.acc_bias[7]  #Rotation along x axis
		y = self.data['yaw'] - self.acc_bias[8]  #Rotation along z axis
		R_b2i_alt = np.array([[np.cos(y)*np.cos(p), np.cos(y)*np.sin(r)*np.sin(p) - np.cos(r)*np.sin(y), np.sin(r)*np.sin(y) + np.cos(r)*np.cos(y)*np.sin(p)],
							  [np.cos(p)*np.sin(y), np.cos(r)*np.cos(y) + np.sin(r)*np.sin(y)*np.sin(p), np.cos(r)*np.sin(y)*np.sin(p) - np.cos(y)*np.sin(r)],
							  [-np.sin(p), np.cos(p)*np.sin(r), np.cos(r)*np.cos(p)]]).transpose()
		g_vec 	= np.dot(R_b2i_alt, np.array([0, 0, 1.0]))
		# acc = np.array([self.data['scaledAccelX'], self.data['scaledAccelY'], self.data['scaledAccelZ']]) + g_vec - self.acc_bias
		acc = np.array([self.data['scaledAccelX'], self.data['scaledAccelY'], self.data['scaledAccelZ']]) + g_vec - self.acc_bias[0:3]
		return [acc[0], acc[1], acc[2],
				self.data['scaledGyroX'] - self.acc_bias[3], self.data['scaledGyroY'] - self.acc_bias[4], self.data['scaledGyroZ']-self.acc_bias[5], p, r, y]


	def close(self):
		try:
			self.node.setToIdle()
		except mscl.Error_Connection:
			pass
		self.connection.disconnect()
		
	def __del__(self):
		self.close()

	# def sensorChannels(self):
	# 	nodeInfo = self.node.info()
	# 	channel_fields = nodeInfo.supportedChannelFields(mscl.InertialTypes.CATEGORY_SENSOR)
	# 	for field in channel_fields:
	# 		print mscl.InertialTypes.channelFieldToStr(field)

	# def filterChannels(self):
	# 	nodeInfo = self.node.info()
	# 	channel_fields = nodeInfo.supportedChannelFields(mscl.InertialTypes.CATEGORY_ESTFILTER)
	# 	for field in channel_fields:
	# 		print mscl.InertialTypes.channelFieldToStr(field)

	# def euler(self):
	# 	self.update()
	# 	return [self.data['estRoll'], self.data['estPitch'], self.data['estYaw']]
