#!/usr/bin/env python
import copy
import rospy
import math
import struct
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

pub=0
My_mat = np.zeros((480,640))
maxim = np.zeros((480,640))
def callback(msg):
	#global umin,umax,increment
	global pub
	global My_mat
	global maxim
	#maxim[:]=np.nan
	#My_mat = [[]] My_mat = np.zeros((nr_rows,nr_columns))
	nr_rows = msg.height
	nr_columns = msg.width
	pas=msg.step
	codare=msg.encoding
	rospy.loginfo('randuri:{} ,coloane:{} ,pas:{},enc:{}'.format(nr_rows,nr_columns,pas,codare))
	#My_mat = [[0 for x in range(nr_rows)] for y in range(nr_columns)]
        
	for i in range(nr_rows):
	 for j in range(nr_columns):
           elemente = [msg.data[i*nr_columns+4*j],msg.data[i*nr_columns+4*j+1],msg.data[i*nr_columns+4*j+2],msg.data[i*nr_columns+4*j+3]]
           My_byte_float = bytearray(elemente)#msg.data[i*nr_columns+4*j],msg.data[i*nr_columns+4*j+1],msg.data[i*nr_columns+4*j+2],msg.data[i*nr_columns+4*j+3])
	   My_float = struct.unpack('<f',My_byte_float) #little edian
	   #My_float = struct.unpack('>f',My_byte_float) #big edian
           My_mat[i][j]=My_float[0]
 	   
	
	
	#for i in range(nr_rows):
	 for j in range(nr_columns):
		#if maxim[i][j]<My_mat[i][j]:
		 #maxim[i][j]=My_mat[i][j]
		 print(My_mat[479][j])

	#float umin,umax,increment
	umin=-0,4
	umax=0.4
	increment=0.0016
	

	ls=LaserScan()
	
	ls.angle_min=-0.52
	ls.angle_max=0.52
 	ls.angle_increment=0.0016
	
	ls.time_increment=0	
	ls.scan_time=0
	ls.range_min=0.5
	ls.range_max=10
	ls.ranges=np.copy(My_mat[479,:])
	#ls.intensities
	pub.publish(ls)

def main():
	global pub
	global My_mat
	rospy.init_node('camera')
	My_mat = np.zeros((480,640))
	pub=rospy.Publisher("/mylidar/scan",LaserScan,queue_size = 10)
	#callback(rospy.wait_for_message("/camera/depth/image_raw",Image))	
	rospy.Subscriber("/camera/depth/image_raw",Image,callback)
      
    	rospy.spin()

if __name__=='__main__':
	main()	
