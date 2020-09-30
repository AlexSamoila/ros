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
max_dist=np.zeros(480)
distanta=np.zeros((480,640))
minim=np.zeros(640)
def callback(msg):
	#global umin,umax,increment
	global pub
	global My_mat
	global maxim
	global max_dist
	global distanta	
	global minim
	h_rob=0.287
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
 	   
	
	umin=-0,4
	umax=0.4
	increment=0.0016	

	for i in range(240):          #pana la jumate, distanta maxima va fi 10 m
	 for j in range(nr_columns):
	   maxim[i][j]=10                  #matricea de maxime are valoarea 10

	for i in range(240,nr_rows):    #de la jumate in jos calculam distnata in functie de unghi 
	  alfa=(i-239)*increment         # si de inaltimea robotului
	  for j in range(nr_columns):
		maxim[i][j]=h_rob/np.sin(math.fabs(alfa))
		if (maxim[i][j]>10):       #daca depaseste rangeul atunci maximul va fi 10 m
		 maxim[i][j]=10
	

	
	for i in range(239):    #pentru partea de sus
	 beta=(239-i)*increment   #calculam unghiurile si pentru fiecare, distanta maxima pe  orizontala
	 max_dist[i]=maxim[i][0]*np.cos(math.fabs(beta))
	 for j in range(nr_columns):
	      if (np.isnan(My_mat[i][j])):  #apoi daca nu e nan calculez distanta
               distanta[i][j]=np.nan
	      else:
		distanta[i][j]=(max_dist[i]*My_mat[i][j])/maxim[i][0] #distanta pana la obstacol pe oriz

	for j in range(nr_columns):
	 distanta[239][j]=My_mat[239][j]   #linia de la jumate va fi exact aia
	 print(distanta[239][j])

	for i in range(240,nr_rows):  #pentru partea de jos calculam distanta maxima pe orizontal pt fiecare linie
	 max_dist[i]=math.sqrt((maxim[i][0]*maxim[i][0])-(h_rob*h_rob))
	 for j in range(nr_columns): 
	      if (np.isnan(My_mat[i][j])):   #daca nu e nan calculez distanta
               distanta[i][j]=np.nan
	      else:
		distanta[i][j]=(max_dist[i]*(maxim[i][0]-My_mat[i][j]))/maxim[i][0]
	
	for j in range(nr_columns):  #
	 #for i in range(nr_rows):
	 minim[j]=min(distanta[:,j])

	

	ls=LaserScan()
	
	ls.angle_min=-0.52
	ls.angle_max=0.52
 	ls.angle_increment=0.0016
	
	ls.time_increment=0	
	ls.scan_time=0
	ls.range_min=0.5
	ls.range_max=10
	ls.ranges=np.copy(minim)
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
