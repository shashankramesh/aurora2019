#!/usr/bin/env python
import rospy
from math import *
import time
import sys
from termcolor import colored

from sensor_msgs.msg import NavSatFix
from navigation.msg import Goal,Planner_state
from navigation.srv import plan_state
from threading import Thread


class GPS() :
	def __init__(self):
		rospy.init_node("gdm")
		#rospy.wait_for_service('Planner_state_ctrl')
		self.state_srv = rospy.ServiceProxy('Planner_state_ctrl', plan_state)
		
		self.pub_goal = rospy.Publisher('goal', Goal,queue_size=10) #Publisher to planner
		rospy.Subscriber("fix", NavSatFix, self.gpsCallback) #From nmea node
		rospy.Subscriber("planner_state",Planner_state, self.plannerCallback)
		
		'''file_path = "/home/anveshak/aurora2018/src/navigation/config/gps_data.txt"
		try:
			self.f=open(file_path,'r')
			self.dest_lat_cont,self.dest_lon_cont = [],[]
			for l in self.f:
				row = l.split()
				self.dest_lat_cont.append(row[0])
				self.dest_lon_cont.append(row[1])
		except Exception:
			print colored("GPS data file not opened",'red')'''
			
		self.dest_lat_cont,self.dest_lon_cont = [],[]
		print "Enter GPS way-points one by one in latitude<>longitude format"
		print colored('$ Type ok once done', 'green')
		l = raw_input('GPSprompt>>>')
		while (l != 'ok'):
			row = l.split()
			self.dest_lat_cont.append(row[0])
			self.dest_lon_cont.append(row[1])
			l = raw_input('GPSprompt>>>')
			
		self.curr_lat = 0.0
		self.curr_lon = 0.0
		self.bearing=0
		self.planner_status = 0
		self.distance = 0
		
		
		
	def run(self):
		goal = Goal()
		
		while not rospy.is_shutdown():
			for i in range(len(self.dest_lat_cont)): 
				self.dest_lat= float(self.dest_lat_cont[i])
				self.dest_lon= float(self.dest_lon_cont[i])
				self.dist_gps,self.bearing=self.cal()
			
				goal.distance = self.dist_gps
				goal.bearing = self.bearing
				self.pub_goal.publish(goal)
				time.sleep(0.001)
			
				while(~self.planner_status):
					try:
						self.dist_gps,self.bearing=self.cal()
						goal.distance = self.dist_gps
						goal.bearing = self.bearing
						self.pub_goal.publish(goal)
						time.sleep(0.001)
					except KeyboardInterrupt:
						sys.exit()
				#ball detecton service
				self.srv("rst")
				
			print 'Successfully past all waypoints'
		
	
	def srv(self,arg):
		temp = [0,0,0]
		if(arg == "pause"):
			print "Pausing...."
			temp[0] = 1
		elif(arg == "contin"):
			print "Continuing..."
			temp[1] = 1
		elif(arg == "rst"):
			print "Resetting..."
			temp[2] = 1
		else:
			return colored('Error','red')
			
		try:
			resp = self.state_srv(temp[0],temp[1],temp[2])
			print "Service response: %s"%resp
			return resp
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			return 'Error'
	
	def cal(self):
		lon1, lat1, lon2, lat2 = map(radians, [self.curr_lon, self.curr_lat, self.dest_lon, self.dest_lat])
		dlon = lon2 - lon1
		dlat = lat2 - lat1
		a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
		c = 2 * atan2(sqrt(a), sqrt(1-a))
		dist_gps = 6371 * c*1000
		bearing = atan2(sin(lon2-lon1)*cos(lat2), (cos(lat1)*sin(lat2))-(sin(lat1)*cos(lat2)*cos(lon2-lon1)))
		bearing = degrees(bearing)
		return dist_gps,bearing
		
	def gpsCallback(self,msg):
		self.curr_lat = msg.latitude
		self.curr_lon = msg.longitude
		self.status = msg.status
		
	def plannerCallback(self,msg):
		self.planner_status = msg.status
		
	def key_intrp(self):
		print "\n Type 'p' to pause ----- 'c' to contin ----- 'r' to rst the Planner \n" 
		while True:
        		try:
        			text = raw_input('$GPS_node>>> ')
				if (text == 'p'):
  					self.srv("pause")
		  		elif (text == 'c'):
		  			self.srv("cotin")
		  		elif (text == 'r'):
		  			self.srv("rst")
		  		else:
		  			print 'Invalid command'
			except KeyboardInterrupt:
			  	print 'Stopping....'
			  	sys.exit()
	
if __name__ == '__main__':
	x = raw_input('Do you want to start the node? (y/n) : ')
	
	gps = GPS()
	
	#gps.key_intrp()
	if(x == 'y'):
		gps.run()
	else:
		sys.exit()
		