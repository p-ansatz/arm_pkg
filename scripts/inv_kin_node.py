#!/usr/bin/env python

from arm_pkg.srv import *
from arm_pkg.msg import *
import rospy
from std_msgs.msg import *
from math import cos,sin,acos,asin,atan,pi
from geometry_msgs.msg import Point

class CinematicaInversaNodo():

	def __init__(self):
		
		# ------ PARAMETRI ------
		self.L1 = rospy.get_param('eod/arm/l1',0.0955)
		self.L2 = rospy.get_param('eod/arm/l2',0.105)
		self.L3 = rospy.get_param('eod/arm/l3',0.0957)
		self.L4 = rospy.get_param('eod/arm/l4',0.1)

		self.t1_in = pi/2
		self.t2_in = pi/2+pi/4
		self.t3_in = pi/4
		self.t4_in = 0.0

		self.delta_q = rospy.get_param('eod/arm/delta_q',5)
		# ------ SERVIZI ------
		# Servizio per il calcolo della cinematica inversa		
		rospy.init_node('cin_inv_nodo')
		cinInv= rospy.Service('cinematica_inversa', CinematicaInversa, self.cinematica_inversa_callback)

		#Px=0.0 , Py_riposo=0.12 , Pz=0.17
	def cinematica_inversa_callback(self, msg):

		L1 = self.L1
		L2 = self.L2
		L3 = self.L3
		L4 = self.L4

		t1 = self.t1_in
		t2 = self.t2_in
		t3 = self.t3_in
		t4 = self.t4_in

		Px = msg.P.x
		Py = msg.P.y
		Pz = msg.P.z

		if(Px==0.0):
			Px=0.001
		if(Py==0.0):
			Py=0.001
		if Px == 0.0:
			t1 = pi/2
		elif ( Px > 0.0 ):
			t1 = atan(Py/Px)
		else:
			t1 = pi+atan(Py/Px)		

		l = Py/sin(t1)
		H = L1+L2*sin(t2)+L3*sin(t2+t3-pi)
		Q = L4+L2*cos(t2)+L3*cos(t2+t3-pi)
	
		while(Q<l):
			t2 = t2-(pi/180)
			t3 = pi-t2+asin((H-L1-L2*sin(t2))/L3)
			Q = L4+L2*cos(t2)+L3*cos(t2+t3-pi)

		alpha=t2+t3-pi

		if(Pz>H):
			while(Pz>H):
				alpha = alpha+(pi/180)
				t2 = acos((Q-L4-L3*cos(alpha))/L2)
				t3 = alpha-t2+pi
				H = L1+L2*sin(t2)+L3*sin(t2+t3-pi)
		else:
			while(Pz<H):
				alpha = alpha-(pi/180)
				t2 = acos((Q-L4-L3*cos(alpha))/L2)
				t3 = alpha-t2+pi
				H = L1+L2*sin(t2)+L3*sin(t2+t3-pi)

		t4=pi-t2-t3

		q1 = t1*180/pi
		q2 = t2*180/pi
		q3 = t3*180/pi
		q4 = 90+t4*180/pi
		alpha=alpha*180/pi

		q1 = q1-90
		q2 = q2-45
		q3 = q3-135
		q4 = q4-90

		print "q1 :",q1,"q2 :",q2,"q3 :",q3,"q4 :",q4
		print ""
		print "Q"
		print Q
		print "H"
		print H


		return CinematicaInversaResponse([q1,q2,q3,q4])

	def loop(self):
		rospy.spin()

if __name__ == "__main__":
	cin = CinematicaInversaNodo()
	cin.loop()