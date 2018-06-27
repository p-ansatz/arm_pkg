#! /usr/bin/env python

import rospy
import actionlib
import time

from arm_pkg.msg import *
from arm_pkg.srv import *

from std_msgs.msg import String, Int16, Bool
from geometry_msgs.msg import Point

class ElementaryArmActionServer():
	def __init__(self):
		rospy.init_node('elementary_arm_action_node')	

		# ------ PARAMETRI ------
		# frequenza del loop 
		self.fq = rospy.get_param('/eod/loop_freq/default')
		self.rate = rospy.Rate(self.fq)
		# incremento minimo angolo per discretizzazione
		self.delta_q = rospy.get_param('eod/arm/delta_q',5)

		# ------ VARIABILI ------
		self.q1_rip = rospy.get_param('eod/arm/riposo1',0)
		self.q2_rip = rospy.get_param('eod/arm/riposo2',90)
		self.q3_rip = rospy.get_param('eod/arm/riposo3',-90)
		self.q4_rip = rospy.get_param('eod/arm/riposo4',0)
		self.q = [self.q1_rip, self.q2_rip, self.q3_rip, self.q4_rip]
		self.q_old = self.q		
		

		self.q1_traj = []
		self.q2_traj = []
		self.q3_traj = []
		self.q4_traj = [] 

		self.posa_raggiunta_flag=False

		# ------ AZIONI ------
		self.arm_pose_server = actionlib.SimpleActionServer('arm_pose', ArmPoseAction, self.arm_pose_callback, False)
		self.arm_pose_server.start()
		self.arm_claw_server = actionlib.SimpleActionServer('arm_claw', ArmClawAction, self.arm_claw_callback, False)
		self.arm_claw_server.start()

		# ------ TOPIC ------
		self.pub_stato_pinza = rospy.Publisher('stato_pinza', Bool, queue_size = 10)	
		self.pub_traj = rospy.Publisher('traiettoria', TraiettoriaDiscretizzata, queue_size = 10)
		self.subPosaRaggiunta = rospy.Subscriber('posa_raggiunta', Bool, self.posa_raggiunta_callback)
		# Topic sul quale si scambiano info sulle variabili di giunto: q_i
		self.sub_giunti = rospy.Subscriber('stato_giunti', Giunti, self.stato_giunti_callback)
		
		# ------ SERVIZI ------
		# Servizio per il calcolo della cinematica inversa
		rospy.wait_for_service('cinematica_inversa')
		self.risolviCinematica = rospy.ServiceProxy('cinematica_inversa', CinematicaInversa)

	def arm_claw_callback(self, msg):
		self.pub_stato_pinza.publish(msg.open)
		time.sleep(1) # attende apertura/chiusura pinza
		result = ArmClawResult()
		result.res.data = 'success'
		self.arm_claw_server.set_succeeded(result)

	def arm_pose_callback(self, msg):
		goal = msg #point
		print goal
		self.q = self.risolviCinematica(msg.p)
		self.q = self.q.q
		#print self.q
		self.discretizza_traiettoria()
		self.pubblica_traiettoria()
		while not self.posa_raggiunta_flag:
			pass # we love busy-waiting, fuck D'Angelo!! 
		result = ArmPoseResult()
		result.res.data = 'success'
		self.arm_pose_server.set_succeeded(result)
		self.posa_raggiunta_flag = False
	
	def discretizza_traiettoria(self):
		# discretizza traiettoria tra q_old e q
		q_in = self.q_old
		print q_in
		print self.q

		if(q_in[0] < self.q[0]):
			while(q_in[0] < self.q[0]):
				q_in[0] = q_in[0] + self.delta_q
				self.q1_traj.append(q_in[0])
		else:
			while(q_in[0] > self.q[0]):
				q_in[0] = q_in[0] - self.delta_q
				self.q1_traj.append(q_in[0])


		if(q_in[1] < self.q[1]):
			while(q_in[1] < self.q[1]):
				q_in[1] = q_in[1] + self.delta_q
				self.q2_traj.append(q_in[1])
		else:
			while(q_in[1] > self.q[1]):
				q_in[1] = q_in[1] - self.delta_q
				self.q2_traj.append(q_in[1])


		if(q_in[2] < self.q[2]):
			while(q_in[2] < self.q[2]):
				q_in[2] = q_in[2] + self.delta_q
				self.q3_traj.append(q_in[2])
		else:
			while(q_in[2] > self.q[2]):
				q_in[2] = q_in[2] - self.delta_q
				self.q3_traj.append(q_in[2])

		if(q_in[3] < self.q[3]):

			while(q_in[3] < self.q[3]):
				q_in[3] = q_in[3] + self.delta_q
				self.q4_traj.append(q_in[3])
		else:

			while(q_in[3] > self.q[3]):
				q_in[3] = q_in[3] - self.delta_q
				self.q4_traj.append(q_in[3])


	def pubblica_traiettoria(self):
		traj = TraiettoriaDiscretizzata()
		traj.q1 = self.q1_traj
		traj.q2 = self.q2_traj
		traj.q3 = self.q3_traj
		traj.q4 = self.q4_traj

		self.pub_traj.publish(traj)	

		self.q1_traj = []
		self.q2_traj = []
		self.q3_traj = []
		self.q4_traj = []
	
	def stato_giunti_callback(self, msg):
		self.q_old = msg.q1,msg.q2,msg.q3,msg.q4
		self.q_old = list(self.q_old)
		print "stato giunti"
		print self.q_old

	def posa_raggiunta_callback(self,msg):
		if msg.data == True:
			self.posa_raggiunta_flag=True
			


	def loop(self):
		while not rospy.is_shutdown():
			self.rate.sleep()


if __name__ == '__main__':
	eaas = ElementaryArmActionServer()
	eaas.loop()
