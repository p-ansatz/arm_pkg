#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from arm_pkg.msg import *
from arm_pkg.srv import *
from gpiozero import AngularServo

class AttuatoreNodo():

	def __init__(self):
		rospy.init_node('attuatore_nodo')

		# ------ PARAMETRI ------
		# frequenza del loop 
		self.freq = rospy.get_param('eod/loop_freq/arm',2)
		self.rate = rospy.Rate(self.freq)

		# angoli riposo
		self.q1_rip = rospy.get_param('eod/arm/riposo1',0)
		self.q2_rip = rospy.get_param('eod/arm/riposo2',90)
		self.q3_rip = rospy.get_param('eod/arm/riposo3',-90)
		self.q4_rip = rospy.get_param('eod/arm/riposo4',0)
		self.q5_rip = rospy.get_param('eod/arm/riposo5',90)

		# angoli min-max pinza
		self.max_angle_pinza = rospy.get_param('eod/arm/min_angle_pinza',90)
		self.min_angle_pinza = rospy.get_param('eod/arm/max_angle_pinza',-90)

		# pin motori
		pin1 = rospy.get_param('eod/arm/pin1',26)
		pin2 = rospy.get_param('eod/arm/pin2',19)
		pin3 = rospy.get_param('eod/arm/pin3',13)
		pin4 = rospy.get_param('eod/arm/pin4',6)
		pin5 = rospy.get_param('eod/arm/pin5',5)

		# motori servo
		self.servo1 = AngularServo(pin1,initial_angle=self.q1_rip,min_angle=-90,max_angle=90,min_pulse_width=0.6/1000, max_pulse_width=2.27/1000, frame_width=20.0/1000)
		self.servo2 = AngularServo(pin2,initial_angle=self.q2_rip,min_angle=-90,max_angle=90,min_pulse_width=0.65/1000, max_pulse_width=2.6/1000, frame_width=20.0/1000)
		self.servo3 = AngularServo(pin3,initial_angle=self.q3_rip,min_angle=-90,max_angle=90,min_pulse_width=0.65/1000, max_pulse_width=2.6/1000, frame_width=20.0/1000)
		self.servo4 = AngularServo(pin4,initial_angle=self.q4_rip,min_angle=-90,max_angle=90,min_pulse_width=0.6/1000, max_pulse_width=2.25/1000, frame_width=20.0/1000)
		self.servo5 = AngularServo(pin5,initial_angle=self.q5_rip,min_angle=-90,max_angle=90,min_pulse_width=1.2/1000, max_pulse_width=1.9/1000, frame_width=20.0/1000)
		
		# ------ VARIABILI ------
		self.q1_traj = []
		self.q2_traj = []
		self.q3_traj = []
		self.q4_traj = [] 

		self.flag_traj = False

		self.q1_cur = rospy.get_param('eod/arm/riposo1',0)
		self.q2_cur = rospy.get_param('eod/arm/riposo2',-90)
		self.q3_cur = rospy.get_param('eod/arm/riposo3',0)
		self.q4_cur = rospy.get_param('eod/arm/riposo4',90)

		# ------ TOPIC ------
		# Topic sul quale vengono pubblicate le traiettorie da inseguire
		self.sub_traj = rospy.Subscriber('traiettoria', TraiettoriaDiscretizzata, self.traiettoria_callback)
		# Topic sul quale viene pubblicata l'azione che deve compiere la pinza
		self.sub_pinza = rospy.Subscriber('stato_pinza', Bool, self.pinza_callback)
		# Topic sul quale viene pubblicato il raggiungimento della posa
		self.pub_posa_raggiunta = rospy.Publisher('posa_raggiunta', Bool, queue_size = 10)		
		# Topic sul quale si scambiano info sulle variabili di giunto: q_i
		self.pub_giunti = rospy.Publisher('stato_giunti', Giunti, queue_size = 10)
	
	def loop(self):
		#print "loop"
		while not rospy.is_shutdown():

			if (not self.q1_traj) and (not self.q2_traj) and (not self.q3_traj) and (not self.q4_traj):
				# tutte le liste qi_traj sono state svuotate
				if self.flag_traj:
					self.flag_traj = False
					self.pub_posa_raggiunta.publish(True)
					self.pub_giunti.publish(self.q1_cur,self.q2_cur,self.q3_cur,self.q4_cur)

			else:
				if self.q1_traj:
					# la lista q1_traj non e' vuota
					self.q1_cur = self.q1_traj.pop(0)
					# attuazione giunto q1
					self.servo1.angle=int(self.q1_cur)
					#print "q1"
					#print self.q1_cur

				if self.q2_traj:
					# la lista q2_traj non e' vuota
					self.q2_cur = self.q2_traj.pop(0)
					# attuazione giunto q2
					self.servo2.angle=int(self.q2_cur)
					#print "q2"
					#print self.q2_cur

				if self.q3_traj:
					# la lista q3_traj non e' vuota
					self.q3_cur = self.q3_traj.pop(0)
					# attuazione giunto q3
					self.servo3.angle=int(self.q3_cur)
					#print "q3"
					#print self.q3_cur

				if self.q4_traj:
					# la lista q4_traj non e' vuota
					self.q4_cur = self.q4_traj.pop(0)
					# attuazione giunto q4
					self.servo4.angle=int(self.q4_cur)
					#print "q4"
				
				print "q1:",self.q1_cur," q2:",self.q2_cur," q3:",self.q3_cur," q4:",self.q4_cur
				#print self.q1_cur,self.q2_cur,self.q3_cur,self.q4_cur				
			
			print "q1:",self.q1_cur," q2:",self.q2_cur," q3:",self.q3_cur," q4:",self.q4_cur
			self.rate.sleep()


	def traiettoria_callback(self, msg):
		# msg = TraiettoriaDiscretizzata da inseguire
	
		self.flag_traj = True

		self.q1_traj = list(msg.q1)
		self.q2_traj = list(msg.q2)
		self.q3_traj = list(msg.q3)
		self.q4_traj = list(msg.q4)

		#print self.q2_traj
	
	def pinza_callback(self, msg):

		print msg
		if msg.data:
			print "pinza aperta"	
			# aprire pinza
			self.servo5.angle = self.min_angle_pinza
		else:
			print "pinza chiusa"
			# chiudere pinza
			self.servo5.angle = self.max_angle_pinza 	

if __name__ == "__main__":	
	an = AttuatoreNodo()
	an.loop()

