#!/usr/bin/env python

#mg996r
from gpiozero import Servo
servo=Servo(26,min_pulse_width=0.6/1000,max_pulse_width=2.3/1000, frame_width=20.0/1000)

#mg995r
from gpiozero import Servo
servo=Servo(26,min_pulse_width=0.65/1000,max_pulse_width=2.6/1000, frame_width=20.0/1000)

#ultimo servo 
from gpiozero import Servo
servo=Servo(26,min_pulse_width=0.6/1000,max_pulse_width=2.25/1000, frame_width=20.0/1000)

#pinza
from gpiozero import Servo
servo=Servo(26,initial_value=0,min_pulse_width=1.2/1000,max_pulse_width=1.9/1000, frame_width=20.0/1000)


#angular servo
#pinza
from gpiozero import AngularServo
servo=AngularServo(26,initial_angle=-90,min_angle=-90,max_angle=90,min_pulse_width=1.2/1000, max_pulse_width=1.9/1000, frame_width=20.0/1000)

#ultimo servo 
from gpiozero import AngularServo
servo=AngularServo(26,initial_angle=-90,min_angle=-90,max_angle=90,min_pulse_width=1.2/1000, max_pulse_width=1.9/1000, frame_width=20.0/1000)

from gpiozero import AngularServo
servo=AngularServo(26,initial_angle=0,min_angle=-90,max_angle=90,min_pulse_width=0.6/1000, max_pulse_width=2.27/1000, frame_width=20.0/1000)

from gpiozero import AngularServo
servo=AngularServo(26,initial_angle=0,min_angle=-90,max_angle=90,min_pulse_width=0.6/1000, max_pulse_width=2.25/1000, frame_width=20.0/1000)