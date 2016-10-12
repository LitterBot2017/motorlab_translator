#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Bool
from motorlab_msgs.msg import MotorLab_Arduino
from motorlab_msgs.msg import MotorLab_Arduino_Translation

class Arduino_Translator(object):
	"""docstring for Arduino_Translator"""
	def __init__(self):
		self.Arduino_result = MotorLab_Arduino()
		self.Arduino_translation = MotorLab_Arduino_Translation()

		self.arduino_sub = rospy.Subscriber("ArduinoMsg", MotorLab_Arduino, self.ArduinoCb)
		self.arduino_pub = rospy.Publisher("ArduinoTranslated", MotorLab_Arduino_Translation, queue_size = 1)
		self.light_gate_pub = rospy.Publisher("LightGateState", Bool, queue_size = 1)
		self.button_pub = rospy.Publisher("ActuatorsOn", Bool, queue_size = 1)
		
	def ArduinoCb(self,data):
		self.Arduino_result = data

	def publishArduino(self):
		self.Arduino_translation.dc_motor_position = self.Arduino_result.dc_motor_position
		self.Arduino_translation.dc_motor_speed = self.Arduino_result.dc_motor_speed
		self.Arduino_translation.servo_position = self.Arduino_result.servo_position
		self.Arduino_translation.stepper_motor_position = self.Arduino_result.stepper_motor_position
		self.Arduino_translation.temperature = self.translate_temp(self.Arduino_result.temperature)
		self.Arduino_translation.light_gate_state = self.Arduino_result.light_gate_state
		self.Arduino_translation.ultrasonic_distance = self.Arduino_result.ultrasonic_distance
		self.Arduino_translation.ir_distance = self.translate_IR(self.Arduino_result.ir_distance)
		self.Arduino_translation.button_state = self.Arduino_result.button_state
		self.light_gate_pub.publish(bool(self.Arduino_result.light_gate_state))
		self.button_pub.publish(bool(self.Arduino_result.button_state))
		self.arduino_pub.publish(self.Arduino_translation)

	def translate_temp(self,rawADC):
		Temp = 0.0
		if rawADC > 0:
			# Temp = math.log(10000.0*((1024.0/rawADC-1))); 
			# Temp = math.log(10000.0/(1024.0/rawADC-1)) # for pull-up configuration
			# Temp = 1.0 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp ))* Temp );

			# Temp = Temp - 273.15;            # Convert Kelvin to Celcius
			# Temp = (Temp * 9.0)/ 5.0 + 32.0; # Convert Celcius to Fahrenheit
			Temp = 0.0968*rawADC-22.943
			Temp = (Temp * 9.0)/ 5.0 + 32.0; # Convert Celcius to Fahrenheit
		else:
			Temp = -300
		return Temp;

	def translate_IR(self,rawADC):
		dist = -0.072*rawADC+41.531
		if dist > 40.0:
			return 40.0
		return dist

	def translate_ultra(self, pulseWidth):
		dist = (pulseWidth*0.013)-4.4 #inches
		if dist > 40.0:
			return 40.0
		return dist

if __name__ == '__main__':

  # Init Node
  rospy.init_node('Arduino_Translator')

  ATranslator = Arduino_Translator()

  rate = rospy.Rate(120) #hz
  while not rospy.is_shutdown():
    ATranslator.publishArduino()
    rate.sleep()