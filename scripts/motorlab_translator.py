#!/usr/bin/env python

import rospy
from motorlab_msgs.msg import MotorLab_Arduino
from motorlab_msgs.msg import MotorLab_Arduino_Translation

class Arduino_Translator(object):
	"""docstring for Arduino_Translator"""
	def __init__(self):
		self.Arduino_result = MotorLab_Arduino()
		self.Arduino_translation = MotorLab_Arduino_Translation()

		self.arduino_sub = rospy.Subscriber("ArduinoMsg", MotorLab_Arduino, self.ArduinoCb)
		self.arduino_pub = rospy.Publisher("ArduinoTranslated", MotorLab_Arduino_Translation, queue_size = 1)
		
	def ArduinoCb(self,data):
		self.Arduino_result = data

	def publishArduino(self):
		self.Arduino_translation.dc_motor_position = self.Arduino_result.dc_motor_position
		self.Arduino_translation.dc_motor_speed = self.Arduino_result.dc_motor_speed
		self.Arduino_translation.servo_position = self.Arduino_result.servo_position
		self.Arduino_translation.stepper_motor_position = self.Arduino_result.stepper_motor_position
		self.Arduino_translation.temperature = self.Arduino_result.temperature
		self.Arduino_translation.light_gate_state = self.Arduino_result.light_gate_state
		self.Arduino_translation.ultrasonic_distance = self.Arduino_result.ultrasonic_distance
		self.Arduino_translation.ir_distance = self.Arduino_result.ir_distance
		self.arduino_pub.publish(self.Arduino_translation)

if __name__ == '__main__':

  # Init Node
  rospy.init_node('Arduino_Translator')

  ATranslator = Arduino_Translator()

  rate = rospy.Rate(120) #hz
  while not rospy.is_shutdown():
    ATranslator.publishArduino()
    rate.sleep()