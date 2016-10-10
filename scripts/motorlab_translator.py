import rospy
from motorlab_msgs.msg import MotorLab_Arduino
from motorlab_msgs.msg import MotorLab_Arduino_Translation


Arduino_result = MotorLab_Arduino()
Arduino_translation = MotorLab_Arduino_Translation()

def ArduinoCb(data):
	Arduino_result = data

def publishArduino():
	Arduino_translation.dc_motor_position = Arduino_result.dc_motor_position
	Arduino_translation.dc_motor_speed = Arduino_result.dc_motor_speed
	Arduino_translation.servo_position = Arduino_result.servo_position
	Arduino_translation.stepper_motor_position = Arduino_result.stepper_motor_position
	Arduino_translation.temperature = Arduino_result.temperature
	Arduino_translation.light_gate_state = Arduino_result.light_gate_state
	Arduino_translation.ultrasonic_distance = Arduino_result.ultrasonic_distance
	Arduino_translation.ir_distance = Arduino_result.ir_distance
	arduino_pub.publish(Arduino_translation)

if __name__ == '__main__':

  # Init Node
  rospy.init_node('Arduino_Translator')
  
  arduino_sub = rospy.Subscriber("ArduinoMsg", MotorLab_Arduino, ArduinoCb)
  arduino_pub = rospy.Publisher("ArduinoTranslated", MotorLab_Arduino_Translation, queue_size = 1)

  rate = rospy.Rate(120) #hz
  while not rospy.is_shutdown():
    publishArduino()
    rate.sleep()