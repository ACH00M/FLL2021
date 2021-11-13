
# imports
from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
import math

# setup
hub = PrimeHub()
hub.light_matrix.show_image('HAPPY')
hub.motion_sensor.reset_yaw_angle()
wheels = MotorPair('A','D')
left_motor = Motor('E')
right_motor = Motor('C')


WHEEL_DIAMETER_CM = 6.24



def move_forward(self, amount, unit, steering, speed):
    """move_forward is an improved version of MotorPair.move
    Improvements:
        - Use yaw value to correct direction automatically
        - Have acceleration and deceleration at begin and end of movement
          for smoothness and avoid too much friction on wheels or wiggling
    """

    hub.motion_sensor.reset_yaw_angle()
    
    # steering is ignored, code only works with steering equals 0
    assert(steering == 0)
    # Code based in cm for simplicity, convert amount to cm first
    distance_cm = 0
    if unit == 'cm':
        distance_cm = amount
    elif unit == 'in':
        distance_cm = amount*2.54
    elif unit == 'mm':
        distance_cm = amount/10
    else:
        raise Exception("Invalid unit")

    # Calculation for relating the number of rotations and wheel circumference
    # Circumference of our wheel calculation        
    distance_per_rotation_cm = WHEEL_DIAMETER_CM * math.pi

    # This Solves for rotation blind-spot (only seeing left motor or only seeing right motor)
    # Creates an averaged degrees counted for left motor and right motor
    # _move_wrapper.pair.primary().get()[1] is internal access to Motor.get_degrees_counted
    average_degrees_counted_LeftandRight = (self._move_wrapper.pair.primary().get()[1] +
                                            self._move_wrapper.pair.secondary().get()[1]*-1) /2

    #Plugs our inputed distance in cm into the ratio (360 is there to convert motor rotations to desired motor degrees)
    degreesNecessary = distance_cm / distance_per_rotation_cm * 360
    #Uses our degreesNecessary to find the total degrees necessary (solving for the motor being in a different place everytime)
    totalDegreesNecessary = degreesNecessary + average_degrees_counted_LeftandRight
    #Drives straight until the degrees counted from the motors is bigger or equal to the total necessary degrees
    # power_down = False
    # power_up = False
    # # start half power to accelerate in while loop
    # speed //=2

    while(average_degrees_counted_LeftandRight <= totalDegreesNecessary):
        #Continues to update average degrees
        average_degrees_counted_LeftandRight = (self._move_wrapper.pair.primary().get()[1] +
                                                self._move_wrapper.pair.secondary().get()[1]*-1) /2

        #MOVE FORWARD
        #define angle based on gyro sensor
        deviation_angle = hub.motion_sensor.get_yaw_angle()

        correction = deviation_angle*1

        # if((not power_down) and (average_degrees_counted_LeftandRight >= (totalDegreesNecessary - degreesNecessary*0.3))):
        #     power_down = True
        #     speed //= 2
        #Move forward (till it exits while loop) with correction as steering with our desired power
        self.start(correction, speed)

    #Now the robot stops once it exits the while loop- we are now where we need to be
    self.stop()

# Monkey patch move method to add gyro error correction code
MotorPair.old_move = MotorPair.move
MotorPair.move = move_forward

# platooning trucks
wheels.move(21, 'cm', 0, 40)
wheels.old_move(7.01 * math.pi / 4, 'cm', 100, 20)
wheels.move(60, 'cm', 0, 40)