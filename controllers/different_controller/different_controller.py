# Authors: Jenna Mast, Spurah Nayak, Sara Hallam, with reference from Webots examples

# Project: CPE 416 Final - Follow the Leader Robots
# File description: Controller for Red Leader e-Puck robot. Drives leader robot around arena while avoiding obstacles.
# Status: Tested, works.Final version

from controller import Robot, Motor

import random

# Constants
LEFT = 0                # For object detected on left side
RIGHT = 1               # For object detected on Right side
BOTH = 2                # For object detected on both sides
TRUE = 1
FALSE = 0
NO_SIDE = -1            # No object detected on either side

# Sensor list - size
NUM_SENSORS = 8        # There are 8 IR distance sensors so this is the size of our sensor list.

# Sensor list - indices
DS_RIGHT_00 = 0         # Index number of front right distance sensor  (ps0)   
DS_RIGHT_45 = 1         # Index number of right front adjacent distance sensor (ps1)
zS_RIGHT_90 = 2         # Index number of right side distance sensor (ps2). Not used in this code but included for clarity and debugging.
DS_RIGHT_REAR = 3       # Index number of the back right distance sensor sensor (ps3) 
DS_LEFT_REAR = 4        # Index number of the back left distance sensor sensor (ps3) 
DS_LEFT_90 = 5          # Index number of right side distance sensor (ps5). Not used in this code but included for clarity and debugging.
DS_LEFT_45 = 6          # Index number of left front adjacent distance sensor (ps6)
DS_LEFT_00 = 7          # Index number of front left distance sensor  (ps7) 

# Sensor threshold  

DISTANCE_THRESHOLD = 100   # Minimum sensor value (distance) at which robot will respond. # Good value 10


#Sensor proportional coefficients

K_DS_45 = 0.3         # Good value 0.3  
K_DS_00 = 0.5         # Good value 0.5 


# Forward Speed

ao_FORWARD_SPEED = 300   # The forward speed of the robot

# Global variables
ds = [None] * NUM_SENSORS       # List that holds the IR sensor identifiers ("pso", "ps1", ...)
DS_value = [0] * NUM_SENSORS    # List that hold actual IR sensor values.
stuck_flag = 0

min_act_flag = 0

# Obstacle detector control flow variables

ao_active = False               # Set true when in obstacle detection mode (a sensor value gas exceeded DISTANCE_THRESHOLD)
ao_reset = False                # When true, resets obstacle detection state.
ao_speed = [0, 0]               # Stores the speeds to the motors.
ao_side = NO_SIDE               # Stores the side the obstacle is detected on.


# Random wandering variables

wanderCount = 0                 # Controls the length of time the robot proceeds in a single direction.              
randN1 = 1                      # Coefficient applied to speed of motor 1, introduces randomness for wandering.
randN2 = 2                      # Coefficient applied to speed of motor 2, introduces randomness for wandering.


# Sensing behavior oscillation variables

oCount = 0                      # Controls the rate of left right oscillation for the sensing behavior.
oscillator = 1                  # Coefficient applied to speeds of both motor for toggling for the sensing behavior.


# Start Definition of obstacle avoidance method

def AvoidObjects():

    # Indicate our global variables
    global ao_active, ao_reset, ao_speed, ao_side, wanderCount, oCount, randN1, randN2, oscillator
    
  
    SensorFusion = [0, 0]         # Sensor Fusion list. SensorFusion[left sensor fusion, right sensor fusion]
    max_ds_value = 0            # Holds the maximum sensor value detected.

        
    # Controls amount of wandering in forward driving mode. Changes direction every 40 counts
 
    if wanderCount == 0:
     #   randN1 = random.uniform(1, 2)  # For random float numbers 
        randN1 = random.choice([1, 2]) # For equal chance of 1 or 2.
     #    randN2 = random.uniform(1, 2)  # For random float numbers 
        randN2 = random.choice([1, 2]) # For equal chance of 1 or 2.
        wanderCount = 40
    else:
        wanderCount = wanderCount - 1
       
    #print("randN1", randN1)  #for debugging
    #print("randN2", randN2)  #for debugging 
    
    
   
    # Controlls the amount of oscillation in forward driving mode. Toggles side bias every 2 counts.
   
    if oCount == 0:
    
        oscillator = -1*oscillator

        oCount = 2
 
    else:
        oCount = oCount - 1
         
    
    # Reset objected detected state
    
    if ao_reset:
        ao_active = False            # Set active flag too false
        
        
        # Reset the side detected to NO_SIDE if no object is detected
        
        if SensorFusion[RIGHT] <= min_act_right + 10 and SensorFusion[LEFT] <= min_act_left + 10 : #400
            if(DS_value[DS_RIGHT_00] < DISTANCE_THRESHOLD and DS_value[DS_LEFT_00] < DISTANCE_THRESHOLD):
                ao_side = NO_SIDE
                
    
    ao_reset = False  # Set false in preparation to be set true again.
    
  
  
    # Find the maximum sensor value and fuse the sensor readings. This allows us to do comparisons for object detection 
    # between the left and right side of the robot rather than have to compare the sensors on each side individually.

    # Find these things for the right side. DS_RIGHT_00, DS_RIGHT_45 + 1 give a range of [0,2) so i can be 0 or 1. 
    # This gives us the values for sensors ps0 and ps1. The far right sensor, ps2, is excluded.
    
    for i in range(DS_RIGHT_00, DS_RIGHT_45 + 1):  # Doesn't take into account the sensor on the very right side.
        if DS_value[i] > max_ds_value:             # If sensor value is greater than the current maximum for the right side..
            max_ds_value = DS_value[i]             # max_ds_value to the new maximum sensor reading value for the right side
        SensorFusion[RIGHT] += DS_value[i]         # Put the sum of the right side sensor values at position 1 in the fusion list.
        

    # Find these things for the right side. DS_LEFT_00, DS_LEFT_45 + 1 give a range of [0,2) so i can be 0 or 1. 
    # This gives us the values for sensors ps0 and ps1. The far right sensor, ps2, is excluded.   

    for i in range(DS_LEFT_45, DS_LEFT_00 + 1):    # Doesn't take into account the sensor on the very left side.
        if DS_value[i] > max_ds_value:
            max_ds_value = DS_value[i]
        SensorFusion[LEFT] += DS_value[i]

    # If an object is detected, turn on active mode
    
    if max_ds_value > DISTANCE_THRESHOLD:
        ao_active = True

    # Determine the side the object is detected on so we can set it. We side we are switching from must be NO_SIDE
  
    if ao_active and (ao_side == NO_SIDE or ao_side == BOTH):
        if SensorFusion[RIGHT] >  min_act_right +50 and SensorFusion[LEFT] >  min_act_left + 50 : #500
            if DS_value[DS_RIGHT_00] < DISTANCE_THRESHOLD and DS_value[DS_LEFT_00] < DISTANCE_THRESHOLD:
                ao_side = NO_SIDE                                     
            else:
                ao_side = BOTH
                             
       
        # Here we limit the scope to the case where one or both of the sensor fusions are detecting a value below their baselinen value  ao_active = True and ao_side = NO_SIDE case.
        
        elif ao_side != BOTH:   
            
            ao_side = RIGHT if SensorFusion[RIGHT] > SensorFusion[LEFT] else LEFT
            
  
 # When we first enter method, set these speeds (note these are not final speeds. See main routine for final speeds)...
    
    # If we are boxed in by objects, turn around
    
    if ao_side == BOTH:                             
        ao_speed[LEFT] = ao_FORWARD_SPEED        
        ao_speed[RIGHT] =-ao_FORWARD_SPEED 
              
    
    # Otherwise, go forward   
      
    elif ao_side == NO_SIDE: 
        ao_speed[LEFT] = randN1*ao_FORWARD_SPEED     #+ oscillator, uncomment this part for rooting behavior
        ao_speed[RIGHT] = randN2*ao_FORWARD_SPEED    #- oscillator, uncomment this part for rooting behavior
        
    
    # If an object has been detected, use the fused sensor readings and reactive proportional control to determine how to turn
    
    if ao_active:
        SensorSum = 0
        
        
        # If an object is detected on the left side, determine the proportional amount to change the motor
        # speeds to turn right.
                  
        if ao_side == LEFT:
        
            SensorSum -= int(K_DS_45 * DS_value[DS_LEFT_45])
            SensorSum -= int(K_DS_00 * DS_value[DS_LEFT_00])
            
 
       # Otherwise, if an object is detected on the left side, determine the proportional amount to change 
       # the motor speeds to turn left.
       
        elif ao_side == RIGHT:
        
            SensorSum += int(K_DS_45 * DS_value[DS_RIGHT_45])
            SensorSum += int(K_DS_00 * DS_value[DS_RIGHT_00])


        # Set the proportional speeds to turn to avoid object. These is not the final speeds for the motors.
        # See main routine for final speeds.
        
        ao_speed[LEFT] -= SensorSum
        ao_speed[RIGHT] += SensorSum
        
        
        # Reset for new object detection
        ao_reset = True

# End of definition for obstacle avoidance method


# Initialization
robot = Robot()
time_step = int(robot.getBasicTimeStep())

DS_devices = ["ps0","ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]   # Put sensor identifiers in a list


# Initialization - Store sensor objects in a list and initialize each sensor.

for i in range(NUM_SENSORS):
    ds[i] = robot.getDevice(DS_devices[i])
    ds[i].enable(time_step)
    
  
# Initialization - Create motor objects
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

# Initialization - Put motors into continuous velocity mode by setting the position set point to infinity.
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Initialization - Start with motors off.
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)


# Start the main while loop.

while robot.step(time_step) != -1:
   
    # Read the sensors and store the readings in the DS_value list
    for i in range(NUM_SENSORS):
        DS_value[i] = ds[i].getValue()
        
    # Sensor Calibration: Get minimum fused "no object detected" value for both sides.
    if min_act_flag == 0:
        min_act_left = DS_value[6] + DS_value[7]
        min_act_right = DS_value[0] + DS_value[1]
        min_act_flag = 1
        
    # Call the Avoid Objects Module to detect and avoid objects and drive the leader robot.
    AvoidObjects()

    # Setting the final speeds - Limit maximum speeds
    
    if 0.00628 * ao_speed[LEFT] > 6.28:        # Limit maximum speed in positive direction
        ao_speed[LEFT] = 6.28
    if 0.00628 * ao_speed[LEFT] < -6.28:       # Limit maximum speed in negative direction.
        ao_speed[LEFT] = -6.28
        
    if 0.00628 * ao_speed[RIGHT] > 6.28:
        ao_speed[RIGHT] = 6.28
    if 0.00628 * ao_speed[RIGHT] < -6.28:
        ao_speed[RIGHT] = -6.28
        
    # Setting the final speeds - Actually set the final speeds here
       
    left_motor.setVelocity(0.00628 * ao_speed[LEFT])
    right_motor.setVelocity(0.00628 * ao_speed[RIGHT])
    
   
