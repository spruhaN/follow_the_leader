"""CPE416 Sample Controller"""

from controller import Robot, Motor, Camera, CameraRecognitionObject

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# enable the drive motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# enable ground color sensors
left_ground_sensor = robot.getDevice('gs0')
left_ground_sensor.enable(timestep)

middle_ground_sensor = robot.getDevice('gs1')
middle_ground_sensor.enable(timestep)

right_ground_sensor = robot.getDevice('gs2')
right_ground_sensor.enable(timestep)

right_distance_sensor = robot.getDevice('ps2') #IR sensor pointing to the right
right_distance_sensor.enable(timestep)

# initialize encoders
encoders = []
encoderNames = ['left wheel sensor', 'right wheel sensor']
for i in range(2):
    encoders.append(robot.getDevice(encoderNames[i]))
    encoders[i].enable(timestep)


# Main loop:
# - perform simulation steps until Webots stops the controller

camera = Camera('camera')
#camera = robot.getDevice("camera")
camera.enable(100)

x = 0
while robot.step(timestep) != -1:
    image = camera.getImageArray()
    
    if x == 10:
        #print(image)
        camera.saveImage('testImage.png', 100)
        x = 0

        green_total = 0
        red_total = 0
        blue_total = 0
        if image:
            # display the components of each pixel
            for x in range(0,camera.getWidth()):
                for y in range(0,camera.getHeight()):
                    #print(image)
                    #print(x, " ", y)
                    red   = image[x][y][0]
                    green = image[x][y][1]
                    blue  = image[x][y][2]
                    gray  = (red + green + blue) / 3
                    
                    red_total += red
                    green_total += green
                    blue_total += blue
    
                    #print('r='+str(red)+' g='+str(green)+' b='+str(blue))
            print('-------------------------')
            print("greenTotal: ", green_total)
            print("blueTotal: ", blue_total)
            print("redTotal: ", red_total)
            print('-------------------------')
    x += 1
    # this doesnt work with E-Puck robot type 
    # objects = camera.getRecognitionObjects()
    
    # for obj in objects:
    #     # Print object ID, position, size, and orientation
    #     print("Object ID:", obj.get_id())
    #     print("Position:", obj.get_position())
    #     print("Size:", obj.get_size())
    #     print("Orientation:", obj.get_orientation())

    left_motor.setVelocity(1.0) # set the left motor (radians/second)
    right_motor.setVelocity(1.0) # set the right motor (radians/second) 

    # print(left_ground_sensor.getValue())
    # print(middle_ground_sensor.getValue())
    # print(right_ground_sensor.getValue())
    # print(right_distance_sensor.getValue())
    
    new_encoder_values = [encoder.getValue() for encoder in encoders]
    # print(new_encoder_values)
    #print('-------------------------')
        
    # call robot.getTime() to get the current simulation time in seconds



