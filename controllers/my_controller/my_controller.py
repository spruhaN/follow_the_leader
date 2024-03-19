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
freq = 0
f = open("pixels.txt", "w")
num = 0

while robot.step(timestep) != -1:
    image = camera.getImageArray()

    camera.saveImage('testImage.png', 100)
    print(f"width: {camera.getWidth()}")
    print(f"height: {camera.getHeight()}")

    if image:
        red_matrix = []
        # check each pixel for largest red component
        for x in range(0, camera.getWidth()):
            red_row = []  # each row
            for y in range(0, camera.getHeight()):
                # get components
                red_component = int(image[x][y][0])
                green_component = int(image[x][y][1])
                blue_component = int(image[x][y][2])

                # red_row.append(f"rgb({red_component},{green_component},{blue_component})")

                # if red is largest then leader prev if red > green and red > blue
                # (red_component + 10) > (green_component + blue_component) and red_component > (60)
                if red_component > 50 and green_component < 40 and blue_component < 35:
                    red_row.append(f"o")
                else:
                    red_row.append(f"-")  # background

            red_matrix.append(red_row)  # Add the row to the matrix

        # Determine the maximum width of the numbers in the matrix for formatting
        max_width = max(len(str(item)) for row in red_matrix for item in row)
        num += 1
        # if num == 10:
        for row in red_matrix:
            print(' '.join(f"{item:>{max_width}}" for item in row))
            # print("\n")
        # example of pixel out
        # if num == 15:
        #     f.write ("==========")
        #     for row in red_matrix:
        #         f.write(' '.join(f"{item:>{max_width}}" for item in row))
    else:
        print("NO IMAGE")





    # this doesnt work with E-Puck robot type 
    # objects = camera.getRecognitionObjects()
    
    # for obj in objects:
    #     # Print object ID, position, size, and orientation
    #     print("Object ID:", obj.get_id())
    #     print("Position:", obj.get_position())
    #     print("Size:", obj.get_size())
    #     print("Orientation:", obj.get_orientation())

    left_motor.setVelocity(0.0) # set the left motor (radians/second)
    right_motor.setVelocity(0.0) # set the right motor (radians/second) 

    # print(left_ground_sensor.getValue())
    # print(middle_ground_sensor.getValue())
    # print(right_ground_sensor.getValue())
    # print(right_distance_sensor.getValue())
    
    new_encoder_values = [encoder.getValue() for encoder in encoders]
    # print(new_encoder_values)
    #print('-------------------------')
        
    # call robot.getTime() to get the current simulation time in seconds



