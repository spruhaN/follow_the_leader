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
left_motor.setVelocity(1.0)
right_motor.setVelocity(1.0)

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


def motor_inputs(target_column, total_columns=20, max_speed=1):
    middle_column = (total_columns - 1) / 2.0
    error = target_column - middle_column
    
    base_speed = max_speed / 2.0
    
    Kp = .05 #(max_speed / 2.0) / middle_column
    
    control_signal = Kp * error
    
    left_motor_speed = base_speed - control_signal
    right_motor_speed = base_speed + control_signal
    
    left_motor_speed = max(0, min(max_speed, left_motor_speed))
    right_motor_speed = max(0, min(max_speed, right_motor_speed))
    
    return left_motor_speed, right_motor_speed



# Main loop:
# - perform simulation steps until Webots stops the controller

camera = Camera('camera')
#camera = robot.getDevice("camera")
camera.enable(100)

x = 0
freq = 0
f = open("pixels.txt", "w")
num_red = 0

while robot.step(timestep) != -1:

    image = camera.getImageArray()

    camera.saveImage('testImage.png', 100)
    print(f"width: {camera.getWidth()}")
    print(f"height: {camera.getHeight()}")

    if image:
        print("===============================")
        red_matrix = []
        # check each pixel for largest red component
        height = camera.getHeight()
        width = camera.getWidth()

        column_sums = [0] * height

        # Ensuring we access valid indices
        for w in range(width):  # Iterate over rows
            sum = 0
            for h in range(height):  # Iterate over columns
                # Accessing the pixel at (h, w) safely
                pixel = image[w][h]
                red_component = int(pixel[0])
                green_component = int(pixel[1])
                blue_component = int(pixel[2])

                # print(f"{red_component},{green_component},{blue_component} ", end = "")
                # Determine if the pixel is predominantly red
                if red_component > 50 and green_component < 40 and blue_component < 35:
                    print(f"{red_component} ", end="")
                    column_sums[h] += red_component
                    num_red += 1
                else:
                    print("-  ", end="")
            print(f"({sum})")
        
        target = column_sums.index(max(column_sums))
        if num_red < 3 or max(column_sums) == 0:
            target = -1

        FACTOR = 1.5
        # left,right = motor_inputs(target) PID CONTROLLER
        if target == -1:
            left = 0 * FACTOR
            right = 1 * FACTOR
        elif target < 15 and target > 6:
            left = 1 * FACTOR
            right = 1 * FACTOR
        elif target >= 15:
            left = 1.3 * FACTOR
            right = 1 * FACTOR
        else:
            left = 1 * FACTOR
            right = 1.3 * FACTOR

        left_motor.setVelocity(left)
        right_motor.setVelocity(right)
        print(f"target: {target} and {left} {right}")
    else:
        print("NO IMAGE")
