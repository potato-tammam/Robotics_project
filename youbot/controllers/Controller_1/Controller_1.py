from collections import deque
from controller import Robot
import math
import time

# Pick_Up_from_Box_Matrix1 = [-0.08, -0.7, -0.32, -0.88, 0]
# Pick_Up_from_Box_Matrix2 = [-0.08, -0.85, -0.32, -0.88, 0]
# Pick_Up_from_Box_Matrix3 = [0.0, -1.0, -0.32, -0.88, 0]
# Pick_Up_from_Box_Matrix4 = [0.0, -1.5, -0.32, -0.88, 0]
Pick_Up_from_Box_Matrix = [
    [-0.0, -0.72, -0.32, -0.88, 0],
    [-0.0, -0.85, -0.35, -0.88, 0],
    [-0.0, -0.85, -0.32, -0.88, 0],
    [-0.0, -0.85, -0.39, -0.83, 0]
]
# PUT_ON_WALL_MATRIX1 = [-0.00, -0.73, -0.32, -0.88, 0]
# PUT_ON_WALL_MATRIX2 = [-0.00, -0.63, -0.32, -0.88, 0]
# PUT_ON_WALL_MATRIX3 = [-0.00, -0.85, -0.32, -0.88, 0]
# PUT_ON_WALL_MATRIX4 = [-0.00, -0.7, -0.32, -0.88, 0]

PUT_ON_WALL_MATRIX = [
    [-0.00, -0.77, -0.34, -0.88, 0],
    [-0.00, -0.67, -0.40, -0.88, 0],
    [-0.00, -0.79, -0.33, -0.88, 0],
    [-0.00, -0.73, -0.37, -0.88, 0]
]
Fold_Matrix = [-2.9, 1.5, -2.6, 1.7, 0]

COLOR_IN_HAND = None

TIME_STEP = 20
YOUBOT_MAX_VELOCITY = 10.0
LINE_DESIRED_ERROR = 0
LINE_DETECTION_THRESHOLD = 1000
YOUBOT_WHEEL_RADIUS = 0.05  # meters
YOUBOT_WHEEL_BASE = 0.5

# Robot Controller Class
class RobotController(Robot):
    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())

        self.integral = 0
        self.last_error = 0
        self.Kp = 0.01
        self.Ki = 0
        self.Kd = 0.01

        # Motors (YouBot has four wheel motors)
        self.wheels = [
            self.getDevice("wheel1"),  # Front-right
            self.getDevice("wheel2"),  # Front-left
            self.getDevice("wheel3"),  # Rear-right
            self.getDevice("wheel4"),  # Rear-left
        ]
        for wheel in self.wheels:
            wheel.setPosition(float("inf"))  # Set to velocity control mode
            wheel.setVelocity(0)

        """
        Initialize the arms and claws with explicit joint values for positions.
        """
        self.armMotors1 = []  # motors for arm 1
        self.armMotors2 = []  # motors for arm 2

        self.armMotors1.append(self.getDevice("arm1"))
        self.armMotors1.append(self.getDevice("arm2"))
        self.armMotors1.append(self.getDevice("arm3"))
        self.armMotors1.append(self.getDevice("arm4"))
        self.armMotors1.append(self.getDevice("arm5"))

        self.armMotors2.append(self.getDevice("front arm1"))
        self.armMotors2.append(self.getDevice("front arm2"))
        self.armMotors2.append(self.getDevice("front arm3"))
        self.armMotors2.append(self.getDevice("front arm4"))
        self.armMotors2.append(self.getDevice("front arm5"))

        # Set the maximum motor velocity.
        for arm in self.armMotors1:
            arm.setVelocity(1.5)

        for arm in self.armMotors2:
            arm.setVelocity(1.5)

        # Initialize arm position sensors.
        # These sensors can be used to get the current
        # joint position and monitor the joint movements.
        self.arm1PositionSensors = []
        self.arm2PositionSensors = []

        self.arm1PositionSensors.append(self.getDevice("arm1sensor"))
        self.arm1PositionSensors.append(self.getDevice("arm2sensor"))
        self.arm1PositionSensors.append(self.getDevice("arm3sensor"))
        self.arm1PositionSensors.append(self.getDevice("arm4sensor"))
        self.arm1PositionSensors.append(self.getDevice("arm5sensor"))

        self.arm2PositionSensors.append(self.getDevice("front arm1sensor"))
        self.arm2PositionSensors.append(self.getDevice("front arm2sensor"))
        self.arm2PositionSensors.append(self.getDevice("front arm3sensor"))
        self.arm2PositionSensors.append(self.getDevice("front arm4sensor"))
        self.arm2PositionSensors.append(self.getDevice("front arm5sensor"))

        for sensor in self.arm1PositionSensors:
            sensor.enable(TIME_STEP)

        for sensor in self.arm2PositionSensors:
            sensor.enable(TIME_STEP)

        # Initialize gripper motors.
        self.Arm1finger1 = self.getDevice("finger::left")
        self.Arm1finger2 = self.getDevice("finger::right")

        self.Arm2finger1 = self.getDevice("front finger::left")
        self.Arm2finger2 = self.getDevice("front finger::right")

        # Set the maximum motor velocity.

        self.Arm1finger1.setVelocity(1.5)
        self.Arm1finger2.setVelocity(1.5)  # 0.03

        self.Arm2finger1.setVelocity(1.5)
        self.Arm2finger2.setVelocity(1.5)

        # Read the miminum and maximum position of the gripper motors.
        self.fingerMinPosition = self.Arm1finger1.getMinPosition()
        self.fingerMaxPosition = self.Arm1finger1.getMaxPosition()

        # Line sensors
        self.line_sensor = self.getDevice("line sensor")
        self.line_sensor.enable(self.timestep)

        self.front_right_motor_sensor = self.wheels[0].getPositionSensor()
        self.front_right_motor_sensor.enable(self.timestep)
        
        self.back_right_motor_sensor = self.wheels[2].getPositionSensor()
        self.back_right_motor_sensor.enable(self.timestep)
        # Camera
        self.camera = self.getDevice("cam")
        self.camera.enable(self.timestep)

        # State management
        self.colors_detected = deque(maxlen=4)
        self.state = "COLLECTING_INITIAL_COLORS"

        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.timestep)

        self.direction = "FORTH"
        self.detected_colors_on_line = set()

    def detect_color(self, cameraArray):
        """
        Detect the color based on the averaged RGB values in the center of the camera's image.
        """
        if cameraArray is None:
            return "Unknown"

        # Define a region of interest (e.g., 5x5 square in the center)
        roi_size = 5
        center_x, center_y = len(cameraArray) // 2, len(cameraArray[0]) // 2

        # Average the RGB values in the ROI
        red, green, blue = 0, 0, 0
        count = 0
        for i in range(center_x - roi_size, center_x + roi_size):
            for j in range(center_y - roi_size, center_y + roi_size):
                pixel = cameraArray[i][j]
                red += pixel[0]
                green += pixel[1]
                blue += pixel[2]
                count += 1

        # Calculate average
        red //= count
        green //= count
        blue //= count

        # Use thresholds to detect colors
        if red > 80 and green < 50 and blue < 50:
            return "red"
        elif green > 85 and red < 50 and blue < 50:
            return "green"
        elif blue > 120 and red < 100 and green < 100:
            return "blue"
        elif red > 80 and green > 90 and blue < 50:
            return "yellow"
        else:
            return "Unknown"

    def collect_colors(self):
        """
        Move forward and collect distinct colors into a queue of size 4.
        """
        cameraArray = self.camera.getImageArray()
        if cameraArray:
            detected_color = self.detect_color(cameraArray)
            if (
                detected_color != "Unknown"
                and detected_color not in self.colors_detected
            ):
                self.colors_detected.append(detected_color)
                print(f"Collected color: {detected_color}")

    def set_velocities(self, wheel1v=None, wheel2v=None, wheel3v=None, wheel4v=None):
        if wheel1v:
            self.wheels[0].setVelocity(wheel1v)
        if wheel2v:
            self.wheels[1].setVelocity(wheel2v)
        if wheel3v:
            self.wheels[2].setVelocity(wheel3v)
        if wheel4v:
            self.wheels[3].setVelocity(wheel4v)

    def move_forward(self, velocity=YOUBOT_MAX_VELOCITY, distance=None):
        if distance:
            circumference = 2 * math.pi * YOUBOT_WHEEL_RADIUS
            number_of_rotations = distance / circumference
            angle = number_of_rotations * 2 * math.pi

            initial_sensor_value = self.back_right_motor_sensor.getValue()

            self.set_velocities(velocity, velocity, velocity, velocity)

            while (
                self.back_right_motor_sensor.getValue() - initial_sensor_value < angle
            ):
                self.step(self.timestep)

            self.stop()

        else:
            self.set_velocities(velocity, velocity, velocity, velocity)

    def move_backward(self, velocity=YOUBOT_MAX_VELOCITY , distance = None):
        if distance:
            circumference = 2 * math.pi * YOUBOT_WHEEL_RADIUS
            number_of_rotations = distance / circumference
            angle = number_of_rotations * 2 * math.pi

            initial_sensor_value = self.front_right_motor_sensor.getValue()

            self.set_velocities(-velocity, -velocity, -velocity, -velocity)

            while (
                self.front_right_motor_sensor.getValue() - initial_sensor_value < angle
            ):
                self.step(self.timestep)

            self.stop()

        else:
            self.set_velocities(-velocity, -velocity, -velocity, -velocity)

    def stop(self):
        """
        Stop all wheel motors.
        """
        for wheel in self.wheels:
            wheel.setVelocity(0)

    def move_arm(self, arm, matrix):
        if arm == 1:
            target_positions = matrix
            for i, motor in enumerate(self.armMotors1):
                motor.setPosition(target_positions[i])

        elif arm == 2:
            target_positions = matrix
            for i, motor in enumerate(self.armMotors2):
                motor.setPosition(target_positions[i])

        else:
            print("Please set the arm")

    def close_grippers(self, arm):
        if arm == 1:
            self.Arm1finger1.setPosition(0.001)
            self.Arm1finger2.setPosition(0.001)
            self.step(50 * TIME_STEP)
        elif arm == 2:
            self.Arm2finger1.setPosition(0.001)
            self.Arm2finger2.setPosition(0.001)
            self.step(50 * TIME_STEP)
            
        else:
            print("please set the arm ")

    def hand_up(self, arm):
        if arm == 1:
            for i, motor in enumerate(self.armMotors1):
                motor.setPosition(0)
        elif arm == 2:
            for i, motor in enumerate(self.armMotors2):
                motor.setPosition(0)
        else:
            print("please set the arm ")

    def fold_arms(self, arm):
        target_postion = Fold_Matrix
        if arm == 1:
            for i, motor in enumerate(self.armMotors1):
                motor.setPosition(target_postion[i])
        elif arm == 2:
            for i, motor in enumerate(self.armMotors2):
                motor.setPosition(target_postion[i])
        else:
            print("error please set the arm")

    def open_gribbers(self, arm):
        if arm == 1:
            self.Arm1finger1.setPosition(self.fingerMaxPosition)
            self.Arm1finger2.setPosition(self.fingerMaxPosition)
        elif arm == 2:
            self.Arm2finger1.setPosition(self.fingerMaxPosition)
            self.Arm2finger2.setPosition(self.fingerMaxPosition)
        else:
            print("please set the arm ")

        # Open gripper.

    def grab_And_retract(self, arm , matrix):
        """
        This mthod will take small boxes of the big boxes and fold the arm

        Args:
            arm (1,2): which arm you trying to grab with
        """

        self.move_arm(arm, matrix)
        print("reaching")
        self.step(10 * TIME_STEP)
        self.open_gribbers(arm)
        print("oppening the gribbers")
        self.step(25 * TIME_STEP)
        print("closing the gribbers")
        self.close_grippers(arm)
        self.step(3 * TIME_STEP)
        print("floding")
        self.hand_up(arm)
        self.step(5 * TIME_STEP)

    def loop_Function(self , i):
        if i == 0:
            self.grab_And_retract(1 , Pick_Up_from_Box_Matrix[i])
            print(Pick_Up_from_Box_Matrix[i])
            self.step(75 * TIME_STEP)
        else :
            self.grab_And_retract(1 , Pick_Up_from_Box_Matrix[i])
            print(Pick_Up_from_Box_Matrix[i])
            self.step(75 * TIME_STEP)
            self.move_forward(distance= 0.025)
            print("Moved a bit ")
            self.step(15 * TIME_STEP)
            
        self.rotate_in_place(-180)
        self.step(20 * TIME_STEP)
    
        self.grab_And_retract(2 , Pick_Up_from_Box_Matrix[i+1])
    
        print(Pick_Up_from_Box_Matrix[i+1])
        self.step(75 * TIME_STEP)
        print("picked up cubes from the Box")
        if i == 0:
            
            self.move_forward(distance= 0.75)
        else :
            self.move_forward(distance= 0.72)
            
        self.step(75 * TIME_STEP)
        self.Put_Box_On_wall(1 ,PUT_ON_WALL_MATRIX[i] )
        print(PUT_ON_WALL_MATRIX[i])
        
        self.step(75 * TIME_STEP)
        self.rotate_in_place(179)
        self.step(25 * TIME_STEP)
        self.move_forward(distance= 0.025 )
        print("Moved a bit ")
        self.step(10 * TIME_STEP)
        self.Put_Box_On_wall(2 , PUT_ON_WALL_MATRIX[i+1])
        print(PUT_ON_WALL_MATRIX[i+1])
        
        self.step(75 * TIME_STEP)
        print(" put cubes on wall ")
        if i == 0:
            self.move_forward(distance= 0.75)
        else:
            self.move_forward(distance= 0.2)
            
        self.step(20 * TIME_STEP)

    def Put_Box_On_wall(self, arm, matrix):
        """
        This mthod will put small boxes on the wall and retract the arm

        Args:
            arm (1,2): which arm you trying to grab with
        """
        self.move_arm(arm, matrix)
        print("reaching")
        self.step(100 * TIME_STEP)
        self.open_gribbers(arm)
        print("oppening the gribbers")
        self.step(25 * TIME_STEP)
        print("retacting")
        self.hand_up(arm)
        self.step(10 * TIME_STEP)
        print("closing the gribbers")
        self.close_grippers(arm)

    def detected_a_line(self):
        print(self.line_sensor.getValue())
        return self.line_sensor.getValue() >= LINE_DETECTION_THRESHOLD

    def rotate_in_place(self, degrees, velocity=YOUBOT_MAX_VELOCITY):
        if degrees == 0:
            self.stop()
            return

        target_rotation = math.radians(degrees)

        rotation_angle = 0

        if degrees > 0:
            self.set_velocities(
                wheel1v=velocity, wheel2v=-velocity, wheel3v=velocity, wheel4v=-velocity
            )
        else:
            self.set_velocities(
                wheel1v=-velocity, wheel2v=velocity, wheel3v=-velocity, wheel4v=velocity
            )

        while self.step(self.timestep) != -1:
            angular_velocity = self.gyro.getValues()[2]
            rotation_angle += angular_velocity * (self.timestep / 1000.0)
            if abs(rotation_angle) >= abs(target_rotation):
                break

        self.stop()

    def switch_direction(self):
        self.direction = "BACK" if self.direction == "FORTH" else "FORTH"
        print(f"Switching direction to {self.direction}")

        self.rotate_in_place(180 if self.direction == "BACK" else -180)
        self.stop()
        self.detected_colors_on_line.clear()
        print("Rotation complete, direction switched.")

    def run(self):
        while self.step(self.timestep) != -1:
            if self.state == "COLLECTING_INITIAL_COLORS":
                self.move_forward(velocity=YOUBOT_MAX_VELOCITY)
                self.collect_colors()
                if len(self.colors_detected) == 4:
                    print(f"Collected all four colors: {list(self.colors_detected)}")
                    self.state = "SKIPPING_THE_LAST_COLOR_SQUARE"

            elif self.state == "SKIPPING_THE_LAST_COLOR_SQUARE":
                self.move_forward(distance=0.12, velocity=YOUBOT_MAX_VELOCITY)
                print("Skipped the last color square")
                self.state = "DETECTING_LINE"

            elif self.state == "DETECTING_LINE":
                self.move_forward(velocity=YOUBOT_MAX_VELOCITY*0.5)
                if self.detected_a_line():
                    self.stop()
                    print("detected a line and stopped")
                    self.state = "ROTATING_90_DEGREES_CLOCKWISE"

            elif self.state == "ROTATING_90_DEGREES_CLOCKWISE":
                self.rotate_in_place(-90)
                print("turned 90 degrees clockwise")
                self.state = "DETECTING_COLOR"
                
            elif self.state == "DETECTING_COLOR":
                self.move_forward()
                detected_color = self.detect_color(self.camera.getImageArray())
                if detected_color == self.colors_detected[0]:
                    COLOR_IN_HAND = detected_color
                    self.stop()
                    print("Detected the desired color and stopped")
                    self.state = "ROTATING_90_DEGREES_COUNTER_CLOCKWISE"
                    
            elif self.state == 'ROTATING_90_DEGREES_COUNTER_CLOCKWISE':
                self.rotate_in_place(90)
                print("turned 90 degrees counter clockwise")
                self.state = 'PICKING_CUBE'
            
            elif self.state == 'PICKING_CUBE':
                self.move_forward(distance=0.03)
                self.loop_Function(i=0)
                self.stop()
                
            else:
                print("quit")
                break


if __name__ == "__main__":
    robot = RobotController()
    robot.run()
    # robot.move_forward()
    # detected_color = robot.detect_color(robot.camera.getImageArray())
    # if detected_color == "yellow" :
    #     robot.stop()
    #     robot.move_forward(distance=COLOR_SQUARE_SIDE_LENGTH / 2.3)
    #     robot.stop()
    # robot.rotate_in_place(-90)
    # robot.stop()
    # robot.move_forward()
    