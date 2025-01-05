from collections import deque
from controller import Robot
import math
import time

# Pick_Up_from_Box_Matrix1 = [-0.08, -0.7, -0.32, -0.88, 0]
# Pick_Up_from_Box_Matrix2 = [-0.08, -0.85, -0.32, -0.88, 0]
# Pick_Up_from_Box_Matrix3 = [0.0, -1.0, -0.32, -0.88, 0]
# Pick_Up_from_Box_Matrix4 = [0.0, -1.5, -0.32, -0.88, 0]
Pick_Up_from_Box_Matrix = [
    [-0.02, -0.72, -0.32, -0.88, 0],
    [-0.02, -0.85, -0.35, -0.88, 0],
    [-0.02, -0.85, -0.32, -0.88, 0],
    [-0.02, -0.80, -0.39, -0.80, 0]
]
# PUT_ON_WALL_MATRIX1 = [-0.00, -0.73, -0.32, -0.88, 0]
# PUT_ON_WALL_MATRIX2 = [-0.00, -0.63, -0.32, -0.88, 0]
# PUT_ON_WALL_MATRIX3 = [-0.00, -0.85, -0.32, -0.88, 0]
# PUT_ON_WALL_MATRIX4 = [-0.00, -0.7, -0.32, -0.88, 0]

PUT_ON_WALL_MATRIX = [
    [-0.00, -0.77, -0.34, -0.88, 0],
    [-0.01, -0.67, -0.40, -0.88, 0],
    [-0.02, -0.79, -0.33, -0.88, 0],
    [-0.02, -0.74, -0.38, -0.88, 0]
]
Fold_Matrix = [-2.9, 1.5, -2.6, 1.7, 0]

COLOR_IN_HAND = None

TIME_STEP = 20
YOUBOT_MAX_VELOCITY = 10.0
LINE_DESIRED_ERROR = 0
LINE_DETECTION_THRESHOLD = 1000
YOUBOT_WHEEL_RADIUS = 0.05  # meters
COLOR_SQUARE_SIDE_LENGTH = 0.1
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

        self.line_sensor_left_outer = self.getDevice("ls1")
        self.line_sensor_left_inner = self.getDevice("ls2")
        self.line_sensor_right_outer = self.getDevice("ls3")
        self.line_sensor_right_inner = self.getDevice("ls4")
        
        self.line_sensor_left_outer.enable(self.timestep)
        self.line_sensor_left_inner.enable(self.timestep)
        self.line_sensor_right_inner.enable(self.timestep)
        self.line_sensor_right_outer.enable(self.timestep)
        
        self.line_sensor_left_outer_back = self.getDevice("ls1b")
        self.line_sensor_left_inner_back = self.getDevice("ls2b")
        self.line_sensor_right_outer_back = self.getDevice("ls3b")
        self.line_sensor_right_inner_back = self.getDevice("ls4b")
        
        self.line_sensor_left_outer_back.enable(self.timestep)
        self.line_sensor_left_inner_back.enable(self.timestep)
        self.line_sensor_right_inner_back.enable(self.timestep)
        self.line_sensor_right_outer_back.enable(self.timestep)
        
        
        
        
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
            # self.center_on_line_with_pid()
            self.set_velocities(velocity, velocity, velocity, velocity)
            while (
                self.back_right_motor_sensor.getValue() - initial_sensor_value < angle
            ):
                #  if (self.back_right_motor_sensor.getValue() - initial_sensor_value  == angle/2):
                
                self.step(self.timestep)

            self.stop()

        else:
            # self.center_on_line_with_pid()
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
        self.step(2 * TIME_STEP)
        print("floding")
        self.hand_up(arm)
        self.step(2 * TIME_STEP)
        
    def center_and_move_forward(self, velocity=YOUBOT_MAX_VELOCITY, distance=None):
        """
        Centers the robot on the line while moving forward.
        Combines line-centering logic with forward motion using PID control.
        The velocity never exceeds the specified maximum.
        Optionally moves forward for a specified distance.
        """
        # PID parameters for line centering
        Kp = 0.05  # Proportional gain
        Ki = 0.01  # Integral gain
        Kd = 0.02  # Derivative gain

        # Variables for PID control
        integral = 0
        last_error = 0

        # Threshold for sensor readings to detect the line
        threshold = 200

        # Tolerance for stopping centering adjustments
        tolerance = 5  # Minimum error to consider the line centered

        # Distance calculation setup (if distance is specified)
        if distance:
            circumference = 2 * math.pi * YOUBOT_WHEEL_RADIUS
            number_of_rotations = distance / circumference
            angle = number_of_rotations * 2 * math.pi

            initial_sensor_value = self.back_right_motor_sensor.getValue()

        while self.step(self.timestep) != -1:
            # Read sensor values
            left_outer = self.line_sensor_left_outer.getValue()
            left_inner = self.line_sensor_left_inner.getValue()
            right_inner = self.line_sensor_right_inner.getValue()
            right_outer = self.line_sensor_right_outer.getValue()

            # Normalize sensor values based on the threshold
            left_error = ((left_outer < threshold) + (left_inner < threshold)) / 2
            right_error = ((right_inner < threshold) + (right_outer < threshold)) / 2
            total_error = left_error - right_error

            # Accumulate integral of error
            integral += total_error

            # Calculate derivative of error
            derivative = total_error - last_error

            # Calculate PID adjustment
            adjustment = Kp * total_error + Ki * integral + Kd * derivative

            # Apply adjusted velocities to maintain line centering
            left_wheel_velocity = max(-velocity, min(velocity - adjustment, velocity))
            right_wheel_velocity = max(-velocity, min(velocity + adjustment, velocity))

            self.set_velocities(
                wheel1v=right_wheel_velocity,  # Front-right
                wheel2v=left_wheel_velocity,   # Front-left
                wheel3v=right_wheel_velocity,  # Rear-right
                wheel4v=left_wheel_velocity    # Rear-left
            )

            # Check if the robot is centered on the line
            if abs(total_error) < tolerance:
                print("Robot is centered on the line while moving forward.")

            # Update last error for derivative calculation
            last_error = total_error

            # If distance is specified, stop after moving the required distance
            if distance:
                current_angle = self.back_right_motor_sensor.getValue() - initial_sensor_value
                if current_angle >= angle:
                    print("Reached the specified distance.")
                    self.stop()
                    break
   
    def loop_Function(self , i):
        if i == 0:
            self.grab_And_retract(1 , Pick_Up_from_Box_Matrix[i])
            print(Pick_Up_from_Box_Matrix[i])
            self.step(55 * TIME_STEP)
        else :
            self.grab_And_retract(1 , Pick_Up_from_Box_Matrix[i])
            print(Pick_Up_from_Box_Matrix[i])
            self.step(55 * TIME_STEP)
            self.move_forward(distance= COLOR_SQUARE_SIDE_LENGTH /4 )
            print("Moved a bit ")
            self.center_on_line_laterally_with_pid_Front()
            self.step(10 * TIME_STEP)
            
        self.rotate_in_place(180)
        self.center_on_line_laterally_with_pid_back()
        # self.center_on_line_laterally_with_pid_Front()
        
        self.step(20 * TIME_STEP)
    
        self.grab_And_retract(2 , Pick_Up_from_Box_Matrix[i+1])
        self.center_on_line_laterally_with_pid_Front()
        print(Pick_Up_from_Box_Matrix[i+1])
        self.step(60 * TIME_STEP)
        print("picked up cubes from the Box")
        if i == 0:
            self.center_on_line_laterally_with_pid_Front()
            self.center_and_move_forward(distance= COLOR_SQUARE_SIDE_LENGTH * 7.5)
            self.center_on_line_laterally_with_pid_Front()
            
            
        else :
            # self.center_on_line_laterally_with_pid_Front()
            self.center_on_line_laterally_with_pid_Front()
            self.center_on_line_with_pid()
            self.move_forward(distance= COLOR_SQUARE_SIDE_LENGTH * 7.2)
            self.center_on_line_with_pid()
            self.center_on_line_laterally_with_pid_Front()
            self.correct_rotation_with_pid()
            
            
        self.step(60 * TIME_STEP)
        self.Put_Box_On_wall(1 ,PUT_ON_WALL_MATRIX[i] )
        print(PUT_ON_WALL_MATRIX[i])
        
        self.step(50 * TIME_STEP)
        self.rotate_in_place(180)
        self.center_on_line_laterally_with_pid_Front()
        
        self.step(25 * TIME_STEP)
        self.move_forward(distance= COLOR_SQUARE_SIDE_LENGTH/4 )
        print("Moved a bit ")
        self.center_on_line_laterally_with_pid_back()
        self.step(10 * TIME_STEP)
        self.Put_Box_On_wall(2 , PUT_ON_WALL_MATRIX[i+1])
        print(PUT_ON_WALL_MATRIX[i+1])
        
        self.step(60 * TIME_STEP)
        print(" put cubes on wall ")
        if i == 0:
            self.center_on_line_laterally_with_pid_Front()
            
            self.center_on_line_with_pid()
            self.center_and_move_forward(distance= COLOR_SQUARE_SIDE_LENGTH * 7.5)
            self.center_on_line_laterally_with_pid_Front()
            

        else:
            self.center_on_line_laterally_with_pid_Front()
            
            self.center_on_line_with_pid()
            self.move_forward(distance= COLOR_SQUARE_SIDE_LENGTH * 2)
            self.center_on_line_laterally_with_pid_Front()

            
        self.step(10 * TIME_STEP)
        
    def center_on_line_with_pid(self):
        """
        Adjust the wheel velocities using PID control to center the robot on the line
        by moving sideways without rotating. Ensures the line is centered under all sensors.
        Uses an extra center sensor for finer corrections.
        The velocity never exceeds 10 units. The line sensors read analog data, with a threshold <200.
        """

        # PID parameters (tune as needed)
        Kp = 0.05  # Proportional gain
        Ki = 0.00  # Integral gain
        Kd = 0.02  # Derivative gain

        # PID variables
        integral_front = 0
        last_error_front = 0
        integral_back = 0
        last_error_back = 0
        integral_center = 0
        last_error_center = 0

        # Threshold for detecting the line
        threshold = 200

        # Tolerance for stopping (adjust for sensitivity)
        tolerance = 5  

        # Base velocity for lateral movement
        base_velocity = 10  # Max sideways speed

        while self.step(self.timestep) != -1:
            # Read sensor values
            front_left_outer = self.line_sensor_left_outer.getValue()
            front_left_inner = self.line_sensor_left_inner.getValue()
            front_right_inner = self.line_sensor_right_inner.getValue()
            front_right_outer = self.line_sensor_right_outer.getValue()

            back_left_outer = self.line_sensor_left_outer_back.getValue()
            back_left_inner = self.line_sensor_left_inner_back.getValue()
            back_right_inner = self.line_sensor_right_inner_back.getValue()
            back_right_outer = self.line_sensor_right_outer_back.getValue()

            center_value = self.line_sensor.getValue()  # Extra center sensor

            # Normalize sensor values based on threshold
            front_left_error = ((front_left_outer < threshold) + (front_left_inner < threshold)) / 2
            front_right_error = ((front_right_inner < threshold) + (front_right_outer < threshold)) / 2
            front_total_error = front_left_error - front_right_error

            back_left_error = ((back_left_outer < threshold) + (back_left_inner < threshold)) / 2
            back_right_error = ((back_right_inner < threshold) + (back_right_outer < threshold)) / 2
            back_total_error = back_left_error - back_right_error

            center_error = -1 if center_value < threshold else 0  # -1 means the center is off the line

            # Accumulate integral of errors
            integral_front += front_total_error
            integral_back += back_total_error
            integral_center += center_error

            # Calculate derivative of errors
            derivative_front = front_total_error - last_error_front
            derivative_back = back_total_error - last_error_back
            derivative_center = center_error - last_error_center

            # Compute PID adjustments for front, back, and center
            adjustment_front = Kp * front_total_error + Ki * integral_front + Kd * derivative_front
            adjustment_back = Kp * back_total_error + Ki * integral_back + Kd * derivative_back
            adjustment_center = Kp * center_error + Ki * integral_center + Kd * derivative_center

            # Compute final adjustment
            adjustment = max(abs(adjustment_front), abs(adjustment_back)) * (1 if adjustment_front >= 0 or adjustment_back >= 0 else -1)
            
            # Center sensor correction (higher priority if active)
            if center_error != 0:
                adjustment += adjustment_center

            # Ensure velocity limits
            sideways_velocity = max(-10, min(base_velocity + adjustment, 10))

            # Set lateral movement velocities (moving sideways without rotation)
            self.set_velocities(
                wheel1v=sideways_velocity,  # Front-right moves laterally
                wheel2v=-sideways_velocity, # Front-left moves laterally
                wheel3v=sideways_velocity,  # Rear-right moves laterally
                wheel4v=-sideways_velocity  # Rear-left moves laterally
            )

            # Check if both front and back errors + center sensor are within tolerance
            if abs(front_total_error) < tolerance and abs(back_total_error) < tolerance and center_error == 0:
                print("Line is centered under the robot.")
                self.stop()  # Stop the robot
                break

            # Update last errors for next iteration
            last_error_front = front_total_error
            last_error_back = back_total_error
            last_error_center = center_error

  
    def center_on_line_laterally_with_pid_Front(self):
        """
        Adjust the wheel velocities using PID control to center the robot on the line
        by moving sideways without rotating. Stops once the line is centered under the robot.
        # PID parameters (tune these values based on your robot's behavior)
           Adjust the wheel velocities using PID control to center the robot on the line
        by moving the entire robot sideways without any rotation or angular deviation.
        Stops once the line is centered under the robot.
    """
        # PID parameters (tune these values based on your robot's behavior)
        Kp = 0.05  # Proportional gain
        Ki = 0.00  # Integral gain
        Kd = 0.02  # Derivative gain

        # Variables for PID control
        integral = 0
        last_error = 0

        # Tolerance for stopping (adjust based on sensitivity requirements)
        tolerance = 8  # Minimum error to consider the line centered

        # Base velocity for the wheels (used for lateral movement)
        base_velocity = YOUBOT_MAX_VELOCITY * 0.3

        while self.step(self.timestep) != -1:
            # Read sensor values
            left_outer = self.line_sensor_left_outer.getValue()
            left_inner = self.line_sensor_left_inner.getValue()
            right_inner = self.line_sensor_right_inner.getValue()
            right_outer = self.line_sensor_right_outer.getValue()

            # Calculate errors
            left_error = (left_outer + left_inner) / 2
            right_error = (right_inner + right_outer) / 2
            total_error = left_error - right_error

            # Accumulate integral of error
            integral += total_error

            # Calculate derivative of error
            derivative = total_error - last_error

            # Calculate PID adjustment
            adjustment = Kp * total_error + Ki * integral + Kd * derivative

            # Set lateral movement velocities
            # Positive adjustment moves the bot right; negative moves it left.
            # Left wheels move in one direction, right wheels in the opposite direction.
            sideways_velocity = base_velocity + adjustment
            self.set_velocities(
                wheel1v=sideways_velocity,  # Front-right (moves laterally)
                wheel2v=-sideways_velocity, # Front-left (moves laterally)
                wheel3v=sideways_velocity,  # Rear-right (moves laterally)
                wheel4v=-sideways_velocity  # Rear-left (moves laterally)
            )

            # Check if the line is centered (error within tolerance)
            if abs(total_error) < tolerance:
                print("Line is centered under the robot **FRONT**.")
                self.stop()  # Stop the robot
                break

            # Update last error for derivative calculation
            last_error = total_error

    def center_on_line_laterally_with_pid_back(self):
        """
        Adjust the wheel velocities using PID control to center the robot on the line
        by moving sideways without rotating. Stops once the line is centered under the robot.
        # PID parameters (tune these values based on your robot's behavior)
           Adjust the wheel velocities using PID control to center the robot on the line
        by moving the entire robot sideways without any rotation or angular deviation.
        Stops once the line is centered under the robot.
    """
        # PID parameters (tune these values based on your robot's behavior)
        Kp = 0.05  # Proportional gain
        Ki = 0.00  # Integral gain
        Kd = 0.02  # Derivative gain

        # Variables for PID control
        integral = 0
        last_error = 0

        # Tolerance for stopping (adjust based on sensitivity requirements)
        tolerance = 8 # Minimum error to consider the line centered

        # Base velocity for the wheels (used for lateral movement)
        base_velocity = YOUBOT_MAX_VELOCITY * 0.3

        while self.step(self.timestep) != -1:
            # Read sensor values
            left_outer = self.line_sensor_right_outer_back.getValue()
            left_inner = self.line_sensor_right_inner_back.getValue()
            right_inner = self.line_sensor_left_inner_back.getValue()
            right_outer = self.line_sensor_left_outer_back.getValue()

            # Calculate errors
            left_error = (left_outer + left_inner) / 2
            right_error = (right_inner + right_outer) / 2
            total_error = left_error - right_error

            # Accumulate integral of error
            integral += total_error

            # Calculate derivative of error
            derivative = total_error - last_error

            # Calculate PID adjustment
            adjustment = Kp * total_error + Ki * integral + Kd * derivative

            # Set lateral movement velocities
            # Positive adjustment moves the bot right; negative moves it left.
            # Left wheels move in one direction, right wheels in the opposite direction.
            sideways_velocity = base_velocity + adjustment
            self.set_velocities(
                wheel1v=sideways_velocity,  # Front-right (moves laterally)
                wheel2v=-sideways_velocity, # Front-left (moves laterally)
                wheel3v=sideways_velocity,  # Rear-right (moves laterally)
                wheel4v=-sideways_velocity  # Rear-left (moves laterally)
            )

            # Check if the line is centered (error within tolerance)
            if abs(total_error) < tolerance:
                print("Line is centered under the robot **BACK**.")
                self.stop()  # Stop the robot
                break

            # Update last error for derivative calculation
            last_error = total_error
            
    def correct_rotation_with_pid(self):
        """
        Ensures the robot is not standing with any degree of rotation.
        Uses the gyroscope sensor to correct rotational drift.
        Adjusts wheel velocities to rotate in place until the bot is aligned.
        """

        # PID parameters for rotation correction
        Kp = 0.05  # Proportional gain (how strongly to correct)
        Ki = 0.01  # Integral gain (accumulates past errors)
        Kd = 0.02  # Derivative gain (prevents oscillations)

        # PID control variables
        integral = 0
        last_error = 0

        # Tolerance for stopping (how close to 0 rotation is acceptable)
        tolerance = 0.01  # Tune based on gyroscope sensitivity

        while self.step(self.timestep) != -1:
            # Read gyroscope values (assuming Z-axis gives rotation)
            rotation_rate = self.gyro.getValues()[2]  # Get rotational velocity (yaw)

            # Calculate error (we want 0 rotation)
            error = -rotation_rate

            # Accumulate integral term
            integral += error

            # Compute derivative term
            derivative = error - last_error

            # Compute PID adjustment
            adjustment = Kp * error + Ki * integral + Kd * derivative

            # Update last error for the next iteration
            last_error = error

            # Apply correction: Counteract rotation using wheel speeds
            self.set_velocities(
                wheel1v=adjustment,  # Front-right (turning effect)
                wheel2v=-adjustment, # Front-left  (opposite turning effect)
                wheel3v=adjustment,  # Rear-right  (turning effect)
                wheel4v=-adjustment  # Rear-left   (opposite turning effect)
            )

            # Stop the bot when within acceptable tolerance
            if abs(error) < tolerance:
                print("Robot is correctly aligned.")
                self.stop()
                break
  
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
        # There should be better state handling, but this worked for now.

        while self.step(self.timestep) != -1:
            if self.state == "COLLECTING_INITIAL_COLORS":
                self.move_forward(velocity=YOUBOT_MAX_VELOCITY)
                self.collect_colors()
                if len(self.colors_detected) == 4:
                    print(f"Collected all four colors: {list(self.colors_detected)}")
                    self.state = "SKIPPING_THE_LAST_COLOR_SQUARE"

            elif self.state == "SKIPPING_THE_LAST_COLOR_SQUARE":
                self.move_forward(distance=COLOR_SQUARE_SIDE_LENGTH)
                print("Skipped the last color square")
                self.state = "DETECTING_LINE"

            elif self.state == "DETECTING_LINE":
                self.move_forward()
                if self.detected_a_line():
                    # self.move_forward(distance=0.002)
                    self.stop()
                    print("detected a line and stopped")
                    self.state = "ROTATING_90_DEGREES_CLOCKWISE_First"

            elif self.state == "ROTATING_90_DEGREES_CLOCKWISE_First":
                self.rotate_in_place(-90)
                self.stop()       
                self.center_on_line_with_pid()
                print("turned 90 degrees clockwise and stopped")
                self.state = "DETECTING_COLOR_Forth"

            elif self.state == "ROTATING_90_DEGREES_CLOCKWISE_Second":
                self.rotate_in_place(-90)
                self.stop()
                self.step(10*TIME_STEP)
                self.move_forward(distance= COLOR_SQUARE_SIDE_LENGTH /9 )
                
                print("turned 90 degrees clockwise and stopped")
                self.state = "PICKING_UP_BOTH_BOXES"
                
            elif self.state == "DETECTING_COLOR_Forth":
                
                self.move_forward()
                detected_color = self.detect_color(self.camera.getImageArray())
                if detected_color == self.colors_detected[0]:
                    COLOR_IN_HAND = detected_color
                    self.stop()
                    # self.move_forward(distance=COLOR_SQUARE_SIDE_LENGTH /2)
                    self.stop()
                    print("Detected the desired color and stopped")
                    self.state = "ROTATING_90_DEGREES_COUNTER_CLOCKWISE_First"
                elif detected_color != "Unknown"  :
                    self.center_on_line_with_pid() 
                    self.center_on_line_laterally_with_pid_Front()
                    # self.center_on_line_laterally_with_pid_back()
                    self.correct_rotation_with_pid()  
                    self.move_forward(distance=0.3)
                    self.center_on_line_with_pid() 
                    self.center_on_line_laterally_with_pid_Front()
                    self.center_on_line_laterally_with_pid_back()
                    self.center_on_line_laterally_with_pid_Front()
                    self.center_on_line_laterally_with_pid_back()
                    self.move_forward(distance=0.2)
                    self.center_on_line_with_pid() 
                    self.center_on_line_laterally_with_pid_Front()
                    self.center_on_line_laterally_with_pid_back()
                    self.correct_rotation_with_pid() 
                    self.state = "DETECTING_COLOR_Forth"
                # elif len(detected_color) >= len(self.colors_detected) :
                #     self.stop()
                #     self.rotate_in_place(180)
                #     self.stop()
                #     self.state = "DETECTING_COLOR_BACK"


                
            elif self.state == "DETECTING_COLOR_BACK":
                self.move_forward()
                detected_color = self.detect_color(self.camera.getImageArray())
                if detected_color == self.colors_detected[0]:
                    COLOR_IN_HAND = detected_color
                    self.stop()
                    # self.move_forward(distance=COLOR_SQUARE_SIDE_LENGTH / 2)
                    self.stop()
                    print("Detected the desired color and stopped")
                    self.rotate_in_place(-89.3)
                    self.stop()
                    self.move_forward(distance= COLOR_SQUARE_SIDE_LENGTH /9 )
                    
                    print("turned 90 degrees clockwise and stopped")
                    self.state = "PICKING_UP_BOTH_BOXES"
                elif detected_color != "Unknown"  :
                    self.center_on_line_with_pid() 
                    self.center_on_line_laterally_with_pid_Front()
                    # self.center_on_line_laterally_with_pid_back()
                    self.correct_rotation_with_pid()  
                    self.move_forward(distance=0.3)
                    self.center_on_line_with_pid() 
                    self.center_on_line_laterally_with_pid_Front()
                    self.center_on_line_laterally_with_pid_back()
                    self.center_on_line_laterally_with_pid_Front()
                    self.center_on_line_laterally_with_pid_back()
                    self.move_forward(distance=0.2)
                    self.center_on_line_with_pid() 
                    self.center_on_line_laterally_with_pid_Front()
                    self.center_on_line_laterally_with_pid_back()
                    self.correct_rotation_with_pid()
                    self.correct_rotation_with_pid() 
                    self.state = "DETECTING_COLOR_BACK"
                # elif len(detected_color) >= len(self.colors_detected) :
                #     self.stop()
                #     self.rotate_in_place(180)
                #     self.stop()
                #     self.state = "DETECTING_COLOR_First"

                # # the following condition works on its own, but haven't tested it with the previous condition.
                # # when detecting the four colors on line change direction.
                # if detected_color in self.colors_detected:
                #     self.detected_colors_on_line.add(detected_color)
                #     if self.detected_colors_on_line == set(self.colors_detected):
                #         print("All colors detected on the line.")
                #         self.switch_direction()
                #         # change the state. the nonsense is just for testing.
                #         self.state = "hisdfs"

            elif self.state == "ROTATING_90_DEGREES_COUNTER_CLOCKWISE_First":
                self.rotate_in_place(90)
                self.stop()
                self.center_on_line_laterally_with_pid_Front()
                self.step(10*TIME_STEP)
                self.move_forward(distance= COLOR_SQUARE_SIDE_LENGTH /9 )
                print("turned 90 degrees counter clockwise and stopped")
                self.state = "PICKING_UP_BOTH_BOXES"
                
            elif self.state == "ROTATING_90_DEGREES_COUNTER_CLOCKWISE_SECOND":
                self.rotate_in_place(90)
                self.stop()
                print("turned 90 degrees counter clockwise and stopped")
                self.state = "DETECTING_COLOR_BACK"

            elif self.state == "PICKING_UP_BOTH_BOXES":
                
                self.loop_Function(0)
                self.loop_Function(2)
                self.colors_detected.remove(COLOR_IN_HAND)
                print(self.colors_detected)
                self.state = "WHERE_IN_LINE"
                
            elif self.state == "WHERE_IN_LINE":     
                print("returing to Line following")
                self.move_forward()
                detected_color = self.detect_color(self.camera.getImageArray())
                print(COLOR_IN_HAND)
                print(detected_color)
                if detected_color == COLOR_IN_HAND and COLOR_IN_HAND != 'red':
                    # self.stop()
                    self.move_forward(distance=0.005)
                    self.stop()
                    self.correct_rotation_with_pid()
                    self.center_on_line_with_pid()
                    print("Detected the desired color and stopped")
                    self.rotate_in_place(-89.2)
                    self.stop()
                    self.center_on_line_with_pid() 
                    self.center_on_line_laterally_with_pid_Front()
                    # self.center_on_line_laterally_with_pid_back()
                    self.correct_rotation_with_pid()  
                    self.move_forward(distance=0.3)
                    self.center_on_line_with_pid() 
                    self.center_on_line_laterally_with_pid_Front()
                    self.center_on_line_laterally_with_pid_back()
                    self.correct_rotation_with_pid() 
                    self.state = "DETECTING_COLOR_Forth"
                 
                    
                elif detected_color == 'yellow' and len(self.colors_detected) != 4 :
                    self.stop()
                    # self.move_forward(distance=COLOR_SQUARE_SIDE_LENGTH / 2)
                    self.stop()
                    self.state = "BACK_FROM_THE_FIRST_COLOR"
                elif detected_color == COLOR_IN_HAND and COLOR_IN_HAND == "red":
                    self.move_forward(distance=0.005)
                    self.stop()
                    self.correct_rotation_with_pid()
                    self.center_on_line_with_pid()
                    print("Detected the desired color and stopped")
                    self.rotate_in_place(89.2)
                    self.stop()
                    self.center_on_line_with_pid() 
                    self.center_on_line_laterally_with_pid_Front()
                    # self.center_on_line_laterally_with_pid_back()
                    self.correct_rotation_with_pid()  
                    self.move_forward(distance=0.3)
                    self.center_on_line_with_pid() 
                    self.center_on_line_laterally_with_pid_Front()
                    self.center_on_line_laterally_with_pid_back()
                    self.correct_rotation_with_pid()
                    self.state = "DETECTING_COLOR_BACK"
                    
                elif detected_color == "red" and "red" != self.colors_detected[0]: 
                    self.stop()
                    # self.move_forward(distance=COLOR_SQUARE_SIDE_LENGTH / 2)
                    self.stop()
                    print("Detected the desired color and stopped")
                    self.state = "BACK_FROM_THE_LAST_COLOR"

            
            elif self.state == "BACK_FROM_THE_LAST_COLOR" :
                print("comming back from the last color")
                self.rotate_in_place(180)
                self.stop()
                self.state == "DETECTING_COLOR_BACK"
                
            elif self.state == "BACK_FROM_THE_FIRST_COLOR"  :
                print("comming back from the First color")
                self.rotate_in_place(180)
                self.stop()
                self.state == "DETECTING_COLOR_Forth"
                  
            else:
                print("quit")
                break


if __name__ == "__main__":
    robot = RobotController()
    robot.run()

    