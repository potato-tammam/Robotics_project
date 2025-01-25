from collections import deque
from controller import Robot

# Constants
TIME_STEP = 32
YOUBOT_MAX_VELOCITY = 10.0  # Adjusted for the YouBot's speed range
Pick_Up_from_Box_Matrix =[0.05, -0.63, -0.3, -0.88, 0]
Fold_Matrix = [-2.9, 1.5 , -2.6 , 1.7 , 0 ]
# Constants
WALL_DETECTION_THRESHOLD = 1000

WHEEL_RADIUS = 0.047  # Radius of the wheels in meters
AXLE_LENGTH = 0.3  # Distance between the wheels in meters
TURN_90_DURATION = 2.5  # Approximate duration for 90-degree turn (adjust for precision)
# PID Constants
Kp = 0.9
Kd = 0.1
Ki = 0.0

LINE_DESIRED_ERROR = 0
LINE_DETECTION_THRESHOLD = 0


# PID Controller Class
class PIDController:
    def __init__(self, Kp, Kd, Ki):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.last_error = 0
        self.integral = 0

    def compute_correction(self, error):
        P = self.Kp * error
        D = self.Kd * (error - self.last_error)
        I = self.Ki * self.integral
        self.last_error = error
        self.integral += error
        return P + D + I


# Robot Controller Class
class RobotController(Robot):
    def __init__(self, pid):
        super().__init__()
        self.pid = pid
        self.timestep = int(self.getBasicTimeStep())

        # Motors
        self.wheels = [
            self.getDevice("wheel1"),  # Front-left
            self.getDevice("wheel3"),  # Rear-left
            self.getDevice("wheel2"),  # Front-right
            self.getDevice("wheel4"),  # Rear-right
        ]
        for wheel in self.wheels:
            wheel.setPosition(float("inf"))  # Set to velocity control mode
            wheel.setVelocity(0)
        
        """
        Initialize the arms and claws with explicit joint values for positions.
        """
        self.armMotors1 = []  #motors for arm 1
        self.armMotors2 = []  #motors for arm 2
        
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
        for arm in self.armMotors1 :
            arm.setVelocity(1.5)

        for arm in self.armMotors2 :
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
        self.Arm1finger2.setVelocity(1.5) # 0.03
        
        self.Arm2finger1.setVelocity(1.5)
        self.Arm2finger2.setVelocity(1.5)
        
        # Read the miminum and maximum position of the gripper motors.
        self.fingerMinPosition = self.Arm1finger1.getMinPosition()
        self.fingerMaxPosition = self.Arm1finger1.getMaxPosition()
        
        
        # Initialize the front infrared sensors
        self.wall_sensors = [self.getDevice(f"front-sensor-{i}") for i in range(5)]
        for wall_sensor in self.wall_sensors:
            wall_sensor.enable(self.timestep)
        
        # Line sensors
        self.line_sensors = [self.getDevice(f"lfs{i}") for i in range(8)]
        self.line_weights = [-1000, -1000, -1000, -1000, 1000, 1000, 1000, 1000]  # Adjusted weights
        for sensor in self.line_sensors:
            sensor.enable(self.timestep)

        # Camera
        self.camera = self.getDevice("cam")
        self.camera.enable(self.timestep)

        # State management
        self.colors_detected = deque(maxlen=4)  # Queue to hold 4 colors
        self.collecting_colors = True

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







    def set_wheel_speeds(self , front_left, front_right, rear_left, rear_right):
        """Set velocities for the omniwheels."""
        
        self.wheels[0].setVelocity(front_left)   # Front-left wheel
        self.wheels[1].setVelocity(front_right)  # Front-right wheel
        self.wheels[2].setVelocity(rear_left)    # Rear-left wheel
        self.wheels[3].setVelocity(rear_right)   # Rear-right wheel

    def turn_90(self):
        """Turn the robot 90 degrees on the spot."""
        angular_velocity = 2.0  # Set angular velocity for turning
        duration = TURN_90_DURATION  # Adjust based on testing in Webots
        
        # Omni-wheel rotation configuration for turning in place
        self.set_wheel_speeds(-angular_velocity, angular_velocity, -angular_velocity, angular_velocity)
        start_time = self.getTime()
        
        while self.step(TIME_STEP) != -1:
            if self.getTime() - start_time >= duration:
                break
        
        self.set_wheel_speeds(0, 0, 0, 0)  # Stop the robot

    def turn_180(self):
        """Turn the robot 180 degrees on the spot."""
        angular_velocity = 2.0  # Set angular velocity for turning
        duration = TURN_90_DURATION * 2  # Twice the time for a 180-degree turn
        
        # Omni-wheel rotation configuration for turning in place
        self.set_wheel_speeds(-angular_velocity, angular_velocity, -angular_velocity, angular_velocity)
        start_time = self.getTime()
        
        while self.step(TIME_STEP) != -1:
            if self.getTime() - start_time >= duration:
                break
        
        self.set_wheel_speeds(0, 0, 0, 0)  # Stop the robot

    def move_forward(self,distance):
        """Move the robot forward by a specific distance."""
        velocity = 2.0  # Linear velocity
        duration = distance / (velocity * WHEEL_RADIUS)  # Duration based on distance and speed
        
        # Omni-wheel configuration for forward motion
        self.set_wheel_speeds(velocity, velocity, velocity, velocity)
        start_time = self.getTime()
        
        while self.step(TIME_STEP) != -1:
            if self.getTime() - start_time >= duration:
                break
        
        self.set_wheel_speeds(0, 0, 0, 0)  # Stop the robot

    # Example usage
    # Uncomment these lines to test
    # turn_90()
    # turn_180()
    # move_forward(1.0)  # Move forward by 1 meter


    def follow_line(self):
        """
        Follow the line using PID control.
        """
        error = self.get_line_error()
        correction = self.pid.compute_correction(error)

        left_velocity = self.clamp_velocity(YOUBOT_MAX_VELOCITY - correction)
        right_velocity = self.clamp_velocity(YOUBOT_MAX_VELOCITY + correction)
        
        # Set velocities for skid steering
        self.wheels[0].setVelocity(left_velocity)  # Front-left
        self.wheels[1].setVelocity(right_velocity)  # Front-right
        self.wheels[2].setVelocity(left_velocity)  # Rear-left
        self.wheels[3].setVelocity(right_velocity)  # Rear-right
    
    # def wait_until_position_reached(self, arm, target_positions, tolerance=0.01):
    #     sensors = self.arm1PositionSensors if arm == 1 else self.arm2PositionSensors
    #     while True:
    #         all_reached = False
    #         for i, sensor in enumerate(sensors):
    #             if abs(sensor.getValue() - target_positions[i]) > tolerance:
    #                 print("not there yet")
    #                 print(abs(sensor.getValue() - target_positions[i]) > tolerance)
    #                 print(abs(sensor.getValue() - target_positions[i]))
    #                 all_reached = False
  
    #                 break

    #         if all_reached:
    #             print("I am there ")
    #             break
    #         self.step(TIME_STEP)  # Step simulation to allow movement

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
    def center_on_line_with_pid(self):
        """
        Adjust the wheel velocities using PID control to center the robot on the line
        by moving sideways without rotating. Ensures the line is centered under all 8 sensors.
        The velocity never exceeds 10 units.
        """
        # PID parameters (tune these values based on your robot's behavior)
        Kp = 0.05  # Proportional gain
        Ki = 0.01  # Integral gain
        Kd = 0.02  # Derivative gain

        # Variables for PID control
        integral_front = 0
        last_error_front = 0
        integral_back = 0
        last_error_back = 0


        # Tolerance for stopping (adjust based on sensitivity requirements)
        tolerance = 5  # Minimum error to consider the line centered

        # Base velocity for the wheels (used for lateral movement)
        base_velocity = 10  # Ensure this doesn't exceed the maximum velocity

        while self.step(self.timestep) != -1:
            # Read sensor values
            front_left_outer = self.line_sensor_left_outer.getValue()
            front_left_inner = self.line_sensor_left_inner.getValue()
            front_right_inner = self.line_sensor_right_outer.getValue()
            front_right_outer = self.line_sensor_right_inner.getValue()

            back_left_outer = self.line_sensor_left_outer_back.getValue()
            back_left_inner = self.line_sensor_left_inner_back.getValue()
            back_right_inner = self.line_sensor_left_inner_back.getValue()
            back_right_outer = self.line_sensor_right_outer_back.getValue()

            # Calculate errors for front and back sensors
            front_left_error = (front_left_outer + front_left_inner) / 2
            front_right_error = (front_right_inner + front_right_outer) / 2
            front_total_error = front_left_error - front_right_error

            back_left_error = (back_left_outer + back_left_inner) / 2
            back_right_error = (back_right_inner + back_right_outer) / 2
            back_total_error = back_left_error - back_right_error

            # Accumulate integral of errors
            integral_front += front_total_error
            integral_back += back_total_error

            # Calculate derivative of errors
            derivative_front = front_total_error - last_error_front
            derivative_back = back_total_error - last_error_back

            # Calculate PID adjustments for front and back
            adjustment_front = Kp * front_total_error + Ki * integral_front + Kd * derivative_front
            adjustment_back = Kp * back_total_error + Ki * integral_back + Kd * derivative_back

            # Use the larger of the two adjustments for velocity setting
            adjustment = max(abs(adjustment_front), abs(adjustment_back)) * (1 if adjustment_front >= 0 or adjustment_back >= 0 else -1)

            # Ensure the adjustment does not cause velocity to exceed limits
            sideways_velocity = max(-10, min(base_velocity + adjustment, 10))


            # Set lateral movement velocities
            # Positive adjustment moves the bot right; negative moves it left.
            # Left wheels move in one direction, right wheels in the opposite direction.
            self.set_velocities(
                wheel1v=sideways_velocity,  # Front-right (moves laterally)
                wheel2v=-sideways_velocity, # Front-left (moves laterally)
                wheel3v=sideways_velocity,  # Rear-right (moves laterally)
                wheel4v=-sideways_velocity  # Rear-left (moves laterally)
            )


            # Check if both front and back errors are within tolerance
            if abs(front_total_error) < tolerance and abs(back_total_error) < tolerance:
                print("Line is centered under the robot.")
                self.stop()  # Stop the robot
                break

            # Update last errors for derivative calculation
            last_error_front = front_total_error
            last_error_back = back_total_error
  
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
        Ki = 0.01  # Integral gain
        Kd = 0.02  # Derivative gain

        # Variables for PID control
        integral = 0
        last_error = 0

        # Tolerance for stopping (adjust based on sensitivity requirements)
        tolerance = 5  # Minimum error to consider the line centered

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
        Ki = 0.01  # Integral gain
        Kd = 0.02  # Derivative gain

        # Variables for PID control
        integral = 0
        last_error = 0

        # Tolerance for stopping (adjust based on sensitivity requirements)
        tolerance = 5  # Minimum error to consider the line centered

        # Base velocity for the wheels (used for lateral movement)
        base_velocity = YOUBOT_MAX_VELOCITY * 0.3

        while self.step(self.timestep) != -1:
            # Read sensor values
            left_outer = self.line_sensor_left_outer_back.getValue()
            left_inner = self.line_sensor_left_inner_back.getValue()
            right_inner = self.line_sensor_right_inner_back.getValue()
            right_outer = self.line_sensor_right_outer_back.getValue()

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
            

      
    def close_grippers(self , arm):
        if arm==1:
          
            self.Arm1finger1.setPosition(0.001)    
            self.Arm1finger2.setPosition(0.001)
            self.step(100 * TIME_STEP)
        elif arm==2:
            self.Arm2finger1.setPosition(0)     
            self.Arm2finger2.setPosition(0)
        else:
            print("please set the arm ")


    def hand_up(self,arm):
        if arm==1:
            for i, motor in enumerate(self.armMotors2):
                motor.setPosition(0)
        elif arm==2:
            for i, motor in enumerate(self.armMotors2):
                motor.setPosition(0)
        else:
            print("please set the arm ")
       

    def fold_arms(self , arm):
        target_postion = Fold_Matrix
        if arm==1:
            for i, motor in enumerate(self.armMotors1):
                motor.setPosition(target_postion[i])
        elif arm ==2:
            for i, motor in enumerate(self.armMotors2):
                motor.setPosition(target_postion[i])
        else:
            print("error please set the arm")
        

    def open_gribbers(self, arm):
        if arm==1:
            self.Arm1finger1.setPosition(self.fingerMaxPosition)
            self.Arm1finger2.setPosition(self.fingerMaxPosition)
        elif arm==2:
            self.Arm2finger1.setPosition(self.fingerMaxPosition)
            self.Arm2finger2.setPosition(self.fingerMaxPosition)
        else:
            print("please set the arm ")

        # Open gripper.




    def grab_And_retract(self,arm):
        """
        This mthod will take small boxes of the big boxes and fold the arm 

        Args:
            arm (1,2): which arm you trying to grab with 
        """
        
        self.move_arm(arm , Pick_Up_from_Box_Matrix)
        print("reaching")
        self.step(10 * TIME_STEP)
        self.open_gribbers(arm)
        print("oppening the gribbers")
        self.step(25 * TIME_STEP)
        print("closing the gribbers")
        self.close_grippers(arm)
        self.step(5 * TIME_STEP)
        print("floding")
        self.fold_arms(arm)
        self.step(5 * TIME_STEP)
        
    def Put_Box_On_wall(self,arm):
        """
        This mthod will put small boxes on the wall and retract the arm 

        Args:
            arm (1,2): which arm you trying to grab with 
        """
        self.move_arm(arm , Pick_Up_from_Box_Matrix)
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
    def move_forward_1(self, velocity=5):
        """
        Move forward at a constant speed.
        """
        for wheel in self.wheels:
            wheel.setVelocity(velocity)

    def stop_moving(self):
        """
        Stop the robot's movement.
        """
        for wheel in self.wheels:
            wheel.setVelocity(0)
        for _ in range(100):
            if self.step(self.timestep) == -1:
                break
    
    def detect_wall(self):
        """
        Detect a wall using the front infrared sensor.
        :return: True if a wall is detected, False otherwise.
        """
        for wall_sensor in self.wall_sensors:
            if wall_sensor.getValue() < WALL_DETECTION_THRESHOLD:
                print(f"Wall detected by sensor: {wall_sensor.getName()} with value {wall_sensor.getValue()}")
                return True
        print("No wall detected by front sensors")
        return False
    
    def rotate_180(self):
        """
        Rotate the robot by 180 degrees in place.
        """
        print('start rotating')
        rotation_velocity = YOUBOT_MAX_VELOCITY / 2
        # Rotate in place by setting one side's wheels to rotation_velocity and the other side's wheels to 0
        for i in range(2):
            self.wheels[i].setVelocity(rotation_velocity)
            self.wheels[i + 2].setVelocity(-rotation_velocity)
        duration = 1740
        for _ in range(duration):
            for wheel in self.wheels:
                print(f'{wheel.getName()} speed is {wheel.getVelocity()}')
            if self.step(self.timestep) == -1:
                break
        print('finish rotating')
           
    def run(self):
        """
        Main loop for the robot.
        """
        # while self.step(self.timestep) != -1:
        # grab_positions = [
        #     0.0,  # Base joint rotation
        #     0.5,  # Shoulder joint position
        #     -1.2, # Elbow joint position
        #     0.6,  # Wrist joint position
        #     0.0   # End-effector rotation
        #     ]
        # print("Starting task...")
        # self.execute_task("left", "left", grab_positions, action="grab")
        # print("Task completed.")
        while self.step(self.timestep) != -1:
            if False:
                print('collecting colors')
                self.move_forward_1()  # Move forward to detect colors
                cameraArray = self.camera.getImageArray()
                if cameraArray:
                    detected_color = self.detect_color(cameraArray)
                    print(f'detected color is: {detected_color}')
                    if detected_color != "Unknown" and detected_color not in self.colors_detected:
                        self.colors_detected.append(detected_color)
                        print(f"Collected color: {detected_color}")
                    print(len(self.colors_detected))
                if len(self.colors_detected) == 4:
                    self.collecting_colors = False
            else:
                if self.detect_wall():
                    self.stop_moving()
                    self.rotate_180()
                    self.stop_moving()
                else:
                    self.move_forward_1()


if __name__ == "__main__":
    pid = PIDController(Kp, Kd, Ki)
    robot = RobotController(pid)
    # robot.run(2)
    # robot.fold_arms(1)
    
    robot.grab_And_retract(1)
    robot.step(100 * TIME_STEP)
    robot.Put_Box_On_wall(1)
# if (robot.pick_up(1)):
#     print("closing")
#     robot.close_grippers(1)
    # robot.hand_up()
