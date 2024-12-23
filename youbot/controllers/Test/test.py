from collections import deque
from controller import Robot

# Constants
TIME_STEP = 32
YOUBOT_MAX_VELOCITY = 10.0  # Adjusted for the YouBot's speed range

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
            self.getDevice("wheel2"),  # Front-right
            self.getDevice("wheel3"),  # Rear-left
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

    def move_forward(self, steps=None, velocity=2):
        """
        Move forward at a constant speed. If steps provided, move for that duration.
        """
        count = 0
        while steps is None or count < steps:
            for wheel in self.wheels:
                wheel.setVelocity(velocity)
            count += 1 if steps else 0
            if self.step(self.timestep) == -1:
                break

    def rotate(self, angle):
        """
        Rotate the robot by a given angle in degrees.
        """
        velocity = YOUBOT_MAX_VELOCITY / 2
        duration = int(abs(angle) / 90 * 10)  # Adjust duration based on angle
        direction = -1 if angle < 0 else 1
        for wheel in self.wheels:
            wheel.setVelocity(velocity * direction)
        for _ in range(duration):
            if self.step(self.timestep) == -1:
                break

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

    def detect_wall(self, threshold=500):
        """
        Detect a wall using three front-facing proximity sensors.
        :param threshold: The distance value above which a wall is considered detected.
        :return: True if a wall is detected, False otherwise.
        """
        # Assuming the three front-facing proximity sensors are named "ps0", "ps1", and "ps2"
        front_sensors = [self.getDevice(f"ps{i}") for i in range(3)]
        for sensor in front_sensors:
             sensor.enable(self.timestep)

        # Check the values of the front-facing sensors
        for sensor in front_sensors:
            if sensor.getValue() > threshold:  # Compare the value to the threshold
                print(f"Wall detected by sensor: {sensor.getName()} with value {sensor.getValue()}")
                return True

        print("No wall detected by front sensors")
        return False
   
    
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

    def pick_up(self, arm):
        if arm == 1:
            target_positions = [0.05, -0.63, -0.3, -0.88, 0]
            for i, motor in enumerate(self.armMotors1):
                motor.setPosition(target_positions[i])
                


            
        elif arm == 2:
            target_positions = [0.05, -0.63, -0.3, -0.88, 0]
            for i, motor in enumerate(self.armMotors2):
                motor.setPosition(target_positions[i])

        else:
            print("Please set the arm")


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
            self.armMotors1[0].setPosition(0)
            self.armMotors1[1].setPosition(0)
            self.armMotors1[2].setPosition(0)
            self.armMotors1[3].setPosition(0)
            self.armMotors1[4].setPosition(0)
        elif arm==2:
            self.armMotors2[0].setPosition(0)
            self.armMotors2[1].setPosition(0)
            self.armMotors2[2].setPosition(0)
            self.armMotors2[3].setPosition(0)
            self.armMotors2[4].setPosition(0)
        else:
            print("please set the arm ")
       

    def fold_arms(self , arm):
        if arm==1:
            self.armMotors1[0].setPosition(-2.9)
            self.armMotors1[1].setPosition(1.5)
            self.armMotors1[2].setPosition(-2.6)
            self.armMotors1[3].setPosition(1.7)
            self.armMotors1[4].setPosition(0)
        elif arm ==2:
            self.armMotors2[0].setPosition(-2.9)
            self.armMotors2[1].setPosition(1.5)
            self.armMotors2[2].setPosition(-2.6)
            self.armMotors2[3].setPosition(1.7)
            self.armMotors2[4].setPosition(0)
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
        
        self.pick_up(arm)
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
        self.pick_up(arm)
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
            if self.collecting_colors:
                self.move_forward()  # Move forward to detect colors
                cameraArray = self.camera.getImageArray()
                if cameraArray:
                    detected_color = self.detect_color(cameraArray)
                    if detected_color != "Unknown" and detected_color not in self.colors_detected:
                        self.colors_detected.append(detected_color)
                        print(f"Collected color: {detected_color}")
                if len(self.colors_detected) == 4:
                    self.collecting_colors = False
            else:
                # Implement rest of the pseudocode logic here
                pass


if __name__ == "__main__":
    pid = PIDController(Kp, Kd, Ki)
    robot = RobotController(pid)
    # robot.fold_arms(2)
    # robot.fold_arms(1)
    
    robot.grab_And_retract(1)
    robot.step(100 * TIME_STEP)
    robot.Put_Box_On_wall(1)
# if (robot.pick_up(1)):
#     print("closing")
#     robot.close_grippers(1)
    # robot.hand_up()
