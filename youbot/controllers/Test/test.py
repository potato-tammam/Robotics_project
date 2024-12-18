from collections import deque
from controller import Robot

# Constants
TIME_STEP = 32
YOUBOT_MAX_VELOCITY = 10.0  # Adjusted for the YouBot's speed range
WALL_DETECTION_THRESHOLD = 1000

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
        self.armMotors = []
        self.armMotors.append(self.getDevice("arm1"))
        self.armMotors.append(self.getDevice("arm2"))
        self.armMotors.append(self.getDevice("arm3"))
        self.armMotors.append(self.getDevice("arm4"))
        self.armMotors.append(self.getDevice("arm5"))
        # Set the maximum motor velocity.
        self.armMotors[0].setVelocity(1.5) # maxVelocity = 1.5
        self.armMotors[1].setVelocity(1.5)
        self.armMotors[2].setVelocity(1.5)
        self.armMotors[3].setVelocity(0.5)
        self.armMotors[4].setVelocity(1.5)

        #! Initialize arm position sensors.
        # These sensors can be used to get the current 
        # joint position and monitor the joint movements.
        self.armPositionSensors = []
        self.armPositionSensors.append(self.getDevice("arm1sensor"))
        self.armPositionSensors.append(self.getDevice("arm2sensor"))
        self.armPositionSensors.append(self.getDevice("arm3sensor"))
        self.armPositionSensors.append(self.getDevice("arm4sensor"))
        self.armPositionSensors.append(self.getDevice("arm5sensor"))
        for sensor in self.armPositionSensors:
            sensor.enable(TIME_STEP)

        #! Initialize gripper motors.
        self.finger1 = self.getDevice("finger::left")
        self.finger2 = self.getDevice("finger::right")
        # Set the maximum motor velocity.
        self.finger1.setVelocity(1.5)
        self.finger2.setVelocity(1.5) # 0.03
        # Read the miminum and maximum position of the gripper motors.
        self.fingerMinPosition = self.finger1.getMinPosition()
        self.fingerMaxPosition = self.finger1.getMaxPosition()
        
        
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
    
    def pick_up(self):
        self.armMotors[0].setPosition(1.7)
        self.armMotors[1].setPosition(-1)
        self.armMotors[2].setPosition(-1)
        self.armMotors[3].setPosition(0)
        self.finger1.setPosition(self.fingerMaxPosition)
        self.finger2.setPosition(self.fingerMaxPosition)

    def close_grippers(self):
        self.finger1.setPosition(0.013)     # Close gripper.
        self.finger2.setPosition(0.013)

    def hand_up(self):
        self.armMotors[0].setPosition(0)
        self.armMotors[1].setPosition(0)
        self.armMotors[2].setPosition(0)
        self.armMotors[3].setPosition(0)
        self.armMotors[4].setPosition(0)

    def fold_arms(self):
        self.armMotors[0].setPosition(-2.9)
        self.armMotors[1].setPosition(1.5)
        self.armMotors[2].setPosition(-2.6)
        self.armMotors[3].setPosition(1.7)
        self.armMotors[4].setPosition(0)

    def drop(self):
        # Move arm down
        self.armMotors[3].setPosition(0)
        self.armMotors[2].setPosition(-0.3)
        self.step(100 * TIME_STEP)

        self.armMotors[1].setPosition(-1.0)
        self.step(100 * TIME_STEP)

        self.armMotors[3].setPosition(-1.5)
        self.step(100 * TIME_STEP)

        self.armMotors[2].setPosition(-0.4)
        self.step(50 * TIME_STEP)
        self.armMotors[4].setPosition(-1)

        # Open gripper.
        self.finger1.setPosition(self.fingerMaxPosition)
        self.finger2.setPosition(self.fingerMaxPosition)
        self.step(50 * TIME_STEP)

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
    robot.run()
    # robot.close_grippers()
    # robot.hand_up()
