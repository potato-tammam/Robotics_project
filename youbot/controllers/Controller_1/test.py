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


def move_arm(self, arm_name, positions):
    """
    Move the specified arm joints to the given positions.
    :param arm_name: Name of the arm ('left' or 'right').
    :param positions: List of target positions for each joint.
    """
    if arm_name == "left":
        arm_joints = [
            self.getDevice("left_arm_joint1"),
            self.getDevice("left_arm_joint2"),
            self.getDevice("left_arm_joint3"),
            self.getDevice("left_arm_joint4"),
            self.getDevice("left_arm_joint5")
        ]
    elif arm_name == "right":
        arm_joints = [
            self.getDevice("right_arm_joint1"),
            self.getDevice("right_arm_joint2"),
            self.getDevice("right_arm_joint3"),
            self.getDevice("right_arm_joint4"),
            self.getDevice("right_arm_joint5")
        ]
    else:
        raise ValueError(f"Invalid arm name: {arm_name}")

    # Set the arm joints to the target positions
    for joint, position in zip(arm_joints, positions):
        joint.setPosition(position)
    
    print(f"{arm_name.capitalize()} arm moved to positions: {positions}")
    self.step(self.timestep)  # Allow time for the arm to move

def control_claw(self, claw_name, action):
    """
    Control the specified claw to grab or release a cube.
    :param claw_name: Name of the claw ('left' or 'right').
    :param action: Action to perform ('grab' or 'release').
    """
    if claw_name == "left":
        claw = self.getDevice("left_claw")
    elif claw_name == "right":
        claw = self.getDevice("right_claw")
    else:
        raise ValueError(f"Invalid claw name: {claw_name}")

    if action == "grab":
        claw.setPosition(0)  # Close the claw to grab
        print(f"{claw_name.capitalize()} claw has grabbed the cube.")
    elif action == "release":
        claw.setPosition(1)  # Open the claw to release
        print(f"{claw_name.capitalize()} claw has released the cube.")
    else:
        raise ValueError(f"Invalid action: {action}")

    self.step(self.timestep)  # Allow time for the claw to perform the action

def grab_and_place(self, arm_name, claw_name, grab_positions, release_positions):
    """
    Perform the sequence of grabbing a cube, moving to a destination, and placing the cube.
    :param arm_name: Name of the arm ('left' or 'right').
    :param claw_name: Name of the claw ('left' or 'right').
    :param grab_positions: List of joint positions to extend the arm for grabbing.
    :param release_positions: List of joint positions to place the cube.
    """
    # Extend the arm to grab position
    print(f"Moving {arm_name} arm to grab position...")
    self.move_arm(arm_name, grab_positions)

    # Grab the cube with the specified claw
    print(f"Grabbing cube with {claw_name} claw...")
    self.control_claw(claw_name, "grab")

    # Retract the arm to a transport position (optional)
    print(f"Retracting {arm_name} arm to transport position...")
    transport_positions = [0.0, 1.0, 0.0, 0.0, 0.0]  # Example transport position
    self.move_arm(arm_name, transport_positions)

    # Simulate moving to the destination
    print("Moving to destination...")
    self.move_forward(steps=200)  # Move forward to the destination

    # Extend the arm to release position
    print(f"Moving {arm_name} arm to release position...")
    self.move_arm(arm_name, release_positions)

    # Release the cube
    print(f"Placing cube with {claw_name} claw...")
    self.control_claw(claw_name, "release")

    # Retract the arm to resting position
    print(f"Retracting {arm_name} arm to resting position...")
    resting_positions = [0.0, 1.0, 0.0, 0.0, 0.0]  # Example resting position
    self.move_arm(arm_name, resting_positions)

    def run(self):
        """
        Main loop for the robot.
        """
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
    robot.run()
