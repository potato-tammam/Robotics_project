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
        self.timestep = int(self.getBasicTimeStep())
        
        self.integral = 0
        self.last_error = 0
        self.Kp = 0.01
        self.Ki = 0
        self.Kd = 0.01
        
        # Motors (YouBot has four wheel motors)
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
        self.colors_detected = deque(maxlen=4)
        self.collecting_colors = True

    def get_weighted_sensor_value(self):
        value = 0
        for index, sensor in enumerate(self.line_sensors):
            if sensor.getValue() > 200:
                value += self.weights[index]
        return value
    
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
            if detected_color != "Unknown" and detected_color not in self.colors_detected:
                self.colors_detected.append(detected_color)
                print(f"Collected color: {detected_color}")

        # Transition to line-following after collecting all colors
        if len(self.colors_detected) == 4:
            print(f"Collected all four colors: {list(self.colors_detected)}")
            self.collecting_colors = False

    def get_line_error(self):
        """
        Calculate error for line-following using weighted light sensor values.
        """
        total_error = 0
        for i, sensor in enumerate(self.line_sensors):
            if sensor.getValue() > LINE_DETECTION_THRESHOLD:
                total_error += self.line_weights[i]
        print(total_error - LINE_DESIRED_ERROR)
        return total_error - LINE_DESIRED_ERROR

    def clamp_velocity(self, velocity):
        """
        Clamp motor velocities within allowed limits.
        """
        return max(-YOUBOT_MAX_VELOCITY, min(YOUBOT_MAX_VELOCITY, velocity))

    def follow_line(self, velocity=YOUBOT_MAX_VELOCITY):
        pass
    


    def move_forward(self, velocity=2):
        """
        Move forward at a constant speed.
        """
        for wheel in self.wheels:
            wheel.setVelocity(velocity)

    def stop(self):
        """
        Stop all wheel motors.
        """
        for wheel in self.wheels:
            wheel.setVelocity(0)

    def run(self):
        """
        Main loop for the robot.
        """
        while self.step(self.timestep) != -1:
            if self.collecting_colors:
                self.move_forward(velocity=YOUBOT_MAX_VELOCITY)  # Move forward to detect colors
                self.collect_colors()
            else:
                self.follow_line(velocity=YOUBOT_MAX_VELOCITY)  # Follow the line after collecting colors


if __name__ == "__main__":
    pid = PIDController(Kp, Kd, Ki)
    robot = RobotController(pid)
    robot.run()
