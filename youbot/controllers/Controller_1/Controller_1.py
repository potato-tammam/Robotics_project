from collections import deque
from controller import Robot
import math
import time

YOUBOT_MAX_VELOCITY = 10.0 
LINE_DESIRED_ERROR = 0
LINE_DETECTION_THRESHOLD = 1000
YOUBOT_WHEEL_RADIUS = 0.05 #meters
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

        # Line sensors
        self.line_sensor = self.getDevice("line sensor")
        self.line_sensor.enable(self.timestep)
        
        self.front_right_motor_sensor = self.wheels[0].getPositionSensor()
        self.front_right_motor_sensor.enable(self.timestep)
        
        # Camera
        self.camera = self.getDevice("cam")
        self.camera.enable(self.timestep)

        # State management
        self.colors_detected = deque(maxlen=4)
        self.state = 'COLLECTING_INITIAL_COLORS'

        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.timestep)
        
        self.direction = 'FORTH'
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
            if detected_color != "Unknown" and detected_color not in self.colors_detected:
                self.colors_detected.append(detected_color)
                print(f"Collected color: {detected_color}")


    def clamp_velocity(self, velocity):
        """
        Clamp motor velocities within allowed limits.
        """
        return max(-YOUBOT_MAX_VELOCITY, min(YOUBOT_MAX_VELOCITY, velocity))
    
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
            
            initial_sensor_value = self.front_right_motor_sensor.getValue()

            self.set_velocities(velocity, velocity, velocity, velocity)

            while self.front_right_motor_sensor.getValue() - initial_sensor_value < angle:
                self.step(self.timestep)

            self.stop()
            
        else:
            self.set_velocities(velocity, velocity, velocity, velocity)
        
    def move_backward(self, velocity=YOUBOT_MAX_VELOCITY):
        self.set_velocities(-velocity, -velocity, -velocity, -velocity)

    def stop(self):
        """
        Stop all wheel motors.
        """
        for wheel in self.wheels:
            wheel.setVelocity(0)

    def run(self):
        
        # There should be better state handling, but this worked for now.
        
        while self.step(self.timestep) != -1:
            if self.state == 'COLLECTING_INITIAL_COLORS':
                self.move_forward(velocity=YOUBOT_MAX_VELOCITY)
                self.collect_colors()
                if len(self.colors_detected) == 4:
                    print(f"Collected all four colors: {list(self.colors_detected)}")
                    self.state = 'SKIPPING_THE_LAST_COLOR_SQUARE'
            
            elif self.state == 'SKIPPING_THE_LAST_COLOR_SQUARE':
                self.move_forward(distance=COLOR_SQUARE_SIDE_LENGTH)
                print("Skipped the last color square")
                self.state = 'DETECTING_LINE'
                
            elif self.state == 'DETECTING_LINE':
                self.move_forward()
                if self.detected_a_line():
                    self.stop()
                    print("detected a line and stopped")
                    self.state = 'ROTATING_90_DEGREES_CLOCKWISE'
                    
            elif self.state == 'ROTATING_90_DEGREES_CLOCKWISE':
                self.rotate_in_place(-90)
                self.stop()
                print("turned 90 degrees clockwise and stopped")
                self.state = 'DETECTING_COLOR'
            
            elif self.state == 'DETECTING_COLOR':
                self.move_forward()
                detected_color = self.detect_color(self.camera.getImageArray())
                if detected_color == self.colors_detected[0]:
                    self.stop()
                    self.move_forward(distance = COLOR_SQUARE_SIDE_LENGTH/2)
                    self.stop()
                    print('Detected the desired color and stoppeds')
                    self.state = 'ROTATING_90_DEGREES_COUNTER_CLOCKWISE'
                    
                #the following condition works on its own, but haven't tested it with the previous condition.
                # when detecting the four colors on line change direction.
                if detected_color in self.colors_detected:
                    self.detected_colors_on_line.add(detected_color)
                    if self.detected_colors_on_line == set(self.colors_detected):
                        print("All colors detected on the line.")
                        self.switch_direction()
                        # change the state. the nonsense is just for testing.
                        self.state = 'hisdfs'
                
            elif self.state == 'ROTATING_90_DEGREES_COUNTER_CLOCKWISE':
                self.rotate_in_place(90)
                self.stop()
                print("turned 90 degrees counter clockwise and stopped")
                self.state = 'hello'
            
            else:
                print('quit')
                break
                

                    
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
            self.set_velocities(wheel1v=velocity, wheel2v=-velocity, wheel3v=velocity, wheel4v=-velocity)
        else:
            self.set_velocities(wheel1v=-velocity,wheel2v=velocity,wheel3v=-velocity,wheel4v=velocity)

        while self.step(self.timestep) != -1:
            angular_velocity = self.gyro.getValues()[2]
            rotation_angle += angular_velocity * (self.timestep / 1000.0)
            if abs(rotation_angle) >= abs(target_rotation):
                break

        self.stop()
        
    def switch_direction(self):

        self.direction = 'BACK' if self.direction == 'FORTH' else 'FORTH'
        print(f"Switching direction to {self.direction}")

        self.rotate_in_place(180 if self.direction == 'BACK' else -180)
        self.stop()
        self.detected_colors_on_line.clear()
        print("Rotation complete, direction switched.")

if __name__ == "__main__":
    robot = RobotController()
    robot.run()
