# Copyright (C) 2024  Jose Ángel Pérez Garrido
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""Obstacle_Robot controller."""

import random
import time
import math
from controller import Robot, Motor, DistanceSensor, LightSensor, Accelerometer

# Set of predefined colors for LEDs.
LED_COLORS = {
    "red":0xFF0000,  # RED
    "green":0x00FF00,  # GREEN
    "blue":0x0000FF,  # BLUE
    "mint":0x00FFFF,  # MINT
    "purple":0xFF00FF,  # PURPLE
    "yellow":0xFFFF00,  # YELLOW
    "white":0xFFFFFF   # WHITE
}

# Sensor maximum value constants
MAX_LIGHT = 1024 # Maximum value of light detectable by the light sensor
MAX_DISTANCE = 128 # Maximum value of distance detectable by the distance sensor
        
# Environment variables (may need to be calibrated according to the environment)
light_threshold = 270 # Threshold to detect light
obstacle_threshold = 90 # Distance to detect an obstacle
close_obstacle_threshold = 15 # Distance to consider the robot is too close to an obstacle
stuck_threshold = 0.005  # Minimum movement needed to not be considered stucked
max_stuck_time = 1.5  # Maximum time considered stuck (in seconds)

# Choose 3 global random numbers.
rand_a = random.uniform(-1.5, 1.0)
rand_b = random.uniform(-1.5, 1.0)
rand_c = random.uniform(1.0, 2.0)

class ObstacleRobot:
        
    def __init__(self):   
        self.robot = Robot()
         
        global TIME_STEP
        TIME_STEP = int(self.robot.getBasicTimeStep()) # get the time step of the current world.
        
        self.initialize_variables()
        
        # Initialize sensors
        self.light_sensors = self.initialize_light_sensors()
        self.distance_sensors = self.initialize_distance_sensors()
        self.accelerometer = self.initialize_accelerometer_sensor()
        self.gps = self.initialize_gps_sensor()
        
        # Initialize actuators
        # Wheel motor
        self.wheel_motor = self.initialize_wheel_motor()
        # Set the robot's maximum speed and cruising speed
        self.MAX_SPEED = self.wheel_motor.getMaxVelocity()
        self.CRUISING_SPEED = 0.5 * self.MAX_SPEED
        
        # Rotation (pivot) motor
        self.pivot_motor = self.initialize_pivot_motor()
        # Set the robot's maximum rotation
        self.MAX_ROTATION = self.pivot_motor.getMaxPosition()
        
        self.twister_motor = self.initialize_twister_motor()
        self.leds = self.initialize_leds()
       
        
        
    ################################
    #                              #
    #   COMPONENTS INITIALIZATION  #
    #                              #
    ################################
    def initialize_wheel_motor(self):
        """ 
            Return a wheel motor initialized
        """ 
        wheel_motor = self.robot.getDevice('motor')
        wheel_motor.setPosition(float('inf'))
        wheel_motor.setVelocity(0.0)
        return wheel_motor
        
    def initialize_pivot_motor(self): 
        """ 
            Return a pivot motor initialized
        """
        pivot_motor = self.robot.getDevice('pivot')
        pivot_motor.setPosition(0.0)  
        pivot_motor.setVelocity(5.0) # Default rotation speed     
        return pivot_motor
        
    def initialize_twister_motor(self): 
        """ 
            Return a twister motor initialized
        """
        twister_motor = self.robot.getDevice('twister')
        twister_motor.setPosition(0.0)
        twister_motor.setVelocity(5.0) # Default rotation speed 
        return twister_motor

    def initialize_light_sensors(self):
        """ 
            Return a list of Light Sensors initialized
        """ 
        light_sensors_names = ['left light sensor', 'right light sensor']
        light_sensors = [self.robot.getDevice(name) for name in light_sensors_names]
        for sensor in light_sensors:
            sensor.enable(TIME_STEP)
        return light_sensors
        
    def initialize_distance_sensors(self):
        """ 
            Return a list of Distance Sensors initialized
        """ 
        distance_sensors_names = ['left distance sensor', 'right distance sensor']
        distance_sensors = [self.robot.getDevice(name) for name in distance_sensors_names]
        for sensor in distance_sensors:
            sensor.enable(TIME_STEP)
        return distance_sensors
        
    def initialize_accelerometer_sensor(self): 
        """ 
            Return an accelerometer device initialized
        """
        accelerometer = self.robot.getDevice('accelerometer')
        accelerometer.enable(TIME_STEP)
        return accelerometer
        
    def initialize_gyro_sensor(self):
        """ 
            Return a gyroscope device initialized
        """
        gyro = self.robot.getDevice('gyro')
        gyro.enable(TIME_STEP)
        return gyro
        
    def initialize_gps_sensor(self):
        """ 
            Return a GPS device initialized
        """
        gps = self.robot.getDevice('gps')
        gps.enable(TIME_STEP)
        return gps
        
    def initialize_leds(self): 
        """ 
            Return a list of LEDS initialized
        """
        led_names = ['left light sensor led', 'right light sensor led',
            'left distance sensor led','right distance sensor led',
            'motor led', 'pivot led']
        leds = [self.robot.getDevice(name) for name in led_names]
        for led in leds:
            led.set(LED_COLORS.get("red")) # Set color RED
        return leds

    def initialize_variables(self):
        """ 
            Set robot initial temporal variables values
        """
        self.inverted = 0 # Is the robot overturned
        self.goal = 0 # Is the goal reached
        self.stuck_timer = self.robot.getTime() # Last time the robot has moved
        self.prev_position = [0, 0, 0] # Previous position of the robot (GPS)
        


    ################################
    #                              #
    #       CONTROL FUNCTIONS      #
    #                              #
    ################################
    def move_robot(self,speed):
        """ 
            Move robot adjusting the wheel motor and speed.
            It also takes into account the robot's orientation (inverted).
        """
        self.wheel_motor.setVelocity(self.inverted*speed)
        
    def rotate_robot(self,position):
        """ 
            Steer the robot adjusting the pivot motor position.
        """
        self.pivot_motor.setPosition(position)
        
        
        
    ################################
    #                              #
    #           BEHAVIOURS         #
    #                              #
    ################################
    def goal_reached(self):
        """ 
            Return whether goal has been reached
        """
        return self.goal 
        
        
    def goal_seeking_behaviour(self):
        """
            Explore the world trying to find the goal.
            Sinusoidal movement with random parameters.
        """
        print("GOAL SEEKING")    
        
        current_time = self.robot.getTime()

        self.rotate_robot(0.5 * (math.sin(current_time * rand_c + rand_b) * rand_a))
        self.move_robot(self.CRUISING_SPEED)
            
        
    def light_follower_behaviour(self):
        """
            Use two light sensors to set pivot rotation to drive the robot towards the goal (light)
        """                
        #NOTE: Light sensor max value: 1024
    
        # Read light sensor values
        left_value = self.light_sensors[0].getValue()
        right_value = self.light_sensors[1].getValue()
        
        #print("Left:",left_value)
        #print("Right:",right_value)
        #print(light_threshold)
        
        # Update LED colors depending to the light value
        if(left_value > light_threshold):
            self.leds[0].set(LED_COLORS.get("green")) # Set left light sensor LED color GREEN
        else:
            self.leds[0].set(LED_COLORS.get("red")) # Set left light sensor LED color RED   
        
        if(right_value > light_threshold):
            self.leds[1].set(LED_COLORS.get("green")) # Set right light sensor LED color GREEN
        else:
            self.leds[1].set(LED_COLORS.get("red")) # Set right light sensor LED color RED   

        
        # CHECK IF LIGHT IS DETECTED
        if (left_value <= light_threshold or right_value <= light_threshold):
            return False
        
        # CHECK IF GOAL LIGHT IS REACHED
        if (left_value >= (MAX_LIGHT-10.0) and right_value >= (MAX_LIGHT-10.0)):
            # GOAL ACHIEVED
            print("--GOAL REACHED--")
            
            self.move_robot(0.0) # Stop robot
            self.goal = True
            
        else:
            print("LIGHT FOLLOWER BEHAVIOUR")
            
            # Compute delta (difference of sensors with respect to the light)
            delta = left_value - right_value
            #print(delta)
        
            self.move_robot(self.CRUISING_SPEED)  # Move forward
            
            # Adjust robot's behavior based on sensor readings (delta)
            self.rotate_robot(max(-self.MAX_ROTATION, min(-0.2 * delta, self.MAX_ROTATION)))  # Rotate pivot to align with the brighter side
            
        return True

        
    def label_distance_range(self, distance):
        """
            Label the distance to an obstacle according to the robot's distance sensor values.
        """
        if distance >= obstacle_threshold:
            label = "NOT DETECTED"
        elif close_obstacle_threshold <= distance < obstacle_threshold:
            label = "FAIR DISTANCE"
        else:
            label = "CLOSE DISTANCE"
    
        return label
    
    def obstacle_avoidance_behaviour(self):
        """
            Use two distance sensors to avoid obstacles.
            Updates distance sensor color LEDs.
        """
        #NOTE: Distance sensor max value: 128
    
        # Read distance sensor values
        left_value = self.distance_sensors[0].getValue()
        right_value = self.distance_sensors[1].getValue()
        
        # Get distance label 
        left_range = self.label_distance_range(left_value)
        right_range = self.label_distance_range(right_value)
        
        # Update LED colors depending to the computed distance range (label)
        if(left_range != "NOT DETECTED"):
            self.leds[2].set(LED_COLORS.get("green")) # Set left distance sensor LED color GREEN
        else:
            self.leds[2].set(LED_COLORS.get("red")) # Set left distance sensor LED color RED   
        
        if(right_range != "NOT DETECTED"):
            self.leds[3].set(LED_COLORS.get("green")) # Set right distance sensor LED color GREEN
        else:
            self.leds[3].set(LED_COLORS.get("red")) # Set right distance sensor LED color RED   

                          
        # Obstacle only found by right distance sensor
        if(left_range == "NOT DETECTED" and right_range == "FAIR DISTANCE"):
            self.move_robot(self.CRUISING_SPEED)  # Move forward
            self.rotate_robot(-1.5) # Rotate left
            
            self.robot.step(500)  # Move for 0.5 seconds
            
        # Obstacle only found by left distance sensor or by both of them
        elif(left_range == "FAIR DISTANCE" and right_range in ["NOT DETECTED", "FAIR DISTANCE"]):
            self.move_robot(self.CRUISING_SPEED)  # Move forward
            self.rotate_robot(1.5) # Rotate right
                       
            self.robot.step(500)  # Move for 0.5 seconds
            
        # Obstacle found too close by left distance sensor
        elif(left_range == "CLOSE DISTANCE"):
            self.move_robot(-self.CRUISING_SPEED)  # Move backward 
            self.rotate_robot(-1.5) # Rotate left
                        
            self.robot.step(1500)  # Move for 1.5 seconds
            
        # Obstacle found too close by right distance sensor
        elif(right_range == "CLOSE DISTANCE"):
            self.move_robot(-self.CRUISING_SPEED)  # Move backward 
            self.rotate_robot(1.5) # Rotate right
            
            self.robot.step(1500)  # Move for 1.5 seconds
            
        else:
            return False

        print("OBSTACLE AVOIDANCE")
        return True
        
        
        
    def revert_handler_behaviour(self):
        """
            Use an accelerometer to detect the robot's current orientation and update the navigation system orientation
            Measure the gravity acceleration. If gravity is negative, the robot is inverted.
        """
        #print("REVERT DETECTION")
        gravity = self.accelerometer.getValues()[0]
        
        #print(gravity)
        
        if(gravity < 0):
            self.inverted = 1
        else:
            self.inverted = -1
            
    
    def stuck_handler_behaviour(self):
        """
            Use a GPS to detect whether the robot is stuck and try to get unstuck 
        """
        #print("STUCK DETECTION")
        
        # Get the current position data from the GPS sensor
        position = self.gps.getValues()
            
        # Calculate the change in position
        delta_position = sum([abs(pos - prev_pos) for pos, prev_pos in zip(position, self.prev_position)])
        self.prev_position = position  # Update previous position data
        
        #print(delta_position)
        
        # Check if the robot has got stuck
        if delta_position < stuck_threshold:
            # Check if the time stuck exceeds the maximum
            if (self.robot.getTime() - self.stuck_timer) > max_stuck_time:
                # Perform recovery routine
                print("Robot is stuck. Implementing recovery...")
                
                # Perform a maneuver to get unstuck
                # Moving backward with a random angle
                self.move_robot(-self.CRUISING_SPEED)
                self.rotate_robot(random.uniform(-self.MAX_ROTATION, self.MAX_ROTATION))
                self.twister_motor.setPosition((random.uniform(-self.MAX_ROTATION*0.6, self.MAX_ROTATION*0.6)))
                self.robot.step(2500)  # Move backward for 2.5 seconds
                
                # Reset the timer after recovery
                self.stuck_timer = self.robot.getTime()
                
                return True
    
        else:
            self.stuck_timer = self.robot.getTime()  # Reset the timer if the robot is not stuck
            self.twister_motor.setPosition(0.0)
    
        return False
               
    def stuck_handler_behaviour_gyro(self):
        """
            Use a gyroscope to detect whether the robot is stuck and try to get unstuck.
            NOTE: Deprecated since it has problems detecting movement when the robot moves straight
        """
        #print("STUCK DETECTION")
        gyro_values = self.gyro.getValues()
        
        angular_velocity_magnitude = (abs(gyro_values[0])**2 + abs(gyro_values[1])**2
            + abs(gyro_values[2])**2)**0.5
        
        #print(angular_velocity_magnitude)
        
        # The robot has gotten stuck
        if(angular_velocity_magnitude <= stuck_threshold):
            self.stuck_timer += TIME_STEP / 1000  # Convert to seconds
            
            if self.stuck_timer > max_stuck_time:
                # Perform recovery routine
                print("Robot is stuck. Implementing recovery...")
                
                # Perform a maneuver to get unstuck
                # Moving backward with a random angle
                self.move_robot(-self.CRUISING_SPEED)
                self.rotate_robot(random.uniform(-self.MAX_ROTATION, self.MAX_ROTATION))
                self.twister_motor.setPosition((random.uniform(-self.MAX_ROTATION, self.MAX_ROTATION)))
                self.robot.step(2500)  # Move backward for 2.5 seconds
                
                # Reset the timer after recovery
                self.stuck_timer = 0
                
                return True
    
        else:
            self.stuck_timer = 0  # Reset the timer if the robot is not stuck
            self.twister_motor.setPosition(0.0)
    
        return False
  
  
    ################################
    #                              #
    #        ROBOT FUNCTIONS       #
    #                              #
    ################################      
    def get_time(self):
        return self.robot.getTime()
        
    def step(self, timestep):
        return self.robot.step(timestep)
            
 


if __name__ == '__main__':
    robot = ObstacleRobot()
    
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller or the goal is reached
    while robot.step(TIME_STEP) != -1 and robot.goal_reached() != 1:
        
        # SUBSUMPTION ARCHITECTURE
        robot.revert_handler_behaviour()
        
        if not(robot.stuck_handler_behaviour()):
            if not(robot.obstacle_avoidance_behaviour()):
                if not(robot.light_follower_behaviour()):
                    robot.goal_seeking_behaviour()
