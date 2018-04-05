#import threading
#import Queue
import time
import datetime
import pigpio
import ctypes
import os
import math
import numpy as np
import subprocess
import RPi.GPIO as GPIO

from lidar_reader import LidarReader
from servo_handler import ServoHandler
from servo_feedback_reader import ServoFeedbackReader

# To access Raspberry Pi's GPIO pins:
#import RPi.GPIO as GPIO

class Robot(object):

    def __init__(self):
        # Run the pigpio daemon
        #os.system('sudo pigpiod') # TODO: Make sure this actually works
        #subprocess.call(['sudo', 'pigpiod'])
        
        GPIO.setmode(GPIO.BCM)     # Number GPIOs by channelID
        GPIO.setwarnings(False)    # Ignore Errors
        
        # Setup all GPIO pins as low.
        GPIO.setup(6, GPIO.OUT)     # GPIO_06 = Lidar0
        GPIO.setup(13, GPIO.OUT)     # GPIO_13 = Lidar1
        GPIO.setup(19, GPIO.OUT)     # GPIO_19 = Lidar2
        GPIO.setup(26, GPIO.OUT)     # GPIO_26 = Lidar3
        
        # Chip enable
        GPIO.output(6, 1)
        GPIO.output(13, 1)
        GPIO.output(19, 1)
        GPIO.output(26, 1)
        
        self.lib = ctypes.cdll.LoadLibrary(os.path.abspath('/home/pi/A-Maze/'
                                                        'libsensordata.so'))
        self.lib.init()
        self.pi = pigpio.pi() # binds to port 8888 by default
        # For servo feedback data: set the return types to double
        self.lib.servo_angle_ne.restype = ctypes.c_double
        self.lib.servo_angle_nw.restype = ctypes.c_double
        self.lib.servo_angle_se.restype = ctypes.c_double
        self.lib.servo_angle_sw.restype = ctypes.c_double
        
        # For lidar mm range data: set the return types to int
        self.lib.lidar_distance_north.restype = ctypes.c_int
        self.lib.lidar_distance_south.restype = ctypes.c_int
        self.lib.lidar_distance_east.restype = ctypes.c_int
        self.lib.lidar_distance_west.restype = ctypes.c_int
        
        self.servo_handler = None
        self.initialize_lidars()
        self.initialize_servos()
        self.lidar_queue = self.lreader.lidar_queue
        self.servo_feedback_queue = self.servo_feedback_reader.servo_feedback_queue
        # At a distance of 60 mm, robot is close to wall
        self.near_wall_thresh = 60
        # NOTE: If the robot is perfectly centered and oriented straight in a
        #       square, then a lidar's distance to a corresponding neighboring
        #       wall is about 42 mm (calculation based on 3D CAD model
        #       dimensions and roughly verified by observation of lidar readout)
        # NOTE: Upper bound of servo range is about 100 mm
        self.motion_list = [self.servo_handler.move_north,
                            self.servo_handler.move_south,
                            self.servo_handler.move_east,
                            self.servo_handler.move_west]
        self.robot_radius = 62.0 # mm. Radius from geometric center of robot's
                               # base to center of each omniwheel
        self.omniwheel_radius = 19.0 # mm

    # TODO: theta (heading) should perhaps be accounted for. Currently set to a
    #       default value of 0.
    def move(self, u, v, t, theta=0):
        pass
        
    def initialize_lidars(self):
        # Create a LidarReader object
        self.lreader = LidarReader(self.lib)
        # Call the LidarReader object's start() method, starting its thread
        self.lreader.start()
        print 'LidarReader thread started'
        
    def initialize_servos(self):
        self.servo_handler = ServoHandler(self.lib, self.pi)
        self.servo_feedback_reader = ServoFeedbackReader(self.servo_handler)
        # Start the servo feedback reader's thread
        self.servo_feedback_reader.start()
        print 'Servo feedback thread started'
    
    def mainloop(self):
        
        # Manually test rotation of servo
        # while True:
        #     print self.servo_handler.get_angle_position_feedback()[0]
        
        print 'Start of main loop'
        t_elapsed = 0
        t_start = time.time()
        cur_time = t_start
        
        # Get initial angular positions of each servo:
        prev_angles = self.servo_handler.get_angle_position_feedback()
                
        # Initial pose and velocity and time
        prev_pose = PoseVelTimestamp(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, t_start)
        
        self.servo_handler.move_north()
        
                
        # # Test approximately 1 revolution of the northeast omniwheel
        # start_angle = self.servo_handler.get_angle_position_feedback()[0]
        # cur_angle = start_angle
        # while True:
        #     cur_angle = self.servo_handler.get_angle_position_feedback()[0]
        #     if cur_angle < start_angle and cur_angle > (start_angle - 0.01):
        #         return

        self.prev_move_dir = None
        
        # Indicates absence of lidar data at any given loop
        lidar_data = None
        # Indicates absence of servo position feedback data at any given loop
        feedback_data = None
        
        # while True:

        #while (t_elapsed < 1.5):
        while (abs(prev_pose.hdg) < math.radians(45)):
            cur_time = time.time() # TODO: @ CONSIDER TESTING

        # TODO: filter heading values to not fluctuate so quickly based on a
        #       single instance of servo position feedback data?
        # while (abs(prev_pose.hdg) < math.radians(45)):
            # lidar_data = self.lidar_queue.get(block=True)
            # Try to read Lidar data, if new data has come in
            if not self.lidar_queue.empty():
                lidar_data = self.lidar_queue.get()
                print '------------------'
                print 'LIDAR:'
                print lidar_data.to_string()
                
            # TODO: Work on this angle position feedback getting. Perhaps there
            # is a notable disparity between servos' feedback frequencies, which
            # would probably need to be accounted for in a more sophisticated
            # manner...
            #time.sleep(0.1)
            if not self.servo_feedback_queue.empty():
                feedback_data = self.servo_feedback_queue.get()
                #print '------------------'
                #print 'SERVO FEEDBACK:'
                #print feedback_data.to_string()
                
                angles = [feedback_data.ne_angle,
                          feedback_data.nw_angle,
                          feedback_data.se_angle,
                          feedback_data.sw_angle]
            
            
                delta_angles = self.servo_handler.get_delta_angles(prev_angles, # TODO: @ CONSIDER TESTING
                                                                    angles)
            
                #print 'T DIFF (period): '
                #print time.time() - cur_time
                cur_pose = self.compute_cur_pose(prev_pose, delta_angles, cur_time)
                prev_pose = cur_pose
                
                #print cur_pose.to_string()
            
                prev_angles = angles
                
            lidar_data = None
            feedback_data = None
            # Exit condition
            t_elapsed = cur_time - t_start
            
            
            
            # TODO: should heading be error-corrected in a PID control loop?
            # Ideally, we want to keep a constant heading and simply translate
            # the robot
                        
            # Make the robot move based on lidar data and servo position
            # feedback
            # TODO: Implement servo position feedback
            # TODO: This is a very naive test of movement based on sensor data
            # if lidar_data is not None:
            #     self.move_to_open_space(lidar_data) # TODO

            
        self.servo_handler.stop_all()
            
    def compute_cur_pose(self, prev_pose, delta_angles, t):
        '''
        Input:
            - prev_pose: a Pose object that is the robot's previous pose in the
                         global coordinate frame
            - delta_angles: a list of 4 angle measurements (angle measured from
                            0 to 1) that represent are the change in angular
                            position of each servo as a result of rotation
            - t: current timestamp (seconds since the epoch)
        Output:
            - the current pose after applying the rotation and translation
              given by the servo position feedback data
               - or None if angular velocities do not pass the filter
        
        Steps to compute current pose:
            1. Compute average angular velocity of each omniwheel over the time
               interval delta_t
            2. Use the derived rigid-body equations that describe this system.
               These solution equations include three equations of interest:
                    1) V_er = self.omniwheel_radius/math.sqrt(2) * (w_A - w_B),
                        where V_er is the component of the robot's velocity in
                        the e_r (forward) direction (local robot coordinates),
                        and where w_A, for example, is the angular velocity of
                        omniwheel A (northeast omniwheel)
                    2) V_eT = self.omniwheel_radius/math.sqrt(2) * (w_C - w_B),
                              where V_eT is the component
                              of the robot's velocity in the e_T (sideways)
                              direction (local robot coordinates)
                    3) w_v = (-self.omniwheel_radius*(w_A + w_C))/(2*self.robot_radius),
                        where w_v is the angular velocity of the robot
            3. Multiply w_v by delta_t to obtain an estimate of the change in
               heading of the robot over the time interval delta_t
            4. Use a rotation matrix/tensor with theta equal to previous heading
               + the change in heading over duration of delta_t. Multiply the
               rotation matrix with the {V_er; V_eT} vector to obtain average
               velocity in global frame over time interval delta_t.
                    - Use a linear interpolation of delta_theta (change in
                      heading) to estimate average heading as prev_heading +
                      (delta_theta/2). This is with w_v approximated as constant
                      over delta_t
            5. Multiply average velocity in global frame by delta_t to get
               displacement in global frame
        
        Eventually can also incorporate lidar data to correct these poses,
        perhaps. It is likely that this pose calculation will be prone to
        accumulation/estimation errors.
        '''

        # Change in time between two pose calculations
        delta_t = t - prev_pose.t
        #print 'Delta t: ' + str(delta_t)
        
        # 1. Omniwheel angular velocities (signed magnitudes indicating rotation
        #    direction about k axis)
        angular_vels = self.compute_omniwheel_angular_vels(delta_angles,
                                                           delta_t)
        #print angular_vels
        
        # (Pretend the w's are omegas for angular velocity. Negate these to
        # align with CCW=positive mathematical convention)
        w_A = -angular_vels[0] # average angular velocity of NE omniwheel
        w_B = -angular_vels[1] # average angular velocity of NW omniwheel
        w_C = -angular_vels[3] # average angular velocity of SW omniwheel
        w_D = -angular_vels[2] # average angular velocity of SE omniwheel
        # Note that fourth servo (e.g., SE, the one labeled D) makes rigid-body
        # system overconstrained.
        
        #angular_vels_alphabetical = [w_A, w_B, w_C, w_D]
        #prev_angular_vels = [prev_pose.w_A, prev_pose.w_B, prev_pose.w_C, prev_pose.w_D]
        # TODO: Consider implementing a more robust filter such as a Kalman
        #       filter
        # omegas_passed = self.filter_omegas(prev_angular_vels, angular_vels_alphabetical)
        # if not omegas_passed:
        #     return None
                                                           
        # 2. Using derived rigid-body equations
        # Velocities components of the robot in local coordinates
        #V_er = self.omniwheel_radius/math.sqrt(2) * (w_A - w_B)
        R = self.omniwheel_radius
        V_er = (math.sqrt(2)*R*w_A)/4 - (math.sqrt(2)*R*w_B)/4 - (math.sqrt(2)*R*w_C)/4 + (math.sqrt(2)*R*w_D)/4
        #V_eT = self.omniwheel_radius/math.sqrt(2) * (w_A/2 - w_B + w_C/2)
        #V_eT = self.omniwheel_radius/math.sqrt(2) * (w_C - w_B)
        V_eT = (math.sqrt(2)*R*w_C)/4 - (math.sqrt(2)*R*w_B)/4 - (math.sqrt(2)*R*w_A)/4 + (math.sqrt(2)*R*w_D)/4
        
        # Angular velocity of entire robot
        # w_v = ((self.omniwheel_radius/math.sqrt(2) * (-w_A/2 - w_C/2))/
        #             self.robot_radius)
        #w_v = (-self.omniwheel_radius*(w_A + w_C))/(2*self.robot_radius)
        R_v = self.robot_radius
        # Does this following equation just do an average like we originally had
        # assumed by intuition / simple analysis? Maybe with extra factors?
        w_v = -(R*w_A)/(4*R_v) - (R*w_B)/(4*R_v) - (R*w_C)/(4*R_v) - (R*w_D)/(4*R_v)
                
        # 3. Estimate of the change in heading over time interval delta_t
        #print '###### w_v: ' + str(w_v)
        robot_delta_hdg = w_v * delta_t
        
        # 4. Convert from local to global coordinates with a rotation matrix
        # Average heading estimate
        #robot_average_hdg = prev_pose.hdg + (robot_delta_hdg/2)
        robot_average_hdg = prev_pose.hdg + robot_delta_hdg
        
        # Apply filter to raw heading value (robot_average_hdg)
        # TODO

        # TODO: Do this coordinate frame transformation correctly...
        average_vel_robot_frame = np.matrix(
                        [[V_eT], # like i component in local coords.
                         [V_er]]) # like j component in local coords.
        #print average_vel_robot_frame
                                                     
        # Convert from local robot coordinates to world coordinates by
        # multiplying a rotation matrix by the robot's velocity in the local
        # frame
        # TODO: Do this coordinate frame transformation correctly...
        average_vel_world_frame = self.apply_rotation_matrix(
                                            average_vel_robot_frame,
                                            robot_average_hdg)
        # 5. Multiply average velocity in global frame by delta_t to get
        #    displacement in global frame
        # Displacement in global frame (s = v_(avg) * t)
        displacement_world_frame = [average_vel_world_frame[0] * delta_t,
                                    average_vel_world_frame[1] * delta_t]
        return PoseVelTimestamp(prev_pose.x + displacement_world_frame[0],
                                prev_pose.y + displacement_world_frame[1],
                                average_vel_world_frame[0],
                                average_vel_world_frame[1],
                                robot_average_hdg,
                                w_v,
                                w_A,
                                w_B,
                                w_C,
                                w_D,
                                t)
                                
    def filter_omegas(self, prev_angular_vels, cur_anguler_vels):
        for i in range(len(cur_anguler_vels)):
            if abs(cur_anguler_vels[i] - prev_angular_vels[i]) > 2:
                return False
        return True

    def compute_omniwheel_angular_vels(self, delta_angles, delta_t):
        angular_vels = []
        for angle in delta_angles:
            theta = (2 * math.pi * angle)
            # Omega is rate of change of theta. Get an average value by dividing
            # by delta_t
            omega = theta/delta_t
            # Note that clockwise rotation of an omniwheel corresponds to a
            # positive value of omega
            angular_vels.append(omega)
        return angular_vels
        
    def apply_rotation_matrix(self, original_matrix, hdg):
        rotation_mat = np.matrix([[math.cos(hdg), -math.sin(hdg)],
                                  [math.sin(hdg), math.cos(hdg)]])
        return rotation_mat*original_matrix
            
    # TODO: Use a better algorithm for movement. This is just for basic testing
    # of responsiveness to lidar data
    def move_to_open_space(self, lidar_data):
        data = [lidar_data.north_dist, lidar_data.south_dist,
                lidar_data.east_dist, lidar_data.west_dist]
        room_to_move = []
        num_move_options = 0
        for dist in data:
            if dist > self.near_wall_thresh:
                room_to_move.append(True)
                num_move_options += 1
            else:
                room_to_move.append(False)
                
        if self.prev_move_dir is None:
            # Set initial motion
            self.prev_move_dir = room_to_move.index(True)
            self.move_by_lidar_wall_avoidance(self.prev_move_dir)
            return

        if num_move_options == 1:
            self.move_dir = room_to_move.index(True)
        elif num_move_options > 1:
            # There are multiple feasible directions
            # If the current direction is not a feasible direction, then go in
            # the first free one.
            if not room_to_move[self.prev_move_dir]:
                # Go in the first free one
                self.move_dir = room_to_move.index(True)
                
        if self.prev_move_dir != self.move_dir:
            # Only send a servo signal if the movement directions are different
            self.move_by_lidar_wall_avoidance(self.move_dir)
        self.prev_move_dir = self.move_dir

            
    def move_by_lidar_wall_avoidance(self, direction):
        # Based on lidar closeness
        # direction is an integer indicating which of the 4 cardinal directions
        # to move in. 0 = north, 1 = south, 2 = east, 3 = west
        self.motion_list[direction]()
        
    #def is_about_to_hit_wall(self)
        
    def shutdown(self):
        # TODO: Add things to this?
        if self.servo_handler is not None:
            self.servo_handler.close_handler()
            

# (Really just a vehicle state class...)
# Class that represents the robot's pose in the global/world coordinate frame:
#   - x-coordinate (mm) in global frame
#   - y-coordinate (mm) in global frame
#   - u: velocity in x direction (mm/s) in global frame
#   - v: velocity in y direction (mm/s) in global frame
#   - heading (radians) in global frame
#   - w_v: angular velocity of robot (rad/s)
#   - w_A = average angular velocity of NE omniwheel
#   - w_B = average angular velocity of NW omniwheel
#   - w_C = average angular velocity of SW omniwheel
#   - w_D = average angular velocity of SE omniwheel
class PoseVelTimestamp(object):
    def __init__(self, x, y, u, v, hdg, w_v, w_A, w_B, w_C, w_D, t):
        self.x = x
        self.y = y
        self.u = u
        self.v = v
        self.hdg = hdg
        self.w_v = w_v
        self.w_A = w_A
        self.w_B = w_B
        self.w_C = w_C
        self.w_D = w_D
        self.t = t
        
        self.heading_deg = math.degrees(self.hdg)
        self.w_v_deg = math.degrees(self.w_v)
        self.w_A_deg = math.degrees(self.w_A)
        self.w_B_deg = math.degrees(self.w_B)
        self.w_C_deg = math.degrees(self.w_C)
        self.w_D_deg = math.degrees(self.w_D)

        
    def to_string(self):
        return_str = '----------\n'
        return_str += 'x: {0}\n'
        return_str += 'y: {1}\n'
        return_str += 'u: {2}\n'
        return_str += 'v: {3}\n'
        return_str += 'heading (deg): {4}\n'
        return_str += 'w_v (deg/s): {5}\n'
        return_str += 'w_A (deg/s): {6}\n'
        return_str += 'w_B (deg/s): {7}\n'
        return_str += 'w_C (deg/s): {8}\n'
        return_str += 'w_D (deg/s): {9}\n'
        return_str += 't (sec): {10}\n'
        return_str += '----------'
        return (return_str.format(self.x, self.y,
                                  self.u, self.v,
                                  self.heading_deg, self.w_v_deg,
                                  self.w_A_deg, self.w_B_deg,
                                  self.w_C_deg, self.w_D_deg,
                                  self.t))
                
        
if __name__ == '__main__':
    robot = Robot()
    try:
        robot.mainloop()
    except KeyboardInterrupt as e:
        print e
        robot.shutdown()
    finally:
        print 'Terminating robot.py program'
        GPIO.cleanup()
        #subprocess.call(['sudo', 'killall', 'pigpiod'])
        robot.shutdown()
        print 'FINALLY'
