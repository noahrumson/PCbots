#import threading
#import Queue
import time
import datetime
import pigpio
import ctypes
import os
import math
import numpy as np

from lidar_reader import LidarReader
from servo_handler import ServoHandler

# To access Raspberry Pi's GPIO pins:
#import RPi.GPIO as GPIO

class Robot(object):

    def __init__(self):
        # Run the pigpio daemon
        os.system('sudo pigpiod') # TODO: Make sure this actually works
        
        self.servo_handler = None
        self.initialize_lidars()
        self.initialize_servos()
        self.lidar_queue = self.lreader.lidar_queue
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
        self.robot_radius = 62 # mm. Radius from geometric center of robot's
                               # base to center of each omniwheel

    # TODO: theta (heading) should perhaps be accounted for. Currently set to a
    #       default value of 0.
    def move(self, u, v, t, theta=0):
        pass
        
    def initialize_lidars(self):
        # Create a LidarReader object
        self.lreader = LidarReader()
        # Call the LidarReader object's start() method, starting its thread
        self.lreader.start()
        
    def initialize_servos(self):
        self.servo_handler = ServoHandler()
    
    def mainloop(self):
        print 'Start of main loop'
        t_elapsed = 0
        t_start = time.time()

        # Get initial angular positions of each servo:
        prev_angles = self.servo_handler.get_angle_position_feedback()
        
        # Initial pose and velocity and time
        prev_pose = PoseVelTimestamp(0, 0, 0, 0, 0, t_start)
        
        self.servo_handler.move_north()
        
        self.prev_move_dir = None
        
        lidar_data = None # indicates absence of lidar data at any given loop
        # while True:
        # while (t_elapsed < 4):
        while (abs(prev_pose.hdg) < math.radians(45)):
            # lidar_data = self.lidar_queue.get(block=True)
            # Try to read Lidar data, if new data has come in
            if not self.lidar_queue.empty():
                lidar_data = self.lidar_queue.get()
                print '------------------'
                print 'LIDAR:'
                print lidar_data.to_string()
            # TODO: Consider getting angle position feedback in a separate
            # thread, especially if we are waiting to process the data until we
            # get a new position feedback from each servo...
            angles = self.servo_handler.get_angle_position_feedback()
            # angles = self.get_new_angle_position_feedbacks(prev_angles)
            
            # TODO: Work on this angle position feedback getting. Perhaps there
            # is a notable disparity between servos' feedback frequencies, which
            # would probably need to be accounted for in a more sophisticated
            # manner...
            delta_angles = [0.0, 0.0, 0.0, 0.0]
            while 0.0 in delta_angles:
                angles = self.servo_handler.get_angle_position_feedback()
                delta_angles = self.servo_handler.get_delta_angles(prev_angles,
                                                                   angles)
            cur_time = time.time()
            cur_pose = self.compute_cur_pose(prev_pose, delta_angles, cur_time)
            print cur_pose.to_string()
            
            # TODO: should heading be error-corrected in a PID control loop?
            # Ideally, we want to keep a constant heading and simply translate
            # the robot
                        
            # Make the robot move based on lidar data and servo position
            # feedback
            # TODO: Implement servo position feedback
            # TODO: This is a very naive test of movement based on sensor data
            # if lidar_data is not None:
            #     self.move_to_open_space(lidar_data) # TODO

            prev_angles = angles
            prev_pose = cur_pose
            lidar_data = None
            t_elapsed = cur_time - t_start
            
        self.servo_handler.stop_all()
        
    def get_new_angle_position_feedbacks(self, prev_angles):
        '''
        Return servo position feedback in packets of four. Only process
        position, etc., when we get new position feedback from each servo. Note
        that with the current implementation of this method, this only works if
        all four servos are being driven and are expected to produce new
        position feedback values. Another possible way to implement this would
        be to sleep until new values are pretty much guaranteed to appear,
        based on the frequency of the servo position feedback, which is about
        1 kHz.
        '''
        # angles = self.servo_handler.get_angle_position_feedback()
        # final_angles = [None, None, None, None]
        # got_all_new_angles = False
        # print 'Starting to get new angle position feedbacks'
        # while not got_all_new_angles:
        #     got_all_new_angles = True
        #     for num, angle in enumerate(angles):
        #         if angle != prev_angles[num]:
        #             # Got a new position feedback value
        #             final_angles[num] = angle
        #         else:
        #             got_all_new_angles = False
        # print 'Got new angle position feedbacks'
        # return final_angles
        
        time.sleep(0.05)
        return self.servo_handler.get_angle_position_feedback()
            
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
              
        Steps to compute current pose:
            1. Compute differences of tangential velocities of each omniwheel.
               Result is net tangential average velocity over time interval
               delta_t
            2. Use v=r*(omega) to get robot's average angular velocity over
               delta_t.
            3. Compute average angular displacement, i.e., a change in heading.
            4. Compute an estimate for the velocity vector of the robot's center
               of mass (assumed/estimated to be geometric center).
                - Average the four velocity vectors
                    - This gets an average velocity in robot's local frame
                - Use a rotation matrix with average heading over duration of
                  delta_t to get average velocity in global frame
                    - Use a linear interpolation of delta_theta (change in
                      heading) to estimate average heading as
                      (delta_theta/2) + prev_heading
                - Multiply average velocity in global frame by delta_t to get
                  displacement in global frame
                  
        It might be possible that we could just bypass the conversion from
        displacement to velocity back to displacement, but it is helpful for
        intuition at least.
        
        Eventually can also incorporate lidar data to correct these poses,
        perhaps. It is likely that this pose calculation will be prone to
        accumulation/estimation errors.
        '''
        
        # Change in time between two pose calculations
        delta_t = t - prev_pose.t
        
        # Omniwheel tangential velocity vectors (local coords.) and magnitudes
        # of these velocity vectors
        vels, vel_mags = self.compute_tangential_vels(delta_angles, delta_t)
        
        # Compute differences of tangential velocities of each omniwheel. These
        # magnitudes are signed, so sum them up to get net tangential velocity
        net_tangential_vel = 0
        for vel in vel_mags:
            net_tangential_vel += vel
            print vel
            
        # Compute robot's average rotational/angular velocity. Use v=r*(omega),
        # rearranged to solve for omega
        robot_omega = net_tangential_vel/self.robot_radius
        
        # Average change in heading (angular displacement)
        robot_delta_hdg = robot_omega*delta_t
        
        # Average heading estimate
        robot_average_hdg = prev_pose.hdg + (robot_delta_hdg/2)
        #robot_average_hdg = prev_pose.hdg + robot_delta_hdg
        
        # Average the four omniwheels' velocity vectors
        omniwheel_vel_sum_x = 0
        omniwheel_vel_sum_y = 0
        for vel in vels:
            omniwheel_vel_sum_x += vel[0]
            omniwheel_vel_sum_y += vel[1]
        
        # Average over 4 omniwheels
        average_vel_robot_frame = np.matrix([[float(omniwheel_vel_sum_x/4)],
                                             [float(omniwheel_vel_sum_y/4)]])
                                                     
        # Convert from local robot coordinates to world coordinates by
        # multiplying the robot's velocity in the local frame by a rotation
        # matrix
        average_vel_world_frame = self.apply_rotation_matrix(
                                            average_vel_robot_frame,
                                            robot_average_hdg)
                                            
        # Displacement in global frame (s = v_(avg) * t)
        displacement_world_frame = [average_vel_world_frame[0] * delta_t,
                                    average_vel_world_frame[1] * delta_t]
        
        return PoseVelTimestamp(prev_pose.x + displacement_world_frame[0],
                                prev_pose.y + displacement_world_frame[1],
                                average_vel_world_frame[0],
                                average_vel_world_frame[1],
                                robot_average_hdg,
                                t)

    def compute_tangential_vels(self, delta_angles, delta_t):
        # Direct the x and y components of the wheel's linear displacement based
        # on the geometry of each servo
        direction_vectors = [[-1, 1], # northeast
                             [-1, -1], # northwest
                             [1, 1], # southeast
                             [1, -1]] # southwest
        vels = []
        vel_mags = [] # just magnitudes of velocity
        for num, angle in enumerate(delta_angles):
            # Magnitude of displacement and of velocity in vehicle's
            # xy-coordinate system is the same in both directions, as each
            # wheel is positioned 45 degrees relative to each axis.
            # Uses the circular motion equation v=r*omega, where omega is
            # angular frequency in radians.
            # Radius of each omniwheel is 19 mm
            theta = (2 * math.pi * angle)
            # Omega is rate of change of theta. Get an average value by dividing
            # by delta_t.
            omega = theta/delta_t
            # Note that clockwise rotation of an omniwheel corresponds to a
            # positive value of omega
            v = 19 * omega # in mm/s
            #vel_component = v/math.sqrt(2) # in mm
            vel_component = v*math.sqrt(2) # in mm
            # Get x and y components by multiplying by the appropriate director
            vel = [vel_component*direction_vectors[num][0],
                            vel_component*direction_vectors[num][1]]
            vels.append(vel)
            vel_mags.append(v)
        return (vels, vel_mags)
        
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
            

# Class that represents the robot's pose in the global/world coordinate frame:
#   - x-coordinate (mm) in global frame
#   - y-coordinate (mm) in global frame
#   - u: velocity in x direction (mm/s) in global frame
#   - v: velocity in y direction (mm/s) in global frame
#   - heading (radians) in global frame
class PoseVelTimestamp(object):
    def __init__(self, x, y, u, v, hdg, t):
        self.x = x
        self.y = y
        self.u = u
        self.v = v
        self.hdg = hdg
        self.t = t
        
    def to_string(self):
        heading_deg = math.degrees(self.hdg)
        return_str = '----------\n'
        return_str += 'x: {}\n'
        return_str += 'y: {}\n'
        return_str += 'u: {}\n'
        return_str += 'v: {}\n'
        return_str += 'heading (deg): {}\n'
        return_str += 't (sec): {}\n'
        return_str += '----------'
        return (return_str.format(self.x, self.y,
                                  self.u, self.v,
                                  heading_deg, self.t))
                
        
if __name__ == '__main__':
    robot = Robot()
    try:
        robot.mainloop()
    except KeyboardInterrupt:
        robot.shutdown()
    finally:
        print 'Terminating robot.py program'
        robot.shutdown()
