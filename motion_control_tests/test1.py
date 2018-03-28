import pigpio
import time
import ctypes
import os
import math


#TODO: Verify the validity of the final cumulative displacements. It seems they
# could be off by about a factor of 1/2, but more investigation is needed (i.e.,
# the kinematics / equations of motion could be examined more closely)

lib = ctypes.cdll.LoadLibrary(os.path.abspath('../libservofeedback.so'))
lib.init()
lib.servo_angle_ne.restype = ctypes.c_double
lib.servo_angle_nw.restype = ctypes.c_double
lib.servo_angle_se.restype = ctypes.c_double
lib.servo_angle_sw.restype = ctypes.c_double
pi = pigpio.pi()

# Servo signal pins
ne = 4 # northeast
nw = 17 # northwest
sw = 22 # southwest
se = 10 # southeast

# Set PWM frequencies to 50 (shouldn't change)
pi.set_PWM_frequency(ne, 50)
pi.set_PWM_frequency(nw, 50)
pi.set_PWM_frequency(se, 50)
pi.set_PWM_frequency(sw, 50)

def main():

    # Set pulsewidths, which controls each servo's velocity
    # 1500 is neutral, 1300 is max clockwise, 1700 is max counterclockwise
    pi.set_servo_pulsewidth(ne, 1300)
    pi.set_servo_pulsewidth(nw, 1700)
    pi.set_servo_pulsewidth(se, 1300)
    pi.set_servo_pulsewidth(sw, 1700)
    t_elapsed = 0
    t_start = time.time()

    # Get initial angular positions of each servo:
    prev_angles = get_angle_position_feedback()
    #prev_ne_angle = lib.servo_angle_ne()
    #prev_nw_angle = lib.servo_angle_nw()
    #prev_se_angle = lib.servo_angle_se()
    #prev_sw_angle = lib.servo_angle_sw()

    cumulative_disp = [[0, 0],
                       [0, 0],
                       [0, 0],
                       [0, 0]]

    # Drive for 3 seconds
    while (t_elapsed < 3):
        angles = get_angle_position_feedback()
        delta_angles = []

        # Compute difference in revolutions:
        for i in range(len(angles)):
            cur_angle = angles[i]
            prev_angle = prev_angles[i]
            delta_angle = cur_angle - prev_angle

            # Check if the feedback values looped (this could perhaps be
            # improved to be more reliable. Could perhap look at servo drive
            # commands, or could look at magnitude of the difference between
            # prev and current angles, for example)
            if (prev_angle > 0.9) and (cur_angle < 0.1) and (delta_angle < 0):
                # Wheel is estimated to be going clockwise, crossed over a loop
                delta_angle = cur_angle + (1 - prev_angle)
            elif ((prev_angle < 0.1) and (cur_angle > 0.9) and
                 (delta_angle > 0)):
                # Wheel is estimated to be going counterclockwise, crossed over
                # a loop
                delta_angle = -(prev_angle + (1 - cur_angle))
            delta_angles.append(delta_angle)

        displacements = compute_linear_displacements(delta_angles)
        #print_displacements(displacements)

        # Add displacements to cumulative displacements
        for i in range(len(displacements)):
            cumulative_disp[i][0] += displacements[i][0]
            cumulative_disp[i][1] += displacements[i][1]
        if t_elapsed == 0:
            print_displacements(cumulative_disp)
        #print 'SW: ' + str(sw_angle)
        #print 'SE: ' + str(se_angle)
        #print 'NE: ' + str(ne_angle)
        #print 'NW: ' + str(nw_angle)
        #time.sleep(0.1)
        
        prev_angles = angles
        t_elapsed = time.time() - t_start

    print_displacements(cumulative_disp)

    pi.set_servo_pulsewidth(ne, 1500)
    pi.set_servo_pulsewidth(nw, 1500)
    pi.set_servo_pulsewidth(se, 1500)
    pi.set_servo_pulsewidth(sw, 1500)
    pi.stop()
    lib.terminate()

def compute_linear_displacements(delta_angles):
    # Direct the x and y components of the wheel's linear displacement based on
    # the geometry of each servo
    direction_vectors = [[-1, 1], # northeast
                         [-1, -1], # northwest
                         [1, 1], # southeast
                         [1, -1]] # southwest
    displacements = []
    for num, angle in enumerate(delta_angles):
        # Magnitude of displacement in vehicle's xy-coordinate system is the
        # same in both directions, as each wheels is positioned 45 degrees
        # relative to each axis.
        # Uses the circular motion equation s=r*theta, where theta is in
        # radians.
        displacement_component = (38*math.pi)/math.sqrt(2) * angle # in mm
        # Get x and y components by multiplying by the appropriate director
        displacement = [displacement_component*direction_vectors[num][0],
                        displacement_component*direction_vectors[num][1]]
        displacements.append(displacement)
    return displacements

def print_displacements(displacements):
    print '--------------'
    print 'Displacements:'
    disp_format = '\t{0}: ({1}, {2})'
    print disp_format.format('NE', displacements[0][0], displacements[0][1])
    print disp_format.format('NW', displacements[1][0], displacements[1][1])
    print disp_format.format('SE', displacements[2][0], displacements[2][1])
    print disp_format.format('SW', displacements[3][0], displacements[3][1])

def get_angle_position_feedback():
    return [lib.servo_angle_ne(),
            lib.servo_angle_nw(),
            lib.servo_angle_se(),
            lib.servo_angle_sw()]

main()
