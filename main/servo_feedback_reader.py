import threading
import Queue
import time


class ServoFeedbackReader(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self, servo_handler)
        self.daemon = True
        self.servo_handler = servo_handler
        self.servo_feedback_queue = Queue.Queue()
        self.prev_feedback_data = None
                                                   
    def get_data(self):
        
        if self.prev_feedback_data is None:
            angles = self.servo_handler.get_angle_position_feedback()
            cur_time = time.time()
            ne_time = cur_time
            nw_time = cur_time
            se_time = cur_time
            sw_time = cur_time
            
            ne_angle = angles[0]
            nw_angle = angles[1]
            se_angle = angles[2]
            sw_angle = angles[3]
            
        else:
            # Compare new data to previous data
            # prev_angles = [self.prev_feedback_data.ne_angle,
            #                self.prev_feedback_data.nw_angle,
            #                self.prev_feedback_data.se_angle,
            #                self.prev_feedback_data.sw_angle]
            # got_all_new_angles = False
            # while not got_all_new_angles:
            #     angles = self.servo_handler.get_angle_position_feedback()
            #     cur_time = time.time()
            #     for i in range(len(angles)):
            #         if angles[i] == prev_angles[i]:
            #             continue
            #     got_all_new_angles = True
            
            prev_angles = [self.prev_feedback_data.ne_angle,
                           self.prev_feedback_data.nw_angle,
                           self.prev_feedback_data.se_angle,
                           self.prev_feedback_data.sw_angle]
            new_angles = [None, None, None, None]
            #got_new_angles_list = [False, False, False, False]
            #while got_new_angles_list.count(True) < 4:
            while None in new_angles:
                # While at least one of the servos did not receive a new
                # position feedback angle
                angles = self.servo_handler.get_angle_position_feedback()
                cur_time = time.time()
                for i in range(len(angles)):
                    if angles[i] != prev_angles[i]:
                        # Got new feedback data
                        new_angles[i] = (cur_time, angles[i])
                        #got_new_angles_list[i] = True
            ne_time, ne_angle = new_angles[0]
            nw_time, nw_angle = new_angles[1]
            se_time, se_angle = new_angles[2]
            sw_time, sw_angle = new_angles[3]
        
        feedback_data = FeedbackData(ne_angle, ne_time,
                                     nw_angle, nw_time,
                                     se_angle, se_time,
                                     sw_angle, sw_time)
        
        self.servo_feedback_queue.put(feedback_data)
        prev_feedback_data = feedback_data
        
    def run(self):
        # Run the thread
        while True:
            self.get_data()
            
    
# Class that stores an instance of servo feedback data
class FeedbackData(object):
    def __init__(self, ne_angle, ne_time,
                       nw_angle, nw_time,
                       se_angle, se_time,
                       sw_angle, sw_time):
        self.ne_angle = ne_angle
        self.ne_time = ne_time
        self.nw_angle = nw_angle
        self.nw_time = nw_time
        self.se_angle = se_angle
        self.se_time = se_time
        self.sw_angle = sw_angle
        self.sw_time = sw_time
        
    def to_string(self):
        return_format = '-----\n{dir}\n\tangle (0-1): {angle}\n\tTime (s): {t}\n'
        return_string = return_format.format(dir='Northeast', angle=self.ne_angle,
                                             t=self.ne_time)
        return_string += return_format.format(dir='Northwest', angle=self.nw_angle,
                                             t=self.nw_time)
        return_string += return_format.format(dir='Southeast', angle=self.se_angle,
                                             t=self.se_time)
        return_string += return_format.format(dir='Southwest', angle=self.sw_angle,
                                             t=self.sw_time)
        return return_string

