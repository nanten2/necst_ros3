#! /usr/bin/env python3

name = 'dome_door_cmd_mapper'

# ----
import threading
import rospy
import std_msgs.msg

import topic_utils


class domne_door_cmd_sim(object):
    def __init__(self, travel_time_left, travel_time_right):
        self.sim = dome_door_simulator(travel_time_left, travel_time_right)
        
        self.topic_from1 = rospy.Subscriber(
            name = 'dome_door_cmd2',
            data_class = std_msgs.msg.String,
            callback = self.update_cmd,
            queue_size = 1,
        )

        self.pub_left = rospy.Publisher(
            name = 'dome_door_left_position',
            data_class = std_msgs.msg.String,
            latch = True,
            queue_size = 1,
        )
        
        self.pub_right = rospy.Publisher(
            name = 'dome_door_right_position',
            data_class = std_msgs.msg.String,
            latch = True,
            queue_size = 1,
        )        
        pass
    
    def update_cmd(self, msg):
        self.sim.set_command(msg.daga)
        return
    
    def publish_status(self):
        pos_left_last = self.sim.pos_left
        pos_right_last = self.sim.pos_right
        
        while not rospy.is_shutdown():
            if self.sim.pos_left != pos_left_last:
                self.pub_left.publish(self.sim.pos_left)
                pos_left_last = self.sim.pos_left
                pass
                
            if self.sim.pos_right != pos_right_last:
                self.pub_right.publish(self.sim.pos_right)
                pos_right_last = self.sim.pos_right
                pass

            time.sleep(0.05)
            continue
        
        return


class dome_door_simulator(object):
    pos_left = 0
    pos_right = 0
    
    travel_time_left = 0
    travel_time_right = 0
    velocity_left = 0
    velocity_right = 0
    
    cmd = 'CLOSE'
    current_left = 'CLOSE'
    current_right = 'CLOSE'
    
    rate = 0.05
    
    def __init__(self, travel_time_left, travel_time_right):
        tt_left = travel_time_left if travel_time_left != 0 else self.rate
        tt_right = travel_time_right if travel_time_right != 0 else self.rate
        self.travel_time_left = tt_left
        self.travel_time_right = tt_right
        self.velocity_left = 1 / tt_left * self.rate
        self.velocity_right = 1 / tt_right * self.rate

        pub_thread = threading.Thread(
            target = self.simulate,
            daemon = True,
        )
        pub_thread.start()
        pass
    
    def simulate(self):
        while not rospy.is_shutdown():
            if self.cmd == 'CLOSE':
                self.pos_left -= self.velocity_left
                self.pos_right -= self.velocity_right
            elif self.cmd == 'OPEN':
                self.pos_left += self.velocity_left
                self.pos_right += self.velocity_right
                pass

            if self.pos_left >= 1:
                self.pos_left = 1
                self.current_left = 'OPEN'
            elif self.pos_left <= 0:
                self.pos_left = 0
                self.current_left = 'CLOSE'
            else:
                self.current_left = 'MOVING'
                pass
                
            if self.pos_right >= 1:
                self.pos_right = 1
                self.current_right = 'OPEN'
            elif self.pos_right <= 0:
                self.pos_right = 0
                self.current_right = 'CLOSE'
            else:
                self.current_right = 'MOVING'
                pass

            time.sleep(self.rate)
            continue
        return

    def set_command(self, cmd):
        self.cmd = cmd
        return
        


if __name__=='__main__':
    rospy.init_node(name)
    travel_time_left = rospy.get_param('~travel_time_left')
    travel_time_right = rospy.get_param('~travel_time_right')
    
    sim = dome_door_cmd_sim(travel_time_left, travel_time_right)
    pub_thread = threading.Thread(
        target = sim.publish_status,
        daemon = True,
    )
    pub_thread.start()
    rospy.spin()
