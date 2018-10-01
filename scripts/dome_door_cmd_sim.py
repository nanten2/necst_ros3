#! /usr/bin/env python3

name = 'dome_door_cmd_sim'

# ----
import time
import threading
import rospy
import std_msgs.msg

import topic_utils


class dome_door_cmd_sim(object):
    def __init__(self, travel_time_left, travel_time_right):
        self.sim = dome_door_simulator(travel_time_left, travel_time_right)
        
        self.topic_from1 = rospy.Subscriber(
            name = 'dome_door_cmd2',
            data_class = std_msgs.msg.String,
            callback = self.update_cmd,
            queue_size = 1,
        )
        
        self.pub_left = rospy.Publisher(
            name = 'dome_door_leftposition',
            data_class = std_msgs.msg.String,
            latch = True,
            queue_size = 1,
        )
        
        self.pub_right = rospy.Publisher(
            name = 'dome_door_rightposition',
            data_class = std_msgs.msg.String,
            latch = True,
            queue_size = 1,
        )

        self.pub_left_act = rospy.Publisher(
            name = 'dome_door_leftaction',
            data_class = std_msgs.msg.String,
            latch = True,
            queue_size = 1,
        )
        
        self.pub_right_act = rospy.Publisher(
            name = 'dome_door_rightaction',
            data_class = std_msgs.msg.String,
            latch = True,
            queue_size = 1,
        )
        pass
    
    def update_cmd(self, msg):
        self.sim.set_command(msg.data)
        return
    
    def publish_status(self):
        pos_left_last = ''
        pos_right_last = ''
        act_left_last = ''
        act_right_last = ''
        
        while not rospy.is_shutdown():
            pos_left = self.sim.current_left
            if pos_left != pos_left_last:
                self.pub_left.publish(pos_left)
                pos_left_last = pos_left
                pass
            
            pos_right = self.sim.current_right
            if pos_right != pos_right_last:
                self.pub_right.publish(pos_right)
                pos_right_last = pos_right
                pass

            act_left = self.sim.current_left_action
            if act_left != act_left_last:
                self.pub_left_act.publish(act_left)
                act_left_last = act_left
                pass
            
            act_right = self.sim.current_right_action
            if act_right != act_right_last:
                self.pub_right_act.publish(act_right)
                act_right_last = act_right
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
    current_left_action = 'STOP'
    current_right_action = 'STOP'
    
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
                self.current_left_action = 'STOP'
            elif self.pos_left <= 0:
                self.pos_left = 0
                self.current_left = 'CLOSE'
                self.current_left_action = 'STOP'
            else:
                self.current_left = 'TRANSIT'
                if self.cmd == 'STOP':
                    self.current_left_action = 'STOP'
                else:
                    self.current_left_action = 'MOVE'
                    pass
                pass
                
            if self.pos_right >= 1:
                self.pos_right = 1
                self.current_right = 'OPEN'
                self.current_right_action = 'STOP'
            elif self.pos_right <= 0:
                self.pos_right = 0
                self.current_right = 'CLOSE'
                self.current_right_action = 'STOP'
            else:
                self.current_right = 'TRANSIT'
                if self.cmd == 'STOP':
                    self.current_right_action = 'STOP'
                else:
                    self.current_right_action = 'MOVE'
                    pass
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
