#! /usr/bin/env python3

name = 'door_simulator'

# ----
import time
import threading
import rospy
import std_msgs.msg

import topic_utils


class dome_door_sim(object):
    p = {
        'do1': None,
        'do2': None,
    }
    
    def __init__(self, travel_time_left, travel_time_right):
        self.sim = dome_door_simulator(travel_time_left, travel_time_right)
        
        self.do1 = rospy.Subscriber(
            name = '/cpz2724_rsw2/do05',
            data_class = std_msgs.msg.Bool,
            callback = self.callback,
            callback_args = 'do1',
            queue_size = 1,
        )
        
        self.do2 = rospy.Subscriber(
            name = '/cpz2724_rsw2/do06',
            data_class = std_msgs.msg.Bool,
            callback = self.callback,
            callback_args = 'do2',
            queue_size = 1,
        )
        
        self.pub_left1 = rospy.Publisher(
            name = '/cpz2724_rsw2/di06',
            data_class = std_msgs.msg.Bool,
            latch = True,
            queue_size = 1,
        )
        
        self.pub_left2 = rospy.Publisher(
            name = '/cpz2724_rsw2/di07',
            data_class = std_msgs.msg.Bool,
            latch = True,
            queue_size = 1,
        )
        
        self.pub_right1 = rospy.Publisher(
            name = '/cpz2724_rsw2/di03',
            data_class = std_msgs.msg.Bool,
            latch = True,
            queue_size = 1,
        )
        
        self.pub_right2 = rospy.Publisher(
            name = '/cpz2724_rsw2/di04',
            data_class = std_msgs.msg.Bool,
            latch = True,
            queue_size = 1,
        )
        
        self.pub_left_act = rospy.Publisher(
            name = '/cpz2724_rsw2/di05',
            data_class = std_msgs.msg.Bool,
            latch = True,
            queue_size = 1,
        )
        
        self.pub_right_act = rospy.Publisher(
            name = '/cpz2724_rsw2/di02',
            data_class = std_msgs.msg.Bool,
            latch = True,
            queue_size = 1,
        )
        pass

    def callback(self, msg, argname):
        self.p[argname] = msg.data
        self.callback2()
        return

    def callback2(self):
        if self.p['do1'] == True and self.p['do2'] == True:
            self.sim.set_command('OPEN')
        elif self.p['do1'] == False and self.p['do2'] == True:
            self.sim.set_command('CLOSE')
        elif self.p['do1'] == False and self.p['do2'] == False:
            self.sim.set_command('STOP')
        else:
            pass
        return
            
    def publish_status(self):
        pos_left_last = ''
        pos_right_last = ''
        act_left_last = ''
        act_right_last = ''
        
        while not rospy.is_shutdown():
            pos_left = self.sim.current_left
            if pos_left != pos_left_last:
                if pos_left == 'OPEN':
                    self.pub_left1.publish(True)
                    self.pub_left2.publish(True)
                elif pos_left == 'CLOSE':
                    self.pub_left1.publish(False)
                    self.pub_left2.publish(True)
                elif pos_left == 'TRANSIT':
                    self.pub_left1.publish(False)
                    self.pub_left2.publish(False)
                    pass
                pos_left_last = pos_left
                pass
            
            pos_right = self.sim.current_right
            if pos_right != pos_right_last:
                if pos_right == 'OPEN':
                    self.pub_right1.publish(True)
                    self.pub_right2.publish(True)
                elif pos_right == 'CLOSE':
                    self.pub_right1.publish(False)
                    self.pub_right2.publish(True)
                elif pos_right == 'TRANSIT':
                    self.pub_right1.publish(False)
                    self.pub_right2.publish(False)
                    pass
                pos_right_last = pos_right
                pass

            act_left = self.sim.current_left_action
            if act_left != act_left_last:
                if act_left == 'MOVE':
                    self.pub_left_act.publish(True)
                elif act_left == 'STOP':
                    self.pub_left_act.publish(False)
                    pass
                act_left_last = act_left
                pass
            
            act_right = self.sim.current_right_action
            if act_right != act_right_last:
                if act_right == 'MOVE':
                    self.pub_right_act.publish(True)
                elif act_right == 'STOP':
                    self.pub_right_act.publish(False)
                    pass
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
    
    sim = dome_door_sim(travel_time_left, travel_time_right)
    pub_thread = threading.Thread(
        target = sim.publish_status,
        daemon = True,
    )
    pub_thread.start()
    rospy.spin()
