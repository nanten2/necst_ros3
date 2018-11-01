#! /usr/bin/env python3

name = 'memb_simulator'

# ----
import time
import threading
import rospy
import std_msgs.msg

import topic_utils


class dome_memb_sim(object):
    p = {
        'do1': None,
        'do2': None,
    }
    
    def __init__(self, travel_time_left, travel_time_right):
        self.sim = dome_memb_simulator(travel_time)
        
        self.do1 = rospy.Subscriber(
            name = '/cpz2724_rsw2/do07',
            data_class = std_msgs.msg.Bool,
            callback = self.callback,
            callback_args = 'do1',
            queue_size = 1,
        )
        
        self.do2 = rospy.Subscriber(
            name = '/cpz2724_rsw2/do08',
            data_class = std_msgs.msg.Bool,
            callback = self.callback,
            callback_args = 'do2',
            queue_size = 1,
        )
        
        self.pub_1 = rospy.Publisher(
            name = '/cpz2724_rsw2/di09',
            data_class = std_msgs.msg.Bool,
            latch = True,
            queue_size = 1,
        )
        
        
        self.pub_2 = rospy.Publisher(
            name = '/cpz2724_rsw2/di10',
            data_class = std_msgs.msg.Bool,
            latch = True,
            queue_size = 1,
        )
        
        
        self.pub_act = rospy.Publisher(
            name = '/cpz2724_rsw2/di08',
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
        pos_last = ''
        act_last = ''
        
        while not rospy.is_shutdown():
            
            pos = self.sim.current
            if pos != pos_last:
                if pos == 'OPEN':
                    self.pub_1.publish(True)
                    self.pub_2.publish(True)
                elif pos == 'CLOSE':
                    self.pub_1.publish(False)
                    self.pub_2.publish(True)
                elif pos == 'TRANSIT':
                    self.pub_1.publish(False)
                    self.pub_2.publish(False)
                    pass
                pos_last = pos
                pass

            act = self.sim.current_action
            if act != act_last:
                if act == 'MOVE':
                    self.pub_act.publish(True)
                elif act == 'STOP':
                    self.pub_act.publish(False)
                    pass
                act_last = act
                pass
            
            time.sleep(0.05)
            continue
        
        return


class dome_memb_simulator(object):
    pos = 0
    
    travel_time = 0
    velocity = 0
    
    cmd = 'CLOSE'
    current = 'CLOSE'
    current_action = 'STOP'
    
    rate = 0.05
    
    def __init__(self, travel_time):
        tt = travel_time if travel_time != 0 else self.rate
        self.travel_time = tt
        self.velocity = 1 / tt * self.rate

        pub_thread = threading.Thread(
            target = self.simulate,
            daemon = True,
        )
        pub_thread.start()
        pass
    
    def simulate(self):
        while not rospy.is_shutdown():
            if self.cmd == 'CLOSE':
                self.pos -= self.velocity
            elif self.cmd == 'OPEN':
                self.pos += self.velocity
                pass

            if self.pos >= 1:
                self.pos = 1
                self.current = 'OPEN'
                self.current_action = 'STOP'
            elif self.pos <= 0:
                self.pos = 0
                self.current = 'CLOSE'
                self.current_action = 'STOP'
            else:
                self.current = 'TRANSIT'
                if self.cmd == 'STOP':
                    self.current_action = 'STOP'
                else:
                    self.current_action = 'MOVE'
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
    travel_time = rospy.get_param('~travel_time')
    
    sim = dome_memb_sim(travel_time)
    pub_thread = threading.Thread(
        target = sim.publish_status,
        daemon = True,
    )
    pub_thread.start()
    rospy.spin()
