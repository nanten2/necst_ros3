#! /usr/bin/env python3

name = 'topic_monitor'

# ----
import time
import threading
import rospy
import std_msgs.msg


class topic_monitor(object):
    values = {}
    
    def __init__(self):
        def new(name):
            rospy.Subscriber(
                name = name,
                data_class = std_msgs.msg.String,
                callback = self.callback,
                callback_args = {'name': name},
                queue_size = 1,
            )
            return            
            
        # Dome
        # ----
        new('dome_control')
        new('dome_door')
        new('dome_door_cmd')
        new('dome_door_cmd2')
        new('dome_leftaction')
        new('dome_leftposition')
        new('dome_lock')
        new('dome_rightaction')
        new('dome_rightposition')
        new('dome_emergency')
        
        pass
    
    def callback(self, msg, args):
        self.values[args['name']] = msg.data
        self.refresh()
        return

    def refresh(self):
        maxlen = max([len(_k) for _k in self.values.keys()])
        for key in sorted(self.values):
            print('----')
            print(('{0:<'+str(maxlen)+'} {1}').format(key, self.values[key]))
            continue        
        return
    


if __name__=='__main__':
    rospy.init_node(name)
    rospy.spin()
