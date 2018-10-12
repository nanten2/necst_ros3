#! /usr/bin/env python3

name = 'topic_monitor'

# ----
import time
import threading
import rospy
import std_msgs.msg


class topic_monitor(object):
    values = {}
    refreshing = False
    
    def __init__(self):
        def new(name, data_class):
            rospy.Subscriber(
                name = name,
                data_class = data_class,
                callback = self.callback,
                callback_args = {'name': name},
                queue_size = 1,
            )
            return            
            
        # Dome
        # ----
        new('dome_control', std_msgs.msg.String)
        new('dome_door', std_msgs.msg.String)
        new('dome_door_cmd', std_msgs.msg.String)
        new('dome_door_cmd2', std_msgs.msg.String)
        new('dome_door_leftaction', std_msgs.msg.String)
        new('dome_door_leftposition', std_msgs.msg.String)
        new('dome_door_lock', std_msgs.msg.Bool)
        new('dome_door_rightaction', std_msgs.msg.String)
        new('dome_door_rightposition', std_msgs.msg.String)
        new('dome_emergency', std_msgs.msg.Bool)
        
        # Weather
        # -------
        new('/weather/B_humi', std_msgs.msg.Float32)
        new('/weather/B_temp', std_msgs.msg.Float32)
        new('/weather/D_humi', std_msgs.msg.Float32)
        new('/weather/D_temp', std_msgs.msg.Float32)
        new('/weather/cabin_temp1', std_msgs.msg.Float32)
        new('/weather/cabin_temp2', std_msgs.msg.Float32)
        new('/weather/dome_temp1', std_msgs.msg.Float32)
        new('/weather/dome_temp2', std_msgs.msg.Float32)
        new('/weather/outside1_humi', std_msgs.msg.Float32)
        new('/weather/outside1_temp', std_msgs.msg.Float32)
        new('/weather/outside2_humi', std_msgs.msg.Float32)
        new('/weather/outside2_temp', std_msgs.msg.Float32)
        new('/weather/press', std_msgs.msg.Float32)
        new('/weather/rain', std_msgs.msg.Float32)
        new('/weather/wind_direction', std_msgs.msg.Float32)
        new('/weather/wind_speed', std_msgs.msg.Float32)

        pass
    
    def callback(self, msg, args):
        self.values[args['name']] = msg.data
        self.refresh()
        return

    def refresh(self):
        while self.refreshing == True:
            time.sleep(0.1)
            continue
        
        self.refreshing = True
        maxlen = max([len(_k) for _k in self.values.keys()])
        print('----')
        for key in sorted(self.values):
            print(('{0:<'+str(maxlen)+'} {1}').format(key, self.values[key]))
            continue
        self.refreshing = False
        return
    


if __name__=='__main__':
    rospy.init_node(name)
    tm = topic_monitor()
    rospy.spin()
