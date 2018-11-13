#! /usr/bin/env python3

name = 'topic_monitor_pri'

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
            

        # Antenna
        # -------
        new('/antenna/drive', std_msgs.msg.String)
        new('/antenna/drive_cmd', std_msgs.msg.String)
        new('/az', std_msgs.msg.Float64)
        new('/antenna/az_cmd', std_msgs.msg.Float64)
        new('/az_speed', std_msgs.msg.Float64)
        new('/el', std_msgs.msg.Float64)
        new('/antenna/el_cmd', std_msgs.msg.Float64)
        new('/el_speed', std_msgs.msg.Float64)
        new('/antenna/drive_lock', std_msgs.msg.Bool)
        new('/antenna/az_lock', std_msgs.msg.Bool)
        new('/antenna/el_lock', std_msgs.msg.Bool)
        new('/antenna/control', std_msgs.msg.String)
        new('/antenna/emergency', std_msgs.msg.Bool)
        new('/antenna/error', std_msgs.msg.String)
        
        # Dome
        # ----
        new('dome/az_action', std_msgs.msg.String)
        new('dome/az_tracking', std_msgs.msg.Bool)
        new('dome/az', std_msgs.msg.Float32)
        new('dome/az_cmd', std_msgs.msg.Float32)
        new('dome/az_speed_cmd', std_msgs.msg.String)
        new('dome/az_switch', std_msgs.msg.Int8)
        new('dome/door', std_msgs.msg.String)
        new('dome/door_cmd', std_msgs.msg.String)
        new('dome/door_leftaction', std_msgs.msg.String)
        new('dome/door_leftposition', std_msgs.msg.String)
        new('dome/door_rightaction', std_msgs.msg.String)
        new('dome/door_rightposition', std_msgs.msg.String)
        new('dome/memb_action', std_msgs.msg.String)
        new('dome/memb', std_msgs.msg.String)
        new('dome/memb_cmd', std_msgs.msg.String)
        new('dome/az_lock', std_msgs.msg.Bool)
        new('dome/door_lock', std_msgs.msg.Bool)
        new('dome/memb_lock', std_msgs.msg.Bool)
        new('dome/control', std_msgs.msg.String)
        new('dome/emergency', std_msgs.msg.Bool)
        new('dome/error', std_msgs.msg.String)
        
        # Hot
        # ---
        new('/hot/position', std_msgs.msg.String)
        new('/hot/position_cmd', std_msgs.msg.String)
        new('/hot/position_lock', std_msgs.msg.Bool)
        
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
        self.values[args['name']] = str(msg.data)
        self.refresh()
        return

    def refresh(self):
        while self.refreshing == True:
            time.sleep(0.1)
            continue
        
        self.refreshing = True
        maxlen = max([len(_k) for _k in self.values.keys()])
        #print('----')
        s = ""
        for key in sorted(self.values):
            s += (('{0:<'+str(maxlen)+'} {1:.7s}').format(key, self.values[key]) + "\n")
            continue
        print(s)
        self.refreshing = False
        return
    


if __name__=='__main__':
    rospy.init_node(name)
    tm = topic_monitor()
    rospy.spin()