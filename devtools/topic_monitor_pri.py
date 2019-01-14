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
        new('/antenna/az', std_msgs.msg.Float64)
        new('/antenna/az_cmd', std_msgs.msg.Float64)
        new('/antenna/az_cmd2', std_msgs.msg.Float64)
        new('/antenna/az_speed', std_msgs.msg.Float64)
        new('/antenna/el', std_msgs.msg.Float64)
        new('/antenna/el_cmd', std_msgs.msg.Float64)
        new('/antenna/el_cmd2', std_msgs.msg.Float64)
        new('/antenna/el_speed', std_msgs.msg.Float64)
        new('/antenna/drive_lock', std_msgs.msg.Bool)
        new('/antenna/az_lock', std_msgs.msg.Bool)
        new('/antenna/el_lock', std_msgs.msg.Bool)
        new('/antenna/control', std_msgs.msg.String)
        new('/antenna/emergency', std_msgs.msg.Bool)
        new('/antenna/error', std_msgs.msg.String)
        new('/antenna/az_soft_limit', std_msgs.msg.Bool)
        new('/antenna/el_soft_limit', std_msgs.msg.Bool)
        
        # Dome
        # ----
        new('/dome/az_action', std_msgs.msg.String)
        new('/dome/az_tracking', std_msgs.msg.Bool)
        new('/dome/az', std_msgs.msg.Float32)
        new('/dome/az_cmd', std_msgs.msg.Float32)
        new('/dome/az_speed_cmd', std_msgs.msg.String)
        new('/dome/az_switch', std_msgs.msg.Int8)
        new('/dome/door', std_msgs.msg.String)
        new('/dome/door_cmd', std_msgs.msg.String)
        new('/dome/memb_action', std_msgs.msg.String)
        new('/dome/memb', std_msgs.msg.String)
        new('/dome/memb_cmd', std_msgs.msg.String)
        new('/dome/az_lock', std_msgs.msg.Bool)
        new('/dome/door_lock', std_msgs.msg.Bool)
        new('/dome/memb_lock', std_msgs.msg.Bool)
        new('/dome/control', std_msgs.msg.String)
        new('/dome/error', std_msgs.msg.String)
        
        # Hot
        # ---
        new('/hot/position', std_msgs.msg.String)
        new('/hot/position_cmd', std_msgs.msg.String)
        new('/hot/position_lock', std_msgs.msg.Bool)
        
        # M4
        # ---
        new('/m4/position', std_msgs.msg.String)
        new('/m4/position_cmd', std_msgs.msg.String)
        
        #new('/cpz7204_rsw0/busy', std_msgs.msg.Bool)
        #new('/cpz7204_rsw0/m_EL', std_msgs.msg.Bool)
        #new('/cpz7204_rsw0/p_EL', std_msgs.msg.Bool)
        #new('/cpz7204_rsw0/step', std_msgs.msg.Bool)

        # M2
        # ---
        new('/m2/position', std_msgs.msg.Float64)
        new('/m2/position_cmd', std_msgs.msg.Float64)
        new('/m2/limit', std_msgs.msg.String)

        # Weather
        # ---
        new('/weather/wind_speed', std_msgs.msg.Float32)
        pass
    
    def callback(self, msg, args):
        if type(msg.data) == float: 
            self.values[args['name']] = '%10f'%(msg.data)
        else:
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
            s += (('{0:<'+str(maxlen)+'} {1:.10s}').format(key, self.values[key]) + "\n")
            continue
        print(s)
        self.refreshing = False
        return
    


if __name__=='__main__':
    rospy.init_node(name)
    tm = topic_monitor()
    rospy.spin()
