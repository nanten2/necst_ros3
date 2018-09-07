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
        
        # cpz2724 rsw2
        # ------------
        new('cpz2724_rsw2_di01', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di02', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di03', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di04', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di05', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di06', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di07', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di08', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di09', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di10', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di11', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di12', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di13', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di14', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di15', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di16', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di17', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di18', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di19', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di20', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di21', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di22', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di23', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di24', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di25', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di26', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di27', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di28', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di29', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di30', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di31', std_msgs.msg.Bool)
        new('cpz2724_rsw2_di32', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do01', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do02', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do03', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do04', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do05', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do06', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do07', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do08', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do09', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do10', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do11', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do12', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do13', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do14', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do15', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do16', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do17', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do18', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do19', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do20', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do21', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do22', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do23', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do24', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do25', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do26', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do27', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do28', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do29', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do30', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do31', std_msgs.msg.Bool)
        new('cpz2724_rsw2_do32', std_msgs.msg.Bool)
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
