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
        new('dome/control', std_msgs.msg.String)
        new('dome/door', std_msgs.msg.String)
        new('dome/door_cmd', std_msgs.msg.String)
        new('dome/door_cmd2', std_msgs.msg.String)
        new('dome/door_leftaction', std_msgs.msg.String)
        new('dome/door_leftposition', std_msgs.msg.String)
        new('dome/door_lock', std_msgs.msg.Bool)
        new('dome/door_rightaction', std_msgs.msg.String)
        new('dome/door_rightposition', std_msgs.msg.String)
        new('dome/emergency', std_msgs.msg.Bool)
        
        # cpz2724 rsw2
        # ------------
        new('cpz2724_rsw2/di01', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di02', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di03', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di04', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di05', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di06', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di07', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di08', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di09', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di10', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di11', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di12', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di13', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di14', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di15', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di16', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di17', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di18', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di19', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di20', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di21', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di22', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di23', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di24', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di25', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di26', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di27', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di28', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di29', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di30', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di31', std_msgs.msg.Bool)
        new('cpz2724_rsw2/di32', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do01', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do02', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do03', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do04', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do05', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do06', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do07', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do08', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do09', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do10', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do11', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do12', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do13', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do14', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do15', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do16', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do17', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do18', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do19', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do20', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do21', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do22', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do23', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do24', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do25', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do26', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do27', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do28', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do29', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do30', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do31', std_msgs.msg.Bool)
        new('cpz2724_rsw2/do32', std_msgs.msg.Bool)
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
