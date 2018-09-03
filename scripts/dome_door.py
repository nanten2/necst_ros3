#! /usr/bin/env python3

name = 'dome_door'

# ----
import rospy
import std_msgs.msg

def dome_door_mapper(status):
    topic_to.publish(status)
    return

class dome_door_mapper(object):
    bit_status = [0,0,0,0,0,0]
    
    def __init__(self):
        self.topic_to = rospy.Publisher(
            name = name,
            data_class = std_msgs.msg.String,
            latch = True,
            queue_size = 1,
        )

        self.topic_from = []
        for i, dioch in enumerate([2,3,4,5,6,7]):
            topic_from_ = rospy.Subscriber(
                name = 'cpz2724_rsw2_dio%d'%(dioch),
                data_class = std_msgs.msg.Bool,
                callback = dome_emergency_mapper,
                queue_size = 1,
            )
                
            
        
    


if __name__=='__main__':
    rospy.init_node(name)

    topic_to = rospy.Publisher(
        name = name,
        data_class = std_msgs.msg.Bool,
        latch = True,
        queue_size = 1,
    )
    
    topic_from = rospy.Subscriber(
        name = 'cpz2724_rsw2_dio1', # dio ch need to be checked
        data_class = std_msgs.msg.Bool,
        callback = dome_emergency_mapper,
        queue_size = 1,
    )

    rospy.spin()
