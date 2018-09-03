#! /usr/bin/env python3

name = 'dome_control'

# ----
import rospy
import std_msgs.msg

def dome_control_mapper(status):
    if status.data[0].lower() == 'l':
        topic_to.publish('LOCAL')
        
    elif status.data[0].lower() == 'r':
        topic_to.publish('REMOTE')
        
    else:
        pass
    
    return


if __name__=='__main__':
    rospy.init_node(name)

    topic_to = rospy.Publisher(
        name = name,
        data_class = std_msgs.msg.String,
        latch = True,
        queue_size = 1,
    )
    
    topic_from = rospy.Subscriber(
        name = 'dome_control_sim',
        data_class = std_msgs.msg.String,
        callback = dome_control_mapper,
        queue_size = 1,
    )

    rospy.spin()
