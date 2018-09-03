
import rospy
import time


def recv(topic_name, data_class):
    recv_msg = None
    received = False
    
    def callback(msg):
        recv_msg = msg
        received = True
        return
    
    sub = rospy.Subscriber(
        name = topic_name,
        data_class = data_class,
        callback = callback,
        queue_size = 1,
    )
    
    while True:
        if reveived == True:
            break

        time.sleep(0.001)
        continue

    return recv_msg


__all__ = [
    'recv',
]
