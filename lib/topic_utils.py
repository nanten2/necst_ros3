
import rospy
import time


class receiver(object):
    recv_msg = None
    received = False
    
    def __init__(self, topic_name, data_class):
        self.subscriber = rospy.Subscriber(
            name = topic_name,
            data_class = data_class,
            callback = self.callback,
            queue_size = 1,
        )
        pass
    
    def __del__(self):
        self.subscriber.unregister()
        pass
    
    def callback(self, msg):
        self.recv_msg = msg
        self.received = True
        return
    
    def recv(self):
        while True:
            if self.received == True:
                break
            time.sleep(0.001)
            continue
        return self.recv_msg
    

    
def recv(topic_name, data_class):
    r = receiver(topic_name, data_class)
    msg = r.recv()
    del(r)
    return msg


__all__ = [
    'receiver',
    'recv',
]
