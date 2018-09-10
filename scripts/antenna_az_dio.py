#!/usr/bin/env python3

name = "antenna_az_dio"

import pyinterface
import rospy
import std_msgs.msg


class antenna_az_dio(object):
    
    bit_status = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    count = 0

    board_name = 2724
    rsw_id = 0

    def __init__(self):
        self.do = pyinterface.open(self.board_name, self.rsw_id)
        
        self.topic_from = []
        for i in range(1,17):
            topic_from_ = rospy.Subscriber(
                    name = 'cpz2724_rsw0_do%d'%(i),
                    data_class = std_msgs.msg.Byte,
                    callback = self.update_bit_status,
                    callback_args = {'index': i-1 },
                    queue_size = 1,
                )
            self.topic_from.append(topic_from_)
            continue
        pass
    
    def update_bit_status(self, command, args):
        index = args["index"]
        self.bit_status[index] = command.data

        self.count += 1
        if self.count == 16:
            self.antenna_az_do()

        return

    def antenna_az_do(self):
        self.do.output_point(self.bit_status, 1)
        return


if __name__ == "__main__":
    rospy.init_node(name)
    az_dio = antenna_az_dio()
    rospy.spin()
