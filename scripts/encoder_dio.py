#!/usr/bin/env python3

name = "encoder_dio"

import time
import pyinterface
import rospy
import std_msgs.msg


board_name = 6204
rsw_id = 0



def board_initialize():
    if di.get_mode().to_bit() == "00000000" :
        di.initialize()
        di.set_mode(mode="MD0 SEL1",direction=1, equal=0, latch=0, ch=1)
        di.set_mode(mode="MD0 SEL1",direction=1, equal=0, latch=0, ch=2)
        di.set_z_mode(clear_condition="CLS0", latch_condition="", z_polarity=0, ch=1)
        di.set_z_mode(clear_condition="CLS0", latch_condition="", z_polarity=0, ch=2)
    else:
        pass

if __name__ == "__main__":
    rospy.init_node(name)
    di = pyinterface.open(board_name, rsw_id)
    board_initialize()

    topic_to1 = rospy.Publisher(
            name = "cpz6204_rsw0_di1",
            data_class = std_msgs.msg.Int64,
            latch = True
            queue_size = 1,
        )

    topic_to2 = rospy.Publisher(
            name = "cpz6204_rsw0_di2",
            data_class = std_msgs.msg.Int64,
            latch = True
            queue_size = 1,
        )
    
    enc_az_last = 0
    enc_el_last = 0
    while True:
        enc_az = int(di.get_counter(1).to_int())
        enc_el = int(di.get_counter(2).to_int())

        if enc_az != enc_az_last or enc_el != enc_el_last:
            topic_to1.publish(enc_az)
            topic_to2.publish(enc_el)
            
            enc_az_last = enc_az
            enc_el_last = enc_el

        time.sleep(0.001)
        continue

