#!/usr/bin/env python3

name = "antenna_el_feedback"

import rospy
import std_msgs.msg
import topic_utils


class antenna_el_feedback(object):

    speed_d = 0.0
    pre_arcsrc = 0.0
    pre_hensa = 0.0
    ihensa = 0.0
    t_now = t_past = 0.0

    arcsec_enc = 0.0

    p_coeff = 3.7
    i_coeff = 3.0
    d_coeff = 0.0
    
    def __init__(self):
        self.topic_to = rospy.Publisher(
                name = name,
                data_class = std_msgs.msg.Float64,
                queue_size = 1,
            )

        topic_from1 = rospy.Subscriber(
                name = "antenna_el_cmd2",
                data_class = std_msgs.msg.Float64,
                callback = self.antenna_el_feedback,
                queue_size = 1,
            )

        topic_from2 = rospy.Subscriber(
                name = "encoder_el",
                data_class = std_msgs.msg.Float64,
                callback = self.antenna_el_encoder,
                queue_size = 1,
            )

        topic_from_pid = rospy.Subscriber(
                name = name + "_pid",
                data_class = std_msgs.msg.Float64MultiArray,
                callback = self.antenna_el_pid,
                queue_size = 1,
            )

        pass

    def antenna_el_feedback(self, command):
        MOTOR_MAXSTEP = 1000
        MOTOR_EL_MAXRATE = 10000
        rate_to_arcsec = (12/7)*(3600/10000)
        
        arcsec_cmd = command.data * 3600.
        
        if self.t_past == 0.0:
            self.t_past = time.time()
        else:
            pass
        self.t_now = time.time()

        ret = calc_pid(arcsec_cmd, self.arcsec_enc,
                self.pre_arcsec, self.pre_hensa, self.ihensa,
                self.t_now, self.t_past,
                self.p_coeff, self.i_coeff, self.d_coeff)
        
        speed = ret[0]

        #update
        self.pre_hensa = arcsec_cmd - self.arcsec_enc
        self.pre_arcsec = arcsec_cmd
        self.ihensa = ret[1]
        self.t_past = self.t_now

        #limit of acceleraion
        if abs(speed - self.speed_d) < MOTOR_MAXSTEP*rate_to_arcsec:
            self.speed_d = speed
        else:
            if (speed - self.speed_d) < 0:
                a = -1
            else:
                a = 1
            self.speed_d += a*MOTOR_MAXSTEP*rate_to_arcsec

        #limit of max speed
        if self.speed_d > MOTOR_EL_MAXRATE*rate_to_arcsec:
            self.speed_d = MOTOR_EL_MAXRATE*rate_to_arcsec
        if self.speed_d < -MOTOR_EL_MAXRATE*rate_to_arcsec:
            self.speed_d = -MOTOR_EL_MAXRATE*rate_to_arcsec
        
        command_speed = self.speed_d

        lock = topic_utils.recv(name +"_lock", std_msgs.msg.Bool).data
        if self.lock == True:
            self.speed_d = 0.0
            self.topic_to_publish(0.0)
            return
        else:
            self.topic_to.publish(command_speed)
        return

    def antenna_el_encoder(self, status):
        self.arcsec_enc = status.data
        return

    def antenna_el_pid(self, status):
        self.p_coeff = status.data[0]
        self.i_coeff = status.data[1]
        self.d_coeff = status.data[2]
        return
    
    def antenna_el_feedback_lock(self, status):
        self.lock = status.data
        return

def calc_pid(target_arcsec, encoder_arcsec, pre_arcsec, pre_hensa, ihensa, enc_before, t_now, t_past, p_coeff, i_coeff, d_coeff):
    """                                                                         
    DESCRIPTION                                                                 
    ===========                                                                 
    This function determine az&el speed for antenna                             
    """

    #calculate ichi_hensa
    hensa = target_arcsec - encoder_arcsec

    dhensa = hensa - pre_hensa
    if math.fabs(dhensa) > 1:
        dhensa = 0

    if (encoder_arcsec - enc_before) != 0.0:
        current_speed = (encoder_arcsec - enc_before) / (t_now-t_past)

    if pre_arcsec == 0: # for first move
        target_speed = 0
    else:
        target_speed = (target_arcsec - pre_arcsec)/(t_now - t_past)

    ihensa += (hensa + pre_hensa)/2
    if math.fabs(hensa) > 50:
        ihensa = 0.0

        #PID
    rate = target_speed + p_coeff*hensa + i_coeff*ihensa*(t_now-t_past) + d_coeff*dhensa/(t_now-t_past)

    return [rate, ihensa]

if __name__ == "__main__":
    rospy.init_node(name)
    feedback = antenna_el_feedback()
    rospy.spin()
