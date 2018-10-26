#!/usr/bin/env python3

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

