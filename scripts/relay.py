#! /usr/bin/env python3

name = 'relay'

# ----
import configparser
import rospy
import std_msgs.msg

subscribers = {}
publishers = {}
params = {}
logic = ''

def callback(msg, argname):
    global params
    params[argname] = msg.data
    callback2()
    return

def callback2():
    global params
    global publishers
    global logic
    
    for _key in params.keys():
        exec('{argname} = params["{argname}"]'.format(argname=_key))
        continue
    
    for _key in publishers.keys():
        exec('{argname} = publishers["{argname}"]'.format(argname=_key))
        continue
    
    exec(logic)
    return


if __name__=='__main__':
    rospy.init_node(name)
    
    config_file = rospy.get_param('~config_file')
    cfg = configparser.ConfigParser()
    cfg.read(config_file)
    
    logic = cfg['Logic']['code']
    
    for argname, kwargs_str in cfg['Publishers'].items():
        kwargs = exec(kwargs_str)
        publishers[argname] = rospy.Publisher(**kwargs)
        continue
    
    for argname, kwargs_str in cfg['Subscribers'].items():
        kwargs = exec(kwargs_str)
        kwargs['callback'] = callback,
        kwargs['callback_args'] = argname
        subscribers[argname] = rospy.Subscriber(**kwargs)
        continue
    
    rospy.spin()
