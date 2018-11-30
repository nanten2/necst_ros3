#import pdb; pdb.set_trace()
import rospy
import time
import signal
import sys
#sys.path.append('/home/amigos/n2lite')
#import n2lite
import sqlite3
import queue
from std_msgs.msg import String

sql_queue = queue.Queue(maxsize=0)

rospy.init_node('sql_logger')
topic_list = rospy.get_published_topics()

###config
ns_topic = ['/rosout', '/rosout_agg', '/logger_flag']
ns_topic_word = ['/cpz27', '/cpz62', '/sqlog']
###cmd
msg_import_cmd = 'from {0}.msg import {1}'
sub_r_cmd = '_{} = rospy.Subscriber(i[0][1:], eval(msg_type[1]),save, i[0][1:], queue_size = 1)'

list_cmd = '{}_list = []'
append_cmd = '{}_list.append([now, req.data])'
###main
c = sqlite3.connect('./test_1101_03.db', check_same_thread=False)
cc = c.cursor()

logger_flag = 0

def set_flag(req):
    global logger_flag
    if req.data == '':
        logger_flag = 0
    else:
        logger_flag = 1

rospy.Subscriber('logger_flag', String, set_flag, queue_size=1)

def handler(signal, frame):
    for x in topic_list:
        try:
            exec("_{}.unregister()".format(x[0][1:]))
        except:
            pass
    print('save end')
    cc.close()
    rospy.signal_shutdown('(>_<)')
    

def save(req, topic):
    try:
        if logger_flag == 0:
            print('stop logging')
            time.sleep(0.1)
            return
        else:pass
        now = time.time()
        exec(append_cmd.format(topic))
        if len(eval('{}_list'.format(topic))) > 100:
            global sql_queue; sql_queue.put([topic, eval('{}_list'.format(topic))])
            ####REMOVE LIST###
            exec('global {0}_list; {0}_list = []'.format(topic))
        time.sleep(0.04)
    except Exception as e:
        pass
    
print('Total subscriber :',len(topic_list))
###Get topic list
for (i,x) in enumerate(topic_list):
    msg_type = x[1].split('/')
    if x[0] in ns_topic:
        del topic_list[i]
    elif x[0][0:6] in ns_topic_word:
        print('del {}'.format(i), x)
        del topic_list[i]
    else:
        pass
print('Subscribe :',len(topic_list))

table_make = "CREATE table if not exists {} {}"

for i in topic_list:
    cc.execute(table_make.format(i[0][1:], "('time' float, '{}' float)".format(i[0][1:])))
    exec(list_cmd.format(i[0][1:]))

c.commit()

for i in topic_list:
    msg_type = i[1].split('/')
    exec(msg_import_cmd.format(msg_type[0],msg_type[1]))
    exec(sub_r_cmd.format(i[0][1:]))

print('*** logging start ***')
while not rospy.is_shutdown():
    if sql_queue.qsize() == 0:
        time.sleep(0.001)
        continue
    sqldata = sql_queue.get()
    cc.executemany("INSERT INTO {} (time, {}) VALUES (?,?)".format(sqldata[0], sqldata[0]), sqldata[1])
    c.commit()
    time.sleep(0.00001)
rospy.spin()

