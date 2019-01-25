import datetime
from astropy.time import Time
import math
import time
import numpy as np
from PIL import Image
from PIL import ImageOps
import os
import shutil
import rospy #debug all_sky_shot
import sys
sys.path.append('/home/amigos/ros/src/necst_ros3/scripts')
import v3_controller
con = v3_controller.controller()
import v3_reader
red = v3_reader.reader(node=False)


class ccd_controller(object):

    error = []
    status = {}
    error_count = 0
    
    def __init__(self):
        pass
    
    def save_status(self, x, y, number, magnitude, az_star, el_star, mjd, data_name, secofday, status):
        f = open("/home/amigos/data/opt/"+str(data_name)+"/process.log", "a")
        geo_status = [0,0,0,0]
        geo_x = 0
        geo_y = 0
        geo_temp = [0, 0]
        
        #write papram
        f.write(str(number)+" "+str(magnitude)+" "+str(mjd)+" "+str(secofday)+" "+str(status["Command_Az"])+" "+str(status["Command_El"])+" "\
        +str(status["Current_Az"])+" "+str(status["Current_El"])+" "+str(status["Current_Dome"])+" "+str(x)+" "+str(y)+" "+str(status["OutTemp"])+" "+str(status["Press"])\
        +" "+str(status["OutHumi"])+" "+str(az_star)+" "+str(el_star)+" "+str(geo_x)+" "+str(geo_y)+" "+str(geo_status[0])+" "+str(geo_status[1])\
        +" "+str(geo_temp[0])+" "+str(geo_status[2])+" "+str(geo_status[3])+" "+str(geo_temp[1]))
        f.write("\n")
        f.close()
        return
    
    def all_sky_shot(self, number, magnitude, az_star, el_star, data_name):
        thr = 80 #threshold of brightness
        
        
        date = datetime.datetime.today()
        hour = str("{0:02d}".format(date.hour))
        minute = str("{0:02d}".format(date.minute))
        second = str("{0:02d}".format(date.second))
        name = hour+minute+second
        
        #oneshot
        try:
            con.camera.oneshot(name)
            self.error_count = 0

            status = {
                    "Command_Az": red.antenna.az_cmd(),
                    "Command_El": red.antenna.el_cmd(),
                    "Current_Az": red.antenna.az(),
                    "Current_El": red.antenna.el(),
                    "Current_Dome": red.dome.az(),
                    "OutTemp": red.weather.out_temp(),
                    "Press": red.weather.pressure(),
                    "OutHumi": red.weather.out_humi(),
                    }

        except Exception as e:
            self.error_count += 1
            if self.error_count > 3:
                raise 
            else:
                pass
        mjd = Time(date).mjd
        secofday = date.hour*60*60 + date.minute*60 + date.second + date.microsecond*0.000001
        
        while not rospy.is_shutdown():
            if os.path.exists("/home/amigos/data/opt/"+name+".jpg") == True:
                break
            time.sleep(0.1)
            continue

        time.sleep(2.)

        ###triming
        origin_image = Image.open("/home/amigos/data/opt/"+name+".jpg")
        trim_image = origin_image.crop((2080.0, 1360.0, 2720.0, 1840.0))
        trim_image.save("/home/amigos/data/opt/"+data_name+"/"+name+"_trim.jpg")
        ###triming end

        in_image = Image.open("/home/amigos/data/opt/"+data_name+"/"+name+"_trim.jpg")
        image = np.array(ImageOps.grayscale(in_image))
        
        #threshold
        width = len(image[0])
        height = len(image)
        for i in range(height):
            for j in range(width):
                if image[i][j] < thr:
                    image[i][j] = 0
        
        #calc dimention
        p_array = np.zeros(256)
        for i in range(height):
            for j in range(width):
                p_array[image[i][j]] += 1
        
        #find color
        num = 1
        nmax = 1
        for i in range(255):
            if nmax < p_array[i+1]:
                nmax = p_array[i+1]
                num = i+1
        
        #find star
        x = 0
        y = 0
        n = 0
        for i in range(height):
            for j in range(width):
                if image[i][j] == num:
                    x += j
                    y += i
                    n += 1
        if n == 0:
            print("CAN'T FIND STAR") #black photograph
            return 1
        x = x/n
        y = y/n
        
        #find center
        xx = 0.
        yy = 0.
        f = 0.
        x = int(x)
        y = int(y)
        try:
            for i in range(21):
                for j in range(21):
                    xx += (x+j-10.)*image[y+i-10][x+j-10]
                    yy += (y+i-10.)*image[y+i-10][x+j-10]
                    f += image[y+i-10][x+j-10]
        except Exception as e:
            print(e)
            rospy.loginfo(e)
            xx = 10000
            yy = 10000
            f = 0
        
        if f == 0.: #two or more stars
            print("MANY STARS ARE PHOTOGRAPHED")
            return 2
        
        xx = xx/f
        yy = yy/f
        print(xx)
        print(yy)
        
        self.save_status(xx, yy, number, magnitude, az_star, el_star, mjd, data_name, secofday, status)

        shutil.move("/home/amigos/data/opt/"+name+".jpg", "/home/amigos/data/opt/"+data_name)
        shutil.move("/home/amigos/data/opt/"+name+"_trim.jpg", "/home/amigos/data/opt/"+data_name)

        return
    
