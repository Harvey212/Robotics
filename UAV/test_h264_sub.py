#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64MultiArray
import av
import cv2
import numpy as np
import threading
import traceback
import time


import apriltag

class StandaloneVideoStream(object):
    def __init__(self):
        self.cond = threading.Condition()
        self.queue = []
        self.closed = False

    def read(self, size):
        self.cond.acquire()
        try:
            if len(self.queue) == 0 and not self.closed:
                self.cond.wait(2.0)
            data = bytes()
            while 0 < len(self.queue) and len(data) + len(self.queue[0]) < size:
                data = data + self.queue[0]
                del self.queue[0]
        finally:
            self.cond.release()
        return data

    def seek(self, offset, whence):
        return -1

    def close(self):
        self.cond.acquire()
        self.queue = []
        self.closed = True
        self.cond.notifyAll()
        self.cond.release()

    def add_frame(self, buf):
        self.cond.acquire()
        self.queue.append(buf)
        self.cond.notifyAll()
        self.cond.release()


stream = StandaloneVideoStream()


def callback(msg):
    #rospy.loginfo('frame: %d bytes' % len(msg.data))
    stream.add_frame(msg.data)

def find_Mask(img):   #red
  lr0 = np.array([0, 70, 0])
  ur0 = np.array([5, 255, 255])
  lr1 = np.array([175, 70, 0])
  ur1 = np.array([180, 255, 255])
  rm0 = cv2.inRange(img, lr0, ur0)
  rm1 = cv2.inRange(img, lr1, ur1)
  rm = cv2.bitwise_or(rm0, rm1)
  return rm


def find_Mask2(img): #blue
  lb0 = np.array([100, 70, 0])
  ub0 = np.array([130, 255, 255])
  bm = cv2.inRange(img, lb0, ub0)
  return bm

def find_Mask3(img): #green
  lg0 = np.array([45, 70, 0])
  ug0 = np.array([75, 255, 255])
  gm = cv2.inRange(img, lg0, ug0)
  return gm



def main():

    fourcc = cv2.VideoWriter_fourcc('X', 'V', 'I', 'D')
    out = cv2.VideoWriter('test.avi', fourcc, 20.0, (1920, 720))
    
    rospy.init_node('h264_listener')
    rospy.Subscriber("/tello/image_raw/h264", CompressedImage, callback)
    point_pub = rospy.Publisher("/target_point", Float64MultiArray, queue_size = 10)
    container = av.open(stream)
    rospy.loginfo('main: opened')
    frame_skip = 300
    ################################3
    start_detect = True
    hasPassedTwice=0
    cammiddle=1
    mission=0

    width=960
    height=720    
    detector = apriltag.Detector()
    
    mygoal=3

    ########################################


    for frame in container.decode(video=0):
        if 0 < frame_skip:
          frame_skip -= 1
          continue
        start_time = time.time()
        image = cv2.cvtColor(np.array(
            frame.to_image()), cv2.COLOR_RGB2BGR)
        
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        if mission==0:
         mask = find_Mask(hsv_img)
        elif mission==1:
         mask = find_Mask2(hsv_img)
        else:
         mask = find_Mask3(hsv_img)

        c_c, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        show_image = cv2.cvtColor(np.zeros(image.shape[:2], dtype=np.uint8), cv2.COLOR_GRAY2BGR)
        #cv2.drawContours(show_image, [max_c], -1, (0, 0, 255), -1)

        ###################################################
        #         #        #         #
        ###########        #         #  
        #         #        #         #
        #   #     #        #         #
        ###########        #         #
        #         #        #         #
        #         #        #         #
        ###################################################
        approach=0
        
       
        if mission==2:
         ###########################################################################3
         if len(c_c)==0:
          cammiddle=0
          rec_x=-1
          rec_y=-1
          point_pub.publish(Float64MultiArray(data = [rec_x, rec_y, 0,hasPassedTwice,cammiddle,mission]))
         else:
          cammiddle=1
          max_c = max(c_c, key = cv2.contourArea)
          rect = cv2.minAreaRect(max_c)
          r_w, r_h = rect[1]
          rec_x = rect[0][0]    #the rectangle center x
          rec_y = rect[0][1]    #the rectangle center y
          cv2.rectangle(show_image, (int(rec_x-0.5*r_w), int(rec_y-0.5*r_h)), (int(rec_x+0.5*r_w), int(rec_y+0.5*r_h)), (0,255,0), -1)
          cv2.circle(show_image, (int(rec_x), int(rec_y)), 10, (0,50,175), -1)
          cv2.putText(show_image, str((r_w * r_h) / (960*720.)), (10,40), 5, 2, (255,255,0))

          search = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
          results = detector.detect(search)
          
          if len(results)==0:
           mission=2
          else:
           goal = results[0].tag_id
           print(goal)
           
           if int(goal)==10:
            mission=3
            mygoal=1
           elif int(goal)==11:
            mission=3
            mygoal=0
           else:
            mission=2
          
          #if r_w * r_h < width*height*0.001:
          # approach=1
          
          point_pub.publish(Float64MultiArray(data = [rec_x, rec_y, 0,hasPassedTwice,cammiddle,mission]))
         ###########################################################################################################3 
        elif mission==3:
         if len(c_c)==0:
          cammiddle=0
          rec_x=-1
          rec_y=-1
          point_pub.publish(Float64MultiArray(data = [rec_x, rec_y, 0,hasPassedTwice,cammiddle,mission]))

          mission=mygoal
         

         else:    
          cammiddle=1
          max_c = max(c_c, key = cv2.contourArea)
          rect = cv2.minAreaRect(max_c)
          r_w, r_h = rect[1]
          rec_x = rect[0][0]    #the rectangle center x
          rec_y = rect[0][1]    #the rectangle center y
          cv2.rectangle(show_image, (int(rec_x-0.5*r_w), int(rec_y-0.5*r_h)), (int(rec_x+0.5*r_w), int(rec_y+0.5*r_h)), (0,255,0), -1)
          cv2.circle(show_image, (int(rec_x), int(rec_y)), 10, (0,50,175), -1)
          cv2.putText(show_image, str((r_w * r_h) / (960*720.)), (10,40), 5, 2, (255,255,0))
          
          #rdW=int(width/3)
          #halfrdW=int(1/2*(rdW))
          #traceW=rdW+halfrdW
          
          #if r_w * r_h >width*height*0.001:
          # if (rec_x-traceW)<0:
          #  mission=mygoal
        

        #################################################################################################3
        else:
         ######################################################################################3
         if len(c_c)==0:
          cammiddle=0
          rec_x=-1
          rec_y=-1

          if hasPassedTwice==2:
           #point_pub.publish(Float64MultiArray(data = [rec_x, rec_y, 1,hasPassedTwice,cammiddle]))
           stream.close()
           cv2.destroyAllWindows()
           break
          else:
           start_detect=True
           if hasPassedTwice==1:
            if mygoal==3:
             mission=2

           point_pub.publish(Float64MultiArray(data = [rec_x, rec_y, 0,hasPassedTwice,cammiddle,mission]))
           
           

         else:
          cammiddle=1
          max_c = max(c_c, key = cv2.contourArea)
          rect = cv2.minAreaRect(max_c)
          r_w, r_h = rect[1]
          rec_x = rect[0][0]    #the rectangle center x
          rec_y = rect[0][1]    #the rectangle center y
      

          if mission==0:
           cv2.rectangle(show_image, (int(rec_x-0.5*r_w), int(rec_y-0.5*r_h)), (int(rec_x+0.5*r_w), int(rec_y+0.5*r_h)), (0,0,255), -1)
          else:
           cv2.rectangle(show_image, (int(rec_x-0.5*r_w), int(rec_y-0.5*r_h)), (int(rec_x+0.5*r_w), int(rec_y+0.5*r_h)), (255,0,0), -1)
        
          print("minAreaRect x: ", rec_x)
          print("minAreaRect y: ", rec_y)
          cv2.circle(show_image, (int(rec_x), int(rec_y)), 10, (0,50,175), -1)
          cv2.putText(show_image, str((r_w * r_h) / (960*720.)), (10,40), 5, 2, (255,255,0))
         
          if start_detect:
           if r_w * r_h >= width*height*0.35:
            hasPassedTwice+=1
            point_pub.publish(Float64MultiArray(data = [rec_x, rec_y, 1,hasPassedTwice,cammiddle,mission]))
            start_detect=False

           else:
            if r_w * r_h < width*height*0.01:
             cammiddle=0
             rec_x=-1
             rec_y=-1

            point_pub.publish(Float64MultiArray(data = [rec_x, rec_y, 0,hasPassedTwice,cammiddle,mission]))

         ########################################################################################################3

            
        
        out.write(np.concatenate((image, show_image), axis = 1))    
        cv2.imshow('result', np.concatenate((image, show_image), axis = 1))
        cv2.waitKey(1)
        if frame.time_base < 1.0/60:
          time_base = 1.0/60
        else:
          time_base = frame.time_base
        frame_skip = int((time.time() - start_time)/time_base)

if __name__ == '__main__':
    try:
        main()
    except BaseException:
        traceback.print_exc()
    finally:
        stream.close()
        cv2.destroyAllWindows()
