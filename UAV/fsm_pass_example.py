#!/usr/bin/env python
import rospy
import simple_tello

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from statemachine import StateMachine, State
from time import sleep

t1 = simple_tello.Tello_drone()
height=720
fakey=170


rdH=int(height/3)
#halfrdH=int(1/2*(rdH))
#traceH=rdH+halfrdH


#width=960
#3rdW=int(width/3)
#half3rdW=int(1/2*(3rdW))
#traceW=3rdW+half3rdW
#dx = t1.state.target_x - half3rdW




class sMachine(StateMachine):
    
    ##################################################
    # state
    hover = State('Hover', initial = True)
    correction = State('Correction')
    forward = State('Forward')

    ##passing rapidly
    addSp = State('AddSp')
    #########################################
    rotate=State('Rotate')
    #################################################
    trace=State('Trace')
    traceback=State('Traceback')
    


    ##################################################################3
    ## trans #define how state is transfeered
    ## | is or : when need2correct is called , if your current state is forward, it will move to forward state. if your current state is correction, it will move to correction state
    ######################################################
    stop2do = addSp.to(hover) | traceback.to(hover)
    refly = rotate.to(hover)
    need2rotate = rotate.to(rotate) | hover.to(rotate) | correction.to(rotate) | forward.to(rotate)    #addSp.to(rotate) |
    #####################################################
    wait4data = hover.to(hover)
    start2correct = hover.to(correction)
    need2correct = correction.to(correction) | forward.to(correction)
    start2forawrd = hover.to(forward)
    need2forawrd = correction.to(forward) | forward.to(forward)
    need2addSp = forward.to(addSp) | correction.to(addSp)
    #######################################################################
    need2trace = hover.to(trace) | trace.to(trace)
    need2traceback= trace.to(traceback) | traceback.to(traceback)

  





    #define what you will do when you enter the state
    #the named is followung certain pattern vs defined in StateMachine class
    #if you want to turn to blue after passing through, you have to define one more state


    def on_enter_hover(self):
      msg = Twist()
      t1.controler.move(msg, 0.5)

    def on_enter_correction(self):
      msg = Twist()
      dx = t1.state.target_x - 480
      dy = t1.state.target_y - fakey
      if dx != 0:
        msg.linear.x = dx / abs(dx) * 0.1
      if dy != 0:
        msg.linear.z = -dy / abs(dy) * 0.2
      t1.controler.move(msg, 0.5)

    def on_enter_forward(self):
      msg = Twist()
      msg.linear.y = 0.2
      t1.controler.move(msg, 0.5)

    def on_enter_addSp(self):
      msg = Twist()
      msg.linear.y = 0.4
      t1.controler.move(msg, 4)
      
      msg = Twist()
      msg.linear.y = 0.5
      t1.controler.move(msg, 4.3)

    
    def on_enter_rotate(self):
      msg = Twist()
      msg.angular.z = 0.4
      t1.controler.move(msg, 0.6)
    ############################################################
    def on_enter_trace(self):
      msg = Twist()
      #traceH=rdH*2
      traceH=height/2
      dy = t1.state.target_y -(traceH+20)
      msg.linear.x = -0.1
      if dy != 0:
       msg.linear.z = -dy / abs(dy) * 0.1
      t1.controler.move(msg, 0.5)
    ############################################################3
    def on_enter_traceback(self):
     msg = Twist()
     #traceH=rdH
     traceH=height/2
     dy = t1.state.target_y -traceH
     msg.linear.x = 0.1
     if dy != 0:
      msg.linear.z = -dy / abs(dy) * 0.1
     t1.controler.move(msg, 0.5)

class MyModel(object):
    def __init__(self, state):
        self.state = state
                
    def run(self, fsm):
        while not rospy.is_shutdown():
            print(self.state)
            
            ##############################
            if fsm.is_trace:
             if t1.state.mission==2:
              fsm.need2trace()
             else: #3
              fsm.need2traceback()
            ########
            
            elif fsm.is_traceback:
             if t1.state.mission==3:
              fsm.need2traceback()
             else:
              fsm.stop2do()
            ###################################3

            elif fsm.is_hover:

                
                if t1.state.mission==2:
                 fsm.need2trace()
                elif t1.state.target_x == -1 and t1.state.target_y == -1:
                 if t1.state.hasPassedTwice==0:
                  fsm.wait4data()
                 else:
                  fsm.need2rotate()

                else:
                  dx = t1.state.target_x - 480
                  dy = t1.state.target_y - fakey
                  if t1.state.cammiddle == 1:
                   if abs(dx) < 30 and abs(dy) < 30:
                    fsm.start2forawrd()
                   else:
                    fsm.start2correct()
                  
                  else:
                   fsm.need2rotate()


            elif fsm.is_correction:
                
                if t1.state.canPass == 1:
                    fsm.need2addSp()
                else:
                    dx = t1.state.target_x - 480
                    dy = t1.state.target_y - fakey
                    
                    if t1.state.cammiddle == 1:  
                     if abs(dx) < 30 and abs(dy) < 30:
                      fsm.need2forawrd()
                     else:
                      fsm.need2correct()
                    else:
                     fsm.need2rotate()
                    

            elif fsm.is_forward:
                if t1.state.canPass == 1:
                    fsm.need2addSp()
                else:
                    dx = t1.state.target_x - 480
                    dy = t1.state.target_y - fakey

                    if t1.state.cammiddle == 1:
                     if abs(dx) < 30 and abs(dy) < 30:
                      fsm.need2forawrd()
                     else:
                      fsm.need2correct()
                    
                    else:
                     fsm.need2rotate()

            #########################################################################
            elif fsm.is_addSp:
                if t1.state.hasPassedTwice == 2:
                    break
                else:
                    fsm.stop2do()

            elif fsm.is_rotate:
                if t1.state.cammiddle == 0:
                    fsm.need2rotate()
                else:
                    fsm.refly()
            #############################################################################
                


###################################################3
#hasPassedTwice in simple_tello.py and test_h264_sub.py
#t1.state.cammidlle in simple_tello.py and test_h264_sub.py


#################################################################

def main():
  
  
  while t1.state.is_flying == False: 
    t1.controler.takeoff()
  
  while t1.state.fly_mode != 6:
    print("wait...")
  
  obj = MyModel(state='hover')
  fsm = sMachine(obj)
  obj.run(fsm)
  
  while t1.state.fly_mode != 6:
    print("wait...") 
    
  while t1.state.is_flying == True:
    t1.controler.land() 

if __name__ == '__main__':
    rospy.init_node('h264_pub', anonymous=True)
    main()


