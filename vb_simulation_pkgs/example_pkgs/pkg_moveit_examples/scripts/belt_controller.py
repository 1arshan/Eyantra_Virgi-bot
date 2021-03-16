#! /usr/bin/env python2.7

import rospy
import sys
import copy
import math

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.msg import LogicalCameraImage
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_moveit_examples.msg import DelayRosService,Bot1Running,Bot2Running

def trigger_converbelt(value):
    belt_controller = rospy.ServiceProxy('eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
    belt_power_value = belt_controller(value)
    global is_conver_active
    is_conver_active = value

def func_callback_topic_bot1_running(myMsg):
    global conveyerBeltStop
    global conveyerBeltRunning

    if myMsg.message == "busy":
        if conveyerBeltRunning==False and conveyerBeltStop ==True:
            rospy.sleep(1)

    #rospy.loginfo("Data Received: ",myMsg.name)
    print("Data receive(bot1): ",myMsg.message)

def func_callback_topic_bot2_running(myMsg):
    global conveyerBeltStop
    global conveyerBeltRunning
    if myMsg.message == "busy":
        if conveyerBeltRunning==False and conveyerBeltStop ==True:
            rospy.sleep(1)
    #rospy.loginfo("Data Received: ",myMsg.name)
    print("Data receive(bot2): ",myMsg.message)



def hangle_conveyor(data):
    var_handle_pub = rospy.Publisher('my_topic', DelayRosService, queue_size=1) #new
    rospy.Subscriber("bot1_running",Bot1Running, func_callback_topic_bot1_running,queue_size=1)
    rospy.Subscriber("bot2_running",Bot2Running, func_callback_topic_bot2_running,queue_size=1)
    obj_msg = DelayRosService()  #new
    obj_msg.name="busy" #new

    global conveyerBeltStop
    global conveyerBeltRunning
    global should_start_conver
    print(data.models)
    models_length = len(data.models)
    if (models_length > 0):
        should_start_conver = False
        if(models_length == 1):
            x = data.models[0].pose.position.x
            y = data.models[0].pose.position.y
            package = data.models[0].type
        else:    
            x = data.models[1].pose.position.x
            y = data.models[1].pose.position.y
            package = data.models[1].type

            if conveyerBeltStop==False:
                obj_msg.name="busy"  #new
                var_handle_pub.publish(obj_msg)  #new

        if(y < 0.1 and package != "ur5" and conveyerBeltStop ==False) :#--pkg1,2,3
            obj_msg.name="busy" #new
            var_handle_pub.publish(obj_msg) #new
            #rospy.sleep(.5)
            #var_handle_pub.publish(obj_msg)
            trigger_converbelt(0)
            conveyerBeltStop =True
            conveyerBeltRunning=False
            print("msg sent")
            should_start_conver = True
            #obj_msg.name="free" #new
            #var_handle_pub.publish(obj_msg)
            rospy.sleep(2)
            rospy.loginfo('\033[94m' + "Ye Wala" + '\033[0m')

        #-arshan --else if ony ur5 found
        else:
            conveyerBeltStop =False
            if conveyerBeltRunning ==True:
                print("belt running")
            else:
                obj_msg.name="busy" #new
                var_handle_pub.publish(obj_msg)
                conveyerBeltRunning =True
                rospy.sleep(.5)
                obj_msg.name="busy"  #new
                var_handle_pub.publish(obj_msg)
                trigger_converbelt(75) #Running Belt
                print("msg sent")
                #obj_msg.name="free" #new
                #var_handle_pub.publish(obj_msg)

    else:
        rospy.loginfo('\033[96m' + "Waiting for Box" + '\033[0m')
        if(not is_conver_active and should_start_conver):
            # bot pose command ---
            conveyerBeltStop =False
            if conveyerBeltRunning ==True:
                print("belt running")
            else:
                obj_msg.name="busy" #new
                var_handle_pub.publish(obj_msg)
                rospy.sleep(.5)
                obj_msg.name="busy"  #new
                var_handle_pub.publish(obj_msg)
                trigger_converbelt(75)
                conveyerBeltRunning =True

                print("msg sent")
                #obj_msg.name="free" #new
                #var_handle_pub.publish(obj_msg)




def main():
    rospy.init_node('node_task3_belt_controller')
    global t
    global freq
    #global x
    global is_conver_active
    global should_start_conver
    should_start_conver = True
    freq = 0
    global conveyerBeltRunning
    global conveyerBeltStop
    conveyerBeltRunning =True
    conveyerBeltStop =False
    trigger_converbelt(100) #--testing for parallelism
    t = rospy.Subscriber('/eyrc/vb/logical_camera_2',LogicalCameraImage,hangle_conveyor,queue_size=1)
    rospy.spin()
    
if __name__ == '__main__':
    main()
