#! /usr/bin/env python2.7

import rospy

from pkg_vb_sim.msg import LogicalCameraImage
from pkg_vb_sim.srv import conveyorBeltPowerMsg
    

def trigger_converbelt(value):
    x = rospy.ServiceProxy('eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
    y = x(value)
    global is_conver_active
    is_conver_active = value

def hangle_conveyor(data):
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
        if(y < 0.1 and package != "ur5") :#--pkg1,2,3
            trigger_converbelt(0)
            should_start_conver = True
            rospy.loginfo('\033[94m' + "Ye Wala" + '\033[0m')

        #-arshan --else if ony ur5 found
        else:
            trigger_converbelt(100) #Running Belt

    else:
        rospy.loginfo('\033[96m' + "Waiting for Box" + '\033[0m')
        if(not is_conver_active and should_start_conver):
            # bot pose command ---
            trigger_converbelt(100)




def main():
    rospy.init_node('node_task4_belt_controller')
    global t
    global freq
    global is_conver_active
    global should_start_conver
    should_start_conver = True
    freq = 0
    trigger_converbelt(100) #--testing for parallelism
    t = rospy.Subscriber('/eyrc/vb/logical_camera_2',LogicalCameraImage,hangle_conveyor,queue_size=1)
    rospy.spin()
    
if __name__ == '__main__':
    main()
