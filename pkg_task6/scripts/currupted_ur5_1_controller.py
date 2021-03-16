#! /usr/bin/env python2.7
"""
This file control ur5_1 all the operations performed by ur5_1 bot.
"""

import rospy
import time
import datetime
from pkg_vb_sim.msg import LogicalCameraImage
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from ur5_moveit.ur5_moveit import Ur5Moveit,define_joint_angle_list,get_item_details
from sensor_msgs.msg import Image
import heapq as hq #heap
from iot_client.iot_client import IotClient
from pkg_task6.msg import OrderDetailArray
import socket #tcp/ip connection
import json




def add_all_boxes(ur5_bot):
    """
    This functions add all boxes in the shelf to the planning scene.
    :param ur5_bot: it is instance of class ur5_moveit.
    :return: null
    """
    pose_x=[.28,0,-.28]
    pose_y= -0.41  # -.41
    pose_z=[1.92,1.66,1.43,1.20]
    for x in range(0,3):
        for z in range(0,4): 
            ur5_bot.add_box(pose_x[x],pose_y,pose_z[z],'box'+str(z)+str(x))


def handle_box_clashing(data):
    global ur5
    global is_box_in_camera
    models_length = len(data.models)
    print("inside ros service")
    if (models_length == 2):
        is_box_in_camera = True
        rospy.sleep(1)    
    else:
        is_box_in_camera = False
        #time.sleep(10)


def handle_bot1():

    """
    Controls ur5_1 pick boxes from the shelf and place them on the belt.
    :return: null
    """

    global ur5
    action_client = IotClient()

    #connet to local server1
    client_multi_socket = socket.socket()
    host = '127.0.0.1'
    port = 2004

    print('Waiting for connection1 response')
    try:
        client_multi_socket.connect((host, port))
    except socket.error as e:
        print(str(e))
    client_multi_socket.recv(1024)
    #----->>>>


    #connet to local server2
    client_multi_socket2 = socket.socket()
    host = '127.0.0.1'
    port2 = 2010

    print('Waiting for connection2 response')
    try:
        client_multi_socket2.connect((host, port2))
    except socket.error as e:
        print(str(e))
    client_multi_socket2.recv(1024)
    #----->>>>

    coordinate_matrix = [

        [
            define_joint_angle_list(161, -120, 12, -83, 20, 0),
            define_joint_angle_list(-124,-91,26,-117,-59,0),    #[01]
            # define_joint_angle_list(-124, -90, 26, -117, -59, 0),
            define_joint_angle_list(56, -115, 2, -70, 123, 0),
        ],
        [
            define_joint_angle_list(-54, -97, 82, -166, -128, 0),
            define_joint_angle_list(-122, -103, 55, 46, 61, 0),
            define_joint_angle_list(55, -84, -82, -17, 123, 0),
        ],
        [
            define_joint_angle_list(-55, -98, 88, 7, 125, 0),
            define_joint_angle_list(-122, -119, 103, 14, 59, 0),
            define_joint_angle_list(52, -85, -83, 173, -130, 0)
        ],
        [
            define_joint_angle_list(-56, -94, 119, -27, 127, 0),
            define_joint_angle_list(-125,-118,135,-17,55,0), #31
            define_joint_angle_list(55,-88,-117,-154,-122,0) #32
            # define_joint_angle_list(-117, -118, 134, -17, 66, 0),
            # define_joint_angle_list(-161, -93, 118, -27, 22, 0)
        ]
    ]

    # coordinate_matrix = [

    #     [
    #         define_joint_angle_list(164, -117, 11, -84, 17, 0),
    #         define_joint_angle_list(-123,-86,14,-106,-62,0),    #[01]
    #         define_joint_angle_list(56, -114, 4, -69, 125, 0),
    #     ],
    #     [
    #         define_joint_angle_list(163, -82, -84, -10, 18, 0),
    #         define_joint_angle_list(123, -62, -96, -21, 58, 0),
    #         define_joint_angle_list(54, -82, -82, -14, 128, 0),
    #     ],
    #     [
    #         define_joint_angle_list(-54, -96, 116, 158, -125, 0),
    #         define_joint_angle_list(122, -58, -131, 9, 59, 0),
    #         define_joint_angle_list(-163, -99, 90, 9, 16, 0)
    #     ],
    #     [
    #         define_joint_angle_list(-54, -93, 120, -27, 125, 0),
    #         define_joint_angle_list(-123,-120,135,-14,55,0), #31
    #         define_joint_angle_list(-163,-96,120,-24,16,0) #32
           
    #     ]
    # ]

    add_all_boxes(ur5)

    #requesting local server to provide inventory data
    msg="inventory"
    client_multi_socket.send(str.encode(msg))
    res = client_multi_socket.recv(1024)
    res =res.encode('utf-8')
    
    bad_char=["{","}","'"]
    for i in bad_char:
        res=res.replace(i,'')
    match_box_color_with_index={}
    l =[]
    for element in res.split(','):
        for a in element.split(':'):
            temp =str(a.strip())
            l.append(temp)    
        #print(json.dumps(l))
        match_box_color_with_index[l[0]]=l[1]
        l=[]            

    # ----editing have to delete those which are not picking----
    # del match_box_color_with_index['32']
    # del match_box_color_with_index['01']
    # del match_box_color_with_index['31']
    

    order_info=[]
    hq.heapify(order_info)
    max_order_id =0
    print(order_info,'Order Info')
    order_no_number_pub = rospy.Publisher('/eyrc/vb/order_number',OrderDetailArray,queue_size=1)
    order_dictionary={}  
    first_iteration = True
    global is_box_in_camera
    print(len(order_info),'Ggg',is_box_in_camera)
    
    while first_iteration or len(order_info)>0:
        camera1_service = rospy.Subscriber('/eyrc/vb/logical_camera_1',LogicalCameraImage,handle_box_clashing,queue_size=1)
        time.sleep(.5)
        camera1_service.unregister()
        if not is_box_in_camera: 
            first_iteration = False
            order_dictionary["max_order_id"]=max_order_id
            order_dictionary["order_info"]=order_info
            msg=json.dumps(order_dictionary)
            client_multi_socket2.send(str.encode(msg))
            res2 = client_multi_socket2.recv(1024)
            print("res: ",res2)
            #res=str(res)
            res2=json.loads(res2)
            order_to_be_procced=res2["order_to_be_procced"]
            max_order_id=res2["max_order_id"]
            order_info=res2["order_info"]

            for key, value in match_box_color_with_index.items():
                if order_to_be_procced[2] == value:
                    box_to_be_pick=key  #key pta chl gyi
                    match_box_color_with_index.pop(key)
                    break
                
            print(order_to_be_procced, max_order_id)
            i =int(box_to_be_pick[0])
            z =int(box_to_be_pick[1])
            print(i,z,': Box\n')
            order_details_sheet = get_item_details(order_to_be_procced[2])
            order_details = OrderDetailArray()
            order_details.order_id = order_to_be_procced[1].encode("utf-8")
            order_details.name = 'packagen'+str(i)+str(z)
            order_details.city = order_to_be_procced[3].encode("utf-8")
            order_no_number_pub.publish(order_details)
            #print(order_details,'string')
            global vaccum
            status = ur5.hard_set_joint_angles(coordinate_matrix[i][z],4)
            print(status,"STATUS")
            while not status:
                ur5.hard_set_joint_angles(define_joint_angle_list(0, -90, 0, 0, 0, 0),4) #straight up
                status = ur5.hard_set_joint_angles(coordinate_matrix[i][z],4)
            ur5.attach_box('box'+str(i)+str(z))
            # attach_box_thread(True)            

            #checking if somewhere else ros service is being use
            ros_service_status ="busy"
            while ros_service_status=="busy":
                msg="status_check"
                client_multi_socket.send(str.encode(msg))
                res = client_multi_socket.recv(1024)
                ros_service_status=str(res.decode('utf-8'))
                #print(res.decode('utf-8'))
                if ros_service_status=="busy":
                    rospy.sleep(1)
                else:
                    msg="busy"
                    client_multi_socket.send(str.encode(msg))
                    rospy.sleep(.2)
                    break
    #---->>>>>>>

            y = vaccum(True)
            #print("Vaccum Gripper : ",y)
            rospy.sleep(.4)
            msg="free" #test
            client_multi_socket.send(str.encode(msg)) #test
            ur5.ee_cartesian_translation(0,1.4, 0)
            ur5.hard_set_joint_angles(define_joint_angle_list(179,-57,86,-119,-88,0),4)
            ur5.detach_box('box'+str(i)+str(z))

            #testing--->>>
            ros_service_status ="busy"
            while ros_service_status=="busy":
                msg="status_check"
                client_multi_socket.send(str.encode(msg))
                res = client_multi_socket.recv(1024)
                ros_service_status=str(res.decode('utf-8'))
                #print(res.decode('utf-8'))
                if ros_service_status=="busy":
                    rospy.sleep(1)
                else:
                    msg="busy"
                    client_multi_socket.send(str.encode(msg))
                    rospy.sleep(.2)
                    break
    #---->>>>>>>
            y = vaccum(False)   
            #print("Vaccum Gripper: ",y)
            rospy.sleep(.4)
            msg="free" #test
            client_multi_socket.send(str.encode(msg)) #test

            ur5.remove_box('box'+str(i)+str(z))
            dispatch_date = datetime.datetime.now()
            dispatch_time_str = dispatch_date.strftime("%a")+' '+dispatch_date.strftime("%b")+' '+dispatch_date.strftime("%d")+' '+dispatch_date.strftime("%Y")+' - '+dispatch_date.strftime("%X")
            print(dispatch_time_str,'DispatchTime')
            dipatch_order_string = 'OrdersDispatched,'+order_to_be_procced[1].encode("utf-8")+','+order_to_be_procced[3].encode("utf-8")+','+order_details_sheet['item']+','+order_details_sheet['priority']+',1,'+order_details_sheet['cost']+',Yes,'+dispatch_time_str
            goal_handle1 = action_client.send_goal("mqtt", "pub", action_client.config_mqtt_pub_topic, dipatch_order_string)
            action_client.goal_handles['1'] = goal_handle1
            #del ur5

        else:
            rospy.sleep(1)



def main():
    rospy.init_node('node_task5_ur5_1', anonymous=True)
    global ur5 
    global is_box_in_camera
    is_box_in_camera = False
    ur5 = Ur5Moveit('ur5_1')
    #rospy.Subscriber('/eyrc/vb/logical_camera_1',LogicalCameraImage,handle_conveyor,queue_size=1)
    global vaccum
    vaccum = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
     
    handle_bot1()
    rospy.spin()

if __name__ == '__main__':
    global ur5
    global vaccum
    global is_box_in_camera
    main()

