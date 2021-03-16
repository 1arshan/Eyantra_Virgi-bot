#! /usr/bin/env python2.7
"""
This file control ur5_2, handle conveyor belt and placement of boxes from belt to bin.
"""
import rospy
import time
import datetime
from camera_one.camera_one import Camera1,get_item_details
from ur5_moveit.ur5_moveit import Ur5Moveit,define_pose,define_joint_angle_list
from iot_client.iot_client import IotClient
from pkg_vb_sim.msg import LogicalCameraImage
from pkg_vb_sim.srv import vacuumGripper
from sensor_msgs.msg import Image
from pkg_task6.msg import OrderDetailArray
import socket #tcp/ip connection


def get_time_str(estimated_time_of_delivery):
    """
    This function return a time string of now plus estimated_time_of_delivery days
    :param estimated_time_of_delivery: Number of days to be added in time right now.
    :return: String of time.
    """
    x = datetime.datetime.now() + datetime.timedelta(days=int(estimated_time_of_delivery))

    return x.strftime("%a")+' '+x.strftime("%b")+' '+x.strftime("%d")+' '+x.strftime("%Y")+' - '+x.strftime("%X")


def bot_2_controller(data):
    """
    This function control complete functioning of ur5_2
    Pick and Placement operation of boxes and belt start and stop operations.
    :param data: It contains LogicalCameraImage information published from topic /eyrc/vb/logical_camera_2
    :return:  null
    """
    models_length = len(data.models)
    global box_place_list
    global should_start_conver
    global ur5
    global l
    global dispatched_orders
    global ic
    global action_client

    global client_multi_socket

    if models_length > 0:
        package_color=''
        if models_length == 1:
            x = data.models[0].pose.position.x
            y = data.models[0].pose.position.y
            package = data.models[0].type
            print("package name: ",package)
            #condition_breaker =1
        else:    
            x = data.models[1].pose.position.x
            y = data.models[1].pose.position.y
            print(data.models)
            package = data.models[1].type
            print(package)

            if package != "ur5":
                package_color=l[package]
                print("package color: ",package_color)
                msg="busy"
                client_multi_socket.send(str.encode(msg))

        # ur5.set_joint_angles(define_joint_angle_list(168.661264258,-50,50.3742109357,-97.2680961679,-90.0263818056,-11.2957158804))
        if y < 0.1 and package != "ur5":#--pkg1,2,3
            #condition_breaker =1
            # ur5.change_go_to_home()
            # thread = threading.Thread(target=trigger_converbelt,args=(0,))

            ur5.trigger_converbelt(0)  # has to take care
            rospy.sleep(.4)
            msg="free" #test
            client_multi_socket.send(str.encode(msg)) #test
            # thread.start()
            if package in box_place_list:
                print("returning")
                return 1
            gazebo_y = round(y,3)
            constant_x=1.00500025103
            constant_diff=0.00000018366
            # constant_diff=0.0000003163
            
            multiplier=(x-constant_x)/constant_diff
            multiplier=round(multiplier,1)
            print(multiplier)
            #gazebo_x =-(0.8+.05*multiplier)
            gazebo_x= -0.686        
            box_length = 0.15               # Length of the Package
            vacuum_gripper_width = 0.115    # Vacuum Gripper Width
            delta = vacuum_gripper_width + (box_length/2)  # 0.19
            # global vaccum     
            print(delta)
            #gazebo_x =-0.69
            # Teams may use this info in Tasks

            global freq
            ur5.add_box(gazebo_x,gazebo_y, 0.99,'box')
            #ur5.add_box(gazebo_x,gazebo_y, 0.99)
            ur5.hard_go_to_pose(define_pose(gazebo_x,gazebo_y,(1 + vacuum_gripper_width + (box_length/2))),4)

            #testing--->>>
            ros_service_status ="busy"
            while ros_service_status=="busy":
                msg="status_check"
                client_multi_socket.send(str.encode(msg))
                res = client_multi_socket.recv(1024)
                ros_service_status=str(res.decode('utf-8'))
                print(res.decode('utf-8'))
                if ros_service_status=="busy":
                    rospy.sleep(1)
                else:
                    msg="busy"
                    client_multi_socket.send(str.encode(msg))
                    rospy.sleep(.2)
                    break
#---->>>>>>>


            newl = vaccum(True)
            print(newl)
            # attach_box_thread(True)
            rospy.sleep(.4)
            msg="free" #test
            client_multi_socket.send(str.encode(msg)) #test
            
            rospy.loginfo('\033[94m' + "Translating EE by 0.5m in x from current position." + '\033[0m')

            if package_color == "red":
            #----Coordinate of red bin----->>>>>>>>>>>
                ur5.attach_box('box')
                ur5.ee_cartesian_translation(0,0,0.2)
                isDone = ur5.hard_go_to_pose(define_pose(0.2,0.6,1.265),4)
                print(isDone,'IsDone')
                if not isDone:
                    ur5.detach_box('box')
                    ur5.remove_box('box')

                    # ur5.set_joint_angles(define_joint_angle_list(179,-57,86,-119,-88,0),4)
                    ur5.set_joint_angles(define_joint_angle_list(168.661264258,-50,50.3742109357,-97.2680961679,-90.0263818056,-11.2957158804))
                    box_place_list.append(package)    
                
                #newt = vaccum(False)
                # attach_box_thread(False)

                # ur5.hard_set_joint_angles(define_joint_angle_list(61.6301577746,19.9056122889,-90.7433259006,-19.1991556074,-90.037481863,-118.422844125),4)
                rospy.loginfo('\033[96m' + "Successfully Placed Red Box" + '\033[0m')
            elif package_color == "green":
                #----Coordinate of green bin----->>>>>>>>>>>
                ur5.attach_box('box')
                ur5.ee_cartesian_translation(0,0,0.2)
                list_joint_values =ur5.group.get_current_joint_values()
                rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
                rospy.loginfo(list_joint_values)
                #print(list_joint_values)
                
                isDone = ur5.hard_go_to_pose(define_pose(0.2,-0.65,1.265),4)
                print(isDone,'IsDone')
                if not isDone:
                    ur5.detach_box('box')
                    ur5.remove_box('box')
                    ur5.set_joint_angles(define_joint_angle_list(168.661264258,-50,50.3742109357,-97.2680961679,-90.0263818056,-11.2957158804))
                    box_place_list.append(package)    
                
                #newt = vaccum(False)
                # attach_box_thread(False)
                # ur5.hard_set_joint_angles(define_joint_angle_list(-82.1363218895,-60.1866792126,81.5471504912,-111.31441537,-89.9371709008,97.8410717962),4)
                                    #ur5.hard_set_joint_angles(define_joint_angle_list(-8.97200920273,-57.5998250621,77.4253084494,-109.817161576,-90.0504541341,170.994315015),4)
                rospy.loginfo('\033[96m' + "Successfully Placed Green Box" + '\033[0m')
            elif package_color == "yellow":
                #----Coordinate of yellow bin----->>>>>>>>>>>
                ur5.attach_box('box')
                ur5.ee_cartesian_translation(0,0,0.2)
                isDone = ur5.hard_go_to_pose(define_pose(0.7,0,1.265),4)
                print(isDone,'IsDone')
                if not isDone:
                    ur5.detach_box('box')
                    ur5.remove_box('box')
                    ur5.set_joint_angles(define_joint_angle_list(168.661264258,-50,50.3742109357,-97.2680961679,-90.0263818056,-11.2957158804))
                    box_place_list.append(package)    
                
                #newt = vaccum(False)
                # attach_box_thread(False)
            # ur5.hard_set_joint_angles(define_joint_angle_list(-8.97200920273,-57.5998250621,77.4253084494,-109.817161576,-90.0504541341,170.994315015),4)
                                            #ur5.hard_set_joint_angles(define_joint_angle_list(-82.1363218895,-60.1866792126,81.5471504912,-111.31441537,-89.9371709008,97.8410717962),4)
                rospy.loginfo('\033[96m' + "Successfully Placed Yellow Box" + '\033[0m')
            
            else:
                rospy.loginfo('\033[96m' + "Box Type Unknown" + '\033[0m')
            
            #testing--->>>
            ros_service_status ="busy"
            while ros_service_status=="busy":
                msg="status_check"
                client_multi_socket.send(str.encode(msg))
                res = client_multi_socket.recv(1024)
                ros_service_status=str(res.decode('utf-8'))
                print(res.decode('utf-8'))
                if ros_service_status=="busy":
                    rospy.sleep(1)
                else:
                    msg="busy"
                    client_multi_socket.send(str.encode(msg))
                    rospy.sleep(.2)
                    break
#---->>>>>>>


            newt = vaccum(False)
            print(newt)
            rospy.sleep(.4)
            msg="free" #test
            client_multi_socket.send(str.encode(msg)) #test

            ur5.detach_box('box')
            ur5.remove_box('box')
            box_place_list.append(package)
            should_start_conver = True
            order_id = dispatched_orders[package]['id']
            order_city = dispatched_orders[package]['city']
            order_details = get_item_details(package_color)
            # {'item':'Medicines','sku_alpha':'R','priority':'HP','cost':'450','Estimated Time of Delivery':'1'}
            print(order_id,order_details,'details')
            shipped_str = 'OrdersShipped,'+order_id+','+order_city+','+order_details['item']+','+order_details['priority']+',1,'+order_details['cost']+',Yes,'+get_time_str(0)+','+get_time_str(order_details['Estimated Time of Delivery'])+','+order_details['Estimated Time of Delivery']
            print(shipped_str,'string ship')
            goal_handle1 = action_client.send_goal("mqtt", "pub", action_client.config_mqtt_pub_topic, shipped_str)
            action_client.goal_handles['1'] = goal_handle1
            #rospy.sleep(3)
        else:
            if models_length<3:
                print("need to look here; package name: ",package)
                #ur5.trigger_converbelt(75) #trigger_converbelt(100)
            print("need to look here; package name: ",package)
    else:
        
        ur5.set_joint_angles(define_joint_angle_list(168.661264258,-50,50.3742109357,-97.2680961679,-90.0263818056,-11.2957158804))
        if not ur5.is_conver_active and should_start_conver:
            # bot pose command ---
            
            #testing--->>>
            ros_service_status ="busy"
            while ros_service_status=="busy":
                msg="status_check"
                client_multi_socket.send(str.encode(msg))
                res = client_multi_socket.recv(1024)
                ros_service_status=str(res.decode('utf-8'))
                print(res.decode('utf-8'))
                if ros_service_status=="busy":
                    rospy.sleep(1)
                else:
                    msg="busy"
                    client_multi_socket.send(str.encode(msg))
                    rospy.sleep(.2)
                    break
#---->>>>>>>

            ur5.trigger_converbelt(90)
            msg="free" #test
            client_multi_socket.send(str.encode(msg)) #test



def match_boxcolor_with_no(data):
    global ic
    ic.callback(data)
    inv = ic.inventory
    global action_client
    for str in inv:
        time.sleep(4)
        goal_handle1 = action_client.send_goal("mqtt", "pub", action_client.config_mqtt_pub_topic, str)
        action_client.goal_handles['1'] = goal_handle1
        

    print("func")

def get_order_number(data):
    """
    Callback function
    It publishes data to the topic /eyrc/vb/order_number
    :param data: It is a msg containing order_id,city and name.
    :return: null
    """
    order_detail = data
    global dispatched_orders
    dispatched_orders.update({order_detail.name:{'id':order_detail.order_id,'city':order_detail.city}})
    print(dispatched_orders)

def main():
    global ic 
    rospy.init_node('ur5_2_controller',anonymous=True)
    global action_client
    action_client = IotClient()
    global temp
    global dispatched_orders
    dispatched_orders = {}   
    temp = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,match_boxcolor_with_no,queue_size=1)
    ic= Camera1(temp)
    
    rospy.sleep(1)
    
    global l #list
    l = ic.list
    print(l,'list')

    global freq
    global ur5
    global vaccum
    vaccum = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
    global should_start_conver
    should_start_conver = True
    ur5 = Ur5Moveit('ur5_2')
    freq = 0
    global box_place_list
    box_place_list =[]

    ur5.trigger_converbelt(90) #--testing for parallelism
    
    global client_multi_socket 
    #testing
    client_multi_socket = socket.socket()
    host = '127.0.0.1'
    port = 2004

    print('Waiting for connection response')
    while True:
        try:
            client_multi_socket.connect((host, port))
            break
        except socket.error as e:
            #print(str(e))
            print("please start local server")
            rospy.sleep(1.5)
    client_multi_socket.recv(1024)
    #----->>>>

    ur5.set_joint_angles(
            define_joint_angle_list(168.661264258, -50, 50.3742109357, -97.2680961679, -90.0263818056, -11.2957158804))
    rospy.Subscriber('/eyrc/vb/logical_camera_2',LogicalCameraImage,bot_2_controller,queue_size=1)
    rospy.Subscriber('/eyrc/vb/order_number',OrderDetailArray,get_order_number,queue_size=1)    
    
    rospy.spin()
    
if __name__ == '__main__':
    global box_place_list
    global should_start_conver
    global ur5
    global l
    global temp
    global vaccum
    global dispatched_orders
    global ic
    global client_multi_socket
    global freq
    global action_client

    main()