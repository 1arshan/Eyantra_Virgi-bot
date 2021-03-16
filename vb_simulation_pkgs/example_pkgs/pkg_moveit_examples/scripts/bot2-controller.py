#! /usr/bin/env python2.7

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import rospkg
from pkg_vb_sim.msg import LogicalCameraImage
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import vacuumGripper
import yaml
import os
import math
import time
import sys
import copy
from std_srvs.srv import Empty
import cv2
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
from pkg_moveit_examples.msg import DelayRosService,Bot1Running,Bot2Running



class Ur5Moveit:

    # Constructor
    def __init__(self, arg_robot_name):

        self._box_name = 'box'
        self.box_name = 'box'
        self._robot_ns = '/'  + arg_robot_name
        self._planning_group = "manipulator"
        
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        


        # Attribute to store computed trajectory by the planner	
        self._computed_plan = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')


        #rp = rospkg.RosPack()
        #self._pkg_path = rp.get_path('pkg_moveit_examples')
        #self._file_path = self._pkg_path + '/config/saved_trajectories/'
        #rospy.loginfo( "Package Path: {}".format(self._file_path) )


        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        # 1. Create a empty list to hold waypoints
        waypoints = []

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)  
        wpose.position.y = waypoints[0].position.y + (trans_y)  
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5


        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))


        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")

        # The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
        # https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488
        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
        box_name = self.box_name
        scene = self._scene

    
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
          # Test if the box is in attached objects
          attached_objects = scene.get_attached_objects([box_name])
          is_attached = len(attached_objects.keys()) > 0

          # Test if the box is in the scene.
          # Note that attaching the box will remove it from known_objects
          is_known = box_name in scene.get_known_object_names()

          # Test if we are in the expected state
          if (box_is_attached == is_attached) and (box_is_known == is_known):
            print("box is attached")
            return True

      # Sleep so that we give other threads time on the processor
        rospy.sleep(0.1)
        seconds = rospy.get_time()

     # If we exited the while loop without returning then we timed out
        return False
    ## END_SUB_TUTORIAL


    def add_box(self,x,y,z):
   
        box_name = self.box_name
        scene = self._scene

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 0.0
        box_pose.pose.position.x = x #0.02 
        box_pose.pose.position.y = y # 0.45 
        box_pose.pose.position.z = z 
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.15, 0.15, 0.15))

       
        self.box_name=box_name
     
        return self.wait_for_state_update(box_is_known=True, timeout=4)
    
    def attach_box(self, timeout=4):
    
        box_name = self.box_name
        robot = self._robot
        scene = self._scene
        eef_link = self._eef_link

    
        grasping_group = self._planning_group
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)

        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)
   
   
    def detach_box(self, timeout=4):
        box_name = self.box_name
        scene = self._scene
        eef_link = self._eef_link

        scene.remove_attached_object(eef_link, name=box_name)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)


    def remove_box(self, timeout=4):
        box_name = self.box_name
        scene = self._scene

        scene.remove_world_object(box_name)

        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo('\033[94m' + ">>> go_to_pose_angle() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def hard_go_to_pose(self, arg_pose, arg_max_attempts):

        number_attempts = 0
        flag_success = False
        
        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            flag_success = self.go_to_pose(arg_pose)
            rospy.logwarn("attempts: {}".format(number_attempts) )


    def clear_octomap(self):
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        # rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        # rospy.loginfo(pose_values)

        if (flag_plan == True):
            pass
            # rospy.loginfo(
            #     '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            pass
            # rospy.logerr(
            #     '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

        number_attempts = 0
        flag_success = False
        
        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # self.clear_octomap()


    # Destructor
"""
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')
"""

def define_joint_angle_list(shoulder_pan_joint,shoulder_lift_joint,elbow_joint,wrist_1_joint,wrist_2_joint,wrist_3_joint):
    joint_angles = [math.radians(shoulder_pan_joint),
                          math.radians(shoulder_lift_joint),
                          math.radians(elbow_joint),
                          math.radians(wrist_1_joint),
                          math.radians(wrist_2_joint),
                          math.radians(wrist_3_joint)]
    return joint_angles



def hangle_conveyor(data):
    global should_start_conver
    models_length = len(data.models)
    if (models_length > 0):
        if(models_length == 1):
            x = data.models[0].pose.position.x
            y = data.models[0].pose.position.y
            package = data.models[0].type
        else:    
            x = data.models[1].pose.position.x
            y = data.models[1].pose.position.y
            package = data.models[1].type    
        if(package != "ur5") :#--pkg1,2,3
            should_start_conver = False
            # rospy.loginfo('\033[94m' + "Ye Wala" + '\033[0m')

        #-arshan --else if ony ur5 found
        

def define_pose(x,y,z):
    ur5_pose = geometry_msgs.msg.Pose()
    ur5_pose.position.x = x
    ur5_pose.position.y = y
    ur5_pose.position.z = z
    ur5_pose.orientation.x = -0.5
    ur5_pose.orientation.y = -0.5
    ur5_pose.orientation.z = 0.5
    ur5_pose.orientation.w = 0.5
    return ur5_pose



def trigger_converbelt(value):
    x = rospy.ServiceProxy('eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
    y = x(value)
    global is_conver_active
    is_conver_active = value
 

def func_callback_topic_my_topic(myMsg):

    if myMsg.name == "busy":
        rospy.sleep(1)
    #rospy.loginfo("Data Received: ",myMsg.name)
    print("Data receive:(belt) :",myMsg.name)


def func_callback_topic_bot1_running(myMsg):

    if myMsg.message == "busy":
        rospy.sleep(1.2)
    #rospy.loginfo("Data Received: ",myMsg.name)
    print("Data receive(bot1): ",myMsg.message)



def get_coordinate234(data):
    rospy.Subscriber("my_topic", DelayRosService, func_callback_topic_my_topic,queue_size=1)
    rospy.Subscriber("bot1_running",Bot1Running, func_callback_topic_bot1_running,queue_size=1)
    var_handle_pub = rospy.Publisher('bot2_running',Bot2Running, queue_size=1) #new
    obj_msg =Bot2Running()  #new
    obj_msg.message="busy" #new

    models_length = len(data.models)
    global box_place_list
    if (models_length > 0):
        if(models_length == 1):
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
            package_color=l[package]
            #package_index_down=int(package[slice(8,9,1)])
            #package_index_left=int(package[slice(9,10,1)])
            #print("index: ",package_index_down,package_index_left)    
            #package=l[package_index_down*3+package_index_left]
            print("package color: ",package_color)
            #condition_breaker =0

        if(y < 0.1 and package != "ur5") :#--pkg1,2,3
            #condition_breaker =1
            if package in box_place_list:
                print("returning")
                return 1
            gazebo_y = round(y,3)
            constant_x=1.00500025103
            constant_diff=0.00000018366
            # constant_diff=0.0000003163
            
            multiplier=(x-constant_x)/constant_diff
            multiplier=round(multiplier,1)
            #gazebo_x =-(0.8+.05*multiplier)
            gazebo_x= -0.686        
            box_length = 0.15               # Length of the Package
            vacuum_gripper_width = 0.115    # Vacuum Gripper Width
            delta = vacuum_gripper_width + (box_length/2)  # 0.19
            vaccum = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)     

            global ur5
            #gazebo_x =-0.69
            # Teams may use this info in Tasks

            ur5_2_home_pose = geometry_msgs.msg.Pose()
            ur5_2_home_pose.position.x = gazebo_x       # changign x acc to logical camera
            ur5_2_home_pose.position.y = gazebo_y
            ur5_2_home_pose.position.z = 1 + vacuum_gripper_width + (box_length/2)
            ur5_2_home_pose.orientation.x = -0.5
            ur5_2_home_pose.orientation.y = -0.5
            ur5_2_home_pose.orientation.z = 0.5
            ur5_2_home_pose.orientation.w = 0.5

            global freq
            ur5.add_box(gazebo_x,gazebo_y, 0.99)
            #ur5.add_box(gazebo_x,gazebo_y, 0.99)
            ur5.hard_go_to_pose(ur5_2_home_pose,4)
            obj_msg.message="busy" #new
            var_handle_pub.publish(obj_msg) #new
            print("msg sent")
            rospy.sleep(.55)
            var_handle_pub.publish(obj_msg) #new
            newt = vaccum(True)

            rospy.loginfo('\033[94m' + "Translating EE by 0.5m in x from current position." + '\033[0m')

            if(package_color == "red"):
            #----Coordinate of red bin----->>>>>>>>>>>
                ur5.attach_box()
                ur5.ee_cartesian_translation(0,0,0.2)
                ur5.hard_go_to_pose(define_pose(0.2,0.6,1.265),4)
                ur5.detach_box()
                #newt = vaccum(False)

                # ur5.hard_set_joint_angles(define_joint_angle_list(61.6301577746,19.9056122889,-90.7433259006,-19.1991556074,-90.037481863,-118.422844125),4)
                rospy.loginfo('\033[96m' + "Successfully Placed Red Box" + '\033[0m')
            elif(package_color == "green"):
                #----Coordinate of green bin----->>>>>>>>>>>
                ur5.attach_box()
                ur5.ee_cartesian_translation(0,0,0.2)
                list_joint_values =ur5._group.get_current_joint_values()
                rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
                rospy.loginfo(list_joint_values)
                #print(list_joint_values)
                ur5.hard_go_to_pose(define_pose(0.2,-0.65,1.265),4)
                ur5.detach_box()
                #newt = vaccum(False)

                # ur5.hard_set_joint_angles(define_joint_angle_list(-82.1363218895,-60.1866792126,81.5471504912,-111.31441537,-89.9371709008,97.8410717962),4)
                                    #ur5.hard_set_joint_angles(define_joint_angle_list(-8.97200920273,-57.5998250621,77.4253084494,-109.817161576,-90.0504541341,170.994315015),4)
                rospy.loginfo('\033[96m' + "Successfully Placed Green Box" + '\033[0m')
            elif(package_color == "yellow"):
                #----Coordinate of yellow bin----->>>>>>>>>>>
                ur5.attach_box()
                ur5.ee_cartesian_translation(0,0,0.2)
                ur5.hard_go_to_pose(define_pose(0.7,0,1.265),4)
                ur5.detach_box()
                #newt = vaccum(False)
            # ur5.hard_set_joint_angles(define_joint_angle_list(-8.97200920273,-57.5998250621,77.4253084494,-109.817161576,-90.0504541341,170.994315015),4)
                                            #ur5.hard_set_joint_angles(define_joint_angle_list(-82.1363218895,-60.1866792126,81.5471504912,-111.31441537,-89.9371709008,97.8410717962),4)
                rospy.loginfo('\033[96m' + "Successfully Placed Yellow Box" + '\033[0m')
            else:
                rospy.loginfo('\033[96m' + "Box Type Unknown" + '\033[0m')
            
            obj_msg.message="busy"
            var_handle_pub.publish(obj_msg) #new
            print("msg sent")
            rospy.sleep(.55)
            var_handle_pub.publish(obj_msg) #new
            newt = vaccum(False)
            #ur5.detach_box()
            ur5.remove_box()
            box_place_list.append(package)
            print("going to sleep")
            #rospy.sleep(3)
        else:
            #trigger_converbelt(100)
            print("need to look here; package name: ",package)
            #should_start_conver = True
            #pass
    else:
        #trigger_converbelt(100)
        ur5.set_joint_angles(define_joint_angle_list(168.661264258,-50,50.3742109357,-97.2680961679,-90.0263818056,-11.2957158804))
        #should_start_conver = True
        #pass


class Camera1:

  def __init__(self):
    self.bridge = CvBridge()
    #self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback)
    self.temp_con = 0
  
  def get_qr_data(self, arg_image):
    qr_result = decode(arg_image)
    #print(qr_result)
    for i in range(len(qr_result)):
        print(qr_result[i].data)
    if ( len( qr_result ) > 0):
      #print(qr_result[0])
        return (qr_result[0].data)
    else :
        return ('NA')

  
  def callback(self,data):
    global temp_con
    if temp_con == 0:  
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          rospy.logerr(e)

        (rows,cols,channels) = cv_image.shape

        image = cv_image
        global l
        l={}
        #l=[]
        # Resize a 720x1280 image to 360x640 to fit it on the screen
        #resized_image = cv2.resize(image, (720/2, 1280/2)) 
        for i in range(4):
          for j in range(3):
            down=160*i
            side=180*j
            crop_image=image[280+down:440+down,90+side:270+side]
            
            l['packagen'+str(i)+str(j)]=self.get_qr_data(crop_image)
            #l.append(self.get_qr_data(crop_image))
            #print(l)
            rospy.loginfo(self.get_qr_data(crop_image))
        print(l,'Callback')   
        crop_image=image[280:440,450:630] #top:bottom,right:left ---pkg01
        #cv2.imshow("/eyrc/vb/camera_1/image_raw1", image)
        #cv2.imshow("/eyrc/vb/camera_1/image_raw", crop_image) 
        global temp
        # global temp1
        temp.unregister()
        # temp1=1
        cv2.waitKey(3)
        temp_con = 1




def match_boxcolor_with_no(data):
  global ic
  ic.callback(data)
  print("func")



def main():
    
    global ic 
    ic= Camera1()
    rospy.init_node('bot2-controller')
    global t
    global temp
    global temp_con
    temp_con =0
    
    temp = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,match_boxcolor_with_no,queue_size=1)
    rospy.sleep(1)
    
    global l #list
    
    #l =['red','green','yellow','green','yellow','red','yellow','red','green','red','yellow','green']
    #print(l)
    global freq
    #global x
    global is_conver_active
    is_conver_active = False
    global ur5
    global should_start_conver
    should_start_conver = True
    ur5 = Ur5Moveit(sys.argv[1])
    freq = 0
    global box_place_list
    box_place_list =[]
    ur5.set_joint_angles(define_joint_angle_list(168.661264258,-50,50.3742109357,-97.2680961679,-90.0263818056,-11.2957158804))
    #ur5.set_joint_angles(define_joint_angle_list(168.661264258,-43.1096955262,50.3742109357,-97.2680961679,-90.0263818056,-11.2957158804))
    rospy.Subscriber('/eyrc/vb/logical_camera_2',LogicalCameraImage,get_coordinate234,queue_size=1)    
    rospy.spin()



if __name__ == '__main__':
    main()

