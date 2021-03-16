#! /usr/bin/env python2.7

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from pkg_vb_sim.msg import LogicalCameraImage
from pkg_vb_sim.srv import vacuumGripper
import yaml
import math
import sys
import copy
from pkg_moveit_examples.msg import DelayRosService,Bot2Running,Bot1Running
from std_srvs.srv import Empty

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
        self._group.set_planning_time(100)

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


        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

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

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False,box_name='box00', timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
        # box_name = self.box_name
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


    def add_box(self,x,y,z,box_name):
   
        # box_name = self.box_name
        scene = self._scene

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 0.0
        box_pose.pose.position.x = x #0.02 
        box_pose.pose.position.y = y # 0.45 
        box_pose.pose.position.z = z 
    
        scene.add_box(box_name, box_pose, size=(0.15, 0.15, 0.15))

       
        self.box_name=box_name
     
        return self.wait_for_state_update(box_is_known=True, box_name=box_name,timeout=4)
    
    def attach_box(self, box_name,timeout=4):
        print('Box: ',box_name,)
        robot = self._robot
        scene = self._scene
        eef_link = self._eef_link

        grasping_group = self._planning_group
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)

        return self.wait_for_state_update(box_is_attached=True, box_is_known=False,box_name=box_name, timeout=timeout)
   
   
    def detach_box(self,box_name, timeout=4):
        scene = self._scene
        eef_link = self._eef_link

        scene.remove_attached_object(eef_link, name=box_name)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, box_name=box_name, timeout=timeout)


    def remove_box(self,box_name,timeout=4):
        
        scene = self._scene

        scene.remove_world_object(box_name)

        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, box_name=box_name, timeout=timeout)




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
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan


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
    
    
    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        file_path = arg_file_path + arg_file_name
        
        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)
        
        ret = self._group.execute(loaded_plan)
        # rospy.logerr(ret)
        return ret

    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
        number_attempts = 0
        flag_success = False

        while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # # self.clear_octomap()
        
        return True


    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

        number_attempts = 0
        flag_success = False
        
        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # self.clear_octomap()
    
    def change_file_path(self,name):
        pass
        #self._file_path = self._pkg_path + '/config/saved_trajectories/' +name

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

def define_joint_angle_list(shoulder_pan_joint,shoulder_lift_joint,elbow_joint,wrist_1_joint,wrist_2_joint,wrist_3_joint):
    joint_angles = [math.radians(shoulder_pan_joint),
                          math.radians(shoulder_lift_joint),
                          math.radians(elbow_joint),
                          math.radians(wrist_1_joint),
                          math.radians(wrist_2_joint),
                          math.radians(wrist_3_joint)]
    return joint_angles


def add_all_boxes(ur5):
    pose_x=[.28,0,-.28]
    pose_y= -0.41  # -.47
    pose_z=[1.92,1.66,1.43,1.20]
    for x in range(0,3):
        for z in range(0,4): 
            ur5.add_box(pose_x[x],pose_y,pose_z[z],'box'+str(z)+str(x))


def func_callback_topic_my_topic(myMsg):

    if myMsg.name == "busy":
        rospy.sleep(1)
    #rospy.loginfo("Data Received: ",myMsg.name)
    print("Data receive(belt): ",myMsg.name)

def func_callback_topic_bot2_running(myMsg):

    if myMsg.message == "busy":
        rospy.sleep(1)
    #rospy.loginfo("Data Received: ",myMsg.name)
    print("Data receive(bot2): ",myMsg.message)

def handle_bot1():
    ur5 = Ur5Moveit(sys.argv[1])

    coordinate_matrix = [
    
    [
        define_joint_angle_list(161,-119,12,-83,20,3),
        define_joint_angle_list(-124,-90,26,-117,-59,77),
        define_joint_angle_list(56,-114,2,-70,123,0),
    ],
    [
        define_joint_angle_list(-54,-96,82,-166,-128,-92),
        define_joint_angle_list(-122,-102,55,46,61,0),
        #define_joint_angle_list(53,-84,-79,-19,125,53),
        define_joint_angle_list(55,-83,-82,-17,123,0),

    ],
    [   #-53,-98,90,7,128,-88
        define_joint_angle_list(-53,-98,90,7,128,-88),
        define_joint_angle_list(117,-60,-134,12,68,-51),
        define_joint_angle_list(52,-84,-83,173,-130,-6)
    ],
    [
        define_joint_angle_list(-58,-79,138,122,-125,95),
        define_joint_angle_list(-121,-116,135,-17,59,179),
        define_joint_angle_list(-161,-93,118,-28,19,99)
     ]
    ]

    add_all_boxes(ur5)

    # ur5.hard_set_joint_angles(lst_joint_angles_3,4)

    x = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
    rospy.Subscriber("bot2_running",Bot2Running, func_callback_topic_bot2_running,queue_size=1)
    rospy.Subscriber("my_topic", DelayRosService, func_callback_topic_my_topic,queue_size=1)
    var_handle_pub = rospy.Publisher('bot1_running',Bot1Running, queue_size=1) #new
    obj_msg =Bot1Running()  #new
    obj_msg.message="busy" #new

    for i in range(0,4):
        for z in range(0,3):
            if (i == 3 and z == 0) or (i == 2 and z==1) or (i==3 and z == 2):
                continue
            print(i,z,': index\n')
           
            ur5.hard_set_joint_angles(coordinate_matrix[i][z],4)
        
            ur5.attach_box('box'+str(i)+str(z))
            obj_msg.message="busy" #new
            var_handle_pub.publish(obj_msg) #new
            print("msg sent")
            rospy.sleep(.45)
            var_handle_pub.publish(obj_msg) #new
            y = x(True)
            print("result is: ",y)
           
            ur5.ee_cartesian_translation(0,1.4, 0)
            print("cartition ")
           
            # ur5.hard_set_joint_angles(define_joint_angle_list(4,-126,-72,-71,90,0),4)
            # ur5.hard_set_joint_angles(define_joint_angle_list(179,-60,76,-112,-88,-90),4)
            # ur5.hard_set_joint_angles(define_joint_angle_list(179,-67,86,-111,-88,-90),4)
            #ur5.hard_set_joint_angles(define_joint_angle_list(179,-64,75,-102,-88,-90),4)
            ur5.hard_set_joint_angles(define_joint_angle_list(179,-57,86,-119,-88,-90),4)


            ur5.detach_box('box'+str(i)+str(z))

            obj_msg.message="busy"
            var_handle_pub.publish(obj_msg) #new
            print("msg sent")
            rospy.sleep(.45) #new
            var_handle_pub.publish(obj_msg) #new
            y = x(False)
            #rospy.sleep(.2) #new
            print("result is: ",y)
            ur5.remove_box('box'+str(i)+str(z))
    
    #del ur5


def main():
    rospy.init_node('shelf_controller', anonymous=True)
    handle_bot1()
    rospy.spin()


if __name__ == '__main__':
    main()

