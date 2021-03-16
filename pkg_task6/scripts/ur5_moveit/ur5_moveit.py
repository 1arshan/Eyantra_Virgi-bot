#! /usr/bin/env python2.7
"""
This file contain Ur5Moveit class to control bot and rviz planning scene.
"""
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from pkg_vb_sim.srv import conveyorBeltPowerMsg
import math
import sys
import copy
from std_srvs.srv import Empty
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode


def define_joint_angle_list(shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint):
    """
    This function takes float values for each joint and returns a list.
    :param shoulder_pan_joint: shoulder pan joint angle
    :param shoulder_lift_joint: shoulder lift join angle
    :param elbow_joint: elbow joint angle
    :param wrist_1_joint: wrist 1 joint angle
    :param wrist_2_joint: wrist 2 joint angle
    :param wrist_3_joint: wrist 3 joint angle
    :return: list of all joint angles
    """

    joint_angles = [math.radians(shoulder_pan_joint),
                          math.radians(shoulder_lift_joint),
                          math.radians(elbow_joint),
                          math.radians(wrist_1_joint),
                          math.radians(wrist_2_joint),
                          math.radians(wrist_3_joint)]
    return joint_angles


def define_pose(x, y, z):
    """
    This function define geometry_msgs.msg.Pose() from giving coordinates.
    :param x: x coordinate
    :param y: y coordinate
    :param z: z coordinate
    :return: ur5_pose of type geometry_msgs.msg.Pose() containing x,y,z coordinate.
    """
    ur5_pose = geometry_msgs.msg.Pose()
    ur5_pose.position.x = x
    ur5_pose.position.y = y
    ur5_pose.position.z = z
    ur5_pose.orientation.x = -0.5
    ur5_pose.orientation.y = -0.5
    ur5_pose.orientation.z = 0.5
    ur5_pose.orientation.w = 0.5
    return ur5_pose


def get_item_details(item):
    """
    This function finds packgen details using item
    :param item: item is a string containing packgen content type
    :return: it returns dict of details of packgen.
    """
    details = {'Estimated Time of Delivery':'Na','priority':'Na','cost':'Na','item':'Na'}

    if item == u'Medicines' or item == u'Medicine':
        details = {'priority':'HP','cost':'450','Estimated Time of Delivery':'1','item':'Medicines'}
    elif item == u'Food':
        details = {'priority':'MP','cost':'250','Estimated Time of Delivery':'3',"item":'Food'}
    elif item == u'Clothes':
        details = {'priority':'LP','cost':'150','Estimated Time of Delivery':'5','item':'Clothes'}

    return details


class Ur5Moveit:

    """
    This class enables bot and rviz connection, and setup the configuration of ur5 bot.
    Select bot name, planning group ,etc.
    """
    def __init__(self,robot_name):
        """
        Constructor
        :param robot_name: name of bot which you want to control.
        """
        self.is_conver_active = False
        self._box_name = 'box'
        self.box_name = 'box'
        self._robot_ns = '/'+robot_name
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
        self._computed_plan = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo('\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[94m' + " >>> ur5_moveit init done." + '\033[0m')

    @property
    def group(self):
        return self._group


    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        """
        This function helps in translating bot in x,y,z direction using cartesian coordinate system.
        :param trans_x: Displacement in x direction.
        :param trans_y: Displacement in y direction.
        :param trans_z: Displacement in z direction.
        """
        # 1. Create a empty list to hold waypoints
        waypoints = [self._group.get_current_pose().pose]

        # 2. Add Current Pose to the list of waypoints

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + trans_x
        wpose.position.y = waypoints[0].position.y + trans_y
        wpose.position.z = waypoints[0].position.z + trans_z
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

        # The reason for deleting the first two waypoints from the computed Cartesian Path can be found here,
        # https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488
        num_pts = len(plan.joint_trajectory.points)
        if num_pts >= 3:
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False,box_name='box', timeout=4):
        """
        Check status of box in rviz
        :param timeout: time period
        :param box_is_known: State of box
        :param box_is_attached: State of box
        :param box_name: Name of box in rviz
        :return: boolean
        """
        box_name = box_name or self.box_name
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

     # If we exited the while loop without returning then we timed out
        return False
    ## END_SUB_TUTORIAL


    def add_box(self,x,y,z,b_name):
        """
        This function adds a box in rviz planning scene.
        :param x: x coordinate of box
        :param y: y coordinate of box
        :param z: z coordinate of box
        :param b_name: name to be given to the box in planning scene
        :return: boolean
        """
        box_name = b_name or self.box_name
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
    
    def attach_box(self,b_name, timeout=4):
        """
        This function attach to box with vacuum gripper in rviz planning scene to avoid collision
        :param b_name:box name which needs to be attached.
        :param timeout: time period
        :return: boolean
        """
        box_name = b_name or self.box_name
        robot = self._robot
        scene = self._scene
        eef_link = self._eef_link

    
        grasping_group = self._planning_group
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)

        return self.wait_for_state_update(box_is_attached=True, box_is_known=False,box_name=box_name, timeout=timeout)
   
   
    def detach_box(self,b_name, timeout=4):
        """
                This function detach to box from vacuum gripper in rviz planning scene to avoid collision
                :param b_name:box name which needs to be detached.
                :param timeout: time period
                :return: boolean
        """
        box_name = b_name or self.box_name
        scene = self._scene
        eef_link = self._eef_link

        scene.remove_attached_object(eef_link, name=box_name)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False,box_name=box_name, timeout=timeout)


    def remove_box(self,b_name, timeout=4):
        """
                This function removes box form rviz planning scene.
                :param timeout: time period
                :param b_name: name of box which need to be removed from  planning scene
                :return: boolean
        """
        box_name = b_name or self.box_name
        scene = self._scene

        scene.remove_world_object(box_name)

        return self.wait_for_state_update(box_is_attached=False, box_is_known=False,box_name=box_name, timeout=timeout)


    def go_to_pose(self, arg_pose):
        """
        This function plans path to a pose and move ur5 to that pose.
        :param arg_pose: pose of desired location of type geometry_msgs.msg.Pose()
        :return: boolean
        """
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

        if flag_plan:
            rospy.loginfo('\033[94m' + ">>> go_to_pose_angle() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def hard_go_to_pose(self, arg_pose, arg_max_attempts):
        """
        This function calls go_to_pose() function multiple time until it succeed or reaches maximum attempt limit.
        :param arg_pose: pose of desired location of type geometry_msgs.msg.Pose()
        :param arg_max_attempts: maximum attempts for calling go_to_pose()
        :return: boolean
        """
        number_attempts = 0
        flag_success = False
        
        while (number_attempts <= arg_max_attempts) and  (flag_success is False):
            number_attempts += 1
            flag_success = self.go_to_pose(arg_pose)
            rospy.logwarn("attempts: {}".format(number_attempts) )
        return flag_success

    def clear_octomap(self):
        clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
        return clear_octomap_service_proxy()

    def set_joint_angles(self, arg_list_joint_angles):
        """
        This function sets angle of each joint of ur5 to reach a specific position.
        :param arg_list_joint_angles: list of joints
        :return: boolean
        """

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)

        if flag_plan:
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):
        """
        This function calls set_joint_angles() multiple times until it succeed or reaches maximum attempt limit
        :param arg_list_joint_angles: list of all joint angles
        :param arg_max_attempts: maximum attempt
        :return: boolean
        """

        number_attempts = 0
        flag_success = False
        
        while (number_attempts <= arg_max_attempts) and  (flag_success is False):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # self.clear_octomap()

        return flag_success

    def trigger_converbelt(self,value):
        """
        this function sets belt speed using ros service
        :param value: value of speed.
        :return: null
        """
        x = rospy.ServiceProxy('eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
        y = x(value)
        print(y)
        self.is_conver_active = value


    def __del__(self):
        """
        Destructor
        :return: null
        """
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class ur5_moveit Deleted." + '\033[0m')

            