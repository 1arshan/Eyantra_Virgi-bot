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

# packagen3 blue
# packagen2 green
# packagen1 red

class CartesianPath:

    # Constructor
    def __init__(self):
        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')

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

    def set_joint_angles(self, arg_list_joint_angles):
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan
    
    def print_joint_angles(self):
        list_joint_values = self._group.get_current_joint_values()

        rospy.loginfo('\033[94m' + "\nJoint Values: \n\n" +
                      "ur5_shoulder_pan_joint: {}\n".format(math.degrees(list_joint_values[0])) +
                      "ur5_shoulder_lift_joint: {}\n".format(math.degrees(list_joint_values[1])) +
                      "ur5_elbow_joint: {}\n".format(math.degrees(list_joint_values[2])) +
                      "ur5_wrist_1_joint: {}\n".format(math.degrees(list_joint_values[3])) +
                      "ur5_wrist_2_joint: {}\n".format(math.degrees(list_joint_values[4])) +
                      "ur5_wrist_3_joint: {}\n".format(math.degrees(list_joint_values[5])) +
                      '\033[0m')

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


    def add_box(self,x,y,z,timeout=4):
   
        scene = self._scene

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 0.0
        box_pose.pose.position.x = x #0.02 
        box_pose.pose.position.y =  y # 0.45 
        box_pose.pose.position.z = z 
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.15, 0.15, 0.15))

       
        self.box_name=box_name
     
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)
    
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

"""
    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')
"""

def define_joint_angle_list(shoulder_pan_joint,shoulder_lift_joint,elbow_joint,wrist_1_joint,wrist_2_joint,wrist_3_joint):
    joint_angles = [math.radians(shoulder_pan_joint),
                          math.radians(shoulder_lift_joint),
                          math.radians(elbow_joint),
                          math.radians(wrist_1_joint),
                          math.radians(wrist_2_joint),
                          math.radians(wrist_3_joint)]
    return joint_angles


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


def get_coordinate234(data):
    models_length = len(data.models)
    if (models_length > 0):
        if(models_length == 1):
            x = data.models[0].pose.position.x
            y = data.models[0].pose.position.y
            package = data.models[0].type
        else:    
            x = data.models[models_length-1].pose.position.x
            y = data.models[models_length-1].pose.position.y
            package = data.models[models_length-1].type    

        if(y < 0.1 and package != "ur5") :#--pkg1,2,3
            gazebo_y = round(y,3)
            constant_x=1.00500025103
            constant_diff=0.00000018366
            multiplier=(x-constant_x)/constant_diff
            multiplier=round(multiplier,1)
            gazebo_x =-(0.8+.05*multiplier)        
            box_length = 0.15               # Length of the Package
            vacuum_gripper_width = 0.115    # Vacuum Gripper Width
            delta = vacuum_gripper_width + (box_length/2)  # 0.19

            global ur5

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
            ur5.go_to_pose(ur5_2_home_pose)
            x = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)    
            y = x(True)
            rospy.loginfo('\033[94m' + "Translating EE by 0.5m in x from current position." + '\033[0m')

            if(package == "packagen1"):
            #----Coordinate of red bin----->>>>>>>>>>>
                ur5.attach_box()
                ur5.ee_cartesian_translation(0,0,0.17)
                trigger_converbelt(100)
                ur5.set_joint_angles(define_joint_angle_list(61.6301577746,19.9056122889,-90.7433259006,-19.1991556074,-90.037481863,-118.422844125))
                rospy.loginfo('\033[96m' + "Successfully Placed Red Box" + '\033[0m')
            elif(package == "packagen2"):
                #----Coordinate of green bin----->>>>>>>>>>>
                ur5.attach_box()
                trigger_converbelt(100)
                ur5.set_joint_angles(define_joint_angle_list(-8.97200920273,-57.5998250621,77.4253084494,-109.817161576,-90.0504541341,170.994315015))
                rospy.loginfo('\033[96m' + "Successfully Placed Green Box" + '\033[0m')
            elif(package == "packagen3"):
                #----Coordinate of blue bin----->>>>>>>>>>>
                ur5.attach_box()
                trigger_converbelt(100)
                ur5.set_joint_angles(define_joint_angle_list(-82.1363218895,-60.1866792126,81.5471504912,-111.31441537,-89.9371709008,97.8410717962))
                rospy.loginfo('\033[96m' + "Successfully Placed Blue Box" + '\033[0m')
            else:
                rospy.loginfo('\033[96m' + "Box Type Unknown" + '\033[0m')
            y = x(False)
            ur5.detach_box()
            ur5.remove_box()
        else:
            pass
    else:
            pass




def main():
    rospy.init_node('node_task3_bot_controller')
    global t
    global freq
    global x
    global is_conver_active
    is_conver_active = False
    global ur5
    global should_start_conver
    should_start_conver = True
    ur5 = CartesianPath()
    freq = 0
    ur5.set_joint_angles(define_joint_angle_list(168.661264258,-43.1096955262,50.3742109357,-97.2680961679,-90.0263818056,-11.2957158804))
    rospy.Subscriber('/eyrc/vb/logical_camera_2',LogicalCameraImage,get_coordinate234,queue_size=1)    
    rospy.spin()


if __name__ == '__main__':
    main()
