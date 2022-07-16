#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
# import pyassimp
from moveit_python import PlanningSceneInterface
from geometry_msgs.msg import Pose, PoseStamped, Point
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, ObjectColor
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from shape_msgs.msg import MeshTriangle, Mesh, SolidPrimitive, Plane
import tf
import math
from geometry_msgs.msg import PoseStamped
from tf import TransformListener
from tf2_ros import TransformException
## END_SUB_TUTORIAL



def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    
    robot = moveit_commander.RobotCommander()
    print "debug-=-------------------------"
    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()
    
    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "cr5_arm"
    
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    print "============ Printing robot pose"
    print move_group.get_current_pose()
    print ""

    # print "============ Printing robot state"
    # print robot.get_current_state()
    # print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self.listener = tf.TransformListener()

    # all_pose is [geometry_msgs.Pose().position.x , y ,z , geometry_msgs.Pose().orientation.x, y ,z ,w]
    self.pose_start = [0.313478,0.25734,0.756146,0.882028,-0.350559,0.167868,-0.266376]
    self.pose_goal = [0.149814,-0.108493,0.839925,-0.542926,0.778994,-0.313085,0.0194277]

    # self.go_to_pose_use_tf() args
    self.transformPose_name_end_link = 'Link6'  # can change this to 'zivid-waike' tf_listener_get_pose  listener.transformPose('Link6',pose)
    self.transformPose_name_end_link_begin = 'base_link'  # tf_listener_get_pose  listener.transformPose('base_link',pose_tf_pose)
    
    # self.add_meshes() args
    # self.add_mesh_pose = [-0.5,1.1,0.25,0,0,0,0]
    # self.add_mesh_pose = [0.197784,0.397331,-0.383783,0.0894667,-0.28682,0.710938,0.635844]
    # self.add_mesh_pose = [0,0,0,0,0,0,0]
    # self.add_mesh_pose = [0.5389,-0.4035,0.8156,-0.1669,-0.3911,-0.3456,0.8364]  #POSE1 ERROR
    # self.add_mesh_pose = [0.1211,-0.0500,0.4761,-0.0716,0.4138,0.8276,0.3723]    #POSE2 ERROR
    self.add_mesh_pose = [0.151587,0.145192,-0.220096,0.0494624,-0.158571,0.393048,0.90439]    #POSE20220225--right!!!-->'/home/longxiaoze/3D_models/stop/Mesh[unpower_bogie_meter_1_mesh.stl'
    # self.file_stl_name = '/home/longxiaoze/3D_models/bogie_del5_real_scale_half.stl'
    # self.file_stl_name = '/home/longxiaoze/3D_models/unpower_bogie_meter.ply'   #this format can load
    # self.file_stl_name = '/home/longxiaoze/3D_models/bogie_scale.stl'           #this format can load
    # self.file_stl_name = '/home/longxiaoze/3D_models/AGV.dae'                   #this format can load
    # self.file_stl_name = '/home/longxiaoze/3D_models/try2trans.stl'
    self.file_stl_name = '/home/longxiaoze/3D_models/stop/Mesh[unpower_bogie_meter_1_mesh.stl'
    
    # self.add_agv_meshes() args
    self.add_agv_pose = [0,0,0,0, 0, 0.3894183, 0.921061]
    self.file_agv_name = '/home/longxiaoze/3D_models/AGV.dae'

    # self.add_wall() args  TODO:need to change this and link to mesh!!!!!!!
    self.add_wall_left_pose = [-0.59,0.06,0,0.0010418, -0.0004277, 0.3855434, -0.922689]################################################
    self.add_wall_right_pose = [-0.06,0-0.75,0,0.0324761, -0.0139378, -0.371269, 0.9278526]################################################
    self.add_wall_size = [0.1, 2, 2]

    # self.add_ground() args
    self.add_ground_pose = [0,0,-0.1,0,0,0,0]



  def tf_listener_get_pose(self,pose_goal,distance = 0.2):
    listener = self.listener
    rate = rospy.Rate(10.0)
    pose = PoseStamped()
    pose.header.frame_id = 'base_link'

    pose.pose.position.x = pose_goal.position.x
    pose.pose.position.y = pose_goal.position.y
    pose.pose.position.z = pose_goal.position.z
    pose.pose.orientation.x =  pose_goal.orientation.x
    pose.pose.orientation.y = pose_goal.orientation.y
    pose.pose.orientation.z = pose_goal.orientation.z
    pose.pose.orientation.w = pose_goal.orientation.w

    print pose
    try:
      print '++++++++++++++++++++++++++++++++++++'
      # pose_tf = listener.transformPose('zivid-waike',pose)
      pose_tf = listener.transformPose(self.transformPose_name_end_link , pose)
      print pose_tf
      print '++++++++++++++++++++++++++++++++++++'
      pose_tf_pose = pose_tf
      pose_tf_pose.pose.position.z += distance
      pose_tf_pose = listener.transformPose(self.transformPose_name_end_link_begin,pose_tf_pose)
      print pose_tf_pose
      print '++++++++++++++++++++++++++++++++++++'
      return pose_tf_pose

    except :
      return None

  def go_to_pose_use_tf(self):   
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:

    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = -0.266376
    # pose_goal.position.x = 2.95976602748
    # pose_goal.position.y = 3.31944757968
    # pose_goal.position.z = 3.69224151038
    # pose_goal.orientation.x =  0.882028
    # pose_goal.orientation.y = -0.350559
    # pose_goal.orientation.z = 0.167868

    # #(1)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 0.0194277
    # pose_goal.position.x = 0.149814
    # pose_goal.position.y = -0.108493
    # pose_goal.position.z = 0.839925
    # pose_goal.orientation.x =  -0.542926
    # pose_goal.orientation.y = 0.778994
    # pose_goal.orientation.z = -0.313085

    #(2)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = -0.0270134
    # pose_goal.position.x =  0.114335
    # pose_goal.position.y = -0.106603
    # pose_goal.position.z = 0.825535
    # pose_goal.orientation.x =  0.534803
    # pose_goal.orientation.y = -0.780194
    # pose_goal.orientation.z = 0.323348

    # #(3)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 0.22105
    # pose_goal.position.x = -0.304324
    # pose_goal.position.y = -0.360256
    # pose_goal.position.z = 0.786937
    # pose_goal.orientation.x =  -0.117548
    # pose_goal.orientation.y = 0.944906
    # pose_goal.orientation.z = -0.210876
    # #(4)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 0.224127
    # pose_goal.position.x = -0.186267
    # pose_goal.position.y = -0.460263
    # pose_goal.position.z = 0.788241
    # pose_goal.orientation.x =  -0.157511
    # pose_goal.orientation.y = 0.942456
    # pose_goal.orientation.z = -0.191661
    # #(5)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 0.0506901
    # pose_goal.position.x = 0.139795
    # pose_goal.position.y = -0.0515554
    # pose_goal.position.z = 0.827831
    # pose_goal.orientation.x =  -0.587747
    # pose_goal.orientation.y =  0.743891
    # pose_goal.orientation.z = -0.314022

    #(2)
    pose_goal_set = self.pose_goal
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = pose_goal_set[0]
    pose_goal.position.y = pose_goal_set[1]
    pose_goal.position.z = pose_goal_set[2]
    pose_goal.orientation.x =  pose_goal_set[3]
    pose_goal.orientation.y = pose_goal_set[4]
    pose_goal.orientation.z = pose_goal_set[5]
    pose_goal.orientation.w = pose_goal_set[6]

    flag = True
    try_num = 15
    count_num = try_num
    distance = 0.1
    while flag == True:
      print distance
      pose_tf_pose = self.tf_listener_get_pose(pose_goal,distance)

      pose_goal = geometry_msgs.msg.Pose()
      pose_goal.position.x = pose_tf_pose.pose.position.x
      pose_goal.position.y = pose_tf_pose.pose.position.y
      pose_goal.position.z = pose_tf_pose.pose.position.z
      pose_goal.orientation.x =  pose_tf_pose.pose.orientation.x
      pose_goal.orientation.y = pose_tf_pose.pose.orientation.y 
      pose_goal.orientation.z = pose_tf_pose.pose.orientation.z 
      pose_goal.orientation.w = pose_tf_pose.pose.orientation.w

      # move_group.set_pose_target(pose_goal,end_effector_link ="zivid-waike")
      move_group.set_pose_target(pose_goal)

      
      ## Now, we call the planner to compute the plan and execute it.
      plan = move_group.go(wait=True)
      print plan
      # Calling `stop()` ensures that there is no residual movement
      move_group.stop()
      # It is always good to clear your targets after planning with poses.
      # Note: there is no equivalent function for clear_joint_value_targets()
      move_group.clear_pose_targets()
      if plan ==False:
        print 'plan_error plan again, now we delate on last link in z axis with same orientation and plan again!'
        try_num  = try_num - 1
        print try_num
        if try_num == 0:
          distance = -0.1-count_num*0.1
        elif try_num == -1:
          distance = -0.1
        if try_num == -15:
          print "move cannot find a plan, this pose can not use!!!!!!!!!!!!!!!!!!"
          break
      else:
        break
        # if try_num == -15:  # keep moving
        #   break             # keep moving
    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def add_agv_meshes(self, timeout=4):
    scene = self.scene
    mesh_pose = geometry_msgs.msg.PoseStamped()
    mesh_pose.header.frame_id = "base_link"

    mesh_pose.pose.position.x = self.add_agv_pose[0]
    mesh_pose.pose.position.y = self.add_agv_pose[1]
    mesh_pose.pose.position.z = self.add_agv_pose[2]
    mesh_pose.pose.orientation.x = self.add_agv_pose[3]
    mesh_pose.pose.orientation.y = self.add_agv_pose[4]
    mesh_pose.pose.orientation.z = self.add_agv_pose[5]
    mesh_pose.pose.orientation.w = self.add_agv_pose[6]
    file_stl = self.file_agv_name

    scene.add_mesh('mesh_agv', mesh_pose, file_stl)

  def add_meshes(self, timeout=4):
    scene = self.scene
    mesh_pose = geometry_msgs.msg.PoseStamped()
    mesh_pose.header.frame_id = "base_link"
    # # mesh_pose.pose.orientation.w = 1.0
    # # mesh_pose.pose.orientation.x = 1.0
    # # mesh_pose.pose.orientation.y = 1.0
    # # mesh_pose.pose.orientation.z = 1.0
    # mesh_pose.pose.position.x = -0.5 
    # mesh_pose.pose.position.y = 1.1
    # mesh_pose.pose.position.z = 0.25

    mesh_pose.pose.position.x = self.add_mesh_pose[0]
    mesh_pose.pose.position.y = self.add_mesh_pose[1]
    mesh_pose.pose.position.z = self.add_mesh_pose[2]
    mesh_pose.pose.orientation.x = self.add_mesh_pose[3]
    mesh_pose.pose.orientation.y = self.add_mesh_pose[4]
    mesh_pose.pose.orientation.z = self.add_mesh_pose[5]
    mesh_pose.pose.orientation.w = self.add_mesh_pose[6]
    file_stl = self.file_stl_name

    scene.add_mesh('mesh', mesh_pose, file_stl)

  def add_wall(self,wall_name, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    scene = self.scene
    if wall_name == 'wall_left':
      wall_pose_set = self.add_wall_left_pose
    elif wall_name =='wall_right':
      wall_pose_set = self.add_wall_right_pose
    else:
      print 'change wall_name to wall_left or wall_right,and set self.add_wall_left_pose etl'
    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    wall_pose = geometry_msgs.msg.PoseStamped()
    wall_pose.header.frame_id = "base_link"
    wall_pose.pose.orientation.x = wall_pose_set[3]
    wall_pose.pose.orientation.y = wall_pose_set[4]
    wall_pose.pose.orientation.z = wall_pose_set[5]
    wall_pose.pose.orientation.w = wall_pose_set[6]
    wall_pose.pose.position.x = wall_pose_set[0] # slightly above the end effector
    wall_pose.pose.position.y = wall_pose_set[1]
    wall_pose.pose.position.z = wall_pose_set[2]
    wall_name = wall_name
    scene.add_box(wall_name, wall_pose, size=(self.add_wall_size[0], self.add_wall_size[1], self.add_wall_size[2]))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def go_to_pose_start(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:

    # # (1)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = -0.266376
    # pose_goal.position.x = 0.313478
    # pose_goal.position.y = 0.25734
    # pose_goal.position.z = 0.756146
    # pose_goal.orientation.x =  0.882028
    # pose_goal.orientation.y = -0.350559
    # pose_goal.orientation.z = 0.167868
    

    #(1)pingyi z = z+0.2
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.position.x = 0.409851050058
    # pose_goal.position.y = 0.327822450347
    # pose_goal.position.z = 0.595695199392
    # pose_goal.orientation.x =  0.882027627756
    # pose_goal.orientation.y = -0.350558852053
    # pose_goal.orientation.z = 0.167867929154 
    # pose_goal.orientation.w = -0.266375887581
    # (1)goal-random orientation
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 0.0194277
    # pose_goal.position.x = 0.149814
    # pose_goal.position.y = -0.108493
    # pose_goal.position.z = 0.839925
    # pose_goal.orientation.x =  0.12918
    # pose_goal.orientation.y = 0.0019221
    # pose_goal.orientation.z = 0.95653
    # pose_goal.orientation.w = -0.037766
    # (2)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = -0.267499
    # pose_goal.position.x =  0.283818
    # pose_goal.position.y = 0.257077
    # pose_goal.position.z = 0.743982   
    # pose_goal.orientation.x =  0.878782
    # pose_goal.orientation.y = -0.350464
    # pose_goal.orientation.z = 0.18265
    # # (3)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = -0.0273244
    # pose_goal.position.x =  -0.0420086
    # pose_goal.position.y = -0.212872
    # pose_goal.position.z = 0.808511   
    # pose_goal.orientation.x =  -0.382124
    # pose_goal.orientation.y =  0.862073
    # pose_goal.orientation.z = -0.331761
    # # (4)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = -0.0157909
    # pose_goal.position.x = 0.0627556
    # pose_goal.position.y = -0.291287
    # pose_goal.position.z = 0.809011  
    # pose_goal.orientation.x =  -0.42142
    # pose_goal.orientation.y = -0.321937
    # pose_goal.orientation.z = -0.0157909
    # # (5)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = -0.284415
    # pose_goal.position.x = 0.257385
    # pose_goal.position.y = 0.332951
    # pose_goal.position.z = 0.750592 
    # pose_goal.orientation.x =  0.899413
    # pose_goal.orientation.y = -0.291265
    # pose_goal.orientation.z = 0.159151
    # test_mesh_pose
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.position.x= -0.43813764921
    # pose_goal.position.y= -0.285925896352
    # pose_goal.position.z= 0.820336658446
    # pose_goal.orientation.x = -5.86557414841e-05
    # pose_goal.orientation.y = -0.182421875408
    # pose_goal.orientation.z = -0.983220347671
    # pose_goal.orientation.w = 6.21126577036e-05

    
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = self.pose_start[0]
    pose_goal.position.y = self.pose_start[1]
    pose_goal.position.z = self.pose_start[2]
    pose_goal.orientation.x =  self.pose_start[3]
    pose_goal.orientation.y = self.pose_start[4]
    pose_goal.orientation.z = self.pose_start[5]
    pose_goal.orientation.w = self.pose_start[6]

    # move_group.set_pose_target(pose_goal,end_effector_link ="zivid-waike")
    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def go_to_pose_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:

    # #(1)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 0.0194277
    # pose_goal.position.x = 0.149814
    # pose_goal.position.y = -0.108493
    # pose_goal.position.z = 0.839925
    # pose_goal.orientation.x =  -0.542926
    # pose_goal.orientation.y = 0.778994
    # pose_goal.orientation.z = -0.313085

    #(2)
    # pose_goal = geometry_msgs.msg.Pose()
    # # pose_goal.orientation.w = -0.0270134
    # pose_goal.position.x =  0.114335
    # pose_goal.position.y = -0.106603
    # # pose_goal.position.y = -0.18
    # pose_goal.position.z = 0.825535
    # # pose_goal.orientation.x =  0.534803
    # # pose_goal.orientation.y = -0.780194
    # # pose_goal.orientation.z = 0.323348
    # #(3)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 0.22105
    # pose_goal.position.x = -0.304324
    # pose_goal.position.y = -0.360256
    # pose_goal.position.z = 0.786937
    # pose_goal.orientation.x =  -0.117548
    # pose_goal.orientation.y = 0.944906
    # pose_goal.orientation.z = -0.210876
    # #(4)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 0.224127
    # pose_goal.position.x = -0.186267
    # pose_goal.position.y = -0.460263
    # pose_goal.position.z = 0.788241
    # pose_goal.orientation.x =  -0.157511
    # pose_goal.orientation.y = 0.942456
    # pose_goal.orientation.z = -0.191661
    # #(5)
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 0.0506901
    # pose_goal.position.x = 0.139795
    # pose_goal.position.y = -0.0515554
    # pose_goal.position.z = 0.827831
    # pose_goal.orientation.x =  -0.587747
    # pose_goal.orientation.y =  0.743891
    # pose_goal.orientation.z = -0.314022
    
    # test_mesh_pose
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.position.x= 2.82837076328e-06
    # pose_goal.position.y= -0.245999999999
    # pose_goal.position.z= 1.04700038568
    # pose_goal.orientation.x = 1.29867411873e-06
    # pose_goal.orientation.y = -0.707106781181
    # pose_goal.orientation.z = 0.707106781181
    # pose_goal.orientation.w = 3.89602235603e-06

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = self.pose_goal[0]
    pose_goal.position.y = self.pose_goal[1]
    pose_goal.position.z = self.pose_goal[2]
    pose_goal.orientation.x =  self.pose_goal[3]
    pose_goal.orientation.y = self.pose_goal[4]
    pose_goal.orientation.z = self.pose_goal[5]
    pose_goal.orientation.w = self.pose_goal[6]

    # move_group.set_pose_target(pose_goal,end_effector_link ="zivid-waike")
    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def add_ground(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.orientation.w = self.add_ground_pose[6]
    box_pose.pose.position.z = self.add_ground_pose[2] # slightly above the end effector
    box_name = "ground"
    scene.add_box(box_name, box_pose, size=(1, 1, 0.1))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

########################### not use code #################################################################################
  def plan_cartesian_path(self, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. If executing  interactively in a
    ## Python shell, set scale = 1.0.
    ##
    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step 1cm
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

    ## END_SUB_TUTORIAL

  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL

  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
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
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL

  def add_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "Link6"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = 0.1 # slightly above the end effector
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.2))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def attach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    ## BEGIN_SUB_TUTORIAL attach_object
    ##
    ## Attaching Objects to the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
    ## robot be able to touch them without the planning scene reporting the contact as a
    ## collision. By adding link names to the ``touch_links`` array, we are telling the
    ## planning scene to ignore collisions between those links and the box. For the Panda
    ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
    ## you should change this value to the name of your end effector group name.
    grasping_group = 'cr5_arm' #you should change this name to your end effector group name!!!!!!!!!!!!!!
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  def detach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    ## BEGIN_SUB_TUTORIAL detach_object
    ##
    ## Detaching Objects from the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can also detach and remove the object from the planning scene:
    scene.remove_attached_object(eef_link, name=box_name)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

  def remove_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL remove_object
    ##
    ## Removing Objects from the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can remove the box from the world.
    scene.remove_world_object(box_name)

    ## **Note:** The object must be detached before we can remove it from the world
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

  def add_area(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "Link1"
    box_pose.pose.orientation.w = 0
    box_pose.pose.position.z = 1.38 # slightly above the end effector
    box_pose.pose.position.x = 0.45 # slightly above the end effector
    box_pose.pose.position.y = 0.3 # slightly above the end effector
    box_name = "area"
    scene.add_box(box_name, box_pose, size=(0.1, 2, 2))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

  def go_to_joint_state(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0.8892652952806639
    joint_goal[1] = 0.7779352074689527
    joint_goal[2] = -1.4838353281846606
    joint_goal[3] = -0.8655974713766813
    joint_goal[4] = -1.5705821053379334
    joint_goal[5] = -2.459579312371455
    #[0.8892652952806639, 0.7779352074689527, -1.4838353281846606,
    # -0.8655974713766813, -1.5705821053379334, -2.459579312371455]
    # joint_goal[6] = 0  #cr5 dont need this

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)
########################### not use code #################################################################################

def main():
  try:
    print ""
    print "----------------------------------------------------------"
    print "Welcome to the MoveIt MoveGroup Python Interface Tutorial"
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print ""
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
    raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()

    print "============ Press `Enter` to add left and right wall to the planning scene ..."
    raw_input()
    tutorial.add_wall("wall_left")
    tutorial.add_wall("wall_right")

    print "============ Press `Enter` to add_mesh================== ..."
    raw_input()
    tutorial.add_meshes()

    print "============ Press `Enter` to add_agv_mesh================== ..."
    raw_input()
    tutorial.add_agv_meshes()

    # print "============ Press `Enter` to add a ground to the planning scene ..."
    # raw_input()
    # tutorial.add_ground()

    print "============ Press `Enter` to execute a movement using a start goal ..."
    raw_input()
    tutorial.go_to_pose_start()
    print "============ Press `Enter` to execute a movement using go_to_pose_use_tf ..."
    raw_input()
    tutorial.go_to_pose_use_tf()









    # print "============ Press `Enter` to add a area to the planning scene ..."
    # raw_input()
    # tutorial.add_area()

    # print "============ Press `Enter` to add a box to the planning scene ..."
    # raw_input()
    # tutorial.add_box()

    # print "============ Press `Enter` to attach a Box to the Panda robot ..."
    # raw_input()
    # tutorial.attach_box()

    # print "============ Press `Enter` to execute a movement using a joint state goal ..."
    # raw_input()
    # tutorial.go_to_joint_state()

    # print "============ Press `Enter` to execute a movement using a pose goal ..."
    # raw_input()
    # tutorial.go_to_pose_goal()

    # print "============ Press `Enter` to plan and display a Cartesian path ..."
    # raw_input()
    # cartesian_plan, fraction = tutorial.plan_cartesian_path()

    # print "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
    # raw_input()
    # tutorial.display_trajectory(cartesian_plan)

    # print "============ Press `Enter` to execute a saved path ..."
    # raw_input()
    # tutorial.execute_plan(cartesian_plan)

    # print "============ Press `Enter` to add a box to the planning scene ..."
    # raw_input()
    # tutorial.add_box()

    # print "============ Press `Enter` to attach a Box to the Panda robot ..."
    # raw_input()
    # tutorial.attach_box()

    # print "============ Press `Enter` to plan and execute a path with an attached collision object ..."
    # raw_input()
    # cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
    # tutorial.execute_plan(cartesian_plan)

    # print "============ Press `Enter` to detach the box from the Panda robot ..."
    # raw_input()
    # tutorial.detach_box()

    # print "============ Press `Enter` to remove the box from the planning scene ..."
    # raw_input()
    # tutorial.remove_box()

    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/melodic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/melodic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
## .. _rospy:
##    http://docs.ros.org/melodic/api/rospy/html/
## CALL_SUB_TUTORIAL imports
## CALL_SUB_TUTORIAL setup
## CALL_SUB_TUTORIAL basic_info
## CALL_SUB_TUTORIAL plan_to_joint_state
## CALL_SUB_TUTORIAL plan_to_pose
## CALL_SUB_TUTORIAL plan_cartesian_path
## CALL_SUB_TUTORIAL display_trajectory
## CALL_SUB_TUTORIAL execute_plan
## CALL_SUB_TUTORIAL add_box
## CALL_SUB_TUTORIAL wait_for_scene_update
## CALL_SUB_TUTORIAL attach_object
## CALL_SUB_TUTORIAL detach_object
## CALL_SUB_TUTORIAL remove_object
## END_TUTORIAL
