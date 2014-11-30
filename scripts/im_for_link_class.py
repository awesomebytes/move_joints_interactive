#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 30/11/14

@author: Sam Pfeiffer

"""
import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from tf.transformations import euler_from_quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from get_mesh_urdf import get_link_mesh_info
from joint_limits_urdf import get_joint_limits
from geometry_msgs.msg import Pose

class LinkInteractiveMarker():
    IM_MARKER_SCALE = 0.2
    
    def __init__(self, joint_name, controller_name, joint_names):
        rospy.loginfo("Creating interactive marker for: " + str(joint_name))
        self.joint_name = joint_name
        self.controller_name = controller_name
        self.joint_names = joint_names
        self.base_name = joint_name.replace('_joint', '')
        # Get joint limits for the marker
        free_joints = get_joint_limits()
        if free_joints[joint_name]["has_position_limits"]:
            self.upper_joint_limit = float(free_joints[joint_name]["max_position"])
            self.lower_joint_limit = float(free_joints[joint_name]["min_position"])
            rospy.loginfo("Setting joint limits for " + self.base_name +
                           ":\nUpper: " + str(self.upper_joint_limit) +
                           "\nLower: " + str(self.lower_joint_limit) )
        else:
            rospy.logwarn("No joint limits found for " + joint_name + " setting up dummys.")
            self.upper_joint_limit = 4.0
            self.lower_joint_limit = -4.0
        self.ims = InteractiveMarkerServer("ims_" + self.base_name)
        self.makeRotateMarker(self.base_name)
        self.ims.applyChanges()
        # Publisher for the joint
        cmd_topic = "/" + controller_name + "/command"
        self.pub = rospy.Publisher(cmd_topic, JointTrajectory)
        rospy.loginfo("Done, now you should spin!")
        
    def makeRotateMarker(self, base_name):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = base_name + "_link"
        #int_marker.pose.position.x = 1.0
        int_marker.scale = self.IM_MARKER_SCALE
    
        int_marker.name = "rotate_" + base_name + "_joint"
        int_marker.description = "Rotate " + base_name + "_joint"
    
        self.makeBoxControl(int_marker)
    
        control = InteractiveMarkerControl()
        # at least arm joints follow the standard of going of with blue axe
        # maybe we also need to update the pose of the marker with the current one of the robot?
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)
    
        self.ims.insert(int_marker, self.processFeedback)

    def makeBoxControl(self, msg):
        control =  InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append( self.makeBox(msg) )
        msg.controls.append( control )
        return control

    def makeBox(self, msg):
        marker = Marker()
    
        # Automatically get mesh resource
        mesh_path, mesh_scale = get_link_mesh_info(self.base_name + '_link')
        if mesh_path == "": # if no mesh found, put a cylinder, works for reemc
            marker.type = Marker.CYLINDER
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
        else:
            marker.type = Marker.MESH_RESOURCE
            marker.mesh_resource = str(mesh_path)
            if mesh_scale != None:
                try:
                    marker.scale.x = float(mesh_scale.split()[0])
                    marker.scale.y = float(mesh_scale.split()[1])
                    marker.scale.z = float(mesh_scale.split()[2])
                except:
                    rospy.logwarn("Scale was not correctly found for " + str(self.base_name) +
                                  ", setting up 1.0 for all dimensions")
                    marker.scale.x = 1.0
                    marker.scale.y = 1.0
                    marker.scale.z = 1.0
            else:
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0
    
        return marker
    
    def processFeedback(self, feedback):
        s = "Feedback from marker '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"
    
        mp = ""
        if feedback.mouse_point_valid:
            mp = " at " + str(feedback.mouse_point.x)
            mp += ", " + str(feedback.mouse_point.y)
            mp += ", " + str(feedback.mouse_point.z)
            mp += " in frame " + feedback.header.frame_id
            
        ori = feedback.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        rospy.loginfo("r p y = " + str((roll, pitch, yaw)))
        
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            rospy.loginfo( s + ": button click" + mp + "." )
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            rospy.loginfo( s + ": pose changed")
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            rospy.loginfo( s + ": mouse down" + mp + "." )
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            rospy.loginfo( s + ": mouse up" + mp + "." )
            self.createGoalWithValueAndPublish(yaw)
            #self.ims.applyChanges()
            # set up the marker in the current pose
            self.ims.setPose("rotate_" + self.base_name + "_joint", Pose(), header=Header(frame_id = self.base_name +"_link"))

        # With this we apply joint limits
        if yaw > self.upper_joint_limit:
            rospy.logwarn("Going over upper joint limit on " + self.base_name +
                          " val: " + str(yaw) + " > " + str(self.upper_joint_limit))
            return
        if yaw < self.lower_joint_limit:
            rospy.logwarn("Going under lower joint limit on " + self.base_name +
                          " val: " + str(yaw) + " > " + str(self.lower_joint_limit))
            return
        #self.ims.applyChanges()


        
    def createGoalWithValueAndPublish(self, value):
        jt = JointTrajectory()
        jt.joint_names = self.joint_names
        jtp = JointTrajectoryPoint()
        # Get current values and modify the one we want
        names, values = self.getNamesAndMsgListAndModifyValue(self.joint_names,
                                                              self.joint_name,
                                                              value)
        
        jtp.positions = values
        jtp.velocities = [0.0] * len(self.joint_names)
        jtp.accelerations = [0.0] * len(self.joint_names)
        # TODO: Make duration dependent on how big is the movement
        jtp.time_from_start = rospy.Duration(1.0) 
        jt.points.append(jtp)
        self.pub.publish(jt)
        rospy.sleep(1.0)
        # maybe update the marker while we go there?


    def getNamesAndMsgListAndModifyValue(self, joint_names, joint_to_overwrite, value_to_overwrite):
        """ Get the joints for the specified joint_names and return this name list and a list of it's values in joint_states
        Note: the names and values are correlated in position """

        list_to_iterate = joint_names
        curr_j_s = rospy.wait_for_message('/joint_states', JointState)
        ids_list = []
        msg_list = []
        rospy.logdebug("Current message: " + str(curr_j_s))
        for joint in list_to_iterate:
            idx_in_message = curr_j_s.name.index(joint)
            ids_list.append(idx_in_message)
            if joint == joint_to_overwrite:
                msg_list.append(value_to_overwrite)
            else:
                msg_list.append(curr_j_s.position[idx_in_message])
        rospy.logdebug("Current position of joints in message: " + str(ids_list))
        rospy.logdebug("Current msg:" + str(msg_list))

        return list_to_iterate, msg_list # names, values

if __name__=="__main__":
    rospy.init_node("basic_controls")
    rospy.loginfo("Setting up IMS")
    LIM = LinkInteractiveMarker("arm_right_2_joint")
    rospy.spin()

