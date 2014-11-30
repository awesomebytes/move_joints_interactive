#!/usr/bin/env python

# TODO: Use urdf_parser_py.urdf instead. I gave it a try, but got
#  Exception: Required attribute not set in XML: upper
# upper is an optional attribute, so I don't understand what's going on
# See comments in https://github.com/ros/urdfdom/issues/36

import xml.dom.minidom
from math import pi

import rospy


def get_link_mesh_info(link_name, key='robot_description'):
    description = rospy.get_param(key)
    robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
    mesh_path = ""
    mesh_scale = None
    for child in robot.childNodes:
        #print "child is: " + str(child)
        if child.nodeType is child.TEXT_NODE:
            continue
        if child.localName == 'link':
            #print "child is: " + str(child)
            name_link = child.getAttribute('name')
            #print "Name is: " + name_link
            if name_link == link_name:
                #print "Found " + joint_name + "!!"
                # visual > geometry > mesh
                visual_tag = child.getElementsByTagName('visual')[0]
                for _child in visual_tag.childNodes:
                    if _child.nodeType is _child.TEXT_NODE:
                        continue
                    if _child.localName == 'geometry':
                        #print "found geometry in visual"
                        mesh_tag = _child.getElementsByTagName('mesh')[0]
                        mesh_path = mesh_tag.getAttribute('filename')
                        mesh_scale = mesh_tag.getAttribute('scale')
    return mesh_path, mesh_scale


if __name__=="__main__":
    rospy.init_node("test_get_mesh")
    mesh_path, mesh_scale = get_link_mesh_info("arm_right_2_link")
    print "mesh path: " + str(mesh_path)
    print "mesh_scale: " + str(mesh_scale)
    