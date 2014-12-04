#!/usr/bin/env python

# TODO: Use urdf_parser_py.urdf instead. I gave it a try, but got
#  Exception: Required attribute not set in XML: upper
# upper is an optional attribute, so I don't understand what's going on
# See comments in https://github.com/ros/urdfdom/issues/36

import xml.dom.minidom
from math import pi

import rospy


def get_link_mesh_info(link_name, key='robot_description'):
    """Given a link name, search in the URDF for the mesh_path and scale of it.
    If a basic shape (cylinder, box, sphere) is used, the mesh_path returned is the name
    of the basic shape. If no scale is found None is returned.
    Note that mesh_path is as is stored in URDF, most probably unicode.
    Note that scale is a string with the 3 values of scale if it's a mesh.
    If it's a basic shape, it's a dictionary with it's members and values."""
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
                try: # there are frames that are links without visual :/
                    visual_tag = child.getElementsByTagName('visual')[0]
                except:
                    print "Warning: No visual tag for " + name_link
                    continue
                for _child in visual_tag.childNodes:
                    if _child.nodeType is _child.TEXT_NODE:
                        continue
                    if _child.localName == 'geometry':
                        #print "found geometry in visual"
                        found_shape = False
                        found_tag = None
                        try: # if there is no mesh, there is some simple shape used most probably
                            found_tag = _child.getElementsByTagName('mesh')[0]
                            mesh_path = found_tag.getAttribute('filename')
                            found_shape = True
                            try:
                                mesh_scale = found_tag.getAttribute('scale')
                            except Exception, e:
                                print "No scale found for " + str(link_name)
                        except IndexError, e:
                            print "No mesh found for " + str(link_name) + ", trying basic shapes"
                        if not found_shape:
                            try:
                                found_tag = _child.getElementsByTagName('cylinder')[0]
                                mesh_scale = {}
                                length = found_tag.getAttribute('length')
                                radius = found_tag.getAttribute('radius')
                                mesh_scale["length"] = length
                                mesh_scale["radius"] = radius
                                mesh_path = "CYLINDER"
                                found_shape = True
                            except IndexError, e:
                                pass
                        if not found_shape:
                            try:
                                found_tag = _child.getElementsByTagName('box')[0]
                                mesh_scale = {}
                                size = found_tag.getAttribute('size')
                                mesh_scale["size"] = size
                                mesh_path = "CUBE"
                                found_shape = True
                            except IndexError, e:
                                pass
                        if not found_shape:
                            try:
                                found_tag = _child.getElementsByTagName('sphere')[0]
                                mesh_scale = {}
                                radius = found_tag.getAttribute('radius')
                                mesh_scale["radius"] = radius
                                mesh_path = "SPHERE"
                                found_shape = True
                            except IndexError, e:
                                pass
                            # <cylinder length="0.10" radius="0.04"/>
                            # <sphere radius="0.001"/>
                            # <box size="0.02 0.14 0.210"/>

                        if not found_shape:
                            print "No mesh nor basic shape for " + str(link_name) + " found. ERROR."


    return mesh_path, mesh_scale


if __name__=="__main__":
    rospy.init_node("test_get_mesh")
    mesh_path, mesh_scale = get_link_mesh_info("arm_right_2_link")
    print "mesh path: " + str(mesh_path)
    print "mesh_scale: " + str(mesh_scale)
    