move_joints_interactive, a package to move all the joints controller by joint_trajectory_controller using Rviz markers on each joint.

===

By consulting ros_controllers list_controllers service we get all the controllers and the joints they control.
For every joint we create a Interactive Marker with the mesh (if available) used for it and we attach a ROTATE_AXIS
InteractiveMarkerControl to it hopefully aligned with the axis of rotation (Z axis in my tests, blue axe).
When the Interactive Marker is moved the joint limits previously parsed are checked. If the joint is in a position
between the joint limits a goal for the controller is created (by checking current joint states and overriding with the new
value in the goal).

TODO: 
* Check IK if the movement is safe (make it optional but enabled by default).
* Fix the visualization, when you move a joint, it moves twice (the amount you moved 
in the interactive marker + the movement of the real joint).
* Give other backend controllers (joint_states without controller like MoveIt!? 
also other controllers, I've only used joint_trajectory controllers)
* Enable options to override sizes of markers and meshes.
* Read and use basic geometries from URDF when there is no mesh