move_joints_interactive, a package to move all the joints controller by joint_trajectory_controller using Rviz markers on each joint.

===

By consulting ros_controllers list_controllers service we get all the controllers and the joints they control.
For every joint we create a Interactive Marker with the mesh (if available) used for it and we attach a ROTATE_AXIS
InteractiveMarkerControl to it hopefully aligned with the axis of rotation (Z axis in my tests, blue axe).
When the Interactive Marker is moved the joint limits previously parsed are checked. If the joint is in a position
between the joint limits a goal for the controller is created (by checking current joint states and overriding with the new
value in the goal).

[![Video of the tool working on REEM-C](http://img.youtube.com/vi/xDY2wfNel6o/0.jpg)](http://youtu.be/xDY2wfNel6o)

Once you run (yeah, I should rename the node):

    rosrun move_joints_interactive node.py

You'll get a message saying something like:

    Rviz configuration file created at: /home/YOUR_USER/nov_reemc_ws/src/move_joints_interactive/rviz/current_rviz.rviz

Just open a Rviz with that file:

    rosrun rviz rviz -d /home/YOUR_USER/nov_reemc_ws/src/move_joints_interactive/rviz/current_rviz.rviz

And you will be able to play with it.

Should work with robots using ros_control and joint_trajectory_controller controllers (with topic /command interface).

=== 

TODO: 
* Check IK if the movement is safe (make it optional but enabled by default).
* Give other backend controllers (joint_states without controller like MoveIt!? 
also other controllers, I've only used joint_trajectory controllers)
* Enable options to override sizes of markers and meshes.
* Make it work with other robots (pr2 initiated)