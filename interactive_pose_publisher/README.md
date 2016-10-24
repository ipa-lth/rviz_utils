# interactive_pose_publisher
This node create an interactive marker and a tf-frame attached to it.
You may add the 'interactive marker'-plugin to your rviz configuration and use it to move the tf-frame around

After launching the node you can manipulate it via __dynamic_reconfigure__!

Use rosparam to configure the node for startup.

```roslaunch interactive_transform_publisher publisher.launch```
