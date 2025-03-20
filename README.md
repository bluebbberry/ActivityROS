# ActivityROS

## Demo

In the following [Demo](https://makertube.net/w/5StbcXbQcytXS1VVknSgMY), you can see the Prius from the Gazebo simulation being controlled via Mastodon.

## 1. Build and run package

In ros2_ws, run:

```
colcon build
```

and

```
source ../install/local_setup.bash
```

and then run the package under `ros2_ws/src`:

```
ros2 run cpp_pubsub talker
```

## 2. Run gazebo

```
gz sim
```

## 3. Run bridge between Gazebo and ROS

```
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
```
