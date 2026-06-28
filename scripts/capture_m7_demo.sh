#!/bin/bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Starting full system (with MuJoCo GUI)..."
ros2 launch teleop_bringup full_system.launch.py &
LAUNCH_PID=$!

echo "Waiting for controllers to activate..."
while ! ros2 control list_controllers | grep -A 1 cartesian_impedance_controller | grep active > /dev/null; do
  sleep 0.5
done
echo "Controllers active! Stabilizing physics..."
sleep 1

echo "Switching command type to POSE tracking..."
ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: 2}"
sleep 1

echo "Starting dummy heartbeat for safety monitor..."
ros2 topic pub /teleop/heartbeat std_msgs/msg/Header "{frame_id: 'dummy'}" -r 50 &
HB_PID=$!

# Wait for ros2 topic pub to initialize so the heartbeat isn't interrupted
sleep 2

echo "Stopping manual teleop to allow batch generator to take over..."
pkill -f "teleop_input_node" || true

echo "Starting GIF recorder..."
python3 scripts/record_demo_gif.py media/m7/grasp_demo.gif --seconds 20 --fps 15 &
REC_PID=$!

# Give recorder a moment to subscribe
sleep 2

echo "Starting batch generator..."
ros2 run synth_data_gen batch_generator --ros-args \
  -p episodes:=1 \
  -p hover_duration:=4.0 \
  -p descend_duration:=4.0 \
  -p grasp_pause:=2.0 \
  -p lift_duration:=4.0 &
BATCH_PID=$!

echo "Waiting for batch generator and recorder to finish..."
wait $BATCH_PID
wait $REC_PID

echo "Killing ROS 2 launch..."
kill -INT $LAUNCH_PID
kill $HB_PID 2>/dev/null || true
sleep 3
pkill -f "ros2 launch" || true
pkill -f "mujoco_sim" || true
pkill -f "servo_node" || true
pkill -f "controller_manager" || true

echo "Done capturing M7 demo!"
