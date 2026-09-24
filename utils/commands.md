> Run colcon build sequential
colcon build --event-handlers desktop_notification- status- --executor sequential

> Open vineyard world
cd ros2_gazebo_mestrado/
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch agrobot_gazebo simulation.launch.py


>> new terminal
cd ros2_gazebo_mestrado/
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch agrobot_description spawn_agrobot.launch.py


$env:IPAddress = "YOUR_IP"