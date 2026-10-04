# ROS commands from lecture 2 for using ROS
## ROS 2 terms
1. Nodes - Applications that do things. Can Publish Messages or Subscribe to Messages through Topics.
2. Messages - Data packets exchanged between Nodes. Makes sense why? Because in Python or any other programming language, the output of one e.g. .py file can be used in another program as data. 
3. Topics - One-way Message channel between Publishers and Subscriber to transfer different Messages. (Publisher) -> (msg) -> (Subscriber), where -> (msg) -> the topic is the method by which Message "msg" is transferred from publisher to the subscriber.
4. Publisher - Part of the Node (Application, e.g. in the .py file), that handles the sending of messages on a Topic (Message channel).
5. Subscriber - Part of the Node receiving the messages on a Topic.

## Alternative ROS 2 terms we come across
1. Services - Two-way communication between Nodes via Topics where one sends a request for certain Messages and the other Node responds with that Message.
2. Actions - Long running tasks (Services) with Feedback during runs.
3. Launch file - So, in our case, it is the python files or scripts for launching the multiple applications (Nodes), we need for our different control stacks (Navigation, Perception, Control, etc.).
4. Packages - ROS Modules containing these Nodes, Messages, Services, and other whatnots.

# Now, actual commands in terminal for different things
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc  # add ros2 path to hidden configuration file

source ~/.bashrc  # reset the environment variables and configurations

ros2 run demo_nodes_py talker  # run the talker demo node in one terminal. Output: Publishing [msg]

ros2 run demo_nodes_py listener  # run the listener demo node in another terminal. Output: I heard [msg]

ros2 topic list  # list all the active topics. For you, means what one way channels Messages are being published from and subscribed to by different Nodes.

ros2 node list  # list all the active nodes

ros2 topic info /chatter  # show the topic information

ros2 topic info /chatter --verbose  # show the topic information with additional information like names of Nodes publishing and subscribing to topics, their Quality of Service (QoS) profiles, etc.

ros2 run rqt_graph rqt_graph  # Run Ros Graph GUI. After launching, click on refresh Node graph to the left of the Nodes view , e.g. "Nodes only", to view active nodes, topics, and their connections.

ros2 topic pub /title std_msgs/String {“data: ‘R7021E: Advanced Robotics’”}  # publish a String message on the topic “/title” with the data “R7021E: Advanced Robotics”. std_msgs is the library that message format comes from.

ros2 topic echo /title  # echo (print) the message on the topic “/title”

* Message libraries to ROS 2 (default): std_msgs, geometry_msgs, sensor_msgs


## Creating a workspace

1. Create a directory:
mkdir –p new_ws/src  # create a base folder (work station - ws) with a source files folder (src) inside it.

2. change the working directory
cd new_ws/src

3. create the python (.py) package inside the src folder to hold Nodes, Messages, Topics, etc.
Structure for ROS 2 .py package, e.g. package 1:
<br>py_package_1(Folder)/</br>
<br>package.xml</br>
<br>resource(Subfolder)/py_package_1</br>
<br>setup.cfg</br>
<br>setup.py</br>
<br>py_package_1/</br>

Command:
ros2 pkg create –-build-type ament_python test_pkg  # create a python package using python build-type. Package name = test_pkg here. After creating, it will have built the file structure shown above.

3. Create a python Node (.py script) using rclpy library (py API for ROS2)

touch sender.py  # create a new python script inside src/test_pkg/test_pkg folder created via command no.2. The node inside the sender script will be called Sender(Node), via class inheritance conventions.

Basics in any standard Python script that the Node will follow:
<b>Imports -> Class (for Node) -> Main Function (def main) -> Entry points</b>












