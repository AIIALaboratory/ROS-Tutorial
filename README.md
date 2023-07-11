# ROS TUTORIAL
This is an introductory tutorial on creating the workspace, package and nodes for a functioning ROS system. In this repository, the necessary intrusctions for the creation of the files are provded and also an example of ROS nodes for images segmentation is given.

## ROS INSTALLATION
For the intallation of ROS-Noetic to your PC follow the instractions presented in the [ROS Wiki](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

To confirm that the ROS is properly installed, run `roscore` in your terminal.

## ROS WORKSPACE & PACKAGE
The development process for ROS must be done inside the ROS workspace. By sourcing the workspace the ROS system knows where to find the packages and the related files. Specifically, the ROS package is the basic organizational unit containing all the related files, nodes, codes and dependencies. 

To set up the ROS environment to develop it is necessary to follow a certain structure. 
1. Create the workspace: Create a directory and inside of it a folder src. Then inside the direcotry perform `catkin_make` (creates direcotries `build` and `devel` and runs `cmake` command)
2. Create the package: Inside the source folder `src` create the folder of the package by running the command `catkin_create_pkg <name> rospy std_msgs` (DO NOT USE CAPTIAL LETTERS IN THE NAME OF THE PACKAGE FOLDER).
3. Inside the workspace to build the changes of step 2 run the command `catkin_make`.
4. Open the `.bashrc` file (located inside the `HOME` directory and to find it press `ctrl + H` to show hidden files), using the developing enviroment (VSCode), and source the workspace by pasting `source ~/<workspace_name>/devel/setup.bash`.

As a developing enviroment VSCode can be used.



## ROS NODES AND TOPICS 
Most of the communication for a robot is happening on the nodes through topics and messages. Specifically, these terms are defined as:

* Nodes: A software module that performs computations or controls a specific hardware component. ROS node is performes a single task in a ROS system and it publishes or subscribes to a topic or contains a program that enables a ROS service. The communication between nodes is carried out by sending messages through topics. The command `rosrun` is used to run node: `rosrun <package name> <node name>`.
* Topics: Topics allow nodes to communicate by publishing and subscribing to messages. One topic represents a stream of data where information is published by a node and can be received by a node subsrcibed to the topic. Topics can boradcast data to multiple subscribers simultaneously.
* Messages: Individual set of data published by a node to a topic. There are a lot of different types of messages but the standard can be found [here](http://wiki.ros.org/std_msgs)

Each node has the following information:
1. Message type which node uses
2. Whether or not it is a publisher
3. Whether or not it is a subscriber

To visualize the communication of nodes in ROS the command `rqt graph` can be used.
![image](assets/rgt_graph.png)

## ROS NODES EXAMPLE

Here will be presented the most important parts of the provided example nodes, in order to understand how to create a node that publishes and subscribes.

```python
#!/usr/bin/env python3
```
First of all it is necessary to add at the start of each of the python files the above line in order to find the python installation.

First, the publisher node `raw_image_publisher.py` will be presented.

```python
import rospy
from sensor_msgs.msg import Image
```
To write a ROS node the `rospy` library must be imported. The import of `sensor_msgs.msg` allows the node to publish a message of type `Image`. 

```python
rospy.init_node('raw_image_publisher', anonymous = True)
pub = rospy.Publisher('raw_image', Image, queue_size = 1)
```

First the node is initialized `rospy.init_node('raw_image_publisher', anonymous = True)` by providing the name of the node. Without this information rospy cannot start communicating with the ROS Master, which registers the nodes and coordinates the communication between them in the ROS system. The `anonymous = True` argument ensures that each node has a unique name by adding randomn numbers at the end of the name.
`pub = rospy.Publisher('raw_image', Image, queue_size = 1)` declares that the node publishes to the `raw_image` topic using the message type `Image`, which is the class `sensor_msgs.msg`. The messages are published asynchronously by the node and sent to a queue to be processed (first in first out). The `queue_size` argument limits the amount of outgoing messages. The queue_size must be set accordingly to the publishing rate. In case the node publishes faster than the queue size can handle messages will be dropped. In this example the queue size is set to 1, meaning that a new published message will always prevent any older not yet sent messages to be dropped.

```python
rate = rospy.Rate(0.2)
bridge = CvBridge()
```
These lines create a `Rate` object and a `CvBridge` object. The `Rate` object is accompanied by the method sleep(), which regulates the execution of the loop in order to achieve the desired frequency set inside the `Rate` object. In this example the frequency is set to 0.2 meaning the node publishes every 5 seconds an `Image` message to the topic. This frequency is selected because later in the `visualizer` node each image is shown for a duartion of 5 seconds. The `CvBridge` object converts between ROS `Image` messages and OpenCV images.

```python
while not rospy.is_shutdown():
    raw_img = cv2.imread(os.path.join(frames_dir,frames[counter]))
    pub.publish(bridge.cv2_to_imgmsg(raw_img, "bgr8"))
    counter = counter + 1
    if counter == n:
        counter = 0

    rate.sleep()
```
This is the loop that executes the task associated with the node. First, the node checks the `rospy.is_shutdown()` flag if it should exit. Next it reads an image and publishes it to the `raw_image_publisher` topic. The published image has been converted to a message `Image` type via the `CvBridge()`. Finally, the counter is updated and the loop calls `rate.sleep()`, which sleeps long enough to maintain the desired rate through the loop. 

```python
try:
    img_publish()
except rospy.ROSInterruptException:
    print('Shutting down')
```
In addition to the standard Python `__main__` check, this catches a `rospy.ROSInterruptException` exception. This exception can be thrown by `rospy.sleep()` and `rospy.Rate.sleep()` methods when `ctrl-C` is pressed or the node is otherwise shutdown.

Next will be presented the most important parts of an example of a node that receives messages.

```python
def main(args):
    node_name = 'simar_segmentation_node'
    rospy.init_node(node_name, anonymous=True)
    Pipeline_Segmentation_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down')
    return 0

if __name__ == '__main__':
    main(sys.argv)
```

Another way to create a node is by using object oriented programming, which allows for a file to be both a publisher and a subscriber. In this case after the standard Python `__main__` check instead of calling the function of the node the `main` function is called. Inside this function the node is initialized the same as in the previous node and the node object `Pipeline_Segmentation_node()` is called. Finally in order to start the event loop the `rospy.spin()` is used. It i responsible for handling events such as receiving and processing incoming messages, inviking callbacks and responding to system events. Also, while in the event loop the `rospy.spin()` keeps the main function running to perform multiple tasks and listens for the termination signal (`ctrl + C`) amd other shutdown requests.

```python
# init model
self.model_segmentation = PipeSeg(cfg.num_classes, is_training = False, criterion = None, ohem_criterion = False)
state_dict = torch.load(self.model_weights_path, map_location = torch.device('cpu'))
if 'model' in state_dict.keys():
    state_dict = state_dict['model']
self.model_segmentation.load_state_dict(state_dict, strict = False)
self.model_segmentation.to(self.device)
```
In this example the node needs to produce the segmentation mask of each RGB Image. Hence fisrt the segmentation model is initialized.

```python
self.img_sub_topic = 'raw_image'
self.sub_image = rospy.Subscriber( self.img_sub_topic, Image, self.semantic_segmentation, queue_size = 1)
```
First the topic that the node will be subscribed to is defined `img_sub_topic = 'raw_image'`. Then follows the `rospy.Subscriber( self.img_sub_topic, Image, self.semantic_segmentation, queue_size = 1)` which receives the message. First, it's declared the topic that the node listens to, next the type of the received message and the queue_size and finally the callback function `self.semantic_segmentation`. The `Subscriber` once it receives the message it goes to the callback function and passes the message as a function. 

```python
def semantic_segmentation(self, img_msg):
    frame = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
    mask, frame_processed = predict(cfg, self.model_segmentation, frame, self.device)
    mask = mask.astype('uint8')
    
    self.pub_image.publish(self.bridge.cv2_to_imgmsg(frame_processed, "bgr8"))
    self.pub_mask.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
```
The callback function first converts the message to an OpenCV image then predicts the segmentation mask and publishes, as presented previously, the segmentation mask and the RGB image that was processed.

```python
self.mask_pub_topic = 'mask_image'
self.pub_mask = rospy.Publisher(self.mask_pub_topic, Image, queue_size = 1)
self.img_pub_topic = 'raw_of_mask_image'
self.pub_image = rospy.Publisher(self.img_pub_topic, Image, queue_size = 1)
```
The publishers are declared the same way as presented in the `raw_image_publisher.py` node.

Once the node files are created they must be saved inside the src file of the package and change their mode. Their mode can be changed through the command `chmod +x *`.

Next, add to the same file src of the package all the necessary folders for the segmentation model.

Finally, to check that the nodes performs their task and communicates appropriately with other follow these steps.
1. Run `roscore` in terminal
2. In a new terminal window change directory to the package `roscd <package name>/src`
3. Run the node with the command `rosrun <package name> <node name>`
4. repeat step 3 in a new terminal window for the rest nodes.
5. (Optional) in a new terminal window use the command `rqt_graph` to visualize the communication between the nodes.
