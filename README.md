# batch_ros

The purpose of **batch_ros** package is to provide a framework for batch execution under ROS, to ensure a node consumes every message in a rosbag, allowing for repeatable results and ensure no message dropping. This not only allows for rigorous testing of a ROS node but also running tests on Continuous Integration (CI) based on rosbag data.

## Motivation

 ROS uses a publish-subscribe mechanism, where messages are not guaranteed to arrive (an outbound or inbound queue may fill up and drop messages). While this makes sense for a live-running ROS node, for testing and debugging, it is benefitial to process absolutely every message in a rosbag and in the same order every time.
 
 The usual solution for this is to write your node so that it supports loading a rosbag itself and then processes messages invoking your own callbacks. This becomes tedious and requires to have a ROS and non-ROS workflow. Moreover, there are complications with ROS time handling and transform loading.
 
 A more naive approach is to simply play your rosbag at a reduced rate where you hope your node keeps up and thus does not loose any message. Besides having no guarantee on this, there are usually only some messages that take longer to process than others (e.g.: an Image vs. an IMU reading). Thus, execution can unnecessarily take very long time.
 
 This package allows you to write your ROS nodes as always and, with minor modification, support batch execution. This means that messages from the bag file will be published as fast as possible but waiting for your node to process them.
 
# How it works 

This package provides a replacement for the **rosbag** player utility. It consists of a ROS node which is able to load a rosbag and be controlled via a ROS service. From the topics included in the bag file, you can specify which ones your node expects and of which the player should ensure proper reception from your node. 

To achieve this, after a message is sent on one of these topics, playing stops immediately and waits for a *trigger* service to be invoked from your node which signals the player to continue publishing. Since services are blocking, this guarantees proper reception of the message on your side. On the other hand, message publishing goes as fast as possible without attempting to reproduce real-time.

For topics which should not be waited upon, the player also reproduces them but without expecting a confirmation after sending. These messages can also be reproduced without waiting or with a time multiplier with respect to real time.

The player also publishes the clock using the current message time, so it works like rosbag with the `--clock` option.

## How to use it

There are two modifications you need to do on your node. First, you need to signal the player to start playing by invoking the trigger service. Thus, you should add this code at the end of your initialization logic (after all topics haven been advertised and subscriptions have been made):

    ros::service::waitForService("/play_bag/trigger");
    std_srvs::TriggerRequest req;
    std_srvs::TriggerResponse resp;
    ros::service::call("/play_bag/trigger", req, resp);

 Second, on every callback associated to a "wait" topic of the player, you need to again invoke the trigger at the end of your callback. Simply repeat this portion:
 
    std_srvs::TriggerRequest req;
    std_srvs::TriggerResponse resp;
    ros::service::call("/play_bag/trigger", req, resp);
    
For most cases this is all that is needed. However, a common pattern is to have a callback for a set of synchronized messages (for example image and camera info). This means that you would produce a single trigger for several topics. To deal with this, you can specify "groupped" topics to wait on. In this case, messages for a topic group are all sent without waiting until the last message in the group is sent.

## Configuration

Configuration is made through ROS parameters, which can be easily specified in a YAML file. Check the `launch/` directory to see an example configuration.
    
## Caveats

### Bag timestamps

For batch_ros to work correctly, you need to fix your bag timestamps so that the message recording time is equal to the header time of the message. This allows proper handling of groupped topics.

### Simulated time

You need to set `use_sim_time` parameter to `true` for the node to work correctly.

### Loading TF transforms

Since the ROS clock will be halted before the bag starts playing, if your node needs to retrieve TF transforms before initialization, calls to `lookupTransform()` will block since any `sleep()` call will never return (as the time will not advance). For the most usual case of loading a set of static transforms, you can use `tf2` which handles fixed transforms with a latched topic which does not require time to advance. However, for this to work you need to perform the lookup like this:

    while(ros::ok() && !tf_buffer->canTransform(from, to, ros::Time(0))) { }
    geometry_msgs::TransformStamped T = tf_buffer->lookupTransform(from, to, ros::Time(0));
    
This will perform a busy wait without depending on ROS time to advance and obtain the latest available transform.
 


 
 