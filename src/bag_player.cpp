#include <rosgraph_msgs/Clock.h>
#include <rosbag/view.h>
#include <topic_tools/shape_shifter.h>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include "bag_player.h"

batch_ros::BagPlayer::BagPlayer(ros::NodeHandle& _nh, const std::set<std::string>& _wait_topics) :
  nh(_nh), service_spinner(1, &service_queue), wait_topics(_wait_topics)
{
  /* check sim time */
  if (!ros::Time::isSimTime())
  {
    throw std::runtime_error("Simulated time needs to be set");
  }

  /* load topics to wait on */
  if (wait_topics.empty() && nh.hasParam("wait_topics"))
  {
    XmlRpc::XmlRpcValue values;
    nh.getParam("wait_topics", values);
    for(int i = 0; i < values.size(); i++) { wait_topics.insert(values[i]); }
  }

  if (wait_topics.empty())
  {
    std::cerr << "No topics to wait on defined, will publish without waiting for trigger" << std::endl;
  }
  else
  {
    std::set<std::string> final_wait_topics;

    /* process groupped topics */
    for (const std::string& s : wait_topics)
    {
      std::list<std::string> strs;
      boost::split(strs, s, boost::is_any_of(":"));
      if (strs.empty()) throw std::runtime_error("Empty topic name passed");
      else if (strs.size() == 1)
      {
        final_wait_topics.insert(strs.front());
      }
      else
      {
        std::shared_ptr<std::map<std::string, bool>> received_states = std::make_shared<std::map<std::string,bool>>();
        for (const std::string& t : strs) { received_states->emplace(t, false); }
        for (const std::string& t : strs) { groupped_topic_states.emplace(t, received_states); }
        wait_topics.
      }
    }

    std::cout << "Wait on:" << std::endl;
    for (const std::string& topic : wait_topics) { std::cout << "\t" << topic << std::endl; }
  }

  /* load parameters */
  nh.param("clock_rate", clock_rate, clock_rate);
  nh.param("print_waits", print_waits, print_waits);
  nh.param("delay_multiplier", delay_multiplier, delay_multiplier);
  nh.param("output_queue_size", output_queue_size, output_queue_size);

  /* prepare trigger service */
  ros::AdvertiseServiceOptions opts = ros::AdvertiseServiceOptions::create<std_srvs::Trigger>("trigger", boost::bind(&BagPlayer::on_trigger, this, _1, _2), ros::VoidPtr(), &service_queue);
  trigger_srv = nh.advertiseService(opts);
  service_spinner.start();

  /* start clock publisher */
  clock_publisher = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);
  clock_thread = std::thread(&BagPlayer::clock_loop, this);
}

batch_ros::BagPlayer::~BagPlayer(void)
{
  service_spinner.stop();
  signal_continue();

  should_run = false;
  clock_thread.join();
}

void batch_ros::BagPlayer::load_bag(const std::string& path)
{
  /* open bag */
  std::cout << "Loading: " << path << std::endl;
  bag.open(path);

  /* get all topics and initiate publishers */
  std::cout << "Advertising: " << std::endl;
  rosbag::View view(bag);
  for (const rosbag::ConnectionInfo* cinfo : view.getConnections())
  {
    ros::AdvertiseOptions opts(cinfo->topic, output_queue_size, cinfo->md5sum, cinfo->datatype, cinfo->msg_def);
    publishers.emplace(cinfo->topic, nh.advertise(opts));
    std::cout << "\t" << cinfo->topic << " (" << cinfo->datatype << ")" << std::endl;
  }
}

bool batch_ros::BagPlayer::on_trigger(std_srvs::TriggerRequest&, std_srvs::TriggerResponse& resp)
{
  signal_continue();
  resp.success = true;
  return true;
}

void batch_ros::BagPlayer::signal_continue(void)
{
  std::unique_lock lock(publish_mutex);
  can_continue = true;
  cv.notify_all();
}

void batch_ros::BagPlayer::play(void)
{
  std::cout << "Playing bag..." << std::endl;

  /* iterate every message on the bag */
  for (const rosbag::MessageInstance& mi : rosbag::View(bag))
  {
    /* see if we should wait for reception of this message */
    bool should_wait = wait_topics.find(mi.getTopic()) != wait_topics.end();

    /* if not, optionally introduce a delay between messages to avoid filling queues */
    if (!should_wait && last_t.isValid() && delay_multiplier != 0)
    {
      double delta_t = ros::Duration(mi.getTime() - last_t).toSec();
      double delay_time = delta_t * delay_multiplier;
      ROS_DEBUG_STREAM("delaying publish for: " << delay_time << " (real: " << delta_t << ", multiplier: " << delay_multiplier << ")");
      ros::WallRate(ros::Duration(delay_time)).sleep();
    }

    /* advance clock towards current bag time */
    {
      std::unique_lock lock(clock_mutex);
      t = mi.getTime();
    }
    last_t = mi.getTime();

    /* publish the current message */
    ros::Publisher& pub = publishers.at(mi.getTopic());
    pub.publish(mi.instantiate<topic_tools::ShapeShifter>());

    if (should_wait)
    {
      if (print_waits) { std::cout << "Waiting after publish on: " << mi.getTopic() << " t: " << mi.getTime() << std::endl; }

      /* wait for acknowledgement (and safely handle node shutdown request) */
      while (ros::ok())
      {
        std::unique_lock lock(publish_mutex);
        cv.wait_for(lock, std::chrono::milliseconds(100), [this]{ return can_continue; });
        if (can_continue) break;
      }

      /* we were allowed to continue publishing, reset the flag and go ahead to publish */
      can_continue = false;
    }

    /* spin */
    if (!ros::ok()) break;
    ros::spinOnce();
  }

  std::cout << "Done." << std::endl;
}

void batch_ros::BagPlayer::publish_clock(void)
{
  std::lock_guard lock(clock_mutex);

  rosgraph_msgs::Clock clock;
  clock.clock = t;
  clock_publisher.publish(clock);
}

void batch_ros::BagPlayer::clock_loop(void)
{
  ros::WallRate rate(clock_rate);

  while(should_run)
  {
    publish_clock();
    rate.sleep();
  }
}
