#include <rosgraph_msgs/Clock.h>
#include <rosbag/view.h>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include "bag_player.h"

batch_ros::BagPlayer::BagPlayer(ros::NodeHandle& _nh, const std::set<std::set<std::string>>& _wait_topics) :
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
    /* this parameter holds groups as topics joined by a ':' */
    std::vector<std::string> values;
    nh.getParam("wait_topics", values);
    for (const std::string& tg : values)
    {
      /* split groups */
      std::list<std::string> strs;
      boost::split(strs, tg, boost::is_any_of(":"));
      if (strs.empty()) throw std::runtime_error("Empty topic name passed");
      else if (strs.size() == 1)
      {
        /* singleton (not really a group) */
        wait_topics.insert({ strs.front() });
      }
      else
      {
        /* group of topics */
        std::set<std::string> group_topics;
        group_topics.insert(strs.begin(), strs.end());
        if (group_topics.size() != strs.size()) throw std::runtime_error("Duplicate topics for group");
        wait_topics.insert(group_topics);
      }
    }
  }

  /* check for no duplicates among groups */
  std::set<std::string> flat_wait_topics;
  for (const std::set<std::string>& topic_groups : wait_topics)
  {
    size_t before = flat_wait_topics.size();
    flat_wait_topics.insert(topic_groups.begin(), topic_groups.end());
    if (flat_wait_topics.size() != (before + topic_groups.size())) throw std::runtime_error("Duplicate topics among groups");
  }

  if (wait_topics.empty())
  {
    std::cerr << "No topics to wait on defined, will publish without waiting for trigger" << std::endl;
  }
  else
  {
    /* for each topic group, we need to save a "sent" state for each topic in the group */
    for (const std::set<std::string>& topic_group : wait_topics)
    {
      std::shared_ptr<std::map<std::string, SentState>> sent_states = std::make_shared<std::map<std::string, SentState>>();
      for (const std::string& t : topic_group) { sent_states->emplace(t, SentState()); }
      for (const std::string& t : topic_group) { topic_group_states.emplace(t, sent_states); }
    }

    std::cout << "Wait on:" << std::endl;
    for (const std::set<std::string>& topic_group : wait_topics)
    {
      std::cout << (topic_group.size() == 1 ? "Topic: " : "Topic group: ") << std::endl;
      for (const std::string& topic : topic_group)
      {
        std::cout << "\t" << topic << std::endl;
      }
    }
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

void batch_ros::BagPlayer::wait_for_continue(void)
{
  /* wait for acknowledgement (and periodically check for node shutdown request) */
  while (ros::ok())
  {
    std::unique_lock lock(publish_mutex);
    cv.wait_for(lock, std::chrono::milliseconds(100), [this]{ return can_continue; });
    if (can_continue)
    {
      /* we were allowed to continue, reset the flag and stop the wait */
      can_continue = false;
      break;
    }
  }
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
