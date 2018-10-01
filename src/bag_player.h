#ifndef BAG_PLAYER_H
#define BAG_PLAYER_H

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_srvs/Trigger.h>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <ros/callback_queue.h>

namespace batch_ros
{
  class BagPlayer
  {
    public:
      BagPlayer(ros::NodeHandle& nh);
      ~BagPlayer(void);

      void play(void);
      void load_bag(const std::string& path);

    private:
      ros::Time last_t, t;
      rosbag::Bag bag;
      ros::NodeHandle& nh;

      ros::AsyncSpinner service_spinner;
      ros::CallbackQueue service_queue;
      ros::ServiceServer trigger_srv;
      bool on_trigger(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& resp);

      bool can_continue = false;
      std::mutex publish_mutex;
      std::condition_variable cv;
      std::set<std::string> wait_topics;
      void signal_continue(void);

      std::map<std::string, ros::Publisher> publishers;

      ros::Publisher clock_publisher;
      std::mutex clock_mutex;
      int clock_rate = 100;
      std::thread clock_thread;
      bool should_run = true;
      void clock_loop(void);
      void publish_clock(void);

      bool print_waits = false;
      double delay_multiplier = 0;
      int output_queue_size = 10;
  };
}

#endif // BAG_PLAYER_H
