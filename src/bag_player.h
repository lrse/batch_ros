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
      BagPlayer(void) = delete;
      BagPlayer(ros::NodeHandle& nh, const std::set<std::string>& wait_topics = {});
      ~BagPlayer(void);

      void play(void);
      void load_bag(const std::string& path);

    private:
      ros::NodeHandle& nh;
      ros::Publisher clock_publisher;

      /* trigger service */
      ros::AsyncSpinner service_spinner;
      ros::CallbackQueue service_queue;
      ros::ServiceServer trigger_srv;
      bool on_trigger(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& resp);

      /* trigger handling */
      bool can_continue = false;
      std::mutex publish_mutex;
      std::condition_variable cv;
      std::set<std::string> wait_topics;
      void signal_continue(void);

      /* internal state */
      std::map<std::string, ros::Publisher> publishers;
      std::map<std::string, std::shared_ptr<std::map<std::string, bool>>> groupped_topic_states;
      rosbag::Bag bag;
      ros::Time last_t, t;

      /* clock handling */
      std::mutex clock_mutex;
      std::thread clock_thread;
      bool should_run = true;
      void clock_loop(void);
      void publish_clock(void);
      int clock_rate = 100;

      /* options */
      bool print_waits = false;
      double delay_multiplier = 0;
      int output_queue_size = 10;
  };
}

#endif // BAG_PLAYER_H
