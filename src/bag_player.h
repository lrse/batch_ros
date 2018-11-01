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
  /**
   * @brief The BagPlayer class
   *
   * BagPlayer plays a rosbag semi-synchronously. This means that it will publish (on the specified "wait topics")
   * and wait for a Trigger service call. For the rest of the topics it will play them without waiting, at a user defined rate.
   * Moroever, since for some topics there will be a sinchronized subscriber waiting for several messages with matching timestamps,
   * these messages need to be sent and wait only after sending the last one of the group. Thus, "wait topics" can be specified in groups.
   */
  class BagPlayer
  {
    public:
      BagPlayer(void) = delete;
      BagPlayer(ros::NodeHandle& nh);
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
      std::set<std::set<std::string>> wait_topics;
      std::map<std::string, std::string> remaps;
      void signal_continue(void);
      void wait_for_continue(void);

      /* internal state */
      std::map<std::string, ros::Publisher> publishers;
      rosbag::Bag bag;
      ros::Time last_t, t;
      struct SentState
      {
        bool sent = false;
        ros::Time t;
      };
      std::map<std::string, std::shared_ptr<std::map<std::string, SentState>>> topic_group_states;

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
      double start_offset = 0;
  };
}

#endif // BAG_PLAYER_H
