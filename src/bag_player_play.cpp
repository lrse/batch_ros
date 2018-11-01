#include <rosgraph_msgs/Clock.h>
#include <rosbag/view.h>
#include <topic_tools/shape_shifter.h>
#include <iostream>
#include "bag_player.h"

void batch_ros::BagPlayer::play(void)
{
  bool started = false;

  /* get bag time range */
  rosbag::View v(bag);
  ros::Time start = v.getBeginTime();
  //ros::Time end = v.getEndTime();

  /* iterate every message on the bag */
  for (const rosbag::MessageInstance& mi : rosbag::View(bag, start + ros::Duration(start_offset)))
  {
    /* advance clock towards current bag time */
    {
      std::unique_lock lock(clock_mutex);
      t = mi.getTime();
    }

    if (!started)
    {
      std::cout << "Waiting for start trigger..." << std::endl;
      wait_for_continue();
      started = true;

      ros::WallDuration(1.0).sleep(); // TODO: this eliminates race condition, improve
      std::cout << "Playing bag..." << std::endl;
    }

    /* see if we should wait for reception of this message */
    auto it = topic_group_states.find(mi.getTopic());
    bool is_wait_topic = (it != topic_group_states.end());

    /* if not, optionally introduce a delay between messages to avoid filling queues */
    if (!is_wait_topic && last_t.isValid() && delay_multiplier != 0)
    {
      double delta_t = ros::Duration(mi.getTime() - last_t).toSec();
      double delay_time = delta_t * delay_multiplier;
      ROS_DEBUG_STREAM("delaying publish for: " << delay_time << " (real: " << delta_t << ", multiplier: " << delay_multiplier << ")");
      ros::WallRate(ros::Duration(delay_time)).sleep();
    }
    last_t = mi.getTime();

    /* wait for clock message to propagate, so we are sure we are publishing once the clock corresponds to message timestamp */
    ros::WallRate rate(clock_rate);
    while (ros::ok() && ros::Time::now() < t)
    {
      std::cout << "Waiting for clock to advance to " << t << std::endl;
      ros::spinOnce();
      rate.sleep();
    }

    /* publish the current message */
    ros::Publisher& pub = publishers.at(mi.getTopic());
    pub.publish(mi.instantiate<topic_tools::ShapeShifter>());

    /* act on wait topics */
    if (is_wait_topic)
    {
      std::shared_ptr<std::map<std::string, SentState>> sent_states = it->second;
      SentState& state = sent_states->at(mi.getTopic());

      /* topic is in a group */
      if (sent_states->size() > 1)
      {
        bool reset_group = false;

        /* topic is in group and a previous message was already sent on the topic, which means any other message
         * for this group would be of a previous time */
        if (state.sent)
        {
          ROS_WARN_STREAM("message for topic " << mi.getTopic() << " in group was already sent for time " << state.t << " (t: " << mi.getTime());
          reset_group = true;
        }
        /* check if this message is newer than other sent message of this group */
        else
        {
          for (auto& [k,v] : *sent_states)
          {
            if (v.sent && mi.getTime() > v.t)
            {
              ROS_WARN_STREAM("Received a newer message for topic in group");
              reset_group = true;
              break;
            }
          }
        }

        /* we will consider other messages already sent for this group as not sent */
        if (reset_group)
        {
          for (auto& [k,v] : *sent_states)
          {
            v.sent = false;
            v.t = ros::Time();
          }
        }
      }

      /* update sent state for this message */
      state.sent = true;
      state.t = mi.getTime();

      /* now see if we should actually wait or not */
      bool should_wait = false;
      if (sent_states->size() == 1)
      {
        /* for singleton groups we simply wait for trigger */
        should_wait = true;
      }
      else
      {
        /* for topic groups, we will wait only after all messages from the group where sent */
        should_wait = true;
        for (auto& [k,v] : *sent_states)
        {
          if (!v.sent)
          {
            should_wait = false;
            break;
          }
        }

        /* all sent, we will wait. reset all sent states before waiting */
        if (should_wait)
        {
          for (auto& [k,v] : *sent_states)
          {
            v = SentState();
          }
        }
      }

      if (should_wait)
      {
        if (print_waits) { std::cout << "Waiting after publish on: " << mi.getTopic() << " t: " << mi.getTime() << std::endl; }
        wait_for_continue();
        if (print_waits) { std::cout << "Continuing..." << std::endl; }
      }
      else
      {
        if (print_waits) { std::cout << "Not waiting after publish of " << mi.getTopic() << " t: " << mi.getTime() << std::endl; }
      }
    }

    /* spin and check for shutdown requests */
    if (!ros::ok()) break;
  }

  std::cout << "Done." << std::endl;
}
