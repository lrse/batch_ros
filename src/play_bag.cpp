#include <ros/ros.h>
#include "bag_player.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bag_player");
  if (argc < 2)
  {
    std::cerr << "Usage: bag_player <bagfile> [ <topics ...> ]" << std::endl;
    return -1;
  }

  ros::NodeHandle nh("~");
  std::set<std::string> wait_topics;
  for (int i = 2; i < argc; i++) { wait_topics.insert(argv[i]); }
  batch_ros::BagPlayer player(nh, wait_topics);
  player.load_bag(argv[1]);
  player.play();

  return 0;
}
