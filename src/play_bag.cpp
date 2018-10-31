#include <ros/ros.h>
#include "bag_player.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bag_player");
  if (argc < 2)
  {
    std::cerr << "Usage: bag_player <bagfile>" << std::endl;
    return -1;
  }

  ros::NodeHandle nh("~");
  batch_ros::BagPlayer player(nh);
  player.load_bag(argv[1]);
  player.play();

  return 0;
}
