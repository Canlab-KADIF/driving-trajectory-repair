#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>

#include "modifying_factory.h"

class BagModifier{
public:
  BagModifier(std::string raw_file_name, std::string modified_bag_file_name, std::string modified_only_bag_file_name);
  bool Modify();

private:
  bool ScanRawBag();

private:
  rosbag::Bag raw_bag_, modified_bag_, modified_only_bag_;
  ModifyingFactory modifying_factory_;
};