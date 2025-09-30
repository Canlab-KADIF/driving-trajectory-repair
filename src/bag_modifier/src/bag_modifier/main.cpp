#include "bag_modifier.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "bag_modifier");

  // std::string raw_filename = "_all_009_perception_2024-02-29-20-07-33.bag";
  // std::string raw_filename = "_all_008_merged.bag";
  // std::string raw_filename = "_all_009_merged.bag";

  // std::cout << argv[0] << std::endl;
  // std::cout << argv[1] << std::endl;

  // raw_filename = "_all_009_merged.bag";
  // raw_filename = "_all_008_perception_2024-02-29-20-56-30.bag";

  // std::string modified_bag_file_name = "_all_009_modified.bag";
  // std::string modified_only_bag_file_name = "_all_009_modified_only.bag";

  std::string raw_filename = argv[1];
  std::string modified_bag_file_name = raw_filename.substr(0, raw_filename.find(".bag")) + "_modified_ver_2.bag";
  std::string modified_only_bag_file_name = raw_filename.substr(0, raw_filename.find(".bag")) + "_modified_only_ver_2.bag";

  // std::string modified_bag_file_name = "_all_009_modified_ctrv_spline_refac.bag";
  // std::string modified_only_bag_file_name = "_all_009_modified_only_ctrv_spline_refac.bag";

  BagModifier bag_modifier(raw_filename, modified_bag_file_name, modified_only_bag_file_name);

  if(!bag_modifier.Modify()){
    std::cout << "failed to Modify bag!" << std::endl;
    return -1;
  }

  return 0;
}