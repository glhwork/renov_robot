#include "mobile_base_navigation/WallDetect.h"
#include "mobile_base_navigation/LSD.h"
#include "nav_msgs/OccupancyGrid.h"
#include <string>

void testCallback(const nav_msgs::OccupancyGrid& data) {
  line::image_double pixel_map = new line::image_double_s;
  pixel_map->xsize = data.info.width;
  pixel_map->ysize = data.info.height;

  pixel_map->data = new double[pixel_map->xsize * pixel_map->ysize];
  for (size_t i = 0; i < data.data.size(); i++) {
    if (0 == data.data[i]) {
      pixel_map->data[i] = (double)data.data[i];
    } else if (100 == data.data[i]) {
      pixel_map->data[i] = (double)data.data[i];
    } else {
      pixel_map->data[i] = 50.0;
    }
  }
  
  std::cout << "11" << std::endl;
  line::ntuple_list list = new line::ntuple_list_s;
  list = line::lsd(pixel_map);
  
  std::cout << list->dim << std::endl << list->max_size << std::endl << list->size << std::endl;
  delete [] pixel_map->data;
  delete pixel_map;
  delete list;
  // std::cout << sizeof(list->values)/sizeof(double) << std::endl;
  std::cout << "22" << std::endl;

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "detect_node");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("map", 100, testCallback);
  ros::spin();

  return 0;
}