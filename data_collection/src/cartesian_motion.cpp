#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <iterator>
#include <vector>
#include <array>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>

int main() {

  std::ifstream csv_file("/home/mikel/data_collection_pipeline/data/test_traj/demonstration.csv");
  std::vector<std::array<double, 16>> poses;
  std::string line;
  while (std::getline(csv_file, line)) {
    if (line.empty()) continue; // Skip empty lines
    std::array<double, 16> pose{};
    std::stringstream ss(line);
    int i = 0;
    std::string value;
    while (std::getline(ss, value, ',') && i < 16) {
      try {
        pose[i++] = std::stod(value);
      } catch (const std::invalid_argument&) {
        break; // Skip this line if conversion fails
      }
    }
    if (i == 16) poses.push_back(pose); // Only add if 16 values were read
  }

  try {
    franka::Robot robot("172.17.6.164");
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});


    size_t index = 0;
    robot.control([&](const franka::RobotState &robot_state, franka::Duration time_step) -> franka::CartesianPose {
      if (index >= poses.size() - 1) {
        return franka::MotionFinished(franka::CartesianPose(poses.back()));
      }
      // Send next pose
      return franka::CartesianPose(poses[index++]);
    });
  } catch (const franka::ControlException &e) {
    std::cout << e.what() << std::endl;
  } catch (const franka::Exception &e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
}