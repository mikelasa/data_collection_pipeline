// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <franka/exception.h>
#include <franka/robot.h>

int main() {

  // launch the robot with the ip address of the robot
  franka::Robot robot("172.17.6.164");

  try {

    // Start
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    //send robot to goal
    std::cout << "Moving to goal pose" << std::endl;
    std::array<double, 7> q_goal = {{0.250, 0.115, -0.433, -1.497, -0.005, 1.75509, 0.5102}};
        // Interpolate over T seconds
    const double T = 3.0;  // seconds
    std::array<double, 7> q_start{};
    bool inited = false;
    double t = 0.0;
    const double settle = 0.10; 

    robot.control([&](const franka::RobotState& state, franka::Duration period) {
    if (!inited) {
      q_start = state.q;   // capture actual start (avoids jump)
      inited = true;
    }
    t += period.toSec();
    double tau = std::min(1.0, std::max(0.0, t / T));
    double s = 10*pow(tau,3) - 15*pow(tau,4) + 6*pow(tau,5);

    std::array<double,7> q_cmd{};
    for (size_t i = 0; i < 7; ++i)
      q_cmd[i] = q_start[i] + s * (q_goal[i] - q_start[i]);

    franka::JointPositions jp(q_cmd);

    // Hold final pose a bit so velocity truly goes to zero, then finish.
    if (t >= T + settle) {
      return franka::MotionFinished(jp);
    }
    return jp;
  });

  // End move to goal and print joint configuration
  std::cout << "Finished moving to initial joint configuration." << std::endl;
  std::array<double, 7> joint_positions = robot.readOnce().q;
  std::cout << "Current joint positions: ";
  for (const auto& val : joint_positions) {
    std::cout << val << " ";
  }
  std::cout << std::endl;

  //press enter to start the trajectory
  std::cout << "Press Enter to start the trajectory..." << std::endl;
  std::cin.ignore();

  // Load the trajectory from CSV
  std::cout << "Loading csv trajectory" << std::endl;
  std::ifstream csv_file_stream("/home/mikel/data_collection_pipeline/data/test_traj/demonstration.csv");
  std::vector<std::array<double, 7>> samples;
  std::string line;
  //extract the first 7 elements of each line
  while (std::getline(csv_file_stream, line)) {
    std::istringstream line_stream(line);
    std::string cell;
    std::array<double, 7> sample;
    for (size_t i = 0; i < 7; ++i) {
      if (std::getline(line_stream, cell, ',')) {
        sample[i] = std::stod(cell);
      } else {
        sample[i] = 0.0; // default value if not enough data
      }
    }
    samples.push_back(sample);
  }

  //print samples first line
  std::cout << "First sample: ";
  for (const auto& val : samples[0]) {
    std::cout << val << " ";
  }
  std::cout << std::endl;

  // Execute the trajectory
  std::cout << "Executing trajectory" << std::endl;
  size_t index = 0;
  std::vector<franka::RobotState> states;
  
  robot.control([&](const franka::RobotState &robot_state, franka::Duration time_step) -> franka::JointPositions {
    states.push_back(robot_state);
    index += time_step.toMSec();
    if (index >= samples.size() - 1) {
      return franka::MotionFinished(franka::JointPositions(samples.back()));
    }
    return samples[index];
  });

  std::cout << "Finished trajectory" << std::endl;

  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
}