#include <chrono>
#include <iostream>
#include <thread>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/robot.h>

/**
 * @example communication_test.cpp
 * An example indicating the network performance.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main()
{
  franka::Robot robot("172.17.6.164");

  uint64_t counter = 0;
  double avg_success_rate = 0.0;
  double min_success_rate = 1.0;
  double max_success_rate = 0.0;
  uint64_t time = 0;
  std::cout.precision(2);
  std::cout << std::fixed;

  try {
    robot.setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    std::cout << "Finished moving to initial joint configuration." << std::endl << std::endl;
    std::cout << "Starting communication test." << std::endl;

    franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    robot.control(
        [&time, &counter, &avg_success_rate, &min_success_rate, &max_success_rate, zero_torques](
            const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
          time += period.toMSec();
          if (time == 0.0) {
            return zero_torques;
          }
          counter++;

          if (counter % 100 == 0) {
            std::cout << "#" << counter
                      << " Current success rate: " << robot_state.control_command_success_rate
                      << std::endl;
          }
          std::this_thread::sleep_for(std::chrono::microseconds(100));

          avg_success_rate += robot_state.control_command_success_rate;
          if (robot_state.control_command_success_rate > max_success_rate) {
            max_success_rate = robot_state.control_command_success_rate;
          }
          if (robot_state.control_command_success_rate < min_success_rate) {
            min_success_rate = robot_state.control_command_success_rate;
          }

          if (time >= 10000) {
            std::cout << std::endl << "Finished test, shutting down example" << std::endl;
            return franka::MotionFinished(zero_torques);
          }

          // Sending zero torques - if EE is configured correctly, robot should not move
          return zero_torques;
        },
        false, 1000);
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  avg_success_rate = avg_success_rate / counter;

  std::cout << std::endl
            << std::endl
            << "#######################################################" << std::endl;
  uint64_t lost_robot_states = time - counter;
  if (lost_robot_states > 0) {
    std::cout << "The control loop did not get executed " << lost_robot_states << " times in the"
              << std::endl
              << "last " << time << " milliseconds! (lost " << lost_robot_states << " robot states)"
              << std::endl
              << std::endl;
  }

  std::cout << "Control command success rate of " << counter << " samples: " << std::endl;
  std::cout << "Max: " << max_success_rate << std::endl;
  std::cout << "Avg: " << avg_success_rate << std::endl;
  std::cout << "Min: " << min_success_rate << std::endl;

  if (avg_success_rate < 0.90) {
    std::cout << std::endl
              << "WARNING: THIS SETUP IS PROBABLY NOT SUFFICIENT FOR FCI!" << std::endl;
    std::cout << "PLEASE TRY OUT A DIFFERENT PC / NIC" << std::endl;
  } else if (avg_success_rate < 0.95) {
    std::cout << std::endl << "WARNING: MANY PACKETS GOT LOST!" << std::endl;
    std::cout << "PLEASE INSPECT YOUR SETUP AND FOLLOW ADVICE ON" << std::endl
              << "https://frankaemika.github.io/docs/troubleshooting.html" << std::endl;
  }
  std::cout << "#######################################################" << std::endl << std::endl;

  return 0;
}
