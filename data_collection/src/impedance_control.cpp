#include <cmath>
#include <iostream>
#include <iterator>
#include <mutex>
#include <thread>
#include <fstream> 
#include <queue>   

#include <Eigen/Dense>

#include <franka/exception.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/model.h>

int main ()
{
    // launch the robot with the ip address of the robot
    franka::Robot robot("172.17.6.164");
    
    try {

        // Compliance parameters
        const double K_trans{0};
        const double K_rot{0};
        const double k_nullspace_trans{0};
        const double k_nullspace_rot{0};

        //create the impedance matrixes
        Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
        stiffness.setZero();
        stiffness.topLeftCorner(3, 3) << K_trans * Eigen::MatrixXd::Identity(3, 3);
        stiffness.bottomRightCorner(3, 3) << K_rot * Eigen::MatrixXd::Identity(3, 3);
        damping.setZero();
        damping.topLeftCorner(3, 3) << 2.0 * sqrt(K_trans) * Eigen::MatrixXd::Identity(3, 3);
        damping.bottomRightCorner(3, 3) << 2.0 * sqrt(K_rot) *Eigen::MatrixXd::Identity(3, 3);

        // Nullspace parameters same as the compliance parameters
        Eigen::MatrixXd null_stiffness(6, 6), null_damping(6, 6);
        null_stiffness.setZero();
        null_stiffness.topLeftCorner(3, 3) << k_nullspace_trans * Eigen::MatrixXd::Identity(3, 3);
        null_stiffness.bottomRightCorner(3, 3) << k_nullspace_rot * Eigen::MatrixXd::Identity(3, 3);
        null_damping.setZero();
        null_damping.topLeftCorner(3, 3) << 2.0 * sqrt(k_nullspace_trans) * Eigen::MatrixXd::Identity(3, 3);
        null_damping.bottomRightCorner(3, 3) << 2.0 * sqrt(k_nullspace_rot) * Eigen::MatrixXd::Identity(3, 3);
        
        // Set additional parameters always before the control loop, NEVER in the control loop!
        // Set collision behavior.
        robot.setCollisionBehavior(
            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
            {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
            {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
        robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
        robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

        //set robot payload
        robot.setLoad(
            0.0,                                       // mass
            {0.0, 0.0, 0.1},                          // CoM in tool frame (example)
            {0.0, 0.0, 0.0,                            // 3x3 inertia matrix (flat array)
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0}
        );

        // Load the robot model
        franka::Model model(robot.loadModel());
        
        // Read the robot state once
        franka::RobotState initial_state = robot.readOnce();

        // equilibrium point is the initial position
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        Eigen::Vector3d position_d(initial_transform.translation());
        Eigen::Quaterniond orientation_d(initial_transform.rotation());

        // Define the desired joint configuration
        Eigen::VectorXd q_d(7);
        q_d << initial_state.q[0], initial_state.q[1], initial_state.q[2],
            initial_state.q[3], initial_state.q[4], initial_state.q[5], initial_state.q[6];
        //print initial position
        std::cout << "Initial position: " << position_d.transpose() << std::endl
                << "Initial orientation: " << orientation_d.coeffs().transpose() << std::endl
                << "Press Enter to continue..." << std::endl
                << "Press Enter to start impedance control..." << std::endl;
        std::cin.ignore();

        // Define the impedance control callback
        std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [&model, &stiffness, &damping, &null_stiffness, &null_damping, &q_d, &robot,
                                      &position_d, &orientation_d](const franka::RobotState& robot_state,
                                        franka::Duration /*duration*/) -> franka::Torques {

            // get state variables
            std::array<double, 7> coriolis_array = model.coriolis(robot_state);
            std::array<double, 42> jacobian_array =model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
            std::array<double, 7> gravity_array = model.gravity(robot_state);
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());

            // convert to Eigen
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
            Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
            Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
            Eigen::Vector3d position(transform.translation());
            Eigen::Quaterniond orientation(transform.rotation());
            Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(7, 7);
            Eigen::MatrixXd jacobian_pinv = jacobian.completeOrthogonalDecomposition().solve(Eigen::MatrixXd::Identity(6, 6));
            Eigen::MatrixXd nullspace_projector = identity - jacobian.transpose() * jacobian_pinv.transpose();

            // compute error to desired equilibrium pose
            // position error
            Eigen::Matrix<double, 6, 1> error;
            error.head(3) << position - position_d;

            // orientation error
            // "difference" quaternion
            if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
                orientation.coeffs() << -orientation.coeffs();
            }
            // "difference" quaternion
            Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
            error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
            // Transform to base frame
            error.tail(3) << -transform.rotation() * error.tail(3);

            // compute control
            Eigen::VectorXd tau_task(7), tau_d(7), tau_nullspace(7);

            // Spring damper system with damping ratio=1
            tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
            tau_nullspace = nullspace_projector * (null_stiffness * (q_d - q) - null_damping * dq);
            tau_d << tau_task + tau_nullspace + coriolis;

            std::array<double, 7> tau_d_array{};
            Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
            //print the control command
            std::cout << "Control command: " << tau_d.transpose() << std::endl;
            return tau_d_array;

        };

    robot.control(impedance_control_callback, false, 1000.0);

    } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

  return 0;
}