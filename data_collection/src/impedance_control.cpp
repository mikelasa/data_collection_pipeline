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
        const double K_trans{200};
        const double K_rot{20};

        const double k_nullspace_trans{10};
        const double k_nullspace_rot{1};

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
            {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0}}, {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0}},
            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
            {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0}}, {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0}},
            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
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
        Eigen::Vector3d initial_position(initial_transform.translation());
        Eigen::Vector3d target_position(0.430179, 0.0, 0.520758); // desired target
        double trajectory_duration = 5.0; // seconds
        Eigen::Vector3d position_d = initial_position;
        Eigen::Quaterniond orientation_d(initial_transform.rotation());

        // Quintic trajectory generator
        auto quintic_trajectory = [](const Eigen::Vector3d& p0, const Eigen::Vector3d& pf, double t, double T) {
            double tau = std::min(std::max(t / T, 0.0), 1.0);
            double tau2 = tau * tau;
            double tau3 = tau2 * tau;
            double tau4 = tau3 * tau;
            double tau5 = tau4 * tau;
            double s = 10 * tau3 - 15 * tau4 + 6 * tau5;
            return p0 + s * (pf - p0);
        };

        // Precompute trajectory points
        int traj_steps = static_cast<int>(trajectory_duration * 1000.0); // 1kHz
        std::vector<Eigen::Vector3d> trajectory(traj_steps);
        for (int i = 0; i < traj_steps; ++i) {
            double t = static_cast<double>(i) / 1000.0;
            trajectory[i] = quintic_trajectory(initial_position, target_position, t, trajectory_duration);
        }

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
                                        &trajectory, &traj_steps, &position_d, &orientation_d](const franka::RobotState& robot_state,
                                                            franka::Duration duration) -> franka::Torques {

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

            // Step through precomputed trajectory
            static int traj_idx = 0;
            if (traj_idx < traj_steps) {
                position_d = trajectory[traj_idx];
                ++traj_idx;
            }
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
            //print robot state
            std::cout << "robot_state.O_T_EE: " << position.transpose() << " " << orientation.coeffs().transpose() << std::endl;

            return tau_d_array;

        };

    robot.control(impedance_control_callback, false, 1000.0);

    } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

  return 0;
}