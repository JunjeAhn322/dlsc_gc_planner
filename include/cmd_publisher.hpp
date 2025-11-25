#ifndef DLSC_GC_PLANNER_CMD_PUBLISHER_H
#define DLSC_GC_PLANNER_CMD_PUBLISHER_H

#include <ros/ros.h>
#include <param.hpp>
#include <mission.hpp>
#include <util.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <linear_kalman_filter.hpp>
#include <dlsc_gc_msgs/FullState.h>


namespace MATP {
    /**
     * ===================================================================================
     * [THEORY: Perception & State Estimation Interface - Park et al. 2023/2025]
     * ===================================================================================
     *
     * CmdPublisher serves as the PRIMARY PERCEPTION INTERFACE for the agent.
     * It bridges the gap between physical sensor data and the planning pipeline.
     *
     * KEY RESPONSIBILITIES:
     * 1. AGENT STATE ACQUISITION:
     *    - Receives agent's own pose via TF transforms (observed_agent_position)
     *    - This is the "ego-state" input to the planning algorithm
     *
     * 2. DYNAMIC OBSTACLE STATE ACQUISITION:
     *    - Receives real dynamic obstacle positions via TF transforms
     *    - Applies Linear Kalman Filter (LKF) to estimate obstacle velocity
     *    - Outputs filtered obstacle state (position + velocity)
     *
     * 3. TRAJECTORY EXECUTION:
     *    - Publishes desired state commands to the low-level controller
     *    - Monitors disturbance (deviation between desired and observed state)
     *
     * [QUADRUPED ADAPTATION NOTE]:
     * For a quadruped robot, this class is the injection point for:
     * - Custom EKF/UKF modules for more complex state estimation
     * - IMU + leg odometry fusion
     * - 2D/2.5D state representation (x, y, yaw) instead of full 3D
     * - Terrain-aware height estimation
     * ===================================================================================
     */
    class CmdPublisher {
    public:
        CmdPublisher(const ros::NodeHandle &nh, const Param &param, const Mission &mission, int agent_id);

        void updateTraj(const Trajectory<point3d> &new_traj, const ros::Time &traj_start_time);

        void landingCallback();

        [[nodiscard]] bool isDisturbed() const;

        [[nodiscard]] bool isAgentPoseUpdated() const;

        [[nodiscard]] bool isObsPoseUpdated(int obs_id) const;

        [[nodiscard]] bool landingFinished() const;

        [[nodiscard]] point3d getObservedAgentPosition() const;

        [[nodiscard]] State getObservedObstacleState(int obs_id) const;

    private:
        Param param;
        Mission mission;

        ros::NodeHandle nh;
        ros::Timer cmd_timer;
        ros::Publisher pub_cmd;
        ros::Publisher pub_cmd_stop;
        ros::Publisher pub_cmd_vis;
        /**
         * [PERCEPTION] TF Listener for external pose updates.
         * Receives transforms from motion capture / localization system.
         */
        tf::TransformListener tf_listener;

        int agent_id;
        size_t M, n;
        double dt, landing_time;

        /**
         * [AGENT STATE] Current observed position from external localization.
         * This is the "ground truth" position used when external pose is available.
         */
        point3d observed_agent_position;

        /**
         * [LKF FOR OBSTACLES] Array of Linear Kalman Filters, one per dynamic obstacle.
         * Each LKF estimates velocity from position-only measurements.
         * State vector: x = [p_x, p_y, p_z, v_x, v_y, v_z]^T
         */
        std::vector<LinearKalmanFilter> linear_kalman_filters;

        /**
         * [OBSTACLE STATE] Filtered obstacle odometry (position + velocity).
         * Key: obstacle ID, Value: filtered odometry message.
         */
        std::map<int, nav_msgs::Odometry> observed_obs_odoms;
        std::queue<Trajectory<point3d>> traj_queue;
        std::queue<ros::Time> traj_start_time_queue;
        Trajectory<point3d> current_traj;
        ros::Time current_traj_start_time, landing_start_time;
        bool initialized, external_agent_pose_update, external_obs_pose_update, is_disturbed, landing;
        double average_diff, diff_count, max_diff;

        void cmdTimerCallback(const ros::TimerEvent &event);

        void listenTF();

        void loadCurrentTraj();

        bool computeDesiredState(State &desired_state);

        void detectDisturbance(State &desired_state);

        void publishCommand(const State &desired_state);

        void publishLandingCommand(const State &desired_state);

        void failsafe();
    };
}

#endif //DLSC_GC_PLANNER_CMD_PUBLISHER_H
