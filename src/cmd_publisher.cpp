#include "cmd_publisher.hpp"

namespace MATP {
    CmdPublisher::CmdPublisher(const ros::NodeHandle &_nh, const Param &_param, const Mission &_mission, int _agent_id)
            : nh(_nh), param(_param), mission(_mission), agent_id(_agent_id) {
        nh.param<double>("landing_time", landing_time, 5.0);
        cmd_timer = nh.createTimer(ros::Duration(0.02), &CmdPublisher::cmdTimerCallback, this);
        pub_cmd = nh.advertise<dlsc_gc_msgs::FullState>(
                "/cf" + std::to_string(mission.agents[agent_id].cid) + "/cmd_full_state", 1);
        pub_cmd_stop = nh.advertise<std_msgs::Empty>(
                "/cf" + std::to_string(mission.agents[agent_id].cid) + "/cmd_stop", 1);
        pub_cmd_vis = nh.advertise<visualization_msgs::MarkerArray>("/cmd_full_state_vis", 1);

        M = param.M;
        n = param.n;
        dt = param.dt;
        initialized = false;
        external_agent_pose_update = false;
        external_obs_pose_update = false;
        is_disturbed = false;
        average_diff = 0;
        max_diff = 0;
        diff_count = 0;
        landing = false;
        linear_kalman_filters.resize(mission.on);
        observed_obs_odoms.clear();
    }

    void CmdPublisher::updateTraj(const Trajectory<point3d> &new_traj, const ros::Time &traj_start_time) {
        if (not initialized) {
            initialized = true;
        }

        traj_queue.push(new_traj);
        traj_start_time_queue.push(traj_start_time);
    }

    void CmdPublisher::landingCallback() {
        if (not landing) {
            landing = true;
            landing_start_time = ros::Time::now();
        }
    }

    bool CmdPublisher::isDisturbed() const {
        return is_disturbed;
    }

    bool CmdPublisher::isAgentPoseUpdated() const {
        return external_agent_pose_update;
    }

    bool CmdPublisher::isObsPoseUpdated(int obs_id) const {
        if(not external_obs_pose_update){
            return false;
        }
        else{
            auto search = observed_obs_odoms.find(obs_id);
            return search != observed_obs_odoms.end();
        }
    }

    bool CmdPublisher::landingFinished() const {
        return (ros::Time::now() - landing_start_time).toSec() > landing_time;
    }

    point3d CmdPublisher::getObservedAgentPosition() const {
        if (not external_agent_pose_update) {
            ROS_ERROR("[CmdPublisher] external pose is not updated");
        }
        return observed_agent_position;
    }

    State CmdPublisher::getObservedObstacleState(int obs_id) const {
        State state;
        if (not external_obs_pose_update) {
            ROS_WARN("[CmdPublisher] external obstacle pose is not updated, use origin instead");
            return state;
        }

        auto search = observed_obs_odoms.find(obs_id);
        if(search == observed_obs_odoms.end()){
            ROS_ERROR_STREAM("[CmdPublisher] Fail to find obstacle with id: " << obs_id);
            throw std::invalid_argument("[CmdPublisher] Fail to find obstacle");
        }

        state.position = pointMsgToPoint3d(search->second.pose.pose.position);
        state.velocity = vector3MsgToPoint3d(search->second.twist.twist.linear);
        return state;
    }

    void CmdPublisher::cmdTimerCallback(const ros::TimerEvent &event) {
        listenTF();
        loadCurrentTraj();

        State desired_state;
        bool success = computeDesiredState(desired_state);
        if (success) {
            detectDisturbance(desired_state);

            if (landing) {
                publishLandingCommand(desired_state);
            } else {
                publishCommand(desired_state);
            }
        } else if (initialized and external_agent_pose_update and current_traj.empty()) {
            failsafe();
        }
    }

    /**
     * ===================================================================================
     * [THEORY: Agent & Obstacle State Acquisition - Park et al. 2023/2025]
     * ===================================================================================
     *
     * This function is the PRIMARY SENSOR INTERFACE for the planning pipeline.
     * It acquires states for both the ego-agent and dynamic obstacles.
     *
     * PART 1 - AGENT STATE (Ego-localization):
     *   - Listens to TF transform: world_frame -> agent_frame
     *   - Updates observed_agent_position with current position
     *   - Fallback: Use ideal (predicted) state if TF unavailable
     *
     * PART 2 - OBSTACLE STATE (Dynamic Obstacle Tracking):
     *   - Listens to TF transform for each real dynamic obstacle
     *   - Raw position is passed through Linear Kalman Filter (LKF)
     *   - LKF estimates velocity from position-only measurements
     *   - Output: Full state (position + velocity) for trajectory prediction
     *
     * [QUADRUPED ADAPTATION NOTE]:
     * This is the injection point for custom state estimation:
     * - Replace TF listener with EKF/UKF fusing IMU + leg odometry
     * - For 2D navigation: extract (x, y, yaw) from full pose
     * - Consider terrain height map for z-coordinate estimation
     * ===================================================================================
     */
    void CmdPublisher::listenTF() {
        /**
         * -------------------------------------------------------------------------------
         * PART 1: AGENT STATE ACQUISITION (Ego-localization)
         * -------------------------------------------------------------------------------
         * Listen to TF and get agent's current position.
         * If external pose update is unavailable, the planner will use the ideal
         * (predicted) state from trajectory execution instead.
         */
        tf::StampedTransform transform;
        try {
            tf_listener.lookupTransform(param.world_frame_id, "/cf" + std::to_string(mission.agents[agent_id].cid),
                                        ros::Time(0), transform);
            observed_agent_position = vector3MsgToPoint3d(transform.getOrigin());
            external_agent_pose_update = true;
        }
        catch (tf::TransformException &ex) {
            ROS_WARN_ONCE("[MultiSyncSimulator] tf listener failed, use ideal state instead.");
            external_agent_pose_update = false;
        }

        /**
         * -------------------------------------------------------------------------------
         * PART 2: DYNAMIC OBSTACLE STATE ACQUISITION (Obstacle Tracking + LKF)
         * -------------------------------------------------------------------------------
         * For each real dynamic obstacle:
         * 1. Listen to TF transform for obstacle position (position-only measurement)
         * 2. Pass raw position through Linear Kalman Filter
         * 3. LKF estimates velocity using constant velocity motion model:
         *      x_k+1 = F * x_k + w_k,  where x = [p, v]^T
         * 4. Store filtered odometry (position + estimated velocity)
         *
         * The estimated velocity is CRITICAL for trajectory prediction phase:
         *   p_future = p_current + v_estimated * t
         */
        external_obs_pose_update = true;
        for(int oi = 0; oi < mission.on; oi++) {
            if (mission.obstacles[oi]->getType() == ObstacleType::DYN_REAL) {

                geometry_msgs::PoseStamped obs_pose;
                obs_pose.header.stamp = ros::Time::now();
                try {
                    tf_listener.lookupTransform(param.world_frame_id, "/obs" + std::to_string(oi),
                                                ros::Time(0), transform);
                    point3d obs_position = vector3MsgToPoint3d(transform.getOrigin());
                    obs_pose.pose.position = point3DToPointMsg(obs_position);
                    obs_pose.pose.orientation = defaultQuaternion();
                }
                catch (tf::TransformException &ex) {
                    ROS_WARN_STREAM_ONCE("[MultiSyncSimulator] tf listener failed to listen obs" << oi
                                                                                                 << ", use origin instead.");
                    obs_pose.pose.position = defaultPoint();
                    obs_pose.pose.orientation = defaultQuaternion();
                    external_obs_pose_update = false;
                }

                /**
                 * [LKF VELOCITY ESTIMATION]
                 * Linear Kalman Filter processes position measurement and outputs
                 * full odometry (position + estimated velocity).
                 * This is essential for constant-velocity trajectory prediction.
                 */
                nav_msgs::Odometry obs_odom = linear_kalman_filters[oi].pose_cb(obs_pose);
                observed_obs_odoms.insert_or_assign(oi, obs_odom);
            }
        }
    }

    bool CmdPublisher::computeDesiredState(State &desired_state) {
        // Check current trajectory is received
        if (current_traj.empty()) {
//            ROS_WARN("[CmdPublisher] There is no current trajectory, start planning first!");
            return false;
        }

        // time
        double t;
        t = (ros::Time::now() - current_traj_start_time).toSec();
        if (t < 0) {
            return false;
        }

        if (t > M * dt) {
            desired_state = current_traj.getStateAt(M * dt);
            desired_state.velocity = point3d(0, 0, 0);
            desired_state.acceleration = point3d(0, 0, 0);
        } else {
            desired_state = current_traj.getStateAt(t);
        }

        return true;
    }

    void CmdPublisher::detectDisturbance(State &desired_state) {
        if (not external_agent_pose_update or landing) {
            is_disturbed = false;
            return;
        }

        // if diff between ideal and current position is under threshold, use ideal state
        // else use current_state
        point3d desired_current_position = desired_state.position;
        double diff = observed_agent_position.distance(desired_current_position);
        if ((not is_disturbed and diff > param.reset_threshold) or (is_disturbed and diff > 0.05)) {
            is_disturbed = true;
        } else {
            is_disturbed = false;
        }

        if (is_disturbed) {
            current_traj.clear();
            std::queue<traj_t> empty_traj_queue;
            std::queue<ros::Time> empty_time_queue;
            std::swap(traj_queue, empty_traj_queue);
            std::swap(traj_start_time_queue, empty_time_queue);

            desired_state.position = observed_agent_position;
            desired_state.velocity = point3d(0, 0, 0);
            desired_state.acceleration = point3d(0, 0, 0);
        }

        diff_count++;
        average_diff = (1.0 - (1.0 / diff_count)) * average_diff + (1.0 / diff_count) * diff;
        if(diff_count > 300 and  diff > max_diff){
            max_diff = diff;
        }
//        if(int(diff_count) % 10 == 0){
//            ROS_INFO_STREAM("average_diff: " << average_diff << ", max_diff: " << max_diff);
//        }
    }


    void CmdPublisher::loadCurrentTraj() {
        if (traj_queue.empty()) {
            return;
        }

        if (current_traj.empty() or ros::Time::now() > traj_start_time_queue.front()) {
            current_traj = traj_queue.front();
            current_traj_start_time = traj_start_time_queue.front();

            traj_queue.pop();
            traj_start_time_queue.pop();
        }
    }

    void CmdPublisher::publishCommand(const State &desired_state) {
        // command for crazyflie
        dlsc_gc_msgs::FullState msg_cmd;
        msg_cmd.pose.position = point3DToPointMsg(desired_state.position);
        msg_cmd.pose.orientation = defaultQuaternion();
        msg_cmd.twist = point3DToTwistMsg(desired_state.velocity);
        msg_cmd.acc = point3DToVector3Msg(desired_state.acceleration);
        pub_cmd.publish(msg_cmd);

        // visualization
        visualization_msgs::MarkerArray msg_cmd_vis;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.ns = std::to_string(agent_id);
        marker.id = agent_id;
        marker.color = mission.color[agent_id];
        marker.color.a = 1.0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.05;
        marker.scale.y = 0.1;
        marker.pose.position = defaultPoint();
        marker.pose.orientation = defaultQuaternion();

        geometry_msgs::Point start_point, end_point;
        start_point = point3DToPointMsg(desired_state.position);
        end_point = point3DToPointMsg(desired_state.position + desired_state.velocity * 1.0);
        marker.points.emplace_back(start_point);
        marker.points.emplace_back(end_point);
        msg_cmd_vis.markers.emplace_back(marker);
        pub_cmd_vis.publish(msg_cmd_vis);
    }

    void CmdPublisher::publishLandingCommand(const State &new_state) {
        // Don't consider the disturbance while landing
        double t_land;
        t_land = (ros::Time::now() - landing_start_time).toSec();

        if (t_land > landing_time) {
            std_msgs::Empty msg_stop;
            pub_cmd_stop.publish(msg_stop);
        } else {
            State desired_state = new_state;
            desired_state.position.z() =
                    0.03 + (desired_state.position.z() - 0.03) * std::max(1 - t_land / landing_time, 0.0);
            desired_state.velocity = point3d(0, 0, 0);
            desired_state.acceleration = point3d(0, 0, 0);
            publishCommand(desired_state);
        }
    }

    void CmdPublisher::failsafe() {
        State desired_state;
        desired_state.position = observed_agent_position;
        desired_state.velocity = point3d(0, 0, 0);
        desired_state.acceleration = point3d(0, 0, 0);
        publishCommand(desired_state);
    }
}