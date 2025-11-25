//
// Created by jungwon on 22. 2. 8..
//

#ifndef DYNAMIC_PLANNER_LINEARKALMANFILTER_H
#define DYNAMIC_PLANNER_LINEARKALMANFILTER_H

/**
 * ===================================================================================
 * [THEORY: Linear Kalman Filter for Velocity Estimation - Park et al. 2023/2025]
 * ===================================================================================
 *
 * This LKF estimates velocity from position-only measurements for dynamic obstacles.
 * It is used in the PERCEPTION phase to provide velocity estimates for trajectory
 * prediction.
 *
 * STATE SPACE MODEL (Constant Velocity Motion Model):
 * -------------------------------------------------
 * State vector:  x = [p_x, p_y, p_z, v_x, v_y, v_z]^T  (6 states)
 *
 * State transition:
 *   x_{k+1} = F * x_k + G * w_k
 *
 * where F = | I   dt*I |    (State transition matrix)
 *           | 0     I  |
 *
 *       G = | 0.5*dt^2*I |  (Process noise input matrix)
 *           |    dt*I    |
 *
 * Measurement model:
 *   y_k = H * x_k + v_k
 *
 * where H = [I  0]  (Observation matrix - position only)
 *
 * KALMAN FILTER EQUATIONS:
 * -----------------------
 * Predict:
 *   x_predict = F * x_old
 *   P_predict = F * P_old * F^T + Q
 *
 * Update:
 *   K = P_predict * H^T * (H * P_predict * H^T + R)^{-1}  (Kalman gain)
 *   x_estimate = x_predict + K * (y - H * x_predict)
 *   P_estimate = P_predict - K * H * P_predict
 *
 * OUTPUT:
 * - Filtered position (smoother than raw measurement)
 * - Estimated velocity (derived from position time series)
 *
 * [QUADRUPED ADAPTATION NOTE]:
 * For a quadruped with better sensing, consider:
 * - EKF/UKF for non-linear motion models
 * - State augmentation with acceleration for more accurate prediction
 * - Sensor fusion with radar/lidar for direct velocity measurement
 * ===================================================================================
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <string>

#define PUBLISH_RAW_TWIST false

/** State dimension: [p_x, p_y, p_z, v_x, v_y, v_z] = 6 states */
#define Row 6
/** Measurement dimension: [p_x, p_y, p_z] = 3 measurements */
#define Col 3

using namespace Eigen;

/** F: State transition matrix (6x6) - relates x_k to x_{k+1} */
typedef Matrix<double, Row, Row> FMatrix;
/** G: Process noise input matrix (6x3) - maps acceleration noise to state */
typedef Matrix<double, Row, Col> GMatrix;
/** Q: Process noise covariance (6x6) - uncertainty from motion model */
typedef Matrix<double, Row, Row> QMatrix;
/** P: State covariance (6x6) - uncertainty of state estimate */
typedef Matrix<double, Row, Row> PMatrix;
/** R: Measurement noise covariance (3x3) - sensor uncertainty */
typedef Matrix<double, Col, Col> RMatrix;
/** H: Observation matrix (3x6) - maps state to measurement */
typedef Matrix<double, Col, Row> HMatrix;
/** State vector (6x1) */
typedef Matrix<double, Row, 1> NVector;
/** Measurement vector (3x1) */
typedef Matrix<double, Col, 1> MVector;

/**
 * Linear Kalman Filter for obstacle velocity estimation.
 * Processes position-only measurements to estimate full state (position + velocity).
 */
class LinearKalmanFilter {
public:
    LinearKalmanFilter();

    /**
     * Main callback: processes position measurement and returns filtered odometry.
     * @param msg Position measurement from TF listener
     * @return Filtered odometry with estimated velocity
     */
    nav_msgs::Odometry pose_cb(const geometry_msgs::PoseStamped& msg);

private:
    /** State transition matrix: x_{k+1} = F * x_k */
    FMatrix F;
    /** Process noise input matrix: maps acceleration to state change */
    GMatrix G;
    /** Process noise covariance: Q = G * sigma_a^2 * G^T */
    QMatrix Q;
    /** Observation matrix: y = H * x (extracts position from state) */
    HMatrix H;

    /** Previous state estimate */
    NVector x_old;
    /** Predicted state (before measurement update) */
    NVector x_predict;
    /** Updated state estimate (after measurement incorporation) */
    NVector x_estimate;

    /** Previous covariance estimate */
    PMatrix P_old;
    /** Predicted covariance (before measurement update) */
    PMatrix P_predict;
    /** Updated covariance estimate */
    PMatrix P_estimate;

    /** Process noise standard deviations (acceleration uncertainty) */
    MVector sigma_Q;
    /** Measurement noise standard deviations (position sensor noise) */
    MVector sigma_R;

    geometry_msgs::PoseStamped pose_old, pose_new;
    geometry_msgs::TwistStamped twist, twist_raw;
    nav_msgs::Odometry odom;

    ros::WallTime t_old, t_new;
    bool initialized = false;

    /** Kalman predict step: propagate state and covariance forward */
    void predict(const double &dt);
    /** Compute state transition matrix F for given time step */
    static FMatrix computeF(const double &dt);
    /** Compute process noise input matrix G for given time step */
    static GMatrix computeG(const double &dt);
    /** Compute process noise covariance Q from G and sigma */
    static QMatrix computeQ(const GMatrix &G, const MVector &sigma_Q);
    /** Kalman update step: incorporate measurement to refine estimate */
    void update(const double &dt, const geometry_msgs::PoseStamped& msg);
    /** Compute measurement noise covariance R */
    RMatrix computeR();
};

#endif //DYNAMIC_PLANNER_LINEARKALMANFILTER_H
