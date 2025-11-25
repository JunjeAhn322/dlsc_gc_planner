//
// Created by jungwon on 20. 8. 31..
//

#include "linear_kalman_filter.hpp"

LinearKalmanFilter::LinearKalmanFilter() {
    F = FMatrix::Zero();
    G = GMatrix::Zero();
    Q = QMatrix::Zero();
//    R = RMatrix::Zero();
    H = HMatrix::Zero();

    x_old = NVector::Zero();
    x_predict = NVector::Zero();
    x_estimate = NVector::Zero();
    P_old = PMatrix::Zero();
    P_predict = PMatrix::Zero();
    P_estimate = PMatrix::Zero();

    sigma_Q = MVector::Zero();
    sigma_R = MVector::Zero();

    sigma_Q(0,0) = 20.0;
    sigma_Q(1,0) = 20.0;
    sigma_Q(2,0) = 20.0;
    sigma_R(0,0) = 0.001;
    sigma_R(1,0) = 0.001;
    sigma_R(2,0) = 0.001;

    H(0,0) = 1.0;
    H(1,1) = 1.0;
    H(2,2) = 1.0;
}

nav_msgs::Odometry LinearKalmanFilter::pose_cb(const geometry_msgs::PoseStamped& msg)
{
    if(initialized)
    {
        t_new = ros::WallTime::now();
        double dt = (t_new - t_old).toSec();
        //if(dt>0.02) std::cout << "     " << dt << std::endl;
        //else if(dt<0.005) std::cout << "          " << dt << std::endl;
        //else std::cout << dt << std::endl;
        predict(dt);
        update(dt, msg);
        ros::WallTime t_now_wall = ros::WallTime::now();
        ros::Time t_now;
        t_now.sec = t_now_wall.sec;
        t_now.nsec = t_now_wall.nsec;
        twist.header.stamp = t_now;
        twist.twist.linear.x = x_estimate(3,0);
        twist.twist.linear.y = x_estimate(4,0);
        twist.twist.linear.z = x_estimate(5,0);
        //twist_pub.publish(twist);
        odom.pose.pose = msg.pose;

        //convert from inertial velocity to body velocity
        double q0 = msg.pose.orientation.w;
        double q1 = msg.pose.orientation.x;
        double q2 = msg.pose.orientation.y;
        double q3 = msg.pose.orientation.z;
        double phi = atan2(2*(q0*q1 + q2*q3),1-2*(q1*q1+q2*q2));
        double theta = asin(2*(q0*q2-q3*q1));
        double psi = atan2(2*(q0*q3 + q1*q2),1-2*(q2*q2+q3*q3));

        Eigen::Matrix<double,3,3> R;
        R(0,0) = cos(psi)*cos(theta);
        R(0,1) = cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi);
        R(0,2) = sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta);
        R(1,0) = cos(theta)*sin(psi);
        R(1,1) = cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta);
        R(1,2) = cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi);
        R(2,0) = -sin(theta);
        R(2,1) = cos(theta)*sin(phi);
        R(2,2) = cos(phi)*cos(theta);
        Eigen::Vector3d twist_vector;
        twist_vector(0) = twist.twist.linear.x;
        twist_vector(1) = twist.twist.linear.y;
        twist_vector(2) = twist.twist.linear.z;

        //twist_vector = R.transpose()*twist_vector; //TODO: add param to control this

        odom.twist.twist.linear.x = twist_vector(0);
        odom.twist.twist.linear.y = twist_vector(1);
        odom.twist.twist.linear.z = twist_vector(2);
//        odom_pub.publish(odom);

        pose_new = msg;
        twist_raw.header.stamp = t_now;
        twist_raw.twist.linear.x = (pose_new.pose.position.x-pose_old.pose.position.x)/dt;
        twist_raw.twist.linear.y = (pose_new.pose.position.y-pose_old.pose.position.y)/dt;
        twist_raw.twist.linear.z = (pose_new.pose.position.z-pose_old.pose.position.z)/dt;
        //twist_pub_raw.publish(twist_raw);

        x_old = x_estimate;
        P_old = P_estimate;
        t_old = t_new;
        pose_old = pose_new;
    }
    else
    {
        t_old = ros::WallTime::now();
        x_old(0,0) = msg.pose.position.x;
        x_old(1,0) = msg.pose.position.y;
        x_old(2,0) = msg.pose.position.z;

        P_old(0,0) = 0.1;
        P_old(1,1) = 0.1;
        P_old(2,2) = 0.1;
        P_old(3,3) = 1.1;
        P_old(4,4) = 1.1;
        P_old(5,5) = 1.1;

        if(PUBLISH_RAW_TWIST) {
            pose_old = msg;
        }

        initialized = true;
    }

    return odom;
}

/**
 * ===================================================================================
 * [THEORY: Kalman Filter Predict Step]
 * ===================================================================================
 *
 * Propagates state and covariance forward in time using the motion model.
 *
 * EQUATIONS:
 *   x_predict = F * x_old           (State prediction)
 *   P_predict = F * P_old * F^T + Q (Covariance prediction)
 *
 * where:
 *   F = | I   dt*I |  State transition matrix for constant velocity model
 *       | 0     I  |
 *
 *   Q = G * sigma_a^2 * G^T   Process noise covariance
 *
 * The predict step "grows" the uncertainty (P_predict > P_old) because we are
 * less certain about the state as time passes without new measurements.
 * ===================================================================================
 */
void LinearKalmanFilter::predict(const double &dt)
{
    F = computeF(dt);
    G = computeG(dt);
    Q = computeQ(G, sigma_Q);

    x_predict = F*x_old;
    P_predict = F*P_old*F.transpose() + Q;
}

/**
 * ===================================================================================
 * [THEORY: Kalman Filter Update Step]
 * ===================================================================================
 *
 * Incorporates new position measurement to refine state estimate.
 *
 * EQUATIONS:
 *   residual = y - H * x_predict                           (Measurement residual/innovation)
 *   S = H * P_predict * H^T + R                            (Innovation covariance)
 *   K = P_predict * H^T * S^{-1}                           (Kalman gain)
 *   x_estimate = x_predict + K * residual                  (State update)
 *   P_estimate = P_predict - K * S * K^T                   (Covariance update)
 *
 * KEY INSIGHT:
 * - Kalman gain K determines how much to trust measurement vs prediction
 * - Large K (high P, low R): Trust measurement more
 * - Small K (low P, high R): Trust prediction more
 *
 * The velocity estimate (x_estimate[3:5]) is IMPLICITLY derived from the
 * position time series through the state transition model.
 * ===================================================================================
 */
void LinearKalmanFilter::update(const double &dt, const geometry_msgs::PoseStamped& msg)
{
    /** Extract position measurement y = [p_x, p_y, p_z]^T */
    MVector measure = MVector::Zero();
    measure(0,0) = msg.pose.position.x;
    measure(1,0) = msg.pose.position.y;
    measure(2,0) = msg.pose.position.z;

    /** Measurement residual: difference between actual and predicted measurement */
    MVector residual = measure - H*x_predict;

    /** Innovation covariance: uncertainty in the residual */
    RMatrix R = computeR();
    RMatrix innovation = R + H*P_predict*H.transpose();

    /** Kalman gain: optimal blending factor between prediction and measurement */
    GMatrix K = P_predict*H.transpose()*innovation.inverse();

    /** State update: refine estimate using measurement */
    x_estimate = x_predict + K*residual;
    /** Covariance update: measurement "shrinks" uncertainty */
    P_estimate = P_predict - K*innovation*K.transpose();
}

/**
 * Compute State Transition Matrix F for constant velocity model.
 *
 * F = | I   dt*I |  where I is 3x3 identity
 *     | 0     I  |
 *
 * This encodes: p_{k+1} = p_k + v_k * dt  (position update)
 *               v_{k+1} = v_k              (velocity constant)
 */
FMatrix LinearKalmanFilter::computeF(const double &dt)
{
    FMatrix temp = FMatrix::Zero();
    for(int i=0; i < Row; i++)
    {
        temp(i,i) = 1.0;
    }
    /** Off-diagonal blocks: dp/dt = v -> p_{k+1} = p_k + v_k * dt */
    temp(0,3) = dt;
    temp(1,4) = dt;
    temp(2,5) = dt;

    return temp;
}

/**
 * Compute Process Noise Input Matrix G.
 *
 * G = | 0.5*dt^2 * I |   Maps acceleration (process noise) to state
 *     |    dt * I    |
 *
 * Physical interpretation:
 * - Position noise: 0.5 * a * dt^2 (kinematics)
 * - Velocity noise: a * dt
 */
GMatrix LinearKalmanFilter::computeG(const double &dt)
{
    GMatrix temp = GMatrix::Zero();
    for(int i=0; i<3; i++)
    {
        temp(i,i) = 0.5*dt*dt;    /** Position affected by 0.5*a*dt^2 */
        temp(i+3,i) = dt;          /** Velocity affected by a*dt */
    }

    return temp;
}

QMatrix LinearKalmanFilter::computeQ(const GMatrix &G, const MVector &sigma_Q)
{
    RMatrix temp = RMatrix::Zero();
    for(int i=0; i < Col; i++)
    {
        temp(i,i) = sigma_Q(i,0)*sigma_Q(i,0);
    }

    return G*temp*G.transpose();
}

RMatrix LinearKalmanFilter::computeR()
{
    RMatrix temp = RMatrix::Zero();
    for(int i=0; i < Col; i++)
    {
        temp(i,i) = sigma_R(i,0);
    }

    return temp;
}
