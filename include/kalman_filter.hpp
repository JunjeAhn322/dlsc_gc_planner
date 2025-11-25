#pragma once
#include <Eigen/Dense>
#include <sp_const.hpp>

namespace MATP {
    /**
     * ===================================================================================
     * [THEORY: LKF for Obstacle Tracking with Uncertainty Propagation - Park et al. 2023]
     * ===================================================================================
     *
     * This is an ALTERNATIVE LKF implementation specifically designed for:
     * 1. Filtering obstacle position/velocity from noisy measurements
     * 2. Computing UNCERTAINTY RADIUS for trajectory prediction
     *
     * KEY DIFFERENCE FROM linear_kalman_filter.hpp:
     * - Provides getUncertaintyRadius() for RSFC (Relative Safe Flight Corridor)
     * - Uncertainty radius is used to INFLATE obstacle collision model
     *
     * UNCERTAINTY PROPAGATION:
     * As we predict obstacle position into the future, uncertainty grows:
     *   P_future = F * P_current * F^T + Q
     *
     * The uncertainty radius r_uncertainty is derived from position covariance:
     *   r_uncertainty = k * sqrt(2 * trace(P_position))
     *
     * where k = 1.33 corresponds to ~90% confidence bound for 2D Gaussian
     *
     * [QUADRUPED ADAPTATION NOTE]:
     * For 2D navigation, the 3D uncertainty ellipsoid can be projected to a 2D
     * circle using the maximum eigenvalue of the x-y covariance submatrix.
     * ===================================================================================
     */
    class LinearKalmanFilter{
    public:
        /**
         * Initialize LKF with noise parameters.
         * @param _sigma_y_sq Position measurement noise variance
         * @param _sigma_v_sq Velocity process noise variance (unused in current implementation)
         * @param _sigma_a_sq Acceleration process noise variance (for Q computation)
         */
        void initialize(double _sigma_y_sq, double _sigma_v_sq, double _sigma_a_sq){
            n = 6; // the number of states
            m = 3; // the number of observed states
            phi = 2; // position, velocity
            sigma_y_sq = _sigma_y_sq;
            sigma_v_sq = _sigma_v_sq;
            sigma_a_sq = _sigma_a_sq;

            F = Eigen::MatrixXd::Identity(n, n);
            B = Eigen::MatrixXd::Zero(n, n/phi);

            Q = Eigen::MatrixXd::Zero(n, n);
//            Q.block(0, 0, n/phi, n/phi) = Eigen::MatrixXd::Identity(n/phi, n/phi) * sigma_y_sq;
//            Q.block(n/phi, n/phi, n/phi, n/phi) = Eigen::MatrixXd::Identity(n/phi, n/phi) * sigma_v_sq;

            H = Eigen::MatrixXd::Zero(m, n);
            H.block(0, 0, m, m) = Eigen::MatrixXd::Identity(m, m);
            R = Eigen::MatrixXd::Identity(m, m) * sigma_y_sq;

            P = Eigen::MatrixXd::Zero(n, n);
            P.block(0, 0, n/phi, n/phi) = Eigen::MatrixXd::Identity(n/phi, n/phi) * 10;
            P.block(n/phi, n/phi, n/phi, n/phi) = Eigen::MatrixXd::Identity(n/phi, n/phi) * 10;

            Y = Eigen::VectorXd(m);
            X_hat = Eigen::VectorXd(n);

            isFirstInput = true;
        }

        /**
         * ===================================================================================
         * [THEORY: Kalman Filter for Obstacle State Estimation]
         * ===================================================================================
         *
         * Standard LKF predict-update cycle for obstacle position/velocity estimation.
         *
         * STATE VECTOR: X = [p_x, p_y, p_z, v_x, v_y, v_z]^T
         *
         * MOTION MODEL (Constant Velocity):
         *   X_{k+1} = F * X_k + B * w_k
         *
         * MEASUREMENT MODEL:
         *   Y_k = H * X_k + v_k  (position only)
         *
         * OUTPUT:
         *   Filtered obstacle with estimated position and velocity.
         *   The covariance P is retained for uncertainty radius computation.
         * ===================================================================================
         */
        Obstacle filter(const Obstacle& obstacle){
            /** MEASUREMENT: Extract observed position */
            Y << obstacle.position.x(), obstacle.position.y(), obstacle.position.z();
            ros::Time current_update_time = obstacle.update_time;

            if(isFirstInput){
                /** INITIALIZATION: Set initial state from first measurement */
                X_hat << Y, 0, 0, 0;  // Position from measurement, velocity = 0
                isFirstInput = false;
            }
            else{
                /**
                 * PREDICT STEP:
                 * Update state transition matrix F with current dt
                 * F encodes constant velocity model: p_{k+1} = p_k + v_k * dt
                 */
                dt = (current_update_time - prev_update_time).toSec();
                F.block(0, n/phi, n/phi, n/phi) = Eigen::MatrixXd::Identity(n/phi, n/phi) * dt;
                B.block(n/phi, 0, n/phi, n/phi) = Eigen::MatrixXd::Identity(n/phi, n/phi) * dt;
                Q = sigma_a_sq * B * B.transpose();  // Process noise from acceleration

                /** State prediction: propagate state forward */
                X_hat_update = F * X_hat;
                /** Covariance prediction: uncertainty grows */
                Eigen::MatrixXd P_update = F * P * F.transpose() + Q;

                /**
                 * UPDATE STEP:
                 * Incorporate measurement to refine estimate
                 */
                /** Innovation covariance */
                Eigen::MatrixXd S = H * P_update * H.transpose() + R;
                /** Kalman gain: balance prediction vs measurement */
                Eigen::MatrixXd K = P_update * H.transpose() * S.inverse();

                /** State update: X = X_pred + K * (Y - H * X_pred) */
                X_hat = X_hat_update + K * (Y - H * X_hat_update);
                /** Covariance update: measurement "shrinks" uncertainty */
                P = P_update - K * H * P_update;
            }

            /** Extract filtered state for output */
            Obstacle result = obstacle;
            result.position = point3d(X_hat(0, 0), X_hat(1, 0), X_hat(2, 0));
            result.velocity = point3d(X_hat(3, 0), X_hat(4, 0), X_hat(5, 0));

            prev_update_time = current_update_time;
            return result;
        }

        Eigen::MatrixXd getPositionCovariance(){
            return P.block(0, 0, n/phi, n/phi);
        }

        /**
         * ===================================================================================
         * [THEORY: Uncertainty Radius for RSFC - Park et al. 2023, Section 4.2]
         * ===================================================================================
         *
         * Computes the uncertainty radius for a future time t_delta.
         * This is used to INFLATE the obstacle collision model in RSFC constraints.
         *
         * ALGORITHM:
         * 1. Propagate covariance P forward by t_delta using motion model
         * 2. Extract position covariance submatrix SIGMA (3x3)
         * 3. Compute uncertainty radius from trace of SIGMA
         *
         * FORMULA:
         *   P_future = F(t_delta) * P_current * F(t_delta)^T + Q(t_delta)
         *   SIGMA = P_future[0:3, 0:3]  (position covariance)
         *   r_uncertainty = 1.33 * sqrt(2 * trace(SIGMA))
         *
         * INTERPRETATION:
         * - trace(SIGMA) = sigma_x^2 + sigma_y^2 + sigma_z^2 (sum of position variances)
         * - sqrt(2 * trace) approximates the "average radius" of uncertainty ellipsoid
         * - Factor 1.33 corresponds to ~90% confidence bound (chi-squared distribution)
         *
         * USAGE:
         * This radius is ADDED to the obstacle physical radius for collision checking:
         *   r_total(t) = r_physical + r_uncertainty(t)
         *
         * Combined with the RSFC inflation formula:
         *   r(t) = r_0 + 0.5 * a_max * t^2 + r_uncertainty(t)
         *
         * [QUADRUPED ADAPTATION NOTE]:
         * For 2D navigation, use only x-y covariance:
         *   SIGMA_2D = P_future[0:2, 0:2]
         *   r_uncertainty_2D = 1.33 * sqrt(2 * trace(SIGMA_2D))
         * ===================================================================================
         */
        double getUncertaintyRadius(double t_delta){
            /** Update F and B for the prediction horizon t_delta */
            F.block(0, n/phi, n/phi, n/phi) = Eigen::MatrixXd::Identity(n/phi, n/phi) * t_delta;
            B.block(n/phi, 0, n/phi, n/phi) = Eigen::MatrixXd::Identity(n/phi, n/phi) * t_delta;
            Q = sigma_a_sq * B * B.transpose();

            /** Propagate covariance forward in time */
            Eigen::MatrixXd P_update = F * P * F.transpose() + Q;

            /** Extract position covariance submatrix (top-left 3x3 block) */
            Eigen::MatrixXd SIGMA = P_update.block(0, 0, n/phi, n/phi);

            /**
             * Compute uncertainty radius:
             * - sqrt(2 * trace) gives approximate "average" radius of 3D uncertainty ellipsoid
             * - Factor 1.33 provides ~90% confidence coverage
             */
            double uncertainty_radius = 1.33 * sqrt(2.0 * (double)SIGMA.trace());
            return uncertainty_radius;
        }

    private:
        int n, m, phi;
        double sigma_y_sq, sigma_v_sq, sigma_a_sq, dt;
        Eigen::MatrixXd F, B, Q, H, R, P;
        Eigen::VectorXd Y, X_hat, X_hat_update;

        bool isFirstInput;
        ros::Time prev_update_time;
    };
}