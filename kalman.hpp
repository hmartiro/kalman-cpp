/**
* Kalman filter implementation using Eigen. Based on the following
* introductory paper:
*
*     http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf
*
* @author: Hayk Martirosyan
* @date: 2014.11.15
*/

#include <Eigen/Dense>

#pragma once

class KalmanFilter {

public:

  /**
  * Create a Kalman filter with the specified matrices.
  *   A - System dynamics matrix
  *   C - Output matrix
  *   Q - Process noise covariance
  *   R - Measurement noise covariance
  *   P - Estimate error covariance
  */
  KalmanFilter(
      const Eigen::MatrixXd& A,
      const Eigen::MatrixXd& C,
      const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& R,
      const Eigen::MatrixXd& P
  );

  /**
  * Initialize the filter with initial states as zero.
  */
  void init();

  /**
  * Initialize the filter with a guess for initial states.
  */
  void init(const Eigen::VectorXd& x0);

  /**
  * Update the estimated state based on measured values.
  */
  void update(const Eigen::VectorXd& y);

  /**
  * Return the current estimated state.
  */
  Eigen::VectorXd state();


private:

  // System dimensions
  int n, m;

  // Matrices for computation
  Eigen::MatrixXd A, C, Q, R, P, K, I, P0;

  // Estimated states
  Eigen::VectorXd x_hat, x_hat_new;

  // Is the filter initialized?
  bool initialized;
};
