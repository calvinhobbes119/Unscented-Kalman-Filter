#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  // This value was found to work well after some experimentation. Other
  // values which were considered were 10.0, 1.0 and 0.2.
  std_a_ = 0.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  // This value was found to work well after some experimentation.  Other
  // values which were considered were 10.0, 1.0 and 0.2.
  std_yawdd_ = 0.5;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  Complete the initialization. See ukf.h for other member properties.

  */
  // The state vector has 5 components (px, py, speed, yaw, yaw-rate).
  n_x_ = 5;
  // We augment the state-covariance matrix with the process noise for
  // longitudinal and yaw accleration for a more accurate model
  n_aug_ = 7;
  // Use recommended value for spreading parameter lambda
  lambda_ = 3 - n_x_;
  // Instantiate the matrix for storing the predicted sigma points
  Xsig_pred_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  // Instantiate the vector for storing the weights
  weights_ = VectorXd(2 * n_aug_ + 1);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
      Initialize the state vector based on the first measurement.
      A good initial guess for the covariance matrix is based on the measurement
      noise variances of the LiDAR sensor for px and py. Set the variances for
      the remaining state variables to 1.0.
    */
    int n;
    cout << "UKF: " << endl;
    x_ = VectorXd(5);
    x_ << 0, 0, 0, 0, 0; // px, py, v, psi, psi_dot
    P_ <<     0.0225, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0225, 0.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 1.0;
    //set weights as mentioned in the video lectures
    weights_[0] = lambda_/(lambda_ + n_aug_);
    for (n=1; n < 2 * n_aug_ + 1; n++)
    {
        weights_[n] = 1.0/(2.0*(lambda_ + n_aug_));
    }
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      Assume 0 for state variables which cannot be estimated from the measurement.
      */
      x_ << measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]), 
                 measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]), 
                 0, 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      Assume 0 for state variables which cannot be estimated from the measurement.
      */
      x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 
            0, 0, 0;
    }

    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;

    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
   */
    //compute the time elapsed between the current and previous measurements
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;
    Prediction(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

    UpdateRadar(measurement_pack);

  } else {

    UpdateLidar(measurement_pack);
  }  
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  // Variables for storing loop index
  int n, n1;

  // Create a matrix for storing the augmented
  // state covariance matrix
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  //Initialize augmented covariance matrix
  P_aug.setZero(n_aug_,n_aug_);
  // Update the top-left submatrix of P_aug with the state
  // covariance matrix
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  // Store the longitudinal and yaw acceleration noise variance
  // in the lower-right submatrix of P_aug.
  P_aug.bottomRightCorner(2,2) << std_a_*std_a_, 0.0, 0.0, std_yawdd_*std_yawdd_;

  //create augmented mean vector
  VectorXd x_aug = VectorXd(7);  
  //Store the state vector x_ at the beginning of x_aug.
  x_aug.head(5) = x_;
  x_aug.tail(2) << 0.0, 0.0;

  //Compute square root of P_aug
  MatrixXd A = P_aug.llt().matrixL();
  // Start creating augmented sigma points.
  // First entry is the augmented mean vector x_aug.
  Xsig_pred_.col(0) = x_aug;
  // Populate the rest of the sigma points as described
  // in the lectures.
  for (n = 1; n <= n_aug_; n++)
  {
     Xsig_pred_.col(n) = x_aug + sqrt(lambda_ + n_aug_) * A.col(n-1);
     Xsig_pred_.col(n_aug_ + n) = x_aug - sqrt(lambda_ + n_aug_) * A.col(n-1);
  }
   
  // Calculate the predicted sigma points by using the
  // CTRV motion model and accounting for the process noise
  for (n=0; n<2 * n_aug_ + 1; n++)
  {
      MatrixXd process_noise(n_x_,1);
      MatrixXd delta_x(5,1);
      process_noise << 0.5 * delta_t * delta_t * cos(Xsig_pred_(3,n)) * Xsig_pred_(5,n),
                        0.5 * delta_t * delta_t * sin(Xsig_pred_(3,n)) * Xsig_pred_(5,n),
                        delta_t * Xsig_pred_(5,n),
                        0.5 * delta_t * delta_t * Xsig_pred_(6,n),
                        delta_t * Xsig_pred_(6,n);
      // Add process noise to the prediction
      Xsig_pred_.block(0,n,n_x_,1) = Xsig_pred_.block(0,n,n_x_,1) + process_noise;

      if (Xsig_pred_(4,n) == 0.0)
      {
        // Use the zero yaw-rate equations to avoid dividing by zero.
        delta_x << Xsig_pred_(2,n) * cos(Xsig_pred_(3,n)) * delta_t,
                   Xsig_pred_(2,n) * sin(Xsig_pred_(3,n)) * delta_t,
                   0.0,
                   0.0,
                   0.0;
      }
      else
      {
        // Use the normal CTRV model for prediction
        delta_x << (Xsig_pred_(2,n)/Xsig_pred_(4,n)) * (sin(Xsig_pred_(3,n) + Xsig_pred_(4,n)*delta_t) - sin(Xsig_pred_(3,n))),
                   (Xsig_pred_(2,n)/Xsig_pred_(4,n)) * (-1.0 * cos(Xsig_pred_(3,n) + Xsig_pred_(4,n)*delta_t) + cos(Xsig_pred_(3,n))),
                   0.0,
                   Xsig_pred_(4,n)*delta_t,
                   0.0;
      }
      // Finally, update the matrix of predicted sigma points.
      Xsig_pred_.block(0,n,n_x_,1) += delta_x;

  }
 
  // Recover the predicted mean and covariance matrices from the 
  // the predicted sigma points using the weights calculated previously
  // as described in the lectures
  x_ = Xsig_pred_.block(0,0,n_x_,2*n_aug_+1)*weights_;
  //predict state covariance matrix  
  P_.setZero(n_x_,n_x_);
  for (n1 = 0; n1 <2 * n_aug_ + 1; n1++)
  {
      MatrixXd temp(n_x_,n_x_);
      temp = (Xsig_pred_.block(0,n1,n_x_,1) - x_) * (Xsig_pred_.block(0,n1,n_x_,1).transpose() - x_.transpose());
      P_ = P_ + weights_(n1) * temp;
  }

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  Update the state vector and covariance matrix based on LiDAR measurement.
  Since the measurement to state space mapping is linear we can use the EKF
  equations as before.
  */
 
    MatrixXd H = MatrixXd(2,5);
    MatrixXd R_laser = MatrixXd(2, 2);

    H << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0;
        //measurement covariance matrix - laser
      R_laser << std_laspx_*std_laspx_, 0,
        0, std_laspy_*std_laspy_;
  //create example vector for incoming Lidar measurement
  VectorXd z = VectorXd(2);
  z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];

    VectorXd z_pred = H * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H.transpose();
    MatrixXd S = H * P_ * Ht + R_laser;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H) * P_;
    
    // update NIS (Normalized Innovation Squared) metric to
    // detect inconsitencies in filter. Output the NIS metric
    // for offline analysis
    float nis =  (z.transpose() - z_pred.transpose()) * Si * (z - z_pred);
    std::cout << "NIS_lidar" << nis << std::endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

  //set measurement dimension. RADAR can measure (r, phi, r_dot)
  int n_z = 3;
  // variable for loop index
  int n;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  //create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], meas_package.raw_measurements_[2];
  
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);

  //transform sigma points into measurement space
  for (n=0; n<2*n_aug_+1;n++)
  {
    Zsig(0,n) = sqrt(Xsig_pred_(0,n)*Xsig_pred_(0,n) + Xsig_pred_(1,n)*Xsig_pred_(1,n));
    Zsig(1,n) = atan2(Xsig_pred_(1,n),Xsig_pred_(0,n));
    Zsig(2,n) = (Xsig_pred_(0,n) * cos(Xsig_pred_(3,n))*Xsig_pred_(2,n) +
                 Xsig_pred_(1,n) * sin(Xsig_pred_(3,n))*Xsig_pred_(2,n)) /
                 Zsig(0,n);
  }

  //calculate mean predicted measurement
  z_pred = Zsig * weights_  ;
  //calculate predicted measurement covariance matrix S
  S << std_radr_ * std_radr_, 0, 0,
       0, std_radphi_ * std_radphi_, 0,
       0, 0, std_radrd_ * std_radrd_;

  for (n=0; n<2*n_aug_+1;n++)
  {
      S = S + weights_(n) * (Zsig.col(n) - z_pred) * 
                           (Zsig.col(n).transpose() - z_pred.transpose());
  }

  // calculate cross correlation between sigma points in state space
  // and measurement space
  for (n=0;n<2*n_aug_+1;n++)
  {
      VectorXd x_diff = Xsig_pred_.block(0,n,n_x_,1) - x_;
      VectorXd z_diff = Zsig.col(n) - z_pred;
      while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
      while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
                                                                               
      while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
      while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
 
      Tc = Tc + weights_(n) * x_diff * z_diff.transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = MatrixXd::Zero(n_x_, n_z);
  K = Tc * S.inverse();
  //update state mean and covariance matrix
  VectorXd temp = VectorXd(n_z);
  temp = z - z_pred;

  while(temp(1)<-M_PI)
  {
      temp(1) += 2*M_PI;
  }
  while(temp(1)>M_PI)
  {
      temp(1) -= 2*M_PI;
  }
  x_ = x_ + K * temp;
  P_ = P_ - K * S * K.transpose();

  // update NIS (Normalized Innovation Squared) metric to
  // detect inconsitencies in filter. Output the NIS metric
  // for offline analysis  
  float nis =  (z.transpose() - z_pred.transpose()) * S.inverse() * (z - z_pred);
  std::cout << "NIS_radar" << nis << std::endl;
}
