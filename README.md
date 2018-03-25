## Project: Unscented-Kalman-Filter [![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Overview
---
This project implements the Unscented Kalman Filter (UKF) for Udacity Self Driving Nano Degree Term 2 Project. It also estimates the performance of the UKF for a test set under different conditions - when both LiDAR and RADAR data is available, when only LiDAR is available, and finally when only RADAR data is available.

Code changes
---
For this project I made the following changes to the started code.

1. __*tools.cpp*__ Implemented CalculateRMSE method in tools.cpp. Unlike the Extended Kalman Filter (EKF), we do not use Jacobians to linearize the measurement and motion models, so implementation of the Jacobian is not required for UKF. 
2. __*ukf.cpp*__ I initialized the process noise variance for longitudinal and yaw accleration to 0.5 after some experimentation. I also initialized other parameters needed for the UKF such as lambda, and instantiated the matrix to hold the predicted sigma points as well as a vector for holding the weights. Depending on whether the initial measurement is from RADAR or LiDAR I initialize the state vector appropriately. I also instantiate the state covariance matrix appropriately based on assumptions of the sensor accuracy for position.

For the prediction step, I update the time-delta between the previous and current measurements. This time-delta is used to update the process noise matrix used for estimating the predicted sigma points. The prediction of the state vector and covariance matrix is done as follows:

1. Create the augmented state vector and augmented covariance matrix by appending the state vector and state covariance matrix with the parameters from the process noise covariance matrix.
2. Compute the square root of the augmented covariance matrix.
3. Use the columns of the square-root matrix, the spreading parameter lambda and the state vector to derive the sigma points as mentioned in the video lectures.
4. Use the sigma points to compute the predicted sigma points using the CTRV motion model and the process noise. Take care not to divide by zero in cases when the yaw rate is 0.
5. Apply the weight vector to the predicted sigma points to compute the predicted mean and covariance matrix.

For the measurement update step there are 2 cases - one when the measurement sensor is LiDAR and the other when the measurement sensor is RADAR. For the LiDAR case since the transform from state space to measurement space is linear we can reuse the regular Kalman filter update equations from the EKF project. For the RADAR case, we do the following steps:

1. Transform the predicted sigma points to measurement space using cartesian to polar coordinate conversion.
2. Calculate the predicted measurement mean and covariance matrix *S* using the weight vector and the measurement sigma points computed in the previous step.  
3. Compute the cross-correlation matrix *Tc* between the sigma points in state space vs measurement space, taking care to wrap the angle field to lie within the range *-PI* to *PI*.
4. Calculate the Kalman gain *K* using *Tc* and *S*.
5. Finally, use the Kalman gain *K* to calculate the updated state vector *x* and covariance matrix *P*.

For both the RADAR and LiDAR measurements I also compute the Normalized Innovation Squared (NIS) metric as discussed in the video lectures.

Performance
---
The video below shows the performance of the Unscented Kalman Filter when both LiDAR and RADAR data is available. As shown in the video, the RMSE values for the state vector are below the thresholds stated in the Project rubric.

[![Both RADAR and LiDAR data available](https://github.com/calvinhobbes119/Extended-Kalman-Filter/blob/master/Untitled.png)](https://youtu.be/Ka9Zg-VmRME)

I also analyzed the consistency of the Unscented Kalman filter by analyzing the NIS score for both RADAR and LiDAR data. These scores for each measurement sample are plotted below.

![NIS Score - RADAR data](https://github.com/calvinhobbes119/Unscented-Kalman-Filter/blob/master/NIS_Score_RADAR.png)

![NIS Score - LiDAR data](https://github.com/calvinhobbes119/Unscented-Kalman-Filter/blob/master/NIS_Score_LiDAR.png)

The next two videos shows the performance of the UKF when only LiDAR or RADAR measurements are used. As expected the RMSE values are worse when only one sensor data is used. However, in each case the performance is better than for the equivalent case when using EKF because the CTRV motion model and sigma-points based estimatation more accurately captures the movement of the car in the simulator when compared with the linear motion model used in the EKF.

[![Only LiDAR data is available](https://github.com/calvinhobbes119/Extended-Kalman-Filter/blob/master/Untitled.png)](https://youtu.be/eecOsWagkyg)

[![Only RADAR data is available](https://github.com/calvinhobbes119/Extended-Kalman-Filter/blob/master/Untitled.png)](https://youtu.be/YNFqWGSada8)
