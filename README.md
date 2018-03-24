
## Project: Unscented-Kalman-Filter [![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Overview
---
This project implements the Unscented Kalman Filter (UKF) for Udacity Self Driving Nano Degree Term 2 Project. It also estimates the performance of the UKF for a test set under different conditions - when both LiDAR and RADAR data is available, when only LiDAR is available, and finally when only RADAR data is available.

Code changes
---
For this project I made the following changes to the started code.

1. __*tools.cpp*__ Implemented CalculateRMSE method in tools.cpp. Unlike the Extended Kalman Filter (EKF), we do not use Jacobians to linearize the measurement and motion models, so implementation of the Jacobian is not required for UKF. 
2. __*ukf.cpp*__ TBD.

Performance
---
The video below shows the performance of the Unscented Kalman Filter when both LiDAR and RADAR data is available. As shown in the video, the RMSE values for the state vector are below the thresholds stated in the Project rubric.

[![Both RADAR and LiDAR data available](https://github.com/calvinhobbes119/Extended-Kalman-Filter/blob/master/Untitled.png)](https://youtu.be/Ka9Zg-VmRME)

I also analyzed the consistency of the Unscented Kalman filter by analyzing the NIS score for both RADAR and LiDAR data. These scores for each measurement sample are plotted below.

![NIS Score - RADAR data](https://github.com/calvinhobbes119/Unscented-Kalman-Filter/blob/master/NIS_Score_RADAR.png)

![NIS Score - LiDAR data](https://github.com/calvinhobbes119/Unscented-Kalman-Filter/blob/master/NIS_Score_LiDAR.png)

The next two videos shows the performance of the UKF when only LiDAR or RADAR measurements are used. As expected the RMSE values are worse when only one sensor data is used. However, in each case the performance is better than for the equivalent case when using EKF because the CTRV motion model more accurately captures the movement of the car in the simulator when compared with the linear motion model used in the EKF.

[![Only LiDAR data is available](https://github.com/calvinhobbes119/Extended-Kalman-Filter/blob/master/Untitled.png)](https://youtu.be/eecOsWagkyg)

[![Only RADAR data is available](https://github.com/calvinhobbes119/Extended-Kalman-Filter/blob/master/Untitled.png)](https://youtu.be/YNFqWGSada8)
