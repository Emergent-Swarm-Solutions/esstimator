esstimator (SIQ)
================

esstimator is a fork of robot_localization that provides nonlinear state estimation nodes. The original package was developed by Charles River Analytics, Inc.

Please see the upstream robot_localization documentation here: http://wiki.ros.org/robot_localization

For the new EKF/UKF tuning window and practical tuning workflow, see [TUNING_VISUALIZER_GUIDE.md](TUNING_VISUALIZER_GUIDE.md).

The tuning visualizer treats each fused input branch as its own stream. For example, an IMU can appear as separate orientation, angular velocity, and linear acceleration inputs, each with its own covariance, innovation, and Mahalanobis behavior. The UI also shows the corresponding ROS topic name for each configured stream and keeps inactive configured streams visible as gray `[no data]` panels.
