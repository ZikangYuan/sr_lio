## Related Works

1. [CT-ICP](https://github.com/jedeschaud/ct_icp): A state-of-the-art LiDAR odometry.
2. [VINs-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono): A state-of-the-art as bundle adjustment (BA) based visual-inertial odometry.
3. [Open_VINs](https://github.com/vell001/open_vins): A robust, real-time LiDAR-IMU initialization method without motion excitation.

# SR-LIO
**SR-LIO** (LiDAR-Inertial Odometry with Sweep Reconstruction) is a accurate and robust LiDAR-inertial odometry (LIO) package that can adjust the execution frequency beyond the sweep frequency. It segments and reconstructs raw input sweeps from spinning LiDAR to obtain reconstructed sweeps with higher frequency for shortening the time period of IMU pre-integration, so as to reduce the error of IMU preintegration. The main contributions of our package are as follow:
1. We propose a sweep reconstruction method, which can increase the frequency of spinning LiDAR sweeps and reduce the error of IMU pre-integration in LIO systems;
2. We embed the proposed sweep reconstruction method into our newly designed BA based LIO system and achieve the stateof-the-art accuracy;

## Demo Video (2022-10-17 Update)
<div align="left">
<img src="doc/running.gif" width=49.6% />
<img src="doc/result.gif" width = 49.6% >
</div>
**Watch the high-definition (HD) video on youtu**:
1. [Real-Time Performance](https://youtu.be/KYGFNe-8On4)
2. [Global Map and Trajectory](https://youtu.be/7XpBDc41uUA)
