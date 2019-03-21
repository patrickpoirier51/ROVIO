# ROVIO
Visual Odometry System using MPU9250 IMU and Ueye Global Shutter Camera

https://files.gitter.im/ArduPilot/VisionProjects/eJ5O/image.png

Building and Testing a Monocular Visual Odometry System

Since the addition of VISION_POSITION_ESTIMATE to Ardupilot, a new field of experimentations offers the possibility to implement NON-GPS navigation for the autonomous flight control. I propose here a Monocular Visual Odometry System that can be builded using off the shelves components and open-source software developped by the ETH Zurich Autonomous Systems Lab. The software is running on a Intel Pentium based Companion Computer running UBUNTU 16.04 and ROS KINETIC. The VIO system is controlling a Linux Based Flight Controler -BBBMINI- installed on 450 Size Quadcopter.

Visual Inertial Odometry
Using a camera system and an Inertial Measurement Unit - IMU , we can estimate a 6 DoF (Degree of Freedom) state corresponding to 3D position (xyz) and 3 Axis rotation (roll-pitch-yaw), in relation to a fixed reference W (World/ Map / Home). 

https://www.researchgate.net/publication/319235394/figure/fig4/AS:614047967879181@1523411852971/Coordinate-system-definitions-6-There-are-5-frames-world-W-odometry-O-body-B-camera.png


This state estimation is then transmitted to the flight controler using the mavlink message https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE corresponding to the actual vehicle state in space:

x	float	m	Global X position
y	float	m	Global Y position
z	float	m	Global Z position
roll	float	rad	Roll angle
pitch	float	rad	Pitch angle
yaw	float	rad	Yaw angle



The system can be broken down in 3 major components:
A) The Visual Inertial sensor : A Global Shutter USB Camera with a MPU9250 IMU connected to an Arduino
B) The state estimator : ROVIO (Robust Visual Inertial Odometry) that runs on the Companion Computer
C) The Flight Controler : On this system I am using the BeagleBone Black with a DIY sensrs cape = The BBBMINI



 
The Visual Inertial sensor

https://files.gitter.im/patrickpoirier51/ZJsM/image.png

Inspired by this excellent blog, http://grauonline.de/wordpress/?page_id=1951, I decided to build my own sensor.
Searching on EBAY for global shutter cameras , I ordered two UEye-1220LE cameras at a fraction of the price and replaced the lens with a wide angle (Fisheye) M12. After some experimentation, I was able to make it run as a calibrated ROS camera. 

Camera and IMU synchronization
Next step is making the IMU sensor device, based on a MPU9250 connected to an Arduino Pro Mini using the I2C bus.
I made a simple "thru-hole" board, soldered the components and connectors, mounted this board on the back of camera using nylon posts and inserted the IMU module into connector so it is sitting on top of camera.
 
The IMU is free-running with its own internal DMP processor that generates a stable 200 Hz interrupted updates of the GYRO and ACC readings. The Arduino reads the IM, and based on a predefined number of interrupts, it will triggers the camera shutter(via the trigger line) to capture a new image. This way we have a continous 200 hz flow of IMU data with a 20 HZ image capture that is hardware synchronized with the IMU.  I did some modification on the Arduino code and on the ROS Node so it can be more stable and less prone to drift.


The state estimator : ROVIO
https://github.com/ethz-asl/rovio

Robust Visual Inertial Odometry (ROVIO) is a state  estimator  based  on  an  extended  Kalman  Filter(EKF),  which  proposed  several  novelties.  In  addition  to FAST  corner  features,  whose  3D  positions  are  parameterized  with  robotcentric  bearing  vectors  and  distances, multi-level  patches  are  extracted  from  the  image  stream around these features. The patch features are tracked, warped based  on  IMU-predicted  motion,  and  the  photometric  errors  are  used  in  the  update  step  as  innovation  terms.  ROVIO  was  developed  as  a  monocular  VIO pipeline,  is  available  as  an  open-source software package. 

This system requires a powerfull Companion Computer in order to process and output state estimate on realtime with good refresh rate and minimum delay. I installed a AAEON PICO-APL3 with a Pentium N4200 and 4 Gb RAM, loaded UBUNTU 16.04 and ROS KINETIC

---specs ---  https://www.aaeon.com/en/p/pico-itx-boards-pico-apl3


Building the code according to the insructions on github, using catkin tools, you must reduce the number of concurrent jobs an memory usage so it does not hang after exhausting all resources. Here is the command: catkin build rovio -j 1 --mem-limit 50% --cmake-args -DCMAKE_BUILD_TYPE=Release

Tuning computational costs
The following parameters can be adapted to reduce computation costs
rovio_node.cpp:
    Reduce feature count to 12
    Reduce the patch size to 4
    If no external pose measurements are used nPose should definetely be set to 0

rovio.info:
    Reduce the number of processed image levels by getting startLevel and endLevel closer to each other, e.g. 2 and 1.
    Set alignMaxUniSample to 0 such that only 1 sample is evaluated when searching patches
    Eventually disable patch warping if your application does not involve too fast motions


Calibration
https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration
Take a look at the video first as it explain pretty well how this calibration works.

This is quite tricky and requires a lot of experimentation, this is where most of the experimenters get stucked and it takes a lot of determination to make it successful. The most difficult step is estimating correctly the IMU parameters as there is no references for the MPU9250 so I had to extract the correct numbers from a graph generated by the Allan Vaiance test https://github.com/GAVLab/allan_variance.  

Once the calibration is complete, the values are exported in the rovio.info file in the Camera Calibration section. The extended  Kalman  Filter(EKF)  parameters have to be tweeked as well, particulay the values of the Prediction Noise. Since we use consumer grade IMU we must increase the values of vel, acb, gyb, and att., so during tests, we increase them separetely by factors of 10 until we have nice results, meaning a stable estimation that does not drift and diverge. 



Launching the estimator
The system start in sequence , The Camera, the IMU and finaly the Estimator
roslaunch ueye_cam cam0_ext_400.launch 
roslaunch mpu6050_serial_to_imu demo.launch
roslaunch rovio rovio_ueye.launch

During development I launche them as 3 separate launch files, making it easier to debug and optimise.
At final I merged them in a single launch file roslaunch rovio rovio_full.launch

MAVROS LAUNCH
Last we launch MAVROS with the apm profile and talking to the flight controler (BBBMINI) on Ethernet 
roslaunch mavros apm.launch fcu_url:=udp://0.0.0.0:14551@

Specific configuration
We must make sure that VISION is not blacklisted in the apm_pluginlists.yaml
And we disable the IMESYNC in apm_config.yaml
timesync_rate: 0.0    # TIMESYNC rate in Hertz (feature disabled if 0.0)
Depending on system configuration , we might have to augment mavsys rate
rosrun mavros mavsys rate --all 100

ARDUPILOT LAUNCH
This is how the ArduCopter is launched on the BBBMINI
sudo ./arducopter -C udp:192.168.8.34:14550 -D udp:10.0.0.2:14551

You set Serial1 (-C) and Serial2 (-D) as Mavlink2 (speed is irrelevant on IP)and the connexion to the GCS is on a WIFI AP mode set on 192.168.8.xx:14550 and connexion to the CC as a standard private network in the 10.0.0x range.



Testing
Bench Test

Walk Test

Initial Flight Test

Autonomous Flight

 



