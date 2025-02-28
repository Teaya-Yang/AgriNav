/* A simulator with manually incremented ROS clock.
 *
 */

#include <memory>
#include <mutex>
#include <Eigen/Dense>

#include <ros/ros.h>

#include "Common/Time/ManualTimer.hpp"
#include "Common/Time/HardwareTimer.hpp"
#include "Components/Simulation/Quadcopter_T.hpp"
#include "Components/Simulation/UWBNetwork.hpp"
#include "Components/Simulation/CommunicationsDelay.hpp"

#include <fstream>

#include "hiperlab_rostools/mocap_output.h"
#include "hiperlab_rostools/simulator_truth.h"
#include "hiperlab_rostools/radio_command.h"
#include "hiperlab_rostools/telemetry.h"
#include <std_msgs/Bool.h> 
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>
#include <rosgraph_msgs/Clock.h>


using namespace std;

mutex cmdRadioChannelMutex; // protect against concurrency problems

class SimVehicle
{
public:
  struct
  {
    shared_ptr<
        Simulation::CommunicationsDelay<
            RadioTypes::RadioMessageDecoded::RawMessage>>
        queue;
  } cmdRadioChannel;

  shared_ptr<ros::Subscriber> subRadioCmd;
  shared_ptr<ros::Subscriber> subImageLeft;
  shared_ptr<ros::Subscriber> subImageRight;
  shared_ptr<ros::Subscriber> subImageDepth;
  shared_ptr<ros::Publisher> pubSimTruth;
  shared_ptr<ros::Publisher> pubMoCap;
  shared_ptr<ros::Publisher> pubTelemetry;
  shared_ptr<ros::Publisher> pubOdom;
  shared_ptr<ros::Publisher> pubHeartbeat;
  shared_ptr<ros::Publisher> pubCamIMU;
  shared_ptr<ros::Publisher> pubSimTime;

  rosgraph_msgs::Clock simTimeMsg; // Simulation clock

  bool imageReceivedLeft = false;
  bool imageReceivedRight = false;
  bool imageReceivedDepth = false;

  int id;

  shared_ptr<Simulation::SimulationObject6DOF> vehicle;

  void callbackRadioCmd(const hiperlab_rostools::radio_command &msg)
  {
    std::lock_guard<std::mutex> guard(cmdRadioChannelMutex);
    // todo: should be a nicer way to do this, using e.g. memcpy...
    RadioTypes::RadioMessageDecoded::RawMessage rawMsg;
    for (int i = 0; i < RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE; i++)
    {
      rawMsg.raw[i] = msg.raw[i];
    }
    cmdRadioChannel.queue->AddMessage(rawMsg);
    return;
  }

  // Callback function to look at images received
  void callbackLeftImage(const sensor_msgs::CameraInfoConstPtr& infra1_msg)
  {
      imageReceivedLeft = true;
  }
  void callbackRightImage(const sensor_msgs::CameraInfoConstPtr& infra2_msg)
  {
      imageReceivedRight = true;
  }

  void callbackDepthImage(const sensor_msgs::CameraInfoConstPtr& depth_msg)
  {
      imageReceivedDepth = true;
  }
};

std::vector<shared_ptr<SimVehicle>> vehicles;

template <typename Real>
string toCSV(const Vec3<Real> v)
{
  stringstream ss;
  ss << v.x << "," << v.y << "," << v.z << ",";
  return ss.str();
}

int main(int argc, char **argv)
{
  ////////////////////////////////////////////////////////////////
  // ROS setup
  ////////////////////////////////////////////////////////////////
  bool simVIO = true;
  bool simDepth = true;
  

  int numVehicles = argc - 1;
  if (numVehicles < 1)
  {
    cout << "ERROR: Must specify at least one vehicle ID\n";
    cout
        << "\t You can input a list of vehicle IDs, and the simulator will spawn many vehicles.\n";
    return -1;
  }

  ros::init(argc, argv, "sync_simulator");
  ros::NodeHandle n;
  n.setParam("/use_sim_time", true); // Tell ROS we're using sim time

  for (unsigned i = 0; i < numVehicles; i++)
  {
    int const vehicleId = atol(argv[1 + i]);
    if (vehicleId <= 0 || vehicleId > 255)
    {
      cout << "\n\n\n";
      cout << "ERROR: invalid vehicle ID <" << argv[1 + i] << ">\n";
      return -1;
    }

    shared_ptr<SimVehicle> v;
    v.reset(new SimVehicle());
    v->id = vehicleId;
    v->subRadioCmd.reset(
        new ros::Subscriber(
            n.subscribe("radio_command" + std::to_string(vehicleId), 1,
                        &SimVehicle::callbackRadioCmd, v.get())));

    if (simVIO){
      v->subImageLeft.reset(
        new ros::Subscriber(
            n.subscribe("/d455/infra1/camera_info", 1, &SimVehicle::callbackLeftImage, v.get())));
      v->subImageRight.reset(
        new ros::Subscriber(
            n.subscribe("/d455/infra2/camera_info", 1, &SimVehicle::callbackRightImage, v.get())));
    }
    
    if (simDepth){
      v->subImageDepth.reset(
        new ros::Subscriber(
            n.subscribe("/d455/depth/camera_info", 1, &SimVehicle::callbackDepthImage, v.get())));
    }

    v->pubSimTruth.reset(
        new ros::Publisher(
            n.advertise<hiperlab_rostools::simulator_truth>(
                "simulator_truth" + std::to_string(vehicleId), 1)));
    v->pubMoCap.reset(
        new ros::Publisher(
            n.advertise<hiperlab_rostools::mocap_output>(
                "mocap_output" + std::to_string(vehicleId), 1)));
    v->pubTelemetry.reset(
        new ros::Publisher(
            n.advertise<hiperlab_rostools::telemetry>(
                "telemetry" + std::to_string(vehicleId), 1)));
    v->pubSimTime.reset(
      new ros::Publisher(n.advertise<rosgraph_msgs::Clock>("/clock", 1, true)));

    if (simVIO){
      v->pubCamIMU.reset(
        new ros::Publisher(
          n.advertise<sensor_msgs::Imu>(
            "d455/imu", 1)));

    }else{
      v->pubOdom.reset(
        new ros::Publisher(
            n.advertise<nav_msgs::Odometry>(
                "odom_data", 1)));
    }
    
    v->pubHeartbeat.reset(
        new ros::Publisher(
            n.advertise<std_msgs::Bool>(
                "heartbeat_signal", 1)));
    vehicles.push_back(v);
  }

  ////////////////////////////////////////////////////////////////
  // Simulator setup
  ////////////////////////////////////////////////////////////////
  // Basic timing:
  const double frequencySimulation = 1000.0; // run the simulation at this rate
  const double frequencyMocapOutput = 200;  //[s] Ideally, 1/mocapOuptputFreq is a constant multiple of 1/frequencySimulation.
  const double frequencyTelemetry = 100;    //[s] Ideally, 1/mocapOuptputFreq is a constant multiple of 1/frequencySimulation.
  const double frequencyHeartbeat = 10;
  const double frequencyCamIMU = 400;
  const double freqencyImage = 15;

  // const double endTime = 10.0f; //[s]
  const double imageStep = 1.0 / freqencyImage; // We need to wait for images after this time
  const double stepTime = 1.0 / frequencySimulation; // Increment clock by this
  ManualTimer simTimer; // Use manual timer instead


  // The communication transport delay:
  double const timeDelayOffboardControlLoop = 20e-3; //[s] TODO: we should measure this!

  // noise in the UWB system:
  double uwbNoiseStdDev = 50e-3; //[m]
  double uwbOutlierProb = 0.01;
  double uwbOutlierStdDev = 10; //[m]

  unsigned numPacketsOverTelemetry = TelemetryPacket::NUM_PACKETS_OVER_TELEMETRY;

  for (auto v : vehicles)
  {
    // create the vehicles
    Onboard::QuadcopterConstants::QuadcopterType quadcopterType =
        Onboard::QuadcopterConstants::GetVehicleTypeFromID(v->id);

    Onboard::QuadcopterConstants vehConsts(quadcopterType);
    // Create the quadcopter:
    double const mass = vehConsts.mass;                                    //[kg]
    double const inertia_xx = vehConsts.Ixx;                        //[kg.m**2]
    double const inertia_yy = inertia_xx;                                  //[kg.m**2]
    double const inertia_zz = vehConsts.Izz;                        //[kg.m**2]
    double armLength = vehConsts.armLength;                                //[m]
    double propThrustFromSpeedSqr = vehConsts.propellerThrustFromSpeedSqr; //[N/(rad/s)**2]
    double propTorqueFromSpeedSqr = vehConsts.propellerTorqueFromThrust * vehConsts.propellerThrustFromSpeedSqr;

    double motorTimeConst = vehConsts.motorTimeConst; // [s]
    double motorInertia = vehConsts.motorInertia;     // [kg.m**2]
    double motorMinSpeed = vehConsts.motorMinSpeed;   // [rad/s]
    double motorMaxSpeed = vehConsts.motorMaxSpeed;   // [rad/s]

    Vec3d centreOfMassError = Vec3d(0, 0, 0);

    Eigen::Matrix<double, 3, 3> inertiaMatrix;
    inertiaMatrix << inertia_xx, 0, 0, 0, inertia_yy, 0, 0, 0, inertia_zz;

    // drag coefficients
    Vec3d linDragCoeffB = Vec3d(vehConsts.linDragCoeffBx,
                                vehConsts.linDragCoeffBy,
                                vehConsts.linDragCoeffBz);

    v->vehicle.reset(
        new Simulation::Quadcopter(&simTimer, mass, inertiaMatrix, armLength,
                                   centreOfMassError, motorMinSpeed,
                                   motorMaxSpeed, propThrustFromSpeedSqr,
                                   propTorqueFromSpeedSqr, motorTimeConst,
                                   motorInertia, linDragCoeffB, v->id,
                                   quadcopterType, 1.0 / frequencySimulation));

    // create an initial error:
    double maxDistAlongAxis = 1.5;
    double minSpacing = 0.5;
    int NN = int(maxDistAlongAxis / minSpacing + 0.5);

    // int ix = v->id % (2 * NN) - NN;
    // int iy = v->id / (2 * NN) - NN;

    double initPosx = 0.0;
    double initPosy = 0.0;
    Vec3d const initErrPos = Vec3d(initPosx, initPosy, 0);
    Rotationd const initErrAtt = Rotationd::FromEulerYPR(0 * M_PI / 180.0,
                                                         0 * M_PI / 180.0, 0);

    cout << "Adding vehicle # " << int(v->id) << " ("
         << Onboard::QuadcopterConstants::GetNameString(quadcopterType)
         << ") at <" << toCSV(initErrPos) << ">\n";
    v->vehicle->SetPosition(initErrPos);
    v->vehicle->SetAttitude(initErrAtt);

    v->cmdRadioChannel.queue.reset(
        new Simulation::CommunicationsDelay<
            RadioTypes::RadioMessageDecoded::RawMessage>(
            &simTimer, timeDelayOffboardControlLoop));
  }

  // Create other UWB objects:
  double const uwbNetworkRangingPeriod = 1.0 / 100; //[s], runs at approx. 100Hz.

  // Create some static UWB radios (Ids = 2,3,4)
  vector<shared_ptr<Simulation::UWBRadio>> staticRadios;

  shared_ptr<Simulation::UWBRadio> r2, r3, r4;
  r2.reset(new Simulation::UWBRadio(&simTimer, 2));
  r2->SetPosition(Vec3d(10, 0, 0));
  r3.reset(new Simulation::UWBRadio(&simTimer, 3));
  r3->SetPosition(Vec3d(0, 10, 0));
  r4.reset(new Simulation::UWBRadio(&simTimer, 4));
  r4->SetPosition(Vec3d(0, 0, 10));

  // add all radios to the network
  shared_ptr<Simulation::UWBNetwork> uwbNetwork;
  uwbNetwork.reset(
      new Simulation::UWBNetwork(&simTimer, uwbNetworkRangingPeriod));
  uwbNetwork->SetNoiseProperties(uwbNoiseStdDev, uwbOutlierProb,
                                 uwbOutlierStdDev);
  uwbNetwork->AddRadio(r2);
  uwbNetwork->AddRadio(r3);
  uwbNetwork->AddRadio(r4);

  for (auto v : vehicles)
  {
    if (v->vehicle->GetRadio())
    {
      uwbNetwork->AddRadio(v->vehicle->GetRadio());
    }
    // Tell the vehicle firmware about the radios:
    v->vehicle->AddUWBRadioTarget(r2->GetId(), r2->GetPosition());
    v->vehicle->AddUWBRadioTarget(r3->GetId(), r3->GetPosition());
    v->vehicle->AddUWBRadioTarget(r4->GetId(), r4->GetPosition());
  }

  cout << "Starting simulation\n";
  Timer t(&simTimer);
  double timePrintNextInfo = 0;
  double timePublishNextMocap = 0;
  double timePublishNextTelemetry = 0;
  double timePublishNextHeartbeat = 0;
  double timePublishNextCamIMU = 0;
  double timeWaitNextImage = imageStep;


  ros::Rate loop_rate(frequencySimulation);

  Timer cmdRadioTimer(&simTimer);

  while (ros::ok())
  {
    ros::spinOnce();

    for (auto v : vehicles)
    { 
      if (simVIO){
        // Advance clock if 1) Image has been received 2) It's not yet time to wait for image
        if (t.GetSeconds<double>() < timeWaitNextImage || (v->imageReceivedLeft && v->imageReceivedRight)){
          // ROS_INFO("Advance clock.");
          if (v->imageReceivedLeft && v->imageReceivedRight)
          {
            // ROS_INFO("Stereo received.");
            v->imageReceivedLeft = false;
            v->imageReceivedRight = false;
            timeWaitNextImage += imageStep;
            // ROS_INFO("Updated timeWaitNextImage: %f", timeWaitNextImage);
          }
          simTimer.AdvanceMicroSeconds(uint64_t(stepTime * 1e6));
          ros::Time currentTime(t.GetSeconds_f());
          v->simTimeMsg.clock = currentTime;
          v->pubSimTime->publish(v->simTimeMsg);  // Overwrite clock topic
          v->vehicle->Run();
        }
        // else{
        //   ROS_INFO("Current sim time: %f, timeWaitNextImage: %f", t.GetSeconds<double>(), timeWaitNextImage);
        // }
      }

      if (simDepth){
        // Advance clock if 1) Image has been received 2) It's not yet time to wait for image
        if (t.GetSeconds<double>() < timeWaitNextImage || v->imageReceivedDepth){
        // if (true){
          // ROS_INFO("Advance clock.");
          if (v->imageReceivedDepth)
          {
            // ROS_INFO("Depth received.");
            v->imageReceivedDepth = false;
            timeWaitNextImage += imageStep;
            // ROS_INFO("Updated timeWaitNextImage: %f", timeWaitNextImage);
          }
          simTimer.AdvanceMicroSeconds(uint64_t(stepTime * 1e6));
          ros::Time currentTime(t.GetSeconds_f());
          v->simTimeMsg.clock = currentTime;
          v->pubSimTime->publish(v->simTimeMsg);  // Overwrite clock topic
          v->vehicle->Run();
        }
        // else{
        //   ROS_INFO("Current sim time: %f, timeWaitNextImage: %f", t.GetSeconds<double>(), timeWaitNextImage);
        // }
      }


    }
    uwbNetwork->Run();

    if (t.GetSeconds<double>() > timePrintNextInfo)
    {
      timePrintNextInfo += 1;
      cout << "Current sim time = " << int(t.GetSeconds<double>() + 0.5)
           << "s; ";
      for (auto v : vehicles)
      {
        cout << "(" << v->id << ") pos = <" << toCSV(v->vehicle->GetPosition())
             << ">";
      }
      cout << "\n";
    }

    {
      // see if there are any new radio messages. We put this in {braces} for the scoped mutex trick

      for (auto v : vehicles)
      {
        std::lock_guard<std::mutex> guard(cmdRadioChannelMutex);
        if (v->cmdRadioChannel.queue->HaveNewMessage())
        {

          v->vehicle->SetCommandRadioMsg(
              v->cmdRadioChannel.queue->GetMessage());
        }
      }
    }

    if (t.GetSeconds<double>() > timePublishNextMocap)
    {
      timePublishNextMocap += 1 / frequencyMocapOutput;

      for (auto v : vehicles)
      {
        hiperlab_rostools::mocap_output mocapOutMsg;
        mocapOutMsg.header.stamp = ros::Time::now();

        mocapOutMsg.vehicleID = v->id;

        // TODO: noise!
        Vec3d measPos = v->vehicle->GetPosition();
        Rotationd measAtt = v->vehicle->GetAttitude();

        mocapOutMsg.posx = measPos.x;
        mocapOutMsg.posy = measPos.y;
        mocapOutMsg.posz = measPos.z;

        mocapOutMsg.attq0 = measAtt[0];
        mocapOutMsg.attq1 = measAtt[1];
        mocapOutMsg.attq2 = measAtt[2];
        mocapOutMsg.attq3 = measAtt[3];

        measAtt.ToEulerYPR(mocapOutMsg.attyaw, mocapOutMsg.attpitch,
                           mocapOutMsg.attroll);

        v->pubMoCap->publish(mocapOutMsg);

        // Publish also the simulation truth:
        hiperlab_rostools::simulator_truth simTruthMsg;
        simTruthMsg.header.stamp = ros::Time::now();
        simTruthMsg.posx = v->vehicle->GetPosition().x;
        simTruthMsg.posy = v->vehicle->GetPosition().y;
        simTruthMsg.posz = v->vehicle->GetPosition().z;

        simTruthMsg.velx = v->vehicle->GetVelocity().x;
        simTruthMsg.vely = v->vehicle->GetVelocity().y;
        simTruthMsg.velz = v->vehicle->GetVelocity().z;

        simTruthMsg.attq0 = v->vehicle->GetAttitude()[0];
        simTruthMsg.attq1 = v->vehicle->GetAttitude()[1];
        simTruthMsg.attq2 = v->vehicle->GetAttitude()[2];
        simTruthMsg.attq3 = v->vehicle->GetAttitude()[3];
        v->vehicle->GetAttitude().ToEulerYPR(simTruthMsg.attyaw,
                                             simTruthMsg.attpitch,
                                             simTruthMsg.attroll);

        simTruthMsg.angvelx = v->vehicle->GetAngularVelocity().x;
        simTruthMsg.angvely = v->vehicle->GetAngularVelocity().y;
        simTruthMsg.angvelz = v->vehicle->GetAngularVelocity().z;

        v->pubSimTruth->publish(simTruthMsg);

        // Publish also to odom_data
        if (!simVIO){
          // If not using VIO in simulation, publish odometry that's identical to truth
          nav_msgs::Odometry odomMsg;
          odomMsg.header.stamp = ros::Time::now();
          odomMsg.header.frame_id = "mocap";
          odomMsg.child_frame_id = "mocap";
          odomMsg.pose.pose.position.x = v->vehicle->GetPosition().x;
          odomMsg.pose.pose.position.y = v->vehicle->GetPosition().y;
          odomMsg.pose.pose.position.z = v->vehicle->GetPosition().z;

          odomMsg.twist.twist.linear.x = v->vehicle->GetVelocity().x;
          odomMsg.twist.twist.linear.y = v->vehicle->GetVelocity().y;
          odomMsg.twist.twist.linear.z = v->vehicle->GetVelocity().z;

          odomMsg.pose.pose.orientation.w = v->vehicle->GetAttitude()[0];
          odomMsg.pose.pose.orientation.x = v->vehicle->GetAttitude()[1];
          odomMsg.pose.pose.orientation.y = v->vehicle->GetAttitude()[2];
          odomMsg.pose.pose.orientation.z = v->vehicle->GetAttitude()[3];
          // v->vehicle->GetAttitude().ToEulerYPR(simTruthMsg.attyaw,
          //                                      simTruthMsg.attpitch,
          //                                      simTruthMsg.attroll);

          odomMsg.twist.twist.angular.x = v->vehicle->GetAngularVelocity().x;
          odomMsg.twist.twist.angular.y = v->vehicle->GetAngularVelocity().y;
          odomMsg.twist.twist.angular.z = v->vehicle->GetAngularVelocity().z;

          v->pubOdom->publish(odomMsg);
        }

      }
    }

    if (t.GetSeconds<double>() > timePublishNextTelemetry)
    {
      // Telemetry message
      timePublishNextTelemetry += 1 / frequencyTelemetry;

      for (auto v : vehicles)
      {
        hiperlab_rostools::telemetry telMsgOut;
        // Fill out the telemetry package
        TelemetryPacket::data_packet_t dataPacketRaw1, dataPacketRaw2, dataPacketRaw3, dataPacketRaw4;
        v->vehicle->GetTelemetryDataPackets(dataPacketRaw1, dataPacketRaw2, dataPacketRaw3, dataPacketRaw4);

        TelemetryPacket::TelemetryPacket dataPacket1, dataPacket2, dataPacket3, dataPacket4;
        TelemetryPacket::DecodeTelemetryPacket(dataPacketRaw1, dataPacket1);
        TelemetryPacket::DecodeTelemetryPacket(dataPacketRaw2, dataPacket2);
        TelemetryPacket::DecodeTelemetryPacket(dataPacketRaw3, dataPacket3);
        TelemetryPacket::DecodeTelemetryPacket(dataPacketRaw4, dataPacket4);

        telMsgOut.packetNumber = dataPacket1.packetNumber;
        for (int i = 0; i < 3; i++)
        {
          telMsgOut.accelerometer[i] = dataPacket1.accel[i];
          telMsgOut.rateGyro[i] = dataPacket1.gyro[i];
          telMsgOut.position[i] = dataPacket1.position[i];
        }

        for (int i = 0; i < 4; i++)
        {
          telMsgOut.motorForces[i] = dataPacket1.motorForces[i];
        }
        telMsgOut.batteryVoltage = dataPacket1.battVoltage;

        for (int i = 0; i < TelemetryPacket::TelemetryPacket::NUM_DEBUG_FLOATS;
             i++)
        {
          telMsgOut.debugVals[i] = dataPacket2.debugVals[i];
        }

        Vec3f attYPR = Rotationf::FromVectorPartOfQuaternion(
                           Vec3f(dataPacket2.attitude[0], dataPacket2.attitude[1],
                                 dataPacket2.attitude[2]))
                           .ToEulerYPR();
        for (int i = 0; i < 3; i++)
        {
          telMsgOut.velocity[i] = dataPacket2.velocity[i];
          telMsgOut.attitude[i] = dataPacket2.attitude[i];
          telMsgOut.attitudeYPR[i] = attYPR[i];
        }
        telMsgOut.panicReason = dataPacket2.panicReason;
        telMsgOut.warnings = dataPacket2.warnings;

        if (numPacketsOverTelemetry >= 3)
        {
          for (int i = 0; i < TelemetryPacket::TelemetryPacket::NUM_FLOATS_PER_PACKET; i++)
          {
            telMsgOut.customPacket1[i] = dataPacket3.customPacket1[i];
          }
        }
        if (numPacketsOverTelemetry >= 4)
        {
          for (int i = 0; i < TelemetryPacket::TelemetryPacket::NUM_FLOATS_PER_PACKET; i++)
          {
            telMsgOut.customPacket2[i] = dataPacket4.customPacket2[i];
          }
        }

        telMsgOut.header.stamp = ros::Time::now();

        v->pubTelemetry->publish(telMsgOut);
      }
    }


    if (t.GetSeconds<double>() > timePublishNextHeartbeat)
    {
      timePublishNextHeartbeat += 1.0 / frequencyHeartbeat;
      for (auto v : vehicles)
      {
        std_msgs::Bool heartbeatMsg;
        heartbeatMsg.data = true;

        v->pubHeartbeat->publish(heartbeatMsg);
      }
    }

    // Publishing Camera IMU message
    if (simVIO){
      if (t.GetSeconds<double>() > timePublishNextCamIMU)
      {
        timePublishNextCamIMU += 1.0 / frequencyCamIMU;
        for (auto v : vehicles)
        {
          // Fill out the telemetry package
          TelemetryPacket::data_packet_t dataPacketRaw1, dataPacketRaw2, dataPacketRaw3, dataPacketRaw4;
          v->vehicle->GetTelemetryDataPackets(dataPacketRaw1, dataPacketRaw2, dataPacketRaw3, dataPacketRaw4);

          TelemetryPacket::TelemetryPacket dataPacket1, dataPacket2, dataPacket3, dataPacket4;
          TelemetryPacket::DecodeTelemetryPacket(dataPacketRaw1, dataPacket1); // Only need 1 to get IMU

          sensor_msgs::Imu imuMsg;
          imuMsg.header.stamp = ros::Time::now();
          imuMsg.header.frame_id = "d455_imu_optical_frame";

          // Fill in the IMU data here
          imuMsg.angular_velocity.x = dataPacket1.gyro[1];
          imuMsg.angular_velocity.y = -dataPacket1.gyro[2];
          imuMsg.angular_velocity.z = -dataPacket1.gyro[0];

          imuMsg.linear_acceleration.x = dataPacket1.accel[1];
          imuMsg.linear_acceleration.y = -(dataPacket1.accel[2] - 9.81f) - 9.81f;
          imuMsg.linear_acceleration.z = -dataPacket1.accel[0];

          v->pubCamIMU->publish(imuMsg);
        }
      }

    }
    // loop_rate.sleep();
  }
  cout << "Done.\n";
}
