#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <poll.h>
#include <signal.h>

#include <memory>
#include <mutex>

#include "cfradio.h"

#include <ros/ros.h>
#include "hiperlab_rostools/radio_command.h"
#include "hiperlab_rostools/telemetry.h"

#include "Common/DataTypes/RadioTypes.hpp"
#include "Common/DataTypes/TelemetryPacket.hpp"
#include "Common/Math/Rotation.hpp"
#include "Common/Time/HardwareTimer.hpp"
#include "Common/Time/Timer.hpp"

using namespace std;

ros::Publisher pubTelemetry;

static libusb_context *ctx;

static bool running;
unsigned volatile numCmdsTx = 0;
unsigned volatile numCmdsReceivedOverNetwork = 0;
unsigned volatile numTelemetryRx = 0;
unsigned volatile numTelemetryDecodingErrs = 0;

static cfradio radio;
static_assert(RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE <= CRTP_MAX_DATA_SIZE, "Cannot have radio packets larger than CRTP_MAX_DATA_SIZE");

struct {
  crtp_message_t buf[1];  // Messages received from GCS which should be sent to usb
  bool volatile haveMsg;
} messageToBeTx;

struct {
  mutex msgMutex;  //protect against concurrency problems
  uint8_t raw[RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE];
  volatile bool isNew;
} latestRadioCmdMessage;

struct {
  mutex telMutex;  //protect against concurrency problems
  hiperlab_rostools::telemetry telMsgOut;
} latestTelemetry;

void callbackRadioCmd(const hiperlab_rostools::radio_command& msg) {

  std::lock_guard<std::mutex> guard(latestRadioCmdMessage.msgMutex);
  latestRadioCmdMessage.isNew = true;

  //todo: should be a nicer way to do this, using e.g. memcpy...
  for (int i = 0; i < RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE; i++) {
    latestRadioCmdMessage.raw[i] = msg.raw[i];
  }

  numCmdsReceivedOverNetwork++;

  static bool printFirst = false;
  if (!printFirst) {
    printf("Received first radio command in callback\n");
    printFirst = true;
  }
}

void signal_sigint(int s) {
  running = false;
}

// Grabs a message from the client buffer if there are any
int fetch_message(crtp_message_t *buf) {
//  if (cfclient_buffer_empty (&client)) {
  if (!messageToBeTx.haveMsg) {
    return 0;
  }

  crtp_message_t *msg = messageToBeTx.buf;
  // TODO: This could be more efficient
  memcpy(buf, msg, sizeof(crtp_message_t));
  messageToBeTx.haveMsg = false;
  return 1;
}

int handle_message(int status, crtp_message_t *msg) {
  if (msg->port != CRTP_PORT_MAVLINK)
    return 0;

  static bool firstHandleMessage = true;
  if (firstHandleMessage) {
    firstHandleMessage = false;
    printf("Got firstHandleMessage!\n");
  }

  int bytes_received = msg->size;
#define BUFFER_SIZE 32//magic number!

  if (bytes_received < 0) {
    printf("recvfrom() failed\n");
    return 0;
  } else if (bytes_received == 0) {
    std::cout << "No bytes received.\n";
  } else {

    TelemetryPacket::data_packet_t data =
        *((TelemetryPacket::data_packet_t *) msg->data);

    if (data.type == TelemetryPacket::PACKET_TYPE_QUAD_TELEMETRY_PT1) {
      //new packet!
      TelemetryPacket::TelemetryPacket src;
      TelemetryPacket::DecodeTelemetryPacket(data, src);

      std::lock_guard<std::mutex> guard(latestTelemetry.telMutex);

      //first in a sequence of packets.
      latestTelemetry.telMsgOut.packetNumber = src.packetNumber;
      for (int i = 0; i < 3; i++) {
        latestTelemetry.telMsgOut.accelerometer[i] = src.accel[i];
        latestTelemetry.telMsgOut.rateGyro[i] = src.gyro[i];
        latestTelemetry.telMsgOut.position[i] = src.position[i];
      }
      for (int i = 0; i < 4; i++) {
        latestTelemetry.telMsgOut.motorForces[i] = src.motorForces[i];
      }
      latestTelemetry.telMsgOut.batteryVoltage = src.battVoltage;

    } else if (data.type == TelemetryPacket::PACKET_TYPE_QUAD_TELEMETRY_PT2) {
      //expect a continuation
      TelemetryPacket::TelemetryPacket src;
      TelemetryPacket::DecodeTelemetryPacket(data, src);

      std::lock_guard<std::mutex> guard(latestTelemetry.telMutex);

      if (src.packetNumber != latestTelemetry.telMsgOut.packetNumber) {
        /*
         printf(
         "Telemetry packet Rx ID error, old | new: packetNumber %d | %d\n",
         latestTelemetry.telMsgOut.packetNumber, src.packetNumber);
         */
        numTelemetryDecodingErrs++;
        //nothing to do, though
      } else {
        //packet OK!
        Vec3f attYPR =
            Rotationf::FromVectorPartOfQuaternion(
                Vec3f(src.attitude[0], src.attitude[1], src.attitude[2]))
                .ToEulerYPR();
        for (int i = 0; i < 3; i++) {
          latestTelemetry.telMsgOut.velocity[i] = src.velocity[i];
          latestTelemetry.telMsgOut.attitude[i] = src.attitude[i];
          latestTelemetry.telMsgOut.attitudeYPR[i] = attYPR[i];
        }
        for (int i = 0; i < TelemetryPacket::TelemetryPacket::NUM_DEBUG_FLOATS;
            i++) {
          latestTelemetry.telMsgOut.debugVals[i] = src.debugVals[i];
        }
        latestTelemetry.telMsgOut.panicReason = src.panicReason;
        latestTelemetry.telMsgOut.warnings = src.warnings;

        if (TelemetryPacket::NUM_PACKETS_OVER_TELEMETRY == 2) {
          latestTelemetry.telMsgOut.header.stamp = ros::Time::now();
          pubTelemetry.publish(latestTelemetry.telMsgOut);
          numTelemetryRx++;
        }
      }

    } else if (data.type == TelemetryPacket::PACKET_TYPE_QUAD_TELEMETRY_PT3) {
      //expect a continuation
      TelemetryPacket::TelemetryPacket src;
      TelemetryPacket::DecodeTelemetryPacket(data, src);

      std::lock_guard<std::mutex> guard(latestTelemetry.telMutex);

      if (src.packetNumber != latestTelemetry.telMsgOut.packetNumber) {
        /*
         printf(
         "Telemetry packet Rx ID error, old | new: packetNumber %d | %d\n",
         latestTelemetry.telMsgOut.packetNumber, src.packetNumber);
         */
        numTelemetryDecodingErrs++;
        //nothing to do, though
      } else {
        //packet OK!
        for (int i = 0; i < TelemetryPacket::TelemetryPacket::NUM_FLOATS_PER_PACKET; i++) {
          latestTelemetry.telMsgOut.customPacket1[i] = src.customPacket1[i];
        }

        if (TelemetryPacket::NUM_PACKETS_OVER_TELEMETRY == 3) {
          latestTelemetry.telMsgOut.header.stamp = ros::Time::now();
          pubTelemetry.publish(latestTelemetry.telMsgOut);
          numTelemetryRx++;
        }
      }

    } else if (data.type == TelemetryPacket::PACKET_TYPE_QUAD_TELEMETRY_PT4) {
      //expect a continuation
      TelemetryPacket::TelemetryPacket src;
      TelemetryPacket::DecodeTelemetryPacket(data, src);

      std::lock_guard<std::mutex> guard(latestTelemetry.telMutex);

      if (src.packetNumber != latestTelemetry.telMsgOut.packetNumber) {
        /*
         printf(
         "Telemetry packet Rx ID error, old | new: packetNumber %d | %d\n",
         latestTelemetry.telMsgOut.packetNumber, src.packetNumber);
         */
        numTelemetryDecodingErrs++;
        //nothing to do, though
      } else {
        //packet OK!
        for (int i = 0; i < TelemetryPacket::TelemetryPacket::NUM_FLOATS_PER_PACKET; i++) {
          latestTelemetry.telMsgOut.customPacket2[i] = src.customPacket2[i];
        }

        latestTelemetry.telMsgOut.header.stamp = ros::Time::now();
        pubTelemetry.publish(latestTelemetry.telMsgOut);
        numTelemetryRx++;
      }

    } else if (data.type == TelemetryPacket::PACKET_TYPE_GENERIC_FLOAT) {
      // Hardcoding number of packets as maximum number allowed (14)
      float floats[14];
      TelemetryPacket::DecodeFloatPacket(data, floats, 14);

      /* Hardcoding limits on floats as a,b here. Depending on what data
       was originally sent, this may not be valid to do. */
      double a = -10;
      double b = 10;

      printf("Type: %d\n", data.type);
      printf("ID: %d\n", data.packetNumber);

      for (int i = 0; i < 14; i++) {
        printf("f%d: %.3f\n", i, TelemetryPacket::MapToAB(floats[i], a, b));
      }
      printf("\n");
    }
  }

  // Forward to udp
  /*mwm: TODO: do something here...
   sendto(client.netfd, &msg->data, msg->size - 1, 0,
   (const sockaddr*) (&client.client_addr), sizeof(client.client_addr));
   */
  return 1;
}

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

int main(int argc, char *argv[]) {
  ////////////////////////////////////////////////////////////////
  //ROS setup
  ////////////////////////////////////////////////////////////////
  if (argc < 2) {
    printf("ERROR: Must specify the vehicle ID\n");
    return -1;
  }

  int const vehicleId = atol(argv[1]);
  if (vehicleId <= 0 || vehicleId > 255) {
    printf("ERROR: invalid vehicle ID\n");
    return -1;
  }

  ros::init(argc, argv, "crazyradio" + std::to_string(vehicleId));

  ros::NodeHandle n;
  ros::Subscriber subRadioCmd = n.subscribe(
      "radio_command" + std::to_string(vehicleId), 1, callbackRadioCmd);
  pubTelemetry = n.advertise<hiperlab_rostools::telemetry>(
      "telemetry" + std::to_string(vehicleId), 1);

  ////////////////////////////////////////////////////////////////
  //USB setup
  ////////////////////////////////////////////////////////////////
  int res = 0;
  if (libusb_init(&ctx) != 0) {
    printf("Failed to init libusb\n");
    return 1;
  }

  //int cfradio_open(cfradio *radio, libusb_context *ctx, cfradio_fetcher fetcher, cfradio_handler handler) {
  if (cfradio_open(&radio, ctx, fetch_message, handle_message, vehicleId)
      != 0) {
    printf("Failed to init radio. ");
    printf("Make sure the radio is connected. ");
    printf("If all else fails, try running as `sudo`. \n");
    return 1;
  }

  struct pollfd fds[16];  // TODO: We won't need all of them, but I don't know how many libusb needs
  int nfds = 0;

  // First descriptor for udp socket
//  fds[0].fd = client.netfd;
//  fds[0].events = POLLIN;  // TODO: Set netfd to non-blocking and allow the OS to buffer the sendto if needed

// Rest for usb
  const struct libusb_pollfd **usbfds = libusb_get_pollfds(ctx);
  for (int i = 0; usbfds[i] != NULL; i++) {
    fds[nfds].fd = usbfds[i]->fd;
    fds[nfds].events = usbfds[i]->events;
    nfds++;
  }
  //libusb_free_pollfds(usbfds);

  struct timeval tv;
  memset(&tv, 0, sizeof(tv));

  printf("Waiting for messages (vehicle id=%d)...\n", vehicleId);

  //Start ROS spinner:
  ros::AsyncSpinner spinner(0);
  spinner.start();

  HardwareTimer timer;
  Timer tPrintTitles(&timer);
  Timer tPrintStats(&timer);

  tPrintTitles.AdjustTimeBySeconds(100);  //force to print immediately.

  const double dtPrintTitles = 10.0f;
  const double dtPrintStats = 1.0f;

  printf("----------------------------------------\n");

  while (ros::ok()) {
    res = poll(fds, nfds, 1);
    if (res < 0) {
      // Error
      continue;
    } else if (res == 0 //timeout
        && !latestRadioCmdMessage.isNew //no new ros msg
        ) {
      cfradio_notify(&radio);
      continue;
    }

    if (tPrintTitles.GetSeconds_d() > dtPrintTitles) {
      tPrintTitles.Reset();
      tPrintStats.AdjustTimeBySeconds(100);  //force print stats
      for (int i = 0; i < 20; i++) {
        printf("\n");
      }
      printf("\n  Counts are totals:\n");
      //       123 1234567890 1234567890 1234567890 1234567890
      printf("+---+----------+----------+----------+----------+\n");
      printf("|ID |cmd Rx ROS|cmd Tx rad|tel Rx    |decode err|\n");
      printf("+---+----------+----------+----------+----------+\n");
      fflush(stdout);
    }

    if (tPrintStats.GetSeconds_d() > dtPrintStats) {
      tPrintStats.Reset();
      printf("\r");
      printf("|%*d", 3, vehicleId);
      printf("|%*d", 10, numCmdsReceivedOverNetwork);
      printf("|%*d", 10, numCmdsTx);
      printf("|%*d", 10, numTelemetryRx);
      printf("|%*d", 10, numTelemetryDecodingErrs);
      printf("|");

      fflush(stdout);
    }

    //scope for mutex:
    {
      //Send out commands, if needed:
      std::lock_guard<std::mutex> guard(latestRadioCmdMessage.msgMutex);
      if (latestRadioCmdMessage.isNew) {
        latestRadioCmdMessage.isNew = false;

        int size = MIN(CRTP_MAX_DATA_SIZE,
                       RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE);

        crtp_message_t *msg = messageToBeTx.buf;
        msg->size = size + 1;
        msg->header = 0;
        msg->port = CRTP_PORT_MAVLINK;
        memcpy(msg->data, latestRadioCmdMessage.raw, size);

        static bool firstRosMsg = true;
        if (firstRosMsg) {
          firstRosMsg = false;
          printf("ROS msg size = %d:\n", msg->size);
          for (int i = 0; i < size; i++) {
            printf("%02d,", int(msg->data[i]));
          }
          printf("\n");
        }

        messageToBeTx.haveMsg = true;

        cfradio_notify(&radio);
        numCmdsTx++;
      }
    }

    // The rest must be usb fds
    if (res > 0) {
      libusb_handle_events_timeout_completed(ctx, &tv, NULL);
    }
  }

  printf("Releasing...\n");

  cfradio_close(&radio);

//  libusb_exit(ctx);
  printf("Done!\n");
}
