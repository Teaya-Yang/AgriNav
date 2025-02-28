/*********************************************************************************
 * ESP Radio PC App
 *
 * After setting up your boards, connect the PC board to your computer and run this program
 * From the terminal, run: rosrun hiperlab_hardware esp_radio vehicleId uartPort
 * Replace vehicleId with the ID of the vehicle you want to connect to
 * Replace uartPort with the port of the ESP board, if you don't put anything, it will default to /dev/ttyUSB0
 * You can find the port by running ls /dev/tty* and looking for the port that appears when you plug in the board
 *
 * - Jerry Tang
 *********************************************************************************/
/* General and UART communication */
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <ctime>

/* ROS and hiperlab messages */
#include "Common/DataTypes/RadioTypes.hpp"
#include "Common/DataTypes/TelemetryPacket.hpp"
#include "Common/Math/Rotation.hpp"
#include "Common/Time/Timer.hpp"
#include "Common/Time/HardwareTimer.hpp"
#include "hiperlab_rostools/radio_command.h"
#include "hiperlab_rostools/telemetry.h"
#include <ros/ros.h>

using namespace std;
/* Functions */
uint16_t updateCRC16(uint16_t crc, uint8_t crc_seed);
uint16_t getCRC16(uint8_t *buf, uint16_t buf_len);

/* Commands and telemetry packets size */
const int command_size = RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE + 2; // + 2 bytes for CRC
const int telemetry_size = TelemetryPacket::ESPPacketSize::TOTAL_BYTES + 2;    // 80 bytes of data + 2 bytes of CRC-16

/* Variables, "receive" and "send" are referring to the UART data flow */
// Telemetry
const int send_buffer_size = command_size;
uint8_t send_buffer[send_buffer_size];
// Command
const int receive_buffer_size = telemetry_size;
uint8_t receive_buffer[receive_buffer_size];

/* Serial port configuration */
int uart;
// const char *device_name = "/dev/ttyUSB0"; // default port
const char *device_name = "/dev/ttyHS1"; // RB5
const speed_t speed = B921600;           // default baud rate

/* ROS and hiperlab messages */
ros::Publisher pubTelemetry; // Publishes telemetry data
hiperlab_rostools::telemetry tel_msg;

/* Debug radio frequency */
long time_now;
long time_last;

unsigned volatile numCmdsTx = 0;
unsigned volatile numTelemetryRx = 0;
unsigned volatile numTelemetryTx = 0;
unsigned volatile numCmdsRx = 0;

/* Open the serial port */
int openUart(const char *uart_name, struct termios *uart_config)
{
    int uart = open(uart_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart == -1)
    {
        std::cerr << "Failed to open serial port." << std::endl;
        return -1;
    }

    /* Fill the struct for the new configuration */
    tcgetattr(uart, uart_config);

    /* Disable output post-processing */
    uart_config->c_oflag &= ~OPOST;
    uart_config->c_cflag |= (CLOCAL | CREAD); /* ignore modem controls */
    uart_config->c_cflag &= ~CSIZE;
    uart_config->c_cflag |= CS8;      /* 8-bit characters */
    uart_config->c_cflag &= ~PARENB;  /* no parity bit */
    uart_config->c_cflag &= ~CSTOPB;  /* only need 1 stop bit */
    uart_config->c_cflag &= ~CRTSCTS; /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    uart_config->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    uart_config->c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    if (cfsetispeed(uart_config, speed) < 0 || cfsetospeed(uart_config, speed) < 0)
    {
        close(uart);
        return -1;
    }
    if (tcsetattr(uart, TCSANOW, uart_config) < 0)
    {
        close(uart);
        return -1;
    }
    return uart;
}

/* Close the serial port */
void closeuart(int uart) { close(uart); }

/* Update the command to the ESP board when a new command is received from the ROS network */
void callbackRadioCmd(const hiperlab_rostools::radio_command &msg)
{
    /* Copy the ROS message to the send buffer */
    for (int i = 0; i < send_buffer_size; i++)
    {
        send_buffer[i] = msg.raw[i];
    }
    /* Calculate CRC-16 and add it to the end of the buffer */
    uint16_t crc = getCRC16(send_buffer, send_buffer_size - 2);
    send_buffer[send_buffer_size - 2] = crc & 0xFF;
    send_buffer[send_buffer_size - 1] = (crc >> 8) & 0xFF;
    /* Send the command */
    int status_write = write(uart, &send_buffer, send_buffer_size);
    numCmdsTx++;
}

int main(int argc, char *argv[])
{
    /* Initialization */
    int vehicleId;
    if (argc >= 2)
    {
        /* Get vehicle ID */
        vehicleId = atol(argv[1]);
        if (vehicleId < 100 || vehicleId > 255)
        {
            printf("ERROR: invalid vehicle ID for ESP-style architecture \n");
            return -1;
        }
        /* Get UART device name */
        if (argc >= 3)
        {
            device_name = argv[2];
        }
    }
    else
    {
        printf("ERROR: missing vehicle ID \n");
        return -1;
    }

    /* ROS initialization */
    ros::init(argc, argv, "esp_radio" + std::to_string(vehicleId));
    ros::NodeHandle esp_radio;
    ros::Subscriber subRadioCmd = esp_radio.subscribe("radio_command" + std::to_string(vehicleId), 1, callbackRadioCmd);
    pubTelemetry = esp_radio.advertise<hiperlab_rostools::telemetry>("telemetry" + std::to_string(vehicleId), 1);

    HardwareTimer timer;
    Timer tPrintTitles(&timer);
    Timer tPrintStats(&timer);

    tPrintTitles.AdjustTimeBySeconds(100); // force to print immediately.

    const double dtPrintTitles = 10.0f;
    const double dtPrintStats = 1.0f;

    /* Open UART */
    struct termios uart_config;
    uart = openUart(device_name, &uart_config);
    if (uart == -1)
    {
        return -1;
    }

    /* Poll descriptor for checking telemetry data update */
    struct pollfd fds[1];
    fds[0].fd = uart;
    fds[0].events = POLLIN;
    const int telemetry_timeout_ms = 100; // 100 ms timeout

    /* Start ROS spin */
    ros::AsyncSpinner spinner(0);
    spinner.start();

    /* Start timer */
    time_now = ros::Time::now().toSec() * 1000;
    time_last = time_now;

    /* Main loop for receiving data */
    while (ros::ok())
    {
        /* Wait for a telemetry message */
        // TODO due to wait for, status cannot be printed in the same loop
        // ? why convert debug_vals back to  telemetry message has negative values
        // ? why uart (cmd, tel) not working
        int status = poll(fds, sizeof(fds) / sizeof(fds[0]), telemetry_timeout_ms);

        /* Receive telemetry section */
        /* Check if there is data available */
        int bytes_available = 0;
        ioctl(uart, FIONREAD, &bytes_available);
        /* Read the data from UART if all the data has been received */
        if (bytes_available >= receive_buffer_size)
        {
            int status_read = read(uart, &receive_buffer, receive_buffer_size);
            /* Get raw CRC from the end of the data buffer */
            uint16_t raw_crc = receive_buffer[receive_buffer_size - 2] | (receive_buffer[receive_buffer_size - 1] << 8);
            /* Calculate CRC of the data */
            uint16_t crc = getCRC16(receive_buffer, receive_buffer_size - 2);
            /* Check if the CRC is valid */
            if (crc != raw_crc)
            {
                /* CRC is invalid, clear the serial buffer */
                tcflush(uart, TCIFLUSH);
                continue;
            }
            /* if CRC is valid, publish the data */
            /* ESP telemetry packet unpacking */
            TelemetryPacket::esp_data_packet_t esp_data_packet;
            memcpy(&esp_data_packet.data, &receive_buffer, sizeof(esp_data_packet.data));
            TelemetryPacket::TelemetryPacket src;
            TelemetryPacket::DecodeESPTelemetryPacket(esp_data_packet, src);
            Vec3f attYPR = Rotationf::FromVectorPartOfQuaternion(Vec3f(src.attitude[0], src.attitude[1], src.attitude[2])).ToEulerYPR();
            for (int i = 0; i < 3; i++)
            {
                tel_msg.accelerometer[i] = src.accel[i];
                tel_msg.rateGyro[i] = src.gyro[i];
                tel_msg.position[i] = src.position[i];
                tel_msg.velocity[i] = src.velocity[i];
                tel_msg.attitude[i] = src.attitude[i];
                tel_msg.attitudeYPR[i] = attYPR[i];
            }
            for (int i = 0; i < 4; i++)
            {
                tel_msg.motorForces[i] = src.motorForces[i];
                tel_msg.esc_rpm[i] = src.esc_rpm[i];
            }
            tel_msg.voltage = src.voltage;
            tel_msg.batteryVoltage = src.voltage;
            tel_msg.current = src.current;
            tel_msg.temperature = src.temperature;
            tel_msg.warnings = src.warnings;
            tel_msg.panicReason = src.panicReason;
            /* Convert status from uint8_t to 8 bools */
            for (int i = 0; i < 8; i++)
            {
                tel_msg.status_bits[i] = (src.status >> i) & 0x01;
            }
            for (int i = 0; i < TelemetryPacket::ESPPacketSize::DEBUG; i++)
            {
                tel_msg.esp_debug_vals[i] = src.esp_debug_vals[i];
            }
            /* Publish the ROS message */
            tel_msg.header.stamp = ros::Time::now();
            pubTelemetry.publish(tel_msg);
            /* Compute the radio rates */
            numTelemetryRx++;
            numTelemetryTx = int(tel_msg.esp_debug_vals[4]);
            numCmdsRx = int(tel_msg.esp_debug_vals[5]);
            time_now = ros::Time::now().toSec() * 1000;
        }

        if (tPrintTitles.GetSeconds_d() > dtPrintTitles)
        {
            tPrintTitles.Reset();
            tPrintStats.AdjustTimeBySeconds(100); // force print stats
            for (int i = 0; i < 20; i++)
            {
                printf("\n");
            }
            printf("\n  Counts are totals:\n");
            //       123 1234567890 1234567890 1234567890 1234567890
            printf("+---+----------+----------+----------+----------+\n");
            printf("|ID |cmd Tx rad|cmd Rx ESP|tel Rx    |tel Tx ESP|\n");
            printf("+---+----------+----------+----------+----------+\n");
            fflush(stdout);
        }

        if (tPrintStats.GetSeconds_d() > dtPrintStats)
        {
            tPrintStats.Reset();
            printf("\r");
            printf("|%*d", 3, vehicleId);
            printf("|%*d", 10, numCmdsTx);
            printf("|%*d", 10, numCmdsRx);
            printf("|%*d", 10, numTelemetryRx);
            printf("|%*d", 10, numTelemetryTx);
            printf("|");

            fflush(stdout);
        }
    }

    /* Close UART upon exit */
    closeuart(uart);
    return 0;
}

/* 16-Bit CRC */
uint16_t updateCRC16(uint16_t crc, uint8_t crc_seed)
{
    uint16_t crc_u, i;
    crc_u = crc;
    crc_u ^= crc_seed;
    for (i = 0; i < 8; i++)
        crc_u = (crc_u & 0x8000) ? 0xa001 ^ (crc_u << 1) : (crc_u << 1);
    return (crc_u);
}

uint16_t getCRC16(uint8_t *buf, uint16_t buf_len)
{
    uint16_t crc = 0, i;
    for (i = 0; i < buf_len; i++)
        crc = updateCRC16(crc, buf[i]);
    return (crc);
}