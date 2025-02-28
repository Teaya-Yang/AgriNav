/* Telemetry packet header file
 * Author: Eugene Lo
 */
#pragma once

#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace TelemetryPacket {
/********************************
 ** GENERAL METHODS/DEFINITIONS **
 ********************************/
enum PacketType {
    PACKET_TYPE_QUAD_TELEMETRY_PT1 = 0,
    PACKET_TYPE_QUAD_TELEMETRY_PT2 = 1,
    PACKET_TYPE_QUAD_TELEMETRY_PT3 = 2,
    PACKET_TYPE_QUAD_TELEMETRY_PT4 = 3,
    PACKET_TYPE_GENERIC_FLOAT = 100
};

/*
 Make sure to change the ROS message file `telemetry.msg` in hiperlab_rostools
 to be compatible with the messages that are defined here.
 */
enum NumberOfPackets {
    NUM_PACKETS_OVER_TELEMETRY = 2,  // Number of packets to send over telemetry. Default: 2, Min: 2, Max: 4
};

// warnings that the vehicle can report. Each is a bit (8 options)
enum TelemetryWarnings {
    WARN_LOW_BATT = 0x01,           // battery has been seen to be low at least once
    WARN_CMD_RATE = 0x02,           // not receiving commands at the expected rate
    WARN_UWB_RESET = 0x04,          // recent onboard estimator reset
    WARN_ONBOARD_FREQ = 0x08,       // Cannot execute the main loop as fast as planned
    WARN_CMD_BATCH_DROP = 0x10,     // dropping packets in large batches
    WARN_RESERVED_6 = 0x20,         // for future use
    WARN_RESERVED_CUSTOM_1 = 0x40,  // Never assigned in the main branch, feel free to use for anything
    WARN_RESERVED_CUSTOM_2 = 0x80,  // Never assigned in the main branch, feel free to use for anything
};

/* Crazyflie telemetry packet */
struct data_packet_t {  // Used for sending the packet to the radio
    uint8_t type;
    uint8_t packetNumber;
    uint16_t data[14];
} __attribute__((packed));

/* PX4-ESP telemetry packet */
enum ESPPacketSize {
    TOTAL = 35,  // number of 16-bit values in the packet
    TOTAL_BYTES = TOTAL * 2,
    DEFINED = 27,  // number of 16-bit values that are defined in the packet, others are debug vals
    DEBUG = TOTAL - DEFINED,
};

struct esp_data_packet_t {
    uint16_t data[ESPPacketSize::TOTAL];
} __attribute__((packed));

/* Map x from [a,b] to [-1,1] */
inline float MapToOnesRange(float x, float a, float b) { return ((x - a) / (b - a)) * 2 - 1; }

/* Map x from [-1,1] to [a,b] */
inline float MapToAB(float x, float a, float b) { return ((x + 1) / 2) * (b - a) + a; }

/* Encode values from range [-1,1] into 2 bytes.
 Input t is mapped to 0 iff t is outside of [-1,1].

 Maximum value of 2 byte uint is 2^16 - 1 = 65535.
 To encode a float from [-1,1], we scale the range up to
 [-2^15-1, 2^15-1] and then offset by +2^15 (since uints are non-negative).
 */
inline uint16_t EncodeOnesRange(float t) {
    uint16_t out;
    if (t < -1 || t > 1) {
        out = 0;
    } else {
        out = 32768 + 32767 * t;
    }
    return out;
}

/* Decode above encoding */
inline float DecodeOnesRange(uint16_t t) {
    if (t == 0) {
        return NAN;
    }
    return (t - 32768) / float(32768);
}

/**********************
 ** TELEMETRY PACKETS **
 **********************/
/* Range limits for packets used in QuadcopterLogic.cpp
 These limits are needed to perform the encoding/decoding from floats to ints
 (see MapToOnesRange, EncodeOnesRange above)
 Limits were determined empirically. */
enum TelemetryRanges {
    TEL_RANGE_ACC_MAX = 30,
    TEL_RANGE_ACC_MIN = -TEL_RANGE_ACC_MAX,
    TEL_RANGE_GYRO_MAX = 35,
    TEL_RANGE_GYRO_MIN = -TEL_RANGE_GYRO_MAX,
    TEL_RANGE_FORCE_MAX = 10,
    TEL_RANGE_FORCE_MIN = 0,
    TEL_RANGE_BATTVOLTAGE_MAX = 15,
    TEL_RANGE_BATTVOLTAGE_MIN = 0,
    TEL_RANGE_POS_MAX = 500,
    TEL_RANGE_POS_MIN = -TEL_RANGE_POS_MAX,
    TEL_RANGE_VEL_MAX = 30,
    TEL_RANGE_VEL_MIN = -TEL_RANGE_VEL_MAX,
    TEL_RANGE_ATT_MAX = 1,
    TEL_RANGE_ATT_MIN = -TEL_RANGE_ATT_MAX,
    /* Generic float range */
    TEL_RANGE_GENERIC_MAX = 250,
    TEL_RANGE_GENERIC_MIN = -TEL_RANGE_GENERIC_MAX,
    /* PX4-ESP specific */
    TEL_RANGE_VOLTAGE_MAX = 20,
    TEL_RANGE_VOLTAGE_MIN = 0,
    TEL_RANGE_CURRENT_MAX = 50,
    TEL_RANGE_CURRENT_MIN = 0,
};

struct TelemetryPacket {
    enum {
        NUM_DEBUG_FLOATS = 6,
        NUM_FLOATS_PER_PACKET = 14,
    };
    // Header Info
    uint8_t type;
    uint8_t packetNumber;  // The ID of the packet, is shared amongst all sub-packets.
    /* seqNum = 0 -> packet includes accel, gyro */
    float accel[3];
    float gyro[3];
    float motorForces[4];
    float position[3];
    float battVoltage;
    /* seqNum = 1 -> packet includes position, attitude, velocity, panicReason */
    float velocity[3];
    float attitude[3];
    float debugVals[NUM_DEBUG_FLOATS];
    uint8_t panicReason;
    uint8_t warnings;
    /* seqNum = 2 -> use as desired */
    float customPacket1[NUM_FLOATS_PER_PACKET];
    /* seqNum = 3 -> use as desired */
    float customPacket2[NUM_FLOATS_PER_PACKET];
    /* PX4-ESP specific */
    float voltage;
    float current;
    uint8_t temperature;
    uint16_t esc_rpm[4];
    uint8_t status = 0;  // 8 bit for boolean status checking, e.g. bit 0: armed...
    float esp_debug_vals[ESPPacketSize::DEBUG];
};

/* PX4-ESP specific encoding and decoding */
inline void EncodeESPTelemetryPacket(TelemetryPacket const &src, esp_data_packet_t &out) {
    for (int i = 0; i < 3; i++) {
        out.data[i + 0] = EncodeOnesRange(MapToOnesRange(src.accel[i], TEL_RANGE_ACC_MIN, TEL_RANGE_ACC_MAX));
        out.data[i + 3] = EncodeOnesRange(MapToOnesRange(src.gyro[i], TEL_RANGE_GYRO_MIN, TEL_RANGE_GYRO_MAX));
        out.data[i + 6] = EncodeOnesRange(MapToOnesRange(src.position[i], TEL_RANGE_POS_MIN, TEL_RANGE_POS_MAX));
        out.data[i + 9] = EncodeOnesRange(MapToOnesRange(src.velocity[i], TEL_RANGE_VEL_MIN, TEL_RANGE_VEL_MAX));
        out.data[i + 12] = EncodeOnesRange(MapToOnesRange(src.attitude[i], TEL_RANGE_ATT_MIN, TEL_RANGE_ATT_MAX));
    }
    for (int i = 0; i < 4; i++) {
        out.data[i + 15] = EncodeOnesRange(MapToOnesRange(src.motorForces[i], TEL_RANGE_FORCE_MIN, TEL_RANGE_FORCE_MAX));
        out.data[i + 19] = src.esc_rpm[i];
    }
    out.data[23] = EncodeOnesRange(MapToOnesRange(src.voltage, TEL_RANGE_VOLTAGE_MIN, TEL_RANGE_VOLTAGE_MAX));
    out.data[24] = EncodeOnesRange(MapToOnesRange(src.current, TEL_RANGE_CURRENT_MIN, TEL_RANGE_CURRENT_MAX));
    /* Combine temperature and status into one uint16_t */
    uint16_t temp_status = (src.temperature << 8) | src.status;
    out.data[25] = temp_status;
    /* Combine panicReason and warnings into one uint16_t */
    uint16_t panic_warnings = (src.panicReason << 8) | src.warnings;
    out.data[26] = panic_warnings;
    /* Encode the debug values */
    for (int i = 0; i < ESPPacketSize::DEBUG; i++) {
        out.data[i + ESPPacketSize::DEFINED] = EncodeOnesRange(MapToOnesRange(src.esp_debug_vals[i], TEL_RANGE_GENERIC_MIN, TEL_RANGE_GENERIC_MAX));
    }
    return;
}
inline void DecodeESPTelemetryPacket(esp_data_packet_t const &in, TelemetryPacket &out) {
    for (int i = 0; i < 3; i++) {
        out.accel[i] = MapToAB(DecodeOnesRange(in.data[i + 0]), TEL_RANGE_ACC_MIN, TEL_RANGE_ACC_MAX);
        out.gyro[i] = MapToAB(DecodeOnesRange(in.data[i + 3]), TEL_RANGE_GYRO_MIN, TEL_RANGE_GYRO_MAX);
        out.position[i] = MapToAB(DecodeOnesRange(in.data[i + 6]), TEL_RANGE_POS_MIN, TEL_RANGE_POS_MAX);
        out.velocity[i] = MapToAB(DecodeOnesRange(in.data[i + 9]), TEL_RANGE_VEL_MIN, TEL_RANGE_VEL_MAX);
        out.attitude[i] = MapToAB(DecodeOnesRange(in.data[i + 12]), TEL_RANGE_ATT_MIN, TEL_RANGE_ATT_MAX);
    }
    for (int i = 0; i < 4; i++) {
        out.motorForces[i] = MapToAB(DecodeOnesRange(in.data[i + 15]), TEL_RANGE_FORCE_MIN, TEL_RANGE_FORCE_MAX);
        out.esc_rpm[i] = in.data[i + 19];
    }
    out.voltage = MapToAB(DecodeOnesRange(in.data[23]), TEL_RANGE_VOLTAGE_MIN, TEL_RANGE_VOLTAGE_MAX);
    out.current = MapToAB(DecodeOnesRange(in.data[24]), TEL_RANGE_CURRENT_MIN, TEL_RANGE_CURRENT_MAX);
    /* Decode the temperature and status */
    uint16_t temp_status = in.data[25];
    out.temperature = temp_status >> 8;
    out.status = temp_status & 0xFF;
    /* Decode the panicReason and warnings */
    uint16_t panic_warnings = in.data[26];
    out.panicReason = panic_warnings >> 8;
    out.warnings = panic_warnings & 0xFF;
    /* Decode the debug values */
    for (int i = 0; i < ESPPacketSize::DEBUG; i++) {
        out.esp_debug_vals[i] = MapToAB(DecodeOnesRange(in.data[i + ESPPacketSize::DEFINED]), TEL_RANGE_GENERIC_MIN, TEL_RANGE_GENERIC_MAX);
    }
    return;
}

/* Encode a TelemetryPacket into a data_packet_t */
inline void EncodeTelemetryPacket(TelemetryPacket const &src, data_packet_t &out) {
    out.type = src.type;
    out.packetNumber = src.packetNumber;
    if (out.type == PACKET_TYPE_QUAD_TELEMETRY_PT1) {
        for (int i = 0; i < 3; i++) {
            out.data[i + 0] = EncodeOnesRange(MapToOnesRange(src.accel[i], TEL_RANGE_ACC_MIN, TEL_RANGE_ACC_MAX));
            out.data[i + 3] = EncodeOnesRange(MapToOnesRange(src.gyro[i], TEL_RANGE_GYRO_MIN, TEL_RANGE_GYRO_MAX));
        }
        for (int i = 0; i < 4; i++) {
            out.data[i + 6] = EncodeOnesRange(MapToOnesRange(src.motorForces[i], TEL_RANGE_FORCE_MIN, TEL_RANGE_FORCE_MAX));
        }
        for (int i = 0; i < 3; i++) {
            out.data[i + 10] = EncodeOnesRange(MapToOnesRange(src.position[i], TEL_RANGE_POS_MIN, TEL_RANGE_POS_MAX));
        }
        out.data[13] = EncodeOnesRange(MapToOnesRange(src.battVoltage, TEL_RANGE_BATTVOLTAGE_MIN, TEL_RANGE_BATTVOLTAGE_MAX));
    } else if (out.type == PACKET_TYPE_QUAD_TELEMETRY_PT2) {
        for (int i = 0; i < 3; i++) {
            out.data[i + 0] = EncodeOnesRange(MapToOnesRange(src.velocity[i], TEL_RANGE_VEL_MIN, TEL_RANGE_VEL_MAX));
            out.data[i + 3] = EncodeOnesRange(MapToOnesRange(src.attitude[i], TEL_RANGE_ATT_MIN, TEL_RANGE_ATT_MAX));
        }
        for (int i = 0; i < TelemetryPacket::NUM_DEBUG_FLOATS; i++) {
            out.data[i + 6] = EncodeOnesRange(MapToOnesRange(src.debugVals[i], TEL_RANGE_GENERIC_MIN, TEL_RANGE_GENERIC_MAX));
        }

        memcpy(&out.data[12], &src.panicReason, 1);
        memcpy(&out.data[13], &src.warnings, 1);
    } else if (out.type == PACKET_TYPE_QUAD_TELEMETRY_PT3) {
        for (int i = 0; i < TelemetryPacket::NUM_FLOATS_PER_PACKET; i++) {
            out.data[i] = EncodeOnesRange(MapToOnesRange(src.customPacket1[i], TEL_RANGE_GENERIC_MIN, TEL_RANGE_GENERIC_MAX));
        }
    } else if (out.type == PACKET_TYPE_QUAD_TELEMETRY_PT4) {
        for (int i = 0; i < TelemetryPacket::NUM_FLOATS_PER_PACKET; i++) {
            out.data[i] = EncodeOnesRange(MapToOnesRange(src.customPacket2[i], TEL_RANGE_GENERIC_MIN, TEL_RANGE_GENERIC_MAX));
        }
    }
    return;
}

/* Decode a data_packet_t into a TelemetryPacket */
inline void DecodeTelemetryPacket(data_packet_t const &in, TelemetryPacket &out) {
    out.type = in.type;
    out.packetNumber = in.packetNumber;
    if (in.type == PACKET_TYPE_QUAD_TELEMETRY_PT1) {
        for (int i = 0; i < 3; i++) {
            out.accel[i] = MapToAB(DecodeOnesRange(in.data[i + 0]), TEL_RANGE_ACC_MIN, TEL_RANGE_ACC_MAX);
            out.gyro[i] = MapToAB(DecodeOnesRange(in.data[i + 3]), TEL_RANGE_GYRO_MIN, TEL_RANGE_GYRO_MAX);
        }
        for (int i = 0; i < 4; i++) {
            out.motorForces[i] = MapToAB(DecodeOnesRange(in.data[i + 6]), TEL_RANGE_FORCE_MIN, TEL_RANGE_FORCE_MAX);
        }
        for (int i = 0; i < 3; i++) {
            out.position[i] = MapToAB(DecodeOnesRange(in.data[i + 10]), TEL_RANGE_POS_MIN, TEL_RANGE_POS_MAX);
        }
        out.battVoltage = MapToAB(DecodeOnesRange(in.data[13]), TEL_RANGE_BATTVOLTAGE_MIN, TEL_RANGE_BATTVOLTAGE_MAX);
    } else if (in.type == PACKET_TYPE_QUAD_TELEMETRY_PT2) {
        for (int i = 0; i < 3; i++) {
            out.velocity[i] = MapToAB(DecodeOnesRange(in.data[i + 0]), TEL_RANGE_VEL_MIN, TEL_RANGE_VEL_MAX);
            out.attitude[i] = MapToAB(DecodeOnesRange(in.data[i + 3]), TEL_RANGE_ATT_MIN, TEL_RANGE_ATT_MAX);
        }
        for (int i = 0; i < TelemetryPacket::NUM_DEBUG_FLOATS; i++) {
            out.debugVals[i] = MapToAB(DecodeOnesRange(in.data[i + 6]), TEL_RANGE_GENERIC_MIN, TEL_RANGE_GENERIC_MAX);
        }
        memcpy(&out.panicReason, &in.data[12], 1);
        memcpy(&out.warnings, &in.data[13], 1);
    } else if (in.type == PACKET_TYPE_QUAD_TELEMETRY_PT3) {
        for (int i = 0; i < TelemetryPacket::NUM_FLOATS_PER_PACKET; i++) {
            out.customPacket1[i] = MapToAB(DecodeOnesRange(in.data[i]), TEL_RANGE_GENERIC_MIN, TEL_RANGE_GENERIC_MAX);
        }
    } else if (in.type == PACKET_TYPE_QUAD_TELEMETRY_PT4) {
        for (int i = 0; i < TelemetryPacket::NUM_FLOATS_PER_PACKET; i++) {
            out.customPacket2[i] = MapToAB(DecodeOnesRange(in.data[i]), TEL_RANGE_GENERIC_MIN, TEL_RANGE_GENERIC_MAX);
        }
    }
    return;
}

/******************
 ** FLOAT PACKETS **
 ******************/
/* Reverts uints to floats in range [-1,1] */
inline void DecodeFloatPacket(data_packet_t const &packetIn, float out[], int const numFloats) {
    for (int i = 0; i < numFloats; i++) {
        if (i >= 14)
            break;
        out[i] = DecodeOnesRange(packetIn.data[i]);
    }
}

/* Assume floats in range [-1,1] */
inline void EncodeFloatPacket(data_packet_t &packetOut, float const floats[], int const numFloats) {
    packetOut.type = PACKET_TYPE_GENERIC_FLOAT;

    int i;
    for (i = 0; i < numFloats; i++) {
        if (i >= 14)
            break;
        packetOut.data[i] = EncodeOnesRange(floats[i]);
    }

    for (; i < 14; i++) {
        packetOut.data[i] = EncodeOnesRange(0);
    }
}

}  // namespace TelemetryPacket
