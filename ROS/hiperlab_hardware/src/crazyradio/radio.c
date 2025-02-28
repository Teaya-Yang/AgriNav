#include <libusb.h>

#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include <time.h>
#include <inttypes.h>

#include "cfradio.h"

#define TYPE_SETTER 0x40
#define TYPE_GETTER 0xC0

#define SET_RADIO_CHANNEL 0x01
#define SET_RADIO_ADDRESS 0x02
#define SET_DATA_RATE 0x03
#define SET_RADIO_POWER 0x04
#define SET_RADIO_ARD 0x05
#define SET_RADIO_ARC 0x06
#define ACK_ENABLE 0x10
#define SET_CONT_CARRIER 0x20
#define START_SCAN_CHANNELS 0x21
#define GET_SCAN_CHANNELS 0x21
#define LAUNCH_BOOTLOADER 0xFF

int set_radio_channel(libusb_device_handle *handle, uint8_t channel);
int set_radio_address(libusb_device_handle *handle, uint64_t addr);
int set_data_rate(libusb_device_handle *handle, uint8_t rate);
int set_radio_power(libusb_device_handle *handle, int level);
int set_radio_ard(libusb_device_handle *handle, uint8_t length);
int set_radio_arc(libusb_device_handle *handle, uint8_t nretries);
int set_ack_enable(libusb_device_handle *handle, bool enabled);
int set_cont_carrier(libusb_device_handle *handle, bool active);

void transfer_callback(struct libusb_transfer *transfer);

int submit_transfer(cfradio *radio, int dir);
int submit_null(cfradio *radio);

int cfradio_open(cfradio *radio, libusb_context *ctx, cfradio_fetcher fetcher,
                 cfradio_handler handler, int vehId) {
  radio->fetcher = fetcher;
  radio->handler = handler;
  radio->state = CFRADIO_STATE_IDLE;

  radio->connected = 0;
  radio->success_count = 0;
  radio->fail_count = 0;

  libusb_device **list = NULL;

  //find relevant radios:
  unsigned count = libusb_get_device_list(ctx, &list);
  unsigned numFound = 0;
  bool success = 0;
  for (unsigned idx = 0; idx < count; ++idx) {
    libusb_device *device = list[idx];
    struct libusb_device_descriptor desc = { 0 };

    int rc = libusb_get_device_descriptor(device, &desc);
    if (rc != 0) {
      printf("idx = %d, rc = %d\n", (int) idx, (int) rc);
    }

    if (!(desc.idVendor == 0x1915 && desc.idProduct == 0x7777)) {
      //not a crazyradio
      continue;
    }

    //crazyradio!
    numFound++;

    printf("Found crazyradio %d: ", (int) idx);

    //try to open the radio:
    int res = libusb_open(device, &radio->handle);
    if (res != 0) {
      printf(" libusb_open = %d\n", res);
      continue;
    }

    if (radio->handle == NULL) {
      printf("handle = 0\n");
      continue;
    }

    int err = libusb_set_configuration(radio->handle, 1);
    if (err != 0) {
      printf("Failed to set usb configuration: err = %d\n", err);
      continue;
    }

    struct libusb_config_descriptor config;

    libusb_get_descriptor(radio->handle, LIBUSB_DT_CONFIG, 0,
                          (unsigned char *) &config, sizeof(config));

    err = libusb_claim_interface(radio->handle, 0);
    if (err != 0) {
      printf("Couldn't claim interface = %d\n", err);
      continue;
    }

    printf("Success!\n");
    success = 1;
    break;
  }

  if (!success) {
    if (numFound) {
      printf("Found %d radios, but couldn't open them.\n", (int) numFound);
      return -1;
    }

    printf("Found no radios.");
    return -1;
  }

  // Configuring radio
  int res = 0;

  //TODO: mwm -- this seems to follow a different convention than the onboard,
  //  specifically, the order of the bytes are flipped. Not sure why this happens.
  // Also, it seems that we cannot set an ID to simply 0, so we add E7 to the second field.
  // This has to be mirrorred in the PC-side radio app
  uint64_t radioAddress = 0;
  uint8_t channel = 0;
  if (vehId > 0 && vehId <= 255) {
    radioAddress = vehId;
    radioAddress += 256 * 0xE7;

    // Space radios chanSpacing channels apart based on vehicle ID to prevent radio interference between crazyflies
    // Code results in the following pattern: [convention: channel(vehId)]
    // 5(1), 10(2), ..., 115(23), 120(24), 2(25), 7(26), ..., 117(48), 122(49), 4(50), 9(51), ...
    // Note: Edit code in syslink_main.cpp to ensure compatibility if changes are desired
    const unsigned int NUM_CHANNELS = 125;  // Documentation: https://wiki.bitcraze.io/doc:crazyradio:usb:index#set_radio_channel
    unsigned int chanSpacing = 5;  // Tested to work well
    unsigned int chanOffset = 2 * ((vehId * chanSpacing) / NUM_CHANNELS);  // Offset channels once the channel range has been filled
    channel = (vehId * chanSpacing + chanOffset) % NUM_CHANNELS;
  } else {
    printf("Illegal vehicle ID!\n");
    return -1;
  }
  /*
   printf("Vehicle id = <%d>\n", vehId);
   printf("Radio address, byte-wise: ");
   uint8_t const *tmp = (uint8_t*) (&radioAddress);
   for(int i=7; i>=0; i--){
   printf("%0X,", (int)tmp[i]);
   }
   printf("\n");
   */

  res |= set_radio_channel(radio->handle, channel);
  res |= set_radio_address(radio->handle, radioAddress);
  res |= set_data_rate(radio->handle, 2);
  res |= set_cont_carrier(radio->handle, false);
  res |= set_radio_power(radio->handle, 3);  // 0dBm
  res |= set_radio_ard(radio->handle, 32);
  res |= set_radio_arc(radio->handle, 3);
  res |= set_ack_enable(radio->handle, true);  // Set to false for single directional transfers

  if (res != 0) {
    printf("Configuration failed\n");
    return 1;
  }

  // Allocate a transfer for general use
  radio->transfer = libusb_alloc_transfer(0);

  return 0;

}

int cfradio_close(cfradio *radio) {
  if (radio->state != CFRADIO_STATE_IDLE) {
    libusb_cancel_transfer(radio->transfer);
  }

  libusb_free_transfer(radio->transfer);

  libusb_release_interface(radio->handle, 0);

  //libusb_close(radio->handle);
  return 0;
}

uint64_t get_posix_clock_time_us() {
  struct timespec ts;
  uint64_t current;
  if (clock_gettime(CLOCK_MONOTONIC, &ts) == 0) {
    current = (uint64_t) (ts.tv_sec * 1000000 + ts.tv_nsec / 1000);
    return current;
  } else {
    return 0;
  }
}

//(hack) null sending waits for some amount of us, then submits a null again.
// mu   | freq ranging  | freq tel
//------+---------------+-----------
// 1400 | ~ 85 Hz       | 120
// 1450 | 113           | 20
// 1500 | 113           | 20
// 1700 | 120           | 0
// 2000 | 118 Hz        | 0
// 2800 | 120 Hz        | 0
uint64_t CUTOFFTIME_us = (uint64_t) (1200);
int firstTime = 1;
static uint64_t timeSinceLastNull = 0;
int cfradio_notify(cfradio *radio) {

  if (radio->state != CFRADIO_STATE_IDLE) {
    return 0;
  }

  if (radio->fetcher(&radio->outbuf)) {
    //printf("send %d\n", radio->outbuf.size);
    //mwm: this actually sends out a msg over the radio
    submit_transfer(radio, LIBUSB_ENDPOINT_OUT);
  } else {
    uint64_t tNow;
    tNow = get_posix_clock_time_us();
    if (firstTime) {
      timeSinceLastNull = tNow;
      firstTime = 0;
    }
    // throttle the null sending to decrease uwb ranging interference.
    if (tNow > timeSinceLastNull + CUTOFFTIME_us) {
      timeSinceLastNull = tNow;
      submit_null(radio);
    }
  }

// TODO: Submit null otherwise

  /* if we can fetch a message send it, otherwise send a null packet */

  return 0;

}

void transfer_callback(struct libusb_transfer *transfer) {

  cfradio *radio = (cfradio *) transfer->user_data;

  switch (transfer->status) {
    case LIBUSB_TRANSFER_COMPLETED:
      if (radio->state == CFRADIO_STATE_SENDING) {
        submit_transfer(radio, LIBUSB_ENDPOINT_IN);
        radio->state = CFRADIO_STATE_RECEIVING;
      } else if (radio->state == CFRADIO_STATE_RECEIVING) {
        radio->state = CFRADIO_STATE_IDLE;

        // Check status
        int ackReceived = radio->inbuf.status & 1;

        // Monitor connection status
        if (ackReceived) {
          radio->success_count++;
          radio->fail_count = 0;
          if (radio->success_count > 20 && !radio->connected) {
            radio->connected = true;
            printf("Connected!\n");
          }
        } else {
          radio->success_count = 0;
          radio->fail_count++;
          if (radio->fail_count > 20 && radio->connected) {
            radio->connected = false;
            printf("Lost connection!\n");
          }
        }

        if (transfer->actual_length > 1) {
          crtp_message_t cmsg;
          cmsg.size = transfer->actual_length - 1;
          memcpy(&cmsg.header, radio->inbuf.data, cmsg.size);

          radio->handler(ackReceived, &cmsg);

          // We got non-empty packets, so trigger more to be sent
        }

        // Trigger a new transfer if needed

        //printf("usb done %d %d\n", transfer->actual_length, radio->inbuf.status & 1);
      }

      break;

    default:
      radio->state = CFRADIO_STATE_IDLE;
      printf("error in transfer: %s\n", libusb_error_name(transfer->status));
  }

  cfradio_notify(radio);
}

// LIBUSB_ENDPOINT_IN    LIBUSB_ENDPOINT_OUT
int submit_transfer(cfradio *radio, int dir) {

  void *buffer;
  int size;

  if (dir == LIBUSB_ENDPOINT_OUT) {
    size = radio->outbuf.size;
    buffer = &radio->outbuf.header;
    radio->state = CFRADIO_STATE_SENDING;
  } else if (dir == LIBUSB_ENDPOINT_IN) {
    size = 32 + 1;  // MTU + 1 byte usb status header
    buffer = &radio->inbuf;
    radio->state = CFRADIO_STATE_RECEIVING;
  }

  libusb_fill_bulk_transfer(radio->transfer, radio->handle, 1 | dir, buffer,
                            size, transfer_callback, radio, 1000);

  // TODO: This can be error checked
  return libusb_submit_transfer(radio->transfer);
}

// Used when there is no data that needs to be sent
int submit_null(cfradio *radio) {
  radio->outbuf.size = 1;
  radio->outbuf.header = 0xff;
  return submit_transfer(radio, LIBUSB_ENDPOINT_OUT);
}

// should be between 0 and  125
int set_radio_channel(libusb_device_handle *handle, uint8_t channel) {
  return libusb_control_transfer(handle, TYPE_SETTER, SET_RADIO_CHANNEL,
                                 channel, 0, NULL, 0, 0) < 0;
}

int set_radio_address(libusb_device_handle *handle, uint64_t addr) {
  return libusb_control_transfer(handle, TYPE_SETTER, SET_RADIO_ADDRESS, 0, 0,
                                 (unsigned char *) &addr, 5, 0) < 0;
}

int set_data_rate(libusb_device_handle *handle, uint8_t rate) {
  return libusb_control_transfer(handle, TYPE_SETTER, SET_DATA_RATE, rate, 0,
  NULL,
                                 0, 0) < 0;
}

/*
 0	-18dBm
 1	-12dBm
 2	-6dBm
 3	0dBm
 */
int set_radio_power(libusb_device_handle *handle, int level) {
  return libusb_control_transfer(handle, TYPE_SETTER, SET_RADIO_POWER, level, 0,
  NULL,
                                 0, 0) < 0;
}

/*
 Value	ARD wait time
 0x00	250us
 0x01	500us
 …	…
 0x0F	4000us

 */
/*
 Value	ACK payload length
 0x80	0Byte
 0x81	1Byte
 …	…
 0xA0	32Bytes
 */
int set_radio_ard(libusb_device_handle *handle, uint8_t length) {
  return libusb_control_transfer(handle, TYPE_SETTER, SET_RADIO_ARD,
                                 length | 0x80, 0, NULL, 0, 0) < 0;
}

/* Number of retries (default to 3) */
int set_radio_arc(libusb_device_handle *handle, uint8_t nretries) {
  return libusb_control_transfer(handle, TYPE_SETTER, SET_RADIO_ARC, nretries,
                                 0,
                                 NULL,
                                 0, 0) < 0;
}

/*
 0	Auto ACK deactivated
 Not 0	Auto ACK enable (default)
 */
int set_ack_enable(libusb_device_handle *handle, bool enabled) {
  return libusb_control_transfer(handle, TYPE_SETTER, SET_RADIO_ARC,
                                 enabled ? 1 : 0, 0, NULL, 0, 0) < 0;
}

int set_cont_carrier(libusb_device_handle *handle, bool active) {
  return libusb_control_transfer(handle, TYPE_SETTER, SET_RADIO_ARC,
                                 active ? 1 : 0, 0, NULL, 0, 0) < 0;
}
