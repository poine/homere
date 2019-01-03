

#include "homere_bbb/sabertooth.h"

// stty -F /dev/ttyS5 sane

SaberTooth::SaberTooth():
  serial_("/dev/ttyS5", 9600),
  addr_(128) {
  
}

#define cmd_drive_forward_m1   0x00
#define cmd_drive_backward_m1  0x01
#define cmd_drive_forward_m2   0x04
#define cmd_drive_backward_m2  0x05
#define cmd_serial_timeout     0x0e
#define cmd_deadband           0x11

bool SaberTooth::init() {
  if (!serial_.init())
    {
      std::printf("Failed to initialize serial port\n");
      return false;
    }
  std::printf("initialized serial port\n");
  send(cmd_serial_timeout, 1);
  send(cmd_deadband, 1);
  return true;
}

void SaberTooth::send_drive(int m1, int m2) {
  if (m1 >= 0)
    send(cmd_drive_forward_m1, m1);
  else
    send(cmd_drive_backward_m1, -m1);
  if (m2 >= 0)
    send(cmd_drive_forward_m2, m2);
  else
    send(cmd_drive_backward_m2, -m2);
}


void SaberTooth::send(uint8_t cmd, uint8_t data) {
  uint8_t buf[4];
  buf[0] = addr_;
  buf[1] = cmd;
  buf[2] = data;
  buf[3] = (addr_ + cmd + data) & 0x7F;
  serial_.send_bytes(buf, 4);
  //std::printf("Sending %d %d %d %d\n", buf[0], buf[1], buf[2], buf[3]);
}

SaberTooth::~SaberTooth() {
  serial_.close();
}
