#ifndef HOMERE_BBB__SABERTOOTH_H
#define HOMERE_BBB__SABERTOOTH_H

#include <async_comm/serial.h>

class SaberTooth {
 public:
  SaberTooth();
  ~SaberTooth();
  bool init();
  void send_drive(int m1, int m2);
  void send(uint8_t cmd, uint8_t data);
 private:
  async_comm::Serial serial_;
  int addr_;
  
};

#endif // HOMERE_BBB__SABERTOOTH_H
