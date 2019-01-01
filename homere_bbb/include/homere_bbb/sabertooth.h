#ifndef HOMERE_BBB__SABERTOOTH_H
#define HOMERE_BBB__SABERTOOTH_H

#include <async_comm/serial.h>

class SaberTooth {
 public:
  SaberTooth();
  ~SaberTooth();
  bool init();
 private:
  async_comm::Serial serial_;
};

#endif // HOMERE_BBB__SABERTOOTH_H
