

#include "homere_bbb/sabertooth.h"


SaberTooth::SaberTooth():
  serial_("/dev/ttyS1", 9600) {
  
}


bool SaberTooth::init() {
  
  return true;
}

SaberTooth::~SaberTooth() {

}
