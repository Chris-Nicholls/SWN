// Stub UserData for SWN Integration
// Removes dependencies on STM32F3 specific headers.

#ifndef PLAITS_USER_DATA_H_
#define PLAITS_USER_DATA_H_

#include "stmlib/stmlib.h"

namespace plaits {

class UserData {
 public:
  enum {
    ADDRESS = 0x00000000, // Dummy
    SIZE = 0x1000
  };

  UserData() { }
  ~UserData() { }

  inline const uint8_t* ptr(int slot) const {
    // Return NULL to indicate no user data (e.g. no DX7 patches loaded from Plaits flash)
    // SWN could potentially provide this buffer from its own flash if needed later.
    return NULL;
  }
  
  inline bool Save(uint8_t* rx_buffer, int slot) {
    // Do nothing. SWN handles saving via PresetManager.
    return true;
  }
};

}  // namespace plaits

#endif  // PLAITS_USER_DATA_H_
