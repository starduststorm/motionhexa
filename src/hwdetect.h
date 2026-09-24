#pragma once

// Runtime board-revision tell for revisions

#include <Arduino.h>
#include <hardware/flash.h>
#include <hardware/sync.h>

struct FlashJedecId {
  uint8_t manufacturer = 0;
  uint8_t type = 0;
  uint8_t capacity = 0;
};
inline FlashJedecId flashJedecId;
inline bool v8Hardware = false;

inline void detectHardwareRevision() {
  // v7 has an external Zetta ZD25WQ16C (JEDEC manufacturer 0xBA), v8 is an RP2354A whose in-package die is a Winbond W25Q16JV (0xEF 0x40 0x15)
  uint8_t txbuf[4] = {0x9f, 0, 0, 0}; // JEDEC ID: 1 command byte, 3 response bytes
  uint8_t rxbuf[4] = {0};
  uint32_t irq = save_and_disable_interrupts();
  flash_do_cmd(txbuf, rxbuf, sizeof(txbuf));
  restore_interrupts(irq);
  flashJedecId = {rxbuf[1], rxbuf[2], rxbuf[3]};
#if HARDWARE_VERSION >= 7
  v8Hardware = (flashJedecId.manufacturer == 0xEF && flashJedecId.type == 0x40 && flashJedecId.capacity == 0x15);
#endif
}
