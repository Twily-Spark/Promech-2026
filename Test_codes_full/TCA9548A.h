#ifndef TCA9548A_H
#define TCA9548A_H

#include "mbed.h"

class TCA9548A {
public:
    TCA9548A(I2C &i2c, uint8_t addr_7bit = 0x70)
        : _i2c(i2c), _addr(addr_7bit << 1) {}

    bool select_channel(uint8_t ch) {
        if (ch > 7) return false;
        char data = 1 << ch;
        return (_i2c.write(_addr, &data, 1) == 0);
    }

    bool disable_all() {
        char data = 0x00;
        return (_i2c.write(_addr, &data, 1) == 0);
    }

private:
    I2C &_i2c;
    int _addr;   // 8-bit address for mbed I2C
};

#endif