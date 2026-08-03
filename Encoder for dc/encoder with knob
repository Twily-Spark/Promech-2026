#include "mbed.h"

I2C i2c(PB_3, PB_10);
Serial pc(USBTX, USBRX, 115200);

const int K1 = 0x3A << 1;  // 0x74
const int K2 = 0x3C << 1;  // 0x78

bool readKnob(int addr, int16_t &count, bool &pressed) {
    char buf[6] = {0};
    if (i2c.read(addr, buf, 6) != 0)
        return false;
    count   = (int16_t)((uint8_t)buf[1] | (uint8_t)buf[2] << 8);
    pressed = (buf[4] == 0x00);
    return true;
}

int main() {
    i2c.frequency(50000);
    wait_ms(500);
    pc.printf("\r\n=== Dual Knob Rotation Counter ===\r\n");

    int16_t prev1 = INT16_MIN, prev2 = INT16_MIN;

    while (true) {
        int16_t c1 = 0, c2 = 0;
        bool b1 = false, b2 = false;

        if (readKnob(K1, c1, b1) && c1 != prev1) {
            pc.printf("Knob1: %d\r\n", (int)c1);
            prev1 = c1;
        }

        if (readKnob(K2, c2, b2) && c2 != prev2) {
            pc.printf("Knob2: %d\r\n", (int)c2);
            prev2 = c2;
        }

        wait_ms(10);
    }
}
