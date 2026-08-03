#include "mbed.h"

I2C i2c(PB_3, PB_10);
Serial pc(USBTX, USBRX, 115200);
DigitalIn sw(PC_4);                  // ADD

const int K1 = 0x3A << 1;
const int K2 = 0x3C << 1;

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
    sw.mode(PullUp);                 // ADD
    wait_ms(500);
    pc.printf("\r\n=== Dual Knob Rotation Counter ===\r\n");

    int16_t prev1 = INT16_MIN, prev2 = INT16_MIN;
    int16_t offset1 = 0, offset2 = 0;    // ADD
    bool prev_sw = 1;                     // ADD

    while (true) {
        int16_t c1 = 0, c2 = 0;
        bool b1 = false, b2 = false;

        // ADD: zero on switch press
        bool cur_sw = sw.read();
        if (prev_sw == 1 && cur_sw == 0) {
            readKnob(K1, c1, b1);
            readKnob(K2, c2, b2);
            offset1 = c1;
            offset2 = c2;
            prev1 = INT16_MIN;
            prev2 = INT16_MIN;
            pc.printf("ZEROED\r\n");
        }
        prev_sw = cur_sw;

        if (readKnob(K1, c1, b1) && c1 - offset1 != prev1) {
            pc.printf("Knob1: %d\r\n", (int)(c1 - offset1));
            prev1 = c1 - offset1;
        }

        if (readKnob(K2, c2, b2) && c2 - offset2 != prev2) {
            pc.printf("Knob2: %d\r\n", (int)(c2 - offset2));
            prev2 = c2 - offset2;
        }

        wait_ms(10);
    }
}
