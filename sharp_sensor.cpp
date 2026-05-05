#include "mbed.h"

#define SHARP_PIN   PA_0
#define VCC_REF     3.3f
#define FILTER_SIZE 5

AnalogIn  sharp(SHARP_PIN);
Serial    pc(USBTX, USBRX, 115200);
DigitalOut led(LED1);

float filter_buf[FILTER_SIZE] = {0.0f};
int filter_idx = 0;

// ================= ACCURATE DISTANCE (LUT METHOD) =================
float get_distance() {
    float voltage = sharp.read() * VCC_REF;

    // Sharp GP2Y0A21YK0F Voltage → Distance Table (5V Sensor / 3.3V ADC)
    static const float lut_v[] = {0.35, 0.45, 0.55, 0.70, 0.90, 1.10, 1.35, 1.65, 2.00, 2.40, 2.75};
    static const float lut_d[] = {80.0, 50.0, 35.0, 25.0, 18.0, 14.0, 11.0, 8.5, 7.0, 5.5, 4.0};
    const int lut_len = 11;

    // Out of range clamping
    if (voltage <= lut_v[0]) return lut_d[0];
    if (voltage >= lut_v[lut_len-1]) return lut_d[lut_len-1];

    // Linear interpolation between table points
    for(int i = 0; i < lut_len - 1; i++) {
        if (voltage >= lut_v[i] && voltage < lut_v[i+1]) {
            float ratio = (voltage - lut_v[i]) / (lut_v[i+1] - lut_v[i]);
            return lut_d[i] + ratio * (lut_d[i+1] - lut_d[i]);
        }
    }
    return 80.0f; // Fallback
}

float get_filtered_distance() {
    filter_buf[filter_idx] = get_distance();
    filter_idx = (filter_idx + 1) % FILTER_SIZE;

    float sum = 0.0f;
    for(int i = 0; i < FILTER_SIZE; i++) sum += filter_buf[i];
    return sum / FILTER_SIZE;
}

// ================= MAIN =================
int main() {
    pc.printf("=== SHARP LUT DISTANCE MONITOR ===\r\n");
    pc.printf("Move hand 5cm -> 30cm\r\n\r\n");

    bool is_detected = false;

    while(1) {
        float dist = get_filtered_distance();

        // Hysteresis for stable detection
        if(dist < 18.0f) is_detected = true;
        if(dist > 22.0f) is_detected = false;

        pc.printf("Dist: %5.1f cm | %s\r\n", dist, is_detected ? "DETECT" : "CLEAR");
        led = is_detected ? 1 : 0;

        thread_sleep_for(200);
    }
}
