// GIM8008 Quad Motor Test - AUTO CYCLE 4 Directions (ID 11,12,13,14)
// NUCLEO_F446RE | MCP2551 | CAN 1Mbps
// Cycle: Uragsh(→) → Baruun(↓) → Hoish(←) → Zvvn(↑) → repeat

#include "mbed.h"
#include <cstdlib>

#define MOTOR_ID_1      9
#define MOTOR_ID_2      12
#define MOTOR_ID_3      13
#define MOTOR_ID_4      14
#define CAN_BAUD        1000000
#define REPLY_ID_1      (MOTOR_ID_1 + 0x100)
#define REPLY_ID_2      (MOTOR_ID_2 + 0x100)
#define REPLY_ID_3      (MOTOR_ID_3 + 0x100)
#define REPLY_ID_4      (MOTOR_ID_4 + 0x100)

// Hardware
DigitalOut led(LED1);
DigitalIn enable_btn(USER_BUTTON);
Serial pc(USBTX, USBRX, 115200);
CAN can(PA_11, PA_12, CAN_BAUD);

// Configuration
const float MAX_VEL = 3.0f;
const float RAMP_TIME = 5.0f;         // Time for ONE velocity ramp (0→max→0)
const float DIRECTION_HOLD_TIME = 5.0f; // Time to hold ONE direction pattern
const int NUM_MOTORS = 4;
const int NUM_PATTERNS = 4;

enum State { DISABLED = 0, ENABLED = 1 };
volatile State motor_state = DISABLED;
volatile bool got_reply[4] = {false, false, false, false};
volatile int rx_count[4] = {0, 0, 0, 0};

CANMessage tx_msg, rx_msg;

float velocities[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// === Direction patterns: +1=CCW, -1=CW ===
// Motor order: [11, 12, 13, 14]
const int directions[NUM_PATTERNS][NUM_MOTORS] = {
    {1, -1, -1, 1},   // Pattern 0: Uragsh (Forward)  ↑
    {1, 1, -1, -1},   // Pattern 1: Baruun  (Right)    →
    {-1, 1, 1, -1},   // Pattern 2: Hoish   (Backward) ↓
    {-1, -1, 1, 1}    // Pattern 3: Zvvn    (Left)     ←
};

const char* pattern_names[] = {"Uragsh", "Baruun", "Hoish", "Zvvn"};

void send_cmd(uint32_t id, float vel, float kp = 0, float kd = 1.0f) {
    if(vel < -30.0f) vel = -30.0f;
    if(vel > 30.0f) vel = 30.0f;
    
    int p_int = 2048;
    int v_int = (int)((vel + 30.0f) * 4095.0f / 60.0f);
    int kp_int = (int)(kp * 4095.0f / 500.0f);
    int kd_int = (int)(kd * 4095.0f / 5.0f);
    int ff_int = 2048;
    
    tx_msg.data[0] = (p_int >> 8) & 0xFF;
    tx_msg.data[1] = p_int & 0xFF;
    tx_msg.data[2] = (v_int >> 4) & 0xFF;
    tx_msg.data[3] = ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F);
    tx_msg.data[4] = kp_int & 0xFF;
    tx_msg.data[5] = (kd_int >> 4) & 0xFF;
    tx_msg.data[6] = ((kd_int & 0x0F) << 4) | ((ff_int >> 8) & 0x0F);
    tx_msg.data[7] = ff_int & 0xFF;
    
    tx_msg.id = id;
    tx_msg.type = CANData;
    tx_msg.format = CANStandard;
    tx_msg.len = 8;
    can.write(tx_msg);
}

void enter_mode(uint32_t id) {
    tx_msg.id = id; tx_msg.type = CANData; tx_msg.format = CANStandard; tx_msg.len = 8;
    for(int i = 0; i < 7; i++) tx_msg.data[i] = 0xFF;
    tx_msg.data[7] = 0xFC;
    can.write(tx_msg);
    wait_ms(50);
}

void exit_mode(uint32_t id) {
    tx_msg.id = id; tx_msg.type = CANData; tx_msg.format = CANStandard; tx_msg.len = 8;
    for(int i = 0; i < 7; i++) tx_msg.data[i] = 0xFF;
    tx_msg.data[7] = 0xFD;
    can.write(tx_msg);
    wait_ms(50);
}

void on_rx() {
    if(can.read(rx_msg)) {
        if(rx_msg.id == REPLY_ID_1 && rx_msg.len >= 6) { got_reply[0] = true; rx_count[0]++; }
        else if(rx_msg.id == REPLY_ID_2 && rx_msg.len >= 6) { got_reply[1] = true; rx_count[1]++; }
        else if(rx_msg.id == REPLY_ID_3 && rx_msg.len >= 6) { got_reply[2] = true; rx_count[2]++; }
        else if(rx_msg.id == REPLY_ID_4 && rx_msg.len >= 6) { got_reply[3] = true; rx_count[3]++; }
    }
}

int main() {
    wait_ms(2000);
    
    pc.printf("\r\n=== GIM8008 Quad Motor [AUTO CYCLE] ===\r\n");
    pc.printf("Motors: %d, %d, %d, %d\r\n", MOTOR_ID_1, MOTOR_ID_2, MOTOR_ID_3, MOTOR_ID_4);
    pc.printf("Patterns: Uragsh→Baruun→Hoish→Zvvn (auto-cycle)\r\n");
    pc.printf("Ramp: %.1fs per direction | Full cycle: %.1fs\r\n\r\n", 
              DIRECTION_HOLD_TIME, DIRECTION_HOLD_TIME * NUM_PATTERNS);
    
    int freq = can.frequency(CAN_BAUD);
    pc.printf("CAN Freq: %d Hz\r\n", freq);
    if(freq == 0) {
        pc.printf("ERROR: CAN init failed!\r\n");
        while(1) { led = !led; wait_ms(200); }
    }
    
    can.attach(&on_rx, CAN::RxIrq);
    can.filter(0x100, 0x700, CANStandard, 0);  // Wildcard filter
    
    pc.printf("State: DISABLED. Press button to start.\r\n\r\n");
    
    bool btn_last = 1;
    Timer cycle_timer, debug_timer, direction_timer;
    cycle_timer.start();
    debug_timer.start();
    direction_timer.start();
    
    int current_pattern = 0;  // Start with Uragsh (Forward)

    while(1) {
        // Button: Enable/Disable
        bool btn_now = enable_btn.read();
        if(btn_last == 1 && btn_now == 0) {
            wait_ms(50);
            if(enable_btn.read() == 0) {
                if(motor_state == DISABLED) {
                    motor_state = ENABLED;
                    enter_mode(MOTOR_ID_1); enter_mode(MOTOR_ID_2);
                    enter_mode(MOTOR_ID_3); enter_mode(MOTOR_ID_4);
                    cycle_timer.reset();
                    direction_timer.reset();
                    current_pattern = 0;
                    for(int i=0; i<4; i++) { got_reply[i]=false; rx_count[i]=0; }
                    pc.printf("[ENABLED] Starting pattern: %s\r\n", pattern_names[current_pattern]);
                } else {
                    motor_state = DISABLED;
                    for(int i=0; i<NUM_MOTORS; i++) send_cmd(MOTOR_ID_1+i, 0, 0, 10.0f);
                    wait_ms(200);
                    for(int i=0; i<NUM_MOTORS; i++) exit_mode(MOTOR_ID_1+i);
                    pc.printf("[DISABLED]\r\n");
                }
                while(enable_btn.read() == 0) wait_ms(10);
            }
        }
        btn_last = btn_now;
        led = (motor_state == ENABLED) ? 1 : 0;
        
        if(motor_state == ENABLED) {
            // === AUTO-CYCLE DIRECTION PATTERN ===
            if(direction_timer.read() >= DIRECTION_HOLD_TIME) {
                direction_timer.reset();
                current_pattern = (current_pattern + 1) % NUM_PATTERNS;
                pc.printf("\r\n>>> Pattern changed: %s <<<\r\n", pattern_names[current_pattern]);
            }
            
            // Velocity ramp: triangle wave 0→MAX→0 over RAMP_TIME
            float elapsed = cycle_timer.read();
            float phase_time = fmod(elapsed, RAMP_TIME);
            float target_mag = (phase_time < RAMP_TIME/2.0f) 
                ? (phase_time / (RAMP_TIME/2.0f)) * MAX_VEL
                : ((RAMP_TIME - phase_time) / (RAMP_TIME/2.0f)) * MAX_VEL;
            
            // === Apply CURRENT pattern's directions ===
            for(int i = 0; i < NUM_MOTORS; i++) {
                velocities[i] = target_mag * directions[current_pattern][i];
            }
            
            // Send commands with small gaps to reduce bus contention
            send_cmd(MOTOR_ID_1, velocities[0], 0, 1.0f); wait_us(100);
            send_cmd(MOTOR_ID_2, velocities[1], 0, 1.0f); wait_us(100);
            send_cmd(MOTOR_ID_3, velocities[2], 0, 1.0f); wait_us(100);
            send_cmd(MOTOR_ID_4, velocities[3], 0, 1.0f); wait_us(100);
            
            // Debug: Print status every 1 second
            if(debug_timer.read() > 1.0f) {
                debug_timer.reset();
                pc.printf("\r[%s] V:%+4.2f | RX: M1:%3d M2:%3d M3:%3d M4:%3d", 
                          pattern_names[current_pattern], target_mag,
                          rx_count[0], rx_count[1], rx_count[2], rx_count[3]);
            }
            
            // Reset reply flags (keep counters for debug)
            for(int i=0; i<4; i++) got_reply[i] = false;
            
            wait_ms(20); // ~50Hz control loop
            
        } else {
            // Disabled: keep motors stopped
            for(int i=0; i<NUM_MOTORS; i++) send_cmd(MOTOR_ID_1+i, 0, 0, 1.0f);
            wait_ms(100);
        }
    }
}
