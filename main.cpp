// GIM8008 Motor Test - PB_8/PB_9 CAN Pins
// NUCLEO_F446RE | MCP2551 | CAN 1Mbps | Motor ID: 11
// Compile: mbed compile -m NUCLEO_F446RE -t ARMC6 --flash

#include "mbed.h"
#include <cstdlib>

#define MOTOR_ID      9
#define CAN_BAUD      1000000
#define REPLY_ID      (MOTOR_ID + 0x100)

// Hardware
DigitalOut led(LED1);
DigitalIn enable_btn(USER_BUTTON);
Serial pc(USBTX, USBRX, 115200);

// CAN Pins: PB_8 (RX) / PB_9 (TX) - More reliable on NUCLEO_F446RE
CAN can(PA_11, PA_12, 1000000);

// State
enum State { DISABLED = 0, ENABLED = 1 };
volatile State motor_state = DISABLED;
volatile bool got_reply = false;
CANMessage tx_msg, rx_msg;

// Pack MIT Command (8 bytes)
void send_cmd(float vel, float kp = 0, float kd = 1.0f) {
    if(vel < -60.0f) vel = -60.0f;
    if(vel > 60.0f) vel = 60.0f;
    
    int p_int = 1;
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
    
    tx_msg.id = MOTOR_ID;
    tx_msg.type = CANData;
    tx_msg.format = CANStandard;
    tx_msg.len = 8;
    
    can.write(tx_msg);
}

// Enter Motor Mode (0xFC)
void enter_mode() {
    tx_msg.id = MOTOR_ID;
    tx_msg.type = CANData;
    tx_msg.format = CANStandard;
    tx_msg.len = 8;
    for(int i = 0; i < 7; i++) tx_msg.data[i] = 0xFF;
    tx_msg.data[7] = 0xFC;
    can.write(tx_msg);
    wait_ms(100);
}

// Exit Motor Mode (0xFD)
void exit_mode() {
    tx_msg.id = MOTOR_ID;
    tx_msg.type = CANData;
    tx_msg.format = CANStandard;
    tx_msg.len = 8;
    for(int i = 0; i < 7; i++) tx_msg.data[i] = 0xFF;
    tx_msg.data[7] = 0xFD;
    can.write(tx_msg);
    wait_ms(100);
}

// CAN ISR
void on_rx() {
    if(can.read(rx_msg)) {
        if(rx_msg.id == REPLY_ID && rx_msg.len >= 6) {
            got_reply = true;
        }
    }
}

int main() {
    wait_ms(2000);
    
    pc.printf("\r\n=== GIM8008 Test ===\r\n");
    pc.printf("Motor ID: %d | CAN: PB_8(RX)/PB_9(TX) | 1Mbps\r\n", MOTOR_ID);
    pc.printf("Button: Enable/Disable | LED: State\r\n\r\n");
    
    // Init CAN
    int freq = can.frequency(CAN_BAUD);
    pc.printf("CAN Freq: %d Hz\r\n", freq);
    
    if(freq == 0) {
        pc.printf("ERROR: CAN init failed! Check MCP2551 wiring.\r\n");
        while(1) { led = 1; wait_ms(200); led = 0; wait_ms(200); }
    }
    
    can.attach(&on_rx, CAN::RxIrq);
    can.filter(REPLY_ID, 0x7FF, CANStandard, 0);
    
    pc.printf("State: DISABLED. Press button to enable.\r\n\r\n");
    
    bool btn_last = 1;
    float vel = 0;
    int dir = 1;
    
    while(1) {
        // Button press detection
        bool btn_now = enable_btn.read();
        if(btn_last == 1 && btn_now == 0) {
            wait_ms(50);
            if(enable_btn.read() == 0) {
                if(motor_state == DISABLED) {
                    motor_state = ENABLED;
                    enter_mode();
                    pc.printf("[ENABLED]\r\n");
                } else {
                    motor_state = DISABLED;
                    send_cmd(0, 0, 10.0f);
                    exit_mode();
                    pc.printf("[DISABLED]\r\n");
                }
                while(enable_btn.read() == 0) wait_ms(10);
            }
        }
        btn_last = btn_now;
        
        // LED state
        led = (motor_state == ENABLED) ? 1 : 0;
        
        // Run motor if enabled
        if(motor_state == ENABLED) {
            vel += dir * 0.2f;
            if(vel >= 3.0f) dir = -1;
            if(vel <= -3.0f) dir = 1;
            
            send_cmd(vel, 0, 1.0f);
            
            if(got_reply) {
                pc.printf("\rV: %5.2f | OK   ", vel);
                got_reply = false;
            }
            
            wait_ms(50);
        } else {
            send_cmd(0, 0, 1.0f);
            wait_ms(100);
        }
    }
}