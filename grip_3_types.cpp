// GIM8008 Quad Motor + Cylinders - Custom Sequence with Working CAN
#include "mbed.h"
#include <cstdlib>
#include <cmath>
#include "stm32f4xx_hal.h"

// ================= MOTOR & CAN CONFIG =================
#define MOTOR_ID_1      11
#define MOTOR_ID_2      12
#define MOTOR_ID_3      13
#define MOTOR_ID_4      14
#define CAN_BAUD        1000000
#define REPLY_ID_1      (MOTOR_ID_1 + 0x100)
#define REPLY_ID_2      (MOTOR_ID_2 + 0x100)
#define REPLY_ID_3      (MOTOR_ID_3 + 0x100)
#define REPLY_ID_4      (MOTOR_ID_4 + 0x100)

DigitalOut led(LED1);
Serial pc(USBTX, USBRX, 115200);
CAN can(PA_11, PA_12, CAN_BAUD);

// DC Motor Limit Sensors
DigitalIn lim_grip(PC_1, PullUp);        
DigitalIn lim_release(PC_4, PullUp);     

// LCD Config
#define I2C_SDA PB_9
#define I2C_SCL PB_8
I2C i2c_lcd(I2C_SDA, I2C_SCL);
uint8_t lcd_addr = 0x27; 
#define LCD_BACKLIGHT 0x08
#define LCD_EN 0x04
#define LCD_RS 0x01
#define LCD_ADDR_1 0x27
#define LCD_ADDR_2 0x3F

// ============================================================================
// MOTOR 1: GRIP / BOX HANDLING (VNH5019)
// ============================================================================
PwmOut     m1_pwm(PB_14);
DigitalOut m1_dira(PC_8);
DigitalOut m1_dirb(PC_6);

// ============================================================================
// CLIMBING MOTORS (M3, M4)
// ============================================================================
DigitalOut m3_ina(PA_8); 
DigitalOut m3_inb(PB_10);
PwmOut     m3_pwm(PA_9); 

DigitalOut m4_ina(PA_13);
DigitalOut m4_inb(PA_14);
PwmOut     m4_pwm(PB_2); 

// --- CONFIGURATION ---
const float MAX_VEL = 3.0f;         
const float LINEAR_SPEED = MAX_VEL;
const float DISTANCE_1 = 5.0f;   
const float DISTANCE_2 = 3.0f;   
const float DISTANCE_3 = 10.0f;  

const float TIME_1 = DISTANCE_1 / LINEAR_SPEED; 
const float TIME_2 = DISTANCE_2 / LINEAR_SPEED; 
const float TIME_3 = DISTANCE_3 / LINEAR_SPEED; 
const float TIME_4 = TIME_2 / 4;

// Ladder Configuration
const float LADDER_SPEED = 0.5f;       
const float LADDER_TIME_20CM = 4.0f;
const float STABILIZE_TIME = 4.0f;
const float STABILIZE_VEL = 5.0f;   
const float STABILIZE_BACK = 0.1f;   

// Directions for M3/M4
const int DIR_UP_M3    = -1; 
const int DIR_UP_M4    = 1;  
const int DIR_DOWN_M3  = 1;  
const int DIR_DOWN_M4  = -1; 

const int NUM_MOTORS = 4;
const int NUM_PATTERNS = 6; 

// ================= STATE ENUM =================
enum State { 
    DISABLED = 0, 
    MOVE_BACK_1 = 1, 
    CYL1_CYCLE = 2,      
    MOVE_BACK_2 = 3, 
    CYL2_CYCLE = 4,      
    MOVE_FOR_1 = 5,          
    MOVE_DC_TO_RELEASE = 6,  
    TURN_TO_LADDER = 7,      
    FIRST_LADDER = 8, 
    STATE_1 = 9,
    STATE_2 = 10,       
    FINISHED = 11
};

// ========= GLOBAL VARIABLES =========
volatile State current_state = DISABLED;
volatile int rx_count[4] = {0, 0, 0, 0};
volatile int current_direction_pattern = 3;
volatile bool button_mode_active = false;

Timer state_timer, debug_timer, loop_timer;
State last_state = DISABLED;
int ladder_sub_step = 0;
int btn1_last = 1;
int btn2_last = 1;

// Separate step counters for STATE_1 and STATE_2
volatile int state_1_step = 0;
volatile int state_2_step = 0;

CANMessage tx_msg, rx_msg;
float velocities[4] = {0.0f, 0.0f, 0.0f, 0.0f};

const float directions[NUM_PATTERNS][NUM_MOTORS] = { 
    {1.0f, -1.0f, -1.0f, 1.0f},   // 0: Forward
    {1.0f, 1.0f, -1.0f, -1.0f},   // 1: Hoish (strafe)
    {-1.0f, 1.0f, 1.0f, -1.0f},   // 2: Reverse-strafe
    {-1.0f, -1.0f, 1.0f, 1.0f},   // 3: Uragsh (backward)
    {0.0f, 1.0f, 0.0f, -1.0f},    // 4: Diagonal
    {-1.0f, -1.0f, -1.0f, -1.0f}  // 5: Turn Arc
};

// ================= SENSORS =================
DigitalIn sens_start(PC_0, PullUp);       
DigitalIn sens_motor_start(PA_5, PullUp); 
DigitalIn button_1(PB_3, PullUp);
DigitalIn button_2(PA_10, PullUp);
DigitalIn sens_motor_stop(PB_0, PullUp);
DigitalIn box_sensor_1(PB_1, PullUp);
DigitalIn box_sensor_2(PA_4, PullUp);
DigitalIn box_sensor_3(PA_15, PullUp);

// Cylinders - STAY ON for gripping (only OFF on full reset)
DigitalOut cyl3(PC_7);   
DigitalOut cyl4(PB_6);
DigitalOut cyl1(PA_7);
DigitalOut cyl2(PA_6);
DigitalOut* relays[4] = {&cyl1, &cyl2, &cyl3, &cyl4};

#define RELAY_ON  1
#define RELAY_OFF 0

// ================= FUNCTION PROTOTYPES =================
void send_cmd(uint32_t id, float vel, float kp = 0, float kd = 1.0f);
void enter_mode(uint32_t id);
void safe_stop_all();
void on_rx();
void cyl_stop_all();
void stopAllClimb();
int read_active_low(DigitalIn& s);
int is_pressed(DigitalIn& s);
void set_motor(int dir, float duty);
void moveM3M4Up(float speed);
void moveM3Down(float speed);
void moveM4Down(float speed);
void moveCANForward(float duty);
void moveCANBackward(float duty);
void wait_us_lcd(int us);
void write_i2c(uint8_t data);
void pulse_enable(uint8_t data);
void send_nibble(uint8_t nibble, uint8_t mode);
void send_command(uint8_t cmd);
void send_char(char c);
void lcd_print(const char* str);
void lcd_print_num(float num, int decimals);
void lcd_clear();
void lcd_set_cursor(uint8_t row, uint8_t col);
bool detect_lcd();
void lcd_init();

// ================= INLINE HELPERS =================
inline int is_pressed(DigitalIn& s) { return (s.read() == 0) ? 1 : 0; }
inline int read_active_low(DigitalIn& s) { return (s.read() == 0) ? 1 : 0; }

// ================= CYLINDER & CLIMB FUNCTIONS =================
void cyl_stop_all() {
    cyl1.write(RELAY_OFF);
    cyl2.write(RELAY_OFF);
    cyl3.write(RELAY_OFF);
    cyl4.write(RELAY_OFF);
}

void stopAllClimb() {
    m3_pwm = 0.0f; m3_ina = 0; m3_inb = 0;
    m4_pwm = 0.0f; m4_ina = 0; m4_inb = 0;
}

void moveM3M4Up(float speed) {
    if (speed > 1.0f) speed = 1.0f;
    m3_pwm = speed; m3_ina = (DIR_UP_M3 > 0); m3_inb = (DIR_UP_M3 < 0);
    m4_pwm = speed; m4_ina = (DIR_UP_M4 > 0); m4_inb = (DIR_UP_M4 < 0);
}

void moveM3Down(float speed) {
    if (speed > 1.0f) speed = 1.0f;
    m3_pwm = speed; m3_ina = (DIR_DOWN_M3 > 0); m3_inb = (DIR_DOWN_M3 < 0);
}

void moveM4Down(float speed) {
    if (speed > 1.0f) speed = 1.0f;
    m4_pwm = speed; m4_ina = (DIR_DOWN_M4 > 0); m4_inb = (DIR_DOWN_M4 < 0);
}

void moveCANForward(float duty) { set_motor(1, duty); }
void moveCANBackward(float duty) { set_motor(-1, duty); }

// ================= MOTOR CONTROL =================
void set_motor(int dir, float duty) {
    if (duty > 1.0f) duty = 1.0f;
    if (duty < 0.0f) duty = 0.0f;
    if (dir == 1) { m1_dira = 0; m1_dirb = 1; m1_pwm = duty; }
    else if (dir == -1) { m1_dira = 1; m1_dirb = 0; m1_pwm = duty; }
    else { m1_dira = 0; m1_dirb = 0; m1_pwm = 0.0f; }
}

// ================= CAN COMMUNICATION =================
void send_cmd(uint32_t id, float vel, float kp, float kd) {
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
    tx_msg.id = id; 
    tx_msg.type = CANData; 
    tx_msg.format = CANStandard; 
    tx_msg.len = 8;
    for(int i = 0; i < 7; i++) tx_msg.data[i] = 0xFF; 
    tx_msg.data[7] = 0xFC;
    can.write(tx_msg); 
    thread_sleep_for(50);
}

void safe_stop_all() {
    for(int k = 0; k < 3; k++) {
        for(int i = 0; i < NUM_MOTORS; i++) {
            send_cmd(MOTOR_ID_1 + i, 0.0f, 0.0f, 10.0f); 
            thread_sleep_for(1);
        }
        thread_sleep_for(10);
    }
}

void on_rx() {
    if(can.read(rx_msg)) {
        if(rx_msg.id == REPLY_ID_1 && rx_msg.len >= 6) rx_count[0]++;
        else if(rx_msg.id == REPLY_ID_2 && rx_msg.len >= 6) rx_count[1]++;
        else if(rx_msg.id == REPLY_ID_3 && rx_msg.len >= 6) rx_count[2]++;
        else if(rx_msg.id == REPLY_ID_4 && rx_msg.len >= 6) rx_count[3]++;
    }
}

// ================= LCD FUNCTIONS =================
void wait_us_lcd(int us) { thread_sleep_for(us / 1000); }

void write_i2c(uint8_t data) { 
    char cmd = data | LCD_BACKLIGHT; 
    i2c_lcd.write(lcd_addr << 1, &cmd, 1); 
}

void pulse_enable(uint8_t data) { 
    write_i2c(data | LCD_EN); 
    wait_us_lcd(100); 
    write_i2c(data & ~LCD_EN); 
    wait_us_lcd(100); 
}

void send_nibble(uint8_t nibble, uint8_t mode) { 
    uint8_t data = (nibble & 0xF0) | mode | LCD_BACKLIGHT; 
    pulse_enable(data); 
}

void send_command(uint8_t cmd) { 
    send_nibble(cmd & 0xF0, 0); 
    send_nibble((cmd << 4) & 0xF0, 0); 
    thread_sleep_for(2); 
}

void send_char(char c) { 
    send_nibble(c & 0xF0, LCD_RS); 
    send_nibble((c << 4) & 0xF0, LCD_RS); 
}

void lcd_print(const char* str) { 
    while (*str) send_char(*str++); 
}

void lcd_print_num(float num, int decimals) { 
    char buf[16]; 
    snprintf(buf, sizeof(buf), "%.*f", decimals, num); 
    lcd_print(buf); 
}

void lcd_clear() { 
    send_command(0x01); 
    thread_sleep_for(2); 
}

void lcd_set_cursor(uint8_t row, uint8_t col) { 
    uint8_t addr = (row == 0) ? 0x80 + col : 0xC0 + col; 
    send_command(addr); 
}

bool detect_lcd() { 
    char dummy = 0; 
    if (i2c_lcd.write(LCD_ADDR_1 << 1, &dummy, 0) == 0) { 
        lcd_addr = LCD_ADDR_1; 
        return true; 
    } 
    if (i2c_lcd.write(LCD_ADDR_2 << 1, &dummy, 0) == 0) { 
        lcd_addr = LCD_ADDR_2; 
        return true; 
    } 
    return false; 
}

void lcd_init() { 
    thread_sleep_for(50); 
    write_i2c(0x03 | LCD_BACKLIGHT); 
    wait_us_lcd(4500); 
    pulse_enable(0x03 | LCD_BACKLIGHT); 
    wait_us_lcd(4500); 
    pulse_enable(0x03 | LCD_BACKLIGHT); 
    wait_us_lcd(150); 
    pulse_enable(0x02 | LCD_BACKLIGHT); 
    send_command(0x28);
    send_command(0x0C);
    send_command(0x06);
    send_command(0x01);
    thread_sleep_for(2); 
}

// ================= MAIN =================
int main() {
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSE_CONFIG(RCC_LSE_OFF);
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) != RESET);

    for (int i = 0; i < 4; i++) { relays[i]->write(RELAY_OFF); }
    
    m3_pwm.period_ms(2); m3_pwm = 0.0f;
    m4_pwm.period_ms(2); m4_pwm = 0.0f;
    stopAllClimb();
    
    m1_pwm.period(0.0001f);
    m1_pwm = 0.0f;
    m1_dira = 0; m1_dirb = 0;
    
    thread_sleep_for(500);
    pc.printf("\r\n=== System Boot ===\r\n");

    if (!detect_lcd()) { led = 1; while(true) { led = !led; thread_sleep_for(200); } }
    lcd_init(); lcd_clear();
    
    lcd_set_cursor(0, 0); lcd_print("ABU Robocon 2026");
    lcd_set_cursor(1, 0); lcd_print("Init CAN...");
    
    int freq = can.frequency(CAN_BAUD);
    if(freq == 0) { lcd_set_cursor(1, 0); lcd_print("CAN FAILED!   "); while(1) { led = !led; thread_sleep_for(100); } }
    
    can.attach(&on_rx, CAN::RxIrq);
    can.filter(0x100, 0x700, CANStandard, 0);
    
    enter_mode(MOTOR_ID_1); enter_mode(MOTOR_ID_2);
    enter_mode(MOTOR_ID_3); enter_mode(MOTOR_ID_4);
    safe_stop_all();
    lcd_set_cursor(1, 0); lcd_print("Ready. Wait St");
    
    loop_timer.start();
    
    while (true) {
        int g_hit = is_pressed(lim_grip);
        int r_hit = is_pressed(lim_release);
        int btn_1_now = is_pressed(button_1);
        int btn_2_now = is_pressed(button_2);
        int st = read_active_low(sens_start);
        
        if (current_state != last_state) {
            state_timer.reset(); state_timer.start(); debug_timer.reset();
            last_state = current_state;
            pc.printf("State: %d\r\n", current_state);
            if (current_state == STATE_1) state_1_step = 0;
            if (current_state == STATE_2) state_2_step = 0;
            if (current_state == FIRST_LADDER) ladder_sub_step = 0;
            
            if (current_state != MOVE_BACK_1 && current_state != MOVE_BACK_2 && 
                current_state != MOVE_FOR_1 && current_state != TURN_TO_LADDER && 
                current_state != FIRST_LADDER && current_state != STATE_1 && current_state != STATE_2) {
                safe_stop_all(); set_motor(0, 0); stopAllClimb();
            }
        }

        // ================= DISABLED =================
        if (current_state == DISABLED) {
            safe_stop_all(); cyl_stop_all(); set_motor(0, 0); stopAllClimb();
            if (read_active_low(sens_start) == 0) { 
                thread_sleep_for(50);
                if (read_active_low(sens_start) == 0) {
                    current_direction_pattern = 3; button_mode_active = false;
                    current_state = MOVE_BACK_1;
                    lcd_clear(); lcd_set_cursor(0, 0); lcd_print("1. MOVE BACK 1");
                }
            }
            else {
                lcd_set_cursor(0, 0); lcd_print("St:"); lcd_print(st ? "1" : "0");
                lcd_set_cursor(1, 0); lcd_print(button_mode_active ? "BTN:DIR4" : "AUTO:DIR3");
                static Timer blink; if (blink.read() > 0.5f) { blink.reset(); led = !led; }
            }
        } 
        
        // ================= MOVE_BACK_1 =================
        else if (current_state == MOVE_BACK_1) {
            float t = state_timer.read();
            if (t >= TIME_1) { safe_stop_all(); thread_sleep_for(300); current_state = CYL1_CYCLE; lcd_clear(); lcd_set_cursor(0, 0); lcd_print("2. CYL1 EXT"); } 
            else {
                for(int i=0; i<NUM_MOTORS; i++) velocities[i] = MAX_VEL * directions[3][i];
                send_cmd(MOTOR_ID_1, velocities[0]); thread_sleep_for(1); send_cmd(MOTOR_ID_2, velocities[1]); thread_sleep_for(1);
                send_cmd(MOTOR_ID_3, velocities[2]); thread_sleep_for(1); send_cmd(MOTOR_ID_4, velocities[3]); thread_sleep_for(1);
                if(debug_timer.read() > 0.2f) { debug_timer.reset(); lcd_set_cursor(1, 0); lcd_print("Moving Back..."); } led = 1;
            }
            thread_sleep_for(20);
        }
        
        // ================= CYL1_CYCLE =================
        else if (current_state == CYL1_CYCLE) {
            cyl1.write(RELAY_ON); thread_sleep_for(2000);
            current_state = MOVE_BACK_2; thread_sleep_for(50);
        }
        
        // ================= MOVE_BACK_2 =================
        else if (current_state == MOVE_BACK_2) {
            float t = state_timer.read();
            if (t >= TIME_2) { safe_stop_all(); thread_sleep_for(300); current_state = CYL2_CYCLE; lcd_clear(); lcd_set_cursor(0, 0); lcd_print("4. CYL2 EXT"); } 
            else {
                for(int i=0; i<NUM_MOTORS; i++) velocities[i] = MAX_VEL * directions[3][i];
                send_cmd(MOTOR_ID_1, velocities[0]); thread_sleep_for(1); send_cmd(MOTOR_ID_2, velocities[1]); thread_sleep_for(1);
                send_cmd(MOTOR_ID_3, velocities[2]); thread_sleep_for(1); send_cmd(MOTOR_ID_4, velocities[3]); thread_sleep_for(1);
                if(debug_timer.read() > 0.2f) { debug_timer.reset(); lcd_set_cursor(1, 0); lcd_print("Moving Back 2..."); } led = 1;
            }
            thread_sleep_for(20);
        }
        
        // ================= CYL2_CYCLE =================
        else if (current_state == CYL2_CYCLE) {
            cyl2.write(RELAY_ON); thread_sleep_for(2000);
            current_state = MOVE_FOR_1; thread_sleep_for(50);
        }

        // ================= MOVE_FOR_1 =================
        else if (current_state == MOVE_FOR_1) {
            float t = state_timer.read();
            if (t >= 6.0) { safe_stop_all(); thread_sleep_for(300); current_state = MOVE_DC_TO_RELEASE; lcd_clear(); lcd_set_cursor(0, 0); lcd_print("5. DC->RELEASE"); } 
            else {
                for(int i=0; i<NUM_MOTORS; i++) velocities[i] = MAX_VEL * directions[4][i];
                send_cmd(MOTOR_ID_1, velocities[0]); thread_sleep_for(1); send_cmd(MOTOR_ID_2, velocities[1]); thread_sleep_for(1);
                send_cmd(MOTOR_ID_3, velocities[2]); thread_sleep_for(1); send_cmd(MOTOR_ID_4, velocities[3]); thread_sleep_for(1);
                if(debug_timer.read() > 0.2f) { debug_timer.reset(); lcd_set_cursor(1, 0); lcd_print("Moving Forward..."); } led = 1;
            }
            thread_sleep_for(20);
        }
        
        // ================= MOVE_DC_TO_RELEASE =================
        else if (current_state == MOVE_DC_TO_RELEASE) {
            cyl1.write(RELAY_ON); wait_ms(1000);
            int release_hit = is_pressed(lim_release);
            set_motor(-1, 0.3f);
            lcd_set_cursor(0, 0); lcd_print("DC: Moving");
            if (release_hit) {
                m1_pwm = 0.0f; m1_dira = 1; m1_dirb = 0; wait_ms(50); m1_dira = 0; m1_dirb = 0; wait_ms(100);
                lcd_clear(); lcd_set_cursor(0, 0); lcd_print("RELEASE HIT");
            }
            if (state_timer.read() > 5.5f) { set_motor(0, 0); thread_sleep_for(50); }
            if (btn_1_now == 0 && btn1_last == 1) { thread_sleep_for(30); if (is_pressed(button_1) == 0) { current_state = STATE_1; pc.printf("Button PC3 -> STATE_1\r\n"); } }
            else if (btn_2_now == 0 && btn2_last == 1) { thread_sleep_for(30); if (is_pressed(button_2) == 0) { current_state = STATE_2; pc.printf("Button PA10 -> STATE_2\r\n"); } }
            thread_sleep_for(20);
        }
        
        // ================= STATE_1 =================
        else if (current_state == STATE_1) {
            float t = state_timer.read();
            if(debug_timer.read() > 1.0f) { 
                debug_timer.reset(); 
                pc.printf("STATE_1 Step: %d\r\n", state_1_step); 
                lcd_set_cursor(1, 0); lcd_print("S1:"); lcd_print_num(state_1_step, 0); 
            }

            if (state_1_step == 0) { 
                stopAllClimb(); safe_stop_all(); 
                cyl2.write(RELAY_ON);
                state_1_step = 1; 
                state_timer.reset(); 
                thread_sleep_for(50); 
            }
            else if (state_1_step == 1) {
                cyl1.write(RELAY_ON); 
                wait_ms(1000);
                int release_hit = is_pressed(lim_release);
                set_motor(1, 0.3f);
                lcd_set_cursor(0, 0); lcd_print("1. DC: Moving");
                
                if (release_hit) {
                    m1_pwm = 0.0f; m1_dira = 0; m1_dirb = 1; wait_ms(50); 
                    m1_dira = 0; m1_dirb = 0; wait_ms(100);
                    lcd_clear(); lcd_set_cursor(0, 0); lcd_print("1.RELEASE HIT");
                    state_1_step = 2;
                    state_timer.reset();
                }
                if (state_timer.read() > 5.2f) { 
                    set_motor(0, 0);  
                    state_1_step = 2;
                    state_timer.reset();
                }
                thread_sleep_for(20);
            }
            else if (state_1_step == 2) {
                if (t >= 5) { 
                    safe_stop_all(); thread_sleep_for(300); 
                    state_1_step = 3; 
                    lcd_clear(); lcd_set_cursor(0, 0); lcd_print("S1: Back"); 
                    state_timer.reset(); 
                } else {
                    for(int i=0; i<NUM_MOTORS; i++) 
                        velocities[i] = MAX_VEL * -1 * directions[4][i];
                    for(int i=0; i<NUM_MOTORS; i++) { 
                        send_cmd(MOTOR_ID_1 + i, velocities[i]); thread_sleep_for(1); 
                    }
                    if(debug_timer.read() > 0.2f) { 
                        debug_timer.reset(); lcd_set_cursor(1, 0); lcd_print("1.RevArc..."); 
                    } 
                    led = 1;
                }
                thread_sleep_for(20);
            }
            else if (state_1_step == 3) {
                if (t >= TIME_2) { 
                    safe_stop_all(); thread_sleep_for(300); 
                    state_1_step = 4; 
                    lcd_clear(); lcd_set_cursor(0, 0); lcd_print("S1: Stabilize"); 
                    state_timer.reset(); 
                } else {
                    for(int i=0; i<NUM_MOTORS; i++) 
                        velocities[i] = MAX_VEL * directions[3][i];
                    for(int i=0; i<NUM_MOTORS; i++) { 
                        send_cmd(MOTOR_ID_1 + i, velocities[i]); thread_sleep_for(1); 
                    }
                    if(debug_timer.read() > 0.2f) { 
                        debug_timer.reset(); lcd_set_cursor(1, 0); lcd_print("1.Back..."); 
                    } 
                    led = 1;
                }
                thread_sleep_for(20);
            }
            else if (state_1_step == 4) { 
                state_1_step = 5; 
                state_timer.reset(); 
                thread_sleep_for(50); 
            }
            else if (state_1_step == 5) {
                if (t >= 5.0) {
                    safe_stop_all(); thread_sleep_for(300);
                    cyl1.write(RELAY_OFF);
                    state_1_step = 6; 
                    lcd_clear(); lcd_set_cursor(0, 0); lcd_print("S1: cyl1 OFF"); 
                    state_timer.reset();
                } else {
                    cyl2.write(RELAY_ON);
                    cyl1.write(RELAY_OFF);
                    for(int i=0; i<NUM_MOTORS; i++) 
                        velocities[i] = MAX_VEL * directions[4][i];
                    for(int i=0; i<NUM_MOTORS; i++) { 
                        send_cmd(MOTOR_ID_1 + i, velocities[i]); thread_sleep_for(1); 
                    }
                    if(debug_timer.read() > 0.2f) { 
                        debug_timer.reset(); lcd_set_cursor(1, 0); lcd_print("1.cyl1OFF+Dir5"); 
                    } 
                    led = 1;
                }
                thread_sleep_for(20);
            }
            else if (state_1_step == 6) { 
                safe_stop_all(); thread_sleep_for(300); 
                current_state = FINISHED; 
                lcd_clear(); lcd_set_cursor(0, 0); lcd_print("STATE_1 DONE"); 
            }
        }
        
        // ================= STATE_2 =================
        else if (current_state == STATE_2) {
            float t = state_timer.read();
            if(debug_timer.read() > 1.0f) { 
                debug_timer.reset(); 
                pc.printf("STATE_2 Step: %d\r\n", state_2_step); 
                lcd_set_cursor(1, 0); lcd_print("S2:"); lcd_print_num(state_2_step, 0); 
            }

            if (state_2_step == 0) { 
                stopAllClimb(); safe_stop_all(); 
                cyl2.write(RELAY_ON);
                state_2_step = 1; 
                state_timer.reset(); 
                thread_sleep_for(50); 
            }
            else if (state_2_step == 1) {
                cyl1.write(RELAY_ON); 
                wait_ms(1000);
                int release_hit = is_pressed(lim_release);
                set_motor(1, 0.3f);
                lcd_set_cursor(0, 0); lcd_print("2. DC: Moving");
                
                if (release_hit) {
                    m1_pwm = 0.0f; m1_dira = 1; m1_dirb = 1; wait_ms(50); 
                    m1_dira = 0; m1_dirb = 0; wait_ms(100);
                    lcd_clear(); lcd_set_cursor(0, 0); lcd_print("2.RELEASE HIT");
                    state_2_step = 2;
                    state_timer.reset();
                }
                if (state_timer.read() > 5.5f) { 
                    set_motor(0, 0);  
                    state_2_step = 2;
                    state_timer.reset();
                }
                thread_sleep_for(20);
            }
            else if (state_2_step == 2) {
                if (t >= 2.3) { 
                    safe_stop_all(); thread_sleep_for(300); 
                    state_2_step = 3; 
                    lcd_clear(); lcd_set_cursor(0, 0); lcd_print("S2: Back"); 
                    state_timer.reset(); 
                } else {
                    for(int i=0; i<NUM_MOTORS; i++) 
                        velocities[i] = MAX_VEL * -1 * directions[4][i];
                    for(int i=0; i<NUM_MOTORS; i++) { 
                        send_cmd(MOTOR_ID_1 + i, velocities[i]); thread_sleep_for(1); 
                    }
                    if(debug_timer.read() > 0.2f) { 
                        debug_timer.reset(); lcd_set_cursor(1, 0); lcd_print("2.FwdArc..."); 
                    } 
                    led = 1;
                }
                thread_sleep_for(20);
            }
            else if (state_2_step == 3) {
                if (t >= 3.0) { 
                    safe_stop_all(); thread_sleep_for(300); 
                    state_2_step = 4; 
                    lcd_clear(); lcd_set_cursor(0, 0); lcd_print("S2: Stabilize"); 
                    state_timer.reset(); 
                } else {
                    for(int i=0; i<NUM_MOTORS; i++) 
                        velocities[i] = MAX_VEL * directions[3][i];
                    for(int i=0; i<NUM_MOTORS; i++) { 
                        send_cmd(MOTOR_ID_1 + i, velocities[i]); thread_sleep_for(1); 
                    }
                    if(debug_timer.read() > 0.2f) { 
                        debug_timer.reset(); lcd_set_cursor(1, 0); lcd_print("2.Back..."); 
                    } 
                    led = 1;
                }
                thread_sleep_for(20);
            }
            else if (state_2_step == 4) { 
                cyl2.write(RELAY_ON);
                cyl1.write(RELAY_OFF);
                state_2_step = 5; 
                state_timer.reset(); 
                thread_sleep_for(200); 
            }
            else if (state_2_step == 5) {
                if (t >= 4.0) {
                    safe_stop_all(); thread_sleep_for(300);
                    cyl1.write(RELAY_OFF);
                    state_2_step = 6; 
                    lcd_clear(); lcd_set_cursor(0, 0); lcd_print("S2: cyl1 OFF"); 
                    state_timer.reset();
                } else {                    
                    for(int i=0; i<NUM_MOTORS; i++) 
                        velocities[i] = MAX_VEL * directions[4][i];
                    for(int i=0; i<NUM_MOTORS; i++) { 
                        send_cmd(MOTOR_ID_1 + i, velocities[i]); thread_sleep_for(1); 
                    }
                    if(debug_timer.read() > 0.2f) { 
                        debug_timer.reset(); lcd_set_cursor(1, 0); lcd_print("2.cyl1OFF+Dir5"); 
                    } 
                    led = 1;
                }
                thread_sleep_for(20);
            }
            else if (state_2_step == 6) { 
                safe_stop_all(); thread_sleep_for(300); 
                current_state = FINISHED; 
                lcd_clear(); lcd_set_cursor(0, 0); lcd_print("STATE_2 DONE"); 
            }
        }
        
        // ================= FINISHED =================
        else if (current_state == FINISHED) {
            safe_stop_all(); set_motor(0, 0); stopAllClimb();
            lcd_set_cursor(0, 0); lcd_print("FINISHED!"); lcd_set_cursor(1, 0); lcd_print("Press START to Reset");
    
            if (read_active_low(sens_start) == 1) {
                thread_sleep_for(50);
                if (read_active_low(sens_start) == 1) {
                    cyl_stop_all();
                    current_state = DISABLED; 
                    button_mode_active = false; 
                    current_direction_pattern = 3;
                    state_1_step = 0; 
                    state_2_step = 0;
                    lcd_clear(); 
                    lcd_print("Ready");
                    pc.printf("Reset to DISABLED\r\n");
                }
            }
            thread_sleep_for(20);
        }

        // ================= LCD & LED =================
        if (debug_timer.read() >= 0.2f) {
            debug_timer.reset(); lcd_set_cursor(0, 0);
            const char* state_names[] = { "DISABLED", "MOVE BACK  ", "CYL1 EXT   ", "BACK 2     ", "CYL2 EXT   ", "DC->REL  ", "TURN SEQ   ", "APPROACH   ", "CLIMBING!  ", "STATE_1    ", "STATE_2    ", "FINISHED!  " };
            if (current_state >= 0 && current_state <= FINISHED) lcd_print(state_names[current_state]); else lcd_print("State: ???    ");
            lcd_set_cursor(1, 0); send_char(g_hit ? 'G' : '.'); send_char(r_hit ? 'R' : '.'); lcd_print(" Btn:"); send_char(btn_1_now ? '1' : '0'); send_char(btn_2_now ? '1' : '0');
        }
        if (current_state == DISABLED || current_state == FINISHED) led = 1; else led = (loop_timer.read_ms() % 200 < 100);
        btn1_last = btn_1_now; btn2_last = btn_2_now; thread_sleep_for(20);
    }
}
