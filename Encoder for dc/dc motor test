/* 
 * ==========================================================================
 *  ABU ROBOCON 2026 - R2: Grip & Release Cycle Control (DIRECTION FIXED)
 *  Target: STM32 Nucleo-F446RE
 *  
 *  CHANGES:
 *  - Motor Direction Logic INVERTED in set_motor() function.
 *  - Now: Dir 1 = Moves FROM Grip TO Release.
 *  - Now: Dir -1 = Moves FROM Release TO Grip.
 *  
 *  PIN MAPPING:
 *  - PC_4: LIM_GRIP (Start/Home Position) - Active Low
 *  - PC_1: LIM_RELEASE (End/Far Position) - Active Low
 *  
 *  BEHAVIOR:
 *  1. Power On: Auto-find nearest limit.
 *  2. Idle: Wait at LIM_GRIP (PC_4).
 *  3. Press Button (1st): Start AUTO-CYCLE (Grip <-> Release).
 *  4. Press Button (2nd): STOP Cycle -> Return to LIM_GRIP -> Stop & Wait.
 * ==========================================================================
 */

#include "mbed.h"

// --- HARDWARE PINS ---
DigitalIn lim_grip(PA_5, PullUp);      // START/HOME Position - Active Low
DigitalIn lim_release(PB_0, PullUp);   // END/FAR Position - Active Low
DigitalIn action_btn(PA_10, PullUp);   // Single Control Button

// Motor Driver (VNH5019)
PwmOut pwm(PB_14);
DigitalOut dira(PC_8);
DigitalOut dirb(PC_6);

// LCD (I2C)
#define I2C_SDA PB_9
#define I2C_SCL PB_8
I2C i2c_lcd(I2C_SDA, I2C_SCL);
uint8_t lcd_addr = 0x27; 
#define LCD_BACKLIGHT 0x08
#define LCD_EN 0x04
#define LCD_RS 0x01

// --- CONFIGURATION ---
const float MOVE_DUTY = 0.4f;      // Speed for moving between limits
const float SEARCH_DUTY = 0.2f;    // Slow speed for initial finding
DigitalOut led(LED1);

// --- STATE ENUMS ---
enum RobotState {
    STATE_INIT_SEARCH,   // Finding first limit on startup
    STATE_IDLE,          // Waiting at GRIP limit
    STATE_AUTO_CYCLE,    // Moving Back & Forth automatically
    STATE_RETURN_GRIP    // Moving specifically to GRIP limit to stop
};

volatile RobotState current_state = STATE_INIT_SEARCH;
volatile int auto_dir = 1; // 1=Fwd (to Release), -1=Rev (to Grip)

// --- LCD HELPERS ---
void wait_us(int us) { thread_sleep_for(us / 1000); }
void write_i2c(uint8_t data) {
    char cmd = data | LCD_BACKLIGHT;
    i2c_lcd.write(lcd_addr << 1, &cmd, 1);
}
void pulse_enable(uint8_t data) {
    write_i2c(data | LCD_EN); wait_us(100);
    write_i2c(data & ~LCD_EN); wait_us(100);
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
void lcd_init() {
    thread_sleep_for(50);
    write_i2c(0x03 | LCD_BACKLIGHT); wait_us(4500);
    pulse_enable(0x03 | LCD_BACKLIGHT); wait_us(4500);
    pulse_enable(0x03 | LCD_BACKLIGHT); wait_us(150);
    pulse_enable(0x02 | LCD_BACKLIGHT);
    send_command(0x28); send_command(0x0C); send_command(0x06); send_command(0x01);
    thread_sleep_for(2);
}
void lcd_clear() { send_command(0x01); thread_sleep_for(2); }
void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0) ? 0x80 + col : 0xC0 + col;
    send_command(addr);
}
void lcd_print(const char* str) { while (*str) send_char(*str++); }
bool detect_lcd() {
    char dummy = 0;
    if (i2c_lcd.write(lcd_addr << 1, &dummy, 0) == 0) return true;
    if (i2c_lcd.write((lcd_addr + 2) << 1, &dummy, 0) == 0) { lcd_addr += 2; return true; }
    return false;
}

// --- MOTOR CONTROL (DIRECTION REVERSED) ---
// dir: 1 = Forward (Towards RELEASE)
// dir: -1 = Reverse (Towards GRIP)
void set_motor(int dir, float duty) {
    if (duty > 1.0f) duty = 1.0f;
    if (duty < 0.0f) duty = 0.0f;
    
    if (dir == 1) { 
        // REVERSED: Forward now sets dira=0, dirb=1
        dira = 0; 
        dirb = 1; 
        pwm = duty; 
    } else if (dir == -1) { 
        // REVERSED: Reverse now sets dira=1, dirb=0
        dira = 1; 
        dirb = 0; 
        pwm = duty; 
    } else { 
        dira = 0; 
        dirb = 0; 
        pwm = 0.0f; 
    }
}

int is_pressed(DigitalIn& s) { return (s.read() == 0) ? 1 : 0; }

int main() {
    // 1. Init LCD
    if (!detect_lcd()) { while(true) { led = !led; thread_sleep_for(100); } }
    lcd_init();
    lcd_clear();
    lcd_print("ABU Robocon 2026");
    lcd_set_cursor(1, 0);
    lcd_print("Grip/Release Ctrl");
    thread_sleep_for(1000);
    lcd_clear();

    // 2. Init Motor
    pwm.period(0.0001f);
    set_motor(0, 0);

    // 3. Startup Check
    int g_hit = is_pressed(lim_grip);    // PC_4
    int r_hit = is_pressed(lim_release); // PC_1

    if (g_hit || r_hit) {
        current_state = STATE_IDLE;
        lcd_print("Ready at Limit");
    } else {
        current_state = STATE_INIT_SEARCH;
        lcd_print("Searching Limit...");
    }
    
    thread_sleep_for(1000);
    lcd_clear();

    Timer loop_timer;
    loop_timer.start();
    
    bool btn_last = 1;

    while (true) {
        int g_hit = is_pressed(lim_grip);    // PC_4
        int r_hit = is_pressed(lim_release); // PC_1
        int btn_now = is_pressed(action_btn);

        int command_dir = 0;
        float command_duty = 0.0f;

        // ==================================================================
        // STATE MACHINE
        // ==================================================================

        if (current_state == STATE_INIT_SEARCH) {
            // Move Forward until ANY limit is hit
            command_dir = 1;
            command_duty = SEARCH_DUTY;

            if (g_hit || r_hit) {
                set_motor(0, 0);
                current_state = STATE_IDLE;
                lcd_clear();
                lcd_print("Found Limit!");
                lcd_set_cursor(1, 0);
                lcd_print("Press Btn to Run");
                thread_sleep_for(1000);
                lcd_clear();
            }
        }

        else if (current_state == STATE_IDLE) {
            // Wait for button press to start Auto-Cycle
            command_dir = 0;
            command_duty = 0.0f;

            // Detect Button Press
            if (btn_last == 0 && btn_now == 1) {
                current_state = STATE_AUTO_CYCLE;
                // Determine initial direction: 
                // If at GRIP, go to RELEASE (Fwd). If at RELEASE, go to GRIP (Rev).
                if (g_hit) auto_dir = 1; 
                else if (r_hit) auto_dir = -1;
                else auto_dir = 1; // Default
                
                thread_sleep_for(50); // Debounce
            }
        }

        else if (current_state == STATE_AUTO_CYCLE) {
            // Move Back and Forth indefinitely
            command_dir = auto_dir;
            command_duty = MOVE_DUTY;

            // Toggle direction when hitting limits
            if (r_hit) {
                auto_dir = -1; // Hit RELEASE, go Reverse (to Grip)
            } else if (g_hit) {
                auto_dir = 1; // Hit GRIP, go Forward (to Release)
            }

            // Detect Button Press to STOP and Return to GRIP
            if (btn_last == 0 && btn_now == 1) {
                current_state = STATE_RETURN_GRIP;
                thread_sleep_for(50); // Debounce
            }
        }

        else if (current_state == STATE_RETURN_GRIP) {
            // Goal: Move to GRIP Limit (PC_4) and Stop there permanently
            command_dir = -1; // Always move Reverse towards GRIP
            command_duty = MOVE_DUTY;

            if (g_hit) {
                // Reached GRIP Limit
                set_motor(0, 0);
                current_state = STATE_IDLE;
                lcd_clear();
                lcd_print("Returned to GRIP");
                lcd_set_cursor(1, 0);
                lcd_print("Press Btn to Run");
                thread_sleep_for(1000);
                lcd_clear();
            }
            
            // Safety: If we hit RELEASE limit while trying to go to GRIP? 
            if (r_hit) {
                auto_dir = -1; 
            }
        }

        // --- EXECUTE MOTOR COMMAND ---
        set_motor(command_dir, command_duty);

        // --- LED FEEDBACK ---
        if (current_state == STATE_IDLE) led = 1; // Solid when waiting
        else led = (loop_timer.read_ms() % 200 < 100); // Blink when moving

        // --- LCD UPDATE ---
        if (loop_timer.read() >= 0.2f) {
            loop_timer.reset();
            
            lcd_set_cursor(0, 0);
            if (current_state == STATE_INIT_SEARCH) lcd_print("Searching...");
            else if (current_state == STATE_IDLE) lcd_print("IDLE (At GRIP)");
            else if (current_state == STATE_AUTO_CYCLE) lcd_print("AUTO CYCLE >> <<");
            else if (current_state == STATE_RETURN_GRIP) lcd_print("RETURNING GRIP <<");

            lcd_set_cursor(1, 0);
            // Display Status: G = Grip (PC4), R = Release (PC1)
            send_char(g_hit ? 'G' : '.');
            send_char(r_hit ? 'R' : '.');
            lcd_print("  Btn:");
            send_char(btn_now ? '1' : '0');
        }

        btn_last = btn_now;
        thread_sleep_for(20);
    }
}
