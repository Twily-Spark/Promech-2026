#include "mbed.h"

// --- LCD Configuration (Hardware I2C on PB_8/PB_9) ---
// DO NOT use PC_8/PC_9 (Causes pinmap error in Mbed OS for Nucleo F446RE)
#define I2C_SDA PB_9
#define I2C_SCL PB_8
#define LCD_ADDR_1 0x27
#define LCD_ADDR_2 0x3F

// --- Sensor & Input Configuration ---
// Active-Low Sensors (PullUp enabled): 0=Pressed(Active), 1=Open(Inactive)
DigitalIn sens_start(PC_11, PullUp);       // Start Sensor
DigitalIn sens_motor_start(PA_5, PullUp); // Motor Start Pos
DigitalIn sens_field_select(PC_3, PullUp);// Game Field: 0=Red, 1=Blue
DigitalIn sens_motor_stop(PC_5, PullUp);  // Motor Stop Pos (Updated from PC3)

// Board Controls
DigitalIn board_switch(D2, PullUp);     // Switch on Board
DigitalIn board_button(D3, PullUp);    // Button on Board (Note: PA10 is USART1_TX, do not use Serial1)

// --- LCD Driver Variables ---
DigitalOut led(LED1);
I2C i2c_lcd(I2C_SDA, I2C_SCL);
uint8_t lcd_addr = 0;

#define LCD_BACKLIGHT 0x08
#define LCD_EN 0x04
#define LCD_RS 0x01

// --- LCD Low-Level Functions ---
void wait_us(int us) { 
    thread_sleep_for(us / 1000); 
}

void write_i2c(uint8_t data) {
    char cmd = data | LCD_BACKLIGHT;
    if (i2c_lcd.write(lcd_addr << 1, &cmd, 1) != 0) { 
        // Silent fail to prevent lockup
    }
}

void pulse_enable(uint8_t data) {
    write_i2c(data | LCD_EN); 
    wait_us(100);
    write_i2c(data & ~LCD_EN); 
    wait_us(100);
}

void send_nibble(uint8_t nibble, uint8_t mode) {
    uint8_t data = (nibble & 0xF0) | mode | LCD_BACKLIGHT;
    pulse_enable(data);
}

void send_command(uint8_t cmd) {
    send_nibble(cmd & 0xF0, 0);
    send_nibble((cmd << 4) & 0xF0, 0);
    thread_sleep_for(2); // Command execution time
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
    send_command(0x28); // Function Set: 4-bit, 2 lines
    send_command(0x0C); // Display On
    send_command(0x06); // Entry Mode
    send_command(0x01); // Clear Display
    thread_sleep_for(2);
}

// [FIXED] Added missing lcd_clear function
void lcd_clear() {
    send_command(0x01); // Clear Display Command
    thread_sleep_for(2); // Wait for completion
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0) ? 0x80 + col : 0xC0 + col;
    send_command(addr);
}

void lcd_print(const char* str) {
    while (*str) send_char(*str++);
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

// Helper: Returns 1 if Active (Low), 0 if Inactive (High)
int read_active_low(DigitalIn& s) {
    return (s.read() == 0) ? 1 : 0; 
}

int main() {
    led = 1;
    
    // Detect LCD
    if (!detect_lcd()) {
        // Fast blink if LCD not found
        while (true) { 
            led = !led; 
            thread_sleep_for(100); 
        }
    }

    lcd_init();
    
    // Splash Screen
    lcd_set_cursor(0, 0);
    lcd_print("ABU Robocon 2026");
    lcd_set_cursor(1, 0);
    lcd_print("R2 Sys Check...");
    thread_sleep_for(1500);
    
    lcd_clear(); // Now this works!

    while (true) {
        // --- Read Inputs ---
        int st = read_active_low(sens_start);         // PC0
        int ms = read_active_low(sens_motor_start);   // PC1
        int sp = read_active_low(sens_motor_stop);    // PC4
        
        // Field Select: 0 (Low) = RED, 1 (High) = BLUE
        int field_raw = sens_field_select.read(); 
        const char* field_str = (field_raw == 0) ? "RED" : "BLU";

        int sw = read_active_low(board_switch);       // PB3
        int btn = read_active_low(board_button);      // PA10

        // --- Update LCD Line 1: Sensors ---
        // Format: "St:0 Ms:0 Sp:0"
        lcd_set_cursor(0, 0);
        lcd_print("St:"); send_char(st ? '1' : '0');
        lcd_print(" Ms:"); send_char(ms ? '1' : '0');
        lcd_print(" Sp:"); send_char(sp ? '1' : '0');
        lcd_print(" "); // Clear tail

        // --- Update LCD Line 2: Config & Controls ---
        // Format: "Fld:RED Sw:0 B:0"
        lcd_set_cursor(1, 0);
        lcd_print("Fld:");
        lcd_print(field_str);
        lcd_print(" Sw:"); send_char(sw ? '1' : '0');
        lcd_print(" B:"); send_char(btn ? '1' : '0');
        lcd_print(" "); // Clear tail

        // --- LED Feedback ---
        if (sp) {
            led = 1; // Solid On if Stop Sensor Hit
        } else {
            led = !led; // Heartbeat
        }

        thread_sleep_for(200); // 5Hz refresh
    }
}
