/* =============================================================================
 forest_tof.h  —  5 × VL53L0X behind TCA9548A mux

 Sensor wiring (mux channel = (user-facing sensor number) - 1):
   ch 0  PROXIMITY   sensor 1  Case 3 sweep,  trip if <= 20 cm
   ch 1  LEFT        sensor 2  side distance (paired with RIGHT)
   ch 2  GROUND      sensor 3  descent guard, perched if >= 30 cm
   ch 3  FRONT       sensor 4  climb-up trigger,  block if <= 5 cm
   ch 4  RIGHT       sensor 5  side distance (paired with LEFT)

 Behaviour summary:
   Sensor 1 (PROXIMITY)  -> object_within_20cm()    used in Case 3
   Sensor 2 + 5 (R + L)  -> balance_correction()    keep robot straight while
                                                    walking blocks
   Sensor 3 (GROUND)     -> at_descent_edge()       skip fwd steps in is_down
   Sensor 4 (FRONT)      -> block_ahead()           positive trigger for is_up

 All reads are cached so calling helpers from a 10 ms control loop doesn't
 hammer the I2C bus. Sensors run continuous-ranging mode; cache only
 throttles mux-switch frequency.
============================================================================= */
#ifndef FOREST_TOF_H
#define FOREST_TOF_H

#include "mbed.h"
#include "VL53L0X.h"
#include "TCA9548A.h"

namespace ForestTOF {

    enum {
        PROXIMITY   = 0,
        LEFT        = 1,
        GROUND      = 2,
        FRONT       = 3,
        RIGHT       = 4,
        NUM_SENSORS = 5
    };

    // Thresholds (millimetres)
    static const uint16_t PROX_THRESHOLD_MM = 200;   // 20 cm — Case 3 sweep
    static const uint16_t FRONT_BLOCK_MM    =  50;   //  5 cm — climb-up trigger
    static const uint16_t GROUND_EDGE_MM    = 300;   // 30 cm — descent perch
    static const uint16_t PAIR_TOL_MM       =  50;   //  5 cm — L/R balance band
    static const uint16_t READ_FAIL         = 0xFFFF;

    // ── internal singletons (function-local statics keep this header-only) ──
    inline TCA9548A*& _mux()      { static TCA9548A* p = 0;                          return p; }
    inline VL53L0X**  _sensors()  { static VL53L0X*  a[NUM_SENSORS] = {0};           return a; }
    inline uint8_t*   _channels() { static uint8_t   c[NUM_SENSORS] = {0,1,2,3,4};   return c; }
    inline uint16_t*  _cache_mm() { static uint16_t  v[NUM_SENSORS] =
                                    {READ_FAIL,READ_FAIL,READ_FAIL,READ_FAIL,READ_FAIL}; return v; }
    inline uint32_t*  _cache_ms() { static uint32_t  t[NUM_SENSORS] = {0,0,0,0,0};   return t; }

    inline bool _select(int idx) {
        TCA9548A* m = _mux();
        if (!m || idx < 0 || idx >= NUM_SENSORS) return false;
        return m->select_channel(_channels()[idx]);
    }

    inline int init(I2C& bus, uint8_t mux_addr_7bit = 0x70) {
        static TCA9548A mux_inst(bus, mux_addr_7bit);
        _mux() = &mux_inst;

        static VL53L0X s0(&bus), s1(&bus), s2(&bus), s3(&bus), s4(&bus);
        VL53L0X** arr = _sensors();
        arr[0] = &s0; arr[1] = &s1; arr[2] = &s2; arr[3] = &s3; arr[4] = &s4;

        mux_inst.disable_all();
        thread_sleep_for(20);

        int ok = 0;
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (!_select(i)) continue;
            thread_sleep_for(10);
            arr[i]->init();
            arr[i]->setModeContinuous();
            arr[i]->startContinuous();
            thread_sleep_for(10);
            ok++;
        }
        return ok;
    }

    inline uint16_t read_mm(int idx) {
        if (!_select(idx)) return READ_FAIL;
        wait_us(2000);
        return _sensors()[idx]->getRangeMillimeters();
    }

    inline uint16_t read_cached(int idx, uint32_t max_age_ms = 50) {
        uint32_t now = us_ticker_read() / 1000;
        if (now - _cache_ms()[idx] > max_age_ms) {
            _cache_mm()[idx] = read_mm(idx);
            _cache_ms()[idx] = now;
        }
        return _cache_mm()[idx];
    }

    // ── Role-specific helpers ───────────────────────────────────────────────

    // Sensor 1 — used in Case 3 sweep
    inline bool object_within_20cm() {
        uint16_t r = read_cached(PROXIMITY, 50);
        return (r != READ_FAIL) && (r != 0) && (r <= PROX_THRESHOLD_MM);
    }

    // Sensor 4 — POSITIVE trigger: a block is right in front, time to climb
    inline bool block_ahead() {
        uint16_t r = read_cached(FRONT, 50);
        return (r != READ_FAIL) && (r != 0) && (r <= FRONT_BLOCK_MM);
    }

    // Sensor 3 — robot is perched at the edge of a high block looking down at
    // a lower block. is_down_group should skip forward driving and just run
    // cylinders. READ_FAIL counts as edge (out-of-range usually = far).
    inline bool at_descent_edge() {
        uint16_t r = read_cached(GROUND, 80);
        if (r == READ_FAIL) return true;
        return r >= GROUND_EDGE_MM;
    }

    // Sensors 2 & 5 — pair difference (mm). -1 if either read failed.
    inline int pair_diff_mm() {
        uint16_t l = read_cached(LEFT,  80);
        uint16_t r = read_cached(RIGHT, 80);
        if (l == READ_FAIL || r == READ_FAIL) return -1;
        int d = (int)l - (int)r;
        return (d < 0) ? -d : d;
    }

    // Returns:
    //    0  → balanced (within PAIR_TOL_MM), keep going forward
    //   +1  → LEFT is closer than RIGHT  → strafe RIGHT to balance
    //   -1  → RIGHT is closer than LEFT  → strafe LEFT  to balance
    //
    // If your physical L/R wiring turns out reversed, swap the +1/-1 returns.
    inline int balance_correction() {
        uint16_t l = read_cached(LEFT,  80);
        uint16_t r = read_cached(RIGHT, 80);
        if (l == READ_FAIL || r == READ_FAIL) return 0;
        int diff = (int)l - (int)r;          // positive ⇒ left is further ⇒ right is closer
        if (diff >  (int)PAIR_TOL_MM) return -1;   // shift LEFT
        if (diff < -(int)PAIR_TOL_MM) return +1;   // shift RIGHT
        return 0;
    }
}

#endif