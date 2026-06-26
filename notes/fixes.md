# Bug Catalog & SRP Violations

## Bugs Found

| # | File | Line | Bug |
|---|------|------|-----|
| 1 | `display.cpp` | 25 | `initSSD1306Driver()` called **before** I2C init check (line 27) — sends I2C commands to uninitialized bus |
| 2 | `main.cpp` | 129 | Uninitialized `line` buffer passed to `drawString` on page 0 — shows garbage instead of per-config jump counts |
| 3 | `jr_ble.cpp` | 168,178,350,422 | `g_streaming` read/written outside critical section — data race |
| 4 | `max30102.cpp` | 70,82,127 | Hardcoded `I2C_NUM_0`, bypasses `I2CManager` mutex — race condition with gyro/display |
| 5 | `main.cpp` | 159 | Hardcoded `HR=1` → frontend shows "1 BPM" as if valid; misleading |
| 6 | `gpio_pin.cpp` | 79 | `update()` hardcodes `!gpio_get_level()` (active-low), ignores `is_inverted` — inconsistent with `read()` |
| 7 | `main.cpp` | 1,10 | Unused includes `algorithm_by_RF.h`, `max30102.h` — HR task commented out |
| 8 | `data_base.js` | 308,320 | `user_id` column is TEXT but passed as integer — SQLite type mismatch |
| 9 | `main.cpp` | 86 | `tickCount * portTICK_PERIOD_MS` overflows after ~49 days of uptime |

## SRP Violations

| # | File | Concern |
|---|------|---------|
| 1 | `main.cpp` | Entry point + task creation + display UI loop + BLE coordination + calibration state — 5 responsibilities |
| 2 | `JumpDetector` | Signal filtering (EMA) + state machine (4x) + adaptive threshold + calibration + FreeRTOS task wrapper + data queries |
| 3 | `SensorReading` | HW init + raw I2C reads + complementary filter + gravity calibration + FreeRTOS task + singleton |
| 4 | `data_base.js` | DB schema init + validation + user CRUD + workout CRUD + promise wrappers |
| 5 | `webApp.js` | Server setup + session config + rate limiting + all route handlers + auth middleware |
| 6 | `ble.js` (frontend) | BLE connection + packet parsing + DOM updates + workout state + backend API calls |
| 7 | `jr_ble.cpp` | NVS init + NimBLE init + GATT server + GAP events + advertising + packet builder + notify task |
| 8 | `gpio_pin.cpp` | Physical read + debounce + press detection + hold detection all in one `update()` |
| 9 | `display.cpp` | I2C comms + framebuffer + font rasterizer + hardware init sequence |
| 10 | Root directory | 6 orphaned duplicate frontend files (`index.html`, `data.html`, `ble.js`, `workout.js`, `data.css`, `webApp.css`) |

## AGENTS.md Coding Standards Applied

Files refactored against `AGENTS.md` rules:
- `main/main.cpp`
- `main/jump.cpp`, `main/jump.h`
- `main/gpio_pin.cpp`, `main/gpio_pin.h`
- `components/ble/jr_ble.cpp`, `components/ble/jr_ble.h`
- `components/gyro/gyro.cpp`, `components/gyro/gyro.h`
- `components/display/display.cpp`, `components/display/display.h`
- `components/i2cInit/i2cInit.cpp`, `components/i2cInit/i2cInit.h`
- `components/common/mutex.h`

New files created:
- `main/display_controller.h` — DisplayController class (extracted from main.cpp)
- `main/display_controller.cpp`

### Rules Applied

| Rule | Description |
|------|-------------|
| CS-001/007/008 | Allman brace style (braces on next line) — via `.clang-format` |
| CS-002 | Data member names end with `M` |
| CS-003 | Function parameter names end with `P` |
| CS-004 | Static variable names end with `S` |
| CS-009 | Function names are camelCase |
| CS-010 | Variable names are camelCase |
| CS-011 | Pointer variables prefixed with `p` |
| CS-012 | Reference variables prefixed with `r` |
| CS-016 | 4-space indentation — via `.clang-format` |
| CS-018 | `#pragma once` instead of `#define` guards |
| CS-019 | Anonymous namespace instead of `static` file-scope |
| CS-021 | `const` after the type it describes |
| CS-025 | Single exit point per function |
| CS-029 | Uniform initializers `{}` |
| CS-030 | RAII — `std::unique_ptr` instead of raw `new`/`delete` |

### Fixes Applied

1. **display.cpp**: Moved I2C init check before `initSSD1306Driver()` call
2. **main.cpp**: Filled uninitialized `line` buffer with `snprintf` on page 0
3. **jr_ble.cpp**: Wrapped `g_streaming` in critical sections
4. **max30102.cpp**: Replaced `I2C_NUM_0` with `I2CManager` + `MutexGuard`
5. **main.cpp**: Changed `jr_ble_set_sensor_snapshot(..., 1, 2, 0)` to `(..., 0, 0, 0)`
6. **gpio_pin.cpp**: Changed `!gpio_get_level()` to respect `is_inverted`
7. **main.cpp**: Removed unused `#include "algorithm_by_RF.h"` and `#include "max30102.h"`
8. **data_base.js**: Cast `uid` to `String(...)` for TEXT column
9. **main.cpp**: Replaced `tickCount * portTICK_PERIOD_MS` with `esp_timer_get_time() / 1000`
