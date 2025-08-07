# AGENTS.md

## 語言與溝通風格
- 所有與使用者的回覆請使用「繁體中文」。
- 所有程式碼、變數命名、註解請使用英文。
- 若需要解釋程式碼，請用繁體中文說明。



# Copilot Instructions for SN8F5602 Induction Cooker Project

## Project Overview

- This is a firmware development project for the SONiX SN8F5602 microcontroller, designed for induction heating applications (e.g., induction cookers).
- Each module typically consists of a `.c` and `.h` file pair.
- The main application logic is implemented in `main.c`.

## Architecture & Data Flow

- The main control loop in `main.c` uses a cooperative task scheduler, executing tasks such as heat control, power control, and temperature measurement through a `TaskType` state machine.
- Tasks are executed every 125 microseconds, triggered by a timer interrupt flag (`ISR_f_125us`).
- Communication between modules is handled via global variables and function calls.
- No operating system or RTOS is used.
- Interrupt Service Routines (ISRs) set flags, which are polled and cleared in the main loop.

## Project-Specific Conventions

- All time-based logic is based on the 125us tick from Timer0. Delay and scheduling mechanisms rely on counters incremented by ISR flags.
- The watchdog timer (`WDTR = 0x5A`) is cleared frequently in both the main loop and delay routines to prevent unexpected resets.
- Magic numbers (e.g., delay counts) are commonly used and should be documented with comments or macros.
- Error handling is centralized in the `Error_Process()` function.
- The system state is managed via global variables.
- Task switching is explicit: after each task finishes, `current_task` is updated to the next state.

## Integration Points

- Hardware-specific headers (e.g., `<SN8F5602.h>`) must be included to access register definitions.

## Examples & Development Patterns

- To add a new hardware feature, create a corresponding `feature.c` / `feature.h` file and initialize it in `main.c`.
- To add a new scheduled task, update the `TaskType` enum and extend the `switch(current_task)` block in `main.c`.
- All time-related logic should be based on ISR flags and counters—--avoid using busy-wait loops--.

## Key Files

- `main.c` – Main loop, system initialization, and task scheduler.
- `system.c/h` – System-wide flags, timing functions, and error handling.


## Refactor Checklist
- [結構優化] 是否將功能模組化？每個 .c/.h 檔是否只負責單一功能？
- [硬體抽象化] 是否將硬體相關邏輯抽離為 HAL 或 driver 層？
- [重複程式碼] 是否有重複邏輯（條件、流程、初始化）？是否能抽出函式重複使用？
- [魔法數字清除] 是否使用 #define 取代硬編碼的數字或字串？
- [統一命名規則] 函式、變數、結構體命名是否一致且具意義？
- [清除死碼] 是否移除未使用的變數、函式、註解、暫存邏輯？
- [註解與說明] 是否有清楚註解每個函式的用途、輸入、輸出、限制？是否使用 Doxygen 格式？
- [初始化清晰] 全部變數、裝置是否在使用前正確初始化？
- [全域變數最小化] 是否將 global 改為 static、local？是否避免跨檔案共用變數？

## Code Review Checklist
- [註解與文檔] 是否有清楚解釋困難邏輯、異常處理、流程說明？是否註解有用而非冗言？
- [邏輯正確性] 所有條件判斷、回圈、switch 是否完整覆蓋？是否處理好例外狀況？
- [重複邏輯] 是否有複製貼上但邏輯雷同的程式碼？可否抽象為通用函式？
- [錯誤處理] 所有函式是否檢查回傳值？錯誤情況是否正確處理？是否忽略某些錯誤碼？
- [記憶體管理] 是否有記憶體泄漏風險？是否使用 unsafe malloc/free？變數有正確初始化？
- [中斷處理（ISR）] ISR 是否短小、無耗時邏輯？是否使用 volatile？是否與主程式安全互動？
- [效能與時效性] 是否有不必要的 delay、重複計算、忙等（busy wait）？
- [邊界測試] 陣列、buffer、input 是否有長度檢查？是否有防止 overflow/underflow？
- [安全與風險] 是否有不安全的型別轉換？是否強制類型轉型可能導致錯誤？
- [資料競爭與同步] 多工情況下是否保護共用資源（如加鎖、禁止中斷）？