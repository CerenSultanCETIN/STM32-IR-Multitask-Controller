🛸 IR-Sentinel: High-Performance Multi-Peripheral Control System
A sophisticated embedded systems architecture designed for the STM32F401RE (ARM Cortex-M4). This project demonstrates an advanced approach to real-time peripheral orchestration, featuring synchronized stepper motor drive and zero-flicker display multiplexing without the overhead of an RTOS.

⚡ The Challenge: The Synchronization Dilemma
In standard embedded programming, blocking delays (like HAL_Delay) paralyze the CPU. If the CPU is busy driving a Stepper Motor, the 7-Segment display typically shuts down or flickers. IR-Sentinel solves this by implementing a "Refresh-While-Wait" paradigm, ensuring the UI remains stable even during high-torque mechanical operations.

🚀 Key Features
NEC IR Protocol Engine: Precision decoding of 32-bit infrared signals using hardware timers.

Synchronized Stepper Drive: Precision control of the 28BYJ-48 motor for 1, 2, or 3 full rotations.

Zero-Flicker Multiplexing: High-frequency 4-digit 7-segment display control using Persistence of Vision (POV).

Interactive Audio/Visual UI: * RGB Feedback: Dynamic state changes (Common Cathode).

Audio Alerts: Pre-emptive buzzer notifications synchronized with motor activation.

Timing Optimization: Replaced blocking functions with calibrated micro-delay loops to maximize motor RPM.

🛠 Tech Stack
Language: Embedded C

Framework: STM32 HAL (Hardware Abstraction Layer)

IDE: STM32CubeIDE

Platform: Nucleo-F401RE (Cortex-M4 @ 84MHz)

🔌 Hardware Configuration
Peripheral,Pinout,Interface
IR Receiver,PA10,EXTI / Timer Input
Stepper Motor,"PC4, PC5, PC6, PC8",GPIO Push-Pull
Active Buzzer,PC7,Digital Output
RGB LED,"PB3, PB4, PB10",Digital Output
7-Segment Display,Port A & Port B,4-Digit Multiplexed

🧠 Software Architecture
The system utilizes a specialized Bekle_Ve_Tazele (Wait-and-Refresh) logic. This allows the MCU to utilize the idle time required by the stepper motor's phase transitions to refresh the display, maintaining a consistent ~60Hz+ refresh rate regardless of mechanical load.

// Example of the Non-Blocking Synchronization Logic
void Step_Motor_Sur(int turns) {
    // 1. Synchronized Audio Feedback
    // 2. High-speed Step Sequence
    // 3. Embedded UI Refresh Loops
}

