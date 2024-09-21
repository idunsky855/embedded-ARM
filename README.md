# STM32L552ZETQ Embedded ARM Project

This project is developed for the STM32L552ZETQ board, based on an ARM processor. It implements a multi-state system controlled through timer interrupts and user button inputs. The system transitions between four distinct states, simulating various lighting patterns using LEDs.

---

## Project Overview

The project runs on the STM32L552ZETQ board and demonstrates the following functionalities:
- **Interrupt-driven State Control**: All state transitions and actions are triggered via timer interrupts and user button presses.
- **LED Light Simulations**: Each state controls the behavior of LED lights to simulate different scenarios.

---

## States

1. **Traffic Light Simulation**: 
   - Standard traffic light sequence using LEDs: red, orange, and green.
2. **Formula 1 Start Light**:
   - Left light on, then left and middle lights, then all three, followed by all lights off to indicate "Go!"
3. **Binary Counter**:
   - The system counts from 0 to 7 in binary using three LEDs.
4. **Crosswalk Light**:
   - Displays red for "Stop" and green for "Walk" using LEDs.

---

[![YouTube video example](https://img.youtube.com/vi/Dkr52m_smfs/0.jpg)](https://www.youtube.com/watch?v=Dkr52m_smfs)

---

## How to Run

1. Flash the project onto the STM32L552ZETQ board using your preferred flashing tool.
2. Press the **User Button** to cycle through the four states.
3. Observe the LED lights change according to the active state.

---

## Dependencies

- STM32CubeIDE or similar for development.
- STM32 HAL libraries for ARM processors.
- An STM32L552ZETQ development board.

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

