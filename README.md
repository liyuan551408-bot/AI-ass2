# Inverted Pendulum Fuzzy Controller

A C++ university assignment project that controls an inverted pendulum on a moving cart. A fuzzy controller calculates a force from the pole angle, angular velocity, cart position and cart velocity. The program animates the simulation and exports a control-surface data file.

## What it demonstrates

- Combining four measurements into two fuzzy-system inputs.
- A 5 × 5 rule table, membership functions and Sugeno-style force outputs.
- Controller tuning against external left/right pushes.
- Exporting controller output for further analysis.

The controller rules and membership functions are in [fuzzylogic.cpp](A2_windows_VSCode/fuzzylogic.cpp). The simulation, keyboard input and control-surface export are in [main.cpp](A2_windows_VSCode/main.cpp). The included Yamakawa paper is a reference for the rule design.

## Build and run

This project targets Windows. It uses Win32 graphics and a makefile that links `gdi32` statically. Install a compatible MinGW C++ toolchain and run from `A2_windows_VSCode`:

```powershell
mingw32-make
.\main.exe
```

The **left** and **right** arrow keys apply external pushes during the simulation. **Esc** stops the trial. After the simulation, the program generates `data_angle_vs_angle_dot.txt` in the working directory.

## How to explore it

Run once without pressing an arrow key, then try a left or right push. Observe whether the controller keeps the pole upright and the cart within the visible bounds. The exported data can be used to inspect how the output force changes with angle and angular velocity.

## Repository context

This is an assignment repository with supplied simulation code, instructions and supporting documents. The fuzzy controller implementation and tuning are the main areas to inspect. The repository does not include automated tests or a documented non-Windows build.
