# Code Guide - Team Torille

This file contains a minimal guide on how we worked with this code base during the embedded project 1 group project and also how to interpret and navigate the different parts of the codebase.

- [Code Guide - Team Torille](#code-guide---team-torille)
  - [General Information](#general-information)
    - [PlatformIO](#platformio)
    - [Header and Cpp files](#header-and-cpp-files)
  - [Main parts and details](#main-parts-and-details)
    - [`hardware/` - Hardware Abstractions](#hardware---hardware-abstractions)
    - [`control/` - Program logic and control](#control---program-logic-and-control)
      - [`control/state` - State Machine](#controlstate---state-machine)


## General Information

Although ArduinoIDE worked wonderfully for the beginning of the course and was very straight forward, we decided to build our main project inside a proper file and directory-based project structure. As some of the team members wanted to get to know c++ better, this offered a great opportunity to gain some experience and knowledge of working with cpp projects.

We worked in visual studio code and installed some extensions / tools to help with development. Most of it came automatically through installing the PlatformIO extension.

### PlatformIO

We used PlatformIO as well as the PlatformIO extension in visual studio code while developing the project. It offers tons of tools and support for different platforms.
As we didn't have a lot of experience with embedded systems or c++ PlatformIO came in quite handy as it is well documented and helps out with scaffolding the project.

PlatformIO made it easy to build the code, upload it to the arduino and also monitor (serial) the device. It also supports writing to serial. This made it optimal to work with the project, as everything was in one place. Although at the beginning, it needed some minor tweaks and getting used to it, but after that it seemed quite stable.

### Header and Cpp files

In cpp there is the convention to split the class and type definitions and implementations into different files. This was also done in our project to some extent. The definitions can be found in the header files inside the `/include` directory. The implementations are located in `/src`. Some files, which are very general or generic or easy to implement, were directly implemented inline inside the header directory. The main code, as well as the exercises and hardware abstractions were implemented in `/src`.

## Main parts and details

As `/include` directly mirrors the structure of `/src`, the components and concepts are described for the common files / structure here.

The most important directories / domains are the following:

- `/` - Includes top-level files like the main entrypoints or barrel files like `hardware.h`.
- `/src/exercises/final.cpp` - The real main program file, bringing everything together.
- `communication/` - Includes files related to communication and protocols like i2c, serial.
- `control/` - Includes files related to controller decisions, program flow, hardware control.
  - `state/` - Includes files related to state / state-machines in general but also the specific machine states that are used in the final code (drive a distance, control with joystick, turn some angle etc.)
- `hardware/` - Includes abstractions of the different hardware parts. Every part exposes some function to interact with the real life hardware
- `util/` - Includes some helpers and general utility functions

### `hardware/` - Hardware Abstractions

To practice working with cpp, these files were created as wrappers around physical hardware parts. They mostly don't contain a lot of domain logic or complex logic. Some components also have a "builder" (builder design pattern) that goes with it to practice cpp and provide a way to construct complex components.  

All of the hardware components come together in `physical.h`. This is one of the more important files, as it contains the actual instances of the hardware classes. These are used in the main parts of the code.  

These hardware files provide some well defined functionality to interact with the hardware the right way. For example, instead of directly setting the motor power pin to some specific value, the `motor.h` abstraction contains methods to set the power between `-1.0` (= `full power backwards`) and `1.0` (= `full power forwards`). This gets translated by the motor abstraction to control the power and direction.  

The same concept applies to the remaining components such as `compass.h`, `encoder.h`, `joystick.h`, `lcd_display.h` (which extends the arduino built-in lcd but provides some further capabilities) and `pin.h`. `pin.h` is one of the most important low-level components as it's used by almost every other component for communication with the hardware. Here, there was some trouble with handling interrupts and interrupt handles. With the help of some rather hacky cpp and ChatGPT, we could implement a way to add multiple interrupt handles to a specific pin. This mechanism helped later to create more complex systems using callbacks.

### `control/` - Program logic and control

Components in this part have to do with control logic. For example, `motor_driver.h` defines different kinds of ways to control the motor. This includes the main controls: driving by joystick, driving a distance, turning until heading.

Some other components include `command.h` which contains a simple framework to tokenize and parse string commands, and `interrupt_dispatcher.h` with `callback.h`. These two files are responsible for the majority of callback logic. For example, when buttons are pressed or pin voltages change. It improves the built-in `attach_interrupt` by allowing multiple callbacks for one event / pin.

#### `control/state` - State Machine

Files in this directory are part of the main state machine that drives the program flow.

- The state machine is a stack of states.
- The top most state is the currently active state.
- The currently active state receives all program events, gets called every tick and is responsible for transferring to the next state.

This way, very flexible states may be implemented, which change the behaviour of the car (drive until specific distance is reached = `drive state`, turn until heading an angle = `turn state`, handle commands from serial = `command state`)