# AtTheCore_DesignChallenge_PSoC62S4
 This repo includes the project files implemented on PSoC62S4 pioneer kit for the Infineon AtTheCore design challenge at element14. This involves using capsense, led bar, ambient light and temperature sensor for doing motor control operation by the PWM generated through PSoC 6 peripheral

## Blogs Posted @ Element14 Community
[AtTheCore: Blog #1 - Intro and what's up with the kit!](https://community.element14.com/challenges-projects/design-challenges/at-the-core-design-challenge/b/blog/posts/atthecore_5f00_blog1_5f00_navadeep)
[AtTheCore: Blog #2 - Interfacing LED Bar with ALS](https://community.element14.com/challenges-projects/design-challenges/at-the-core-design-challenge/b/blog/posts/atthecore-blog-2---interfacing-led-bar-with-als)
[AtTheCore: Blog #3 - PWM and Capsense on Cortex M0+](https://community.element14.com/challenges-projects/design-challenges/at-the-core-design-challenge/b/blog/posts/atthecore-blog-3---pwm-and-capsense-on-cortex-m0)
[AtTheCore: Blog #3 - PWM and Capsense on Cortex M0+](https://community.element14.com/challenges-projects/design-challenges/at-the-core-design-challenge/b/blog/posts/atthecore-blog-4---adding-shared-pwm-testing-with-scope-and-all-about-dual-core)
[AtTheCore: Blog #5 - Brining altogether, Multi-Core deployment and Application](https://community.element14.com/challenges-projects/design-challenges/at-the-core-design-challenge/b/blog/posts/atthecore-blog-5---brining-altogether-tmulti-core-deployment-and-application)

## Flowchart

<img src="images/FirmwareFlow_ATCDC.png" width="580" height="580">

## Core Tasks
#### CM0+
- [x] Capsense Touch and Slider
- [x] 2 x PWM generation - shared pins
- [x] UART Transmit - core's message
- [x] Receive IPC Pipe Message - motor control command

#### CM4
- [x] Ambient Light Sensor(ALS) and Temperature Sensor
- [x] LED Bar Interface - sensor data indication
- [x] UART Transmit - core's message
- [x] Transmit IPC Pipe Message - motor control command

## Build Logs:
> Initializing build: proj_cm0p Debug APP_CY8CKIT-062S4 GCC_ARM

Constructing build rules...
Build rules construction complete

==============================================================================
= Building application =
==============================================================================
Generating compilation database file...
-> ./build/compile_commands.json
Compilation database file generation complete
Building 206 file(s)
Linking output file proj_cm0p.elf
D:/Softwares/ModusToolbox/tools_3.0/gcc/bin/arm-none-eabi-objcopy -O ihex C:/Users/Navadeep/Desktop/AtTheCore/FW/CAPSENSE_on_CM0p/proj_cm0p/build/APP_CY8CKIT-062S4/Debug/proj_cm0p.elf C:/Users/Navadeep/Desktop/AtTheCore/FW/CAPSENSE_on_CM0p/proj_cm0p/build/APP_CY8CKIT-062S4/Debug/proj_cm0p.hex
cp C:/Users/Navadeep/Desktop/AtTheCore/FW/CAPSENSE_on_CM0p/proj_cm0p/build/APP_CY8CKIT-062S4/Debug/proj_cm0p.hex ../build/project_hex/proj_cm0p.hex
==============================================================================
= Build complete =
==============================================================================

Calculating memory consumption: CY8C6244LQI-S4D92 GCC_ARM

   ---------------------------------------------------- 
  | Section Name         |  Address      |  Size       | 
   ---------------------------------------------------- 
  | .text                |  0x10000000   |  32152      | 
  | .ARM.exidx           |  0x10007d98   |  8          | 
  | .copy.table          |  0x10007da0   |  24         | 
  | .zero.table          |  0x10007db8   |  8          | 
  | .data                |  0x08000080   |  1256       | 
  | .cy_sharedmem        |  0x08000568   |  24         | 
  | .noinit              |  0x08000580   |  140        | 
  | .bss                 |  0x0800060c   |  1096       | 
  | .heap                |  0x08000a58   |  58792      | 
   ---------------------------------------------------- 

  Total Internal Flash (Available)          262144     
  Total Internal Flash (Utilized)           33480      

> Initializing build: proj_cm4 Debug APP_CY8CKIT-062S4 GCC_ARM

Constructing build rules...
Build rules construction complete

==============================================================================
= Building application =
==============================================================================
Generating compilation database file...
-> ./build/compile_commands.json
Compilation database file generation complete
Building 191 file(s)
Linking output file proj_cm4.elf
D:/Softwares/ModusToolbox/tools_3.0/gcc/bin/arm-none-eabi-objcopy -O ihex C:/Users/Navadeep/Desktop/AtTheCore/FW/CAPSENSE_on_CM0p/proj_cm4/build/APP_CY8CKIT-062S4/Debug/proj_cm4.elf C:/Users/Navadeep/Desktop/AtTheCore/FW/CAPSENSE_on_CM0p/proj_cm4/build/APP_CY8CKIT-062S4/Debug/proj_cm4.hex
cp C:/Users/Navadeep/Desktop/AtTheCore/FW/CAPSENSE_on_CM0p/proj_cm4/build/APP_CY8CKIT-062S4/Debug/proj_cm4.hex ../build/project_hex/proj_cm4.hex
==============================================================================
= Build complete =
==============================================================================

Calculating memory consumption: CY8C6244LQI-S4D92 GCC_ARM

   ---------------------------------------------------- 
  | Section Name         |  Address      |  Size       | 
   ---------------------------------------------------- 
  | .text                |  0x10010000   |  37680      | 
  | .ARM.exidx           |  0x10019330   |  8          | 
  | .copy.table          |  0x10019338   |  24         | 
  | .zero.table          |  0x10019350   |  8          | 
  | .data                |  0x080102fc   |  1652       | 
  | .cy_sharedmem        |  0x08010970   |  8          | 
  | .noinit              |  0x08010978   |  228        | 
  | .bss                 |  0x08010a5c   |  1188       | 
  | .heap                |  0x08010f00   |  55552      | 
   ---------------------------------------------------- 

  Total Internal Flash (Available)          262144     
  Total Internal Flash (Utilized)           39388      
