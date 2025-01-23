# ACU-25 [STM32 Version]

-----

## Desription
Transferring from Teensy to STM32

----

## Setup

`launch.json`:

    "configurations": [
        {
            "name": "Debug with OpenOCD",
            "cwd": "${workspaceFolder}",
            "executable": "${workspaceFolder}/build/Debug/ACU-25.elf",
            "type": "cortex-debug",
            "device": "STM32G474RE",
            "servertype": "openocd",
            "configFiles": [
                "interface/stlink.cfg",
                "target/stm32g4x.cfg"
            ],
            "searchDir": [],
            "preLaunchTask": "CMake: build",
            "showDevDebugOutput": "none",
            "svdPath": "${workspaceFolder}/.vscode/STM32G474.svd" // optional
        }
    ]
    

`settings.json`:
- add `"vscode-serial-monitor.customBaudRates": [1000000]`


`c_cpp_properties.json`:
- ensure you have the correct compiler path settings to `arm-none-eabi`: `"compilerPath": "/opt/homebrew/bin/arm-none-eabi-gcc",` (MacOS)

## Build & Debug
- Press Build
- Press Debug
    - if Debug doesn't work, open terminal in VSCode and run 
    - `openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "program build/Debug/ACU-25.elf verify reset exit`
    
