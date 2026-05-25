# CycleMeter
The first version of this was made with Arduino. The Arduino I was using was too slow so I decided to update the hardware (this was more time consuming that expected).

## Building
- Download nRF5 v17 from https://www.nordicsemi.com/Software-and-tools/Software/nRF5-SDK/Download
- Extract to ../nRF5SDK17/
- Download S212 from https://www.thisisant.com/developer/components/nrf52832#tab_protocol_stacks_tab
- Extract to ../nRF5SDK17/components/softdevice/s212/headers and ../nRF5SDK17/components/softdevice/s212/hex
- Copy custom_board.h and nrf52840_mdk_usb_dongle.h to ../nRF5SDK17/components/boards
- Uncomment #define ANT_LICENSE_KEY "<license-key>" in ../nRF5SDK17/components/softdevice/s212/headers/nrf_sdm.h
- Add network keys to ../nRF5SDK17/components/ant/ant_key_manager/config/ant_key_manager_config.h from https://www.thisisant.com/developer/ant-plus/ant-plus-basics/network-keys

## Building (nRF Connect for VS Code)
- Installed https://pyocd.io/docs/installing
- Set up ANT SDK from here https://github.com/ant-nrfconnect

### Added to settings
```
    "nrf-connect.applicationOptions": {
        "${workspaceFolder}": {
            "flash": {
                "softreset": false,
                "skipBuild": false,
                "extraArgs": ["--runner", "pyocd", "--target", "nrf52840"]
            }
        }
    }
```
### Added to launch
```
{
	"version": "0.2.0",
	"configurations": [
		{
			"type": "nrf-connect",
			"request": "attach",
			"name": "Launch untitled/build",
			"config": "${workspaceFolder}/build",
			"gdbServer": {
                "address": "127.0.0.1:3333",
                "path": "west",
                "args": "debugserver --runner pyocd --target nrf52840 --build-dir ${activeConfig}"
            }
		},
		{
			"type": "nrf-connect",
			"request": "attach",
			"name": "Launch untitled/build/untitled",
			"config": "${workspaceFolder}/build/untitled",
			"gdbServer": {
                "address": "127.0.0.1:3333",
                "path": "west",
                "args": "debugserver --runner pyocd --target nrf52840 --build-dir ${activeConfig}"
            }
		}
	]
}

```



## Hardware
- nRF52840 MDK USB Dongle
- MPU 9520
- BMP280 (two)
- Hall effect sensor (two)

## Hardware Ideas
### Regulator Low-Voltage Cutoff
 - https://www.pololu.com/product/2871
 - https://www.pololu.com/product/2873
