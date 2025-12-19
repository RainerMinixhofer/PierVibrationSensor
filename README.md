# PierVibrationSensor
Software and Hardware Info Repository for Vibration Sensor to monitor the pier of my private observatory
The sensor system consists of an MEMS 9-axis motion tracker module the [MPU-9250/6500](https://github.com/NelisW/myOpenHab/blob/master/docs/707-MPU-9250-9265%20IMU.md) and two [LGT-4.5 horizontal Geophones](http://longetequ.com/geophone/3.htm) amplified with an [AD620 instrument amplifier](https://protosupplies.com/product/ad620-instrumentation-amplifier-module/) and converted to digital with the [ADS1256 24bit ADC](https://github.com/Arda-Bildik/ADS1256_library).
Both sensor paths are controlled by an [Raspberry Pico 2 W](), programmed in C++. VSCode with PlatformIO Plugin is used.
It is planned to upload key vibration analysis data via MQTT into IOBroker. See [example](https://github.com/mats-bergstrom/DS18B20) for how to do this in MicroPython.
The pico_pdm_spectrogram.ino c++ file is holding an example code for doing a spectrogram with the pico 2 w. The Github repository is under https://github.com/dpwe/pico_pdm_spectrogram/tree/main
## Hardware
Microcontroller is a Raspberry Pico 2 W with a W5500 Ethernet Hat.