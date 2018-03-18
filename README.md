# lorabees

Beehive monitoring project with LoRa / P-NUCLEO-LRWAN1 (STM32L073RZ)

This is an experimental student project focused on learning how to use LoRa. It captures some environmental properites, samples audio and computes its frequency spectrum. Then the principal peaks of the spectrum are detected and reported together with the other information to a LoRa gateway. Those packets can be captured using a Node-RED MQTT flow to a file.

## Sensors

- BME680 (Temperature, Relative Humidity, Atmospheric Pressure, Volatile Organic Compounds)
- Microphone for buzzing capture & analysis (electret mic with a transistor-based amplifier)

## Progress

- [x] LoRa integration
- [x] BME680 communication, basic reading
- [x] MBE680 periodic reporting to LoRa
- [ ] Try to use BSEC for better sensor compensation
- [x] Design and build the microphone amplifier
- [x] Microphone capture via DMA and ADC
- [x] Waveform analysis (Fourier transform, peak detection, noise level..)
- [x] Waveform principal characteristics reporting to LoRa
- [x] Data analysis backend (Mosquitto on RPi receiving packets from The Things Network, storing to DB, some graphs)
