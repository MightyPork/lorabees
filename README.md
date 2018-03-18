# lorabees

Beehive monitoring project with LoRa / P-NUCLEO-LRWAN1 (STM32L073RZ)

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
