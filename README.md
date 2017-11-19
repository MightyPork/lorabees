# lorabees

Beehive monitoring project with LoRa / P-NUCLEO-LRWAN1 (STM32L073RZ)

**This is a seminar project, work in progress. Please DO NOT FORK.**<br>
It's open source but not finished and I don't want broken copies floating around the internet before it's usable.

## Sensors

- BME680 (Temperature, Relative Humidity, Atmospheric Pressure, Volatile Organic Compounds)
- Microphone for buzzing capture & analysis (electret mic with a transistor-based amplifier)

## Progress

- [x] LoRa integration
- [x] BME680 communication, basic reading
- [x] MBE680 periodic reporting to LoRa
- [ ] Try to use BSEC for better sensor compensation
- [x] Design and build the microphone amplifier
- [ ] Microphone capture via DMA and ADC
- [ ] Waveform analysis (Fourier transform, peak detection, noise level..)
- [ ] Waveform principal characteristics reporting to LoRa
- [ ] Data analysis backend (Mosquitto on RPi receiving packets from The Things Network, storing to DB, some graphs)
