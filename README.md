# esphome_smabluetooth
esphome component to read sma inverter via bluetooth
- alpha quality version, seems to work now on SB3000TL-20, and with known limitations on SB1600TL-10 (with BT module)
- a bit better then the development version, i have 2 ESP32 modules running for half a day without restarts
- the goal is still to have modified BluetoothSerial version that either
  - operates asynchroneously (especially for the connect(mac) )
  - or has a built in function to yield or App wdt() enough to avoid the watchdog to be triggered
 
- please note that only the the ESP32 variants support the Bluetooth Serial (classical bluetooth) protocol.
-   that **includes** ESP32-S, ESP32-U, ESP32-D, WROVER, WROOM variants
-   but **excludes** the ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C6 variants
