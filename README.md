# airquality

## How to upload the software to the ESP32 board

1. Download and install the latest Arduino IDE from https://www.arduino.cc

2. Open the Arduino IDE, and install the ESP32 board definitions.  
How-to steps here: https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md  
More detailed version: https://dronebotworkshop.com/esp32-intro/

3. After you've installed the board definitions, please close the Arduino IDE. Locate the HardwareSerial.cpp file in the ESP32 hardware directory on your computer.  
(Warning: there are multiple HardwareSerial.cpp files for the different board types. Please make sure to locate the one referring to ESP32 boards.)  
The path will look something like this:
...AppData\Local\Arduino15\packages\esp32\hardware\esp32\1.0.4\cores\esp32\ 
Open the file, and replace the pin definitions for the first serial port in the beginning of the file:
Replace this segment:
>#ifndef RX1  
>#define RX1 9  
>#endif  
>  
>#ifndef TX1  
>#define TX1 10  
>#endif  

With this:
>#ifndef RX1  
>#define RX1 14  
>#endif  
>  
>#ifndef TX1  
>#define TX1 13  
>#endif  

4. Save the file, and start the Arduino IDE again. Install the "Adafruit Unified Sensor" and the "Adafruit BME280 library" libraries using the built-in Library Manager (Tools / Manage Libraries).

5. Install Alvaro Valdebenito's PMSerial library from https://github.com/avaldebe/PMserial

6. Restart the IDE, open the BLE_AirQuality_Beacon.ino file, and plug in your ESP32 board to your computer with a micro-USB cable (that also supports data transfer).

7. Select your board type and the port in the Tools menu, and hit the Upload button in the IDE. Congratulations, you have a working AirQuality Beacon!
