////////////////////////////////////////////////////////////////////////////////////////////////////////
//  September 2016
//  Author: Juan Jose Chong <juan.chong@analog.com>
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADIS16448_Teensy_BurstRead_Example.ino
////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  This Arduino project interfaces with an ADIS16448 using SPI and the 
//  accompanying C++ libraries, reads IMU data in LSBs, scales the data, and 
//  outputs measurements to a serial debug terminal (PuTTY) via the onboard 
//  USB serial port.
//
//  This project has been tested on a PJRC 32-Bit Teensy 3.2 Development Board, 
//  but should be compatible with any other embedded platform with some modification.
//
//  Permission is hereby granted, free of charge, to any person obtaining
//  a copy of this software and associated documentation files (the
//  "Software"), to deal in the Software without restriction, including
//  without limitation the rights to use, copy, modify, merge, publish,
//  distribute, sublicense, and/or sell copies of the Software, and to
//  permit persons to whom the Software is furnished to do so, subject to
//  the following conditions:
//
//  The above copyright notice and this permission notice shall be
//  included in all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
//  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
//  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
//  LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
//  WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//  Pinout for a Teensy 3.2 Development Board
//  RST = D6
//  SCK = D13/SCK
//  CS = D10/CS
//  DOUT(MISO) = D12/MISO
//  DIN(MOSI) = D11/MOSI
//  DR = D2
//
////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <ADIS16448.h>
#include <SPI.h>

// Initialize Variables
// Temporary Data Array
int16_t *burstData;

// Scaled data array
float scaledData[11];

// Call ADIS16448 Class
ADIS16448 IMU(10,2,6); // Chip Select, Data Ready, Reset Pin Assignments

void setup()
{
    Serial.begin(115200); // Initialize serial output via USB
    IMU.configSPI(); // Configure SPI communication
    delay(1000); // Give the part time to start up
    IMU.regWrite(MSC_CTRL, 0x16);  // Enable Data Ready, set polarity
    delay(20); 
    IMU.regWrite(SENS_AVG, 0x402); // Set digital filter
    delay(20);
    IMU.regWrite(SMPL_PRD, 0X01), // Disable decimation
    delay(20);
    attachInterrupt(2, grabData, RISING); // Attach interrupt to pin 2. Trigger on the rising edge
}

// Function used to read register values when an ISR is triggered using the IMU's DataReady output
void grabData()
{
    burstData = IMU.burstRead(); // Read data and insert into array
    scaleData(); // Scale sensor output data
    printtoserial(); // Print data to the serial port
}

// Print burst data to serial port. Data output rate is determined by the IMU decimation rate
void printtoserial()
{
    Serial.print((*(burstData + 0))); // DIAG_STAT
    Serial.print(",");
    Serial.print(scaledData[0]); // Scaled XGYRO
    Serial.print(",");
    Serial.print(scaledData[1]); // Scaled YGYRO
    Serial.print(",");
    Serial.print(scaledData[2]); // Scaled ZGYRO
    Serial.print(",");
    Serial.print(scaledData[3]); // Scaled XACCL
    Serial.print(",");
    Serial.print(scaledData[4]); // Scaled YACCL
    Serial.print(",");
    Serial.print(scaledData[5]); // Scaled ZACCL
    Serial.print(",");
    Serial.print(scaledData[6]); // Scaled XMAG
    Serial.print(",");
    Serial.print(scaledData[7]); // Scaled YMAG
    Serial.print(",");
    Serial.print(scaledData[8]); // Scaled ZMAG
    Serial.print(",");
    Serial.print(scaledData[9]); // Scaled BARO
    Serial.print(",");
    Serial.print(scaledData[10]); // Scaled TEMP
    Serial.print(",");
    Serial.println((*(burstData + 12))); // CHECKSUM
}

// Function used to scale all acquired data (scaling functions are included in ADIS16448.cpp)
void scaleData()
{
    scaledData[0] = IMU.gyroScale(*(burstData + 1)); //Scale X Gyro
    scaledData[1] = IMU.gyroScale(*(burstData + 2)); //Scale Y Gyro
    scaledData[2] = IMU.gyroScale(*(burstData + 3)); //Scale Z Gyro
    scaledData[3] = IMU.accelScale(*(burstData + 4)); //Scale X Accel
    scaledData[4] = IMU.accelScale(*(burstData + 5)); //Scale Y Accel
    scaledData[5] = IMU.accelScale(*(burstData + 6)); //Scale Z Accel
    scaledData[6] = IMU.magnetometerScale(*(burstData + 7)); //Scale X Mag
    scaledData[7] = IMU.magnetometerScale(*(burstData + 8)); //Scale Y Mag
    scaledData[8] = IMU.magnetometerScale(*(burstData + 9)); //Scale Z Mag
    scaledData[9] = IMU.pressureScale(*(burstData + 10)); //Scale Pressure Sensor
    scaledData[10] = IMU.tempScale(*(burstData + 11)); //Scale Temp Sensor
}

// Main loop
void loop()
{
    // Nothing to do here! The program is entirely interrupt-driven!
}
