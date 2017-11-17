////////////////////////////////////////////////////////////////////////////////////////////////////////
//  November 2017
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

// Uncomment to enable debug
//#define DEBUG

// Initialize Variables
// Temporary Data Array
uint16_t *burstData;

// Accelerometer
float AXS, AYS, AZS = 0;

// Gyro
float GXS, GYS, GZS = 0;

// Magnetometer
float MXS, MYS, MZS = 0;

// Barometer
float BAROS = 0;

// Control registers
int MSC = 0;
int SENS = 0;
int SMPL = 0;

// Temperature
float TEMPS = 0;

// Delay counter variable
int printCounter = 0;

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
    IMU.regWrite(SMPL_PRD, 0x01), // Disable decimation
    delay(20);

    // Read the control registers once to print to screen
    MSC = IMU.regRead(MSC_CTRL);
    SENS = IMU.regRead(SENS_AVG);
    SMPL = IMU.regRead(SMPL_PRD);

    attachInterrupt(2, grabData, RISING); // Attach interrupt to pin 2. Trigger on the rising edge
}

// Function used to read register values when an ISR is triggered using the IMU's DataReady output
void grabData()
{
    IMU.configSPI(); // Configure SPI before the read. Useful when talking to multiple SPI devices
    burstData = IMU.wordBurst(0); // Read data and insert into array
}

// Function used to scale all acquired data (scaling functions are included in ADIS16448.cpp)
void scaleData()
{
    GXS = IMU.gyroScale(*(burstData + 1)); //Scale X Gyro
    GYS = IMU.gyroScale(*(burstData + 2)); //Scale Y Gyro
    GZS = IMU.gyroScale(*(burstData + 3)); //Scale Z Gyro
    AXS = IMU.accelScale(*(burstData + 4)); //Scale X Accel
    AYS = IMU.accelScale(*(burstData + 5)); //Scale Y Accel
    AZS = IMU.accelScale(*(burstData + 6)); //Scale Z Accel
    MXS = IMU.magnetometerScale(*(burstData + 7)); //Scale X Mag
    MYS = IMU.magnetometerScale(*(burstData + 8)); //Scale Y Mag
    MZS = IMU.magnetometerScale(*(burstData + 9)); //Scale Z Mag
    BAROS = IMU.pressureScale(*(burstData + 10)); //Scale Pressure Sensor
    TEMPS = IMU.tempScale(*(burstData + 11)); //Scale Temp Sensor
}

// Main loop. Print data to the serial port. Sensor sampling is performed in the ISR
void loop()
{
    printCounter ++;
    if (printCounter >= 200000) // Delay for writing data to the serial port
    {
        detachInterrupt(2); //Detach interrupt to avoid overwriting data
        scaleData(); // Scale data acquired from the IMU

        //Clear the serial terminal and reset cursor
        //Only works on supported serial terminal programs (Putty)
        Serial.print("\033[2J");
        Serial.print("\033[H");

        // Print header
        Serial.println(" ");
        Serial.println("ADIS16448 Teensy Burst Read Example Program");
        Serial.println("Juan Chong - November 2017");
        Serial.println(" ");

        // Print control registers to the serial port
        Serial.println("Control Registers");
        Serial.print("MSC_CTRL: ");
        Serial.println(MSC,HEX);
        Serial.print("SENS_AVG: ");
        Serial.println(SENS,HEX);
        Serial.print("SMPL_PRD: ");
        Serial.println(SMPL,HEX);
        Serial.println(" ");
        Serial.println("Raw Output Registers");
        
        // Print scaled gyro data
        Serial.print("XGYRO: ");
        Serial.println(GXS);
        Serial.print("YGYRO: ");
        Serial.println(GYS);
        Serial.print("ZGYRO: ");
        Serial.println(GZS);
      
        // Print scaled accel data
        Serial.print("XACCL: ");
        Serial.println(AXS);
        Serial.print("YACCL: ");
        Serial.println(AYS);
        Serial.print("ZACCL: ");
        Serial.println(AZS);

        // Print scaled magnetometer data
        Serial.print("XMAG: ");
        Serial.println(MXS);
        Serial.print("YMAG: ");
        Serial.println(MYS);
        Serial.print("ZMAG: ");
        Serial.println(MZS);

        // Print scaled barometer data
        Serial.print("BARO: ");
        Serial.println(BAROS);

        // Print scaled temp data
        Serial.print("TEMP: ");
        Serial.println(TEMPS);
        Serial.println(" ");

        Serial.println("Status Registers");

        // Print Status Registers
        Serial.print("DIAG_STAT: ");
        Serial.println((*(burstData + 0)));
        Serial.print("CHECKSUM: ");
        Serial.println((*(burstData + 9)));
 
#ifdef DEBUG 
        // Print unscaled gyro data
        Serial.print("XGYRO: ");
        Serial.println((*(burstData + 1)));
        Serial.print("YGYRO: ");
        Serial.println((*(burstData + 2)));
        Serial.print("ZGYRO: ");
        Serial.println((*(burstData + 3)));
      
        // Print unscaled accel data
        Serial.print("XACCL: ");
        Serial.println((*(burstData + 4)));
        Serial.print("YACCL: ");
        Serial.println((*(burstData + 5)));
        Serial.print("ZACCL: ");
        Serial.println((*(burstData + 6)));
        Serial.println(" ");
       
        // Print unscaled temp data
        Serial.print("TEMP: ");
        Serial.println((*(burstData + 7)));
#endif
        printCounter = 0;
        attachInterrupt(2, grabData, RISING);
    }
}
