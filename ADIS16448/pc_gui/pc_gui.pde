////////////////////////////////////////////////////////////////////////////////////////////////////////
//  November 2017
//  Author: Juan Jose Chong <juan.chong@analog.com>
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  pc_gui.pde
////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  This Processing project interfaces with an ADIS16448 using a Teensy and 
//  the Arduino serial library to receive IMU data in LSBs from the sensor
//  and save it to a .csv file. Data is transmitted in binary format (8 bits at
//  a time) using the Serial.write() command instead of Serial.print() to reduce 
//  overhead on the Teensy. 

//  Data is transmitted at every data ready pulse from the ADIS16448 and directly 
//  written to the .csv after each frame is successfully received. Commas are added
//  between each word and a carriage return is inserted at the end of every frame.  
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
////////////////////////////////////////////////////////////////////////////////////////////////////////
import processing.serial.*;

PFont f;
Serial arduino;
int header_flag_cnt = 0;
int rxpointer = 0;
PrintWriter output;
int upper = 0;
int lower = 0;
int ulpointer = 0;
int combined = 0;

void setup() {
  size(380, 200);
  f = createFont("Arial",20,true);
  arduino = new Serial(this, Serial.list()[0], 115200);
  arduino.clear();
  output = createWriter("output.csv"); 
}

void keyPressed() {
  output.flush();
  output.close();
  exit();
}

void draw() 
{
  background(255);
  textFont(f,20);
  fill(0);
  textAlign(LEFT);
  text("Press any key to stop recording data...",10,100);
  while (arduino.available () > 0) 
  {
    int inByte = arduino.read();
    //println(header_flag_cnt);
    
    // If header (0xA5) was detected three times in a row
    if (header_flag_cnt == 3) 
    {
      
      rxpointer++;
      
      // Ping-pong upper/lower bytes. If upper byte is read,
      // join bytes into word, convert 2s complement, and 
      // print to file
      if (ulpointer == 0) {
        lower = inByte;
        ulpointer++;
      }
      
      else if (ulpointer == 1) {
        upper = inByte;
        ulpointer--;
        combined = ((lower << 8) | (upper & 0xFF));
        
        // 2's complement conversion
        if (combined > 32767) 
        {
          combined = combined - 65535;
        }        
        
        output.print(combined);
        
        // Print a comma between each word, but not on the
        // last word in the frame
        if (rxpointer != 24) 
        {
        output.print(",");
        }
        
      }
      
      // If we read everything in this frame, reset counter
      // variables and wait for next header
      if (rxpointer == 24) 
      {
      output.println(" ");
      rxpointer = 0;
      header_flag_cnt = 0;
      }
      
    }
    
    // Increment header count flag if not currently processing data
    if (inByte == 0xA5 && header_flag_cnt <= 2) 
    {
      header_flag_cnt++;
    }
    
  }
  
}