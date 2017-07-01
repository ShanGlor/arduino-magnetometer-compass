// #####################################################################################################################
// ######################################### MAGNETOETER COMPASS #######################################################
// ##################################################################################################################### 
#include <OLED_I2C_128x64_Monochrome.h>   // Own OLED display library
#include <Wire.h>                         // Arduino I2C-BUS library

#define HMC5883L_Address             0x1E // 7bit address of HMC5883L
#define HMC5883L_Mode_Register       0x02 // The mode register
#define HMC5883L_Continuous_Mode     0x00 // Select first mode (0: continous, 1: single, 2: idle)
#define HMC5883L_Data_Output_Address 0x03 // Address 3 = X MSB register (continues till Y LSB)

unsigned long timerSerial  = 0;
unsigned long timerDisplay = 0;

struct HMC5883LAxisData
{
    int X, Y, Z;
} compass;

void setup()
{
    // Startup serial and i2c
    Serial.begin(9600);
    Wire.begin();
    // Send the correct register & mode to the compass
    writeHMC5883L(HMC5883L_Mode_Register, HMC5883L_Continuous_Mode);
    // Setup the lcd
    lcd.initialize();
    lcd.rotateDisplay180();
    lcd.printString("COMPASS",  5, 1);
    lcd.printString("X-Axis: ", 2, 3);
    lcd.printString("Y-Axis: ", 2, 4);
    lcd.printString("Z-Axis: ", 2, 5);
    lcd.printString("Angle: ",  2, 7);
}
 
void loop()
{
    // Get the calculated angle
    float Angle = getAngle();
    // Send data depending on device and timer
    // Serial a bit slower than display
    if (millis() > timerSerial)
    {
        // Print it to the serial monitor
        Serial.print("X: ");
        Serial.print(compass.X);
        Serial.print(" | Y: ");
        Serial.print(compass.Y);
        Serial.print(" | Z: ");
        Serial.print(compass.Z);
        Serial.print(" | Angle: ");
        Serial.println(Angle);
        // Increase timer
        timerSerial = millis() + 500;
    }
    if (millis() > timerDisplay)
    {
        // Print it to the display
        lcd.printNumber(long(compass.X), 10, 3);
        lcd.printString("   ");
        lcd.printNumber(long(compass.Y), 10, 4);
        lcd.printString("   ");
        lcd.printNumber(long(compass.Z), 10, 5);
        lcd.printString("   ");
        lcd.printNumber(float(Angle), 1, 10, 7);
        lcd.printString("   ");
        // Increase timer
        timerDisplay = millis() + 100;
    }
}

float getAngle()
{
    // First of all get the raw data from the HMC5883L module
    readHMC5883L();
    /* 
    Calculate the angle:
        - angle (radiant) = atan(-y, x)
        - angle (degrees) = angle (radiant) * (180 / PI)
    */
    float Angle = atan2(-compass.Y, compass.X) * (180 / M_PI);
    /*
    Calculate the declination. The formula for this is:
        (DEGREE + (MINUTES / 60)) / (180 / PI);
    You can get your declination on http://magnetic-declination.com/
    Be careful, that you also set it positive or negative.

    The declination for Vienna in Austria is +3°57', and so I set my variables
    */
    byte  Declination_Degree = +3;
    byte  Declination_Minute = 57;
    float Declination = (Declination_Degree + (Declination_Minute / 60)) / (180 / M_PI);
    // Add the declination to the angle
    Angle += Declination;
    // Finally correct the angle, so it is shown from 0° to 360° (and not from -180° to +180°)
    if (Angle < 0)
        return Angle + 360;
    else
        return Angle;
}

void writeHMC5883L(int mode, int data)
{
    // Mode register and data have to be sent together !
    Wire.beginTransmission(HMC5883L_Address);
    Wire.write(mode);
    Wire.write(data);
    Wire.endTransmission();
}
void writeHMC5883L(int data)
{
    Wire.beginTransmission(HMC5883L_Address);
    Wire.write(data);
    Wire.endTransmission();
}

void readHMC5883L()
{
    // Begin to read the data to the correct address (X MSB register)
    writeHMC5883L(HMC5883L_Data_Output_Address);
    // Send request to read the axis data (there are 6 register to read)
    Wire.requestFrom(HMC5883L_Address, 6);
    /* 
    Finally start reading the data (each axis has two register)
    The register are set the following way: x_msb, x_lsb, z_msb, z_lsb, y_msb, y_lsb
    */
    if (6 <= Wire.available())
    {
        compass.X  = Wire.read() << 8 | Wire.read();
        compass.Z  = Wire.read() << 8 | Wire.read();
        compass.Y  = Wire.read() << 8 | Wire.read();
    }
}