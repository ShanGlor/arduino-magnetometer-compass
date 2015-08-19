// ############################################################################################################################
// ######################################### MAGNETOETER COMPASS ##############################################################
// ############################################################################################################################
#include <OLED_I2C_128x64_Monochrome.h>
#include <Wire.h>

#define Compass_Address             0x1E // 7bit address of HMC5883L
#define Compass_Mode_Register       0x02
#define Compass_Continuous_Mode     0x00
#define Compass_Data_Output_Address 0x03 // Address 3 = X MSB register

unsigned long Timer_Serial  = 0;
unsigned long Timer_Display = 0;
// ############################################################################################################################
// ######################################### AXIS DATA ########################################################################
// ############################################################################################################################
struct compassAxisData
{
    int X_Axis, Y_Axis, Z_Axis;
    float Angle;
} compass;
// ############################################################################################################################
// ######################################### SETUP ############################################################################
// ############################################################################################################################
void setup()
{
    // Startup serial and i2c
    Serial.begin(9600);
    Wire.begin();
    // Send the correct register & mode to the compass
    writeCompass(Compass_Mode_Register, Compass_Continuous_Mode);
    // Setup the lcd
    lcd.initialize();
    lcd.rotateDisplay180();
    lcd.printString("COMPASS",  5, 1);
    lcd.printString("X-Axis: ", 2, 3);
    lcd.printString("Y-Axis: ", 2, 4);
    lcd.printString("Z-Axis: ", 2, 5);
    lcd.printString("Angle: ",  2, 7);
}
// ############################################################################################################################
// ######################################### LOOP #############################################################################
// ############################################################################################################################
void loop()
{
    // Get the compass data
    readCompass();
    // Send data depending on device and timer
    // Serial a bit slower than display
    if (millis() > Timer_Serial)
    {
        // Print it to the serial monitor
        Serial.print("X: ");
        Serial.print(compass.X_Axis);
        Serial.print(" | Y: ");
        Serial.print(compass.Y_Axis);
        Serial.print(" | Z: ");
        Serial.print(compass.Z_Axis);
        Serial.print(" | Angle: ");
        Serial.println(compass.Angle);
        // Increase timer
        Timer_Serial = millis() + 500;
    }
    if (millis() > Timer_Display)
    {
        // Print it to the display
        lcd.printNumber(long(compass.X_Axis), 10, 3);
        lcd.printString("   ");
        lcd.printNumber(long(compass.Y_Axis), 10, 4);
        lcd.printString("   ");
        lcd.printNumber(long(compass.Z_Axis), 10, 5);
        lcd.printString("   ");
        lcd.printNumber(float(compass.Angle), 1, 10, 7);
        lcd.printString("   ");
        // Increase timer
        Timer_Display = millis() + 100;
    }
}
// ############################################################################################################################
// ######################################### WRITE COMPASS ####################################################################
// ############################################################################################################################
void writeCompass(int Parameter_Mode, int Parameter_Data)
{
    Wire.beginTransmission(Compass_Address);
    Wire.write(Parameter_Mode);
    Wire.write(Parameter_Data);
    Wire.endTransmission();
}
void writeCompass(int Parameter_Data)
{
    Wire.beginTransmission(Compass_Address);
    Wire.write(Parameter_Data);
    Wire.endTransmission();
}
// ############################################################################################################################
// ######################################### READ COMPASS #####################################################################
// ############################################################################################################################
void readCompass()
{
    // Begin to read the data to the correct address (X MSB register)
    writeCompass(Compass_Data_Output_Address);
    // Send request to read the axis data
    Wire.requestFrom(Compass_Address, 6);
    /* 
    Finally start reading the data (each axis has two register)
    The register are set the following way: x_msb, x_lsb, z_msb, z_lsb, y_msb, y_lsb
    */
    if(6 <= Wire.available())
    {
        compass.X_Axis  = Wire.read() << 8 | Wire.read();
        compass.Z_Axis  = Wire.read() << 8 | Wire.read();
        compass.Y_Axis  = Wire.read() << 8 | Wire.read();
    }
    // Calculate the angle
    compass.Angle = (atan2(-compass.Y_Axis, compass.X_Axis) / M_PI) * 180;
    // Get the angle (from 0 to 359)
    if (compass.Angle < 0)
        compass.Angle += 360;
}
// ############################################################################################################################
// ######################################### END OF CODE ######################################################################
// ############################################################################################################################