// ############################################################################################################################
// ######################################### MAGNETOETER COMPASS TILT COMPENSATED #############################################
// ############################################################################################################################
#include <OLED_I2C_128x64_Monochrome.h>   // Own OLED display library
#include <Wire.h>                         // Arduino I2C-BUS library

#define HMC5883L_Address             0x1E // 7bit address of HMC5883L
#define HMC5883L_Mode_Register       0x02
#define HMC5883L_Continuous_Mode     0x00
#define HMC5883L_Data_Output_Address 0x03 // Address 3 = X MSB register

#define MPU6050_Address              0x68 // 7bit address of MPU6050
#define MPU6050_Power_Register       0x6B
#define MPU6050_Power_Mode      		0
#define MPU6050_Data_Output_Address  0x3B
 
#define MPU6050_GYRO_CONFIG      	 0x1B // Gyroscope configuration 
#define MPU6050_ACCEL_CONFIG     	 0x1C // Accelerometer configuration 

#define MPU6050_Range_2G			 0b00

// Gravit force
float Gravity = 9.80665f;
// Range per digit @ 2G
float Range_Per_Digit = 0.000061f;
/* 
	Range per digit list    Range register list
	-  2G: 0.000061f 		-  2G: 0b00
	-  4G: 0.000122f 		-  4G: 0b01
	-  8G: 0.000244f 		-  8G: 0b10
	- 16G: 0.0004882f 		- 16G: 0b11
*/
// Timers
unsigned long Timer_Serial  = 0;
unsigned long Timer_Display = 0;
byte Data_Block_Counter     = 0;
byte Data_Block_Shifter     = 0;
// ############################################################################################################################
// ######################################### AXIS DATA ########################################################################
// ############################################################################################################################
// Compass data
struct HMC5883LAxisData
{
    int X_Axis, Y_Axis, Z_Axis;
} compass;
// Accelerometer & gyroscope data
struct MPU6050AxisData
{
	float X_Axis, Y_Axis, Z_Axis;
} accelerometer, gyroscope;
// MPU temperature
struct MPU6050Temperature
{
	int internal;
} temperature;
// ############################################################################################################################
// ######################################### SETUP ############################################################################
// ############################################################################################################################
void setup()
{
    // Startup serial and i2c
    Serial.begin(9600);
    Wire.begin();
    // Send the correct register & mode to the compass
    writeHMC5883L(HMC5883L_Mode_Register, HMC5883L_Continuous_Mode);
    // Send register & mode to accelerometer & gyroscope
    writeMPU6050(MPU6050_Power_Register, MPU6050_Power_Mode);
    // Setup the lcd
    lcd.initialize();
    lcd.rotateDisplay180();
    lcd.printString("COMPASS" , 5, 1);
    lcd.printString("X-Axis: ", 1, 3);
    lcd.printString("Y-Axis: ", 1, 4);
    lcd.printString("Z-Axis: ", 1, 5);
    lcd.printString("Angle: " , 1, 7);
}
// ############################################################################################################################
// ######################################### LOOP #############################################################################
// ############################################################################################################################
void loop()
{
	// First of all get the raw data from the HMC5883L module
    readHMC5883L();
    // Then read the raw data from the MPU6050
    readMPU6050();
    // Get the calculated angle
    float Angle = getAngle();
    // Send data depending on device and timer
    // Serial a bit slower than display
    if (millis() > Timer_Serial)
    {
        // Print raw compass data
        Serial.print("\n\n COMPASS DATA BLOCK:      ");
        Serial.print("   X: "); Serial.print(compass.X_Axis);
    	Serial.print(" | Y: "); Serial.print(compass.Y_Axis);
        Serial.print(" | Z: "); Serial.print(compass.Z_Axis);
        // Print raw accelerometer data
        Serial.print("\n ACCELEROMETER DATA BLOCK:");
        Serial.print("   X: "); Serial.print(accelerometer.X_Axis * Range_Per_Digit * Gravity);
    	Serial.print(" | Y: "); Serial.print(accelerometer.Y_Axis * Range_Per_Digit * Gravity);
        Serial.print(" | Z: "); Serial.print(accelerometer.Z_Axis * Range_Per_Digit * Gravity);
        // Print raw gyroscope data
        Serial.print("\n GYROSCOPE DATA BLOCK:    ");
        Serial.print("   X: "); Serial.print(gyroscope.X_Axis);
    	Serial.print(" | Y: "); Serial.print(gyroscope.Y_Axis);
        Serial.print(" | Z: "); Serial.print(gyroscope.Z_Axis);
        // Print the angle
        Serial.print("\n ANGLE DATA BLOCK:        ");
        Serial.print("   Angle: "); Serial.print(Angle);
        // Increase timer
        Timer_Serial = millis() + 4000;
    }
    if (millis() > Timer_Display)
    {
        // Show another data block each 5 seconds
        switch(Data_Block_Shifter)
        {
        	// Compass data
        	case 0: 
        		lcd.printString("     COMPASS    ", 0, 1);
		        lcd.printNumber(long(compass.X_Axis), 9, 3);
		        lcd.printString("   ");
		        lcd.printNumber(long(compass.Y_Axis), 9, 4);
		        lcd.printString("   ");
		        lcd.printNumber(long(compass.Z_Axis), 9, 5);
		        lcd.printString("   ");
		        break;
		    // Accelerometer data
        	case 1:
        		lcd.printString("  ACCELEROMETER ", 0, 1);
		        lcd.printNumber(float(accelerometer.X_Axis * Range_Per_Digit * Gravity), 2, 9, 3);
		        lcd.printString("   ");
		        lcd.printNumber(float(accelerometer.Y_Axis * Range_Per_Digit * Gravity), 2, 9, 4);
		        lcd.printString("   ");
		        lcd.printNumber(float(accelerometer.Z_Axis * Range_Per_Digit * Gravity), 2, 9, 5);
		        lcd.printString("   ");
		        break;
		    // Gyroscope data
        	case 2:
        		lcd.printString("    GYROSCOPE   ",  0, 1);
		        lcd.printNumber(long(gyroscope.X_Axis), 9, 3);
		        lcd.printString("   ");
		        lcd.printNumber(long(gyroscope.Y_Axis), 9, 4);
		        lcd.printString("   ");
		        lcd.printNumber(long(gyroscope.Z_Axis), 9, 5);
		        lcd.printString("   ");
		        break;
    	}
    	// Always show the angle
		lcd.printNumber(float(Angle), 1, 9, 7);
		lcd.printString("   ");
		// Shift data block every 5 seconds
		if (Data_Block_Counter > 50)
		{
			Data_Block_Shifter == 2 ? Data_Block_Shifter = 0 : Data_Block_Shifter++;
			Data_Block_Counter = 0;
		}
		else
		{
			Data_Block_Counter++;
		}
        // Increase timer
        Timer_Display = millis() + 100;
    }
}
// ############################################################################################################################
// ######################################### GET ANGLE ########################################################################
// ############################################################################################################################
float getAngle()
{
	// Get normalized axxis data from raw axis data
	float Normalized_X_Axis = accelerometer.X_Axis * Range_Per_Digit * Gravity;
	float Normalized_Y_Axis = accelerometer.Y_Axis * Range_Per_Digit * Gravity;
	// Get pitch and roll
	float Pitch = asin(-Normalized_X_Axis);
	float Roll  = asin(+Normalized_Y_Axis);
	// Set X_Value & Y_Value
	float X_Value, Y_Value;
	// No tilt compensation used
	if (Roll > 0.78 || Roll < -0.78 || Pitch > 0.78 || Pitch < -0.78)
	{
		X_Value = compass.X_Axis;
		Y_Value = compass.Y_Axis;
	}
	// Tilt compensation
	else
	{
		X_Value = compass.X_Axis * cos(Pitch) + compass.Z_Axis * sin(Pitch);
		Y_Value = compass.X_Axis * sin(Roll) * sin(Pitch) + compass.Y_Axis * cos(Roll) - compass.Z_Axis * sin(Roll) * cos(Pitch);
	}
    /* 
    Calculate the angle:
        - angle (radiant) = atan(-y, x)
        - angle (degrees) = angle (radiant) * (180 / PI)
    */
    float Angle = (atan2(-Y_Value, X_Value)) * (180 / M_PI);
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
// ############################################################################################################################
// ######################################### WRITE HMC5883L ###################################################################
// ############################################################################################################################
void writeHMC5883L(int Parameter_Mode, int Parameter_Data)
{
    Wire.beginTransmission(HMC5883L_Address);
    Wire.write(Parameter_Mode);
    Wire.write(Parameter_Data);
    Wire.endTransmission();
}
void writeHMC5883L(int Parameter_Data)
{
    Wire.beginTransmission(HMC5883L_Address);
    Wire.write(Parameter_Data);
    Wire.endTransmission();
}
// ############################################################################################################################
// ######################################### READ HMC5883L ####################################################################
// ############################################################################################################################
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
    if(6 <= Wire.available())
    {
        compass.X_Axis  = Wire.read() << 8 | Wire.read();
        compass.Z_Axis  = Wire.read() << 8 | Wire.read();
        compass.Y_Axis  = Wire.read() << 8 | Wire.read();
    }
}
// ############################################################################################################################
// ######################################### WRITE MPU6050 ####################################################################
// ############################################################################################################################
void writeMPU6050(int Parameter_Mode, int Parameter_Data)
{
    Wire.beginTransmission(MPU6050_Address);
    Wire.write(Parameter_Mode);
    Wire.write(Parameter_Data);
    Wire.endTransmission();
}
void writeMPU6050(int Parameter_Data)
{
    Wire.beginTransmission(MPU6050_Address);
    Wire.write(Parameter_Data);
    Wire.endTransmission();
}
// ############################################################################################################################
// ######################################### READ MPU6050 #####################################################################
// ############################################################################################################################
void readMPU6050()
{
    // Begin to read the data to the correct address
    writeMPU6050(MPU6050_Data_Output_Address);
    // Send request to read the axis data (there are 14 register to read)
    Wire.requestFrom(MPU6050_Address, 14);
    // Read all 14 registers (each axis has two!)
    if(14 <= Wire.available())
    {
        accelerometer.X_Axis = Wire.read() << 8 | Wire.read();     
  		accelerometer.Y_Axis = Wire.read() << 8 | Wire.read();
  		accelerometer.Z_Axis = Wire.read() << 8 | Wire.read();
  		temperature.internal = Wire.read() << 8 | Wire.read();
  		gyroscope.X_Axis     = Wire.read() << 8 | Wire.read();
  		gyroscope.Y_Axis     = Wire.read() << 8 | Wire.read();
  		gyroscope.Z_Axis     = Wire.read() << 8 | Wire.read();
    }
}
// ############################################################################################################################
// ######################################### END OF CODE ######################################################################
// ############################################################################################################################