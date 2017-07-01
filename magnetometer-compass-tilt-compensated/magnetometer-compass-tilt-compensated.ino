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
float gravity = 9.80665f;
// Range per digit @ 2G
float rangePerDigit = 0.000061f;
/* 
	Range per digit list    Range register list
	-  2G: 0.000061f 		-  2G: 0b00
	-  4G: 0.000122f 		-  4G: 0b01
	-  8G: 0.000244f 		-  8G: 0b10
	- 16G: 0.0004882f 		- 16G: 0b11
*/
// Timers
unsigned long timerSerial = 0;
unsigned long timerDisplay = 0;
byte dataBlockCounter = 0;
byte dataBlockShifter = 0;

// Compass data
struct HMC5883LAxisData
{
    int X, Y, Z;
} compass;
// Accelerometer & gyroscope data
struct MPU6050AxisData
{
	float X, Y, Z;
} accelerometer, gyroscope;
// MPU temperature
struct MPU6050Temperature
{
	int internal;
} temperature;

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
    lcd.printString("angle: " , 1, 7);
}

void loop()
{
	// First of all get the raw data from the HMC5883L module
    readHMC5883L();
    // Then read the raw data from the MPU6050
    readMPU6050();
    // Get the calculated angle
    float angle = getAngle();
    // Send data depending on device and timer
    // Serial a bit slower than display
    if (millis() > timerSerial)
    {
        // Print raw compass data
        Serial.print("\n\n COMPASS DATA BLOCK:      ");
        Serial.print("   X: "); Serial.print(compass.X);
    	Serial.print(" | Y: "); Serial.print(compass.Y);
        Serial.print(" | Z: "); Serial.print(compass.Z);
        // Print raw accelerometer data
        Serial.print("\n ACCELEROMETER DATA BLOCK:");
        Serial.print("   X: "); Serial.print(accelerometer.X * rangePerDigit * gravity);
    	Serial.print(" | Y: "); Serial.print(accelerometer.Y * rangePerDigit * gravity);
        Serial.print(" | Z: "); Serial.print(accelerometer.Z * rangePerDigit * gravity);
        // Print raw gyroscope data
        Serial.print("\n GYROSCOPE DATA BLOCK:    ");
        Serial.print("   X: "); Serial.print(gyroscope.X);
    	Serial.print(" | Y: "); Serial.print(gyroscope.Y);
        Serial.print(" | Z: "); Serial.print(gyroscope.Z);
        // Print the angle
        Serial.print("\n angle DATA BLOCK:        ");
        Serial.print("   angle: "); Serial.print(angle);
        // Increase timer
        timerSerial = millis() + 4000;
    }
    if (millis() > timerDisplay)
    {
        // Show another data block each 5 seconds
        switch(dataBlockShifter)
        {
        	// Compass data
        	case 0: 
        		lcd.printString("     COMPASS    ", 0, 1);
		        lcd.printNumber(long(compass.X), 9, 3);
		        lcd.printString("   ");
		        lcd.printNumber(long(compass.Y), 9, 4);
		        lcd.printString("   ");
		        lcd.printNumber(long(compass.Z), 9, 5);
		        lcd.printString("   ");
		        break;
		    // Accelerometer data
        	case 1:
        		lcd.printString("  ACCELEROMETER ", 0, 1);
		        lcd.printNumber(float(accelerometer.X * rangePerDigit * gravity), 2, 9, 3);
		        lcd.printString("   ");
		        lcd.printNumber(float(accelerometer.Y * rangePerDigit * gravity), 2, 9, 4);
		        lcd.printString("   ");
		        lcd.printNumber(float(accelerometer.Z * rangePerDigit * gravity), 2, 9, 5);
		        lcd.printString("   ");
		        break;
		    // Gyroscope data
        	case 2:
        		lcd.printString("    GYROSCOPE   ",  0, 1);
		        lcd.printNumber(long(gyroscope.X), 9, 3);
		        lcd.printString("   ");
		        lcd.printNumber(long(gyroscope.Y), 9, 4);
		        lcd.printString("   ");
		        lcd.printNumber(long(gyroscope.Z), 9, 5);
		        lcd.printString("   ");
		        break;
    	}
    	// Always show the angle
		lcd.printNumber(float(angle), 1, 9, 7);
		lcd.printString("   ");
		// Shift data block every 5 seconds
		if (dataBlockCounter > 50)
		{
			dataBlockShifter == 2 ? dataBlockShifter = 0 : dataBlockShifter++;
			dataBlockCounter = 0;
		}
		else
		{
			dataBlockCounter++;
		}
        // Increase timer
        timerDisplay = millis() + 100;
    }
}

float getAngle()
{
	// Get normalized axxis data from raw axis data
	float normX = accelerometer.X * rangePerDigit * gravity;
	float normY = accelerometer.Y * rangePerDigit * gravity;
	// Get pitch and roll
	float Pitch = asin(-normX);
	float Roll  = asin(+normY);
	// Set valueX & valueY
	float valueX, valueY;
	// No tilt compensation used
	if (Roll > 0.78 || Roll < -0.78 || Pitch > 0.78 || Pitch < -0.78)
	{
		valueX = compass.X;
		valueY = compass.Y;
	}
	// Tilt compensation
	else
	{
		valueX = compass.X * cos(Pitch) + compass.Z * sin(Pitch);
		valueY = compass.X * sin(Roll) * sin(Pitch) + compass.Y * cos(Roll) - compass.Z * sin(Roll) * cos(Pitch);
	}
    /* 
    Calculate the angle:
        - angle (radiant) = atan(-y, x)
        - angle (degrees) = angle (radiant) * (180 / PI)
    */
    float angle = (atan2(-valueY, valueX)) * (180 / M_PI);
    /*
    Calculate the declination. The formula for this is:
        (DEGREE + (MINUTES / 60)) / (180 / PI);
    You can get your declination on http://magnetic-declination.com/
    Be careful, that you also set it positive or negative.

    The declination for Vienna in Austria is +3°57', and so I set my variables
    */
    byte  decliniationDegree = +3;
    byte  decliniationMinute = 57;
    float Declination = (decliniationDegree + (decliniationMinute / 60)) / (180 / M_PI);
    // Add the declination to the angle
    angle += Declination;
    // Finally correct the angle, so it is shown from 0° to 360° (and not from -180° to +180°)
    if (angle < 0)
        return angle + 360;
    else
        return angle;
}

void writeHMC5883L(int mode, int data)
{
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
    if(6 <= Wire.available())
    {
        compass.X  = Wire.read() << 8 | Wire.read();
        compass.Z  = Wire.read() << 8 | Wire.read();
        compass.Y  = Wire.read() << 8 | Wire.read();
    }
}

void writeMPU6050(int mode, int data)
{
    Wire.beginTransmission(MPU6050_Address);
    Wire.write(mode);
    Wire.write(data);
    Wire.endTransmission();
}
void writeMPU6050(int data)
{
    Wire.beginTransmission(MPU6050_Address);
    Wire.write(data);
    Wire.endTransmission();
}

void readMPU6050()
{
    // Begin to read the data to the correct address
    writeMPU6050(MPU6050_Data_Output_Address);
    // Send request to read the axis data (there are 14 register to read)
    Wire.requestFrom(MPU6050_Address, 14);
    // Read all 14 registers (each axis has two!)
    if(14 <= Wire.available())
    {
        accelerometer.X = Wire.read() << 8 | Wire.read();     
  		accelerometer.Y = Wire.read() << 8 | Wire.read();
  		accelerometer.Z = Wire.read() << 8 | Wire.read();
  		temperature.internal = Wire.read() << 8 | Wire.read();
  		gyroscope.X     = Wire.read() << 8 | Wire.read();
  		gyroscope.Y     = Wire.read() << 8 | Wire.read();
  		gyroscope.Z     = Wire.read() << 8 | Wire.read();
    }
}