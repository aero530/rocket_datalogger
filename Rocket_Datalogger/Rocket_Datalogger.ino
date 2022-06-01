/*
  rocket data logger
*/


#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <SparkFunLSM9DS1.h>
#include "SparkFunBME280.h"

//
// 11/6/20 - The sparkfun LSM9DS1 library is jank and settings applied in this code are not
// actually applied to the sensor.  I updated the installed library to hard code the values
// I want set.
//
//


// -----------------------------------------------------------------
// init objects
// -----------------------------------------------------------------

LSM9DS1 imu;  // Create an LSM9DS1 object
BME280 bme;

// -----------------------------------------------------------------
// global vars
// -----------------------------------------------------------------

// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG 0x6B // Would be 0x6A if SDO_AG is LOW

// Global variables to keep track of update rates
unsigned long startTime;

// Global variables to print to serial monitor at a steady rate
unsigned long lastPrint = 0;
const unsigned int PRINT_RATE = 10;

const uint8_t ledPin =  13;

uint32_t start, stop;
volatile float f;

bool record = false;
int fileNumber = 0;

char fileName[13] = "data_000.csv";

// -----------------------------------------------------------------
// Setup functions
// -----------------------------------------------------------------

void printSensorReadings(float runTime);
String fileSensorReadings(float runTime);

void setupGyro()
{
  // [enabled] turns the gyro on or off.
  imu.settings.gyro.enabled = true;  // Enable the gyro
  // [scale] sets the full-scale range of the gyroscope.
  // scale can be set to either 245, 500, or 2000
  imu.settings.gyro.scale = 2000; // Set scale to +/-245dps
  // [sampleRate] sets the output data rate (ODR) of the gyro
  // sampleRate can be set between 1-6
  // 1 = 14.9    4 = 238
  // 2 = 59.5    5 = 476
  // 3 = 119     6 = 952
  imu.settings.gyro.sampleRate = 3; // 119Hz ODR
  // [bandwidth] can set the cutoff frequency of the gyro.
  // Allowed values: 0-3. Actual value of cutoff frequency
  // depends on the sample rate. (Datasheet section 7.12)
  imu.settings.gyro.bandwidth = 0;
  // [lowPowerEnable] turns low-power mode on or off.
  imu.settings.gyro.lowPowerEnable = false; // LP mode off
  // [HPFEnable] enables or disables the high-pass filter
  imu.settings.gyro.HPFEnable = true; // HPF disabled
  // [HPFCutoff] sets the HPF cutoff frequency (if enabled)
  // Allowable values are 0-9. Value depends on ODR.
  // (Datasheet section 7.14)
  imu.settings.gyro.HPFCutoff = 1; // HPF cutoff = 4Hz
  // [flipX], [flipY], and [flipZ] are booleans that can
  // automatically switch the positive/negative orientation
  // of the three gyro axes.
  imu.settings.gyro.flipX = false; // Don't flip X
  imu.settings.gyro.flipY = false; // Don't flip Y
  imu.settings.gyro.flipZ = false; // Don't flip Z
}

void setupAccel()
{
  // [enabled] turns the acclerometer on or off.
  imu.settings.accel.enabled = true; // Enable accelerometer
  // [enableX], [enableY], and [enableZ] can turn on or off
  // select axes of the acclerometer.
  imu.settings.accel.enableX = true; // Enable X
  imu.settings.accel.enableY = true; // Enable Y
  imu.settings.accel.enableZ = true; // Enable Z
  // [scale] sets the full-scale range of the accelerometer.
  // accel scale can be 2, 4, 8, or 16
  imu.settings.accel.scale = 8; // Set accel scale to +/-8g.
  // [sampleRate] sets the output data rate (ODR) of the
  // accelerometer. ONLY APPLICABLE WHEN THE GYROSCOPE IS
  // DISABLED! Otherwise accel sample rate = gyro sample rate.
  // accel sample rate can be 1-6
  // 1 = 10 Hz    4 = 238 Hz
  // 2 = 50 Hz    5 = 476 Hz
  // 3 = 119 Hz   6 = 952 Hz
  imu.settings.accel.sampleRate = 3; // Set accel to 119Hz.
  // [bandwidth] sets the anti-aliasing filter bandwidth.
  // Accel cutoff freqeuncy can be any value between -1 - 3.
  // -1 = bandwidth determined by sample rate
  // 0 = 408 Hz   2 = 105 Hz
  // 1 = 211 Hz   3 = 50 Hz
  imu.settings.accel.bandwidth = 0; // BW = 408Hz
  // [highResEnable] enables or disables high resolution
  // mode for the acclerometer.
  imu.settings.accel.highResEnable = false; // Disable HR
  // [highResBandwidth] sets the LP cutoff frequency of
  // the accelerometer if it's in high-res mode.
  // can be any value between 0-3
  // LP cutoff is set to a factor of sample rate
  // 0 = ODR/50    2 = ODR/9
  // 1 = ODR/100   3 = ODR/400
  imu.settings.accel.highResBandwidth = 0;
}

void setupMag()
{
  // [enabled] turns the magnetometer on or off.
  imu.settings.mag.enabled = true; // Enable magnetometer
  // [scale] sets the full-scale range of the magnetometer
  // mag scale can be 4, 8, 12, or 16
  imu.settings.mag.scale = 12; // Set mag scale to +/-12 Gs
  // [sampleRate] sets the output data rate (ODR) of the
  // magnetometer.
  // mag data rate can be 0-7:
  // 0 = 0.625 Hz  4 = 10 Hz
  // 1 = 1.25 Hz   5 = 20 Hz
  // 2 = 2.5 Hz    6 = 40 Hz
  // 3 = 5 Hz      7 = 80 Hz
  imu.settings.mag.sampleRate = 7; // Set OD rate to 80Hz
  // [tempCompensationEnable] enables or disables
  // temperature compensation of the magnetometer.
  imu.settings.mag.tempCompensationEnable = false;
  // [XYPerformance] sets the x and y-axis performance of the
  // magnetometer to either:
  // 0 = Low power mode      2 = high performance
  // 1 = medium performance  3 = ultra-high performance
  imu.settings.mag.XYPerformance = 3; // Ultra-high perform.
  // [ZPerformance] does the same thing, but only for the z
  imu.settings.mag.ZPerformance = 3; // Ultra-high perform.
  // [lowPowerEnable] enables or disables low power mode in
  // the magnetometer.
  imu.settings.mag.lowPowerEnable = false;
  // [operatingMode] sets the operating mode of the
  // magnetometer. operatingMode can be 0-2:
  // 0 = continuous conversion
  // 1 = single-conversion
  // 2 = power down
  imu.settings.mag.operatingMode = 0; // Continuous mode
}

void setupTemperature()
{
  // [enabled] turns the temperature sensor on or off.
  imu.settings.temp.enabled = false;
}

uint16_t initLSM9DS1()
{
  setupGyro(); // Set up gyroscope parameters
  setupAccel(); // Set up accelerometer parameters
  setupMag(); // Set up magnetometer parameters
  setupTemperature(); // Set up temp sensor parameter

  return imu.begin(LSM9DS1_AG, LSM9DS1_M, Wire); // for SPI use beginSPI()
}

// -----------------------------------------------------------------
// Setup
// -----------------------------------------------------------------

void setup() {
  // Open serial communications and wait for port to open:
  //  Serial.begin(115200);
  //  while (!Serial) {
  //    ; // wait for serial port to connect. Needed for native USB port only
  //  }

  // initialize the digital pin as an output.
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  //  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(BUILTIN_SDCARD)) {
    //    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  //  Serial.println("card initialized.");

  for (int i = 0; i <= 999; i++) {
    sprintf(fileName, "data_%03d.csv", i);
    if (SD.exists(fileName)) {
      //Serial.println("removing existing data file");
      //SD.remove(fileName);
      fileNumber = i + 1;
    }
  }
  //reset filename
  sprintf(fileName, "data_%03d.csv", fileNumber);

  Wire.begin();
  Wire.setClock(400000);

  //  Serial.println("Initializing the LSM9DS1");
  uint16_t status = initLSM9DS1();
  //  Serial.print("LSM9DS1 WHO_AM_I's returned: 0x");
  //  Serial.println(status, HEX);
  //  Serial.println("Should be 0x683D");
  //  Serial.println();


  bme.setI2CAddress(0x76); //Connect to a second sensor
  bme.beginI2C();
  bme.setFilter(2); //0 to 4 is valid. Filter coefficient. See 3.4.4
  bme.setStandbyTime(0); //0 to 7 valid. Time between readings. See table 27.
  bme.setTempOverSample(5); //0 to 16 are valid. 0 disables temp sensing. See table 24. // 1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  bme.setPressureOverSample(5); //0 to 16 are valid. 0 disables pressure sensing. See table 23. // 1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  bme.setHumidityOverSample(0); //0 to 16 are valid. 0 disables humidity sensing. See table 19. // 1 through 5, oversampling *1, *2, *4, *8, *16 respectively
  bme.setMode(MODE_NORMAL); //MODE_SLEEP, MODE_FORCED, MODE_NORMAL is valid. See 3.3

  bme.setReferencePressure(102450); //Adjust the sea level pressure used for altitude calculations - compensate alt calculation to get 289m (948') to match google reported value

  startTime = millis();
}

// -----------------------------------------------------------------
// Loop
// -----------------------------------------------------------------

void loop() {

  digitalWrite(ledPin, !digitalRead(ledPin));

  String dataString = "";

  float runTime = (float)(millis() - startTime) / 1000.0;

  imu.readAccel();
  imu.readGyro();
  imu.readMag();

  // Every PRINT_RATE milliseconds, print sensor data:
  if ((lastPrint + PRINT_RATE) < millis())
  {
    //    printSensorReadings(runTime);
    lastPrint = millis();

    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    File dataFile = SD.open(fileName, FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile) {
      dataString = fileSensorReadings(runTime);
      dataFile.println(dataString);
      dataFile.close();
    }
  }
}




// printSensorReadings prints the latest IMU readings
// along with a calculated update rate.
String fileSensorReadings(float runTime)
{
  String outString = "";
  outString += String(runTime, 3);
  outString += ",";

  outString += String(imu.calcAccel(imu.ax), 5);
  outString += ",";
  outString += String(imu.calcAccel(imu.ay), 5);
  outString += ",";
  outString += String(imu.calcAccel(imu.az), 5);
  outString += ",";

  outString += String(imu.calcGyro(imu.gx), 5);
  outString += ",";
  outString += String(imu.calcGyro(imu.gy), 5);
  outString += ",";
  outString += String(imu.calcGyro(imu.gz), 5);
  outString += ",";

  outString += String(imu.calcMag(imu.mx), 5);
  outString += ",";
  outString += String(imu.calcMag(imu.my), 5);
  outString += ",";
  outString += String(imu.calcMag(imu.mz), 5);
  outString += ",";

  outString += String(bme.readFloatPressure(), 5);
  outString += ",";
  outString += String(bme.readFloatAltitudeMeters(), 5);
  outString += ",";
  outString += String(bme.readTempC(), 5);

  return outString;
}

// printSensorReadings prints the latest IMU readings
// along with a calculated update rate.
void printSensorReadings(float runTime)
{
  //  float runTime = (float)(millis() - startTime) / 1000.0;
  //
  //  Serial.println(" ");
  //  Serial.print("T: ");
  //  Serial.print(runTime,3);
  //  Serial.println(" s");
  //
  //  Serial.print("A: ");
  //  Serial.print(imu.calcAccel(imu.ax),5);
  //  Serial.print(", ");
  //  Serial.print(imu.calcAccel(imu.ay),5);
  //  Serial.print(", ");
  //  Serial.print(imu.calcAccel(imu.az),5);
  //  Serial.println(" g");
  //
  //  Serial.print("G: ");
  //  Serial.print(imu.calcGyro(imu.gx),5);
  //  Serial.print(", ");
  //  Serial.print(imu.calcGyro(imu.gy),5);
  //  Serial.print(", ");
  //  Serial.print(imu.calcGyro(imu.gz),5);
  //  Serial.println(" dps");
  //
  //  Serial.print("M: ");
  //  Serial.print(imu.calcMag(imu.mx),5);
  //  Serial.print(", ");
  //  Serial.print(imu.calcMag(imu.my),5);
  //  Serial.print(", ");
  //  Serial.print(imu.calcMag(imu.mz),5);
  //  Serial.println(" Gs");
  //
  //  Serial.print(" Pressure: ");
  //  Serial.print(bme.readFloatPressure(), 5);
  //
  //  Serial.print(" Alt: ");
  //  Serial.print(bme.readFloatAltitudeMeters(), 5);
  //
  //  Serial.print(" Temp: ");
  //  Serial.print(bme.readTempC(), 2);
}
