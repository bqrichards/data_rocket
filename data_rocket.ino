#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>

#define LOG_TO_SD false
#define POLL_RATE 1000

typedef struct vector_3
{
  double x = 0;
  double y = 0;
  double z = 0;
} vector_3;

typedef struct orientation_t
{
  double roll = 0;
  double pitch = 0;
  double heading = 0;
} orientation_t;

/**
 * Sensor data from IMU and barometric pressure sensor
 */
typedef struct rocket_sensor_data
{
  /**
   * Acceleration (m/s^2)
   */
  vector_3 acceleration = {};

  /**
   * Orientation (Quaternion)
   */
  orientation_t orientation = {};

  /**
   * Pressure (Pascals)
   */
  float pressure = 0;

  /**
   * Altitude (m)
   */
  float altitude = 0;

  /**
   * Ground altitude (m)
   */
  float ground_altitude;

  /**
   * Temperature (celsius)
   */
  float temperature = 0;

  /**
   * Latitude
   */
  float lat = -1;

  /**
   * Longitude
   */
  float lon = -1;
} rocket_sensor_data;


Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BMP280 bmp = Adafruit_BMP280();

SoftwareSerial gpsSerial(3, 4); // rx, tx
TinyGPS gps;

rocket_sensor_data sensor_data;

File log_file;

bool end_flight = false;

String data_line;

void calibrate_altitude()
{
  // Measure average altitude on launchpad
  uint8_t sample_count = 50;
  float samples[sample_count];
  uint8_t i = 0;
  float total_altitude = 0;

  // TODO Use Chauvenet's criterion to detect outliers in data

  while (i < sample_count)
  {
    float alt = bmp.readAltitude();
    samples[i] = alt;
    total_altitude += alt;
    i++;
    delay(100);
  }

  sensor_data.ground_altitude = total_altitude / sample_count;
}

void poll_sensors()
{
  sensors_event_t event;
  bno.getEvent(&event);

  // Acceleration
  imu::Vector<3> li_ac = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  sensor_data.acceleration.x = li_ac.x();
  sensor_data.acceleration.y = li_ac.y();
  sensor_data.acceleration.z = li_ac.z();

  // Orientation
  sensor_data.orientation.roll = event.orientation.roll;
  sensor_data.orientation.pitch = event.orientation.pitch;
  sensor_data.orientation.heading = event.orientation.heading;

  // BMP
  // TODO - readTemperature is called in readPressure, readPressure is called in readAltitude.
  // TODO optimize
  sensor_data.temperature = bmp.readTemperature();
  sensor_data.pressure = bmp.readPressure();
  sensor_data.altitude = bmp.readAltitude();

  // GPS
  if (gpsSerial.available() && gps.encode(gpsSerial.read())) {
    gps.f_get_position(&sensor_data.lat, &sensor_data.lon);
  }
}

void print_sensor_data()
{
  Serial.print("Accel X: ");
  Serial.print(sensor_data.acceleration.x);

  Serial.print(", Y: ");
  Serial.print(sensor_data.acceleration.y);

  Serial.print(", Z: ");
  Serial.println(sensor_data.acceleration.z);

  Serial.print("Roll: ");
  Serial.print(sensor_data.orientation.roll);

  Serial.print(", Pitch: ");
  Serial.print(sensor_data.orientation.pitch);

  Serial.print(", Heading: ");
  Serial.println(sensor_data.orientation.heading);

  Serial.print("Temp: ");
  Serial.print(sensor_data.temperature);

  Serial.print(", Pressure: ");
  Serial.print(sensor_data.pressure);

  Serial.print(", Altitude: ");
  Serial.println(sensor_data.altitude);
  Serial.println("");

  Serial.print("Latitude: " + String(sensor_data.lat, 6));
  Serial.print(", Longitude: " + String(sensor_data.lon, 6));
  Serial.println("");
}

void log_sensor_data()
{
  data_line = "";
  data_line += sensor_data.acceleration.x;
  data_line += ",";
  data_line += sensor_data.acceleration.y;
  data_line += ",";
  data_line += sensor_data.acceleration.z;
  data_line += ",";
  data_line += sensor_data.orientation.roll;
  data_line += ",";
  data_line += sensor_data.orientation.pitch;
  data_line += ",";
  data_line += sensor_data.orientation.heading;
  data_line += ",";
  data_line += sensor_data.temperature;
  data_line += ",";
  data_line += sensor_data.pressure;
  data_line += ",";
  data_line += sensor_data.altitude;
  data_line += ",";
  data_line += sensor_data.lat;
  data_line += ",";
  data_line += sensor_data.lon;
  log_file.println(data_line);
}

void check_end_flight_button()
{
  // TODO - check if button is pressed and set end_flight accordingly
}

void setup()
{
  Serial.begin(9600);
  gpsSerial.begin(9600);
  while (!Serial); // wait for serial port to connect. Needed for native USB
  while (!gpsSerial); // wait for gps serial

  Serial.print("Configuring BMP280...");
  if (!bmp.begin())
  {
    Serial.println("failed");
    while (1);
  }
  Serial.println("done");

  delay(1000);

  Serial.print("Configuring BNO055...");
  if (!bno.begin())
  {
    Serial.println("failed");
    while (1);
  }
  Serial.println("done");

  bno.setExtCrystalUse(true);

  delay(1000);

  Serial.print("Calibrating altitude...");
  calibrate_altitude();
  Serial.println("done");
  Serial.print("Ground altitude: ");
  Serial.print(sensor_data.ground_altitude);
  Serial.println("m");

  #if LOG_TO_SD
  Serial.print("Initializing SD card...");
  if (!SD.begin(4)) {
    Serial.println("failed");
    while (1);
  }
  Serial.println("done");

  Serial.print("Opening log file...");
  log_file = SD.open("log_file.txt", FILE_WRITE);
  if (!log_file) {
    Serial.println("failed");
    while(1);
  }
  Serial.println("done");
  #endif

  Serial.println("Ready");
}

void loop()
{
  poll_sensors();
  print_sensor_data();

  #if LOG_TO_SD
  log_sensor_data();
  #endif

  check_end_flight_button();
  
  if (end_flight) {
    log_file.close();
    while (1);
  }

  delay(POLL_RATE);
}
