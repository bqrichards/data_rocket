#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <SD.h>

#define LOG_TO_SD true
#define GPS false
#define POLL_RATE 1000

const int end_flight_button_pin = 2;

#if GPS
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#endif

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
   * Orientation (roll, pitch, heading)
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

#if GPS
SoftwareSerial gpsSerial(3, 4); // rx, tx
TinyGPS gps;
#endif

rocket_sensor_data sensor_data;

File log_file;

int end_flight = LOW;

String data_line;

void calibrate_altitude()
{
  // Measure average altitude on launchpad
  uint8_t sample_count = 50;
  float samples[sample_count];
  uint8_t i = 0;
  float total_altitude = 0;

  while (i < sample_count)
  {
    float alt = bmp.readAltitude();
    samples[i] = alt;
    total_altitude += alt;
    i++;
    delay(100);
  }

  // TODO Use Chauvenet's criterion to detect outliers in data

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
  // TODO see if optimization is possible
  sensor_data.temperature = bmp.readTemperature();
  sensor_data.pressure = bmp.readPressure();
  sensor_data.altitude = bmp.readAltitude();

  // GPS
#if GPS
  if (gpsSerial.available() && gps.encode(gpsSerial.read())) {
    gps.f_get_position(&sensor_data.lat, &sensor_data.lon);
  }
#endif
}

void print_sensor_data()
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

  Serial.println(data_line);
}

void log_sensor_data()
{
  log_file.println(data_line);
}

void check_end_flight_button()
{
  // Check if button is pressed and set end_flight accordingly
  end_flight = digitalRead(end_flight_button_pin);
}

void setup()
{
  Serial.begin(9600);
#if GPS
  gpsSerial.begin(9600);
#endif
  while (!Serial); // wait for serial port to connect. Needed for native USB
#if GPS
  while (!gpsSerial); // wait for gps serial
#endif

  pinMode(end_flight_button_pin, INPUT);

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
  pinMode(10, OUTPUT);
  if (!SD.begin(10)) {
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
  
  if (end_flight == HIGH) {
    #if LOG_TO_SD
    log_file.close();
    #endif
    Serial.println("Rocket flight finished.");
    while (1);
  }

  delay(POLL_RATE);
}
