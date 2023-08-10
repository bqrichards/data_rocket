#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <SD.h>

#define LOG_TO_SD false
#define GPS false

#if GPS
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#endif

const int end_flight_button_pin = 2;

struct vector_3
{
  /** x, roll if orientation */
  float x = 0;

  /** y, pitch if orientation */
  float y = 0;

  /** z, heading if orientation */
  float z = 0;
};

/**
 * Sensor data from IMU and barometric pressure sensor
 */
struct rocket_sensor_data
{
  /** Timestamp (using Arduino's millis function) */
  unsigned long timestamp;
  
  /**
   * Acceleration (m/s^2)
   */
  struct vector_3 acceleration = {};

  /**
   * Orientation (roll, pitch, heading)
   */
  struct vector_3 orientation = {};

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
};

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BMP280 bmp = Adafruit_BMP280();

#if GPS
SoftwareSerial gpsSerial(3, 4); // rx, tx
TinyGPS gps;
#endif

#if LOG_TO_SD
File log_file;
#endif

/** Poll rate (in ms) */
const bool pollRate = 1000;

struct rocket_sensor_data sensor_data;

/** Input to end flight recording */
int end_flight = LOW;

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

  // Timestamp
  sensor_data.timestamp = millis();

  // Acceleration
  imu::Vector<3> li_ac = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  sensor_data.acceleration.x = li_ac.x();
  sensor_data.acceleration.y = li_ac.y();
  sensor_data.acceleration.z = li_ac.z();

  // Orientation
  sensor_data.orientation.x = event.orientation.roll;
  sensor_data.orientation.y = event.orientation.pitch;
  sensor_data.orientation.z = event.orientation.heading;

  // BMP
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

void output_sensor_headers(Print *printer)
{
  printer->println("Timestamp,AccelX,AccelY,AccelZ,Roll,Pitch,Heading,Temp,Pressure,Altitude,Lat,Long");
}

void output_sensor_data(Print *printer)
{
  printer->print(sensor_data.timestamp);
  printer->print(",");
  printer->print(sensor_data.acceleration.x);
  printer->print(",");
  printer->print(sensor_data.acceleration.y);
  printer->print(",");
  printer->print(sensor_data.acceleration.z);
  printer->print(",");
  printer->print(sensor_data.orientation.x);
  printer->print(",");
  printer->print(sensor_data.orientation.y);
  printer->print(",");
  printer->print(sensor_data.orientation.z);
  printer->print(",");
  printer->print(sensor_data.temperature);
  printer->print(",");
  printer->print(sensor_data.pressure);
  printer->print(",");
  printer->print(sensor_data.altitude);
  printer->print(",");
  printer->print(sensor_data.lat);
  printer->print(",");
  printer->print(sensor_data.lon);
  printer->println("");
}

void check_end_flight_button()
{
  // Check if button is pressed and set end_flight accordingly
  end_flight = digitalRead(end_flight_button_pin);
}

void setup()
{
  Serial.println("Begin program");
  
  Serial.begin(9600);
#if GPS
  Serial.println("Starting GPS");
  gpsSerial.begin(9600);
#endif
  while (!Serial); // wait for serial port to connect. Needed for native USB
#if GPS
  Serial.println("Waiting for GPS");
  while (!gpsSerial); // wait for gps serial
#endif

  pinMode(end_flight_button_pin, INPUT);

  Serial.print("Configuring BMP280 Pressure Sensor...");
  if (!bmp.begin())
  {
    Serial.println("failed");
    while (true);
  }
  Serial.println("done");

  delay(1000);

  Serial.print("Configuring BNO055 IMU...");
  if (!bno.begin())
  {
    Serial.println("failed");
    while (true);
  }
  Serial.println("done");

  bno.setExtCrystalUse(true);

  delay(1000);

  Serial.println("Calibrating altitude...");
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
    while (true);
  }
  Serial.println("done");

  Serial.print("Opening log file...");
  log_file = SD.open("log_file.txt", FILE_WRITE);
  if (!log_file) {
    Serial.println("failed");
    while (true);
  }
  Serial.println("done");
#endif

  Serial.println("Setup finished");

  // Print data column headers
  output_sensor_headers(&Serial);

#if LOG_TO_SD
  output_sensor_headers(&log_file);
#endif
}

void loop()
{
  poll_sensors();
  output_sensor_data(&Serial);

#if LOG_TO_SD
  output_sensor_data(&log_file);
#endif

  check_end_flight_button();
  
  if (end_flight == HIGH) {
#if LOG_TO_SD
    log_file.close();
#endif
    Serial.println("Rocket flight finished.");
    while (true);
  }

  delay(pollRate);
}
