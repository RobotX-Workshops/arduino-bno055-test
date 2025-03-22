#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);

Summary Table
Parameter	What It Measures	Units	Use Case
Orientation (Euler)	Roll, pitch, yaw angles	Degrees	Basic orientation
Acceleration	Total acceleration (motion + gravity)	m/s²	Motion and tilt detection
Gyroscope	Angular velocity	deg/s	Rotation speed
Magnetometer	Magnetic field strength	µT	Heading relative to north
Linear Acceleration	Motion-only acceleration	m/s²	Pure movement detection
Gravity	Gravity component	m/s²	Tilt relative to ground
Quaternion	3D orientation	None	Precise orientation
Temperature	Sensor temperature	°C	Environmental monitoring
Calibration	Calibration status (0-3)	None	Data reliability

void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
}

void loop() {
  // Declare event structures
  sensors_event_t orientationData, accelerationData, gyroData, magData, linearAccelData, gravityData;
  
  // Get all sensor data
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&accelerationData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&magData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
  imu::Quaternion quat = bno.getQuat();
  int8_t temp = bno.getTemp();
  uint8_t system, gyro, accel, mag;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  // >>> Uncomment the section you are interested in <<<

// Orientation (Euler Angles)
// // What it is: Euler angles describe the sensor’s orientation in terms of three angles: roll (x), pitch (y), and yaw/heading (z), measured in degrees.
// // Physical Meaning:
// // Roll: Rotation around the x-axis (tilting side to side).
// // Pitch: Rotation around the y-axis (tilting forward or backward).
// // Yaw: Rotation around the z-axis (turning left or right relative to a reference direction, like magnetic north).
// // Units: Degrees (°).
// // Use: Tells you how the sensor is tilted or oriented relative to a fixed frame. Note that Euler angles can suffer from gimbal lock (a loss of one degree of freedom in certain orientations), which is why quaternions are also provided.
  // Serial.print("Orientation (deg): ");
  // Serial.print(orientationData.orientation.x); Serial.print(", ");
  // Serial.print(orientationData.orientation.y); Serial.print(", ");
  // Serial.print(orientationData.orientation.z); Serial.println();

  Serial.print("Acceleration (m/s²): ");
// What it is: Raw acceleration data from the accelerometer along the x, y, and z axes.
// Physical Meaning: Measures the total acceleration the sensor experiences, which includes:
// Acceleration due to movement (e.g., if you shake the sensor).
// Acceleration due to gravity (9.81 m/s² when stationary, pointing downward).
// Units: Meters per second squared (m/s²).
// Use: Useful for detecting motion or tilt, but you need to separate gravity from movement (see Linear Acceleration below).
  Serial.print(accelerationData.acceleration.x); Serial.print(", ");
  Serial.print(accelerationData.acceleration.y); Serial.print(", ");
  Serial.print(accelerationData.acceleration.z); Serial.print(", ");

  // Serial.print("Gyroscope (deg/s): ");
// // Physical Meaning: Measures how fast the sensor is rotating around each axis. For example:
// // If gyroData.gyro.x is 10 deg/s, the sensor is rotating around the x-axis at 10 degrees per second.
// // If all values are 0, the sensor is not rotating.
// // Units: Degrees per second (deg/s).
// // Use: Tracks rotational speed, not the actual orientation. It’s great for detecting changes in orientation over time but doesn’t tell you the current position without additional processing.
  // Serial.print(gyroData.gyro.x); Serial.print(", ");
  // Serial.print(gyroData.gyro.y); Serial.print(", ");
  // Serial.print(gyroData.gyro.z); Serial.println();

  // Serial.print("Magnetometer (µT): ");
// // Physical Meaning: Measures the local magnetic field, including Earth’s magnetic field, in microteslas (µT). This helps determine the sensor’s heading relative to magnetic north.
// // Units: Microteslas (µT).
// // Use: Provides an absolute reference for yaw in orientation calculations, though it can be affected by nearby magnetic interference (e.g., metal objects).
  // Serial.print(magData.magnetic.x); Serial.print(", ");
  // Serial.print(magData.magnetic.y); Serial.print(", ");
  // Serial.print(magData.magnetic.z); Serial.println();

  // Serial.print("Linear Accel (m/s²): ");
// // What it is: Acceleration due to movement only, with the gravity component removed.
// // Physical Meaning: Represents the sensor’s motion (e.g., if you move it forward or shake it) without the influence of gravity.
// // Units: Meters per second squared (m/s²).
// // Use: Useful for detecting pure motion, like in robotics or motion tracking, without worrying about the sensor’s tilt.
  // Serial.print(linearAccelData.acceleration.x); Serial.print(", ");
  // Serial.print(linearAccelData.acceleration.y); Serial.print(", ");
  // Serial.print(linearAccelData.acceleration.z); Serial.println();

  // Serial.print("Gravity (m/s²): ");
// // What it is: The component of acceleration due to gravity along the x, y, and z axes.
// // Physical Meaning: Shows the direction and magnitude of gravity (typically 9.81 m/s² when stationary), which depends on the sensor’s orientation. For example, if the sensor is flat, gravity will be entirely along the z-axis.
// // Units: Meters per second squared (m/s²).
// // Use: Helps determine the sensor’s tilt relative to the ground and is subtracted from raw acceleration to get linear acceleration.
  // Serial.print(gravityData.acceleration.x); Serial.print(", ");
  // Serial.print(gravityData.acceleration.y); Serial.print(", ");
  // Serial.print(gravityData.acceleration.z); Serial.println();

  // Serial.print("Quaternion: ");
// // What it is: A mathematical representation of the sensor’s orientation using four components: w, x, y, and z.
// // Physical Meaning: Describes the sensor’s current 3D orientation relative to a reference frame. Unlike Euler angles, quaternions avoid gimbal lock and are more stable for complex rotations.
// // Units: Dimensionless (values typically range from -1 to 1).
// // Use: Provides a complete and reliable way to represent orientation, often used in 3D graphics, robotics, or when converting to other formats like Euler angles.
  // Serial.print("W:"); Serial.print(quat.w()); Serial.print(", ");
  // Serial.print("X:"); Serial.print(quat.x()); Serial.print(", ");
  // Serial.print("Y:"); Serial.print(quat.y()); Serial.print(", ");
  // Serial.print("Z:"); Serial.print(quat.z()); Serial.println();

  // Serial.print("Temperature (C): ");
// // What it is: The internal temperature of the BNO055 sensor.
// // Physical Meaning: Measures the sensor’s temperature, which can affect calibration or performance.
// // Units: Degrees Celsius (°C).
// // Use: Monitoring to ensure the sensor operates within its specified range or to detect environmental changes.
  // Serial.println(temp);

  Serial.print("Calibration (sys, gyro, accel, mag): ");
// What it is: Calibration levels for the system, gyroscope, accelerometer, and magnetometer.
// Physical Meaning: Each value ranges from 0 (uncalibrated) to 3 (fully calibrated):
// System: Overall fusion algorithm calibration.
// Gyro: Gyroscope calibration.
// Accel: Accelerometer calibration.
// Mag: Magnetometer calibration.
// Use: Ensures the sensor’s data is reliable. You should wait for higher calibration levels (ideally 3) for accurate readings.

// How to Fix It: Calibrate the Accelerometer
// Since the gyroscope and magnetometer are already at 3, the key is to calibrate the accelerometer. Here’s how:
// 
// Position the Sensor in Multiple Orientations:
// The accelerometer calibrates by measuring gravity in different directions.
// Place the sensor in at least six stable positions, such as:
// Flat on a table (z-axis up).
// On its left side (x-axis up).
// On its front (y-axis up).
// And the inverted versions (e.g., z-axis down).
// Hold each position steady for a few seconds so the sensor can adjust.
// Keep It Still:
// Avoid moving or shaking the sensor during these steps. Any motion can mess up the calibration.
  Serial.print(system); Serial.print(", ");
  Serial.print(gyro); Serial.print(", ");
  Serial.print(accel); Serial.print(", ");
  Serial.print(mag); Serial.print(", ");

  Serial.println();

  // Control sampling rate
  delay(100); // 100ms delay for ~10Hz sampling
}