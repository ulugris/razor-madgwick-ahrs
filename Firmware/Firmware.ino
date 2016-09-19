/*
  "9DOF Razor IMU" hardware version: SEN-10736

  ATMega328@3.3V, 8MHz

  ADXL345  : Accelerometer
  HMC5883L : Magnetometer
  ITG-3200 : Gyro

  Arduino IDE : Select board "Arduino Pro or Pro Mini (3.3v, 8Mhz) w/ATmega328"

  Axis definition (differs from definition printed on the board!):
    X axis pointing forward (towards the short edge with the connector holes)
    Y axis pointing to the right
    and Z axis pointing down

  Serial commands that the firmware understands:

  "#o<params>" - Set OUTPUT mode and parameters. The available options are:
      // Identification
      "#oi" - Output ID number (1 byte)

      // Streaming output
      "#o0" - DISABLE continuous streaming output. Also see #f below.
      "#o1" - ENABLE continuous streaming output.

      // Quaternion output
      "#oqb" - Output quaternion in BINARY format (q0, q1, q2, q3, so one output frame is 4x4 = 16 bytes long).
      "#oqt" - Output quaternion in TEXT format (Output frames have form like "q0 = 0.23, q1 = -0.75...",
               followed by carriage return and line feed [\r\n]).
      "#oab" - Output quaternion in BINARY format (q0, q1, q2, q3) plus accelerometer and gyroscope data (40 bytes).
      "#oat" - Output quaternion in TEXT format, plus accelerometer and gyroscope data.

      // Sensor data output
      "#osct" - Output CALIBRATED SENSOR data of all 9 axes in TEXT format.
                One frame consist of three lines - one for each sensor: acc, mag, gyr.
      "#osrt" - Output RAW SENSOR data of all 9 axes in TEXT format.
                One frame consist of three lines - one for each sensor: acc, mag, gyr.
      "#osbt" - Output BOTH raw and calibrated SENSOR data of all 9 axes in TEXT format.
                One frame consist of six lines - like #osrt and #osct combined (first RAW, then CALIBRATED).
                NOTE: This is a lot of number-to-text conversion work for the little 8MHz chip on the Razor boards.
      "#oscb" - Output CALIBRATED SENSOR data of all 9 axes in BINARY format.
                One frame consist of three 3x3 float values = 36 bytes. Order is: acc x/y/z, mag x/y/z, gyr x/y/z.
      "#osrb" - Output RAW SENSOR data of all 9 axes in BINARY format.
                One frame consist of three 3x3 float values = 36 bytes. Order is: acc x/y/z, mag x/y/z, gyr x/y/z.
      "#osbb" - Output BOTH raw and calibrated SENSOR data of all 9 axes in BINARY format.
                One frame consist of 2x36 = 72 bytes - like #osrb and #oscb combined (first RAW, then CALIBRATED).

      // Error message output
      "#oe0" - Disable ERROR message output.
      "#oe1" - Enable ERROR message output.

  "#f" - Request one output frame - useful when continuous output is disabled and updates are
         required in larger intervals only. Though #f only requests one reply, replies are still
         bound to the internal 10ms (100Hz) time raster. So worst case delay that #f can add is 10ms.

  Newline characters are not required. So you could send "#ob#o1", which
  would set binary output mode and enable continuous streaming output at once.

  The status LED will be on if streaming output is enabled and off otherwise.

  Byte order of binary output is little-endian: least significant byte comes first.

  Binary output frames add a header 0xFF byte at the beginning and a checksum byte at the end.
*/

/*****************************************************************/
/*********** USER SETUP AREA! Set your options here! *************/
/*****************************************************************/

// OUTPUT OPTIONS
/*****************************************************************/
// Set your serial port baud rate used to send out data here!
#define OUTPUT__BAUD_RATE 57600

// Output mode definitions
#define OUTPUT__MODE_SENSORS_CALIB 0     // Outputs calibrated sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_RAW 1       // Outputs raw (uncalibrated) sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_BOTH 2      // Outputs calibrated AND raw sensor values for all 9 axes
#define OUTPUT__MODE_QUATERNION 3        // Outputs a quaternion calculated using Madgwick's algorithm
#define OUTPUT__MODE_ALL 4               // Outputs a quaternion plus accelerometer and gyroscope data

// Output format definitions
#define OUTPUT__FORMAT_TEXT 0            // Outputs data as text
#define OUTPUT__FORMAT_BINARY 1          // Outputs data as binary float

// Select your startup output mode and format here
int output_mode = OUTPUT__MODE_QUATERNION;
int output_format = OUTPUT__FORMAT_TEXT;

// Select if serial continuous streaming output is enabled on startup
#define OUTPUT__STARTUP_STREAM_ON true

// Print elapsed time after each I/O loop
#define DEBUG__PRINT_LOOP_TIME false

// If set true, an error message will be output if we fail to read sensor data
// Message format: "!ERR: reading <sensor>", followed by "\r\n".
bool output_errors = false;

// Madgwick's algorithm options
#define SAMPLEFREQ 100.00f  // Sample frequency in Hz
#define BETADEF    0.0300f  // Sensor fusion parameter
#define ZETADEF    0.0025f  // Bias correction gain

/*****************************************************************/
/****************** END OF USER SETUP AREA!  *********************/
/*****************************************************************/

// Pin number of status LED
#define STATUS_LED_PIN 13

#include <Wire.h>
#include <EEPROM.h>

// Unit ID
byte ID = 0;

// Accelerometer calibration data
float Ka[3][3];
float a0[3];

// Magnetometer calibration data
float Km[3][3];
float m0[3];

// Gyroscope calibration data
float Kg[3][3];
float g0[3];

// Sensor variables (acceleration is negated)
static union { float accel[3];  byte abytes[12]; };
static union { float magnet[3]; byte mbytes[12]; };
static union { float gyro[3];   byte gbytes[12]; };
static union { float temp;      byte tbytes[4]; };

// More output-state variables
boolean output_stream_on;
boolean output_single_on;
int num_accel_errors = 0;
int num_magnet_errors = 0;
int num_gyro_errors = 0;

// Madgwick's algorithm variables
float beta = BETADEF;
float zeta = ZETADEF;
float dt = 1.0f / SAMPLEFREQ;
static union { float q[4]; byte qbytes[16]; };

// Timer interrupt variable
volatile bool rdy = false;

// Read all sensor data
void read_sensors() {
  Read_Accel();  // Read accelerometer
  Read_Magnet(); // Read magnetometer
  Read_Gyro();   // Read gyroscope
}

// Apply calibration to raw sensor readings
void apply_calibration() {
  float tmp[3];

  Matrix_Vector_Multiply(tmp, Ka, accel);
  Vector_Add(accel, tmp, a0);

  Matrix_Vector_Multiply(tmp, Km, magnet);
  Vector_Add(magnet, tmp, m0);

  Matrix_Vector_Multiply(tmp, Kg, gyro);
  Vector_Add(gyro, tmp, g0);  
}

// Initial state
void reset_sensor_fusion() {
  read_sensors();
  apply_calibration();
  init_quaternion(accel, magnet);
}

void turn_output_stream_on()
{
  output_stream_on = true;
  digitalWrite(STATUS_LED_PIN, HIGH);
}

void turn_output_stream_off()
{
  output_stream_on = false;
  digitalWrite(STATUS_LED_PIN, LOW);
}

// Blocks until another byte is available on serial port
char readChar()
{
  while (Serial.available() < 1) { } // Block
  return Serial.read();
}

void setup()
{
  // Get unit ID
  int pos = 0;
  EEPROM.get(pos, ID);   pos += sizeof(ID);

  // Get calibration data from EEPROM
  EEPROM.get(pos, Ka);   pos += sizeof(Ka);
  EEPROM.get(pos, a0);   pos += sizeof(a0);
  EEPROM.get(pos, Km);   pos += sizeof(Km);
  EEPROM.get(pos, m0);   pos += sizeof(m0);
  EEPROM.get(pos, Kg);   pos += sizeof(Kg);
  EEPROM.get(pos, g0);

  noInterrupts();

  // Set timer1 interrupt at 100Hz
  TCCR1A = 0;    // Set entire TCCR1A register to 0
  TCCR1B = 0;    // Same for TCCR1B
  TCNT1  = 0;    // Initialize counter value to 0

  // Set compare match register according to output frequency
  OCR1A = round(8e6/(SAMPLEFREQ*8) - 1);

  TCCR1B |= (1 << WGM12);  // Turn on CTC mode
  TCCR1B |= (1 << CS11);   // Set CS11 bit for 8 prescaler
  TIMSK1 |= (1 << OCIE1A); // Enable timer compare interrupt

  interrupts();

  // Init serial output
  Serial.begin(OUTPUT__BAUD_RATE);

  // Init status LED
  pinMode (STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  // Init sensors
  delay(50);  // Give sensors enough time to start
  Wire.begin();
  Accel_Init();
  Magnet_Init();
  Gyro_Init();
  delay(50);  // Give sensors enough time to collect data

  // Read sensors, init Madgwick's algorithm
  reset_sensor_fusion();

  // Init output
#if (OUTPUT__STARTUP_STREAM_ON == false)
  turn_output_stream_off();
#else
  turn_output_stream_on();
#endif
}

// AHRS cycle is controlled by a timer interrupt
ISR(TIMER1_COMPA_vect)
{
  rdy = true;
}

// Main loop
void loop()
{
  // Read incoming control messages
  if (Serial.available() >= 2)
  {
    if (Serial.read() == '#') // Start of new control message
    {
      int command = Serial.read(); // Commands

      if (command == 'f') // request one output _f_rame
        output_single_on = true;
      else if (command == 'o') // Set _o_utput mode
      {
        char output_param = readChar();
        if (output_param == 'q') // Output _q_uaternion
        {
          char format_param = readChar();
          output_mode = OUTPUT__MODE_QUATERNION;
          if (format_param == 't') // Output values as _t_text
            output_format = OUTPUT__FORMAT_TEXT;
          else if (format_param == 'b') // Output values in _b_inary format
            output_format = OUTPUT__FORMAT_BINARY;
        }
        else if (output_param == 'a') // Output _a_ll
        {
          char format_param = readChar();
          output_mode = OUTPUT__MODE_ALL;
          if (format_param == 't') // Output values as _t_text
            output_format = OUTPUT__FORMAT_TEXT;
          else if (format_param == 'b') // Output values in _b_inary format
            output_format = OUTPUT__FORMAT_BINARY;
        }
        else if (output_param == 's') // Output _s_ensor values
        {
          char values_param = readChar();
          char format_param = readChar();
          if (values_param == 'r')  // Output _r_aw sensor values
            output_mode = OUTPUT__MODE_SENSORS_RAW;
          else if (values_param == 'c')  // Output _c_alibrated sensor values
            output_mode = OUTPUT__MODE_SENSORS_CALIB;
          else if (values_param == 'b')  // Output _b_oth sensor values (raw and calibrated)
            output_mode = OUTPUT__MODE_SENSORS_BOTH;

          if (format_param == 't') // Output values as _t_text
            output_format = OUTPUT__FORMAT_TEXT;
          else if (format_param == 'b') // Output values in _b_inary format
            output_format = OUTPUT__FORMAT_BINARY;
        }
        else if (output_param == '0') // Disable continuous streaming output
        {
          turn_output_stream_off();
        }
        else if (output_param == '1') // Enable continuous streaming output
        {
          turn_output_stream_on();
        }
        else if (output_param == 'i') // Send ID
        {
          Serial.print(ID);
        }
        else if (output_param == 'e') // _e_rror output settings
        {
          char error_param = readChar();
          if (error_param == '0') output_errors = false;
          else if (error_param == '1') output_errors = true;
          else if (error_param == 'c') // get error count
          {
            Serial.print("#AMG-ERR:");
            Serial.print(num_accel_errors);
            Serial.print(",");
            Serial.print(num_magnet_errors);
            Serial.print(",");
            Serial.println(num_gyro_errors);
          }
        }
      }
    }
  }

  // Time to read the sensors again?
  if (rdy)
  {
#if DEBUG__PRINT_LOOP_TIME == true    
    long t0 = micros();
#endif

    rdy = false;

    // Update sensor readings
    read_sensors();

    if (output_mode == OUTPUT__MODE_QUATERNION)
    {
      apply_calibration();
      Madgwick(accel, magnet, gyro);

      if (output_stream_on || output_single_on) output_quaternion();
    }
    else if (output_mode == OUTPUT__MODE_ALL)
    {
      apply_calibration();
      Madgwick(accel, magnet, gyro);

      if (output_stream_on || output_single_on) output_all();
    }
    else  // Output sensor values
    {
      if (output_stream_on || output_single_on) output_sensors();
    }

    output_single_on = false;

#if DEBUG__PRINT_LOOP_TIME == true
    Serial.print("Loop time (ms): ");
    Serial.println((float) (micros() - t0) / 1000.0f);
#endif
  }
}
