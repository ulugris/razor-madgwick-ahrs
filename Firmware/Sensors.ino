#define AC_ADDRESS ((byte) 0x53)
#define MG_ADDRESS ((byte) 0x1E)
#define GR_ADDRESS ((byte) 0x68)

void Accel_Init()
{
  Wire.beginTransmission(AC_ADDRESS);
  Wire.write((byte) 0x2D);  // Power register
  Wire.write((byte) 0x08);  // Measurement mode
  Wire.endTransmission();

  Wire.beginTransmission(AC_ADDRESS);
  Wire.write((byte) 0x31);  // Data format register
  Wire.write((byte) 0x0A);  // Set to full resolution ±8g
  Wire.endTransmission();
  
  Wire.beginTransmission(AC_ADDRESS);
  Wire.write((byte) 0x2C);  // Rate register
  Wire.write((byte) 0x0A);  // Set to 100Hz, normal operation
  Wire.endTransmission();
}

void Read_Accel()
{
  int i = 0;
  byte buff[6];  

  Wire.beginTransmission(AC_ADDRESS); 
  Wire.write((byte) 0x32);
  Wire.endTransmission();
  
  Wire.requestFrom(AC_ADDRESS, byte(6));
  while (Wire.available() && (i < 6))
    buff[i++] = Wire.read();
  
  if (i == 6)
  {
    accel[0] = (buff[3] << 8) | buff[2]; // X axis (internal sensor y axis)
    accel[1] = (buff[1] << 8) | buff[0]; // Y axis (internal sensor x axis)
    accel[2] = (buff[5] << 8) | buff[4]; // Z axis (internal sensor z axis)
  }
  else
  {
    num_accel_errors++;
    if (output_errors) Serial.println("!ERR: reading accelerometer");
  }
}

void Magnet_Init()
{
  Wire.beginTransmission(MG_ADDRESS);
  Wire.write((byte) 0x02);  // Mode register
  Wire.write((byte) 0x01);  // Set single measurement mode
  Wire.endTransmission();
}

void Read_Magnet()
{
  int i = 0;
  byte buff[6];

  Wire.beginTransmission(MG_ADDRESS); 
  Wire.write((byte) 0x03);
  Wire.endTransmission();
  
  Wire.requestFrom(MG_ADDRESS, byte(6));
  while (Wire.available() && (i < 6))
    buff[i++] = Wire.read();
  
  if (i == 6)
  {
    magnet[0] = - ((buff[4] << 8) | buff[5]); // X axis (internal sensor -y axis)
    magnet[1] = - ((buff[0] << 8) | buff[1]); // Y axis (internal sensor -x axis)
    magnet[2] = - ((buff[2] << 8) | buff[3]); // Z axis (internal sensor -z axis)

    // Put the device back into single measurement mode
    Wire.beginTransmission(MG_ADDRESS);
    Wire.write((byte) 0x02);
    Wire.write((byte) 0x01);
    Wire.endTransmission();
  }
  else
  {
    num_magnet_errors++;
    if (output_errors) Serial.println("!ERR: reading magnetometer");
  }
}

void Gyro_Init()
{
  Wire.beginTransmission(GR_ADDRESS);
  Wire.write((byte) 0x3E);  // Power Management register
  Wire.write((byte) 0x80);  // Power up reset defaults
  Wire.endTransmission();
  
  Wire.beginTransmission(GR_ADDRESS);
  Wire.write((byte) 0x16);  // DLPF, Full Scale register
  Wire.write((byte) 0x1B);  // Set full-scale range and 42Hz LP filter
  Wire.endTransmission();
  
  Wire.beginTransmission(GR_ADDRESS);
  Wire.write((byte) 0x15);  // Sample Rate Divider register
  Wire.write((byte) 0x09);  // Set sample rate to 100Hz
  Wire.endTransmission();

  Wire.beginTransmission(GR_ADDRESS);
  Wire.write((byte) 0x3E);  // Power Management register
  Wire.write((byte) 0x03);  // Set clock to PLL with z gyro reference
  Wire.endTransmission();
}

void Read_Gyro()
{
  int i = 0;
  byte buff[8];
  
  Wire.beginTransmission(GR_ADDRESS); 
  Wire.write((byte) 0x1B);
  Wire.endTransmission();
  
  Wire.requestFrom(GR_ADDRESS, byte(8));
  while (Wire.available() && (i < 8))
    buff[i++] = Wire.read();
  
  if (i == 8)
  {
    temp = (buff[0] << 8) | buff[1];        // Temperature
    gyro[0] = - ((buff[4] << 8) | buff[5]); // X axis (internal sensor -y axis)
    gyro[1] = - ((buff[2] << 8) | buff[3]); // Y axis (internal sensor -x axis)
    gyro[2] = - ((buff[6] << 8) | buff[7]); // Z axis (internal sensor -z axis)
  }
  else
  {
    num_gyro_errors++;
    if (output_errors) Serial.println("!ERR: reading gyroscope");
  }
}

