// Output quaternion: q0, q1, q2, q3
void output_quaternion()
{
  if (output_format == OUTPUT__FORMAT_BINARY)
  {
    byte XOR = 0;

    for (int i = 0; i < 16; i++)
      XOR ^= qbytes[i];

    Serial.write(0xff);
    Serial.write(qbytes, 16);
    Serial.write(XOR);
  }
  else if (output_format == OUTPUT__FORMAT_TEXT)
  {
    Serial.print("q0 = ");
    Serial.print(q[0]);
    Serial.print(", q1 = ");
    Serial.print(q[1]);
    Serial.print(", q2 = ");
    Serial.print(q[2]);
    Serial.print(", q3 = ");
    Serial.print(q[3]);
    Serial.print(", Tg = ");
    Serial.println((23000 + temp) / 280.0f, 1);
  }
}

// Output quaternion and inertial sensors: q0, q1, q2, q3, ax, ay, az, gx, gy, gz
void output_all()
{
  if (output_format == OUTPUT__FORMAT_BINARY)
  {
    byte XOR = 0;

    for (int i = 0; i < 16; i++)
      XOR ^= qbytes[i];
    for (int i = 0; i < 12; i++)
      XOR ^= abytes[i] ^ gbytes[i];

    Serial.write(0xff);
    Serial.write(qbytes, 16);
    Serial.write(abytes, 12);
    Serial.write(gbytes, 12);
    Serial.write(XOR);
  }
  else if (output_format == OUTPUT__FORMAT_TEXT)
  {
    Serial.print("q0 = ");
    Serial.print(q[0]);
    Serial.print(", q1 = ");
    Serial.print(q[1]);
    Serial.print(", q2 = ");
    Serial.print(q[2]);
    Serial.print(", q3 = ");
    Serial.print(q[3]);
    Serial.print(", A = ");
    Serial.print(accel[0]); Serial.print(",");
    Serial.print(accel[1]); Serial.print(",");
    Serial.print(accel[2]);
    Serial.print(", G = ");
    Serial.print(gyro[0]); Serial.print(",");
    Serial.print(gyro[1]); Serial.print(",");
    Serial.print(gyro[2]); Serial.println();
  }
}

void output_sensors_text(char raw_or_calibrated)
{
  Serial.print("#A-"); Serial.print(raw_or_calibrated); Serial.print('=');
  Serial.print(accel[0]); Serial.print(",");
  Serial.print(accel[1]); Serial.print(",");
  Serial.print(accel[2]); Serial.println();

  Serial.print("#M-"); Serial.print(raw_or_calibrated); Serial.print('=');
  Serial.print(magnet[0]); Serial.print(",");
  Serial.print(magnet[1]); Serial.print(",");
  Serial.print(magnet[2]); Serial.println();

  Serial.print("#G-"); Serial.print(raw_or_calibrated); Serial.print('=');
  Serial.print(gyro[0]); Serial.print(",");
  Serial.print(gyro[1]); Serial.print(",");
  Serial.print(gyro[2]); Serial.println();
}

void output_sensors_binary()
{
  byte XOR = 0;

  for (int i = 0; i < 12; i++)
    XOR ^= abytes[i] ^ mbytes[i] ^ gbytes[i];

  Serial.write(0xff);
  Serial.write(abytes, 12);
  Serial.write(mbytes, 12);
  Serial.write(gbytes, 12);
  Serial.write(XOR);
}

void output_sensors()
{
  if (output_mode == OUTPUT__MODE_SENSORS_RAW)
  {
    if (output_format == OUTPUT__FORMAT_BINARY)
      output_sensors_binary();
    else if (output_format == OUTPUT__FORMAT_TEXT)
      output_sensors_text('R');
  }
  else if (output_mode == OUTPUT__MODE_SENSORS_CALIB)
  {
    // Apply sensor calibration
    apply_calibration();
    
    if (output_format == OUTPUT__FORMAT_BINARY)
      output_sensors_binary();
    else if (output_format == OUTPUT__FORMAT_TEXT)
      output_sensors_text('C');
  }
  else if (output_mode == OUTPUT__MODE_SENSORS_BOTH)
  {
    if (output_format == OUTPUT__FORMAT_BINARY)
    {
      output_sensors_binary();
      apply_calibration();
      output_sensors_binary();
    }
    else if (output_format == OUTPUT__FORMAT_TEXT)
    {
      output_sensors_text('R');
      apply_calibration();
      output_sensors_text('C');
    }
  }
}

