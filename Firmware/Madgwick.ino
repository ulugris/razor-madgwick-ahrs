void Madgwick(float a[3], float m[3], float g[3])
{
  float st[4];
  float qd[4];
  float gm[3];
  float q00, q01, q02, q03, q11, q12, q13, q22, q23, q33;
  float A11, A22, A12, A13, A21, A23, A1a, A2a, A2m, J11, J12, J13, J14;
  static float g0[3] = {0.0f};
  static float gb[3] = {0.0f};

  for (int i = 0; i < 3; i++)
  {
    gm[i] = 0.5f * (g[i] + g0[i]); // Mean angular velocity (trapezoidal rule integration)
    g0[i] = g[i];                  // Store angular velocity for next step
  }

  // Compute step direction (gradient descent)
  q00 = q[0] * q[0];
  q01 = q[0] * q[1];
  q02 = q[0] * q[2];
  q03 = q[0] * q[3];
  q11 = q[1] * q[1];
  q12 = q[1] * q[2];
  q13 = q[1] * q[3];
  q22 = q[2] * q[2];
  q23 = q[2] * q[3];
  q33 = q[3] * q[3];

  A11 = 0.5f * (q00 + q11 - q22 - q33);
  A22 = 0.5f * (q00 - q11 + q22 - q33);

  A12 = q12 - q03;
  A13 = q13 + q02;
  A21 = q12 + q03;
  A23 = q23 - q01;

  A1a = A11*a[0] + A12*a[1] + A13*a[2];
  A2a = A21*a[0] + A22*a[1] + A23*a[2];
  A2m = A21*m[0] + A22*m[1] + A23*m[2];

  J11 = a[0]*q[0] - a[1]*q[3] + a[2]*q[2];
  J12 = a[0]*q[1] + a[1]*q[2] + a[2]*q[3];
  J13 =-a[0]*q[2] + a[1]*q[1] + a[2]*q[0];
  J14 =-a[0]*q[3] - a[1]*q[0] + a[2]*q[1];

  st[0] = A1a*J11 - A2a*J14 + A2m*(m[0]*q[3] + m[1]*q[0] - m[2]*q[1]);
  st[1] = A1a*J12 - A2a*J13 + A2m*(m[0]*q[2] - m[1]*q[1] - m[2]*q[0]);
  st[2] = A1a*J13 + A2a*J12 + A2m*(m[0]*q[1] + m[1]*q[2] + m[2]*q[3]);
  st[3] = A1a*J14 + A2a*J11 + A2m*(m[0]*q[0] - m[1]*q[3] + m[2]*q[2]);

  // Normalize step direction
  quatNormalize(st);

  // Update and apply gyro bias estimation
  if (zeta != 0.0f)
  {
    gb[0] += 2.0f * zeta * dt * (q[0]*st[1] - q[1]*st[0] - q[2]*st[3] + q[3]*st[2]);
    gb[1] += 2.0f * zeta * dt * (q[0]*st[2] + q[1]*st[3] - q[2]*st[0] - q[3]*st[1]);
    gb[2] += 2.0f * zeta * dt * (q[0]*st[3] - q[1]*st[2] + q[2]*st[1] - q[3]*st[0]);

    gm[0] -= gb[0];
    gm[1] -= gb[1];
    gm[2] -= gb[2];
  }

  // Rate of change of quaternion from gyroscope + feedback step
  qd[0] = 0.5f * (-q[1]*gm[0] - q[2]*gm[1] - q[3]*gm[2]) - beta * st[0];
  qd[1] = 0.5f * ( q[0]*gm[0] - q[3]*gm[1] + q[2]*gm[2]) - beta * st[1];
  qd[2] = 0.5f * ( q[3]*gm[0] + q[0]*gm[1] - q[1]*gm[2]) - beta * st[2];
  qd[3] = 0.5f * (-q[2]*gm[0] + q[1]*gm[1] + q[0]*gm[2]) - beta * st[3];

  // Time integration with computed rate
  q[0] += qd[0] * dt;
  q[1] += qd[1] * dt;
  q[2] += qd[2] * dt;
  q[3] += qd[3] * dt;

  // Normalize quaternion
  quatNormalize(q);
}
