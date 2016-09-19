// Computes the dot product of two vectors
float Vector_Dot_Product(const float v1[3], const float v2[3])
{
  return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]; 
}

// Computes the cross product of two vectors
void Vector_Cross_Product(float out[3], const float v1[3], const float v2[3])
{
  out[0] = v1[1]*v2[2] - v1[2]*v2[1];
  out[1] = v1[2]*v2[0] - v1[0]*v2[2];
  out[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

// Multiply the vector by a scalar
void Vector_Scale(float out[3], const float v[3], const float scale)
{
  for (int c = 0; c < 3; c++)
    out[c] = v[c] * scale; 
}

// Adds two vectors
void Vector_Add(float out[3], const float v1[3], const float v2[3])
{
  for (int c = 0; c < 3; c++)
    out[c] = v1[c] + v2[c];
}

// Multiply two 3x3 matrices: out = a * b
void Matrix_Multiply(float out[3][3], const float a[3][3], const float b[3][3])
{
  for (int x = 0; x < 3; x++)
  {
    for (int y = 0; y < 3; y++)
      out[x][y] = a[x][0]*b[0][y] + a[x][1]*b[1][y] + a[x][2]*b[2][y];
  }
}

// Multiply 3x3 matrix with vector: out = a * b
void Matrix_Vector_Multiply(float out[3], const float a[3][3], const float b[3])
{
  for (int x = 0; x < 3; x++)
    out[x] = a[x][0]*b[0] + a[x][1]*b[1] + a[x][2]*b[2];
}

void init_quaternion(const float a[3], const float m[3])
{
  float recipNorm;
  float n[3]; // North vector
  float e[3]; // East vector
  float d[3]; // Down vector

  // Calculate Down vector
  d[0] = a[0];
  d[1] = a[1];
  d[2] = a[2];

  // Normalize Down vector
  vecNormalize(d);

  // Calculate East vector
  Vector_Cross_Product(e, d, m);

  // Normalize East vector
  vecNormalize(e);

  // Calculate North vector (already normalized)
  Vector_Cross_Product(n, e, d);

  // Calculate quaternion components
  q[0] = 0.50f * sqrt(1.0f + n[0] + e[1] + d[2]);
  q[1] = 0.25f * (d[1] - e[2]) / q[0];
  q[2] = 0.25f * (n[2] - d[0]) / q[0];
  q[3] = 0.25f * (e[0] - n[1]) / q[0];
}

// Fast inverse square-root
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

// Quaternion normalization
void quatNormalize(float *q)
{
  float recipNorm = invSqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  q[0] *= recipNorm;
  q[1] *= recipNorm;
  q[2] *= recipNorm;
  q[3] *= recipNorm;
}

// Vector normalization
void vecNormalize(float *v)
{
  float recipNorm = invSqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  v[0] *= recipNorm;
  v[1] *= recipNorm;
  v[2] *= recipNorm;
}
