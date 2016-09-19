#ifndef CALIBRATION_H
#define CALIBRATION_H

// Calibration template for SEN-10736, based on information from data sheets

// Accelerometer calibration: Ka*a(LSB) + a0 = a(g)
const float Ka[3][3] = {{ 3.900000e-03, 0.0, 0.0},
                        { 0.0, 3.900000e-03, 0.0},
                        { 0.0, 0.0, 3.900000e-03}};

const float a0[3] = { 0.0, 0.0, 0.0};

// Magnetometer calibration: Km*m(LSB) + m0 = m(Gauss)
const float Km[3][3] = {{ 9.174312e-04, 0.0, 0.0},
                        { 0.0, 9.174312e-04, 0.0},
                        { 0.0, 0.0, 9.174312e-04}};

const float m0[3] = { 0.0, 0.0, 0.0};

// Gyroscope calibration: Kg*g(LSB) + g0 = g(rad/s)
const float Kg[3][3] = {{ 1.214142e-03, 0.0, 0.0},
                        { 0.0, 1.214142e-03, 0.0},
                        { 0.0, 0.0, 1.214142e-03}};

const float g0[3] = { 0.0, 0.0, 0.0};

#endif //CALIBRATION_H
