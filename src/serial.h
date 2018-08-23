#ifndef SERIAL_H
#define SERIAL_H

#include <termios.h>

#define A 127
#define B 255

// ************ 4 x 2 (rows x cols) 
// -> activate 2 motors at same row at same time ***********
// m1 m2
// m3 m4
// m5 m6
// m7 m8
// m9 m10  <--- not in use for this config

// ???
// RECURSIVIDADE ??
// unsigned char c1[10] = {A, A, 1, 1, 1, 1, 1, 1, 1, 1};
// unsigned char c2[10] = {1, 1, A, A, 1, 1, 1, 1, 1, 1};
// unsigned char c3[10] = {A, A, A, A, 1, 1, 1, 1, 1, 1};
// unsigned char c4[10] = {1, 1, 1, 1, A, A, 1, 1, 1, 1};
// unsigned char c5[10] = {A, A, 1, 1, A, A, 1, 1, 1, 1};
// unsigned char c6[10] = {1, 1, A, A, A, A, 1, 1, 1, 1};
// unsigned char c7[10] = {1, 1, A, A, A, A, 1, 1, 1, 1};

void open_port();
void close_port();
void fillPatternsMap();
void getOn(unsigned char* out);
void getOff(unsigned char* out);
void getPattern(unsigned char* out, unsigned char* values, size_t size);
void sendToSerialWithSleep(unsigned char* data, size_t size);
void sendToSerial(unsigned char* data, size_t size);
int writeDataToSerial(char *port, unsigned char data[], size_t size);

#endif

