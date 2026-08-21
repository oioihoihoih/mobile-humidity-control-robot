#pragma once

#include <cstddef>
#include <cstdint>

using byte = std::uint8_t;

constexpr byte LOW = 0;
constexpr byte HIGH = 1;
constexpr byte INPUT = 0;
constexpr byte OUTPUT = 1;

void pinMode(byte pin, byte mode);
void digitalWrite(byte pin, byte value);
int digitalRead(byte pin);
void delay(unsigned long milliseconds);
void delayMicroseconds(unsigned int microseconds);
unsigned long millis();
