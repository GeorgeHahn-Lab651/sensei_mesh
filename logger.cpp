
#include "logger.h"
#include <Arduino.h>

void logger_init() {
  Serial.begin(9600);
}

void logger_print(const char str[]) {
  Serial.print(str);
}

void logger_println(const char str[]) {
  Serial.println(str);
}

void logger_print_uint_with_base(unsigned int b, int base) {
  Serial.print(b, base);
}

void logger_print_uint(unsigned int b) {
  Serial.print(b, DEC);
}
