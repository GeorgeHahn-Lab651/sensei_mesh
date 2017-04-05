
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
