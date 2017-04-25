#include "shoe_accel.h"
#include "wiring_digital.h"
#include "wiring_analog.h"
#include "wiring_constants.h"

#define ACCEL_POWER_PIN 23
#define X_PIN 4
#define Z_PIN 6

void shoe_accel_init() {
  pinMode(ACCEL_POWER_PIN, OUTPUT);
}

void enable_shoe_accel() {
  digitalWrite(ACCEL_POWER_PIN, HIGH);
}

void disable_shoe_accel() {
  digitalWrite(ACCEL_POWER_PIN, LOW);
}

void read_shoe_accel(int8_t *x, int8_t *y, int8_t *z) {
  *x = (int8_t)(analogRead(X_PIN) - 512 / 4);
  *y = 0; // Y is not connected currently
  *z = (int8_t)(analogRead(Z_PIN) - 512 / 4);
}
