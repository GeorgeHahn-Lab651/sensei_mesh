#ifndef LOGGER_H
#define LOGGER_H

#ifdef __cplusplus
extern "C" {
#endif

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

void logger_init();
void logger_print(const char[]);
void logger_println(const char[]);
void logger_print_uint_with_base(unsigned int, int);
void logger_print_uint(unsigned int);

#ifdef __cplusplus
}
#endif

#endif // LOGGER_H
