#ifndef LOGGER_H
#define LOGGER_H

#ifdef __cplusplus
extern "C" {
#endif

void logger_init();
void logger_print(const char[]);
void logger_println(const char[]);

#ifdef __cplusplus
}
#endif

#endif // LOGGER_H
