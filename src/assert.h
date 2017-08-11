// Print assertion messages over Segger RTT in debug builds

#if DEBUG == 1
#include "SEGGER_RTT.h"

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#define log(msg) SEGGER_RTT_WriteString(0, msg "\n");

#define APP_ASSERT_EQUAL(A, B, ...)                                            \
  if (A != B) {                                                                \
    SEGGER_RTT_printf(                                                         \
        0, __FILE__                                                            \
        ":" TOSTRING(__LINE__) "\n\texpected: %d actual: %d \n\t" __VA_ARGS__  \
                               "\n",                                           \
        B, A);                                                                 \
  }

#define APP_ASSERT(expr, ...)                                                  \
  if (!(expr)) {                                                               \
    log(__FILE__ " " TOSTRING(__LINE__) " " __VA_ARGS__)                       \
  }

#else
// WARNING: expr will not be evaluated - if it is side effectey, this will break
// your production builds in mysterious ways
#define APP_ASSERT(expr, ...)                                                  \
  {}
#define APP_ASSERT_EQUAL(A, B, ...)                                            \
  {}
#define log(msg)                                                               \
  {}
#endif