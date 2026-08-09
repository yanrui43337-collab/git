#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

int Debug_Printf(const char *format, ...);

#ifdef __cplusplus
}
#endif

/* Serialize all application printf calls through Debug_Printf. */
#define printf Debug_Printf

#endif /* DEBUG_LOG_H */
