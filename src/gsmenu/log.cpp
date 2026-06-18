#include "log.h"
#include <spdlog/spdlog.h>
#include <cstdarg>
#include <cstdio>

#define DEFINE_GSMENU_LOG(name, level)              \
    void name(const char* fmt, ...) {               \
        char buf[8192];                             \
        va_list args;                                \
        va_start(args, fmt);                          \
        vsnprintf(buf, sizeof(buf), fmt, args);        \
        va_end(args);                                   \
        spdlog::level("[gsmenu] {}", buf);                \
    }

DEFINE_GSMENU_LOG(gsmenu_log_trace, trace)
DEFINE_GSMENU_LOG(gsmenu_log_debug, debug)
DEFINE_GSMENU_LOG(gsmenu_log_info, info)
DEFINE_GSMENU_LOG(gsmenu_log_warn, warn)
DEFINE_GSMENU_LOG(gsmenu_log_error, error)
