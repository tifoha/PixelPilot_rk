#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void gsmenu_log_trace(const char* fmt, ...);
void gsmenu_log_debug(const char* fmt, ...);
void gsmenu_log_info(const char* fmt, ...);
void gsmenu_log_warn(const char* fmt, ...);
void gsmenu_log_error(const char* fmt, ...);

#ifdef __cplusplus
}
#endif
