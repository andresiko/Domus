#if 1
#ifndef LV_CONF_H
#define LV_CONF_H

#define LV_COLOR_DEPTH 16

#define LV_MEM_CUSTOM           1
#define LV_MEM_CUSTOM_INCLUDE   "esp_heap_caps.h"
#define LV_MEM_CUSTOM_ALLOC(size)        heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
#define LV_MEM_CUSTOM_REALLOC(ptr, size) heap_caps_realloc(ptr, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
#define LV_MEM_CUSTOM_FREE(ptr)          heap_caps_free(ptr)

#define LV_USE_LOG     1
#define LV_LOG_LEVEL   LV_LOG_LEVEL_WARN

#define LV_USE_LABEL    1
#define LV_USE_BTN      1
#define LV_USE_IMG      1
#define LV_USE_ARC      1
#define LV_USE_LINE     1
#define LV_USE_SPINNER  1
#define LV_USE_MSGBOX   1
#define LV_USE_KEYBOARD 1
#define LV_USE_TILEVIEW 1
#define LV_USE_SWITCH   1

#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_20 1
#define LV_FONT_MONTSERRAT_40 1
#define LV_FONT_DEFAULT &lv_font_montserrat_20

#define LV_USE_PERF_MONITOR 0

#endif
#endif
