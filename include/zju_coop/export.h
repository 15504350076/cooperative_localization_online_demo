/* 动态库符号导出控制：Windows 生成 DLL，Linux/RK3588 生成共享对象。 */
#ifndef ZJU_COOP_EXPORT_H
#define ZJU_COOP_EXPORT_H

#if defined(_WIN32) || defined(__CYGWIN__)
#  if defined(ZJU_COOP_BUILDING_LIBRARY)
#    define ZJU_COOP_API __declspec(dllexport)
#  else
#    define ZJU_COOP_API __declspec(dllimport)
#  endif
#  define ZJU_COOP_CALL __cdecl
#elif defined(__GNUC__) && (__GNUC__ >= 4)
#  define ZJU_COOP_API __attribute__((visibility("default")))
#  define ZJU_COOP_CALL
#else
#  define ZJU_COOP_API
#  define ZJU_COOP_CALL
#endif

#endif
