/*
 * 模块职责：统一控制协同定位动态库的符号可见性和C调用约定。
 * Windows构建DLL时区分导出/导入，Linux与RK3588构建.so时只暴露标记接口；
 * 本文件不包含任何算法逻辑，避免平台编译细节泄漏到C ABI声明中。
 */
#ifndef ZJU_COOP_EXPORT_H
#define ZJU_COOP_EXPORT_H

/* 关键平台分支：调用方和库本体必须看到一致的宏，否则会产生链接或ABI问题。 */
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
