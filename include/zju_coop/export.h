/*
 * 模块职责：统一控制协同定位动态库的符号可见性和C调用约定。
 * Windows构建DLL时区分导出/导入，Linux与RK3588构建.so时只暴露标记接口；
 * 本文件不包含任何算法逻辑，避免平台编译细节泄漏到C ABI声明中。
 *
 * C/C++初学者阅读提示：
 * 1. 这里的“宏”可理解为编译前的文本替换规则，以“#”开头的语句由预处理器执行。
 * 2. ZJU_COOP_API放在公开函数前，告诉编译器该函数是否需要从动态库导出或导入。
 * 3. ZJU_COOP_CALL固定Windows函数调用约定，防止库和调用程序对参数传递方式理解不一致。
 * 4. 本文件只解决“能否找到库函数”，不会接收IMU、计算定位或发送网络数据。
 */
#ifndef ZJU_COOP_EXPORT_H
/* 首次包含本头文件时定义保护宏；再次包含时会跳到文件末尾，防止重复声明。 */
#define ZJU_COOP_EXPORT_H

/* 关键平台分支：编译器只会保留符合当前操作系统/编译器条件的一个分支。 */
#if defined(_WIN32) || defined(__CYGWIN__)
#  if defined(ZJU_COOP_BUILDING_LIBRARY)
     /* 正在编译本库：把公开函数写进DLL的导出表。 */
#    define ZJU_COOP_API __declspec(dllexport)
#  else
     /* 正在编译调用者：声明公开函数来自另一个DLL。 */
#    define ZJU_COOP_API __declspec(dllimport)
#  endif
   /* __cdecl规定参数、返回值和栈清理方式，库与调用者必须保持相同。 */
#  define ZJU_COOP_CALL __cdecl
#elif defined(__GNUC__) && (__GNUC__ >= 4)
   /* GCC/Clang默认可隐藏内部符号，这里只让公开C ABI在.so外部可见。 */
#  define ZJU_COOP_API __attribute__((visibility("default")))
#  define ZJU_COOP_CALL
#else
   /* 其他编译器没有已知的可见性标记时退化为空宏，至少保持源代码可编译。 */
#  define ZJU_COOP_API
   /* 非Windows平台不需要显式指定__cdecl，因此调用约定宏展开为空。 */
#  define ZJU_COOP_CALL
#endif

/* 与文件顶部#ifndef配对，结束头文件保护范围。 */
#endif
