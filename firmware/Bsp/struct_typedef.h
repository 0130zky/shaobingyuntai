/**
 *  ███╗   ██╗███████╗██╗  ██╗████████╗  ███████╗
 *  ████╗  ██║██╔════╝╚██╗██╔╝╚══██╔══╝  ██╔════╝
 *  ██╔██╗ ██║█████╗   ╚███╔╝    ██║     █████╗  
 *  ██║╚██╗██║██╔══╝   ██╔██╗    ██║     ██╔══╝  
 *  ██║ ╚████║███████╗██╔╝ ██╗   ██║     ███████╗
 *  ╚═╝  ╚═══╝╚══════╝╚═╝  ╚═╝   ╚═╝     ╚══════╝
 *
 *  @Author : Kaiyue Zhang
 *  @Email  : kaiyue.link@gmail.com
 *  @date   : 2026-10-30
 *  @description ：工程通用定宽整数类型定义。
 */
#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

/* Fixed-width integer types come from the toolchain.
 *
 * The hand-rolled typedefs that used to live here (int32_t = signed int)
 * clash with newlib's <stdint.h> on arm-none-eabi, where int32_t is
 * `long int` / uint32_t is `unsigned long int`. Compiling both in the same
 * translation unit makes prototypes disagree across files - the USB CDC
 * callbacks and the HAL end up with incompatible pointer types - so defer
 * to <stdint.h> and keep only the project-specific aliases below. */
#include <stdint.h>

typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

#endif
