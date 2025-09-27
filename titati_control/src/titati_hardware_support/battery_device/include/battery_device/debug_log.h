// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
#ifndef BATTERY_DEVICE_DEBUG_LOG_H_
#define BATTERY_DEVICE_DEBUG_LOG_H_
#include <stdarg.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

#define ERR_LEVEL 1
#define WARN_LEVEL 2
#define INFO_LEVEL 3

#define DEBUG_LEVEL 3

void __attribute__((format(printf, 1, 2))) LOG_INFO(const char * fmt, ...)
{
#if (DEBUG_LEVEL >= INFO_LEVEL)
  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);
  va_end(args);
#endif
}

void __attribute__((format(printf, 1, 2))) LOG_WARN(const char * fmt, ...)
{
#if (DEBUG_LEVEL >= WARN_LEVEL)
  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);
  va_end(args);
#endif
}

void __attribute__((format(printf, 1, 2))) LOG_ERR(const char * fmt, ...)
{
#if (DEBUG_LEVEL >= ERR_LEVEL)
  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);
  va_end(args);
#endif
}

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // BATTERY_DEVICE_DEBUG_LOG_H_
