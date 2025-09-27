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
#ifndef TITATI_POWER_SERVICES_COMMON_HPP
#define TITATI_POWER_SERVICES_COMMON_HPP

#define DEF_VAR(varType, varName, funName)                           \
private:                                                             \
  varType varName;                                                   \
                                                                     \
public:                                                              \
  virtual varType get##funName(void) const { return this->varName; } \
                                                                     \
public:                                                              \
  virtual void set##funName(varType var) { this->varName = var; }

#define DEF_VAR_NOSET(varType, varName, funName) \
private:                                         \
  varType varName;                               \
                                                 \
public:                                          \
  virtual varType get##funName(void) const { return this->varName; }

#define DEF_STRUCT_OPT(structName, memberType, memberName)       \
public:                                                          \
  virtual memberType get_##structName##_##memberName(void) const \
  {                                                              \
    return this->structName.memberName;                          \
  }                                                              \
                                                                 \
public:                                                          \
  virtual void set_##structName##_##memberName(memberType var)   \
  {                                                              \
    this->structName.memberName = var;                           \
  }

#endif  // TITATI_POWER_SERVICES_COMMON_HPP
