// Copyright 2019 Bold Hearts
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

#ifndef IMU_PUBLISHER__VISIBILITY_CONTROL_H_
#define IMU_PUBLISHER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define IMU_PUBLISHER_EXPORT __attribute__ ((dllexport))
    #define IMU_PUBLISHER_IMPORT __attribute__ ((dllimport))
  #else
    #define IMU_PUBLISHER_EXPORT __declspec(dllexport)
    #define IMU_PUBLISHER_IMPORT __declspec(dllimport)
  #endif
  #ifdef IMU_PUBLISHER_BUILDING_LIBRARY
    #define IMU_PUBLISHER_PUBLIC IMU_PUBLISHER_EXPORT
  #else
    #define IMU_PUBLISHER_PUBLIC IMU_PUBLISHER_IMPORT
  #endif
  #define IMU_PUBLISHER_PUBLIC_TYPE IMU_PUBLISHER_PUBLIC
  #define IMU_PUBLISHER_LOCAL
#else
  #define IMU_PUBLISHER_EXPORT __attribute__ ((visibility("default")))
  #define IMU_PUBLISHER_IMPORT
  #if __GNUC__ >= 4
    #define IMU_PUBLISHER_PUBLIC __attribute__ ((visibility("default")))
    #define IMU_PUBLISHER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define IMU_PUBLISHER_PUBLIC
    #define IMU_PUBLISHER_LOCAL
  #endif
  #define IMU_PUBLISHER_PUBLIC_TYPE
#endif

#endif  // IMU_PUBLISHER__VISIBILITY_CONTROL_H_
