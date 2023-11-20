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

#ifndef CM730DRIVER__VISIBILITY_CONTROL_H_
#define CM730DRIVER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CM730DRIVER_EXPORT __attribute__ ((dllexport))
    #define CM730DRIVER_IMPORT __attribute__ ((dllimport))
  #else
    #define CM730DRIVER_EXPORT __declspec(dllexport)
    #define CM730DRIVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef CM730DRIVER_BUILDING_LIBRARY
    #define CM730DRIVER_PUBLIC CM730DRIVER_EXPORT
  #else
    #define CM730DRIVER_PUBLIC CM730DRIVER_IMPORT
  #endif
  #define CM730DRIVER_PUBLIC_TYPE CM730DRIVER_PUBLIC
  #define CM730DRIVER_LOCAL
#else
  #define CM730DRIVER_EXPORT __attribute__ ((visibility("default")))
  #define CM730DRIVER_IMPORT
  #if __GNUC__ >= 4
    #define CM730DRIVER_PUBLIC __attribute__ ((visibility("default")))
    #define CM730DRIVER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CM730DRIVER_PUBLIC
    #define CM730DRIVER_LOCAL
  #endif
  #define CM730DRIVER_PUBLIC_TYPE
#endif

#endif  // CM730DRIVER__VISIBILITY_CONTROL_H_
