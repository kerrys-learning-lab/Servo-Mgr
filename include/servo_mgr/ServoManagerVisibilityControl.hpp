// Copyright 2022 Kerry Johnson
//
// This file is part of Servo-Mgr.
//
// Servo-Mgr is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License v3 as published by the Free Software
// Foundation.
//
// Servo-Mgr is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
// A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with
// Servo-Mgr.  If not, see https://www.gnu.org/licenses/.
#ifndef SERVOMANAGERVISIBILITYCONTROL_HPP_
#define SERVOMANAGERVISIBILITYCONTROL_HPP_

#ifdef __cplusplus
extern "C"
{
#endif

  // This logic was borrowed (then namespaced) from the examples on the gcc wiki:
  //     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define SERVO_MGR_EXPORT __attribute__((dllexport))
#define SERVO_MGR_IMPORT __attribute__((dllimport))
#else
#define SERVO_MGR_EXPORT __declspec(dllexport)
#define SERVO_MGR_IMPORT __declspec(dllimport)
#endif
#ifdef SERVO_MGR_BUILDING_DLL
#define SERVO_MGR_PUBLIC SERVO_MGR_EXPORT
#else
#define SERVO_MGR_PUBLIC SERVO_MGR_IMPORT
#endif
#define SERVO_MGR_PUBLIC_TYPE SERVO_MGR_PUBLIC
#define SERVO_MGR_LOCAL
#else
#define SERVO_MGR_EXPORT __attribute__((visibility("default")))
#define SERVO_MGR_IMPORT
#if __GNUC__ >= 4
#define SERVO_MGR_PUBLIC __attribute__((visibility("default")))
#define SERVO_MGR_LOCAL __attribute__((visibility("hidden")))
#else
#define SERVO_MGR_PUBLIC
#define SERVO_MGR_LOCAL
#endif
#define SERVO_MGR_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif /* SERVOMANAGERVISIBILITYCONTROL_HPP_ */
