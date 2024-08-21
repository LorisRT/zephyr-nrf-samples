# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(wsnDrivers)

target_sources(app PRIVATE src/app_mpu6050.c)
target_include_directories(app PRIVATE src/app_mpu6050.h)

target_sources(app PRIVATE src/main.c)
