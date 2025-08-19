# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(littlefs)

target_sources(app PRIVATE 
    src/main.c
    src/mx25l51245g_flash.c
    LittleFS/lfs.c
    LittleFS/lfs_util.c
)

# Add include directories
target_include_directories(app PRIVATE
    LittleFS
    src
)

# Configure LittleFS with basic settings
target_compile_definitions(app PRIVATE
    LFS_NO_DEBUG
    LFS_NO_WARN
    LFS_NO_ERROR
)
