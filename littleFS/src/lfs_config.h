/*
 * LittleFS Configuration for Embedded Zephyr Application
 */

#ifndef LFS_CONFIG_H
#define LFS_CONFIG_H

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

// Use Zephyr's logging instead of printf
#define LFS_NO_DEBUG
#define LFS_NO_WARN
#define LFS_NO_ERROR

// Use Zephyr's assertion
#define LFS_ASSERT(test) __ASSERT(test, "LFS assertion failed")

// Use Zephyr's memory functions for dynamic allocation
#define LFS_MALLOC(size) k_malloc(size)
#define LFS_FREE(ptr) k_free(ptr)

// Thread safety - use Zephyr mutexes if needed
// For now, assume single-threaded access
#define LFS_THREADSAFE 0

#endif /* LFS_CONFIG_H */