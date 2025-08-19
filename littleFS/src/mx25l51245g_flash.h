/*
 * MX25L51245GZ2I-08G NOR Flash Driver with LittleFS Support
 * Header File
 */

#ifndef MX25L51245G_FLASH_H
#define MX25L51245G_FLASH_H

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Public API Functions */
int mx25l51245g_system_init(void);
int mx25l51245g_basic_init(void);
int mx25l51245g_basic_test(void);
int mx25l51245g_write_file(const char *filename, const void *data, size_t len);
int mx25l51245g_read_file(const char *filename, void *buffer, size_t len);
int mx25l51245g_write_struct(const char *filename, const void *data, size_t size);
int mx25l51245g_read_struct(const char *filename, void *buffer, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* MX25L51245G_FLASH_H */