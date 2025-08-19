/*
 * MX25L51245GZ2I-08G NOR Flash Driver with LittleFS Support
 * Implementation File
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <errno.h>
#include "../LittleFS/lfs.h"
#include "mx25l51245g_flash.h"

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

LOG_MODULE_REGISTER(mx25l51245g, LOG_LEVEL_DBG);

/* MX25L51245G Commands */
#define MX25L51245G_CMD_READ_DATA        0x03
#define MX25L51245G_CMD_FAST_READ        0x0B
#define MX25L51245G_CMD_PAGE_PROGRAM     0x02
#define MX25L51245G_CMD_SECTOR_ERASE     0x20
#define MX25L51245G_CMD_BLOCK_ERASE_32K  0x52
#define MX25L51245G_CMD_BLOCK_ERASE_64K  0xD8
#define MX25L51245G_CMD_CHIP_ERASE       0xC7
#define MX25L51245G_CMD_WRITE_ENABLE     0x06
#define MX25L51245G_CMD_WRITE_DISABLE    0x04
#define MX25L51245G_CMD_READ_STATUS      0x05
#define MX25L51245G_CMD_WRITE_STATUS     0x01
#define MX25L51245G_CMD_JEDEC_ID         0x9F
#define MX25L51245G_CMD_POWER_DOWN       0xB9
#define MX25L51245G_CMD_RELEASE_POWER_DOWN 0xAB

/* MX25L51245G Parameters */
#define MX25L51245G_PAGE_SIZE            256
#define MX25L51245G_SECTOR_SIZE          4096    // 4KB
#define FLASH_SECTOR_SIZE_BYTES          4096    // Alternative name to avoid conflicts
#define MX25L51245G_BLOCK_SIZE_32K       32768   // 32KB
#define MX25L51245G_BLOCK_SIZE_64K       65536   // 64KB
#define MX25L51245G_TOTAL_SIZE           67108864 // 64MB (512Mbit)
#define MX25L51245G_PAGES_PER_SECTOR     16
#define MX25L51245G_SECTORS_PER_BLOCK    16
#define MX25L51245G_TOTAL_SECTORS        16384

/* Status Register Bits */
#define MX25L51245G_SR_BUSY              BIT(0)
#define MX25L51245G_SR_WEL               BIT(1)
#define MX25L51245G_SR_BP0               BIT(2)
#define MX25L51245G_SR_BP1               BIT(3)
#define MX25L51245G_SR_BP2               BIT(4)
#define MX25L51245G_SR_BP3               BIT(5)
#define MX25L51245G_SR_QE                BIT(6)
#define MX25L51245G_SR_SRWD              BIT(7)

/* Device Structure */
struct mx25l51245g_data {
    const struct device *spi_dev;
    struct spi_config spi_cfg;
    const struct device *cs_gpio_dev;
    gpio_pin_t cs_pin;
    gpio_dt_flags_t cs_flags;
    bool initialized;
};

/* Global device instance */
static struct mx25l51245g_data mx25l51245g_dev_data;

/* SPI Configuration - adjust according to your board's device tree */
static struct spi_config spi_cfg = {
    .frequency = 1000000,  // 1MHz - slower for debugging
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,  // Try SPI Mode 0 (no CPOL/CPHA)
    .slave = 0,
};

/* Function Prototypes */
static int mx25l51245g_spi_transceive(const uint8_t *tx_buf, size_t tx_len, 
                                     uint8_t *rx_buf, size_t rx_len);
static int mx25l51245g_wait_ready(void);
static int mx25l51245g_read_status(uint8_t *status);
static int mx25l51245g_write_enable(void);
static int mx25l51245g_read_id(uint8_t *id);

/* LittleFS callbacks */
static int lfs_read(const struct lfs_config *c, lfs_block_t block,
                   lfs_off_t off, void *buffer, lfs_size_t size);
static int lfs_prog(const struct lfs_config *c, lfs_block_t block,
                   lfs_off_t off, const void *buffer, lfs_size_t size);
static int lfs_erase(const struct lfs_config *c, lfs_block_t block);
static int lfs_sync(const struct lfs_config *c);

/* LittleFS Configuration - Matching other MCU configuration */
static struct lfs_config lfs_cfg = {
    .read = lfs_read,
    .prog = lfs_prog,
    .erase = lfs_erase,
    .sync = lfs_sync,
    
    .block_size = 4096UL,                      // 4KB sectors
    .block_count = 2048UL,                     // 8MB total (2048 * 4KB)
    .cache_size = 256UL,                       // 256 bytes cache
    .lookahead_size = 256UL,                   // 256 bytes lookahead
    .block_cycles = 100000L,                   // 100,000 erase cycles
    .read_size = 256UL,                        // 256 bytes read size
    .prog_size = 256UL,                        // 256 bytes program size
};

/* LittleFS file system */
static lfs_t lfs;
static uint8_t lfs_read_buffer[MX25L51245G_PAGE_SIZE];
static uint8_t lfs_prog_buffer[MX25L51245G_PAGE_SIZE];
static uint8_t lfs_lookahead_buffer[256];  // 256 bytes for lookahead_size=256

/* SPI Transaction Helper with manual CS control */
static int mx25l51245g_spi_transceive(const uint8_t *tx_buf, size_t tx_len, 
                                     uint8_t *rx_buf, size_t rx_len)
{
    const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    int ret;
    
    struct spi_buf tx_spi_buf = {
        .buf = (void *)tx_buf,
        .len = tx_len
    };
    struct spi_buf_set tx_spi_buf_set = {
        .buffers = &tx_spi_buf,
        .count = 1
    };
    
    struct spi_buf rx_spi_buf = {
        .buf = rx_buf,
        .len = rx_len
    };
    struct spi_buf_set rx_spi_buf_set = {
        .buffers = &rx_spi_buf,
        .count = 1
    };
    
    // Manual CS control (since automatic CS doesn't work properly)
    gpio_pin_set(gpio_dev, 22, 0);  // CS low
    k_usleep(1);  // Small delay
    
    ret = spi_transceive(mx25l51245g_dev_data.spi_dev, &mx25l51245g_dev_data.spi_cfg,
                         &tx_spi_buf_set, rx_len ? &rx_spi_buf_set : NULL);
    
    k_usleep(1);  // Small delay
    gpio_pin_set(gpio_dev, 22, 1);  // CS high
    
    return ret;
}

/* Wait for device ready */
static int mx25l51245g_wait_ready(void)
{
    uint8_t status;
    int retries = 1000;
    
    while (retries--) {
        if (mx25l51245g_read_status(&status) != 0) {
            return -EIO;
        }
        
        if (!(status & MX25L51245G_SR_BUSY)) {
            return 0;
        }
        
        k_msleep(1);
    }
    
    return -ETIMEDOUT;
}

/* Read status register */
static int mx25l51245g_read_status(uint8_t *status)
{
    uint8_t tx_buf[2] = {MX25L51245G_CMD_READ_STATUS, 0};
    uint8_t rx_buf[2];
    int ret;
    
    // Use 2-byte transaction to avoid PAN 58
    ret = mx25l51245g_spi_transceive(tx_buf, 2, rx_buf, 2);
    if (ret == 0) {
        *status = rx_buf[1];  // Status is in second byte
    }
    
    return ret;
}

/* Write enable */
static int mx25l51245g_write_enable(void)
{
    uint8_t tx_buf[1] = {MX25L51245G_CMD_WRITE_ENABLE};
    
    // Try sending exactly 1 byte as per datasheet, even if it might trigger PAN 58
    // This is critical for Write Enable to work correctly
    return mx25l51245g_spi_transceive(tx_buf, 1, NULL, 0);
}

/* Read device ID */
static int mx25l51245g_read_id(uint8_t *id)
{
    uint8_t tx_buf[4] = {MX25L51245G_CMD_JEDEC_ID, 0, 0, 0};
    uint8_t rx_buf[4];
    int ret;
    
    // Use the working method with manual CS control
    ret = mx25l51245g_spi_transceive(tx_buf, 4, rx_buf, 4);
    if (ret == 0) {
        // Copy the ID bytes (skip the first dummy byte)
        id[0] = rx_buf[1];
        id[1] = rx_buf[2];
        id[2] = rx_buf[3];
    }
    
    return ret;
}

/* Initialize MX25L51245G device */
static int mx25l51245g_init(void)
{
    int ret;
    uint8_t id[3];
    
    // Get SPI device
    mx25l51245g_dev_data.spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi0));
    if (!device_is_ready(mx25l51245g_dev_data.spi_dev)) {
        LOG_ERR("SPI device not ready");
        return -ENODEV;
    }
    
    LOG_INF("SPI device ready, configuring...");
    
    // Configure SPI
    mx25l51245g_dev_data.spi_cfg = spi_cfg;
    
    // Test CS pin manually first
    const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (device_is_ready(gpio_dev)) {
        LOG_INF("Testing CS pin (P0.22) manually...");
        gpio_pin_configure(gpio_dev, 22, GPIO_OUTPUT);
        gpio_pin_set(gpio_dev, 22, 1);  // CS high
        k_msleep(10);
        gpio_pin_set(gpio_dev, 22, 0);  // CS low
        k_msleep(10);
        gpio_pin_set(gpio_dev, 22, 1);  // CS high
        LOG_INF("CS pin test completed");
    }
    
    LOG_INF("SPI configured, waiting for flash power-up...");
    
    // Wait for flash chip power-up (some chips need time after power-on)
    k_msleep(100);
    
    // Try to wake up the flash chip in case it's in deep sleep
    uint8_t wakeup_cmd[2] = {0xAB, 0x00};  // Release from Deep Power Down
    mx25l51245g_spi_transceive(wakeup_cmd, 2, NULL, 0);
    k_msleep(10);
    
    // Read and verify device ID
    ret = mx25l51245g_read_id(id);
    if (ret != 0) {
        LOG_ERR("Failed to read device ID: %d", ret);
        return ret;
    }
    
    LOG_INF("Device ID: %02X %02X %02X", id[0], id[1], id[2]);
    
    // For debugging, let's be more lenient and just warn if ID doesn't match
    if (id[0] != 0xC2 || id[1] != 0x20 || id[2] != 0x1A) {
        LOG_WRN("Unexpected device ID, expected C2 20 1A, got %02X %02X %02X", id[0], id[1], id[2]);
        
        // Check if we're getting all 0xFF (no communication) or all 0x00 (different issue)
        if (id[0] == 0xFF && id[1] == 0xFF && id[2] == 0xFF) {
            LOG_ERR("All 0xFF - likely no SPI communication (check wiring/power)");
            return -ENODEV;
        } else if (id[0] == 0x00 && id[1] == 0x00 && id[2] == 0x00) {
            LOG_ERR("All 0x00 - possible SPI timing issue");
            return -ENODEV;
        } else {
            LOG_WRN("Different chip detected, continuing anyway for testing...");
        }
    }
    
    mx25l51245g_dev_data.initialized = true;
    LOG_INF("MX25L51245G initialized successfully");
    
    return 0;
}

/* LittleFS Read Function */
static int lfs_read(const struct lfs_config *c, lfs_block_t block,
                   lfs_off_t off, void *buffer, lfs_size_t size)
{
    // Calculate address: block * 4096 + offset
    uint32_t addr = ((uint32_t)block << 12) + (uint32_t)off;  // << 12 is * 4096
    const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    int ret;
    
    // Simple validation
    if (block >= 2048 || size > 4096) {
        return LFS_ERR_IO;
    }
    
    // Create combined buffer: command + dummy bytes for reading
    // This avoids separate transactions that might trigger PAN 58
    uint8_t combined_buf[4 + 512];  // Command + max read size
    if (size > 512) {
        return LFS_ERR_IO;
    }
    
    // Set up command
    combined_buf[0] = MX25L51245G_CMD_READ_DATA;
    combined_buf[1] = (addr >> 16) & 0xFF;
    combined_buf[2] = (addr >> 8) & 0xFF;
    combined_buf[3] = addr & 0xFF;
    
    // Fill rest with dummy bytes (0xFF)
    memset(combined_buf + 4, 0xFF, size);
    
    // Create receive buffer for the entire transaction
    uint8_t rx_combined[4 + 512];
    memset(rx_combined, 0, sizeof(rx_combined));
    
    // Use our PAN 58 workaround function for the entire transaction
    ret = mx25l51245g_spi_transceive(combined_buf, 4 + size, rx_combined, 4 + size);
    
    if (ret != 0) {
        return LFS_ERR_IO;
    }
    
    // Copy the data part (skip the first 4 bytes which are command echo)
    memcpy(buffer, rx_combined + 4, size);
    
    return LFS_ERR_OK;
}

/* LittleFS Program Function */
static int lfs_prog(const struct lfs_config *c, lfs_block_t block,
                   lfs_off_t off, const void *buffer, lfs_size_t size)
{
    // Calculate address: block * 4096 + offset  
    uint32_t addr = ((uint32_t)block << 12) + (uint32_t)off;  // << 12 is * 4096
    const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    int ret;
    size_t remaining = size;
    const uint8_t *data = (const uint8_t *)buffer;
    
    // Simple validation
    if (block >= 2048 || size > 4096) {
        return LFS_ERR_IO;
    }
    
    while (remaining > 0) {
        // Calculate page boundary
        uint32_t page_offset = addr % MX25L51245G_PAGE_SIZE;
        size_t write_size = MIN(remaining, MX25L51245G_PAGE_SIZE - page_offset);
        
        // Write Enable
        ret = mx25l51245g_write_enable();
        if (ret != 0) {
            LOG_ERR("Write enable failed: %d", ret);
            return LFS_ERR_IO;
        }
        
        // Page Program command with 24-bit address
        uint8_t cmd_buf[4] = {
            MX25L51245G_CMD_PAGE_PROGRAM,
            (addr >> 16) & 0xFF,
            (addr >> 8) & 0xFF,
            addr & 0xFF
        };
        
        // Create combined buffer for command + data to avoid PAN 58
        uint8_t combined_buf[4 + 256];  // Max page size + command
        memcpy(combined_buf, cmd_buf, 4);
        memcpy(combined_buf + 4, data, write_size);
        
        // Use our PAN 58 workaround function
        ret = mx25l51245g_spi_transceive(combined_buf, 4 + write_size, NULL, 0);
        
        if (ret != 0) {
            return LFS_ERR_IO;
        }
        
        // Wait for program completion
        ret = mx25l51245g_wait_ready();
        if (ret != 0) {
            return LFS_ERR_IO;
        }
        
        addr += write_size;
        data += write_size;
        remaining -= write_size;
    }
    return LFS_ERR_OK;
}

/* LittleFS Erase Function */
static int lfs_erase(const struct lfs_config *c, lfs_block_t block)
{
    // Calculate address: block * 4096
    uint32_t addr = (uint32_t)block << 12;  // << 12 is * 4096
    int ret;
    
    // Simple validation
    if (block >= 2048) {
        return LFS_ERR_IO;
    }
    
    // Write Enable
    ret = mx25l51245g_write_enable();
    if (ret != 0) {
        return LFS_ERR_IO;
    }
    
    // Sector Erase (4KB) - using our working SPI function
    uint8_t cmd_buf[4] = {
        MX25L51245G_CMD_SECTOR_ERASE,
        (addr >> 16) & 0xFF,
        (addr >> 8) & 0xFF,
        addr & 0xFF
    };
    
    ret = mx25l51245g_spi_transceive(cmd_buf, 4, NULL, 0);
    if (ret != 0) {
        return LFS_ERR_IO;
    }
    
    // Wait for erase completion
    ret = mx25l51245g_wait_ready();
    if (ret != 0) {
        return LFS_ERR_IO;
    }
    return LFS_ERR_OK;
}

/* LittleFS Sync Function */
static int lfs_sync(const struct lfs_config *c)
{
    // NOR flash doesn't require explicit sync
    return LFS_ERR_OK;
}

/* Initialize LittleFS */
static int littlefs_init(void)
{
    int ret;
    
    // Configure LittleFS buffers
    lfs_cfg.read_buffer = lfs_read_buffer;
    lfs_cfg.prog_buffer = lfs_prog_buffer;
    lfs_cfg.lookahead_buffer = lfs_lookahead_buffer;
    
    LOG_INF("Attempting to mount existing LittleFS filesystem...");
    
    // Try to mount existing filesystem first
    ret = lfs_mount(&lfs, &lfs_cfg);
    if (ret == LFS_ERR_OK) {
        LOG_INF("LittleFS mounted successfully - existing filesystem found");
        return 0;
    }
    
    // Mount failed, check the reason
    LOG_WRN("Mount failed with error %d, checking if format is needed", ret);
    
    if (ret == LFS_ERR_CORRUPT || ret == LFS_ERR_IO || ret == LFS_ERR_INVAL) {
        LOG_INF("Filesystem appears corrupted or unformatted, formatting...");
        
        // Format the filesystem
        ret = lfs_format(&lfs, &lfs_cfg);
        if (ret != LFS_ERR_OK) {
            LOG_ERR("Format failed with error %d", ret);
            return -EIO;
        }
        
        LOG_INF("Format completed successfully, attempting to mount...");
        
        // Try to mount the newly formatted filesystem
        ret = lfs_mount(&lfs, &lfs_cfg);
        if (ret != LFS_ERR_OK) {
            LOG_ERR("Mount after format failed with error %d", ret);
            return -EIO;
        }
        
        LOG_INF("LittleFS mounted successfully after formatting");
        return 0;
    }
    
    // Some other error occurred
    LOG_ERR("Mount failed with unexpected error %d", ret);
    return -EIO;
}

/* Public API Functions */
int mx25l51245g_write_file(const char *filename, const void *data, size_t len)
{
    lfs_file_t file;
    int ret;
    
    ret = lfs_file_open(&lfs, &file, filename, LFS_O_WRONLY | LFS_O_CREAT);
    if (ret < 0) {
        LOG_ERR("Failed to open file for write: %d", ret);
        return ret;
    }
    
    ret = lfs_file_write(&lfs, &file, data, len);
    if (ret < 0) {
        LOG_ERR("Failed to write file: %d", ret);
        lfs_file_close(&lfs, &file);
        return ret;
    }
    
    ret = lfs_file_close(&lfs, &file);
    if (ret < 0) {
        LOG_ERR("Failed to close file: %d", ret);
        return ret;
    }
    
    LOG_INF("File written successfully: %s (%zu bytes)", filename, len);
    return 0;
}

int mx25l51245g_read_file(const char *filename, void *buffer, size_t len)
{
    lfs_file_t file;
    int ret;
    
    ret = lfs_file_open(&lfs, &file, filename, LFS_O_RDONLY);
    if (ret < 0) {
        LOG_ERR("Failed to open file for read: %d", ret);
        return ret;
    }
    
    ret = lfs_file_read(&lfs, &file, buffer, len);
    if (ret < 0) {
        LOG_ERR("Failed to read file: %d", ret);
        lfs_file_close(&lfs, &file);
        return ret;
    }
    
    lfs_file_close(&lfs, &file);
    LOG_INF("File read successfully: %s (%d bytes)", filename, ret);
    return ret;  // Return number of bytes read
}

/* Write struct data to file */
int mx25l51245g_write_struct(const char *filename, const void *data, size_t size)
{
    lfs_file_t file;
    int ret;
    
    ret = lfs_file_open(&lfs, &file, filename, LFS_O_WRONLY | LFS_O_CREAT);
    if (ret < 0) {
        LOG_ERR("Failed to open file for struct write: %d", ret);
        return ret;
    }
    
    ret = lfs_file_write(&lfs, &file, data, size);
    if (ret < 0) {
        LOG_ERR("Failed to write struct to file: %d", ret);
        lfs_file_close(&lfs, &file);
        return ret;
    }
    
    ret = lfs_file_close(&lfs, &file);
    if (ret < 0) {
        LOG_ERR("Failed to close struct file: %d", ret);
        return ret;
    }
    
    LOG_INF("Struct written successfully: %s (%zu bytes)", filename, size);
    return 0;
}

/* Read struct data from file */
int mx25l51245g_read_struct(const char *filename, void *buffer, size_t size)
{
    lfs_file_t file;
    int ret;
    
    ret = lfs_file_open(&lfs, &file, filename, LFS_O_RDONLY);
    if (ret == LFS_ERR_NOENT) {
        LOG_WRN("File does not exist: %s", filename);
        return -ENOENT;
    } else if (ret < 0) {
        LOG_ERR("Failed to open file for struct read: %d", ret);
        return ret;
    }
    
    ret = lfs_file_read(&lfs, &file, buffer, size);
    if (ret < 0) {
        LOG_ERR("Failed to read struct from file: %d", ret);
        lfs_file_close(&lfs, &file);
        return ret;
    }
    
    if ((size_t)ret != size) {
        LOG_ERR("Struct size mismatch: expected %zu, got %d bytes", size, ret);
        lfs_file_close(&lfs, &file);
        return -EIO;
    }
    
    lfs_file_close(&lfs, &file);
    LOG_INF("Struct read successfully: %s (%zu bytes)", filename, size);
    return ret;  // Return number of bytes read
}

/* Basic flash initialization without LittleFS */
int mx25l51245g_basic_init(void)
{
    LOG_INF("Initializing MX25L51245G flash driver (without LittleFS)...");
    return mx25l51245g_init();
}

/* Simple flash test function */
int mx25l51245g_basic_test(void)
{
    int ret;
    uint32_t test_addr = 0x0000;  // Use address 0 - simplest case
    LOG_INF("Test address: 0x%06X (%u decimal)", test_addr, test_addr);
    uint8_t write_test_data[] = "Hello Flash!";
    uint8_t read_test_data[32];
    
    LOG_INF("Starting basic flash test...");
    
    // Erase sector first
    LOG_INF("Erasing sector at address 0x%06X", test_addr);
    ret = mx25l51245g_write_enable();
    if (ret != 0) {
        LOG_ERR("Write enable for erase failed");
        return ret;
    }
    LOG_INF("Write enable completed for erase");
    
    uint8_t erase_cmd[4] = {0x20, (test_addr >> 16) & 0xFF, (test_addr >> 8) & 0xFF, test_addr & 0xFF};
    LOG_INF("Erase command: [%02X %02X %02X %02X]", 
            erase_cmd[0], erase_cmd[1], erase_cmd[2], erase_cmd[3]);
    ret = mx25l51245g_spi_transceive(erase_cmd, 4, NULL, 0);
    if (ret != 0) {
        LOG_ERR("Erase command failed");
        return ret;
    }
    
    // Wait for erase completion
    k_msleep(100);
    
    // Write data
    LOG_INF("Writing data to flash: %s", write_test_data);
    ret = mx25l51245g_write_enable();
    if (ret != 0) {
        LOG_ERR("Write enable for program failed");
        return ret;
    }
    LOG_INF("Write enable completed for program");
    
    // Check status register to verify write enable worked
    uint8_t status;
    ret = mx25l51245g_read_status(&status);
    if (ret == 0) {
        LOG_INF("Status register after write enable: 0x%02X", status);
        if (status & 0x02) {
            LOG_INF("Write Enable Latch (WEL) is set - ready to write");
        } else {
            LOG_ERR("Write Enable Latch (WEL) is NOT set - write will fail!");
        }
    }
    
    uint8_t write_cmd[4 + sizeof(write_test_data)];
    write_cmd[0] = 0x02;  // Page program
    write_cmd[1] = (test_addr >> 16) & 0xFF;
    write_cmd[2] = (test_addr >> 8) & 0xFF;
    write_cmd[3] = test_addr & 0xFF;
    memcpy(write_cmd + 4, write_test_data, sizeof(write_test_data));
    
    // Debug: show what we're writing
    LOG_INF("Write command: [%02X %02X %02X %02X] + data", 
            write_cmd[0], write_cmd[1], write_cmd[2], write_cmd[3]);
    LOG_INF("Write data: [%02X %02X %02X %02X %02X %02X %02X %02X]",
            write_cmd[4], write_cmd[5], write_cmd[6], write_cmd[7],
            write_cmd[8], write_cmd[9], write_cmd[10], write_cmd[11]);
    
    ret = mx25l51245g_spi_transceive(write_cmd, 4 + sizeof(write_test_data), NULL, 0);
    if (ret != 0) {
        LOG_ERR("Write command failed");
        return ret;
    }
    
    // Check status immediately after write command
    ret = mx25l51245g_read_status(&status);
    if (ret == 0) {
        LOG_INF("Status after write command: 0x%02X", status);
        if (status & 0x01) {
            LOG_INF("Device is busy (BUSY bit set)");
        }
        if (status & 0x02) {
            LOG_WRN("WEL still set after write - this is unusual");
        } else {
            LOG_INF("WEL cleared after write - normal behavior");
        }
    }
    
    // Wait for write completion
    k_msleep(10);
    
    // Check if write completed by reading status
    ret = mx25l51245g_wait_ready();
    if (ret != 0) {
        LOG_ERR("Write operation did not complete");
        return ret;
    }
    LOG_INF("Write operation completed successfully");
    
    // First, test if SPI is working by reading device ID again
    LOG_INF("Testing SPI by reading device ID again...");
    uint8_t id_test[3];
    ret = mx25l51245g_read_id(id_test);
    if (ret == 0) {
        LOG_INF("Device ID test: %02X %02X %02X", id_test[0], id_test[1], id_test[2]);
    } else {
        LOG_ERR("Device ID test failed");
        return ret;
    }
    
    // Now try reading from address 0x0000 (where we know there's data from basic test)
    LOG_INF("Reading from address 0x0000 first...");
    uint8_t tx_buf_0[8] = {0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx_buf_0[8];
    
    ret = mx25l51245g_spi_transceive(tx_buf_0, 8, rx_buf_0, 8);
    if (ret == 0) {
        LOG_INF("Read from 0x0000: [%02X %02X %02X %02X %02X %02X %02X %02X]",
                rx_buf_0[0], rx_buf_0[1], rx_buf_0[2], rx_buf_0[3],
                rx_buf_0[4], rx_buf_0[5], rx_buf_0[6], rx_buf_0[7]);
    }
    
    // Try reading using the exact same pattern as device ID read
    LOG_INF("Reading data from flash at 0x%06X using device ID pattern...", test_addr);
    uint8_t tx_buf[16] = {
        0x03,  // Read command
        (test_addr >> 16) & 0xFF, 
        (test_addr >> 8) & 0xFF, 
        test_addr & 0xFF,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0  // Dummy bytes for data
    };
    
    LOG_INF("Read command: [%02X %02X %02X %02X]", 
            tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3]);
    uint8_t rx_buf[16];
    
    ret = mx25l51245g_spi_transceive(tx_buf, 16, rx_buf, 16);
    if (ret != 0) {
        LOG_ERR("Read command failed");
        return ret;
    }
    
    LOG_INF("RX buffer first 8: [%02X %02X %02X %02X %02X %02X %02X %02X]",
            rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3],
            rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7]);
    LOG_INF("RX buffer next 8:  [%02X %02X %02X %02X %02X %02X %02X %02X]",
            rx_buf[8], rx_buf[9], rx_buf[10], rx_buf[11],
            rx_buf[12], rx_buf[13], rx_buf[14], rx_buf[15]);
    
    // Copy read data starting from byte 4 (after command + address)
    memcpy(read_test_data, rx_buf + 4, 12);
    
    LOG_INF("Read data: %s", read_test_data);
    LOG_INF("First few bytes: [%02X %02X %02X %02X]", 
            read_test_data[0], read_test_data[1], read_test_data[2], read_test_data[3]);
    
    // Verify data - compare the actual string content
    if (strncmp((char*)write_test_data, (char*)read_test_data, strlen((char*)write_test_data)) == 0) {
        LOG_INF("🎉 Flash read/write test PASSED! 🎉");
        LOG_INF("Successfully wrote and read: %s", write_test_data);
        return 0;
    } else {
        LOG_ERR("Flash read/write test FAILED!");
        LOG_ERR("Expected: %s", write_test_data);
        LOG_ERR("Got: %s", read_test_data);
        return -1;
    }
}

/* Main initialization function */
int mx25l51245g_system_init(void)
{
    int ret;
    
    LOG_INF("Initializing MX25L51245G NOR Flash system...");
    
    ret = mx25l51245g_init();
    if (ret != 0) {
        LOG_ERR("MX25L51245G initialization failed: %d", ret);
        return ret;
    }
    
    LOG_INF("Basic flash driver initialized, initializing LittleFS...");
    
    ret = littlefs_init();
    if (ret != 0) {
        LOG_ERR("LittleFS initialization failed: %d", ret);
        return ret;
    }
    
    LOG_INF("MX25L51245G system initialized successfully");
    return 0;
}