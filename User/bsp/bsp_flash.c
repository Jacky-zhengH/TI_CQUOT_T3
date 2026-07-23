#include "bsp_flash.h"

#include <stddef.h>
#include <string.h>

#include "stm32f4xx_hal.h"

/* "CAL1"用于快速识别记录类型；版本变化表示记录布局不再兼容。 */
#define BSP_FLASH_MAGIC                   (0x43414C31UL)
#define BSP_FLASH_VERSION                 (1UL)
#define BSP_FLASH_SEQUENCE_FIRST          (1UL)

/*
 * CRC32使用常见的反射多项式0xEDB88320，初值和最终异或均为0xFFFFFFFF。
 * CRC覆盖头部和payload，不包含末尾crc32字段本身。
 */
#define BSP_FLASH_CRC32_POLYNOMIAL        (0xEDB88320UL)
#define BSP_FLASH_CRC32_INITIAL           (0xFFFFFFFFUL)
#define BSP_FLASH_CRC32_FINAL_XOR         (0xFFFFFFFFUL)

#define BSP_FLASH_ALL_ERROR_FLAGS         (FLASH_FLAG_EOP   | \
                                           FLASH_FLAG_OPERR | \
                                           FLASH_FLAG_WRPERR| \
                                           FLASH_FLAG_PGAERR| \
                                           FLASH_FLAG_PGPERR| \
                                           FLASH_FLAG_PGSERR)

typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t payload_size;
    uint32_t sequence;
    bsp_calibration_data_t payload;
    uint32_t crc32;
} bsp_flash_record_t;

/* ARMCC5工程未统一启用C11，使用数组长度完成等价的编译期静态断言。 */
typedef char bsp_flash_float_must_be_32_bit[(sizeof(float) == 4U) ? 1 : -1];
typedef char bsp_flash_payload_must_be_word_aligned[
    ((sizeof(bsp_calibration_data_t) % sizeof(uint32_t)) == 0U) ? 1 : -1];
typedef char bsp_flash_record_must_be_word_aligned[
    ((sizeof(bsp_flash_record_t) % sizeof(uint32_t)) == 0U) ? 1 : -1];
typedef char bsp_flash_record_must_fit_sector[
    (sizeof(bsp_flash_record_t) <= BSP_FLASH_STORAGE_SIZE) ? 1 : -1];
typedef char bsp_flash_address_must_be_word_aligned[
    ((BSP_FLASH_STORAGE_ADDRESS % sizeof(uint32_t)) == 0U) ? 1 : -1];

/*
 * 生产固件直接读取内部Flash映射地址。单元测试可在编译时替换此宏，
 * 将相同读写逻辑指向RAM模拟区，不会改变目标固件行为。
 */
#ifndef BSP_FLASH_MEMORY_WORDS
#define BSP_FLASH_MEMORY_WORDS \
    ((volatile const uint32_t *)BSP_FLASH_STORAGE_ADDRESS)
#endif

static uint32_t bsp_flash_crc32(const uint8_t *data, size_t length)
{
    uint32_t crc = BSP_FLASH_CRC32_INITIAL;
    uint32_t bit_index;
    size_t byte_index;

    for (byte_index = 0U; byte_index < length; byte_index++)
    {
        crc ^= data[byte_index];
        for (bit_index = 0U; bit_index < 8U; bit_index++)
        {
            crc = ((crc & 1U) != 0U) ?
                  ((crc >> 1U) ^ BSP_FLASH_CRC32_POLYNOMIAL) :
                  (crc >> 1U);
        }
    }

    return crc ^ BSP_FLASH_CRC32_FINAL_XOR;
}

static void bsp_flash_read_record(bsp_flash_record_t *record)
{
    uint32_t word;
    size_t word_index;

    /*
     * 逐Word读取可避免未对齐访问；再用memcpy写入RAM结构，避免违反
     * C语言严格别名规则。记录大小已经由编译期断言保证为4字节倍数。
     */
    for (word_index = 0U;
         word_index < (sizeof(*record) / sizeof(uint32_t));
         word_index++)
    {
        word = BSP_FLASH_MEMORY_WORDS[word_index];
        memcpy(((uint8_t *)record) + word_index * sizeof(uint32_t),
               &word,
               sizeof(word));
    }
}

static bsp_flash_result_t bsp_flash_validate_record(
    const bsp_flash_record_t *record)
{
    uint32_t expected_crc;

    if ((record->magic != BSP_FLASH_MAGIC) ||
        (record->version != BSP_FLASH_VERSION) ||
        (record->payload_size != sizeof(record->payload)))
    {
        return BSP_FLASH_RESULT_INVALID_RECORD;
    }

    expected_crc = bsp_flash_crc32(
        (const uint8_t *)record,
        offsetof(bsp_flash_record_t, crc32));
    return (expected_crc == record->crc32) ?
           BSP_FLASH_RESULT_OK : BSP_FLASH_RESULT_INVALID_RECORD;
}

static void bsp_flash_reset_data_cache(void)
{
    /*
     * STM32F407这里是Flash接口数据缓存，不是Cortex-M7的D-Cache。
     * 仅在原本已启用时执行“关闭、复位、重新启用”，避免读回旧缓存行。
     */
    if ((FLASH->ACR & FLASH_ACR_DCEN) != 0U)
    {
        __HAL_FLASH_DATA_CACHE_DISABLE();
        __HAL_FLASH_DATA_CACHE_RESET();
        __HAL_FLASH_DATA_CACHE_ENABLE();
    }
}

static HAL_StatusTypeDef bsp_flash_erase_sector(void)
{
    FLASH_EraseInitTypeDef erase_config;
    uint32_t sector_error = 0U;

    erase_config.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_config.Sector = FLASH_SECTOR_11;
    erase_config.NbSectors = 1U;
    erase_config.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    if (HAL_FLASHEx_Erase(&erase_config, &sector_error) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* HAL成功时sector_error应保持擦除API约定的0xFFFFFFFF。 */
    return (sector_error == 0xFFFFFFFFUL) ? HAL_OK : HAL_ERROR;
}

bsp_flash_result_t bsp_flash_load(bsp_calibration_data_t *data)
{
    bsp_flash_record_t record;
    bsp_flash_result_t result;

    if (data == NULL)
    {
        return BSP_FLASH_RESULT_INVALID_PARAM;
    }
    if (BSP_FLASH_MEMORY_WORDS[0] == 0xFFFFFFFFUL)
    {
        return BSP_FLASH_RESULT_EMPTY;
    }

    bsp_flash_read_record(&record);
    result = bsp_flash_validate_record(&record);
    if (result == BSP_FLASH_RESULT_OK)
    {
        /* 只有完整头部和CRC均通过后，才修改调用者保存的RAM副本。 */
        memcpy(data, &record.payload, sizeof(*data));
    }
    return result;
}

bsp_flash_result_t bsp_flash_save(const bsp_calibration_data_t *data)
{
    bsp_flash_record_t record;
    bsp_flash_record_t verify_record;
    bsp_flash_result_t result = BSP_FLASH_RESULT_OK;
    uint32_t address = BSP_FLASH_STORAGE_ADDRESS;
    uint32_t word;
    size_t word_index;

    if (data == NULL)
    {
        return BSP_FLASH_RESULT_INVALID_PARAM;
    }

    memset(&record, 0, sizeof(record));
    record.magic = BSP_FLASH_MAGIC;
    record.version = BSP_FLASH_VERSION;
    record.payload_size = sizeof(record.payload);
    record.sequence = BSP_FLASH_SEQUENCE_FIRST;
    memcpy(&record.payload, data, sizeof(record.payload));
    record.crc32 = bsp_flash_crc32(
        (const uint8_t *)&record,
        offsetof(bsp_flash_record_t, crc32));

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return BSP_FLASH_RESULT_PROGRAM_ERROR;
    }

    __HAL_FLASH_CLEAR_FLAG(BSP_FLASH_ALL_ERROR_FLAGS);
    if (bsp_flash_erase_sector() != HAL_OK)
    {
        result = BSP_FLASH_RESULT_ERASE_ERROR;
        goto lock_flash;
    }

    for (word_index = 0U;
         word_index < (sizeof(record) / sizeof(uint32_t));
         word_index++)
    {
        memcpy(&word,
               ((const uint8_t *)&record) +
                   word_index * sizeof(uint32_t),
               sizeof(word));
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                              address,
                              word) != HAL_OK)
        {
            result = BSP_FLASH_RESULT_PROGRAM_ERROR;
            goto reset_cache;
        }
        address += sizeof(uint32_t);
    }

reset_cache:
    bsp_flash_reset_data_cache();
    if (result != BSP_FLASH_RESULT_OK)
    {
        goto lock_flash;
    }

    bsp_flash_read_record(&verify_record);
    if (memcmp(&verify_record, &record, sizeof(record)) != 0)
    {
        result = BSP_FLASH_RESULT_VERIFY_ERROR;
        goto lock_flash;
    }
    if (bsp_flash_validate_record(&verify_record) != BSP_FLASH_RESULT_OK)
    {
        result = BSP_FLASH_RESULT_VERIFY_ERROR;
    }

lock_flash:
    if ((HAL_FLASH_Lock() != HAL_OK) &&
        (result == BSP_FLASH_RESULT_OK))
    {
        result = BSP_FLASH_RESULT_PROGRAM_ERROR;
    }
    return result;
}

bsp_flash_result_t bsp_flash_erase(void)
{
    bsp_flash_result_t result = BSP_FLASH_RESULT_OK;

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return BSP_FLASH_RESULT_ERASE_ERROR;
    }

    __HAL_FLASH_CLEAR_FLAG(BSP_FLASH_ALL_ERROR_FLAGS);
    if (bsp_flash_erase_sector() != HAL_OK)
    {
        result = BSP_FLASH_RESULT_ERASE_ERROR;
    }
    bsp_flash_reset_data_cache();

    if ((result == BSP_FLASH_RESULT_OK) &&
        (BSP_FLASH_MEMORY_WORDS[0] != 0xFFFFFFFFUL))
    {
        result = BSP_FLASH_RESULT_VERIFY_ERROR;
    }
    if ((HAL_FLASH_Lock() != HAL_OK) &&
        (result == BSP_FLASH_RESULT_OK))
    {
        result = BSP_FLASH_RESULT_ERASE_ERROR;
    }
    return result;
}
