#ifndef BSP_FLASH_H
#define BSP_FLASH_H

#include <stdint.h>

/* STM32F407ZG的Sector 11：0x080E0000~0x080FFFFF，共128 KB。 */
#define BSP_FLASH_STORAGE_ADDRESS         (0x080E0000UL)
#define BSP_FLASH_STORAGE_SIZE            (0x00020000UL)

/* valid_mask的每一位表示对应类别的标定字段是否可由ALOG使用。 */
#define BSP_CAL_VALID_LENGTH              (1UL << 0)
#define BSP_CAL_VALID_RESISTANCE          (1UL << 1)
#define BSP_CAL_VALID_CAPACITANCE         (1UL << 2)
#define BSP_CAL_VALID_CLASSIFY            (1UL << 3)

/*
 * 标定数据只使用32位字段，便于按Flash Word写入并保持记录格式稳定。
 * BSP只负责原样保存和读取，不解释其中公式；未启用的类别由valid_mask屏蔽。
 */
typedef struct
{
    uint32_t valid_mask;

    /* 长度：后续可用于raw到cm的线性模型。 */
    float length_gain_cm_per_raw;
    float length_offset_cm;
    float propagation_velocity_m_per_s;
    float length_reserved;

    /* 电阻：分压换算、线性修正和电缆寄生电阻补偿。 */
    float adc_vref_v;
    float resistance_reference_ohm;
    float resistance_gain;
    float resistance_offset_ohm;
    float cable_resistance_ohm_per_m;

    /* 电容：NE555器件实测值、周期换算和电缆寄生电容补偿。 */
    float ne555_r1_ohm;
    float ne555_r2_ohm;
    float capacitance_gain_pf_per_tick;
    float capacitance_offset_pf;
    float cable_capacitance_pf_per_m;

    /* 负载分类阈值，具体含义由后续ALOG算法定义。 */
    float resistor_detect_threshold;
    float capacitor_detect_threshold;

    /* 预留字段用于后续扩展，避免轻微增项就立即改变记录版本。 */
    float reserved_f32[8];
    uint32_t reserved_u32[8];
} bsp_calibration_data_t;

typedef enum
{
    BSP_FLASH_RESULT_OK = 0,
    BSP_FLASH_RESULT_EMPTY,
    BSP_FLASH_RESULT_INVALID_PARAM,
    BSP_FLASH_RESULT_INVALID_RECORD,
    BSP_FLASH_RESULT_ERASE_ERROR,
    BSP_FLASH_RESULT_PROGRAM_ERROR,
    BSP_FLASH_RESULT_VERIFY_ERROR
} bsp_flash_result_t;

/** @brief 从Sector 11读取并校验标定数据，失败时不修改调用者数据。 */
bsp_flash_result_t bsp_flash_load(bsp_calibration_data_t *data);

/** @brief 擦除Sector 11并按32位写入、读回校验一份标定记录。 */
bsp_flash_result_t bsp_flash_save(const bsp_calibration_data_t *data);

/** @brief 只擦除Sector 11，并确认记录首字恢复为擦除态。 */
bsp_flash_result_t bsp_flash_erase(void);

#endif /* BSP_FLASH_H */
