/* Auto-generated config file conf_sd_mmc.h */
#ifndef CONF_SD_MMC_H
#define CONF_SD_MMC_H

// <q> Enable the SDIO support
// <id> conf_sdio_support
#define CONF_SDIO_SUPPORT 0

// <q> Enable the MMC card support
// <id> conf_mmc_support
#define CONF_MMC_SUPPORT 1

// <q> Enable the OS support
// <id> conf_sd_mmc_os_support
#ifndef CONF_OS_SUPPORT
#define CONF_OS_SUPPORT 0
#endif

// Detection (card/write protect) timeout (ms/ticks)
// conf_sd_mmc_debounce
#ifndef CONF_SD_MMC_DEBOUNCE
#define CONF_SD_MMC_DEBOUNCE 1000
#endif

#define CONF_SD_MMC_MEM_CNT 2

// <e> SD/MMC Slot 0
// <id> conf_sd_mmc_0_enable
#define CONF_SD_MMC_0_ENABLE 1

// <e> Card Detect (CD) 0 Enable
// <id> conf_sd_mmc_0_cd_detect_en
#ifndef CONF_SD_MMC_0_CD_DETECT_EN
#define CONF_SD_MMC_0_CD_DETECT_EN 0
#endif

// <o> Card Detect (CD) detection level
// <1=> High
// <0=> Low
// <id> conf_sd_mmc_0_cd_detect_value
#ifndef CONF_SD_MMC_0_CD_DETECT_VALUE
#define CONF_SD_MMC_0_CD_DETECT_VALUE 0
#endif
// </e>

// <e> Write Protect (WP) 0 Enable
// <id> conf_sd_mmc_0_wp_detect_en
#ifndef CONF_SD_MMC_0_WP_DETECT_EN
#define CONF_SD_MMC_0_WP_DETECT_EN 0
#endif

// <o> Write Protect (WP) detection level
// <1=> High
// <0=> Low
// <id> conf_sd_mmc_0_wp_detect_value
#ifndef CONF_SD_MMC_0_WP_DETECT_VALUE
#define CONF_SD_MMC_0_WP_DETECT_VALUE 1
#endif
// </e>

// </e>

#ifndef CONF_MCI_OS_SUPPORT
#define CONF_MCI_OS_SUPPORT 0
#endif

// <<< end of configuration section >>>

#endif // CONF_SD_MMC_H
