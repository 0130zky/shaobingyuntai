#include "storage.h"
#include "gimbal_config.h"
#ifdef SENTRY_STM32H7
#include "stm32h7xx_hal.h"
#else
#include "stm32f4xx_hal.h"
#endif
#include <string.h>
#include <stddef.h>
#ifndef STORE_ADDRESS
#define STORE_ADDRESS 0x080E0000UL
#endif
#define STORE_MAGIC 0x534E5431UL
/* Last 128KiB sector (F407:11, H723:7) is exclusively reserved by the Sentry linker script.
 * Commit marker is programmed last; interrupted writes cannot become valid. */
typedef struct { uint32_t ids[4]; float ranges[3]; } Identity;
static const Identity identity={{SENTRY_FOLD_ESC,SENTRY_FOLD_MST,SENTRY_PITCH_ESC,SENTRY_PITCH_MST},
    {SENTRY_P_MAX,SENTRY_V_MAX,SENTRY_T_MAX}};
typedef struct { Identity identity; uint32_t version; SentryCalibration cal; uint32_t hash;
#ifdef SENTRY_STM32H7
    /* Commit in a separate 32-byte flashword, programmed exactly once. */
    uint32_t reserved[6];
#endif
    uint32_t commit;
#ifdef SENTRY_STM32H7
    uint32_t commit_guard[6],commit_end;
#endif
} Record;
_Static_assert(sizeof(Record)%4==0,"word-aligned record");
static uint32_t Hash(const Record *r) {
    const uint8_t *p=(const uint8_t *)r; uint32_t h=2166136261UL;
    for(unsigned i=0;i<offsetof(Record,hash);i++) h=(h^p[i])*16777619UL;
    return h;
}
int Sentry_Load(SentryCalibration *c) {
    Record r; memcpy(&r,(const void *)STORE_ADDRESS,sizeof(r));
    if(memcmp(&r.identity,&identity,sizeof(identity)) || r.version!=2 || r.commit!=STORE_MAGIC || r.hash!=Hash(&r)||
       !Sentry_CalValid(&r.cal,SENTRY_P_MAX,SENTRY_MAX_TRAVEL_RAD,SENTRY_MARGIN_RAD)) return 0;
#ifdef SENTRY_STM32H7
    if(r.commit_end!=~(uint32_t)STORE_MAGIC)return 0;
#endif
    *c=r.cal; return 1;
}
int Sentry_Save(const SentryCalibration *c) {
    if(!Sentry_CalValid(c,SENTRY_P_MAX,SENTRY_MAX_TRAVEL_RAD,SENTRY_MARGIN_RAD)) return 0;
    Record r={0}; r.identity=identity; r.version=2; r.cal=*c; r.hash=Hash(&r); r.commit=STORE_MAGIC;
#ifdef SENTRY_STM32H7
    r.commit_end=~(uint32_t)STORE_MAGIC;
#endif
    FLASH_EraseInitTypeDef erase={0}; uint32_t error=0;
    erase.TypeErase=FLASH_TYPEERASE_SECTORS;
#ifdef SENTRY_STM32H7
    erase.Banks=FLASH_BANK_1;erase.Sector=FLASH_SECTOR_7;
#else
    erase.Sector=FLASH_SECTOR_11;
#endif
    erase.NbSectors=1; erase.VoltageRange=FLASH_VOLTAGE_RANGE_3;
    if(HAL_FLASH_Unlock()!=HAL_OK) return 0;
    int ok=HAL_FLASHEx_Erase(&erase,&error)==HAL_OK;
#ifdef SENTRY_STM32H7
    _Static_assert(offsetof(Record,commit)%32==0,"commit must start new flashword");
    for(unsigned i=0;ok && i<sizeof(r);i+=32U) {
        uint32_t block[8] __attribute__((aligned(32)));
        memset(block,0xff,sizeof(block));
        unsigned n=sizeof(r)-i; if(n>32U)n=32U;
        memcpy(block,(const uint8_t *)&r+i,n);
        ok=HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,STORE_ADDRESS+i,(uintptr_t)block)==HAL_OK;
    }
#else
    for(unsigned i=0;ok && i<sizeof(r)/4;i++) {
        uint32_t word; memcpy(&word,(const uint8_t *)&r+i*4,4);
        ok=HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,STORE_ADDRESS+i*4,word)==HAL_OK;
    }
#endif
    HAL_FLASH_Lock();
    SentryCalibration check;
    return ok && Sentry_Load(&check) && memcmp(c,&check,sizeof(check))==0;
}
