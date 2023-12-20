/*
 * W25qxx.h
 *
 *  Created on: Aug 16, 2023
 *      Author: rinaldo.santos
 */

#ifndef W25QXX_H_
#define W25QXX_H_


#ifdef DEBUGxxx
#define W25_DBG(...) printf(__VA_ARGS__);\
                     printf("\n")
#else
#define W25_DBG(...)
#endif

#define W25QXX_MANUFACTURER_GIGADEVICE	0xC8
#define W25QXX_MANUFACTURER_WINBOND		0xEF

#define W25QXX_DUMMY_BYTE       		0xA5
#define W25QXX_GET_ID           		0x9F
#define W25QXX_READ_DATA        		0x03
#define W25QXX_WRITE_ENABLE     		0x06
#define W25QXX_PAGE_PROGRAM     		0x02
#define W25QXX_SECTOR_ERASE	    		0x20
#define W25QXX_CHIP_ERASE       		0xc7
#define W25QXX_READ_REGISTER_1			0x05

#define ADDR_ID							0x00
#define ADDR_REV						0x02
#define PRG_REVISION					0x01
//
#define TARGET_IRON						0x10
#define TARGET_AIR						0x14
#define TARGET_GUN						0x18

typedef struct {
#ifdef W25QXX_QSPI
    QSPI_HandleTypeDef *qspiHandle;
#else
    SPI_HandleTypeDef *spiHandle;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
#endif
    uint8_t manufacturer_id;
    uint16_t device_id;
    uint32_t block_size;
    uint32_t block_count;
    uint32_t sector_size;
    uint32_t sectors_in_block;
    uint32_t page_size;
    uint32_t pages_in_sector;
} W25QXX_HandleTypeDef;

typedef enum {
    W25QXX_Ok,     // 0
    W25QXX_Err,    // 1
    W25QXX_Timeout // 2
} W25QXX_result_t;

#ifdef W25QXX_QSPI
W25QXX_result_t w25qxx_init(W25QXX_HandleTypeDef *w25qxx, QSPI_HandleTypeDef *qhspi);
#else
W25QXX_result_t w25qxx_init(W25QXX_HandleTypeDef *w25qxx, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
#endif
W25QXX_result_t w25qxx_read(W25QXX_HandleTypeDef *w25qxx, uint32_t address, uint8_t *buf, uint32_t len);
W25QXX_result_t w25qxx_write(W25QXX_HandleTypeDef *w25qxx, uint32_t address, uint8_t *buf, uint32_t len);
W25QXX_result_t w25qxx_erase(W25QXX_HandleTypeDef *w25qxx, uint32_t address, uint32_t len);
W25QXX_result_t w25qxx_chip_erase(W25QXX_HandleTypeDef *w25qxx);
void ReadID_EEPROM(void);
void Save_Default_EEPROM(void);
void Read_Config_EEPROM(void);

#endif /* W25QXX_H_ */
