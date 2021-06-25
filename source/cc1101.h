#ifndef __CC1101_H__
#define __CC1101_H__

#include <stdint.h>

//#define CC_SENDER
#define CC_TESTER

#ifdef CC_SENDER
    #define CC_DEFAULT_ADDR 1
#else
    #define CC_DEFAULT_ADDR 2
#endif

#define CC_BUFF_LEN     64
#define CC_DEFAULT_CH   0x0E

/* R/W offsets */
#define CC_BURST_ACCESS   0x40
#define CC_SINGLE_ACCESS  0x00
#define CC_READ_ACCESS    0x80
#define CC_WRITE_ACCESS   0x00

/* Strobe commands */
#define CC_CMD_SRES 0x30 /* Reset chip */
#define CC_CMD_SRX  0x34 /* Enable RX */
#define CC_CMD_STX  0x35 /* Enable TX */
#define CC_CMD_IDLE 0x36 /* Goto IDLE state */
#define CC_CMD_SFRX 0x3A /* Flush RX FIFO buffer */
#define CC_CMD_SFTX 0x3B /* Flush TX FIFO buffer */
#define CC_CMD_SNOP 0x3D /* No operation */

/* Status register */
#define CC_ST_REG_PARTNUM        0x30 /* Part number */
#define CC_ST_REG_VERSION        0x31 /* Current version number */
#define CC_ST_REG_FREQEST        0x32 /* Frequency offset estimate */
#define CC_ST_REG_LQI            0x33 /* Demodulator estimate for link quality */
#define CC_ST_REG_RSSI           0x34 /* Received signal strength indication */
#define CC_ST_REG_MARCSTATE      0x35 /* Control state machine state */
#define CC_ST_REG_WORTIME1       0x36 /* High byte of WOR timer */
#define CC_ST_REG_WORTIME0       0x37 /* Low byte of WOR timer */
#define CC_ST_REG_PKTSTATUS      0x38 /* Current GDOx status and packet status */
#define CC_ST_REG_VCO_VC_DAC     0x39 /* Current setting from PLL cal module */
#define CC_ST_REG_TXBYTES        0x3A /* Underflow and number of bytes in TXFIFO */
#define CC_ST_REG_RXBYTES        0x3B /* Overflow and number of bytes in RXFIFO */
#define CC_ST_REG_RCCTRL1_STATUS 0x3C /* Last RC Oscillator Calibration Result */
#define CC_ST_REG_RCCTRL0_STATUS 0x3D /* Last RC Oscillator Calibration Result */

/* Config register */
#define CC_CFG_REG_IOCFG2   0x00 /* GDO2 output pin configuration */
#define CC_CFG_REG_IOCFG1   0x01 /* GDO1 output pin configuration */
#define CC_CFG_REG_IOCFG0   0x02 /* GDO0 output pin configuration */
#define CC_CFG_REG_FIFOTHR  0x03 /* RX FIFO and TX FIFO thresholds */
#define CC_CFG_REG_SYNC1    0x04 /* Sync word, high byte */
#define CC_CFG_REG_SYNC0    0x05 /* Sync word, low byte */
#define CC_CFG_REG_PKTLEN   0x06 /* Packet length */
#define CC_CFG_REG_PKTCTRL1 0x07 /* Packet automation control */
#define CC_CFG_REG_PKTCTRL0 0x08 /* Packet automation control */
#define CC_CFG_REG_ADDR     0x09 /* Device address */
#define CC_CFG_REG_CHANNR   0x0A /* Channel number */
#define CC_CFG_REG_FSCTRL1  0x0B /* Frequency synthesizer control */
#define CC_CFG_REG_FSCTRL0  0x0C /* Frequency synthesizer control */
#define CC_CFG_REG_FREQ2    0x0D /* Frequency control word, high byte */
#define CC_CFG_REG_FREQ1    0x0E /* Frequency control word, middle byte */
#define CC_CFG_REG_FREQ0    0x0F /* Frequency control word, low byte */
#define CC_CFG_REG_MDMCFG4  0x10 /* Modem configuration */
#define CC_CFG_REG_MDMCFG3  0x11 /* Modem configuration */
#define CC_CFG_REG_MDMCFG2  0x12 /* Modem configuration */
#define CC_CFG_REG_MDMCFG1  0x13 /* Modem configuration */
#define CC_CFG_REG_MDMCFG0  0x14 /* Modem configuration */
#define CC_CFG_REG_DEVIATN  0x15 /* Modem deviation setting */
#define CC_CFG_REG_MCSM2    0x16 /* Main Radio Cntrl State Machine config */
#define CC_CFG_REG_MCSM1    0x17 /* Main Radio Cntrl State Machine config */
#define CC_CFG_REG_MCSM0    0x18 /* Main Radio Cntrl State Machine config */
#define CC_CFG_REG_FOCCFG   0x19 /* Frequency Offset Compensation config */
#define CC_CFG_REG_BSCFG    0x1A /* Bit Synchronization configuration */
#define CC_CFG_REG_AGCCTRL2 0x1B /* AGC control */
#define CC_CFG_REG_AGCCTRL1 0x1C /* AGC control */
#define CC_CFG_REG_AGCCTRL0 0x1D /* AGC control */
#define CC_CFG_REG_WOREVT1  0x1E /* High byte Event 0 timeout */
#define CC_CFG_REG_WOREVT0  0x1F /* Low byte Event 0 timeout */
#define CC_CFG_REG_WORCTRL  0x20 /* Wake On Radio control */
#define CC_CFG_REG_FREND1   0x21 /* Front end RX configuration */
#define CC_CFG_REG_FREND0   0x22 /* Front end TX configuration */
#define CC_CFG_REG_FSCAL3   0x23 /* Frequency synthesizer calibration */
#define CC_CFG_REG_FSCAL2   0x24 /* Frequency synthesizer calibration */
#define CC_CFG_REG_FSCAL1   0x25 /* Frequency synthesizer calibration */
#define CC_CFG_REG_FSCAL0   0x26 /* Frequency synthesizer calibration */
#define CC_CFG_REG_RCCTRL1  0x27 /* RC oscillator configuration */
#define CC_CFG_REG_RCCTRL0  0x28 /* RC oscillator configuration */
#define CC_CFG_REG_FSTEST   0x29 /* Frequency synthesizer cal control */
#define CC_CFG_REG_PTEST    0x2A /* Production test */
#define CC_CFG_REG_AGCTEST  0x2B /* AGC test */
#define CC_CFG_REG_TEST2    0x2C /* Various test settings */
#define CC_CFG_REG_TEST1    0x2D /* Various test settings */
#define CC_CFG_REG_TEST0    0x2E /* Various test settings */

#define CC_REG_CFG      0x00
#define CC_REG_PATABLE  0x3E
#define CC_REG_TXFIFO   0x7F
#define CC_REG_TXFIFO_S 0x3F
#define CC_REG_RXFIFO   0xFF

uint8_t cc_init( void );
uint8_t cc_read_reg( uint8_t addr, uint8_t* data, uint8_t len );
uint8_t cc_write_reg( uint8_t addr, uint8_t* data, uint8_t len );
uint8_t cc_send( uint8_t* data );
uint8_t cc_receive( uint8_t* data );
uint8_t cc_crc( uint8_t* data );
int8_t cc_calc_rssi( uint8_t raw );

#endif /* __CC1101_H__ */
