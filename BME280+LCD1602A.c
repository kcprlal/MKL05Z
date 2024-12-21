#include <stdlib.h>
#include <stdbool.h>
#include "lcd1602.h"
#include <stdint.h>

#define BME280_PRESS_MSB  0xf7
#define BME280_PRESS_LSB  0xf8
#define BME280_PRESS_XLSB 0xf9
#define BME280_TEMP_MSB   0xfa
#define BME280_TEMP_LSB   0xfb
#define BME280_TEMP_XLSB  0xfc
#define BME280_HUM_MSB 0xfd
#define BME280_HUM_LSB 0xfe

#define addr_dig_T1 0x88
#define addr_dig_T2 0x8a
#define addr_dig_T3 0x8c

#define addr_dig_P1 0x8e
#define addr_dig_P2 0x90
#define addr_dig_P3 0x92
#define addr_dig_P4 0x94
#define addr_dig_P5 0x96
#define addr_dig_P6 0x98
#define addr_dig_P7 0x9a
#define addr_dig_P8 0x9c
#define addr_dig_P9 0x9e
#define writeaddr 0x76
#define readaddr 0x76
#define addr_dig_H1 0xa1
#define addr_dig_H2 0xe1
#define addr_dig_H3 0xe3
#define addr_dig_H4 0xe4
#define addr_dig_H5 0xe5
#define addr_dig_H6 0xe7

#define ctrl_hum 0xf2
#define ctrl_meas 0xf4
// prototypy 
void bme280_init(void);
void bme280_readcalibs(void);
void read_pth(volatile int32_t*, volatile int32_t*, volatile int32_t*);
void _delay_ms(uint32_t);
int32_t bme280_temp32_compensate(int32_t);
uint32_t bme280_press64_compensate(int32_t);
void PIT_Init(void);
void PIT_IRQHandler(void);
void RGB_LED_Init(void);
uint32_t bme280_humidity_compensate(int32_t);
//zmienne globalne
static uint8_t dig_H1, dig_H3, utempdig;
static int8_t dig_H6;
static uint16_t dig_T1, dig_P1;
static int16_t dig_H2, dig_H4, dig_H5, dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static volatile int32_t t_fine, adc_T, adc_P, adc_H, temp;
static volatile uint32_t press, hum;
static char display[]={0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20};


void _delay_ms(uint32_t ms) {
    volatile uint32_t i, j;
    for (i = 0; i < ms; i++) {
        for (j = 0; j < 4800; j++) {
            __asm("NOP");
        }
    }
}


int32_t bme280_temp32_compensate(){
	int32_t var1, var2, T;
	var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

uint32_t bme280_press64_compensate(){
	int64_t var1, var2, p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
	var2 = var2 + (((int64_t)dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
	if (var1 == 0) {
		return 0;
	}
	
	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)dig_P9) * (p >> 13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
	return (uint32_t)p/25600;
}

	
uint32_t bme280_humidity_compensate()
{
    int32_t v_x1_u32r;

    v_x1_u32r = (t_fine - ((int32_t)76800));

    v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) -
                    (((int32_t)dig_H5) * v_x1_u32r)) +
                   ((int32_t)16384)) >>
                  15) *
                 (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) *
                      (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) +
                       ((int32_t)32768))) >>
                     10) +
                    ((int32_t)2097152)) *
                   ((int32_t)dig_H2) +
                  8192) >>
                 14));

    v_x1_u32r = (v_x1_u32r -
                 (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                   ((int32_t)dig_H1)) >>
                  4));

    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    return (uint32_t)(v_x1_u32r >> 12);
}


void bme280_init(void) {
	I2C_WriteReg(writeaddr, ctrl_meas, 0xb7);
	I2C_WriteReg(writeaddr, ctrl_hum, 0x05);
	_delay_ms(100);
}

void bme280_readcalibs(void){
	I2C_ReadReg(readaddr ,addr_dig_T1+1, &utempdig);
	dig_T1 = (uint16_t)utempdig << 8;
	I2C_ReadReg(readaddr ,addr_dig_T1, &utempdig);
	dig_T1 |= utempdig;
	
	I2C_ReadReg(readaddr ,addr_dig_T2+1, &utempdig);
	dig_T2 = (int16_t)utempdig << 8;
	I2C_ReadReg(readaddr ,addr_dig_T2, &utempdig);
	dig_T2 |= utempdig;
	
	I2C_ReadReg(readaddr ,addr_dig_T3+1, &utempdig);
	dig_T3 = (int16_t)utempdig << 8;
	I2C_ReadReg(readaddr ,addr_dig_T3, &utempdig);
	dig_T3 |= utempdig;
	
	I2C_ReadReg(readaddr,addr_dig_P1+1, &utempdig);
	dig_P1 = (uint16_t)utempdig << 8;
	I2C_ReadReg(readaddr,addr_dig_P1, &utempdig);
	dig_P1 |= utempdig;
	
	I2C_ReadReg(readaddr,addr_dig_P2+1, &utempdig);
	dig_P2 = (int16_t)utempdig << 8;
	I2C_ReadReg(readaddr,addr_dig_P2, &utempdig);
	dig_P2 |= utempdig;
	
	I2C_ReadReg(readaddr,addr_dig_P3+1, &utempdig);
	dig_P3 = (int16_t)utempdig << 8;
	I2C_ReadReg(readaddr,addr_dig_P3, &utempdig);
	dig_P3 |= utempdig;
	
	I2C_ReadReg(readaddr,addr_dig_P4+1, &utempdig);
	dig_P4 = (int16_t)utempdig << 8;
	I2C_ReadReg(readaddr,addr_dig_P4, &utempdig);
	dig_P4 |= utempdig;
	
	I2C_ReadReg(readaddr,addr_dig_P5+1, &utempdig);
	dig_P5 = (int16_t)utempdig << 8;
	I2C_ReadReg(readaddr,addr_dig_P5, &utempdig);
	dig_P5 |= utempdig;
	
	I2C_ReadReg(readaddr,addr_dig_P6+1, &utempdig);
	dig_P6 = (int16_t)utempdig << 8;
	I2C_ReadReg(readaddr,addr_dig_P6, &utempdig);
	dig_P6 |= utempdig;
	
	I2C_ReadReg(readaddr,addr_dig_P7+1, &utempdig);
	dig_P7 = (int16_t)utempdig << 8;
	I2C_ReadReg(readaddr,addr_dig_P7, &utempdig);
	dig_P7 |= utempdig;
	
	I2C_ReadReg(readaddr,addr_dig_P8+1, &utempdig);
	dig_P8 = (int16_t)utempdig << 8;
	I2C_ReadReg(readaddr,addr_dig_P8, &utempdig);
	dig_P8 |= utempdig;
	
	I2C_ReadReg(readaddr,addr_dig_P9+1, &utempdig);
	dig_P9 = (int16_t)utempdig << 8;
	I2C_ReadReg(readaddr,addr_dig_P9, &utempdig);
	dig_P9 |= utempdig;
	
	I2C_ReadReg(readaddr,addr_dig_H1, &dig_H1);
	
	I2C_ReadReg(readaddr,addr_dig_H2+1, &utempdig);
	dig_H2 = (int16_t)utempdig << 8;
	I2C_ReadReg(readaddr,addr_dig_H2, &utempdig);
	dig_H2 |= utempdig;
	
	I2C_ReadReg(readaddr,addr_dig_H3, &dig_H3);
	
	I2C_ReadReg(readaddr,addr_dig_H4+1, &utempdig);
	dig_H4 = (int16_t)utempdig << 8;
	I2C_ReadReg(readaddr,addr_dig_H4, &utempdig);
	dig_H4 |= utempdig;
	
	I2C_ReadReg(readaddr,addr_dig_H5+1, &utempdig);
	dig_H5 = (int16_t)utempdig << 8;
	I2C_ReadReg(readaddr,addr_dig_H5, &utempdig);
	dig_H5 |= utempdig;
	
	I2C_ReadReg(readaddr,addr_dig_H6, &utempdig);
	dig_H6 = (int8_t) utempdig;
}

void read_pth(volatile int32_t* pressure, volatile int32_t* temperature, volatile int32_t* humidity) {
	uint8_t lsb, xlsb, msb;
	I2C_ReadReg(readaddr ,BME280_TEMP_MSB, &msb);
	I2C_ReadReg(readaddr ,BME280_TEMP_LSB, &lsb);
	I2C_ReadReg(readaddr ,BME280_TEMP_XLSB, &xlsb);
	*temperature = (msb << 12) | (lsb << 4) | (xlsb >> 4);

	I2C_ReadReg(readaddr ,BME280_PRESS_MSB, &msb);
	I2C_ReadReg(readaddr ,BME280_PRESS_LSB, &lsb);
	I2C_ReadReg(readaddr ,BME280_PRESS_XLSB, &xlsb);
	*pressure = (msb << 12) | (lsb << 4) | (xlsb >> 4);

	I2C_ReadReg(readaddr ,BME280_HUM_MSB, &msb);
	I2C_ReadReg(readaddr ,BME280_HUM_LSB, &lsb);
	*humidity = (msb << 8) | (lsb << 0);

}

void RGB_LED_Init(void) {
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    PORTB->PCR[8] = PORT_PCR_MUX(1);      // PTB8 - Red
    PORTB->PCR[9] = PORT_PCR_MUX(1);      // PTB9 - Green
    PORTB->PCR[10] = PORT_PCR_MUX(1);      // PTB10 - Blue

    PTB->PDDR |= (1 << 8) | (1 << 9) | (1 << 10);
    PTB->PSOR |= (1 << 8) | (1 << 9) | (1 << 10);
}


void PIT_Init(void) {
    SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;    
    PIT->MCR = 0;                        

    PIT->CHANNEL[0].LDVAL = 48000000 / 2;  // Przerwanie co 500 ms (dla zegara 48 MHz)
    PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;  

    NVIC_EnableIRQ(PIT_IRQn);
}

void PIT_IRQHandler(void) {
    if (PIT->CHANNEL[0].TFLG & PIT_TFLG_TIF_MASK) {
        PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;
        read_pth(&adc_P, &adc_T, &adc_H);
				press = bme280_press64_compensate(adc_P);
				temp = bme280_temp32_compensate(adc_T);
				hum = bme280_humidity_compensate(adc_H);
        if (temp > 2500) {
					PTB->PCOR |= (1 << 8);  // Wlacz czrwony
					_delay_ms(100);
					PTB->PSOR |= (1 << 8);
        }
				if (press < 960) {
					PTB->PCOR |= (1 << 9);  // Wlacz zielon
					_delay_ms(100);
					PTB->PSOR |= (1 << 9);
        }
				if (hum/1024 > 70) {
					PTB->PCOR |= (1 << 10);  // Wlacz niebieski
					_delay_ms(100);
					PTB->PSOR |= (1 << 10);
        }
				LCD1602_ClearAll();
				LCD1602_SetCursor(0,0);
				sprintf(display,"P:%d  H:%d%%", press, hum/1024);
				LCD1602_Print(display);
				LCD1602_SetCursor(0,1);
				sprintf(display,"T:%d.%d", temp/100, temp%100);
				LCD1602_Print(display);
    }
}

int main(void) {	
	RGB_LED_Init();
	LCD1602_Init();	// Tu jest równiez inicjalizacja portu I2C0
	LCD1602_Backlight(TRUE);
	bme280_init();
	bme280_readcalibs();
	I2C0->F = I2C_F_ICR(0x02) | I2C_F_MULT(0);
	PIT_Init();
	while (1) {
		__WFI();
	}
}
