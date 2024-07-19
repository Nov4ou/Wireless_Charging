/*
 * main.c
 *
 *  Created on: Mar 21, 2024
 *      Author: nov4ou
 */
#include "F2806x_Device.h"   // F2806x Headerfile
#include "F2806x_Examples.h" // F2806x Examples Headerfile
#include "SPLL_1ph_F.h"
#include "SPLL_1ph_SOGI_F.h"
#include "Solar_F.h"
#include "adc.h"
#include "epwm.h"
#include "timer.h"

#define Kp 50.7089
#define Ki 5000.0
#define ISR_FREQUENCY 10000
#define V_DC_REFERENCE 49.82
#define PI 3.14159265358979
#define GRID_FREQ 50

_Bool flag_inverter = 0;
float V_mod_inverter = 0;
Uint16 compare1, compare2;

SPLL_1ph_SOGI_F spll1;

void LED_Init(void);
void ForceSync();

int main() {
  InitSysCtrl();
  InitPieCtrl();
  IER = 0x0000;
  IFR = 0x0000;
  InitPieVectTable();

  EALLOW;
  PieVectTable.ADCINT1 = &adc_isr;
  EDIS;
  InitAdc(); // For this example, init the ADC
  AdcOffsetSelfCal();
  // ADC
  PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // Enable INT 1.1 in the PIE
  IER |= M_INT1;                     // Enable CPU Interrupt 1
  EINT;
  ERTM;
  PieCtrlRegs.PIEIER8.bit.INTx1 = 1;
  IER |= M_INT8;
  EINT;

  ADC_Init();
  LED_Init();
  // EPWM1_Init(PWM_TBPRD);
  // EPWM2_Init(PWM_TBPRD);
  // EPWM3_Init(PWM_TBPRD);
  // EPWM4_Init(PWM_TBPRD);
  EPWM5_Init(PWM_TBPRD);
  EPWM6_Init(PWM_TBPRD);
  // EPWM7_Init(PWM_TBPRD);
  // EPWM8_Init(PWM_TBPRD);
  ForceSync();

  // SPLL_1ph_SOGI_F_init(GRID_FREQ, ((float)(1.0 / ISR_FREQUENCY)), &spll1);
  // SPLL_1ph_SOGI_F_coeff_update(((float)(1.0 / ISR_FREQUENCY)),
  //                              (float)(2 * PI * GRID_FREQ), &spll1);
  // spll1.lpf_coeff.B0_lf = (float32)((2 * Kp + Ki / ISR_FREQUENCY) / 2);
  // spll1.lpf_coeff.B1_lf = (float32)(-(2 * Kp - Ki / ISR_FREQUENCY) / 2);
  TIM0_Init(90, 101); // 10K

  while (1) {
    if (flag_inverter == 0) {
    }
  }
}

void LED_Init(void) {
  EALLOW;
  GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;
  GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;
  GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;
  GpioDataRegs.GPASET.bit.GPIO0 = 1;
  EDIS;
}

void ForceSync() {
  EPwm1Regs.TBCTL.bit.SWFSYNC = 1; // Master
}

interrupt void TIM0_IRQn(void) {
  // GpioDataRegs.GPATOGGLE.bit.GPIO0 = 1;
  if (flag_inverter == 1) {
  }

  EALLOW;
  PieCtrlRegs.PIEACK.bit.ACK1 = 1;
  EDIS;
}
