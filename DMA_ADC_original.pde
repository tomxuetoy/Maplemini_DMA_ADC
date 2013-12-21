/**
 **********************************************************************************************
 * @file    DualRegSample.pde 
 * @author  Samtal
 * @version V1.1.0
 * @date    27-SEP-2011   (Original 27 AUG 2011)
 * @brief   For Maple r5, IDE 0.0.12 Main program body
 *********************************************************************************
 * PLEASE READ THESE INSTRUCTIONS
 * This program demonstrates the implementation of the STM32F10X Dual-Simultaneous
 * ADC on the Maple r5, using the Maple IDE 0.0.12.
 * Some functions that were missing in the original libMaple adc.h/adc.c where
 * added as indicated and are in the included dadc.h which can be included with 
 * the adc.h or fully replace it.
 * Each such command can be replaced by its adjacent direct registery access 
 * command by un-commenting.
 * The program includes several debugging, training and demonstration options. 
 * 
 * For testing and demo purposes, a digital output channel can be toggled on / off
 * to supply 3.33V that can be divided by one 1 KOhm pot. connected from the output
 * pin (23) to input chan 15 (pin 20) and six 1 KOhm resistors between channels
 * 15-10 (pins (20)-1K-(19)-1K-(18)-1K-(17)-1K-(16)-1K-(15)-1K-(GND). 
 * Adjust the pot to supply 3V to chan 15 (pin 20), which will yield 
 * 0.5V,1V,1.5V,2V,2.5V,3V to the channels.
 * To test voltage existence, connect pin 23 to pin 26 (Dig Input), and the program
 * will show power on / off. 
 * The program is set to load program and read data by SerialUSB. 
 * Although all can be done by SerialUSB , I strongly recommend to use another serial
 * ports for data display (like Serial1), which will enable catch output before the 
 * USB port is returned from programming mode. Change SetrialUSB to Serial1 and initialize the port. 
 ***************************************************************************************************
 * Real-time input parameters:
 * 'r' to display the relevant registers, stop by 'R'.
 * 'v' to display data in real volts, stop by 'V'.
 * 'b' to display the adc raw data in binary to decimal format. Stop by 'B'.
 * 't' to toggle on/off output pin 23 (tied to input pin 27 (an. chan. 8)) and pin 26 (dig in)
 * 
 * Notes: This sample enables 6 X 2 Analog In channels in SQR3 only, out of which any even pair number  
 * can be used (the param 'adc_length' in channel pairs).
 */

#include "stdlib.h"      //Must be included explicitly for proper data printing. 
//#include "dadc.h"      // Optional. Read Instructions !!  USE ONLY IF DADC.H PRESENT !!
#include "dma.h"

uint8 adc_length=4;     //The number of channels to be converted per ADC channel
int16 adc1_data;        //Temporary binary data
int16 adc2_data;        //Temporary binary data
uint32 adc_sequence=0;  //Temporary
boolean dispReg=false;
boolean dispVolts=true;
boolean dispBin=false;
float adc1_Vdata;
float adc2_Vdata;
float voltsConvert=3.350; //Conversion to voltage. Use for fine-tuning calibration.
uint32 rawDataArray[6];  //The dma temporary data array.
uint8 ADC1_Sequence[]={  /*The ADC1 channels sequence, left to right*/
  8,10,12,14,0,0}; /* Set the sequence 1-6 for SQR3 (left will be first). Must top up to all 6 channels with zeros  */
uint8 ADC2_Sequence[]={  /*The ADC2 channels sequence, left to right*/
  9,11,13,15,0,0};
uint8 i;

void setup()

{
  //  Serial1.begin(115200);     //Optional, if Serial1 is used.

  welcome_message();

  /* Set the relevant ADC pins to analog in */
  pinMode(27, INPUT_ANALOG);  //Pins 27,28, ADC channels 8,9
  pinMode(28, INPUT_ANALOG);
  for (int k=15;k<21;k++)    ////Pins 15-20, ADC channels 10-15
  {
    pinMode(k, INPUT_ANALOG);
  }
  /* For testing:*/
  pinMode(23, OUTPUT); //Connect pin 23 via voltage divider to the analog input pins, Set pin 23 as output. 
  togglePin(23); //Initially, toggle pin 23 ON. To change state, send 't'.
  pinMode(26,INPUT); //Connected to output pin 23. Used to test if the output is on or off

  /* 
   From ST Manual: In dual mode, when configuring conversion to be triggered by an external event, 
   the user must set the trigger for the master only and set a software trigger for the slave to 
   prevent spurious triggers to start unwanted slave conversion. However, external triggers must be
   enabled on both master and slave ADCs
   */

  adc_set_extsel(ADC1,ADC_ADC12_SWSTART);   //External trigger Event, ADC1 only!

  /*
   Once the scan bit is set, ADC scans all the channels selected in the ADC_SQRx registers.
   A single conversion is performed for each channel of the group. 
   After each end of conversion the next channel of the group is converted AUTOMATICALLY.
   If the DMA bit is set, the direct memory access controller is used to transfer the converted
   data of regular group channels to SRAM after each EOC.
   */

  /***SELECT BETWEEN THE FOLLOWING OPTIONS*/  //
  ADC1->regs->CR1 |= 1 << 8;  // Set scan mode  
  ADC2->regs->CR1 |= 1 << 8;  // Set scan mode  
  //scan_mode(ADC1,1);      //USE ONLY WITH DADC.H  (not in adc.h)
  //scan_mode(ADC2,1);      //USE ONLY WITH DADC.H  (not in adc.h)

  adc_set_reg_seqlen(ADC1, adc_length);  //The number of channels to be converted. 
  adc_set_reg_seqlen(ADC2, adc_length);
  /*
   * calc_adc_sequence(ADCx_Sequence) converts the SQR3 6 channels' (each ADC1 and ADC2) list into 
   * a valid 6 X 5=30 bits sequence format. Load the sequence onto one of 3 SQR registers.
   * For more channels, repeat the same for SQR2, SQR1. (For SQR1 4 channels only!)
   */
  ADC1->regs->SQR3 |= calc_adc_sequence(ADC1_Sequence);
  ADC2->regs->SQR3 |= calc_adc_sequence(ADC2_Sequence);

  /***SELECT BETWEEN THE FOLLOWING OPTIONS*/  //
  ADC1->regs->CR1 |= 6 << 16;                  //Regular simultaneous mode ADC1 only !!*/
  //set_dual_mode(ADC1,DADC_MODE_6);      // Not in adc files.//USE ONLY WITH DADC.H

  /***SELECT BETWEEN THE FOLLOWING OPTIONS*/  //
  ADC1->regs->CR2 |= 1 << 8; //ADC_CR2_DMA_BIT 8=1, Use DMA for adc. ADC1 only. 
  //adc_dma_enable(ADC1);  //Not in adc files       //USE ONLY WITH DADC.H

  /* Setup of the DMA for the adc data */
  init_dma_xfer();                 
}
void init_dma_xfer(void) 
{
  dma_init(DMA1);      // MUST initiate before ANY other setting!!!!.dma_init: rcc_clk_enable(dev->clk_id)

  dma_setup_transfer
    (
  DMA1,               //dma_device
  DMA_CH1,            //dma_channel 
  &ADC1_BASE->DR,     //*peripheral_address,
  DMA_SIZE_32BITS,    //peripheral_size,
  rawDataArray,       //*memory_address, user defined array.
  DMA_SIZE_32BITS,    //memory_size,
  DMA_MINC_MODE | DMA_CIRC_MODE  //dma mode: Auto-increment memory address,
  // circular mode
  );  

  dma_set_priority(DMA1, DMA_CH1, DMA_PRIORITY_HIGH);    //Optional
  dma_set_num_transfers(DMA1,DMA_CH1,adc_length);
  dma_enable(DMA1,DMA_CH1);                //CCR1 EN bit 0

} //end init_dma_xfer

void loop()
{
  //Shows the test voltage state.
  if (digitalRead(26))
  {
    SerialUSB.println("Output ON");
  }
  else
  {
    SerialUSB.println("Output OFF");
  }

  /* In SCAN mode, 'software_start' activates an automatic scan conversion and and DMA transfer of ONE dual ADC sequence.
   *  to the defined memory array. For repetitive DMA transfers, the DMA Circular mode must be seleted.
   */

  /***SELECT BETWEEN THE FOLLOWING OPTIONS*/  //
  ADC1->regs->CR2 |= 1 << 22;    //software_start(ADC1 only)
  //software_start(ADC1,1); //Not in adc files      //USE ONLY WITH DADC.H

  // /* Check DMA1 transfer complete flag */
  while(!(dma_get_isr_bits(DMA1,DMA_CH1)&1))   // Wait on dma transfer complete on channel (DMA_ISR_TCIF1_BIT)
  { 
    dma_clear_isr_bits(DMA1,DMA_CH1);  //Global Clear DMA1 channel1 transfer flags */

    /***SELECT BETWEEN THE FOLLOWING OPTIONS*/    //
    ADC1->regs->SR |= 1<<1 ;          //Clear the end-of-conversion bit *ADC_SR_EOC_BIT) = 0;
    //end_of_conversion_clear(ADC1);    //Not in adc files. //USE ONLY WITH DADC.H
  }

  for (i= 0; i<adc_length;i++)
  {
    adc1_data = (rawDataArray[i] & 0xFFF);        //ADC1 lower 12 bit out of 16 lower bits in 32 bits data register.
    adc2_data = (rawDataArray[i] >>16 & 0xFFF);   //ADC2 lower 12 bit out of upper 16 bits in 32 bits data register.

    if (dispReg)       //Display the relevant registries
    {
      print_registers();
    }
    if (dispBin)         //Display the raw data binary converted to decimal format
    {
      SerialUSB.print(ADC1_Sequence[i]);
      SerialUSB.print("\t");
      SerialUSB.print(adc1_data);
      SerialUSB.print("\t");
      SerialUSB.print(ADC2_Sequence[i]);
      SerialUSB.print("\t");
      SerialUSB.println(adc2_data);
    }

    if (dispVolts) 
    {
      adc1_Vdata=(float(adc1_data)) / 4095 * voltsConvert;
      adc2_Vdata=((float)(adc2_data)) / 4095 * voltsConvert;

      SerialUSB.print(ADC1_Sequence[i]);
      SerialUSB.print("\t");
      SerialUSB.print((float)adc1_Vdata,3);
      SerialUSB.print(" V\t\t");
      SerialUSB.print(ADC2_Sequence[i]);
      SerialUSB.print("\t");
      SerialUSB.print((float)adc2_Vdata,3);
      SerialUSB.println(" V\t");
    }
  }

  SerialUSB.println();
  SerialUSB.println();
  delay(2000);

  while (SerialUSB.available()) 
  {
    uint8 input = SerialUSB.read();
    SerialUSB.println(input);

    switch(input) 
    {
    case 'r': 
      dispReg=true; 
      break;
    case 'R': 
      dispReg=false; 
      break;
    case 'v': 
      dispVolts=true; 
      break;
    case 'V': 
      dispVolts=false; 
      break;
    case 'b': 
      dispBin=true; 
      break;
    case 'B': 
      dispBin=false; 
      break;
    case 't': 
      togglePin(23); 
      break;   //Toggle test voltage   on / off
    case 'd':
      welcome_message();
    default : 
      SerialUSB.print("Bad input");
      break;
    }
  }

}  //end loop

/*
   * calc_adc_sequence(ADCx_Sequence) converts the SQR3 6 channels' (each ADC1 and ADC2) list into 
 * a valid 6 X 5=30 bits sequence format and returns that 30 bits number. 
 * For more channels, repeat the same for SQR2, SQR1. (For SQR1 4 channels only!)
 */
uint32  calc_adc_sequence(uint8 adc_sequence_array[6])
{
  adc_sequence=0;

  for (int i=0;i<6;i++)     // There are 6 available sequences in each SQR3 SQR2, and 4 in SQR1.
  {
    /*This function converts the array into one number by multiplying each 5-bits channel numbers 
     by multiplications of 2^5
     */
    adc_sequence=adc_sequence + adc_sequence_array[i]*pow(2,(i*5));  
  } 
  return adc_sequence;
}   //end calc_adc_sequence

void print_registers()
{
  SerialUSB.print("ADC1, CR1\t");
  SerialUSB.println(ADC1->regs->CR1,BIN);
  SerialUSB.print("ADC2, CR1\t");
  SerialUSB.println(ADC2->regs->CR1,BIN);
  SerialUSB.print("ADC1, CR2\t");
  SerialUSB.println(ADC1->regs->CR2,BIN);
  SerialUSB.print("ADC2, CR2\t");
  SerialUSB.println(ADC2->regs->CR2,BIN);
  SerialUSB.print("ADC1, SQR3\t");  
  SerialUSB.println(ADC1->regs->SQR3,BIN);
  SerialUSB.print("ADC2, SQR3\t"); 
  SerialUSB.println(ADC2->regs->SQR3,BIN);
  SerialUSB.print("ADC1, SQR1\t");   
  SerialUSB.println(ADC1->regs->SQR1,BIN);
  SerialUSB.print("ADC2, SQR1\t"); 
  SerialUSB.println(ADC2->regs->SQR1,BIN);
  SerialUSB.print("ADC1, SR\t"); 
  SerialUSB.println(ADC1->regs->SR,BIN);
  SerialUSB.print("DMA1, CCR1\t");
  SerialUSB.println(DMA1->regs->CCR1,BIN); 
  SerialUSB.print("DMA1, ISR\t");
  SerialUSB.println(DMA1->regs->ISR,BIN);
  SerialUSB.print("DMA1, CPAR1\t");
  SerialUSB.println(DMA1->regs->CPAR1,BIN);
  SerialUSB.println();
  SerialUSB.println();
} //end print_registers

void welcome_message()
{
  SerialUSB.println("Welcome to the Maple dual simultaneous ADC demo / template by samtal");
  SerialUSB.println(); 
  SerialUSB.println("Real-time input parameters:");
  SerialUSB.println("'r' to display the relevant registers, stop by 'R'.");
  SerialUSB.println("'v' to display data in real volts, stop by 'V'.");
  SerialUSB.println("'b' to display the adc raw data in bin to dec format. Stop by 'B'.");
  SerialUSB.println("'t' to toggle on/off output pin 23 used as voltage input"); 
  SerialUSB.println(" tied to input pin 27 (an. chan. 8) and pin 26 (dig in");
  SerialUSB.println(); 
  SerialUSB.println("Notes: This sample enables 6 X 2 Analog In channels in SQR3 only,");  
  SerialUSB.println("out of which any even pair number can be used (the 'adc_length' in pairs");
  SerialUSB.println();
  SerialUSB.println("Enter d to display this message"); 
  SerialUSB.println();
}
