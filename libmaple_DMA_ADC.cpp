#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "wirish.h"
#include "dma.h"

// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.
__attribute__((constructor)) void premain()
{
    init();
}

#define DMA_DEV DMA1
#define DMA_CHANNEL DMA_CH1 // DMA1 Channel 1: ADC1, TIM2_CH3, TIM4_CH1
#define BUF_TO_PRINT 8
#define BUF_SIZE 512

uint16_t adcDMABuffer[BUF_SIZE];
uint16_t printPeriod = 100;
uint32_t lastPrinted = 0;
uint32_t currentMillis = 0;
uint32_t startedMillis = 0;

int main()
{
    for (int i = 0; i < BUF_SIZE; i++) adcDMABuffer[i] = 0;

    // Setup DMA
    dma_init(DMA_DEV);
    dma_setup_transfer(DMA_DEV, DMA_CHANNEL,
        &ADC1->regs->DR, DMA_SIZE_16BITS,
        adcDMABuffer, DMA_SIZE_16BITS,
        (DMA_MINC_MODE | DMA_CIRC_MODE | DMA_TRNS_CMPLT)
        );
    dma_set_num_transfers(DMA_DEV, DMA_CHANNEL, BUF_SIZE);
    int isDMAEnabled = 0;

    while (isDMAEnabled == 0) {
        currentMillis = millis();
        // Wait 6 seconds before starting DMA.
        if (currentMillis >= 6000) {
            startedMillis = currentMillis;
            isDMAEnabled = 1;
            // Configure ADC and Enable DMA
            adc_init(ADC1);
            adc_set_prescaler(ADC_PRE_PCLK2_DIV_2);
            adc_set_sample_rate(ADC1, ADC_SMPR_1_5); // Sample rate related
            adc_set_exttrig(ADC1, 1); // Set external trigger conversion mode event for regular channels
            adc_set_reg_seqlen(ADC1, 1); // Set the regular channel sequence length. length: Regular channel sequence length, from 1 to 16.
            ADC1->regs->SQR3 = 1; // Channel 1, 0: pin 11, 1:pin 10, 2:pin 9, 3:pin 8, 4: pin7, 8: pin 3，9: pin 33，参考“Maple Mini — Maple v0.0.12 Documentation”
            // DMA, Enable, SW Start, Cont. Mode, SWSTART Ext. Sel
            ADC1->regs->CR2 |= (ADC_CR2_DMA | ADC_CR2_ADON | ADC_CR2_SWSTART | ADC_CR2_CONT | ADC_CR2_EXTSEL);
            dma_enable(DMA1, DMA_CHANNEL);
            SerialUSB.println("DMA Channel Enabled!");
        }
        else {
            if (currentMillis > (lastPrinted + printPeriod)) {
                lastPrinted = currentMillis;
                SerialUSB.print("[");
                SerialUSB.print(currentMillis);
                SerialUSB.println("] Warming Up...");
            }
        }
    }

    uint8_t previousISRBits = 0;
    uint16_t isrCounts = 0;

    while (1) {
        currentMillis = millis();
        uint8_t isr_bits = dma_get_isr_bits(DMA_DEV, DMA_CHANNEL);

        if (currentMillis > (lastPrinted + printPeriod)) {
            lastPrinted = currentMillis;
            dma_channel_reg_map *ch_regs = dma_channel_regs(DMA_DEV, DMA_CHANNEL);
            SerialUSB.print("[");
            SerialUSB.print(currentMillis - startedMillis);
            SerialUSB.print("] ISR bits: 0x");
            SerialUSB.print((int32)isr_bits, HEX);
            SerialUSB.print(" CCR: 0x");
            SerialUSB.print((int64)ch_regs->CCR, HEX);
            SerialUSB.print(" CNDTR: ");
            SerialUSB.print((int64)ch_regs->CNDTR);
            SerialUSB.print(" ISR Counts: ");
            SerialUSB.print(isrCounts);
            SerialUSB.print(" Buffer contents: ");

            for (int i = 0; i < (BUF_SIZE <= BUF_TO_PRINT ? BUF_SIZE : BUF_TO_PRINT); i++) {
                SerialUSB.print('\'');
                SerialUSB.print(adcDMABuffer[i]);
                SerialUSB.print('\'');
                if (i < BUF_SIZE - 1) SerialUSB.print(", ");
            }

            if (BUF_SIZE > BUF_TO_PRINT) SerialUSB.print("...");
            SerialUSB.println();
        }

        if (isr_bits == 0x7) {
            //SerialUSB.println("** Clearing ISR bits.");
            dma_clear_isr_bits(DMA_DEV, DMA_CHANNEL);
        }

        if (previousISRBits != isr_bits && isr_bits == 0x7) {
            isrCounts++;
        }

        previousISRBits = isr_bits;
    }
    return 0;
}
