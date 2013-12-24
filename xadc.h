#include "wirish.h"
#include "adc.h"
#include "dma.h"

#define XADC_BUF_SIZE 2000

//sample rates
#define XADC_SAMPLE_RATE_1000kHz   ADC_SMPR_1_5
#define XADC_SAMPLE_RATE_700kHz    ADC_SMPR_7_5
#define XADC_SAMPLE_RATE_540kHz    ADC_SMPR_13_5
#define XADC_SAMPLE_RATE_340kHz    ADC_SMPR_28_5
#define XADC_SAMPLE_RATE_260kHz    ADC_SMPR_41_5
#define XADC_SAMPLE_RATE_200kHz    ADC_SMPR_55_5
#define XADC_SAMPLE_RATE_160kHz    ADC_SMPR_71_5
#define XADC_SAMPLE_RATE_55kHz     ADC_SMPR_239_5

//8th bit in ADC CR2
#define XADC_ENABLE_DMA  0b100000000
#define XADC_DISABLE_DMA 0b011111111

//DMA defines
#define XADC_DMA_CH  DMA_CH1
#define XADC_DMA_DEV DMA1


class XAdc {
	public:
		static const uint8 Dual=1;
		static const uint8 Single=0;
		static const uint8 Blocking=1;
		static const uint8 NonBlocking=0;
		
		XAdc();
		
		uint8 read();
		void read(uint16 samples);
		
		void setup_single_dma_transfer(uint16 samples);
		void setup_dual_dma_transfer(uint16 samples);
		void setup_fast_dma_transfer(uint16 samples);
		uint8 is_transfer_finished();
		void stop_dma_transfer();
		
		void set_options(uint8 pinA, uint8 pinB, uint8 dual, uint8 blocking, uint16* buffer);
		void set_dual(uint8 dual);
		void set_blocking(uint8 blocking);
		void set_buffer(uint16* buffer);
		void set_pins(uint8 pinA, uint8 pinB);
		void set_pin(uint8 pinA);
		void set_sample_rate(adc_smp_rate smp_rate);
		
		
		uint16* buffer;
		uint8 pin1;
		uint8 pin2;
		uint8 dual;
		uint8 blocking;
		uint8 result;
		uint16 size;
};
