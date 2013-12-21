#include "wirish.h"
#include "xadc.h"

//switch on/off DMA, CONTINUOUS MODE, RIGHT ALIGN
#define XADC_DMA_ON_FUN(dev) dev->regs->CR2 |= XADC_ENABLE_DMA; dev->regs->CR2 |=0x2; dev->regs->CR2 &= (0xfffff7ff)
#define XADC_DMA_OFF_FUN(dev) dev->regs->CR2 |= XADC_DISABLE_DMA; dev->regs->CR2 |=(~0x2); dev->regs->CR2 &= (0xfffff7ff)

//switch SINGLE/DUAL mode for dev
#define XADC_SINGLE_MODE_FUN(dev) dev->regs->CR1 &= 0xfff0ffff
#define XADC_DUAL_MODE_FUN(dev) dev->regs->CR1 &= 0xfff6ffff

//does the above for ADC1 and ADC2
#define XADC_DMA_ON() XADC_DMA_ON_FUN(ADC1);XADC_DMA_ON_FUN(ADC2)
#define XADC_DMA_OFF() XADC_DMA_OFF_FUN(ADC1);XADC_DMA_OFF_FUN(ADC2)

#define XADC_SINGLE_MODE() XADC_SINGLE_MODE_FUN(ADC1);XADC_SINGLE_MODE_FUN(ADC2)
#define XADC_DUAL_MODE() XADC_DUAL_MODE_FUN(ADC1);XADC_DUAL_MODE_FUN(ADC2)


XAdc::XAdc() {
  buffer=NULL;
  size=0;
  dual=Single;
  blocking=Blocking;
  pin1=0;
  pin2=1;
}


//DMA
void XAdc::setup_single_dma_transfer(uint16 samples){
  dma_init(XADC_DMA_DEV);
  dma_setup_transfer(XADC_DMA_DEV, XADC_DMA_CH,
  &ADC1->regs->DR, DMA_SIZE_16BITS,
  buffer,           DMA_SIZE_16BITS,
  (DMA_MINC_MODE | /*DMA_CIRC_MODE |*/ DMA_TRNS_ERR | DMA_TRNS_CMPLT
    ));
  dma_set_num_transfers(XADC_DMA_DEV, XADC_DMA_CH, samples*sizeof(uint16));
  dma_set_priority(XADC_DMA_DEV,XADC_DMA_CH,DMA_PRIORITY_VERY_HIGH);
  dma_enable(XADC_DMA_DEV, XADC_DMA_CH);

  //set ADC for dma transfer
  XADC_SINGLE_MODE();
  XADC_DMA_ON();
}

void XAdc::setup_dual_dma_transfer(uint16 samples){
  dma_init(XADC_DMA_DEV);
  dma_setup_transfer(XADC_DMA_DEV, XADC_DMA_CH,
  &ADC1->regs->DR, DMA_SIZE_32BITS,
  buffer,           DMA_SIZE_32BITS,
  (DMA_MINC_MODE | /*DMA_CIRC_MODE |*/ DMA_TRNS_ERR | DMA_TRNS_CMPLT
    ));
  dma_set_num_transfers(XADC_DMA_DEV, XADC_DMA_CH, samples*sizeof(uint32));
  dma_set_priority(XADC_DMA_DEV,XADC_DMA_CH,DMA_PRIORITY_VERY_HIGH);
  dma_enable(XADC_DMA_DEV, XADC_DMA_CH);

  //set ADC for dma transfer
  XADC_DUAL_MODE();
  XADC_DMA_ON();
}

void XAdc::stop_dma_transfer(){
  XADC_DMA_OFF();
  dma_disable(XADC_DMA_DEV,XADC_DMA_CH);

}

uint8 XAdc::is_transfer_finished() {
  if(dma_get_isr_bits(XADC_DMA_DEV,XADC_DMA_CH)==0x07) {
    result=dma_get_irq_cause(XADC_DMA_DEV,XADC_DMA_CH);//<--clears isr bits
    return 1;
  }
  return 0;
}

//Reading

uint8 XAdc::read(){
  return read(1);
}

uint8 XAdc::read ( uint16 samples ){

  if(dual) setup_dual_dma_transfer(samples);
  else setup_single_dma_transfer(samples);


  adc_set_reg_seqlen(ADC1, 1);
  ADC1->regs->SQR3 = PIN_MAP[pin1].adc_channel;

  if(dual) {
    //set up ADC2
    adc_set_reg_seqlen(ADC2, 1);
    ADC2->regs->SQR3 = PIN_MAP[pin2].adc_channel;
  }
  //start reading. ADC2 is slave, no need to start it
  ADC1->regs->CR2 |= ADC_CR2_SWSTART;

  size=samples;
  if (blocking) {
    while(!is_transfer_finished());
    stop_dma_transfer();
    return (result==DMA_TRANSFER_COMPLETE);
  }

  return 1;
}

//OPTIONS

void XAdc::set_options ( uint8 pinA, uint8 pinB, uint8 dual, uint8 blocking, uint16* buffer ){
  set_pins(pinA,pinB);
  this->dual=dual;
  this->blocking=blocking;
  this->buffer=buffer;
}

void XAdc::set_pin ( uint8 pinA ){
  pin1=pinA;
  pinMode(pin1,INPUT_ANALOG);
}

void XAdc::set_pins( uint8 pinA, uint8 pinB ){
  pin1=pinA;
  pinMode(pin1,INPUT_ANALOG);
  pin2=pinB;
  pinMode(pin2,INPUT_ANALOG);
}

void XAdc::set_blocking ( uint8 blocking ){
  this->blocking=blocking;
}

void XAdc::set_buffer ( uint16* buffer ){
  this->buffer=buffer;
}

void XAdc::set_dual ( uint8 dual ){
  this->dual=dual;
}

void XAdc::set_sample_rate ( adc_smp_rate smp_rate ){
  adc_set_sample_rate(ADC1, smp_rate);
}

uint16 adcbuf[2000];
uint8 linebuf[80];
XAdc xadc;


void dumpline(uint16 value, uint8 p){
  uint8 pos=(p) ? (38):(1);
  for(int j=0;j<35;j++) {
    if(j<(value*36)/4096) linebuf[pos+1+j]= (p) ? ('#'):(' ');
    else linebuf[pos+1+j]=(p) ? (' '):('#');
  }
}


void setup() {
  //Setup XAdc
  xadc.set_options(3,4,XAdc::Dual,XAdc::Blocking,adcbuf);
  xadc.set_sample_rate(XADC_SAMPLE_RATE_55kHz);

  //led
  pinMode(BOARD_LED_PIN, OUTPUT);

  //pin6 on and off
  pinMode(6,OUTPUT);

  //pin 7 always on
  pinMode(7,OUTPUT);
  digitalWrite(7,HIGH);

  //init linebuf
  linebuf[0]=linebuf[38]='[';
  linebuf[37]=linebuf[77]=']';
  linebuf[78]='\r';
  linebuf[79]='\n';
}

void loop(){
  xadc.read(100);
  for(int i=0;i<100;i++) {
    dumpline(adcbuf[i],i%2);
    SerialUSB.write(linebuf,80);
  }
  toggleLED();
  togglePin(6);
  delay(100);
}


// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.
__attribute__((constructor)) void premain() {
  init();
}

int main(void) {
  setup();

  while (1) {
    loop();
  }
  return 0;
}

