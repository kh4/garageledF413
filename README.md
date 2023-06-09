# STM32F413ZH Nucleo based 8 channel WS2812 driver

## Hardware setup
  - STM32F413ZH Nucleo board
  - strings of WS2812 led strips are directly connected to PD0-PD7 pin
  - each string can contain upto around 900 leds (it is recommended to 
    equally divide leds between outputs for best performance)

## Basics of WS2812 led chip driving

  WS2812 uses single wire pulse width modulated data stream where each bit is 
  represented by either long or short pulse:
  - 1 encoded as 0.8us high followed by 0.4us low
  - 0 encoded as 0.4us high followed by 0.8us low
  
  When receiving the data each ws2812 will grab 24 (3x8) bits to use as RGB 
  values and pass rest forward.
  
  A pause in the stream will latch the data into the chips and allow new frame to
  be started. This pause is typically expected to be around 10us however some 
  devices require more.

## Driving large number of WS2812 chips

  Many library implementation utilize SPI hardware to generate this stream 
  however that is limited to either 1 or 2 signals per MCU, this signal can
  be multiplexed by external circuit but it will make updates slower.
  
  Somewhat novel approach is taken here by sending multiple datastreams
  parallel by outputting the signal on GPIO pins using timer based DMA transfer
  from STM32 CPU. In this implementation 8 bit words are used to drive 8 
  parallel streams, however it could easily be extended to 16.
  
  The downside of the GPIO approach is that it needs quite large buffer for the
  output data, when driving single output using this method the amount of buffer 
  space is 24x the original how ever this is reduced to much more tolerable 3x
  when driving output with all 8 bits.
  
## Basic structure

  For sending out data a hardware timer is setup to run with period of about
  0.4us, this is then used to trigger DMA to transfer a single byte (or word)
  from a memory buffer to a GPIO output register.
  
  In this implementation GPIO pins D0 to D7 are being used.
  
  For the encoding described above 3 bytes are needed per bit to create the
  output waveform. Also due to encoding we only need to change the 'middle'
  byte to alter the value since first and last bytes remain same (0xff,-,0x00).
  
## DMA buffer:
```
  #define LEDSPERLINE 300
  #define NUMLEDS (8*LEDSPERLINE)
  #define DMAHEAD 4
  #define DMATRAIL 140
  #define DMALEN (DMAHEAD + LEDSPERLINE * 3 * 24 + DMATRAIL)
  struct {
    unsigned char header[DMAHEAD];
    unsigned char data[LEDSPERLINE * 3 * 24];
    unsigned char trailer[DMATRAIL];
  } dmabuffer[2];
```
  - the buffer is setup with a short prefix set to zero to ensure smooth 
    startup
  - a trailer is added to ensure that a sufficient 'reset' sequence  is 
    always present at the end of the frame
  - the data part contains the actual payload containing the encoded bytes
  - for above code we have set up to 300 * 8 = 2400 leds
  - for each LED we are sending 24 bit each with 3 bytes making the actual 
    buffer 21600 bytes
  - as for reference the source data is 3*8*300 = 7200 bytes so when driving
    with all 8 lines we only get 'optimal' 3x expansion of data
  - There is two buffers for double buffering the output so we can work on new
    data while HW is sending the other one out
    
## DMA buffer initialization
  For initialization we first set the whole buffer to zero and then add the
  static 'ones' for each bit.
  
``` 
  bzero(&dmabuffer, sizeof(dmabuffer));
  for (int b = 0; b < 2; b++)
    for (int i = 0; i < LEDSPERLINE; i++)
      for (int j = 0; j < 24; j++)
        dmabuffer[b].data[(i * 24 + j) * 3 + 0]=0xff;
```

## Updating of the actual RGB values
  To update the RGB values we need to do a transform to set the correct bits
  this is achieved by a following piece of code converting from simple linear
  array of pixels (r1,g1,b1,r2,g2,b2,r3....,r2400,g2400,b2400) directly to the
  DMA output buffer
```
  for (int i=0; i<LEDSPERLINE; i++)
    for (int j=0; j<3; j++) // RGB
      for (int k=0; k < 8; k++) {
        unsigned char bit = 128>>k;
        int offset = i * 3 +j;
        dmabuffer[active_buffer].data[(i*24 + j * 8 + k) * 3 +1] =
            ((leds[LEDSPERLINE*0*3 + offset] & bit) ? 1 : 0 )+
            ((leds[LEDSPERLINE*1*3 + offset] & bit) ? 2 : 0 )+
            ((leds[LEDSPERLINE*2*3 + offset] & bit) ? 4 : 0 )+
            ((leds[LEDSPERLINE*3*3 + offset] & bit) ? 8 : 0 )+
            ((leds[LEDSPERLINE*4*3 + offset] & bit) ? 16 : 0 )+
            ((leds[LEDSPERLINE*5*3 + offset] & bit) ? 32 : 0 )+
            ((leds[LEDSPERLINE*6*3 + offset] & bit) ? 64 : 0 )+
            ((leds[LEDSPERLINE*7*3 + offset] & bit) ? 128 : 0);
     }
```
  Notably this will only touch the middle byte on each 3 byte sequence, also
  all 8 'channels' are processed parallel.

  After the buffer content is updated a DMA transfer is triggered to send out
  the whole dmabuffer (including the header and trailer) by hardware.

  Notably the 'active' buffer written to is immediately swapped and updating
  can start immediately while transfer is in progress.
  
```
  while (1) {
    HAL_DMA_Start(&hdma_tim1_up, (uint32_t)&dmabuffer, (uint32_t)&GPIOD->ODR, DMALEN);
    active_buffer=!active_buffer;
    // work on new data inte leds[]
    update_leds();
    // ensure previous transfer is completed
    HAL_DMA_PollForTransfer(&hdma_tim1_up, HAL_DMA_FULL_TRANSFER, 50000);
  }
```

  The transfer for 300 led strings takes a bit under 9ms (300*24*3*0.4us).

## Updating leds over USB CDC interface

  Led colors can be updated via USB CDC interface by sending data in following format
  - **0xff 0xff** two byte header to sync up
  - N * **PIXH PIXL** where PIXH<0x80 and PIXH = (pixval >> 8) and PIXL (pixval & x0ff)
  - if PIXH value above is >= 0x80 this is considered EOF
  
  pixval is 15bit color for a pixel (5 bits per component) in form of
  ```
  pixval = (r<<10)|(g<<5)|(b) 
  ```

## Performance considerations (and maximum update rate)

  Below we are considering the default setup of 8*300 leds, having more or less will
  adjust in linear fashion.
  
  To update all the leds 2*2400+3 bytes of data needs to be transferred over the CDC 
  interface, upon testing this seems to takes arong 16ms.
  
  After transferring the data update_leds() is run, which takes somewhere around 10ms.

  As the data is transferred to the leds using DMA and will happen in background it
  can execute fully parallel to the above, update of the 300 leds strings will take 
  about 10ms so this clearly is not the limiting factor. (As a sidenote updating 2400
  leds on a single string would need almost 80ms to complete).
  
  Overall the performance is limited by the STM32 CPU performing the USB transfer
  (interrupts) and data_conversion (main loop). In practice best performance is 
  achieved by sending a full update over USB and then waiting enough (10ms) to allow
  the data conversion to execute freely. With this we can achive update rate of around 
  25ms or 40Hz.

  If data is continously sent over USB the actual frame rate to the leds drops to 
  about half (and every other frame is dropped).
  
  Some optimization may be possible to both USB receive and update_leds code, however
  the performace is more than sufficient for currently planned use cases.

## Limits of implementation

  The system can in this form drive up to around 900 leds per output (total 7200).
  This limit isdue to DMA transfer count being limited to 16 bit value.
  
  It is possible to extend to 14400 by using 16 outputs instead of 8, at this 
  point the amount of RAM (320kB) on the CPU will also start to be an issue as
  with double buffering almost everything is used up by the two buffers.
  (2 * 2 * 900 * 24 * 3 = 259kB dma buffers + 43kB framebuffer)
