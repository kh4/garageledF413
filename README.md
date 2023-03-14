STM32F413ZH dicovery WS2812 driver

Basics of WS2812 led chip driving

  WS2812 uses single wire pulse width modulated data stream where each bit is 
  represented by either long or short pulse:
    o 1 -> 0.8us high followed by 0.4us low
    o 0 -> 0.4us high followed by 0.8us low

Driving large number of WS2812 chips

  Many library implementation utilize SPI hardware to generate this stream 
  however that is limited to either 1 or 2 signals per MCU, this signal can
  be multiplexed by external circuit but it will make updates slower.
  
  Somewhat novel approach is taken here by sending multiple datastreams
  parallel by outputting the signal on GPIO pins using timer based DMA transfer
  from STM32 CPU. In this implementation 8 bit words are used to drive 8 
  parallel streams, however it could easily be extended to 16.
  
Basic structure

  For sending out data a hardware timer is setup to run with period of about
  0.4us, this is then used to trigger DMA to transfer a single byte (or word)
  from a memory buffer to a GPIO output register.
  
  In this implementation GPIO pins D0 to D7 are being used.
  
  For the encoding described above 3 bytes are needed per bit to create the
  output waveform. Also due to encoding we only need to change the 'middle'
  byte to alter the value since first and last bytes remain same (0xff,-,0x00).
  
DMA buffer:
<code>
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
</code>
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
    
DMA buffer initialization
  For initialization we first set the whole buffer to zero and then add the
  static 'ones' for each bit.
  
<code> 
	bzero(&dmabuffer, sizeof(dmabuffer));
	for (int b = 0; b < 2; b++)
		for (int i = 0; i < LEDSPERLINE; i++)
			for (int j = 0; j < 24; j++)
				dmabuffer[b].data[(i * 24 + j) * 3 + 0]=0xff;
</code>

Updating of the actual RGB values
  To update the RGB values we need to do a transform to set the correct bits
  this is achieved by a following piece of code converting from simple linear
  array of pixels (r1,g1,b1,r2,g2,b2,r3....,r2400,g2400,b2400) directly to the
  DMA output buffer
<code>
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
</code>
  Notably this will only touch the middle byte on each 3 byte sequence, also
  all 8 'channels' are processed parallel.

  After the buffer content is updated a DMA transfer is triggered to send out
  the whole dmabuffer (including the header and trailer) by hardware.

  Notably the 'active' buffer written to is immediately swapped and updating
  can start immediately while transfer is in progress.
  
<code>
  while (1) {
		HAL_DMA_Start(&hdma_tim1_up, (uint32_t)&dmabuffer, (uint32_t)&GPIOD->ODR, DMALEN);
		active_buffer=!active_buffer;
		// work on new data inte leds[]
  	update_leds();
  	// ensure previous transfer is completed
  	HAL_DMA_PollForTransfer(&hdma_tim1_up, HAL_DMA_FULL_TRANSFER, 50000);
  }
</code>

  The transfer for 300 led strings takes a bit under 9ms (300*24*3*0.4us).

Limits of implementation

  The system can in this form drive up to around 900 leds per output (total 7200).
  This is mainly due to DMA transfer count being limited to 16 bit value.
  
  It is possible to extend to double by using 16 outputs instead of 8, at this 
  point the amount of RAM (320kB) on the CPU will also start to be an issue as
  with double buffering almost everything is used up by the two buffers.
