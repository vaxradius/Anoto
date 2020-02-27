
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "sbc.h" 

#ifndef gcc
#define CHANNEL_BLOCK_SIZE_IN_SAMPLES   (35*1024)
int16_t i16RAW[CHANNEL_BLOCK_SIZE_IN_SAMPLES];
int16_t i16SPP[CHANNEL_BLOCK_SIZE_IN_SAMPLES];
uint32_t g_sampleIndex = 0;
bool print_flag = 0;
#endif

#define FIFOTHR 		64
#define BUF_SIZE		128

#define SBC_IN_RING_BUFF_SIZE           (BUF_SIZE*2)

//
// If using SBC compression, select audio transfer compression ratio
// 1:1 = 256000 bps, 4:1 = 64000 bps, 8:1 = 32000 bps, 16:1 = 16000 bps
//
#define SBC_BLUEZ_COMPRESS_BPS          64000
#define SBC_OUT_RING_BUFF_SIZE          (SBC_BLUEZ_COMPRESS_BPS / 1000)

#define CODEC_IN_RING_BUFF_SIZE     SBC_IN_RING_BUFF_SIZE
#define CODEC_OUT_RING_BUFF_SIZE    SBC_OUT_RING_BUFF_SIZE

int8_t codecInputBuffer[CODEC_IN_RING_BUFF_SIZE];
uint8_t codecOutputBuffer[CODEC_OUT_RING_BUFF_SIZE];
sbc_t   g_BluezSBCInstance;

volatile bool g_bPDMDataReady = false;
int16_t i16PDMBuf[2][BUF_SIZE] = {{0},{0}};
float g_buffer[BUF_SIZE];
uint32_t u32PDMPingpong = 0;
uint16_t PDMBuf_idx = 0;

void nr_process(uint32_t u32PDMpg)
{
	for (int i=0; i< BUF_SIZE; i++) 
		g_buffer[i] = i16PDMBuf[(u32PDMpg-1)%2][i];
#ifdef gcc	
	relajet_nr_go(g_buffer);
#endif	
	for (int i = 0; i < BUF_SIZE; ++i)
		i16PDMBuf[(u32PDMpg-1)%2][i] = (g_buffer[i]);
}

void SBC_init(void)
{
	sbc_encode_init(&g_BluezSBCInstance, 0);  //0: SBC
	g_BluezSBCInstance.endian = SBC_LE;
}

void SBC_process(uint32_t u32PDMpg)
{
	int32_t CompressedLen;
	sbc_encoder_encode(&g_BluezSBCInstance, i16PDMBuf[(u32PDMpg-1)%2], CODEC_IN_RING_BUFF_SIZE, 
                          	      (void *)codecOutputBuffer, CODEC_OUT_RING_BUFF_SIZE, &CompressedLen);
	memcpy(i16PDMBuf[(u32PDMpg-1)%2],(codecOutputBuffer),CODEC_OUT_RING_BUFF_SIZE);
}

#ifndef gcc
void print_audio_data(void)
{
	uint32_t i = 0;
	uint32_t u32PDMpg;
	if(g_bPDMDataReady == true)
	{
		g_bPDMDataReady = false;
		u32PDMpg = u32PDMPingpong;

		memcpy(i16RAW+g_sampleIndex ,i16PDMBuf[(u32PDMpg-1)%2], BUF_SIZE *2);
		
		am_hal_gpio_out_bit_clear(8);
		for (int i=0; i< BUF_SIZE; i++) 
			g_buffer[i] = i16PDMBuf[(u32PDMpg-1)%2][i];
		
		//relajet_nr_go(g_buffer);
		
		for (int i = 0; i < BUF_SIZE; ++i)
			i16PDMBuf[(u32PDMpg-1)%2][i] = (g_buffer[i]);
		am_hal_gpio_out_bit_set(8);

		memcpy(i16SPP+g_sampleIndex ,i16PDMBuf[(u32PDMpg-1)%2], BUF_SIZE *2);
		g_sampleIndex +=  BUF_SIZE;
	}
	
	if(print_flag ==0)
	{
		if(g_sampleIndex >= CHANNEL_BLOCK_SIZE_IN_SAMPLES)
		{
			am_hal_interrupt_master_disable();
			//short *pdata = (short *)i16SPP;
			for(i = 0; i < CHANNEL_BLOCK_SIZE_IN_SAMPLES; i++)
			{
				am_util_stdio_printf("%d\r\n",*(i16RAW+i));
				am_util_stdio_printf("%d\r\n",*(i16SPP+i));
			}
			print_flag = 1;
		} 
	}

}
#endif
//*****************************************************************************
//
// PDM Interrupt Service Routine (ISR)
//
//*****************************************************************************
void am_pdm0_isr(void)
{
	//uint32_t ui32FIFOCount;
	//uint32_t ui32temp;
	uint16_t idx = 0;
	static uint32_t u32isrType;

	u32isrType = AM_REG(PDM, INTSTAT);

	//
	// Grab the FIFO depth
	//
	//ui32FIFOCount = am_hal_pdm_fifo_depth_read();

	//
	// When Threshold causes ISR, Grab samples from packed FIFO and make sure we don't underflow
	//
	if((u32isrType == AM_HAL_PDM_INT_FIFO))
	{
		//for (idx = 0; idx < ui32FIFOCount; idx ++)
		for (idx = 0; idx < FIFOTHR; idx ++)
		{
			*((uint32_t *)(&(i16PDMBuf[(u32PDMPingpong)%2][PDMBuf_idx])))=am_hal_pdm_fifo_data_read();	
			PDMBuf_idx += 2;
			
		}
		if(PDMBuf_idx >=  BUF_SIZE)
		{
			PDMBuf_idx = 0;
			u32PDMPingpong ++;
			g_bPDMDataReady = true;
			
		}
		//
		// Clear PDM Interrupt (write to clear).
		//
		AM_REG(PDM, INTCLR) = AM_HAL_PDM_INT_FIFO | AM_HAL_PDM_INT_UNDFL | AM_HAL_PDM_INT_OVF;     

	}
	else if(u32isrType == AM_HAL_PDM_INT_UNDFL || AM_HAL_PDM_INT_OVF)
	{
		AM_REG(PDM, FLUSH) = 0x1;
	} 
}


//*****************************************************************************
//
// PDM Configuration
//
//*****************************************************************************
void PDMregConfig(void)
{
	am_hal_pdm_config_t g_sPDM_Cfg =
	{
		//
		// uint32_t ui32PDMConfigReg
		// PDM Configuration (PCFG, PDMCFG) register
		// Notes:
		//  Choose from AM_HAL_PDM_PCFG macros.
		//  For completeness, all PCFG fields should be referenced here.
		//
		(
			AM_HAL_PDM_PCFG_LRSWAP_DISABLE      |
			AM_HAL_PDM_PCFG_RIGHT_PGA_P105DB    |	// +10.5dB gain
			AM_HAL_PDM_PCFG_LEFT_PGA_P105DB     |	// +10.5dB gain
			//AM_HAL_PDM_PCFG_RIGHT_PGA_0DB    |	// +10.5dB gain
			//AM_HAL_PDM_PCFG_LEFT_PGA_0DB     |	// +10.5dB gain
			AM_HAL_PDM_PCFG_MCLKDIV_DIV1        |	
			AM_HAL_PDM_PCFG_SINC_RATE(24)       |   // over sample rate (decimation)
			AM_HAL_PDM_PCFG_HPCUTOFF(0xB)       |
			AM_HAL_PDM_PCFG_SOFTMUTE_DISABLE    |
			AM_HAL_PDM_PCFG_PDMCORE_DISABLE
		),

		//
		// uint32_t ui32VoiceConfigReg
		// PDM Voice Config (VCFG, VOICECFG) register
		// Notes:
		//  Choose from AM_HAL_PDM_VCFG macros.
		//  For completeness, all VCFG fields should be referenced here.
		//  AM_HAL_PDM_IOCLK_xxx also sets AM_REG_PDM_VCFG_IOCLKEN_EN
		//  RSTB is set to NORMAL by am_hal_pdm_enable.
		//
		(
			AM_HAL_PDM_IOCLK_750KHZ             |   // AM_REG_PDM_VCFG_IOCLKEN_EN
			AM_HAL_PDM_VCFG_RSTB_RESET          |
			AM_HAL_PDM_VCFG_PDMCLK_ENABLE       |
			AM_HAL_PDM_VCFG_I2SMODE_DISABLE     |
			AM_HAL_PDM_VCFG_BCLKINV_DISABLE     |
			AM_HAL_PDM_VCFG_DMICDEL_0CYC        |
			AM_HAL_PDM_VCFG_SELAP_INTERNAL      |
			AM_HAL_PDM_VCFG_PACK_ENABLE         |
			//AM_HAL_PDM_VCFG_CHANNEL_STEREO
			AM_HAL_PDM_VCFG_CHANNEL_LEFT
			//AM_HAL_PDM_VCFG_CHANNEL_RIGHT
		),

		//
		// uint32_t ui32FIFOThreshold
		// The PDM controller will generate a processor interrupt when the number
		// of entries in the FIFO goes *above* this number.
		//
		(FIFOTHR - 1)
	};

    am_hal_pdm_config(&g_sPDM_Cfg);		
	
}

void I2Spassthrough(void)
{
	am_hal_pdm_config_t g_sPDM_Cfg =
	{
		//
		// uint32_t ui32PDMConfigReg
		// PDM Configuration (PCFG, PDMCFG) register
		// Notes:
		//  Choose from AM_HAL_PDM_PCFG macros.
		//  For completeness, all PCFG fields should be referenced here.
		//
		(
			AM_HAL_PDM_PCFG_LRSWAP_DISABLE      |
			AM_HAL_PDM_PCFG_RIGHT_PGA_P105DB    |	
			AM_HAL_PDM_PCFG_LEFT_PGA_P105DB     |	
			AM_HAL_PDM_PCFG_MCLKDIV_DIV1        |	
			AM_HAL_PDM_PCFG_SINC_RATE(32)       |   // over sample rate (decimation) 1MHz /32 /2 = 15.625KHz
			AM_HAL_PDM_PCFG_HPCUTOFF(0xB)       |
			AM_HAL_PDM_PCFG_SOFTMUTE_DISABLE    |
			AM_HAL_PDM_PCFG_PDMCORE_DISABLE
		),

		//
		// uint32_t ui32VoiceConfigReg
		// PDM Voice Config (VCFG, VOICECFG) register
		// Notes:
		//  Choose from AM_HAL_PDM_VCFG macros.
		//  For completeness, all VCFG fields should be referenced here.
		//  AM_HAL_PDM_IOCLK_xxx also sets AM_REG_PDM_VCFG_IOCLKEN_EN
		//  RSTB is set to NORMAL by am_hal_pdm_enable.
		//
		(
			AM_HAL_PDM_IOCLK_750KHZ             |   // AM_REG_PDM_VCFG_IOCLKEN_EN
			AM_HAL_PDM_VCFG_RSTB_RESET          |
			AM_HAL_PDM_VCFG_PDMCLK_ENABLE       |
			AM_HAL_PDM_VCFG_I2SMODE_ENABLE|
			AM_HAL_PDM_VCFG_BCLKINV_DISABLE     |
			AM_HAL_PDM_VCFG_DMICDEL_0CYC        |
			AM_HAL_PDM_VCFG_SELAP_I2S|
			AM_HAL_PDM_VCFG_PACK_ENABLE         |
			AM_HAL_PDM_VCFG_CHANNEL_STEREO
		),

		//
		// uint32_t ui32FIFOThreshold
		// The PDM controller will generate a processor interrupt when the number
		// of entries in the FIFO goes *above* this number.
		//
		(FIFOTHR - 1)
	};

    am_hal_pdm_config(&g_sPDM_Cfg);		
	
}


//*****************************************************************************
//
// UART configuration settings.
//
//*****************************************************************************
am_hal_uart_config_t g_sUartConfig =
{
    .ui32BaudRate = 460800,
    .ui32DataBits = AM_HAL_UART_DATA_BITS_8,
    .bTwoStopBits = false,
    .ui32Parity   = AM_HAL_UART_PARITY_NONE,
    .ui32FlowCtrl = AM_HAL_UART_FLOW_CTRL_NONE,
};

//*****************************************************************************
//
// Initialize the UART
//
//*****************************************************************************
void
uart_init(uint32_t ui32Module)
{
    //
    // Make sure the UART RX and TX pins are enabled.
    //
    am_bsp_pin_enable(COM_UART_TX);
    am_bsp_pin_enable(COM_UART_RX);

    //
    // Power on the selected UART
    //
    am_hal_uart_pwrctrl_enable(ui32Module);

    //
    // Start the UART interface, apply the desired configuration settings, and
    // enable the FIFOs.
    //
    am_hal_uart_clock_enable(ui32Module);

    //
    // Disable the UART before configuring it.
    //
    am_hal_uart_disable(ui32Module);

    //
    // Configure the UART.
    //
    am_hal_uart_config(ui32Module, &g_sUartConfig);

    //
    // Enable the UART FIFO.
    //
    am_hal_uart_fifo_config(ui32Module, AM_HAL_UART_TX_FIFO_1_2 | AM_HAL_UART_RX_FIFO_1_2);

    //
    // Enable the UART.
    //
    am_hal_uart_enable(ui32Module);
}

void PDMinit(void)
{	
	//
	// Enable power to PDM module and configure PDM
	//
	am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PDM);	

	//
	// Configure the PDM microphone pins
	//
	am_hal_gpio_pin_config(12, AM_HAL_PIN_12_PDM_CLK); // Configure GP12 as PDM clock pin output
	am_hal_gpio_pin_config(11, AM_HAL_PIN_11_PDM_DATA);  // Configure GP11 as PDM data pin using WDS	
	//am_hal_gpio_pin_config(23, AM_HAL_PIN_23_PDM_DATA);// Configure GP23 as PDM data pin for unmodified Shield
	am_hal_gpio_out_bit_set(8);
	am_hal_gpio_pin_config(8, AM_HAL_PIN_OUTPUT);


	//
	// Configure the PDM module
	//	
	PDMregConfig();

	//
	// Make sure interrupts are clear
	//
	AM_REG(PDM, INTCLR) = 0x7;
	AM_REG(PDM, FLUSH) = 0x1;                       // Reset FIFO pointers

	//
	// Enable interrupts PDM
	//
	am_hal_interrupt_enable(AM_HAL_INTERRUPT_PDM);  //NVIC setting

	am_hal_pdm_int_enable(AM_HAL_PDM_INT_FIFO | AM_HAL_PDM_INT_UNDFL | AM_HAL_PDM_INT_OVF);

	am_hal_interrupt_priority_set(AM_HAL_INTERRUPT_PDM, AM_HAL_INTERRUPT_PRIORITY(4));	

	// enable all interrupts
	//am_hal_interrupt_master_enable();	

	//
	// enable the PDM.
	//
	am_hal_pdm_enable();	

} 


void PDMdeinit(void)
{	

	//
	// Disable the PDM.
	//
	am_hal_pdm_disable();

	//
	// Disable interrupts PDM
	//
	am_hal_pdm_int_disable(AM_HAL_PDM_INT_FIFO | AM_HAL_PDM_INT_UNDFL | AM_HAL_PDM_INT_OVF);
	am_hal_interrupt_disable(AM_HAL_INTERRUPT_PDM);  //NVIC setting

	//
	// Clear PDM Interrupt (write to clear).
	//
	AM_REG(PDM, INTCLR) = AM_HAL_PDM_INT_FIFO | AM_HAL_PDM_INT_UNDFL | AM_HAL_PDM_INT_OVF;

	//
	// Configure the PDM microphone pins as HiZ
	//
	am_hal_gpio_pin_config(12, AM_HAL_PIN_DISABLE); // Configure GP12 as disable
	am_hal_gpio_pin_config(11, AM_HAL_PIN_DISABLE);  // Configure GP11 as disable	

	//
	// Disable power to PDM module 
	//
	am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PDM);	
	

}

void I2Sinit(void)
{
	//
	// Enable power to PDM module and configure PDM
	//
	am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PDM);	

	//
	// Configure the PDM microphone pins
	//
	am_hal_gpio_pin_config(12, AM_HAL_PIN_12_PDM_CLK); // Configure GP12 as PDM clock 
	am_hal_gpio_pin_config(11, AM_HAL_PIN_11_PDM_DATA);  // Configure GP11 as PDM data 
	am_hal_gpio_pin_config(19, AM_HAL_PIN_19_I2S_BCLK | AM_HAL_PIN_DIR_INPUT);
	am_hal_gpio_pin_config(28, AM_HAL_PIN_28_I2S_WCLK | AM_HAL_PIN_DIR_INPUT);
	am_hal_gpio_pin_config(30, AM_HAL_PIN_30_I2S_DAT | AM_HAL_PIN_DIR_INPUT);
	I2Spassthrough();

	//
	// enable the PDM.
	//
	am_hal_pdm_enable();	
}

void I2Sdeinit(void)
{
	//
	// Disable the PDM.
	//
	am_hal_pdm_disable();

	//
	// Configure the PDM & I2S pins as HiZ
	//
	am_hal_gpio_pin_config(12, AM_HAL_PIN_DISABLE); // AM_HAL_PIN_12_PDM_CLK
	am_hal_gpio_pin_config(11, AM_HAL_PIN_DISABLE); //AM_HAL_PIN_11_PDM_DATA
	am_hal_gpio_pin_config(19, AM_HAL_PIN_DISABLE); //AM_HAL_PIN_19_I2S_BCLK
	am_hal_gpio_pin_config(28, AM_HAL_PIN_DISABLE); //AM_HAL_PIN_28_I2S_WCLK
	am_hal_gpio_pin_config(30, AM_HAL_PIN_DISABLE); //AM_HAL_PIN_30_I2S_DAT

	//
	// Disable power to PDM module 
	//
	am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PDM);	
}
