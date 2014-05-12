// Define bits mask
#include "mach/map.h"
#define NVTBIT(start,end) ((0xFFFFFFFF >> (31 - start)) & (0xFFFFFFFF << end))


#define GCR_BASE	NUC970_VA_GCR		/* GCR */

#define REG_CHIPID	(GCR_BASE+0x00)	// R	Chip Identification Register
	#define CHIP_VER		NVTBIT(27, 24)	// Chip Version
	#define CHIP_ID			NVTBIT(23, 0)	// Chip Identification

#define REG_CHIPCFG	(GCR_BASE+0x04)	// R/W	Chip Power-On Configuration Register
	#define UDFMODE			NVTBIT(27,24)	// User-Defined Power-On setting mode
	#define MAPSDR			BIT16		// Map SDRAM
	#define USBDEV			BIT7 		// USB Host Selection
	#define CLK_SRC			BIT6		// System Clock Source Selection		
	#define SDRAMSEL		NVTBIT(5, 4)	// SDRAM Type Selection
	#define COPMODE			NVTBIT(3, 0)	// Chip Operation Mode Selection

#define REG_AHBCTL	(GCR_BASE+0x10)	// R/W	AHB Bus Arbitration Control Register
	#define IPACT			BIT5 		// Interrupt Active Status
	#define IPEN			BIT4 		// CPU Priority Raising Enable during Interrupt Period
	#define PRTMOD1			BIT1 		// Priority Mode Control 1
	#define PRTMOD0			BIT0 		// Priority Mode Control 0

#define REG_AHBIPRST	(GCR_BASE+0x14)	// R/W	AHB IP Reset Control Resister
	#define JPGRST			BIT17 		// JPG Reset
	#define BLTRST			BIT16		// 2D Accelerator Reset	 
	#define AESRST			BIT15 		// AES Reset
	#define FSCRST			BIT14 		// FSC Reset
	#define GE4PRST			BIT13 		// GE4P Reset
	#define GPURST			BIT12 		// GPU Reset
	#define CAPRST			BIT11 		// CAP Reset
	#define VPOSTRST		BIT10 		// VPOST Reset
	#define I2SRST			BIT9 		// I2S Reset
	#define SPURST			BIT8 		// SPU Reset
	#define UHCRST			BIT7 		// UHC Reset
	#define UDCRST			BIT6 		// UDC Reset
	#define SICRST			BIT5 		// SIC Reset	
	#define TICRST			BIT4 		// TIC Reset
	#define EDMARST			BIT3 		// EDMA Reset
	#define SRAMRST			BIT2 		// SRAM Reset
	#define SDICRST			BIT0 		// SDIC Reset
	
#define REG_APBIPRST	(GCR_BASE+0x18)	// R/W	APB IP Reset Control Resister
	#define ADCRST			BIT14		// ADC Reset		
	#define SPI1RST			BIT13 		// SPI 1 Reset
	#define SPI0RST			BIT12 		// SPI 0 Reset
	#define PWMRST			BIT10 		// PWM Reset
	#define I2CRST			BIT8 		// I2C Reset		
		
	#define UART1RST		BIT7 		// UART 1 Reset
	#define UART0RST		BIT6 		// UART 0 Reset
	#define TMR1RST			BIT5 		// TMR1 Reset
	#define TMR0RST			BIT4 		// TMR0 Reset
	#define WDTRST			BIT3 		// WDT Reset
	#define RTCRST			BIT2 		// RTC Reset
	#define GPIORST			BIT1 		// RTC Reset
	#define AICRST			BIT0 		// AIC Reset
	
#define REG_MISCR	(GCR_BASE+0x20)	// R/W	Miscellaneous Control Register
	#define LVR_RDY			BIT9 		// Low Voltage Reset Function Ready
	#define LVR_EN			BIT8 		// Low Voltage Reset Function Enable
	#define CPURSTON		BIT1 		// CPU always keep in reset state for TIC 
	#define CPURST		 	BIT0 		// CPU one shutte reset.

#define REG_SDRBIST	(GCR_BASE+0x24)	// R/W	Power Management Control Register
	#define TEST_BUSY		BIT31 		// Test BUSY
	#define CON_BUSY		BIT30		// Connection Test Busy
	#define SDRBIST_BUSY		BIT29		// BIST Test Busy
	#define TEST_FAIL		BIT28 		// Test Failed
	#define CON_FAIL		BIT27		// Connection Test Failed
	#define SDRBIST_FAIL		BIT26 		// BIST Test Failed	

#define REG_CRBIST	(GCR_BASE+0x28)	// R/W	Cache RAM BIST Control & Status Register
	#define ICV_F			BIT29		// I-Cache Valid RAM BIST Failed Flag
	#define ICT_F			BIT28		// I-Cache Tag RAM BIST Failed Flag
	#define ICD3_F			BIT27		// I-Cache Data RAM 3 BIST Failed Flag	
	#define ICD2_F			BIT26		// I-Cache Data RAM 2 BIST Failed Flag
	#define ICD1_F			BIT25		// I-Cache Data RAM 1 BIST Failed Flag
	#define ICD0_F			BIT24		// I-Cache Data RAM 0 BIST Failed Flag
	#define MMU_F			BIT23		// MMU RAM BIST Failed Flag
	#define DCDIR_F			BIT22		// D-Cache Dirty RAM BIST Failed Flag
	#define DCV_F			BIT21		// D-Cache Valid RAM BIST Failed Flag	
	#define DCT_F			BIT20		// D-Cache Tag RAM BIST Failed Flag	
	#define DCD3_F			BIT19		// D-Cache Data RAM 3 BIST Failed Flag
	#define DCD2_F			BIT18		// D-Cache Data RAM 2 BIST Failed Flag
	#define DCD1_F			BIT17		// D-Cache Data RAM 1 BIST Failed Flag
	#define DCD0_F			BIT16		// D-Cache Data RAM 0 BIST Failed Flag
	#define BISTEN			BIT15		// Cache RAM BIST Test Enable

	#define ICV_R			BIT13		// I-Cache Valid RAM BIST Running Flag
	#define ICT_R			BIT12		// I-Cache Tag RAM BIST Running Flag
	#define ICD3_R			BIT11		// I-Cache Data RAM 3 BIST Running Flag
	#define ICD2_R			BIT10		// I-Cache Data RAM 2 BIST Running Flag
	#define ICD1_R			BIT9		// I-Cache Data RAM 1 BIST Running Flag
	#define ICD0_R			BIT8		// I-Cache Data RAM 0 BIST Running Flag	
	#define MMU_R			BIT7		// MMU RAM BIST Running Flag
	#define DCDIR_R			BIT6		// D-Cache Dirty RAM BIST Running Flag
	#define DCV_R			BIT5		// D-Cache Valid RAM BIST Running Flag	
	#define DCT_R			BIT4		// D-Cache Tag RAM BIST Running Flag
	#define DCD3_R			BIT3		// D-Cache Data RAM 3 BIST Running Flag
	#define DCD2_R			BIT2		// D-Cache Data RAM 2 BIST Running Flag
	#define DCD1_R			BIT1		// D-Cache Data RAM 1 BIST Running Flag
	#define DCD0_R			BIT0		// D-Cache Data RAM 0 BIST Running Flag	

#define REG_EDSSR	(GCR_BASE+0x2C)	// R/W	EDMA Service Selection Control Register
	#define CH1_RXSEL		NVTBIT(2, 0)	// EDMA Channel 1 Rx Selection
	#define CH2_RXSEL		NVTBIT(6, 4)	// EDMA Channel 2 Rx Selection
	#define CH3_RXSEL		NVTBIT(10, 8)	// EDMA Channel 3 Rx Selection
	#define CH4_RXSEL		NVTBIT(14, 12)	// EDMA Channel 4 Rx Selection
	#define CH1_TXSEL		NVTBIT(18, 16)	// EDMA Channel 1 Tx Selection
	#define CH2_TXSEL		NVTBIT(22, 20)	// EDMA Channel 2 Tx Selection
	#define CH3_TXSEL		NVTBIT(26, 24)	// EDMA Channel 3 Tx Selection
	#define CH4_TXSEL		NVTBIT(30, 28)	// EDMA Channel 4 Tx Selection
	
#define REG_MISSR	(GCR_BASE+0x30)	// R/W	Miscellaneous Status Register
	#define KPI_WS			BIT31		// KPI Wake-Up Status
	#define ADC_WS			BIT30		// ADC Wake-Up Status
	#define UHC_WS			BIT29		// UHC Wake-Up Status
	#define UDC_WS			BIT28		// UDC Wake-Up Status
	#define UART_WS			BIT27		// UART Wake-Up Status
	#define SDH_WS			BIT26		// SDH Wake-Up Status
	#define RTC_WS			BIT25		// RTC Wake-Up Status
	#define GPIO_WS			BIT24		// GPIO Wake-Up Status
	#define KPI_WE			BIT23		// KPI Wake-Up Enable
	#define ADC_WE			BIT22		// ADC Wake-Up Enable
	#define UHC_WE			BIT21		// UHC Wake-Up Enable
	#define UDC_WE			BIT20		// UDC Wake-Up Enable
	#define UART_WE			BIT19		// UART Wake-Up Enable
	#define SDH_WE			BIT18		// SDH Wake-Up Enable
	#define RTC_WE			BIT17		// RTC Wake-Up Enable
	#define GPIO_WE			BIT16		// GPIO Wake-Up Enable
	#define CPU_RST			BIT4		// CPU Reset Active Status
	#define WDT_RST			BIT3		// WDT Reset Active Status
	#define KPI_RST			BIT2		// KPI Reset Active Status
	#define LVR_RST			BIT1		// LVR Reset Active Status
	#define EXT_RST			BIT0		// External Reset Pin Active Status
	
#define REG_OTP_CTRL	(GCR_BASE+0x40)	// R/W	OTP Control Register
	#define OTP_STAT		NVTBIT(25, 24)	// OTP Burned Status
	#define IBR4_STAT		NVTBIT(23, 22)	// OTP_IBR4 Burned Status
	#define IBR3_STAT		NVTBIT(21, 20)	// OTP_IBR3 Burned Status
	#define IBR2_STAT		NVTBIT(19, 18)	// OTP_IBR2 Burned Status
	#define IBR1_STAT		NVTBIT(17, 16)	// OTP_IBR1 Burned Status			
	#define TEST_OK			BIT4		// MARGIN Read Mode Test OK Flag
	#define MARGIN			BIT1		// OTP MARGIN Read Mode
	#define OTPRD_EN		BIT0		// OTP Read Enable
	
#define REG_OTP_PROG	(GCR_BASE+0x44)	// R/W	OTP Program Control Register
	#define BURN_CYC		NVTBIT(29, 16)	// OTP Program Cycle	
	#define OTP_EN			NVTBIT(12, 4)	// OTP Enable		
	#define VPP_STA			BIT1		// VPP State Indicator
	#define BURN_EN			BIT0		// OTP Program Enable
	
#define REG_OTP_DIS	(GCR_BASE+0x48)	// R/W	OTP Disable Register
	#define CNTRL_DIS		BIT16		// OTP Register Control Disable

#define REG_OTP_KEY1	(GCR_BASE+0x50)	// R/W	OTP Key 1 Register

#define REG_OTP_KEY2	(GCR_BASE+0x54)	// R/W	OTP Key 2 Register	

#define REG_OTP_KEY3	(GCR_BASE+0x58)	// R/W	OTP Key 2 Register	

#define REG_OTP_KEY4	(GCR_BASE+0x5C)	// R/W	OTP Key 2 Register	

#define REG_OTP_IBR1	(GCR_BASE+0x60)	// R/W	OTP IBR Option 1 Register

#define REG_OTP_IBR2	(GCR_BASE+0x64)	// R/W	OTP IBR Option 2 Register

#define REG_OTP_IBR3	(GCR_BASE+0x68)	// R/W	OTP IBR Option 3 Register

#define REG_OTP_IBR4	(GCR_BASE+0x6C)	// R/W	OTP IBR Option 4 Register

#define REG_OTP_CID	(GCR_BASE+0x70)	// R/W	OTP IBR Option 4 Register
	#define UDOption		NVTBIT(31, 8)	// User Defined Option
	#define OTP_CID_CHIP_VER	NVTBIT(7,  4)	// Chip version
	#define CHIP_COD		NVTBIT(29, 28)	// Chip mode
				
#define REG_GPAFUN	(GCR_BASE+0x80)	// R/W	Multi Function Pin Control Register 0
	#define MF_GPA15		NVTBIT(31, 30)	// GPA[15] Multi Function
	#define MF_GPA14		NVTBIT(29, 28)	// GPA[14] Multi Function
	#define MF_GPA13		NVTBIT(27, 26)	// GPA[13] Multi Function	
	#define MF_GPA12		NVTBIT(25, 24)	// GPA[12] Multi Function	
	#define MF_GPA11		NVTBIT(23, 22)	// GPA[11] Multi Function
	#define MF_GPA10		NVTBIT(21, 20)	// GPA[10] Multi Function
	#define MF_GPA9			NVTBIT(19, 18)	// GPA[9] Multi Function
	#define MF_GPA8			NVTBIT(17, 16)	// GPA[8] Multi Function
	#define MF_GPA7			NVTBIT(15, 14)	// GPA[7] Multi Function 
	#define MF_GPA6			NVTBIT(13, 12)	// GPA[6] Multi Function
	#define MF_GPA5			NVTBIT(11, 10)	// GPA[5] Multi Function
	#define MF_GPA4			NVTBIT(9, 8)	// GPA[4] Multi Function
	#define MF_GPA3			NVTBIT(7, 6)	// GPA[3] Multi Function
	#define MF_GPA2			NVTBIT(5, 4)	// GPA[2] Multi Function
	#define MF_GPA1			NVTBIT(3, 2)	// GPA[1] Multi Function
	#define MF_GPA0			NVTBIT(1, 0)	// GPA[0] Multi Function

#define REG_GPBFUN	(GCR_BASE+0x84)	// R/W	Multi Function Pin Control Register 0
	#define MF_GPB15		NVTBIT(31, 30)	// GPB[15] Multi Function
	#define MF_GPB14		NVTBIT(29, 28)	// GPB[14] Multi Function
	#define MF_GPB13		NVTBIT(27, 26)	// GPB[13] Multi Function	
	#define MF_GPB12		NVTBIT(25, 24)	// GPB[12] Multi Function	
	#define MF_GPB11		NVTBIT(23, 22)	// GPB[11] Multi Function
	#define MF_GPB10		NVTBIT(21, 20)	// GPB[10] Multi Function
	#define MF_GPB9			NVTBIT(19, 18)	// GPB[9] Multi Function
	#define MF_GPB8			NVTBIT(17, 16)	// GPB[8] Multi Function
	#define MF_GPB7			NVTBIT(15, 14)	// GPB[7] Multi Function 
	#define MF_GPB6			NVTBIT(13, 12)	// GPB[6] Multi Function
	#define MF_GPB5			NVTBIT(11, 10)	// GPB[5] Multi Function
	#define MF_GPB4			NVTBIT(9, 8)	// GPB[4] Multi Function
	#define MF_GPB3			NVTBIT(7, 6)	// GPB[3] Multi Function
	#define MF_GPB2			NVTBIT(5, 4)	// GPB[2] Multi Function
	#define MF_GPB1			NVTBIT(3, 2)	// GPB[1] Multi Function
	#define MF_GPB0			NVTBIT(1, 0)	// GPB[0] Multi Function

#define REG_GPCFUN	(GCR_BASE+0x88)	// R/W	Multi Function Pin Control Register 0
	#define MF_GPC15		NVTBIT(31, 30)	// GPC[15] Multi Function
	#define MF_GPC14		NVTBIT(29, 28)	// GPC[14] Multi Function
	#define MF_GPC13		NVTBIT(27, 26)	// GPC[13] Multi Function	
	#define MF_GPC12		NVTBIT(25, 24)	// GPC[12] Multi Function	
	#define MF_GPC11		NVTBIT(23, 22)	// GPC[11] Multi Function
	#define MF_GPC10		NVTBIT(21, 20)	// GPC[10] Multi Function
	#define MF_GPC9			NVTBIT(19, 18)	// GPC[9] Multi Function
	#define MF_GPC8			NVTBIT(17, 16)	// GPC[8] Multi Function
	#define MF_GPC7			NVTBIT(15, 14)	// GPC[7] Multi Function 
	#define MF_GPC6			NVTBIT(13, 12)	// GPC[6] Multi Function
	#define MF_GPC5			NVTBIT(11, 10)	// GPC[5] Multi Function
	#define MF_GPC4			NVTBIT(9, 8)	// GPC[4] Multi Function
	#define MF_GPC3			NVTBIT(7, 6)	// GPC[3] Multi Function
	#define MF_GPC2			NVTBIT(5, 4)	// GPC[2] Multi Function
	#define MF_GPC1			NVTBIT(3, 2)	// GPC[1] Multi Function
	#define MF_GPC0			NVTBIT(1, 0)	// GPC[0] Multi Function
	
#define REG_GPDFUN	(GCR_BASE+0x8C)	// R/W	Multi Function Pin Control Register 0
	#define MF_GPD15		NVTBIT(31, 30)	// GPD[15] Multi Function
	#define MF_GPD14		NVTBIT(29, 28)	// GPD[14] Multi Function
	#define MF_GPD13		NVTBIT(27, 26)	// GPD[13] Multi Function	
	#define MF_GPD12		NVTBIT(25, 24)	// GPD[12] Multi Function	
	#define MF_GPD11		NVTBIT(23, 22)	// GPD[11] Multi Function
	#define MF_GPD10		NVTBIT(21, 20)	// GPD[10] Multi Function
	#define MF_GPD9			NVTBIT(19, 18)	// GPD[9] Multi Function
	#define MF_GPD8			NVTBIT(17, 16)	// GPD[8] Multi Function
	#define MF_GPD7			NVTBIT(15, 14)	// GPD[7] Multi Function 
	#define MF_GPD6			NVTBIT(13, 12)	// GPD[6] Multi Function
	#define MF_GPD5			NVTBIT(11, 10)	// GPD[5] Multi Function
	#define MF_GPD4			NVTBIT(9, 8)	// GPD[4] Multi Function
	#define MF_GPD3			NVTBIT(7, 6)	// GPD[3] Multi Function
	#define MF_GPD2			NVTBIT(5, 4)	// GPD[2] Multi Function
	#define MF_GPD1			NVTBIT(3, 2)	// GPD[1] Multi Function
	#define MF_GPD0			NVTBIT(1, 0)	// GPD[0] Multi Function

#define REG_GPEFUN	(GCR_BASE+0x90)	// R/W	Multi Function Pin Control Register 0
	#define MF_GPE15		NVTBIT(31, 30)	// GPE[15] Multi Function
	#define MF_GPE14		NVTBIT(29, 28)	// GPE[14] Multi Function
	#define MF_GPE13		NVTBIT(27, 26)	// GPE[13] Multi Function	
	#define MF_GPE12		NVTBIT(25, 24)	// GPE[12] Multi Function	
	#define MF_GPE11		NVTBIT(23, 22)	// GPE[11] Multi Function
	#define MF_GPE10		NVTBIT(21, 20)	// GPE[10] Multi Function
	#define MF_GPE9			NVTBIT(19, 18)	// GPE[9] Multi Function
	#define MF_GPE8			NVTBIT(17, 16)	// GPE[8] Multi Function
	#define MF_GPE7			NVTBIT(15, 14)	// GPE[7] Multi Function 
	#define MF_GPE6			NVTBIT(13, 12)	// GPE[6] Multi Function
	#define MF_GPE5			NVTBIT(11, 10)	// GPE[5] Multi Function
	#define MF_GPE4			NVTBIT(9, 8)	// GPE[4] Multi Function
	#define MF_GPE3			NVTBIT(7, 6)	// GPE[3] Multi Function
	#define MF_GPE2			NVTBIT(5, 4)	// GPE[2] Multi Function
	#define MF_GPE1			NVTBIT(3, 2)	// GPE[1] Multi Function
	#define MF_GPE0			NVTBIT(1, 0)	// GPE[0] Multi Function

#define REG_MISFUN	(GCR_BASE+0x94)	// R/W	Miscellaneous Multi Function Control Register
	#define MF_NCS0			NVTBIT(5, 4)	// MF_NCS0_ Multi Function
	#define MF_EWAIT		NVTBIT(3, 2)	// MF_EWAIT_ Multi Function		
	#define MF_ECS1			NVTBIT(1, 0)	// MF_ECS1_ Multi Function		
	
#define REG_MISCPCR	(GCR_BASE+0xA0)	// R/W	Miscellaneous Pin Control Register
	#define SL_MD			BIT7		// MD Pin Slew Rate Control
	#define SL_MA			BIT6		// MA Pin Slew Rate Control	
	#define SL_MCTL			BIT5		// Memory I/F Control Pin Slew Rate Control		
	#define SL_MCLK			BIT4		// MCLK Pin Rate Control
	#define DS_MD			BIT3		// MD Pins Driving Strength Control	
	#define DS_MA			BIT2		// MA Pins Driving Strength Control	
	#define DS_MCTL			BIT1		// MCTL Pins Driving Strength Control	
	#define DS_MCLK			BIT0		// MCLK Pins Driving Strength Control			

#define REG_PWRCON	(GCR_BASE+0x200)
	#define PRE_SCALAR		NVTBIT(23, 8)		// Pre-Scalar counter
	#define UP2HCLK3X		BIT5				// Ratio of CPU to HCLK
	#define SEN_OFF_ST		BIT4				// Sensor clock level if clock off state
	#define INT_EN			BIT3				// Power On Interrupt Enable
	#define INTSTS			BIT2				// Power Down interrupt status
	#define XIN_CTL			BIT1				// Crystal pre-divide control for Wake-up from power down mode
	#define XTAL_EN			BIT0				// Crystal (Power Down) Control
#define REG_REGWRPRTR	(GCR_BASE+0x1FC)
#define REG_AHBCLK	(GCR_BASE+0x210)
	#define ADO_CKE			BIT30					// Audio DAC Engine Clock Enable Control0 = Disable1 = Enable
	#define SEN_CKE			BIT29					// Sensor Interface Clock Enable Control0 = Disable1 = Enable
	#define CAP_CKE			BIT28					// Capture Clock Enable Control (Also is Capture engine clock enable control)0 = Disable1 = Enable
	#define SENSOR			BIT27					// VPOST Clock Enable Control (Also is VPOST engine clock enable control)0 = Disable1 = Enable
	#define VCAP			BIT26					// I2S Controller Clock Enable Control0 = Disable1 = Enable
	#define SPU_CKE			BIT25					// SPU Clock Enable Control0 = Disable1 = Enable
	#define HCLK4_CKE		BIT24					// HCLK4 Clock Enable Control0 = Disable1 = Enable
	#define SD_CKE			BIT23					// SD Card Controller Engine Clock Enable Control0 = Disable1 = ENable
	#define NAND_CKE		BIT22					// NAND Controller Clock Enable Control0 = Disable1 = ENable
	#define SIC_CKE			BIT21					// SIC Clock Enable Control0 = Disable1 = ENable
	#define GPU_CKE			BIT20					// Graphic Processing Unit Clock Enable Control0 = Disable1 = ENable
	#define GE4P_CKE		BIT19					// GE4P Clock Enable Control0 = Disable1 = ENable
	#define USBD_CKE		BIT18					// USB Device Clock Enable Control0 = Disable1 = Enable
	#define USBH_CKE		BIT17					// USB Host Controller Clock Enable Control0 = Disable1 = Enable
	#define HCLK3_CKE		BIT16					// HCLK3 Clock Enable Control.0 = Disable1 = Enable
	#define DES_CKE			BIT15					// DES Codec Clock Enable Control
	#define EDMA4_CKE		BIT14					// EDMA Controller Channel 4 Clock Enable Control
	#define EDMA3_CKE		BIT13					// EDMA Controller Channel 4 Clock Enable Control
	#define EDMA2_CKE		BIT12					// EDMA Controller Channel 4 Clock Enable Control
	#define EDMA1_CKE		BIT11					// EDMA Controller Channel 4 Clock Enable Control
	#define EDMA0_CKE		BIT10					// EDMA Controller Channel 4 Clock Enable Control			
	#define EBI_CKE		 	BIT9					// EBI	Clock Enable Control0 = Disable1 = Enable
	#define HCLK1_CKE		BIT8					// HCLK1 Clock Enable Control.0 = Disable1 = Enable
	#define JPG_CKE			BIT7					// JPEG Clock Enable	
	#define FSC_CKE			BIT6					// FSC Clock Enable
	#define BLT_CKE		    BIT5					// GE2D Clock Enable Control0 = Disable1 = Enable
	#define DRAM_CKE		BIT4					// SDRAM and SDRAM Controller Clock Enable Control.0 = Disable1 = Enable
	#define SRAM_CKE		BIT3					// SRAM Controller Clock Enable Control.0 = Disable1 = Enable
	#define HCLK_CKE		BIT2					// HCLK Clock Enable Control. (This clock is used for DRAM controller, SRAM controller and AHB-to-AHB bridge)0 = Disable1 = Enable
	#define APBCLK_CKE		BIT1					// APB Clock Enable Control.0 = Disable1 = Enable
	#define CPU_CKE	 		BIT0		

#define REG_APBCLK	(GCR_BASE+0x208)
	#define KPI_CKE			BIT25 					// KPI Clock Enable Control
	#define	TIC_CKE			BIT24 					// TIC Clock Enable
	#define WDCLK_CKE		BIT15					// Watch Dog Clock Enable Control (Also is Watch Dog engine clock enable control)
	#define TMR1_CKE		BIT9					// Timer1 Clock Enable Control0 = Disable1 = Enable
	#define TMR0_CKE		BIT8					// Timer0 Clock Enable Control0 = Disable1 = Enable
	#define SPIMS1_CKE		BIT7					// SPIM (Master Only) Clock Enable Control0 = Disable1 = Enable
	#define SPIMS0_CKE		BIT6					// SPIMS (Master / Slave) Clock Enable Control0 = Disable1 = Enable
	#define PWM_CKE			BIT5					// PWM Clock Enable Control0 = Disable1 = Enable
	#define UART1_CKE		BIT4					// UART1 Clock Enable Control0 = Disable1 = Enable
	#define UART0_CKE		BIT3					// UART0 Clock Enable Control0 = Disable1 = Enable		
	#define RTC_CKE			BIT2					// RTC Clock Enable Control (NOT X32K clock enable control)0 = Disable1 = Enable
	#define I2C_CKE			BIT1					// I2C Clock Enable Control0 = Disable1 = Enable
	#define ADC_CKE			BIT0					// ADC Clock Enable Control (Also is ADC engine clock enable control)0 = Disable1 = Enable

#define REG_CLKDIV0	(GCR_BASE+0x20C)
	#define SENSOR_N1		NVTBIT(27, 24)			// Sensor clock divide number from sensor clock source
	#define KPI_N1			NVTBIT(23, 21)			// KPI Engine Clock Divider Bits [6:4]
	//#define SENSOR_S		NVTBIT(20, 19)			// Sensor clock source select 00 = XIN. 01 = X32K. 10 = APLL. 11 = UPLL
	#define SENSOR_N0		NVTBIT(18, 16)			// Sensor clock pre-divider number from Sensor clock source if Sensor clock source select is APLL or UPLL
	#define KPI_N0			NVTBIT(15, 12)			// KPI Engine Clock Divider Bits [3:0]
	
	#define SYSTEM_N1		NVTBIT(11, 8)			// SYSTEM clock divide number from system clock source
	#define KPI_S			BIT5						// KPI Engine Clock Source Selection
	#define SYSTEM_S		NVTBIT(4, 3)			// System clock source select 00 = XIN. 01 = X32K. 10 = APLL. 11 = UPLL
	#define SYSTEM_N0		NVTBIT(2, 0)			// SYSTEM clock pre-divider number from system clock source if System clock source select is APLL or UPLL

#define REG_CLKDIV1	(GCR_BASE+0x210)
	#define ADO_N1			NVTBIT(31, 24)			// Audio DAC engine clock divide number from Audio DAC engine clock source
	#define ADO_S			NVTBIT(20, 19)			// Audio DAC engine clock source select 00 = XIN. 01 = X32K. 10 = APLL. 11 = UPLL
	#define ADO_N0			NVTBIT(18, 16)			// Audio DAC engine clock pre-divide number
	#define VPOST_N1		NVTBIT(15, 8)			// VPOST engine clock divide number from VPOST engine clock source
	#define VPOST_S			NVTBIT(4, 3)			// VPOST engine clock source select 00 = XIN. 01 = X32K. 10 = APLL. 11 = UPLL
	#define VPOST_N0		NVTBIT(2, 0)			// VPOST engine clock pre-divide number

#define REG_CLKDIV2	(GCR_BASE+0x214)
	#define SD_N1			NVTBIT(31, 24)			// SD engine clock divide number from SD engine clock source
	#define SD_S			NVTBIT(20, 19)			// SD engine clock source select  00 = XIN. 01 = X32K. 10 = APLL. 11 = UPLL
	#define SD_N0			NVTBIT(18, 16)			// SD engine clock pre-divide number
	#define USB_N1			NVTBIT(11, 8)			// USB engine clock divide number from USB engine clock source
	#define USB_S			NVTBIT(4, 3)			// USB engine clock source select 00 = XIN. 01 = X32K. 10 = APLL. 11 = UPLL
	#define USB_N0			NVTBIT(2, 0)			// USB engine clock Pre-divide number	

#define REG_CLKDIV3	(GCR_BASE+0x22C)
	#define JPG_N			NVTBIT(30, 28) 			// UART1 engine clock source select 00 = XIN. 01 = X32K. 10 = APLL. 11 = UPLL
	#define SENSOR_N		NVTBIT(27, 24)  			// UART1 engine clock pre-divide number from UART1 engine clock source
	#define SENSOR_S		NVTBIT(20, 16) 			// UART0 engine clock divide number from UART1 engine clock source
	#define eMMC_N			NVTBIT(15, 8) 			// UART0 engine clock source select 00 = XIN. 01 = X32K. 10 = APLL. 11 = UPLL
	#define eMMC_S			NVTBIT(4, 0)   				// UART0 engine clock pre-divide number from UART1 engine clock source

#define REG_CLKDIV4	(GCR_BASE+0x21C)
	//#define JPG_N			NVTBIT(26, 24)			// JPG 	engine clock divide number from HCLK3
	#define GPIO_N			NVTBIT(23, 17)			// GPIO engine clock divide number from GPIO engine clock source
	#define GPIO_S			BIT16 					// GPIO engine clock source select0 = XIN1 = X32K
	#define CAP_N			NVTBIT(14, 12)			// Capture engine clock divide number from HCLK4 clock.Engine Clock frequency = HCLK4 / (CAP_N + 1)
	#define APB_N			NVTBIT(11, 8) 			// APB clock divide number from HCLK1 clock. The HCLK1 clock frequency is the lower of system clock divided by 2 or the CPU clockThe APB clock frequency = (HCLK1 frequency) / (APB_N + 1)
	#define HCLK234_N		NVTBIT(7, 	04)			// HCLK2, HCLK3 and HCLK4 clock divide number from HCLK clock. The HCLK clock frequency is the system clock frequency divided by two.The HCLK2,3,4 clock frequency = (HCLK frequency) / (HCLK234_N + 1)
	#define CPU_N			NVTBIT(3, 	0)	 		// CPU clock divide number from System clock.The CPU clock frequency = (System frequency) / (CPU_N + 1)

#define REG_APLLCON	(GCR_BASE+0x220)
#define REG_UPLLCON	(GCR_BASE+0x224)
	#define OE				BIT18					// PLL OE (FOUT enable) pin Control
	#define BP				BIT17					// PLL Bypass Control
	#define PD				BIT16					// Power Down Mode	
	#define OUT_DV			NVTBIT(15,14)			// PLL Output Divider Control Pins (PLL_OD[1:0])
	#define IN_DV			NVTBIT(13,9)			// PLL Input Divider Control Pins (PLL_R[4:0])
	#define FB_DV			NVTBIT(8,0)				// PLL Feedback Divider Control Pins (PLL_F[6:0])	

#define REG_CLK_TREG	(GCR_BASE+0x230)








#define LCM_BA		NUC970_VA_LCD	/* Display, LCM Interface */

/*
 VPOST Control Registers
*/
#define REG_LCM_LCDCCtl  	 		(LCM_BA+0x00)  	// R/W: LCD Controller Control Register
	#define LCDCCtl_FSADDR_SEL		BIT31
	#define LCDCCtl_HAW_656			BIT30
	#define LCDCCtl_PRDB_SEL		NVTBIT(21,20)
	#define LCDCCtl_YUVBL			BIT16				// YUV big endian(0) or little endian(1)
	#define LCDCCtl_FBDS 			NVTBIT(3,1)			// Frame Buffer Data Selection
	#define LCDCCtl_LCDRUN 			BIT0				// LCD Controller Run.

#define REG_LCM_LCDCPrm   		(LCM_BA+0x04)		// R/W: LCD Controller Parameter Register
	#define LCDCPrm_Even_Field_AL	NVTBIT(31,28)
	#define LCDCPrm_Odd_Field_AL	NVTBIT(27,24)
	#define LCDCPrm_F1_EL			NVTBIT(23,15)
	#define LCDCPrm_LCDSynTv		BIT8				// LCD timming Synch with TV
	#define LCDCPrm_SRGB_EL_SEL		NVTBIT(7,6)
	#define LCDCPrm_SRGB_OL_SEL		NVTBIT(5,4)
	#define LCDCPrm_LCDDataSel		NVTBIT(3,2)			// LCD data interface Select
	#define LCDCPrm_LCDTYPE	  		NVTBIT(1,0)			// LCD device Type Select.

#define REG_LCM_LCDCInt 		(LCM_BA+0x08)		// R/W: LCD Controller Interrupt Register
	#define LCDCInt_MPUCPL_INTEN	BIT20 				// MPU Frame Complete Enable
	#define LCDCInt_TVFIELDINTEN	BIT18				// TV Even/Odd Field Interrupt Enable.		
	#define LCDCInt_VINTEN			BIT17				// LCD VSYNC Interrupt Enable.
	#define LCDCInt_HINTEN			BIT16				// LCD HSYNC Interrupt Enable.
	#define LCDCInt_MPUCPL			BIT4				// MPU Frame Complete
	#define LCDCInt_TVFIELDINT		BIT2				// TV Odd/Even Field Interrupt.	
	#define LCDCInt_VINT			BIT1				// LCD VSYNC/RD End Interrupt.	
	#define LCDCInt_HINT			BIT0				// LCD HSYNC/WR End Interrupt.


	#define LCDCInt_VINT			BIT1				// LCD VSYNC/RD End Interrupt.
	

#define REG_FEADDR 			(LCM_BA+0x0c)		// Reserved

#define REG_LCM_TCON1 			(LCM_BA+0x10)		// R/W: Timing Control Register 1
	#define TCON1_HSPW   			NVTBIT(23,16)		// Horizontal sync pulse width determines the HSYNC pulse's high level width by counting the number of the LCD Pixel Clock.
	#define TCON1_HBPD   			NVTBIT(15,8)		// Horizontal back porch is the number of LCD Pixel Clock periods between the falling edge of HSYNC and the start of active data.
	#define TCON1_HFPD   			NVTBIT(7,0)			// Horizontal front porch is the number of LCD Pixel Clock periods between the end of active data and the rising edge of HSYNC.

#define REG_LCM_TCON2 			(LCM_BA+0x14)		// R/W: Timing Control Register 2
	#define TCON2_VSPW				NVTBIT(23,16)		// Vertical sync pulse width determines the VSYNC pulse's high level width by counting the number of inactive lines.
	#define TCON2_VBPD				NVTBIT(15,8)		// Vertical back porch is the number of inactive lines at the start of a frame, after vertical synchronization period.
	#define TCON2_VFPD				NVTBIT(7,0)		// Vertical front porch is the number of inactive lines at the end of a frame, before vertical synchronization period.

#define REG_LCM_TCON3 			(LCM_BA+0x18)		// R/W: Timing Control Register 3
	#define TCON3_PPL				NVTBIT(31,16)		// Pixel Per-LineThe PPL bit field specifies the number of pixels in each line or row of screen.
	#define TCON3_LPP				NVTBIT(15,0)		// Lines Per-Panel The LPP bit field specifies the number of active lines per screen.

#define REG_LCM_TCON4 			(LCM_BA+0x1c)		// R : Timing Control Register 4
	#define TCON4_TAPN				NVTBIT(25,16)		// Horizontal Total Active Pixel Number
	#define TCON4_MVPW				NVTBIT(15,8)
	#define TCON4_MPU_FMARKP		BIT5
	#define TCON4_MPU_VSYNCP		BIT4
	#define TCON4_VSP				BIT3				// LCD VSYNC Polarity.
	#define TCON4_HSP				BIT2				// LCD HSYNC Polarity.
	#define TCON4_DEP				BIT1				// LCD VDEN Polarity.
	#define TCON4_PCLKP				BIT0				// LCD Pixel Clock Polarity.

#define REG_LCM_MPUCMD 			(LCM_BA+0x20)		// R/W: MPU-type LCD Command Register
	#define MPUCMD_MPU_VFPIN_SEL	BIT31
	#define MPUCMD_DIS_SEL			BIT30
	#define MPUCMD_CMD_DISn			BIT29				// Select command mode or display mode
	#define MPUCMD_MPU_CS			BIT28				// Set CS pin
	#define MPUCMD_MPU_ON			BIT27				// Trig to write or read from MPU in command mode
	#define MPUCMD_BUSY				BIT26				// Command interface is busy.
	#define MPUCMD_WR_RS			BIT25				// Write/Read RS Setting.
	#define MPUCMD_MPU_RWn			BIT24				// Read Status or data.
	#define MPUCMD_MPU68			BIT23				// MPU interface selection, reserved in NUC970
	#define MPUCMD_FMARK			BIT22				// Frame Mark Detection Disable/Enable
	#define MPUCMD_MPU_SI_SEL		NVTBIT(19,16)
	#define MPUCMD_MPU_CMD			VTBIT(15,0)			// MPU-type LCD command/parameter data, read data

#define REG_LCM_MPUTS 			(LCM_BA+0x24)		// R/W: MPU type LCD timing setting
	#define MPUTS_CSnF2DCt			NVTBIT(31,24)		// CSn fall edge to data change clock counter
	#define MPUTS_WRnR2CSnRt		NVTBIT(23,16)		// WRn rising edge to CSn rising clock counter
	#define MPUTS_WRnLWt			NVTBIT(15, 8)		// WR low pulse clock counter	
	#define MPUTS_CSnF2WRnFt		NVTBIT( 7, 0)		// CSn falling edge to WR falling edge clock counter	

#define REG_LCM_OSD_CTL 		(LCM_BA+0x28)		// R/W : OSD Control Register
	#define OSD_CTL_OSD_EN			BIT31
	#define OSD_CTL_OSD_FSEL		NVTBIT(27,24)
	#define OSD_CTL_OSD_TC			NVTBIT(23,0)

#define REG_LCM_OSD_SIZE 		(LCM_BA+0x2C)		// R/W: OSD Picture Size
	#define OSD_SIZE_OSD_VSIZE		NVTBIT(25,16)
	#define OSD_SIZE_OSD_HSIZE		NVTBIT(9,0)

#define REG_LCM_OSD_SP	 		(LCM_BA+0x30)		// R/W: OSD Start Position
	#define OSD_SP_OSD_SY			NVTBIT(25,16)
	#define OSD_SP_OSD_SX			NVTBIT(9,0)

#define REG_LCM_OSD_BEP	 		(LCM_BA+0x34)		// R/W: OSD Bar End Position
	#define OSD_BEP_OSD_1BEY		NVTBIT(25,16)
	#define OSD_BEP_OSD_1BEX		NVTBIT(9,0)

#define REG_LCM_OSD_BO			(LCM_BA+0x38)		// R/W: OSD Bar Offset
	#define OSD_BO_OSD_BOY			NVTBIT(25,16)
	#define OSD_BO_OSD_BOX			NVTBIT(9,0)

#define REG_LCM_CBAR	  		(LCM_BA+0x3C)		// R/W: Color Burst Avtive Region
	#define CBAR_CTL_EQ6SEL			BIT28
	#define CBAR_CTL_HCBEPC			NVTBIT(25,16)
	#define CBAR_CTL_HCBBPC			NVTBIT(9,0)

#define REG_LCM_TVCtl			(LCM_BA+0x40)		// R/W: TvControl Register
	#define TVCtl_TvField 			BIT31				// Tv field status (read only)
														//   	 1 = Odd field, 0 = even field
	#define TVCtl_TvCMM 			BIT16				// TV Color Modulation Method, reserved in NUC970
															//   1 = 27 MHz, 0 = 13.5 MHz 
	#define TVCtl_FBSIZE 			NVTBIT(15,14)		// Frame Buffer Size in Tv NonInterlance Mode
															//	00 = 320x240 (QVGA)
															//  01 = 640x240 (HVGA)
															//  10 = 640x480 (VGA)											
															//	11 = reserved
	#define TVCtl_LCDSrc 			NVTBIT(11,10)		// LCD image source selection
	#define TVCtl_TvSrc 			NVTBIT(9,8)			// TV image source selection
	#define TVCtl_TvLBSA 			BIT6				// Tv Line Buffer Scaling Alograthim (320->640)
	#define TVCtl_NotchE 			BIT5				// Notch Filter Enable/Disable
	#define TVCtl_Tvdac 			BIT4				// Tv DAC Enable/Disable
	#define TVCtl_TvInter 			BIT3				// Interlance or Non Interlance
	#define TVCtl_TvSys 			BIT2				// TV System Selection.
	#define TVCtl_TvColor 			BIT1				// TV Color Selection Color/Black.
	#define TVCtl_TvSleep 			BIT0				// TV Encoder Enable/Disable.

#define REG_LCM_IIRA			(LCM_BA+0x44)		// R/W: IIR Denominator Coefficient Register
	#define IIRA_IIRA3				NVTBIT(26,18)		// IIR Denominator A3 Coefficient
	#define IIRA_IIRA2				NVTBIT(17,9)		// IIR Denominator A2 Coefficient
	#define IIRA_IIRA1				NVTBIT(8,0)			// IIR Denominator A1 Coefficient

#define REG_LCM_IIRB			(LCM_BA+0x48)		// R/W: IIR notch filter Numberator Coefficient
	#define IIRB_IIRB3				NVTBIT(26,18)		// IIR Denominator B3 Coefficient
	#define IIRB_IIRB2				NVTBIT(17,9)		// IIR Denominator B2 Coefficient
	#define IIRB_IIRB1				NVTBIT(8,0)			// IIR Denominator B1 Coefficient

#define REG_LCM_COLORSET  		(LCM_BA+0x4C)		// R/W: Backdraw Color Setting Register
	#define  COLORSET_Color_R		NVTBIT(23,16)		// Color R value
	#define  COLORSET_Color_G		NVTBIT(15,8)		// Color G value
	#define  COLORSET_Color_B		NVTBIT(7,0)			// Color B value

#define REG_LCM_FSADDR    		(LCM_BA+0x50)		// R/W: Frame Buffer Start Address

#define REG_LCM_TVDisCtl  		(LCM_BA+0x54)		// R/W: TV Display Start Control Register
	#define TVDisCtl_FFRHS			NVTBIT(31,24)
	#define TVDisCtl_LCDHB			NVTBIT(23,16)		// LCD H bland setting for Syn TV Display
	#define TVDisCtl_TVDVS			NVTBIT(15,8)		// TV Display Start Line Register
	#define TVDisCtl_TVDHS			NVTBIT(7,0)			// TV Display Start Pixel Register

#define REG_LCM_CBACtl  		(LCM_BA+0x58)		// R/W: Color Burst Amplitude Control Register
	#define CBACtl_CBA_CB4			NVTBIT(29,24)
	#define CBACtl_CBA_CB3			NVTBIT(21,16)
	#define CBACtl_CBA_CB2			NVTBIT(13,8)
	#define CBACtl_CBA_CB1			NVTBIT(5,0)

#define REG_LCM_OSD_ADDR  		(LCM_BA+0x5C)		// R/W: OSD Frame Buffer Start Address
#define REG_RESERVED2 	  	(LCM_BA+0x60)		// Reserved

#define REG_LCM_TVContrast		(LCM_BA+0x64)		// R/W: Tv contrast adjust setting register
	#define TVContrast_Cr_contrast	NVTBIT(23,16)		// Cr compoment contrast adjust
	#define TVContrast_Cb_contrast	NVTBIT(15,8)		// Cb compoment contrast adjust
	#define TVContrast_Y_contrast	NVTBIT(7,0)			// Y  compoment contrast adjust

#define REG_LCM_TVBright  		(LCM_BA+0x68)		// R/W: Tv Bright adjust setting register
	#define TVBright_Cr_gain		NVTBIT(23,16)		// Cr compoment bright adjust
	#define TVBright_Cb_gain		NVTBIT(15,8)		// Cb compoment bright adjust
	#define TVBright_Y_bright		NVTBIT(7,0)			// Y  compoment bright adjust

#define REG_RESERVED3	  	(LCM_BA+0x6C)		// Reserved

#define REG_LCM_LINE_STRIPE		(LCM_BA+0x70)		// R/W : Line Stripe Offset
	#define LINE_STRIPE_F1_LSL		NVTBIT(15,0)
	
#define REG_LCM_RGBin		  	(LCM_BA+0x74)		// RGB888 data input for RGB2YCbCr equation

#define REG_LCM_YCbCrout  		(LCM_BA+0x78)		// YCbCr data output for RGB2YCbCr equation

#define REG_LCM_YCbCrin	  		(LCM_BA+0x7C)		// YCbCr data input for YCbCr2RGB equation
	#define YCbCrin_Yin				NVTBIT(23,16)		// Y byte data input
	#define YCbCrin_Cbin			NVTBIT(15, 8)		// Cb byte data input
	#define YCbCrin_Crin			NVTBIT( 7, 0)		// Cr byte data input

#define REG_LCM_RGBout	  		(LCM_BA+0x80)		// RGB data output for YCbCr2RGB equation
	#define RGBout_Rout				NVTBIT(23,16)		// R byte data output
	#define RGBout_Gout				NVTBIT(15, 8)		// G byte data output
	#define RGBout_Bout				NVTBIT( 7, 0)		// B byte data output



/* GPIO Control Registers */
#define GPIO_BA		NUC970_VA_GPIO		/* GPIO Control */

#define REG_GPIOA_OMD   (GPIO_BA+0x0000)   // GPIO port A bit Output mode Enable
#define REG_GPIOA_PUEN  (GPIO_BA+0x0004)	 // GPIO port A Bit Pull-up Resistor Enable
#define REG_GPIOA_DOUT  (GPIO_BA+0x0008)   // GPIO port A data output value 
#define REG_GPIOA_PIN   (GPIO_BA+0x000C)	 // GPIO port A Pin Value

#define REG_GPIOB_OMD   (GPIO_BA+0x0010)   // GPIO port B bit Output mode Enable
#define REG_GPIOB_PUEN  (GPIO_BA+0x0014)	 // GPIO port B Bit Pull-up Resistor Enable
#define REG_GPIOB_DOUT  (GPIO_BA+0x0018)   // GPIO port B data output value 
#define REG_GPIOB_PIN   (GPIO_BA+ 0x001C)	 // GPIO port B Pin Value

#define REG_GPIOC_OMD   (GPIO_BA+0x0020)  // GPIO port C bit Output mode Enable
#define REG_GPIOC_PUEN  (GPIO_BA+0x0024)	// GPIO port C Bit Pull-up Resistor Enable
#define REG_GPIOC_DOUT  (GPIO_BA+0x0028)  // GPIO port C data output value 

#define REG_GPIOC_PIN   (GPIO_BA+0x002C)	// GPIO port C Pin Value

#define REG_GPIOD_OMD   (GPIO_BA+0x0030)  // GPIO port D bit Output mode Enable
#define REG_GPIOD_PUEN  (GPIO_BA+0x0034)	// GPIO port D Bit Pull-up Resistor Enable
#define REG_GPIOD_DOUT  (GPIO_BA+0x0038)  // GPIO port D data output value 
#define REG_GPIOD_PIN   (GPIO_BA+0x003C)	// GPIO port D Pin Value

#define REG_GPIOE_OMD   (GPIO_BA+0x0040)  // GPIO port E bit Output mode Enable
#define REG_GPIOE_PUEN  (GPIO_BA+0x0044)	// GPIO port E Bit Pull-up Resistor Enable
#define REG_GPIOE_DOUT  (GPIO_BA+0x0048)  // GPIO port E data output value 
#define REG_GPIOE_PIN   (GPIO_BA+ 0x004C)	// GPIO port E Pin Value

#define REG_DBNCECON   (GPIO_BA+0x0070)  // External Interrupt Debounce Control

#define REG_IRQSRCGPA  (GPIO_BA+0x0080)  // GPIO Port A IRQ Source Grouping
#define REG_IRQSRCGPB  (GPIO_BA+0x0084)  // GPIO Port B IRQ Source Grouping
#define REG_IRQSRCGPC  (GPIO_BA+0x0088)  // GPIO Port C IRQ Source Grouping
#define REG_IRQSRCGPD  (GPIO_BA+0x008C)  // GPIO Port D IRQ Source Grouping
#define REG_IRQSRCGPE  (GPIO_BA+0x0090)  // GPIO Port E IRQ Source Grouping

#define REG_IRQENGPA   (GPIO_BA+0x00A0)  // GPIO Port A Interrupt Enable
#define REG_IRQENGPB   (GPIO_BA+0x00A4)  // GPIO Port B Interrupt Enable
#define REG_IRQENGPC   (GPIO_BA+0x00A8)  // GPIO Port C Interrupt Enable
#define REG_IRQENGPD   (GPIO_BA+0x00AC)  // GPIO Port D Interrupt Enable
#define REG_IRQENGPE   (GPIO_BA+0x00B0)  // GPIO Port E Interrupt Enable

#define REG_IRQLHSEL   (GPIO_BA+0x00C0)  // Interrupt Latch Trigger Selection Register

#define REG_IRQLHGPA   (GPIO_BA+0x00D0)  // GPIO Port A Interrupt Latch Value
#define REG_IRQLHGPB   (GPIO_BA+0x00D4)  // GPIO Port B Interrupt Latch Value
#define REG_IRQLHGPC   (GPIO_BA+0x00D8)  // GPIO Port C Interrupt Latch Value
#define REG_IRQLHGPD   (GPIO_BA+0x00DC)  // GPIO Port D Interrupt Latch Value
#define REG_IRQLHGPE   (GPIO_BA+0x00E0)  // GPIO Port E Interrupt Latch Value

#define REG_IRQTGSRC0   (GPIO_BA+0x00F0) // IRQ0~3 Interrupt Trigger Source Indicator from GPIO Port A and GPIO Port B
#define REG_IRQTGSRC1   (GPIO_BA+0x00F4) // IRQ0~3 Interrupt Trigger Source Indicator from GPIO Port C
#define REG_IRQTGSRC2   (GPIO_BA+0x00F8) // IRQ0~3 Interrupt Trigger Source Indicator from GPIO Port E

