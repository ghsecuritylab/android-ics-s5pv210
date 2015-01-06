#ifndef ANDROID_SCAN_INTERFACE_H
#define ANDROID_SCAN_INTERFACE_H

#include <hardware/hardware.h>

__BEGIN_DECLS

#define PREFIX_LEN	4
#define SUFFIX_LEN	2
#define MAX_TX_DATA_LEN	50
#define MAX_RX_DATA_LEN	54
#define MAX_TX_BUF_LEN	MAX_TX_DATA_LEN + PREFIX_LEN + SUFFIX_LEN
#define MAX_RX_BUF_LEN	MAX_RX_DATA_LEN + PREFIX_LEN + SUFFIX_LEN

#define READ_TIME_OUT	3000//ms

#define SUCCESS		0
#define SSI_COMMERR		-4570
#define SSI_CHECKSUMERR	-4571
#define SSI_ACKERR	-4572
#define SSI_CMDERR	-4573
#define SSI_BUSBUSY 	-4574
#define SSI_TX_OVERFOLW 	-4575
#define SSI_RX_OVERFOLW 	-4576
#define SSI_SEL_SERIAL_FAIL 	-4577
#define SSI_SOCKET_FAIL 	-4578

#define  HOST	0x04
#define  DECODER	0x00

#define IOCTL_MAGIC		'S'
#define SERIAL_SEL_GPS		_IO(IOCTL_MAGIC, 1)
#define SERIAL_SEL_SCAN		_IO(IOCTL_MAGIC, 2)
#define SERIAL_SEL_EXT_PRINTER		_IO(IOCTL_MAGIC, 3)
#define POWER_GPS		_IOW(IOCTL_MAGIC, 4, unsigned long)
//#define POWER_SCAN			_IOW(IOCTL_MAGIC, 5, unsigned long)
#define POWER_EXT_PRINTER		_IOW(IOCTL_MAGIC, 6, unsigned long)

/*Deactivate aim pattern.*/
#define AIM_OFF		0xC4
/*Activate aim pattern.*/
#define AIM_ON		0xC5
/*Sound the beeper.*/
#define BEEP		0xE6
/*Positive acknowledgment of received packet.*/
#define CMD_ACK		0xD0
/*Negative acknowledgment of received packet*/
#define CMD_NAK		0xD1
/* Custom defaults option to write/restore */
#define CUSTOM_DEFAULTS		0x12
/*Decode data in SSI packet format.*/
#define DECODE_DATA		0xF3
/*Event indicated by associated event code.*/
#define EVENT		0xF6
/*De-activate LED output.*/
#define LED_OFF		0xE8
/*Activate LED output.*/		
#define LED_ON		0xE7
/*Set parameter default values.*/
#define PARAM_DEFAULTS		0xC8
/*Request values of certain parameters.*/
#define PARAM_REQUEST		0xC7
/*Send parameter values.*/
#define PARAM_SEND		0xC6
/*Reply to REQ_REV contains decoders software hardware configuration.*/
#define REPLY_REVISION		0xA4
/*Request the decoder's configuration.*/
#define REQUEST_REVISION		0xA3
/*Prevent the operator from scanning bar codes*/
#define SCAN_DISABLE		0xEA
/*Permit bar code scanning.*/
#define SCAN_ENABLE		0xE9
/*Request to place the decoder into low power.*/
#define SLEEP		0xEB
/*Tell decoder to attempt to decode a bar code.*/
#define START_DECODE		0xE4
/*Tell decoder to abort a decode attempt.*/
#define STOP_DECODE		0xE5
/*N/A Wakeup decoder after it’s been powered dow*/
#define WAKEUP   

enum bar_type
{ 
	BCT_CODE_39 = 0,      // Code 39 
	BCT_CODABAR,      // CodaBar 
	BCT_CODE_128,      // Code 128 
	BCT_DISCRETE_2OF5,     // Discrete 2 of 5 
	BCT_INTERLEAVED_2OF5, // Interleaves 2 of 5 
	BCT_CODE_93,      // Code 93 
	BCT_UPC_A,       // UPC A 
	BCT_UPC_A_2SUPPS,   // UPC A with 2 Supps 
	BCT_UPC_A_5SUPPS,   // UPC A with 5 Supps 
	BCT_UPC_E0,      // UPC E 
	BCT_UPC_E0_2SUPPS,   // UPC E with 2 Supps 
	BCT_UPC_E0_5SUPPS,   // UPC E with 5 Supps  
	BCT_EAN_8,       // EAN 8 
	BCT_EAN_13,      // EAN 13 
	BCT_EAN_13_2SUPPS,   // EAN 13 with 2 Supps 
	BCT_EAN_13_5SUPPS,   // EAN 13 with 5 Supps 
	BCT_MSI_PLESSEY,    // MSI Plessey 
	BCT_EAN_128,      // EAN 128 
	BCT_UPC_E1,      // UPC E1 
	BCT_UPC_E1_2SUPPS,   // UPC E1 with 2 Supps 
	BCT_UPC_E1_5SUPPS,   // UPC E1 with 5 Supps 
	BCT_TRIOPTIC_CODE_39, // TRIOPTIC CODE 39 
	BCT_BOOKLAND_EAN,   // Bookland EAN 
	BCT_COUPON_CODE,    // Coupon Code 
	BCT_RSS_14,              // RSS-14 
	BCT_RSS_LIMITED,        // RSS-Limited 
	BCT_RSS_EXPANDED,       // RSS-Expanded 
	BCT_CODE_11,      // Code 11  
	BCT_CODE_32,      // Code 32 
	BCT_CODE_39_FULLASCII,  // Code 39 full ascii 
	BCT_ISBT_128,            // ISBT-128 
	BCT_UNIDENTIFY,         // unidentifiable code 
}; 

struct barcode_t 
{ 
//	enum bar_type type; //条码的类型
	int len;   //条码的长度。 
	unsigned char code[MAX_RX_BUF_LEN];
}; 

struct prefix_t
{
	/*Length of message not including the check sum
	bytes.*/
	char length;
	/*Identifies the type of packet data being sent.*/
	unsigned char opcode;
	/*Identifies where the message is coming from.*/
	unsigned char message_source;
	unsigned char status ;
};

struct suffix_t
{
	/*Checksum of message formatted as HIGH BYTE - LOW BYTE
	The checksum is a 2 byte checksum and must  be sent as HIGH BYTE followed by LOW BYTE.*/
	unsigned short  checksum;
};

struct SSI_command_t
{
	struct prefix_t prefix;
	unsigned char data[MAX_TX_DATA_LEN] ;
	struct suffix_t suffix;
};

struct SSI_response_t
{
	struct prefix_t prefix;
	unsigned char data[MAX_RX_DATA_LEN] ;
	struct suffix_t suffix;
};

//扫描头配置参数 
struct scanner_params_t
{ 
	// common 
	unsigned char  BeeperVolume; 
	unsigned char  BeeperTone; 
	unsigned char  BeeperFrequencyAdjustment; 
	unsigned char  LaserOnTime; 
	unsigned char  AimDuration; 
	unsigned char  ScanAngle; 
	unsigned char  PowerMode; 
	unsigned char  TriggerMode; 
	unsigned char  TimeoutBetweenSameSymbol; 
	unsigned char  BeepAfterGoodDecode; 
	unsigned char  TransmitNoReadMessage; 
	unsigned char  ParameterScanning; 
	unsigned char  LinearCodeTypeSecurityLevels; 
	unsigned char  BidirectionalRedundancy; 

	// UPC/EAN 
	unsigned char  UPC_A; 
	unsigned char  UPC_E; 
	unsigned char  UPC_E1; 
	unsigned char  EAN_8; 
	unsigned char  EAN_13; 
	unsigned char  Bookland_EAN; 
	unsigned char  Decode_UPC_EAN_Supplementals; 

	unsigned char  Decode_UPC_EAN_SupplementalRedundancy; 
	unsigned char  Transmit_UPC_A_CheckDigit; 
	unsigned char  Transmit_UPC_E_CheckDigit; 
	unsigned char  Transmit_UPC_E1_CheckDigit; 
	unsigned char  UPC_A_Preamble; 
	unsigned char  UPC_E_Preamble; 
	unsigned char  UPC_E1_Preamble; 
	unsigned char  Convert_UPC_E_to_A; 
	unsigned char  Convert_UPC_E1_to_A; 
	unsigned char  EAN_8_ZeroExtend; 
	unsigned char  Convert_EAN_8_to_EAN_13_Type; 
	unsigned char  UPC_EAN_SecurityLevel; 
	unsigned char  UCC_CouponExtendCode; 

	// Code 128 
	unsigned char  Code128; 
	unsigned char  UCC_EAN_128; 
	unsigned char  ISBT_128; 

	// Code 39 
	unsigned char  Code39; 
	unsigned char  TriopticCode39; 
	unsigned char  Covert_Code39_to_Code32; 
	unsigned char  Code32Prefix; 
	unsigned char  SetLengthForCode39; 
	unsigned char  SetLengthForCode39_2; 
	unsigned char  Code39CheckDigitVerification; 
	unsigned char  TransmitCode39CheckDigit; 
	unsigned char  Code39FullAsciiConversion; 

	// Code 93 
	unsigned char  Code93; 
	unsigned char  SetLengthForCode93; 
	unsigned char  SetLengthForCode93_2; 

	// Code 11 
	unsigned char  Code11; 
	unsigned char  SetLengthForCode11; 
	unsigned char  SetLengthForCode11_2; 
	unsigned char  Code11CheckDigitVerification; 
	unsigned char  TransmitCode11CheckDigit; 

	// Interleaved 2 of 5 
	unsigned char  Interleaved_2_5; 


	unsigned char  SetLengthFor_I2_5; 
	unsigned char  SetLengthFor_I2_5_2; 
	unsigned char  I2_5_CheckDigitVerification; 
	unsigned char  Transmit_I2_5_CheckDigit; 
	unsigned char  Convert_I2_5_to_EAN_13; 

	// Descrete 2 of 5 
	unsigned char  Descrete_2_5; 
	unsigned char  SetLengthFor_D2_5; 
	unsigned char  SetLengthFor_D2_5_2; 

	// Chinese 2 of 5 
	unsigned char  Chinese_2_5; 

	// Codabar 
	unsigned char  Codabar; 
	unsigned char  SetLengthsForCodabar; 
	unsigned char  SetLengthsForCodabar_2; 
	unsigned char  CISI_Editing; 
	unsigned char  NOTIS_Editing; 

	// MSI 
	unsigned char  MSI; 
	unsigned char  SetLengthsForMSI; 
	unsigned char  SetLengthsForMSI_2; 
	unsigned char  MSICheckDigits; 
	unsigned char  TransmitMSICheckDigit; 
	unsigned char  MSICheckDigitAlgorithm; 

	// RSS 
	unsigned char  RSS_14; 
	unsigned char  RSS_Limited; 
	unsigned char  RSS_Expanded; 
	unsigned char  Covert_RSS_to_UPC_EAN; 

	// Data options 
	unsigned char  TransmitCodeIDCharacter; 
	unsigned char  Prefix; 
	unsigned char  Suffix1; 
	unsigned char  Suffix2; 
	unsigned char  ScanDataTransmissionFormat; 

	// Serial interface 
	unsigned char  BaudRate; 


	unsigned char  Parity; 
	unsigned char  SoftwareHandshaking; 
	unsigned char  DecodeDataPacketFormat; 
	unsigned char  HostSerialResponseTimeout; 
	unsigned char  StopBitSelect; 
	unsigned char  InterCharacterDelay; 
	unsigned char  HostCharacterTimeout; 

	// Event reporting 
	unsigned char  DecodeEvent; 
	unsigned char  BootupEvent; 
	unsigned char  ParameterEvent; 
};

/**
 * The id of this module
 */
#define SCAN_HARDWARE_MODULE_ID "scan"

/**
 * The id of this device
 */
#define SCAN_HARDWARE_DEVICE_ID "scan"

struct scan_module_t {
	struct hw_module_t common;
};

struct scan_device_t {
	struct hw_device_t common;
	int (*get_barcode)(struct scan_device_t* dev, struct barcode_t *barcode);
};

__END_DECLS

#endif
