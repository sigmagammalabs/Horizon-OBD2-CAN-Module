/****************************************************************************
  FileName:         Horizon_OBD_Teensy_36_V100.c
  FileDescription:  Main Software for Universal Programmable OBD Module
  Author:           SY
  DATE:				03/2020
 ****************************************************************************/
/****************************************************************************
                            VERSIONS
							
	copied from v50.D HSG Software-----
	
	v1.00: 	2020/03/27			First Functional Version
	

 /****************************************************************************/
 
 /****************************************************************************
                            File Inclusions
 /****************************************************************************/
#include <FlexCAN.h>

#include <SerialFlash.h>
#include <SPI.h>
#include <SD.h>
#include <String.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Metro.h> 
#include <T3Mac.h>
#include <Adafruit_SleepyDog.h>
#include <kinetis_flexcan.h>

/****************************************************************************
                            Constants
 /****************************************************************************/


/****************************************************************************/
/****************************************************************************/
/****************************************************************************/


#define FW_VERSION						0x100
#define HW_VERSION						0x310
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/ 
 


/*SD CARD*/
#define SDCARD_MOSI_PIN  				7
#define SDCARD_SCK_PIN   				14
			
#define SDCARD_CS_PIN    				BUILTIN_SDCARD
#define SDCARD_MOSI_PIN  				11  		// not actually used
#define SDCARD_SCK_PIN   				13  		// not actually used
			
/*CAN COMMUNICATION*/	
#define CAN_MSG_LEN 					0x8
#define CAN_TX_EXTENDED_ID 				0x18DF
#define CAN_RX_EXTENDED_ID 				0x18DF
#define CAN_BAUDRATE 					500000

#define CAN_STANDARD_TX_ID 				0x7DF       /* Standard CAN TX ID */
#define CAN_STANDARD_RX_ID_L			0x7E8       /* Standard CAN RX ID */
#define CAN_STANDARD_RX_ID_U			0x7EA       /* Standard CAN RX ID */
#define CAN_EXTENDED_TX_ID 				0x18DAF100  /* Extended CAN TX ID */
#define CAN_EXTENDED_RX_ID 				0x18DB33F1  /* Extended CAN RX ID */
#define CAN_EXTENDED_COMMUNICATION   	0 			/*1: For Extended CAN, 0: For Standard CAN*/
			
#define CAN_WRITE_INTERVAL 				10000 //2000
#define ANALOG_READ_INTERVAL 			100000
#define CAN_FILTER_INTERVAL				200
#define SOUND_CHANGE_INTERVAL 			30000
#define BT_WRITE_INTERVAL				100000
#define LED_CTRL_INTERVAL				100000
#define SOUND_SETTINGS_INTERVAL			100000
#define STARTSTOP_INTERVAL				10000
#define ENGINEOFF_INTERVAL				200000
#define	LED_BLINK_INTERVAL				5
#define SOUND_EFFECT_DAC_INTERVAL		4 //10
#define SOUND_EFFECT_INVOKE_INTERVAL	100000
#define WATCHDOG_RESET					2000

/*OBD*/
#define rpm_dec_level					50
#define rpm_inc_level					50
		
/*SERIAL*/			
#define SERIAL_MONITOR_PORT				Serial
#define BLUETOOTH_SERIAL_PORT_HC05 		Serial3
#define BLUETOOTH_SERIAL_PORT_HM10 		Serial1

#define SERIAL_BANDWIDTH 				115200
#define BLUETOOTH_BANDWIDTH   			9600
			
/*LED*/			
#define LED    							13
#define OBD_LED							30
			
/*IOs*/			
#define	HC05_EN							4
#define	HC05_KEY						5
#define	HM10_EN							2
#define	HM10_KEY						3

#define SYS_ON							23
#define IR_REMOTE						32

/*ANALOG INPUTS*/
#define ANALOG_BAT						A17
#define ANALOG_KL15						A16
			
/*PID*/			
#define RPM     						0x0C
#define SPEED   						0x0D
#define LOAD    						0x04
#define ETH     						0x11
#define VERSION							0xAF
			


/*TIMER*/
#define MIN_TIMERTRIGGER_INTERVAL		5



/*RESTART REGISTERS*/
#define RESTART_ADDR       				0xE000ED0C
#define READ_RESTART()     				(*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val)				((*(volatile uint32_t *)RESTART_ADDR) = (val))



/****************************************************************************
                            Variables Definions
/****************************************************************************/

/*SUPPORT VARIABLES*/
uint16_t 								temp 						= 0;
uint16_t 								i, j, k 					= 0;
uint32_t								UIDML						= 0;
uint32_t								UIDL						= 0;
uint32_t								UIDMH						= 0;
uint8_t 								index_var					= 0;

/*TIMERS*/
Metro serial_debug 						= Metro(500);
Metro tendency_check					= Metro(10);
IntervalTimer 							sample_trigger;


/*CAN INTERFACE*/
static 									CAN_message_t txMsg;
static 									CAN_message_t rxMsg;
static uint8_t 							rxCount 					= 0;
static uint8_t 							hex[17] 					= "0123456789abcdef";
static uint8_t							std_id_flag					= 0;
static uint8_t							ext_id_flag					= 0;
static uint8_t							obd_id_lock					= 0;
static uint8_t							filter_switch				= 0;
static CAN_filter_t 					Can1Filter;
static CAN_filter_t 					Can1Mask;
				
/*STATE MACHINE*/				
static uint8_t 							can_write_switch 			= 0;
static uint8_t							random_sound_effect			= 0;
static uint32_t 						analog_read_ticks 			= 0;
static uint32_t 						can_write_ticks 			= 500;
static uint32_t							can_filter_ticks			= 0;
static uint32_t 						startstop_ticks 			= 0;
static uint32_t							dac_write_interval 			= 25;
static uint32_t							random_interval 			= 0;
static uint32_t							led_blink_ticks				= 0;
static uint32_t							watchdog_reset_ticks		= 0;

				
/*OBD*/				
static uint16_t							OBD_RPM_value 				= 0;
static uint16_t							o_OBD_RPM_value[5];
static uint16_t 						OBD_SPEED_value				= 0; 
static uint16_t 						o_OBD_SPEED_value[5];
static uint16_t 						OBD_ETH_value				= 0; 
static uint16_t 						o_OBD_ETH_value[5];
static uint16_t 						OBD_LOAD_value 				= 0;
static uint16_t 						o_OBD_LOAD_value[5];
static uint8_t							OBD_RPM_rising 				= 0;
static uint8_t							OBD_RPM_falling 			= 0;
static uint8_t							OBD_RPM_static 				= 0;
				

/*BLUETOOTH*/				
static uint8_t 							bluetoothReceivedFrame;				
static uint32_t 						bluetooth_read_ticks 		= 0;
static uint32_t 						bluetooth_write_ticks 		= 500;
static uint8_t							uart_master_counter			= 0;

/*DEBUG*/
static uint8_t							debug_command				= 0;		
static uint8_t							receive_int					= 0;
static String							debug_string				;

/*FLAGS*/
static uint8_t 							flag 						= 0;
static uint8_t 							obd_active_flag 			= 0;
static uint8_t 							bt_active_flag 				= 0;
static uint8_t 							amp_active_flag 			= 0;
static uint8_t 							startstop_flag 				= 0;
static uint8_t 							engineoff_flag 				= 0;

static uint8_t							debug_flag 					= 0;

/*ANALOG*/
static uint16_t							analog_bat					= 0;
static uint16_t							analog_kl15					= 0;


/*****************************************************************************
                            Setup Function (Executed Once)
/****************************************************************************/

void setup() {	

	  
	/*LED*/
	pinMode(LED, OUTPUT);
	
	/*IO CONTROL*/	
	pinMode(HC05_EN, 	OUTPUT);
	pinMode(HM10_EN, 	OUTPUT);
	pinMode(HC05_KEY, 	OUTPUT);
	pinMode(HM10_KEY, 	OUTPUT);
	pinMode(OBD_LED, 	OUTPUT);
	pinMode(SYS_ON, 	OUTPUT);
	
	
	/*Keep System Supply On*/
	digitalWrite(SYS_ON, HIGH);
	
	/*INITIALIZE SERIAL MONITOR, AND WRITE WELCOME MESSAGE*/  
	SERIAL_MONITOR_PORT.begin (SERIAL_BANDWIDTH);
	
	/*START BLUETOOTH COMMUNICATION*/
	BLUETOOTH_SERIAL_PORT_HC05.begin(BLUETOOTH_BANDWIDTH);
	BLUETOOTH_SERIAL_PORT_HM10.begin(BLUETOOTH_BANDWIDTH);
	
	SPI.setMOSI(SDCARD_MOSI_PIN);
	SPI.setSCK(SDCARD_SCK_PIN);
			
	
	/*START CAN1*/
	Can1.begin(CAN_BAUDRATE); 	

	
	/*CAN INTERFACE*/
	//txMsg.id = CAN_STANDARD_TX_ID;
	txMsg.ext = 1;
	txMsg.id = 0x18DAF100;		
	txMsg.len = CAN_MSG_LEN;	
	
	txMsg.buf[0] = 0x2;
	txMsg.buf[1] = 0x1;
	txMsg.buf[2] = RPM;
	txMsg.buf[3] = 0x0;
	txMsg.buf[4] = 0x0;
	txMsg.buf[5] = 0x0;
	txMsg.buf[6] = 0x0;
	txMsg.buf[7] = 0x0; 
		
				
	digitalWrite(HC05_EN, HIGH);
	digitalWrite(HM10_EN, HIGH);
			
}

/*****************************************************************************
                            Loop Function (Executed forever)
/****************************************************************************/
void loop() {	

	can_write_ticks++;
	analog_read_ticks++;	
	can_filter_ticks++;
	bluetooth_write_ticks++;
	watchdog_reset_ticks++;
		
	if (watchdog_reset_ticks > WATCHDOG_RESET){
		Watchdog.reset();				
		watchdog_reset_ticks = 0;
	}

	
	/*READ OUT ANALOG INPUT VALUES*/
	if (analog_read_ticks > ANALOG_READ_INTERVAL){
		analog_bat = analogRead(ANALOG_BAT);
		analog_kl15 = analogRead(ANALOG_KL15);
				
		if (analog_kl15 > 100){
			digitalWrite(SYS_ON, HIGH);
		}
		else {
			delay(3000);
			digitalWrite(SYS_ON, LOW);
		}
		analog_read_ticks = 0;
	}	
	

	/*DEBUG Port Input*/
	if (SERIAL_MONITOR_PORT.available() > 0) {
		
		debug_command = SERIAL_MONITOR_PORT.read();
		switch (debug_command){
		/*letter a*/
		case 0x61:	SERIAL_MONITOR_PORT.println("Info:\n0: OBD Value ETH\n1: OBD Value RPM\n2: OBD Value Load\n3: OBD Value Speed\n4: KL15 ADC Measurement\n5: KL30 ADC Measurement\nV: Firmware Version\nW: Compatible Hardware Version HSG\nb: Enable Debug readout function"); break;			
		/*number 0*/
		case 0x30:	SERIAL_MONITOR_PORT.print("OBD Value ETH: ");SERIAL_MONITOR_PORT.println(o_OBD_ETH_value[0] ,DEC); break;
		/*number 1*/
		case 0x31:	SERIAL_MONITOR_PORT.print("OBD Value RPM: ");SERIAL_MONITOR_PORT.println(o_OBD_RPM_value[0] ,DEC); break;
		/*number 2*/
		case 0x32:	SERIAL_MONITOR_PORT.print("OBD Value LOAD: ");SERIAL_MONITOR_PORT.println(o_OBD_LOAD_value[0] ,DEC); break;
		/*number 3*/
		case 0x33:	SERIAL_MONITOR_PORT.print("OBD Value SPEED: ");SERIAL_MONITOR_PORT.println(o_OBD_SPEED_value[0] ,DEC); break;
		/*number 4*/
		case 0x34:	SERIAL_MONITOR_PORT.print("KL15 ADC Value: ");SERIAL_MONITOR_PORT.println(analog_kl15 ,DEC); break;
		/*number 5*/
		case 0x35:	SERIAL_MONITOR_PORT.print("KL30 ADC Value: ");SERIAL_MONITOR_PORT.println(analog_bat ,DEC); break;
		
		/*letter V*/
		case 0x56:	SERIAL_MONITOR_PORT.print("Firmware Version: ");SERIAL_MONITOR_PORT.println(FW_VERSION ,HEX); break;		
		/*letter W*/
		case 0x57:	SERIAL_MONITOR_PORT.print("Compatible Hardware Version HSG: ");SERIAL_MONITOR_PORT.println(HW_VERSION ,HEX); break;	
		/*letter X*/
		case 0x58:	SERIAL_MONITOR_PORT.print("Reset System: "); WRITE_RESTART(0x5FA0004); break;

		
		
		case 0x62:	debug_flag = 0x1; break;
		default: break;			
		}		
	}
	
	/*DEBUG OUTPUT INTERVAL*/
	if (serial_debug.check() == 1) {		
				
		if (debug_flag){		
			SERIAL_MONITOR_PORT.write(27);	
			SERIAL_MONITOR_PORT.print("[2J");
			SERIAL_MONITOR_PORT.write(27);			
			SERIAL_MONITOR_PORT.print("[H");
			
			SERIAL_MONITOR_PORT.println("PREMIUM SOUND");
			SERIAL_MONITOR_PORT.print("Firmware Version: v.");
			SERIAL_MONITOR_PORT.print(FW_VERSION,HEX);
			SERIAL_MONITOR_PORT.println("\n");	

			SERIAL_MONITOR_PORT.println("-----------------------------------");
			SERIAL_MONITOR_PORT.print("OBD Value RPM: ");
			SERIAL_MONITOR_PORT.println( o_OBD_RPM_value[0]  ,DEC);

			SERIAL_MONITOR_PORT.print("OBD Value LOAD: ");
			SERIAL_MONITOR_PORT.println( o_OBD_LOAD_value[0]  ,DEC);
			
			SERIAL_MONITOR_PORT.print("OBD Value SPEED: ");
			SERIAL_MONITOR_PORT.println( o_OBD_SPEED_value[0]  ,DEC);
			SERIAL_MONITOR_PORT.println("\n");
			
			SERIAL_MONITOR_PORT.print("OBD Value ETH: ");
			SERIAL_MONITOR_PORT.println( o_OBD_ETH_value[0]  ,DEC);
			SERIAL_MONITOR_PORT.println("\n");
			
			SERIAL_MONITOR_PORT.print("CAN Rx ID: ");
			SERIAL_MONITOR_PORT.println( rxMsg.id  ,HEX);
			
			SERIAL_MONITOR_PORT.print("CAN Rx Buf 0: ");
			SERIAL_MONITOR_PORT.println( rxMsg.buf[0]  ,HEX);

			SERIAL_MONITOR_PORT.print("CAN Rx Buf 1: ");
			SERIAL_MONITOR_PORT.println( rxMsg.buf[1]  ,HEX);
			
			SERIAL_MONITOR_PORT.print("CAN Rx Buf 2: ");
			SERIAL_MONITOR_PORT.println( rxMsg.buf[2]  ,HEX);

			SERIAL_MONITOR_PORT.print("CAN Rx Buf 3: ");
			SERIAL_MONITOR_PORT.println( rxMsg.buf[3]  ,HEX);

			SERIAL_MONITOR_PORT.print("CAN Rx Buf 4: ");
			SERIAL_MONITOR_PORT.println( rxMsg.buf[4]  ,HEX);			
			SERIAL_MONITOR_PORT.println("\n");
			
			
			SERIAL_MONITOR_PORT.println("\n");
			SERIAL_MONITOR_PORT.println("-----------------------------------");
									
		}
	}		
		
				
	/*BLUETOOTH TRANSMITTING*/
	if (bluetooth_write_ticks > BT_WRITE_INTERVAL) {
		BLUETOOTH_SERIAL_PORT_HC05.write("Test");
				
		bluetooth_write_ticks = 0;
	}
	
	/*BLUETOOTH RECEIVING HC05*/
	while (BLUETOOTH_SERIAL_PORT_HC05.available()) {
		bluetoothReceivedFrame = BLUETOOTH_SERIAL_PORT_HC05.read();  		
		SERIAL_MONITOR_PORT.print("Bluetooth Data received: "); hexDump(1, &bluetoothReceivedFrame);			
	}	
	
	/*BLUETOOTH RECEIVING HM10*/
	while (BLUETOOTH_SERIAL_PORT_HM10.available()) {
		bluetoothReceivedFrame = BLUETOOTH_SERIAL_PORT_HM10.read();  		
		SERIAL_MONITOR_PORT.print("Bluetooth Data received: "); hexDump(1, &bluetoothReceivedFrame);
		
	}


	/*OBD CAN RECEIVE*/
	while (Can1.available()) {					
				
		if (rxMsg.ext == 0){
			std_id_flag = 1;
			ext_id_flag = 0;			
		}
		else {
			std_id_flag = 0;
			ext_id_flag = 1;			
		}
		
		Can1.read(rxMsg);
		//SERIAL_MONITOR_PORT.println(rxMsg.id);
		if ( ((rxMsg.id >= 0x7E8)&&(rxMsg.id <= 0x7EF)) || ((rxMsg.id >= 0x18DAF100)&&(rxMsg.id <= 0x18DAF1FF)) ) {					
			obd_id_lock = 1;
			obd_active_flag = 1;						
			
			if ( ( (ext_id_flag == 1)  ) || ( (std_id_flag == 1)  ) ){
			
				if (rxMsg.buf[2] == RPM) {
					if ( ((rxMsg.buf[3] << 8) + rxMsg.buf[4]) > 300 ){					
						OBD_RPM_value = ((rxMsg.buf[3] << 8) + rxMsg.buf[4])/4;	
						o_OBD_RPM_value[0] = OBD_RPM_value;													
					}
					else {
						OBD_RPM_value = 300;
					}			
				}
				else if (rxMsg.buf[2] == LOAD){
					OBD_LOAD_value = rxMsg.buf[3];
					o_OBD_LOAD_value[0] = OBD_LOAD_value;
				}
				else if (rxMsg.buf[2] == SPEED){
					OBD_SPEED_value = rxMsg.buf[3];
					o_OBD_SPEED_value[0] = OBD_SPEED_value;
				}				
				else if (rxMsg.buf[2] == ETH){
					OBD_ETH_value = rxMsg.buf[3];
					o_OBD_ETH_value[0] = OBD_ETH_value;
				}
			}
			
			SERIAL_MONITOR_PORT.print("OBD Value RPM: ");
			SERIAL_MONITOR_PORT.println( o_OBD_RPM_value[0]  ,DEC);
			SERIAL_MONITOR_PORT.print("OBD Value LOAD: ");
			SERIAL_MONITOR_PORT.println( o_OBD_LOAD_value[0]  ,DEC);			
			SERIAL_MONITOR_PORT.print("OBD Value SPEED: ");
			SERIAL_MONITOR_PORT.println( o_OBD_SPEED_value[0]  ,DEC);			
			SERIAL_MONITOR_PORT.print("OBD Value ETH: ");
			SERIAL_MONITOR_PORT.println( o_OBD_ETH_value[0]  ,DEC);
			SERIAL_MONITOR_PORT.println("\n");
			
			BLUETOOTH_SERIAL_PORT_HC05.print("OBD Value RPM: ");
			BLUETOOTH_SERIAL_PORT_HC05.println( o_OBD_RPM_value[0]  ,DEC);
			BLUETOOTH_SERIAL_PORT_HC05.print("OBD Value LOAD: ");
			BLUETOOTH_SERIAL_PORT_HC05.println( o_OBD_LOAD_value[0]  ,DEC);			
			BLUETOOTH_SERIAL_PORT_HC05.print("OBD Value SPEED: ");
			BLUETOOTH_SERIAL_PORT_HC05.println( o_OBD_SPEED_value[0]  ,DEC);			
			BLUETOOTH_SERIAL_PORT_HC05.print("OBD Value ETH: ");
			BLUETOOTH_SERIAL_PORT_HC05.println( o_OBD_ETH_value[0]  ,DEC);
			BLUETOOTH_SERIAL_PORT_HC05.println("\n");
			
		}
		else {
			//OBD_RPM_value = 0;
			//OBD_SPEED_value = 0;
			//OBD_LOAD_value = 0;
			//OBD_ETH_value = 0;
		}
	}	
	

	/*OBD CAN WRITE*/
	if (can_write_ticks > CAN_WRITE_INTERVAL) {	
		if (ext_id_flag == 1){
			txMsg.ext = 1;
			txMsg.id = 0x18DB33F1;			
			can_filter_ticks = 0;
		}
		else if (std_id_flag == 1){
			txMsg.ext = 0;
			txMsg.id = 0x7DF;			
			can_filter_ticks = 0;
		}
		else if ((std_id_flag == 0) && (ext_id_flag == 0)){
			txMsg.buf[2] = RPM;
			
			txMsg.ext = 0;
			txMsg.id = 0x7DF;
			Can1.write(txMsg);
			
			delay(100);
			
			txMsg.ext = 1;
			txMsg.id = 0x18DB33F1;		
			Can1.write(txMsg);						
		}
		
		switch(can_write_switch){
		case 0: txMsg.buf[2] = RPM;
				Can1.write(txMsg); can_write_switch++;
				break;
		case 1: txMsg.buf[2] = SPEED;
				Can1.write(txMsg); can_write_switch++;
			break;
		case 2: txMsg.buf[2] = LOAD;
				Can1.write(txMsg); can_write_switch++;
			break;
		case 3: txMsg.buf[2] = ETH;
				Can1.write(txMsg); can_write_switch++;
			break;			
		default: break;
		}
		if (can_write_switch>3) can_write_switch=0;
		can_write_ticks = 0;		
	}
	
	
	/*VISIBLE INDICATION ON TEENSY BOARD*/
	digitalWrite(LED, LOW); 	
}



/*****************************************************************************
                            HEX Function
/***************************************************************************/
static void hexDump(uint8_t dumpLen, uint8_t *bytePtr)
{
	uint8_t working;
	while ( dumpLen-- ) {
		working = *bytePtr++;
		Serial.write( hex[ working >> 4 ] );
		Serial.write( hex[ working & 15 ] );
	}
	Serial.write('\r');
	Serial.write('\n');
}

