#include <Arduino.h>
#include <Wire.h>

const char ADDR[] = {37, 36, 35, 34, 33, 32, 31, 30, 49, 48, 47, 46, 45, 44, 43, 42};
const char DATA[] = {22, 23, 24, 25, 26, 27, 28, 29};

const unsigned int words[]={0x5A5A, 0xA5A5, 0x0101, 0xEFEF, 0xAAAA, 0x5555, 0xAAAA, 0x5555, 0x0011, 0x0022, 0x0044, 0x0088, 0x1100, 0x2200, 0x4400, 0x8800};
const unsigned int twords[]={0x1A5A, 0x25A5, 0x3101, 0x4FEF, 0x5AAA, 0x6555, 0x7AAA, 0x8555, 0x9011, 0xA022, 0xB044, 0xC088, 0xD100, 0xE200, 0xF400, 0x0880};
const unsigned int Cwords[]={0x1111, 0x2222, 0x3333, 0x4444, 0x5555, 0x6666, 0x7777, 0x8888, 0x9999, 0xAAAA, 0xBBBB, 0xCCCC, 0xDDDD, 0xEEEE, 0xFFFF, 0x0000};

//PA0-7: D0-D7: 	22(PA0), 23(PA1) ,24(PA2) ,25(PA3) ,26(PA4) ,27(PA5) ,28(PA6), 29(PA7)
//PC0-7: D8-D15: 	37(PC0), 36(PC1), 35(PC2), 34(PC3), 33(PC4), 32(PC5), 31(PC6) 30(PC7)

#define P_SRAM_CE 2		//PB2	
#define P_SRAM_WR 1		//PB1	
#define P_SRAM_RD 0		//PB0	
#define P_SRAM_BHE 7	//PL7	
#define P_SRAM_BLE 6	//PL6

#define P_595OE 4	  	//PL4	// 595 output enable (OE)
#define P_SER 3	  		//PL3	// 595 serial data in (p14)
#define P_RCLK 2  		//PL2	// 595 clock output register (p12)
#define P_SRCLR 1 		//PL1	// 595 serial clear shift register (p10)
#define P_SRCLK 0 		//PL0	// 595 serial clock shift register (p11)

#define P_A16 	0		//PG0  	address line A16
#define P_A17 	1		//PG1  	address line A16

#define BURNTIME 2
#define INHIBITTIME 10
#define ERASEBYTE 0xAA
#define N_BYTES 256

#define ATWR 1	   // allow to write to memory 0=READ only; 1=WRITE
#define SRAM 0

#define NUM_SEGMENTS 20
#define USING_READBYTES	1	// max segment size 46!	

HardwareSerial *mySerial;		// used as resulting Serial 0 or 1.
int ex_listCommand();
int ex_byteCommand();
int ex_pokeCommand();
void ex_resetSRAMCommand();
void setAddress(unsigned long address);
int readLong(unsigned long *lg);
int readWord(unsigned int *dbdata);
unsigned int readMEM(unsigned long address);
void writeSRAM(unsigned long address, unsigned int indata);
int Tokenize(String *data, char separator, String _part[]);

char buf[80];

unsigned long startaddress, SRAMadr;
unsigned int body_size;

unsigned long mem_size = 0x40000;
unsigned int old_progress,progress;

unsigned int size, ret_word;
unsigned int byteValue;

// CRC 16
unsigned char x;
unsigned short crc = 0xFFFF;
unsigned short i_crc, f_crc, group;

//****************************************************

void setup()
{

	DDRA = 0x00;			// Data bus low byte = input
	DDRC = 0x00;			// Data bus low byte = input

	DDRG = (1<<P_A16 | 1<<P_A17);					// port G pins 0,1 are outputs									

	DDRL = (1 << P_RCLK | 1 << P_SRCLR | 1 << P_SRCLK | 1 << P_SER | 1 << P_595OE);	// Arduino ctrl pins for shiftreg 595 Outputs...
	DDRB |= (1 << P_SRAM_CE | 1 << P_SRAM_RD | 1 << P_SRAM_WR ); 					//Arduino ctrl pins for SRAM outputs!
	DDRL |= (1<<P_SRAM_BHE | 1<<P_SRAM_BLE); 							//Arduino ctrl pins for SRAM outputs!

	PORTG &= ~(1<<P_A16 | 1<<P_A17);				// set adress line A16, A17 low 
	PORTL &= ~(1<<P_595OE); 						// enable output OE-low 

	pinMode(13, OUTPUT);

	mySerial = &Serial;
	Serial.begin(115200); 		// Start Serial1 communication at 115200 bps

#ifdef DEBUG
		Serial.println("DEBUG-SETUP()");
#endif

}

void loop()
{

	SRAMadr = 0;
	SRAMadr =0x100;
	//set address pins
	setAddress(0);

	size = sizeof(words)/sizeof(unsigned int);

/*
	for (unsigned int i=0; i<size; i++)
	{
		writeSRAM(SRAMadr+i*15,twords[i]);

	}

	for (unsigned int i=0; i<size; i++)
	{
		ret_word = readMEM(SRAMadr+i*15);
		sprintf(buf, "Read %04X [%04X] at address:%06lX\n", ret_word, twords[i], SRAMadr+i*15);
		Serial.write(buf);

	}

	delay(1000);
	exit(0);
*/
	//256 k words
	// loop 4 blocks of 64 k words	
	old_progress =0;
	Serial.println();
	Serial.print("I-----------------------------------------------------------------------------------------------------I\rI");

	for (int ha=0; ha<4; ha++) 
	{
		PORTG =0xFD;		// two lsb bits =0
		PORTG |= ha;		// set A16,A17

		//loop 256 blocks with 256 words
		for (uint16_t block16=0; block16 < 256; block16++)
		{
			//loop 16 groups with 16 words....-> 256 words
			// write and verify 256 words...
			SRAMadr = ha*0x10000 + block16*0x100;
		    crc = 0xFFFF;

			for (uint16_t group16=0; group16<16; group16++)
			{
				// write 
				for (uint16_t j = 0; j<16; j++)
				{
					writeSRAM(SRAMadr + group16*16 + j, words[j]);
			        x = crc >> 8 ^ (words[j]&0xFF);
					x ^= x>>4;
        			crc = (crc << 8) ^ ((unsigned short)(x << 12)) ^ ((unsigned short)(x <<5)) ^ ((unsigned short)x);

				}

			}
			i_crc=crc;

			//verify
		    crc = 0xFFFF;
			for (uint16_t group16=0; group16<16; group16++)
			{
				group = group16;	
				// write 
				for (uint16_t j = 0; j<16; j++)
				{
					ret_word= readMEM(SRAMadr + group16*16 + j);
					// Serial.print(SRAMadr + group16*16 + j,HEX);
					// Serial.print(":");
					// Serial.println(ret_word,HEX);

			        x = crc >> 8 ^ (ret_word&0xFF);
					x ^= x>>4;
        			crc = (crc << 8) ^ ((unsigned short)(x << 12)) ^ ((unsigned short)(x <<5)) ^ ((unsigned short)x);


				}

			}
			f_crc=crc;
			if (f_crc == i_crc)
			{
				progress = (100*(SRAMadr+256)/mem_size);
				if (progress>old_progress)
				{
					old_progress = progress;
					Serial.print("->\b");
				// sprintf(buf,"Progress: %02d \r", (int)progress);
				// Serial.write(buf);

				}
			} 
			else
			{
				sprintf(buf, " \nBig block %d  \nBlock ID %d\n group %d\n", ha, block16, group);
				Serial.write(buf);
			} 
		}
	}
	sprintf(buf,"\nFinished!  Close terminal\n");
	Serial.write(buf);
	delay(500);
	exit(1);
}


int ex_byteCommand()
{
	// Get byte from apparent address (prog counter)
	unsigned int result;
#ifdef DEBUG
		Serial.println("Byte begin");
#endif
	//read act value ProgCounter
	readLong(&startaddress);

	result = readMEM(startaddress);
	mySerial->write((uint8_t)(result & 0XFF));
	mySerial->write((uint8_t)(result >> 8));

#ifdef DEBUG
		Serial.println(startaddress, HEX);
		Serial.println(result, HEX);
		Serial.println("Byte end");
#endif
	//reset_wait_DMA_signals();
	return 0;
}

int ex_pokeCommand()
{

	//print mem contents

#ifdef DEBUG
		Serial.println("Poke begin");
#endif
	readLong(&startaddress);
	readWord(&byteValue);

	sprintf(buf, "store value %04X at address:%06lX\n", byteValue, startaddress);
	mySerial->write(buf);

	if (startaddress < 0x200000)
	// Write SRAM adress (strip away leading 0x50)
	{
		writeSRAM(startaddress, byteValue);
	}
	unsigned int s_Val = readMEM(startaddress);
#ifdef DEBUG
		Serial.println(s_Val, HEX);
#endif

	sprintf(buf, "Verified value %04X at startaddress:%06lX\n", s_Val, startaddress);

#ifdef DEBUG
		Serial.println("Poke end");
#endif
	mySerial->write(buf);
	//reset_wait_DMA_signals();
	return 0;
}

void ex_resetSRAMCommand()
{
	unsigned long seg_pos, seg_pos_ptr;

	//clear mem contents

	readLong(&startaddress);
	readWord(&byteValue);

	sprintf(buf, "Clear memory section from:%4lX,   Size:%4X\n", startaddress, body_size);
	mySerial->write(buf);

	seg_pos = 0;
	while (seg_pos < byteValue)
	{
		seg_pos_ptr = startaddress + seg_pos;

		if (seg_pos_ptr < 0x100000)
		{
			//set address pins,
			writeSRAM(seg_pos_ptr, 0x00);
		}

		seg_pos++;
	}
	mySerial->write("MEM Reset\n");
}





/*
	Read long integer (address) to variable
	*/
int readLong(unsigned long *lg)
{
	return mySerial->readBytes((char *)lg, 4);
}

/*
	Read word integer (double byte, 16b data) to variable
	*/
int readWord(unsigned int *dbdata)
{
	return mySerial->readBytes((char *)dbdata, 2);
}

/*
		Output the address bits and outputEnable signal using shift registers.
	*/
void setAddress(unsigned long address)
{
	//pulse the clear pin in 595 registers
	PORTL &= ~(1 << P_SRCLR);
	PORTL |= (1 << P_SRCLR);

	for (int n = 0; n < 16; n++)		//16 first bits (0 - 15), two additional A16 and A17 via PORTG
	{
		((address >> n) & 0x000001) ? PORTL |= 1 << P_SER : PORTL &= ~(1 << P_SER);
		//pulse the clock pin in 595 registers
		PORTL &= ~(1 << P_SRCLK);
		PORTL |= 1 << P_SRCLK;
		PORTL &= ~(1 << P_SRCLK);
	}

	//pulse the output register pin in 595
	PORTL &= ~(1 << P_RCLK);
	_delay_us(1);
	PORTL |= (1 << P_RCLK);
	// address output ready
}

unsigned int readMEM(unsigned long address)
{
	unsigned int data = 0;
	// set adress pins...

	if (address < 0x400000)
	// Read SRAM adress (strip away leading 0x50)
	{
		setAddress(address & 0X3FFFFF); 	// shift adress lines out
		//#ifdef DEBUG Serial.println("Read SRAM Section");
		PORTB &= ~(1 << P_SRAM_CE | 1 << P_SRAM_RD );
		PORTL &= ~(1 << P_SRAM_BLE | 1 << P_SRAM_BHE | 1<<P_595OE);
		PORTB |= (1 << P_SRAM_WR);
		delayMicroseconds(10);

		data = PINC << 8 | PINA;
		PORTB |= (1 << P_SRAM_CE | 1 << P_SRAM_RD);
		PORTL |= (1 << P_SRAM_BLE | 1 << P_SRAM_BHE );
		delayMicroseconds(10);
	}
// #ifdef DEBUG
// 		 {Serial.println("READ data =");Serial.println(data,HEX);}
// #endif

	return data;
}

/*
		Write a byte to the SRAM at the specified address.
		Adress > $8000
	*/

void writeSRAM(unsigned long address, unsigned int indata)
{
	DDRA = 0xFF; //HIGH data output
	DDRC = 0xFF; //HIGH DATA output

	PORTB |= (1 << P_SRAM_WR );	  //*WE high (init)
	PORTB &= ~(1 << P_SRAM_CE);						  // CE LOW (init)
	PORTL &= ~(1 << P_SRAM_BHE | 1 << P_SRAM_BLE);						  // CE LOW (init)

	//set address pins
	setAddress(address);

	PORTA = indata & 0XFF;
	PORTC = (indata >> 8) & 0XFF;
	delayMicroseconds(6);
	//write pulse Tas
	PORTB &= ~(1 << P_SRAM_WR); 				//WR# (pin 29 low)
	delayMicroseconds(6);
	PORTB |= (1 << P_SRAM_WR);					//WR# high
	PORTB |= (1 << P_SRAM_CE);	  			//CE# (pin 29 high)
	PORTL |= (1 << P_SRAM_BHE | 1 << P_SRAM_BLE);	  //BHE,BLE#  pins  high

	DDRA = 0x00; //LOW datapins input
	DDRC = 0x00; //LOW datapins input


}


// https://stackoverflow.com/questions/9072320/split-string-into-string-array
int Tokenize(String *data, char separator, String _part[])
{

	int found = 0;
	int strIndex[] = {0, -1};
	int maxIndex = data->length() - 1;
	String t;

	for (int i = 0; i <= maxIndex; i++)
	{
		if (data->charAt(i) == ':' || i == maxIndex)
		{

			strIndex[0] = strIndex[1] + 1;
			strIndex[1] = (i == maxIndex) ? i + 1 : i;
			_part[found] = data->substring(strIndex[0], strIndex[1]);
			_part[found].trim();
			found++;
		}
	}

	return found;
}
