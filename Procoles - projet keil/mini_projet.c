#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>


/**************** LCD - PCF8574A***************/
#define PCF8574_I2C_ADDRESS 0x20
#define RS_PIN 0 // Po
#define RW_PIN 1 // P1
#define E_PIN  2 // P2
#define D4_PIN 4 // P4
#define D5_PIN 5 // P5
#define D6_PIN 6 // P6
#define D7_PIN 7 // P7
#define BACKLIGHT_PIN 3 

/*********** ACCEL I2C ***********/
#define ADXL_ADDR_R 0XD0
#define ADXL_ADDR_W 0XD1
#define ACCEL_XOUT_H 0X3B
#define ACCEL_XOUT_L 0X3C
#define ACCEL_YOUT_H 0X3D
#define ACCEL_YOUT_L 0X3E
#define ACCEL_ZOUT_H 0X3F
#define ACCEL_ZOUT_L 0X40
#define SENSITIVITY 16384
const char ACCEl_ADDR = 0X68;
/************ SPI DPS310 **************/
#define SPI1_ENABLE_BIT  12
#define PRS_B2 0X00
#define PRS_B1 0X01l
#define PRS_B0 0X02
#define MAX_VALUE_24BITS 16777215
/**********************************/
/**********************PROTOTYPES***************************/
void delay(int ms);

int UART2_init (void);
int UART1_init (void);
void Display_DATA_UART(void);

void SPIConfig (void);
void GPIOConfig (void);
void SPI_Enable (void);
void CS_Enable (void);
void CS_Disable (void);
void adxl_write (uint8_t address, uint8_t value);
void adxl_read (uint8_t address);
void adxl_init (void);

void I2C_init (void);
void I2C_send(uint8_t address, uint8_t data_i2c);
int i2c_Read(uint8_t *data_i2c,char slave_address,char register_address);
void get_accel(void);
void PCF8574_Write(uint8_t data);
void LCD_Init(void);
void LCD_SendCommand(uint8_t command);
void LCD_SendData(uint8_t data);
void LCD_Print(char *str);
void LCD_clear(void);
void LCD_put_cursor(int row, int col);
void Display_DATA_LCD(void);

int compareStrings(const char *string1, const char *string2);
/**************************************************/
/*   old functions
void SPI_init(void);
void SPI_Read(uint8_t address, uint8_t rxdata_i2c);
void SPI_Read(uint8_t address, uint8_t *SDO);
void SPI_Read_burst(uint8_t address, uint32_t *pressure);
void get_pressure(void);
void get_pressure_burst(void);
uint32_t mapValue(uint32_t value);
*/
	
/*******************VARIABLES PROG**********************/
char fdata [100]; //buffer d'envoi UART
int Update_flag = 0; // NOUVELLE REQUETE, 1 = UPDATE
uint32_t DPS_pressure;
uint32_t DPS_pressure_mapped;

char Incomming_command[100];		// Buffer de réception UART
char UART_command[] = "UPDATE";	//Commande UPDATE
int new_command_ready = 0;			// Nouveau message UART

/******** Variables programme SPI *********/
float xg, yg, zg; //accel en g
int16_t x,y,z;		// raw accel
uint8_t RxData[6]; // buffer de réception SPI accel

uint8_t data[3] = {0x02, 0x32, 0xf6}; //config ADXL345

/****************** VAR I2C *********************/
uint8_t data_i2c;
int16_t accel_xout;		// raw accel
int16_t accel_yout;		// raw accel
int16_t accel_zout;		// raw accel
float accel_xout_g;		//accel en g
float accel_yout_g;		//accel en g
float accel_zout_g;		//accel en g


int main (void)
{
/************************ INITS ****************************/
	GPIOConfig();
	delay(100);
	I2C_init();
	delay(100);
	SPIConfig();
	SPI_Enable();
	UART2_init();
	delay(100);
	
	adxl_init();
	LCD_Init(); 
	delay(100);
	LCD_Print("AYY PEPITO !!!!!!");
	delay(5000);
	LCD_put_cursor(1,0);
	LCD_Print("Waiting for RQ");
	while(1)
	{
		/*********************** Réception UART ************************************/
		if(USART2->SR & 0X20){ // bus not ready
			int index = 0;
			while(index<6){
				if(USART2->SR & 0X40){ //Data in buffer
					/****************** Lecture du buffer UART **********************/
					Incomming_command[index] = USART2 -> DR;
					USART2 -> DR = Incomming_command[index];
					index++;
				}
			}
			delay(1);
			USART2 -> DR = '\r';
			delay(1);
			Incomming_command[index] = '\0';		// ajout fin de chaine
			new_command_ready = 1;
		}
		if(new_command_ready){
			/************* Test --> comparaison avec commandes et gestion des flag **************/
			int result = compareStrings(UART_command, Incomming_command);
			if (result == 0) {
				Update_flag = 1;	
			} 
			else if (result != 0) {
			/*********************  COMMANDE NON RECONNUE *******************************/
				USART2 -> DR = 'E';
				delay(1);
				USART2 -> DR = 'R';
				delay(1);
				USART2 -> DR = 'R';
				delay(1);
				USART2 -> DR = 'O';
				delay(1);
				USART2 -> DR = 'R';
				delay(1);
				USART2 -> DR = '\r';
				delay(1);
			}
			new_command_ready = 0;
		}
		/***************************  AFFICHAGE DES DONNEES  ***************************************/
		if(Update_flag){
			/* On récupère les nouvelles données en 1er*/
			/************ SPI ****************/
			adxl_read (0x32);		
			/********************* I2C ************************/
			get_accel();
			/******************* Print data on UART *************************/
			Display_DATA_UART();
			/*************************** Display data LCD *******************************/
			Display_DATA_LCD();
			/*************************** RESET - on vide le buffer et on reset le flag *******************************/
			memset(Incomming_command,'0',sizeof(Incomming_command));
			Update_flag = 0;
		}
	}
}

void GPIOConfig (void){
	GPIOA->MODER = 0;
	/*********** CLOCK ENABLE PORTS USED ***************/
	RCC->AHB1ENR |=  1;	// enable GPIOA clock 
	RCC->AHB1ENR |=  2;  // Enable GPIOB clock
	
	/**********************UART****************************/
	
	RCC->APB1ENR |= 0X20000;
	//alternate 5-4 bit 1-0
	GPIOA -> MODER |= 0xA0; 
	GPIOA -> MODER &=~ 0x50;
	//GPIOA->MODER |= (2<<4)|(2<<6);
	//enable alternate
	GPIOA -> AFR[0] |= 0X7700;    //(7<<8)|(7<<12);//
	
	
	/******************** SPI ***********************/
	//SPI MISO MOSI CLK CS
	GPIOA->MODER |= (2<<10)|(2<<12)|(2<<14)|(1<<18);  // Alternate functions PA5, PA6, PA7 and Output for PA9
	GPIOA->OSPEEDR |= (3<<10)|(3<<12)|(3<<14)|(3<<18);  // HIGH Speed for PA5, PA6, PA7, PA9
	GPIOA->AFR[0] |= (5<<20)|(5<<24)|(5<<28);   // AF5(SPI1) for PA5, PA6, PA7
	
	
	/*********** I/O config ****************/
  //GPIOA->MODER &=~ 0x00000C00;    // clear pin mode 
  //GPIOA->MODER |=  0x00000400;    // set pin to output mode */
	
	
	/********* I2C pin config *************/
	//I2C1_SCL => PB8
	//I2C1_SDA, => PB9
	// I2C1 est sur AF04
	GPIOB->AFR[1] |= 0x44;
	GPIOB->MODER |= (1<<19) |(1<<17); // Alternate functions
	GPIOB->MODER &=~(1<<18) |(1<<16); // Alternate functions
	
	GPIOB->OTYPER |= (1<<9) |(1<<8); // open drain
	
	GPIOB->PUPDR |= (1<<18) |(1<<16); // Pull-up
	GPIOB->PUPDR &=~ (1<<19)|(1<<17); // Pull-up

	/************/
}


/******************** UART ********************/
int UART1_init(void) {

    // Configure USART1
    USART1->CR1 = 0;             // Disable USART1 for configuration
    USART1->CR1 |= USART_CR1_TE;  // Enable transmitter (TX)
    USART1->CR1 |= USART_CR1_RE;  // Enable receiver (RX)
    USART1->CR1 &= ~USART_CR1_M;  // Set word length to 8 bits
    USART1->CR2 &= ~USART_CR2_STOP;  // Set stop bits to 1
    USART1->CR3 &= ~USART_CR3_HDSEL;  // Set half-duplex mode
    USART1->BRR = 417;           // Baud rate for 9600 (assuming 16MHz clock)
    
    // Enable USART1
    USART1->CR1 |= USART_CR1_UE;

    // Send data
    while (!(USART1->SR & USART_SR_TXE));  // Wait until the transmit buffer is empty
    USART1->DR = 'O';

    return 0;
}


int UART2_init (void){
	//config usart TX
	USART2->CR1 |= 0X2000;
	USART2->CR1 &=~ 0X1000;
	USART2->CR1 |= 0X8;
	USART2->CR2 &=~ 0X3000;
	USART2->CR3 &=~ 0X80;
	USART2->BRR |= 0X0683;
	//config usart RX
	USART2->CR1 |= 0X4;
	
	
	//send data
	USART2 -> DR = 'J';
	return 0;
}

/******************** Update USART2 ************************/
void Display_DATA_UART(void){
	/************************* Display SPI --> UART**********************************************/
	memset(fdata,'0',sizeof(fdata));		// reset du buffer
	sprintf(fdata, "SPI ACCEL DATA -------->  Xg = %.2f   ||   Yg = %.2f   ||   Zg = %.2f",xg,yg,zg); //formatage
	for (int i = 0; fdata[i] != '\0'; i++) {
		USART2->DR = fdata[i];
		delay(10);
	}
	USART2 -> DR = '\r';
	/********************** Display I2C --> UART*************************************/	
	memset(fdata,'0',sizeof(fdata));
	sprintf(fdata, "I2C ACCEL DATA -------->  Xg = %.2f   ||   Yg = %.2f   ||   Zg = %.2f",accel_xout_g,accel_yout_g, accel_zout_g);
	for (int i = 0; fdata[i] != '\0'; i++) {
		USART2->DR = fdata[i];
		delay(10);
	}
	USART2 -> DR = '\r';
}

/***************** I2C *******************************/
void I2C_init (void){
	// Enable the I2C1 peripheral clock
	RCC->APB1ENR |= (1<<21); //I2C clock EN
	
	//software reset
	I2C1 -> CR1 |= (1<<15);
	I2C1 -> CR1 &=~ (1<<15);
	
	//Peripheral input clock
	I2C1 -> CR2 |= 0x10; //16MHz
	
	//=> Mode standard (SM) 100kHz 
	/*
		Système d'équations :
		=> Valeurs p.105/149 en ns
		T_r(SCL)=1000
		T_w(SCLH)=4000
		T_f(SCL)=300
		T_w(SCLL)=4700
		T_high=T_r(SCL)+T_w(SCLH)=5000
		T_low=T_f(SCL)+T_w(SCLL)=5000
		TPCLK1=1/16MHz=62.5 n => clock du microcontroleur
		T_high = CCR * TPCLK1
		T_low = CCR * TPCLK1
		CRR=80
	*/
	I2C1->CCR |= 80; 
	/* 
		p.503/844: I2C TRISE register (I2C_TRISE)
		TRISE= (T_r(SCL)/T_PCLK1)+1
		TRISE=(1000/62.5)+1=17
	*/
	I2C1->TRISE |= 17;
	// Enable the I2C1 peripheral
	I2C1->CR1 |= (1<<0); // Set the PE (Peripheral Enable) bit in CR1
}


void I2C_send(uint8_t address, uint8_t data_i2c) {// read single byte data
	while (I2C1->SR2 & (1 << 1));
	I2C1->CR1 |= (1<<8); //start cond
	while(!(I2C1->SR1 & (1<<0))) {}
	// Send the 7-bit slave address with LSB set for write operation
  I2C1->DR = (address << 1) | 0x00;
	// Wait for the address to be sent
  while (!(I2C1->SR1 & (1 << 1))); // Wait for ADDR (Address) flag to be set
  // Clear ADDR flag by reading SR1 and SR2
	// type volatile --> variable pas optimisé en compilation donc détection de la variation du flag certaine
  volatile uint32_t tmp = I2C1 -> SR1;
  tmp = I2C1 -> SR2;
	// send data by writing on dr
	while (!(I2C1->SR1 & (1 << 7))); // Wait for Tx1 rdy
	I2C1 -> DR = data_i2c;
	I2C1 ->CR1 |= (1<<9);
}

int i2c_Read(uint8_t *data_i2c,char slave_address,char register_address){
	volatile uint8_t temp;
	/*Set the START bit in the I2C_CR1 register to generate a Start condition*/
	while (I2C1->SR2 & (1 << 1));
	I2C1->CR1 |=1<<8; // condition du  start
	while (!(I2C1->SR1 & 1<<0)){} // Tant que le bit SB est à 1, on attends!
	I2C1->DR = (slave_address << 1) | 0x00; // Transmission de l'adresse de l'esclave + write
		/*Dès que l'adresse est transmise ADDR est mis à 1*/
		//1: Received address matched.	
	while(!(I2C1->SR1 &(1<<1))){} //on attend ADDR=1
	//temp = I2C1 -> SR1;
	temp = I2C1-> SR2; // Clear ADDR flag: le flag est remis à zero après lecture
			
	while(!(I2C1->SR1 &(1<<7))){} // attendre que TXE=1	
	I2C1->DR = register_address;
			
	while(!(I2C1->SR1 &(1<<7))){} // attendre que TXE=1

		
		/* Conditiopn de Restart*/	
	I2C1->CR1 |=1<<8; // condition du  start
	while (!(I2C1->SR1 & 1<<0)){} // Tant que le bit SB est à 1, on attends!
	temp=I2C1->SR1; // Clear SB flag:  le flag est remis à zero après lecture		
	I2C1->DR = (slave_address << 1)| 0x01; // Transmission de l'adresse de l'esclave + Read incoming data	
		
	while(!(I2C1->SR1 &(1<<1))){} //on attend ADDR=1
	I2C1->CR1 &=~ (1<<10); //  NACK
	temp=I2C1->SR2; // Clear ADDR flag: le flag est remis à zero après lecture
		
	I2C1->CR1 |= (1<<9); // condition de stop après reception de la data
	
	while(!(I2C1->SR1 &(1<<6))){} 
	*data_i2c = I2C1->DR; // Lecture de la donnée dans DR
		
	return 0;
}





/***************** LCD I2C*******************************/
void LCD_Init(void) {
	// 4 bit initialisation
	delay(50);
	LCD_SendCommand (0x30);
	delay(5);  
	LCD_SendCommand (0x30);
	delay(1);  
	LCD_SendCommand (0x30);
	delay(10);
	LCD_SendCommand (0x20);  
	delay(10);

  // dislay initialisation
	LCD_SendCommand (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	delay(1);
	LCD_SendCommand (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	delay(1);
	LCD_SendCommand (0x01);  // clear display
	delay(1);
	delay(1);
	LCD_SendCommand (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	delay(1);
	LCD_SendCommand (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}
void PCF8574_Write(uint8_t data) {
    I2C_send(PCF8574_I2C_ADDRESS, data);
}
void LCD_SendCommand(uint8_t command){
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (command&0xf0);
	data_l = ((command<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	for(int i = 0;i<4;i++){
		PCF8574_Write(data_t[i]);
	}
}
void LCD_SendData(uint8_t data){
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=1
	data_t[1] = data_u|0x09;  //en=0, rs=1
	data_t[2] = data_l|0x0D;  //en=1, rs=1
	data_t[3] = data_l|0x09;  //en=0, rs=1
	for(int i = 0;i<4;i++){
		PCF8574_Write(data_t[i]);
	}
}
void LCD_Print(char *str){
	while (*str) LCD_SendData (*str++);
}
void LCD_clear(void){
	LCD_SendCommand(0x80);
	for(int i = 0; i<70;i++){
		LCD_SendData(' ');
	}
}
void LCD_put_cursor(int row, int col){
	switch(row){
		case 0:
			col |= 0x80;
			break;
		case 1:
			col |= 0xC0;
			break;
	}
	LCD_SendCommand(col);
}
/******************** update LCD ************************/
void Display_DATA_LCD(void){
	/************************* Display SPI **********************************************/
	memset(fdata,'0',sizeof(fdata));
	sprintf(fdata, "SPI ACCEL X|Y|Z");
	LCD_clear();
	LCD_put_cursor(0,0);
	LCD_Print(fdata);
	
	memset(fdata,'0',sizeof(fdata));
	sprintf(fdata, "%.1f||%.1f||%.1f",xg,yg,zg);
	LCD_put_cursor(1,0);
	LCD_Print(fdata);
	/**********************************************/
	delay(3000);
	/********************** Display I2C *************************************/	
	memset(fdata,'0',sizeof(fdata));
	sprintf(fdata, "I2C ACCEL X|Y|Z");
	LCD_clear();
	LCD_put_cursor(0,0);
	LCD_Print(fdata);
	
	memset(fdata,'0',sizeof(fdata));
	sprintf(fdata, "%.1f||%.1f||%.1f",accel_xout_g,accel_yout_g, accel_zout_g);
	LCD_put_cursor(1,0);
	LCD_Print(fdata);
/**********************************************/
	delay(3000);	
}

/*

void LCD_Clear(void) {
    LCD_SendCommand(0x01); // Clear display
    delay(2); // Delay after clearing the display
}

void LCD_Init(void) {
    // Initialization commands for the LCD
    delay(50);  // Wait for power-up

    // 4-bit initialization sequence
    PCF8574_Write(0b00000000);
    delay(10);
    PCF8574_Write(0b00000010);  // Send the second 4 bits (DL=0, N=1, F=0)
    delay(10);

    LCD_SendCommand(0x28); // 4-bit mode, 2 lines, 5x8 font
    delay(5);
    LCD_SendCommand(0x0C); // Display ON, Cursor OFF, Blink OFF
    delay(5);
    LCD_SendCommand(0x06); // Entry mode: Increment cursor position, no display shift
    
    LCD_Clear(); // Clear the display

    // Turn on backlight after initialization
    PCF8574_Write(1 << BACKLIGHT_PIN);
}



void LCD_SendCommand(uint8_t command) {
    // Send high nibble
    PCF8574_Write((command & 0xF0) | (1 << RS_PIN) | (0 << RW_PIN) | (1 << E_PIN));
    delay(1);
    PCF8574_Write((command & 0xF0) | (1 << RS_PIN) | (0 << RW_PIN) | (1 << E_PIN) | (1 << D4_PIN));
    delay(1);
    PCF8574_Write((command & 0xF0) | (1 << RS_PIN) | (0 << RW_PIN) | (1 << E_PIN));
    delay(1);

    // Send low nibble
    PCF8574_Write(((command & 0x0F) << 4) | (1 << RS_PIN) | (0 << RW_PIN) | (1 << E_PIN));
    delay(1);
    PCF8574_Write(((command & 0x0F) << 4) | (1 << RS_PIN) | (0 << RW_PIN) | (1 << E_PIN) | (1 << D4_PIN));
    delay(1);
    PCF8574_Write(((command & 0x0F) << 4) | (1 << RS_PIN) | (0 << RW_PIN) | (1 << E_PIN));
    delay(1);

    delay(2);  // Extra delay for command execution
}


void LCD_SendData(uint8_t data) {
    // Send high nibble
    PCF8574_Write((data & 0xF0) | 0x05);  // RS=1
    delay(1);
    PCF8574_Write((data & 0xF0) | 0x01 | 0x04);  // Enable pulse
    delay(1);
    PCF8574_Write((data & 0xF0) | 0x05 | 0x04);  // Clear enable

    // Send low nibble
    PCF8574_Write(((data & 0x0F) << 4) | 0x05);  // RS=1
    delay(1);
    PCF8574_Write(((data & 0x0F) << 4) | 0x01 | 0x04);  // Enable pulse
    delay(1);
    PCF8574_Write(((data & 0x0F) << 4) | 0x05 | 0x04 | 0x08);  // Clear enable

    delay(2);  // Extra delay for data execution

}

void LCD_Print(char *str, int size) {
    for (int i = 0; i < size; i++) {
        LCD_SendData(str[i]);
        delay(100);
    }
		PCF8574_Write(1 << BACKLIGHT_PIN);
}
*/

/**********************  MPU9250 ***************************************/
void get_accel(void){
	/*             Lecture MPU 9250
	*  6 registres à lire
	*	 + formatage
	*/
	i2c_Read(&data_i2c,ACCEl_ADDR, ACCEL_XOUT_H);
	accel_xout = data_i2c<<8;
	i2c_Read(&data_i2c,ACCEl_ADDR, ACCEL_XOUT_L);
	accel_xout |= data_i2c;
	i2c_Read(&data_i2c,ACCEl_ADDR, ACCEL_YOUT_H);
	accel_yout = data_i2c<<8;
	i2c_Read(&data_i2c,ACCEl_ADDR, ACCEL_YOUT_L);
	accel_yout |= data_i2c;
	i2c_Read(&data_i2c,ACCEl_ADDR, ACCEL_ZOUT_H);
	accel_zout = data_i2c<<8;
	i2c_Read(&data_i2c,ACCEl_ADDR, ACCEL_ZOUT_L);
	accel_zout |= data_i2c;
	
	// Convert raw data to G-force
	accel_xout_g = (float)(accel_xout) / SENSITIVITY;
	accel_yout_g = (float)(accel_yout) / SENSITIVITY;
	accel_zout_g = (float)(accel_zout) / SENSITIVITY;
}


/************************  Fonction contrôle SPI  ****************************************/
void SPIConfig (void){
	RCC->APB2ENR |= (1<<12);  // Enable SPI1 CLock
	SPI1->CR1 = 0;
	SPI1->CR1 |= (1<<0)|(1<<1);   // CPOL=1, CPHA=1
	SPI1->CR1 |= (1<<2);  // Master Mode
	SPI1->CR1 |= (1<<4);  // BR[2:0] = 011: fPCLK/16, PCLK = 16MHz, SPI clk = 1MHz
	SPI1->CR1 &= ~(1<<7);  // LSBFIRST = 0, MSB first
	SPI1->CR1 |= (1<<8) | (1<<9);  // SSM=1, SSi=1 -> Software Slave Management
	SPI1->CR1 &= ~(1<<10);  // RXONLY = 0, full-duplex
	SPI1->CR1 &= ~(1<<11);  // DFF=0, 8 bit data
	SPI1->CR2 = 0;
}



void SPI_Enable (void){
	SPI1->CR1 |= (1<<6);   // SPE=1, Peripheral enabled
}

void SPI_Disable (void){
	SPI1->CR1 &= ~(1<<6);   // SPE=0, Peripheral Disabled
}

void CS_Enable (void){
	GPIOA->BSRR |= (1<<9)<<16;
}

void CS_Disable (void){
	GPIOA->BSRR |= (1<<9);
}

void SPI_Transmit (uint8_t *data, int size){
	int i=0;
	while (i<size){
	   while (!((SPI1->SR)&(1<<1))) {};  // wait for TXE bit to set ->  buffer is empty
	   SPI1->DR = data[i];  // load the data into the Data Register
	   i++;
	}	
	while (!((SPI1->SR)&(1<<1))) {};  // wait for TXE bit to set -> Buffer is empty
	while (((SPI1->SR)&(1<<7))) {};  // wait for BSY bit to Reset -> SPI is not busy 
	
	//  Clear the Overrun flag by reading DR and SR
	uint8_t temp = SPI1->DR;
					temp = SPI1->SR;
	
}

void SPI_Receive (uint8_t *data, int size){
	while (size){
		while (((SPI1->SR)&(1<<7))) {};  // wait for BSY bit to Reset -> SPI is not busy in communication
		SPI1->DR = 0;  // send dummy data
		while (!((SPI1->SR) &(1<<0))){};  // Wait for RXNE to set -> This will indicate that the Rx buffer is not empty
	  *data++ = (SPI1->DR);
		size--;
	}	
}
	

/************************ Gestion ADXL345 ***********************************/
void adxl_init (void){ //init de l'adxl 345
	adxl_write (0x31, 0x00);  // data_format range= +- 2g
	adxl_write (0x2d, 0x00);  // reset all bits
	adxl_write (0x2d, 0x08);  // power_cntl measure and wake up 8hz
}
void adxl_write (uint8_t address, uint8_t value){
	uint8_t data[2];
	data[0] = address|0x40;  // multibyte write
	data[1] = value;
	CS_Enable ();  // pull the cs pin low
	SPI_Transmit (data, 2);  // write data to register
	CS_Disable ();  // pull the cs pin high
}

void adxl_read (uint8_t address){
	address |= 0x80;  // read operation
	address |= 0x40;  // multibyte read
	CS_Enable ();  // pull the pin low
	SPI_Transmit (&address, 1);  // send address
	SPI_Receive (RxData, 6);  // receive 6 bytes data
	CS_Disable ();;  // pull the pin high
	
	x = (int16_t)(RxData[1]<<8|RxData[0]);
	y = (int16_t)(RxData[3]<<8|RxData[2]);
	z = (int16_t)(RxData[5]<<8|RxData[4]);

	xg = (float)x/256;
	yg = (float)y/256;
	zg = (float)z/256;
}

/*****************************  AUTRE FONCTIONS  ***************************************/
int compareStrings(const char *string1, const char *string2) {
	/******************************  RENVOIE 0 SI IDENTIQUE / RETURN int si diff  ********************************************************/
    while (*string1 != '\0' && *string2 != '\0') {
        if (*string1 != *string2) {
            return *string1 - *string2;
        }
        string1++;
        string2++;
    }
    return 0;
}

void delay(int ms){
	int i;
	for(;ms>0;ms--)
		for(i=0;i<3195;i++);
}

uint32_t mapValue(uint32_t value) {
    uint32_t inMin = 0;
    uint32_t inMax = 0xFFFFFF; // Maximum 24-bit value
    uint32_t outMin = 300;
    uint32_t outMax = 1200;

    uint64_t mappedValue = ((uint64_t)(value - inMin) * (outMax - outMin)) / (inMax - inMin) + outMin;

    if (mappedValue > outMax) {
        mappedValue = outMax;
    }

    return (uint32_t)mappedValue;
}

/*****************************************************************************/

