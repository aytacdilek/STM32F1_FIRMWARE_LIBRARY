#include "stm32f10x_hd44780.h"

/*
 * @brief:	Initialize the lcd and peripheral.
 * 			This is the outline of what is required:
 * 1 : Wait 20ms for the display to stabilise after power-up
 * 2 : Set RS and ENABLE low
 * 3 : Wait at least 200us
 * 4 : Send nibble cmd 0x03
 * 5 : Wait at least 200us
 * 6 : Send nibble cmd 0x03 again
 * 7 : Wait at least 200us
 * 8 : Send nibble cmd 0x03 a third time
 * 9 : Wait at least 200us
 * 10: Send nibble cmd 0x02 to enable 4-bit mode
 * 11: Wait 5ms
 * 12: Send byte cmd 0x40 (4-bit communications, 2 lines, 5x8)
 * 13: Send byte cmd 0x0c (Turn the display on)
 * 14: Send byte cmd 0x01 (Clear and home the display)
 * 15: Send byte cmd 0x06 (Set left-to-right direction)
 */
void LCD_Init(void) {
	uint8_t i;

	LCD_LowLevelInit();		// Initialize peripherals
	systick_init();			// Initialize systick timer for delay

	LCD_RS_LOW();			// Select Instructor Register
	LCD_EN_LOW();			// Select Enable
	systick_delayMs(80);

	for (i = 0; i < 3; ++i) {
		LCD_SendNibble(0x03);
		systick_delayMs(5);
	}

	LCD_SendNibble(0x02);	// Enable 4-bit mode
	systick_delayMs(20);

	LCD_SendByte(0, (LCD_SET_FUNCTION | LCD_SET_DATA_LENGTH_4 | LCD_SET_NUMBER_OF_LINES_2 | LCD_SET_FONT_SIZE_5X7));	// 4-bit, 2 line, 5x8 mode
	systick_delayMs(5);
	LCD_SendByte(0, (LCD_SET_DISPLAY | LCD_DISPLAY_ON | LCD_CURSOR_ON));		// Turn on display
	systick_delayMs(5);
	LCD_SendByte(0, (LCD_ENTRY_MODE_SET | LCD_CURSOR_INCREMENT));	// Left to Right
	systick_delayMs(5);
	LCD_SendByte(0, LCD_CLEAR_DISPLAY);	// Clear & home display
	systick_delayMs(5);


//	sendCMD(0x02);
//	Delay(0x3FFFC); //wait 20ms
//	sendCMD(0x28); //LCD configs
//	sendCMD(0x06);
//	sendCMD(0x01);
//	sendCMD(0x0E);
//	Delay(0xffff);
}

/*
 * @brief:	Initialize peripheral
 */
void LCD_LowLevelInit(){
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(	LCD_CTRL_EN_GPIO_CLK 	| LCD_CTRL_RS_GPIO_CLK |
							LCD_DATA_BIT_4_GPIO_CLK	| LCD_DATA_BIT_5_GPIO_CLK |
							LCD_DATA_BIT_6_GPIO_CLK | LCD_DATA_BIT_7_GPIO_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin = LCD_CTRL_EN_PIN;
	GPIO_Init(LCD_CTRL_EN_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LCD_CTRL_RS_PIN;
	GPIO_Init(LCD_CTRL_RS_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LCD_DATA_BIT_4_PIN;
	GPIO_Init(LCD_DATA_BIT_4_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LCD_DATA_BIT_5_PIN;
	GPIO_Init(LCD_DATA_BIT_5_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LCD_DATA_BIT_6_PIN;
	GPIO_Init(LCD_DATA_BIT_6_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LCD_DATA_BIT_7_PIN;
	GPIO_Init(LCD_DATA_BIT_7_GPIO_PORT, &GPIO_InitStructure);

	GPIO_SetBits(LCD_CTRL_EN_GPIO_PORT, LCD_CTRL_EN_PIN);
}



/*
 * @brief:	Send 4-bit data(nibble) to display
 */
void LCD_SendNibble(uint8_t n)
{
	GPIO_WriteBit(LCD_DATA_BIT_4_GPIO_PORT, LCD_DATA_BIT_4_PIN, GET_BIT(n, 0));
	GPIO_WriteBit(LCD_DATA_BIT_5_GPIO_PORT, LCD_DATA_BIT_5_PIN, GET_BIT(n, 1));
	GPIO_WriteBit(LCD_DATA_BIT_6_GPIO_PORT, LCD_DATA_BIT_6_PIN, GET_BIT(n, 2));
	GPIO_WriteBit(LCD_DATA_BIT_7_GPIO_PORT, LCD_DATA_BIT_7_PIN, GET_BIT(n, 3));

	systick_delayMs(1);
	LCD_EN_HIGH();
	systick_delayMs(1);
	LCD_EN_LOW();
}



/*
 * @brief:	Send 8-bit data(nibble) to display
 */
void LCD_SendByte(uint8_t address, uint8_t n){
	LCD_RS_LOW();
	systick_delayMs(1);

	if(address){
		LCD_RS_HIGH();
	}
	else{
		LCD_RS_LOW();
	}
	systick_delayMs(1);
	LCD_EN_LOW();

	LCD_SendNibble(n >> 4);
	LCD_SendNibble(n & 0x0f);
}



/*
 * @brief: Clear Display
 */
void LCD_Clear(void)
{
	LCD_SendByte(0, LCD_CLEAR_DISPLAY);
	LCD_SendByte(0, LCD_RETURN_HOME);
}

/*
 * @brief: Select DDRAM Address
 */
void LCD_SetDdramAddress(uint8_t address)
{
	LCD_SendByte(0, address | LCD_SET_DDRAM_ADDRESS);
}

/*
 * @Brief: Select CGRAM address
 */
void LCD_SetCgramAddress(uint8_t address)
{
	LCD_SendByte(0, address | LCD_SET_CGRAM_ADDRESS);
}

/*
 * @brief: Go to the position
 * @param	x: colomn number
 * 			y: row number
 */
void LCD_Goto(uint8_t x, uint8_t y)
{
	uint8_t address;

	if (y == 1) {
		address = 0;
	}
	else if(y == 2){
		address = LCD_LINE_TWO_ADDR;
	}

	address += x - 1;

	LCD_SetDdramAddress(address);
}
void LCD_WriteCustomCharacter(uint8_t num, const uint8_t c[])
{
	uint8_t i;
	if(num < 7)
	{
		LCD_SendByte(0, 0x40 | (num << 3));
		for(i = 0; i < 8; i++)
			LCD_SendByte(1, c[i]);
		LCD_SetDdramAddress(0);
	}
}

void LCD_Putc(char c)
{
	switch (c) {
		case '\a':
			LCD_Goto(1, 1);
			break;
		case '\f':
			LCD_SendByte(0, LCD_CLEAR_DISPLAY);
			systick_delayMs(2);
			break;
		case '\n':
			LCD_Goto(1, 2);
			break;
		case '\b':
			LCD_SendByte(0, 0x10);
			break;
		default:
			LCD_SendByte(1, c);
			break;
	}
}

void LCD_Puts(const char *s)
{
	const char *p = s;
	while(*p != 0)
	{
		LCD_Putc(*p);
		p++;
	}
}

void LCD_PutSignedInt(int32_t n)
{
	char c1[32];

	if(n == 0)
	{
		LCD_Putc('0');
		return;
	}

	signed int value = n;
	unsigned int absolute;

	int i = 0;

	if(value < 0) {
		absolute = -value;
	} else {
		absolute = value;
	}

	while (absolute > 0) {
		c1[i] = '0' + absolute % 10;
		absolute /= 10;
		i++;
	}

	LCD_Putc((value < 0) ? '-' : '+');

	i--;

	while(i >= 0){
		LCD_Putc(c1[i--]);
	}
}

void LCD_PutUnsignedInt(uint32_t n)
{
	char c1[32];
	uint32_t value = n;
	uint32_t i = 0;

	if(n == 0)
	{
		LCD_Putc('0');
		return;
	}

	while (value > 0) {
		c1[i] = '0' + value % 10;
		value /= 10;
		i++;
	}

	while(i-- > 0){
		LCD_Putc(c1[i]);
	}
}

//void LCD_EntryModeCommand(LCD_EntryModeCmdTypeDef *LCD_EntryModeCmdStruct)
//{
//	uint8_t ctrl;
//	ctrl = LCD_PFX_EntryModeSet |
//		   LCD_EntryModeCmdStruct->CursorDirection |
//		   LCD_EntryModeCmdStruct->DisplayShift;
//	LCD_SendByte(0, ctrl);
//}

//void LCD_DisplayOnOffCommand(LCD_DisplayOnOffCmdTypedef *LCD_DisplayOnOffStruct)
//{
//	uint8_t ctrl;
//	ctrl = LCD_PFX_DisplayOnOff |
//		   LCD_DisplayOnOffStruct->BlinkCursor |
//		   LCD_DisplayOnOffStruct->CursorState |
//		   LCD_DisplayOnOffStruct->DisplayState;
//	LCD_SendByte(0, ctrl);
//}

//void LCD_CursorDisplayShiftCommand(LCD_CursorDisplayShiftCmdTypeDef *LCD_CursorDisplayShiftStruct)
//{
//	uint8_t ctrl;
//	ctrl = LCD_PFX_CursorDisplayShift |
//		   LCD_CursorDisplayShiftStruct->MoveOrShift |
//		   LCD_CursorDisplayShiftStruct->ShiftDirection;
//	LCD_SendByte(0, ctrl);
//}




//void strobeEN(void) {
//	Delay(0xffff);
//	GPIO_SetBits(LCD_Port, EN);
//	Delay(0xffff);
//	GPIO_ResetBits(LCD_Port, EN);
//}

//void upNib(uint8_t c) {
//	if(c & 0x80)
//		GPIO_SetBits(LCD_Port, D7);
//	else
//		GPIO_ResetBits(LCD_Port, D7);
//	if(c & 0x40)
//		GPIO_SetBits(LCD_Port, D6);
//	else
//		GPIO_ResetBits(LCD_Port, D6);
//	if(c & 0x20)
//		GPIO_SetBits(LCD_Port, D5);
//	else
//		GPIO_ResetBits(LCD_Port, D5);
//	if(c & 0x10)
//		GPIO_SetBits(LCD_Port, D4);
//	else
//		GPIO_ResetBits(LCD_Port, D4);
//}

//void downNib(uint8_t c) {
//	if(c & 0x8)
//		GPIO_SetBits(LCD_Port, D7);
//	else
//		GPIO_ResetBits(LCD_Port, D7);
//	if(c & 0x4)
//		GPIO_SetBits(LCD_Port, D6);
//	else
//		GPIO_ResetBits(LCD_Port, D6);
//	if(c & 0x2)
//		GPIO_SetBits(LCD_Port, D5);
//	else
//		GPIO_ResetBits(LCD_Port, D5);
//	if(c & 0x1)
//		GPIO_SetBits(LCD_Port, D4);
//	else
//		GPIO_ResetBits(LCD_Port, D4);
//}

//void sendCMD(uint8_t c) {
//	GPIO_ResetBits(LCD_Port, RS);
//	upNib(c);
//	strobeEN();
//	downNib(c);
//	strobeEN();
//}

//void printChar(uint8_t c) {
//	if(((c>=0x20)&&(c<=0x7F)) || ((c>=0xA0)&&(c<=0xFF))) { //check if 'c' is within display boundry
//		GPIO_SetBits(LCD_Port, RS);
//		upNib(c);
//		strobeEN();
//		downNib(c);
//		strobeEN();
//		GPIO_ResetBits(LCD_Port, RS);
//	}
//}

//void printString(uint8_t *s) {
//	uint8_t i=0;
//	while(s[i] != '\0') {
//		printChar(s[i]);
//		i++;
//	}
//}

//void clearLCD(void) {
//	sendCMD(0x01);
//}
//
//void toLine1(void) {
//	sendCMD(0x80);
//}
//
//void toLine2(void) {
//	sendCMD(0xC0);
//}



/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name : Delay
* Description : Inserts a delay time.
* Input : nCount: specifies the delay time length.
* Output : None
* Return : None
* Note : ffff=5mS
*******************************************************************************/
