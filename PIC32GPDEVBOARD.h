// Code generated by geniopins for PIC32MK1024GPE100 -- DO NOT EDIT

// port initialisation
inline void initIOPins(void) {
	ANSELA   	= 0b0000000000000001;
	TRISACLR 	= 0b0001100000010010;

	ANSELB   	= 0b0000000000000000;
	TRISBCLR 	= 0b0000001010000010;

	ANSELC   	= 0b0000000000000010;
	TRISCCLR 	= 0b0000000000000001;

	ANSELD   	= 0b0000000000000000;
	LATDSET  	= 0b0100000000000000;
	TRISDCLR 	= 0b0100000000000110;

	ANSELE   	= 0b0000001100000000;
	TRISECLR 	= 0b1000000000000001;

	ANSELF   	= 0b0000000000000000;
	CNPDFSET	= 0b0011000000000000;

	ANSELG   	= 0b0000000000000000;
	CNPDGSET	= 0b0000100000000000;
	LATGSET  	= 0b1000000000000000;
	TRISGCLR 	= 0b1111011000000000;

	// input peripheral->pin mappings
	U4RXR	= 0b1101;	// D3
	SDI6R	= 0b0010;	// B0
	C1RXR	= 0b1000;	// E14
	C2RXR	= 0b1100;	// E1
	SDI2R	= 0b1101;	// A14
	U6RXR	= 0b1101;	// A15

	// output pin<-peripheral mappings
	RPG9R	= 0b10001;	// SDO6
	RPA12R	= 0b00010;	// U4TX
	RPA1R	= 0b10001;	// SCK6
	RPB1R	= 0b01100;	// C4TX
	RPC0R	= 0b10001;	// SS6
	RPE15R	= 0b01100;	// C2TX
	RPA4R	= 0b01011;	// U6TX
	RPE0R	= 0b01100;	// C1TX
	RPB7R	= 0b00100;	// SDO2
	RPB9R	= 0b00100;	// SS2
}

// masks for PORTx and LATx[SET|CLR|INV]
enum {
	G_NSDD_CS	= 0x8000,	// G15
	D_DATA_EN	= 0x0002,	// D1
	D_GFX_RESET	= 0x0004,	// D2
	D_UB1_INT	= 0x0010,	// D4
	G_UB1_PWM	= 0x0040,	// G6
	G_LCD_PWM	= 0x0400,	// G10
	A_UB1_RST	= 0x0800,	// A11
	A_UB1_AN	= 0x0001,	// A0
	C_UB2_AN	= 0x0002,	// C1
	G_SWITCH1	= 0x0800,	// G11
	F_SWITCH3	= 0x2000,	// F13
	F_SWITCH2	= 0x1000,	// F12
	D_NOC_CLR	= 0x4000,	// D14
	G_LEDYLW	= 0x4000,	// G14
	G_LEDRED	= 0x1000,	// G12
	G_LEDGRN	= 0x2000,	// G13
};
