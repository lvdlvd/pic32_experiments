package main

/* PIC32MK GP/MC Family Datasheet DS60001402G
   page 247..250 TABLE 13-1: INPUT PIN SELECTION

   <signal>R = <pin>
*/

var imux = []struct {
	signals string
	pins    map[string]uint
}{
	{ // Set 1
		signals: "INT4 T2CK T6CK IC4 IC7 IC12 IC15 U3RX U4CTS U6RX SDI1 SDI3 SCK4 SDI5 SS6 QEA1 HOME2 QAEA3 HOME4 QEA5 HOME6 FLT1 C3RX REFCLKI",
		pins: map[string]uint{
			"A0":  0b0000,
			"B3":  0b0001,
			"B4":  0b0010,
			"B15": 0b0011,
			"B7":  0b0100,
			"C7":  0b0101,
			"C0":  0b0110,
			"A11": 0b1000,
			"D5":  0b1001,
			"G6":  0b1010,
			"F1":  0b1011,
			"E0":  0b1100,
			"A15": 0b1101,
		},
	},
	{ // Set 2
		signals: "INT3 T3CK T7CK IC3 IC8 IC11 IC16 U1CTS U2RX U5CTS SDI2 SDI4 SCK6 QEB1 INDX2 QEB3 INDX4 QEB5 INDX6 C2RX FLT2",
		pins: map[string]uint{
			"A1":  0b0000,
			"B5":  0b0001,
			"B1":  0b0010,
			"B11": 0b0011,
			"B8":  0b0100,
			"A8":  0b0101,
			"C8":  0b0110,
			"B12": 0b0111,
			"A12": 0b1000,
			"D6":  0b1001,
			"G7":  0b1010,
			"G0":  0b1011,
			"E1":  0b1100,
			"A14": 0b1101,
		},
	},
	{ // Set 3
		signals: "INT2 T4CK T8CK IC1 IC5 IC9 IC13 U1RX U2CTS U5RX SS1 SS3 SS4 SS5 INDX1 QEB2 INDX3 QEB4 INDX5 QEB6 C1RX OCFB",
		pins: map[string]uint{
			"B6":  0b0000,
			"C15": 0b0001,
			"A4":  0b0010,
			"B13": 0b0011,
			"B2":  0b0100,
			"C6":  0b0101,
			"C1":  0b0110,
			"A7":  0b0111,
			"E14": 0b1000,
			"C13": 0b1001,
			"G8":  0b1010,
			"F0":  0b1100,
			"D4":  0b1101,
		},
	},
	{ // Set 4
		signals: "INT1 T5CK T9CK IC2 IC6 IC10 IC14 U3CTS U4RX U6CTS SS2 SCK3 SCK5 SDI6 HOME1 QEA2 HOME3 QEA4 HOME5 QEA6 C4RX OCFA",
		pins: map[string]uint{
			"B14": 0b0000,
			"C12": 0b0001,
			"B0":  0b0010,
			"B10": 0b0011,
			"B9":  0b0100,
			"C9":  0b0101,
			"C2":  0b0110,
			"E15": 0b1000,
			"C10": 0b1001,
			"G9":  0b1010,
			"G12": 0b1011,
			"G1":  0b1100,
			"D3":  0b1101,
		},
	},
}

/*
  page 252..255 TABLE 13-2: OUTPUT PIN SELECTION

	RP<pin>R <-  <signal>
*/

var omux = []struct {
	pins    string
	signals map[string]uint
}{
	{ // Set 1
		pins: "A0 B3 B4 B15 B7 C7 C0 A11 D5 G6 F1 E0 A15",
		signals: map[string]uint{
			//"Off":      0b00000,
			"U1TX":     0b00001,
			"U2RTS":    0b00010,
			"SDO1":     0b00011,
			"SDO2":     0b00100,
			"OCI":      0b00101,
			"OC7":      0b00110,
			"C2OUT":    0b00111,
			"C4OUT":    0b01000,
			"OC13":     0b01001,
			"U5RTS":    0b01011,
			"C1TX":     0b01100,
			"SDO3":     0b01110,
			"SCK4":     0b01111,
			"SDO5":     0b10000,
			"SS6":      0b10001,
			"REFCLKO4": 0b10010,
			"QEICMP1":  0b10100,
			"QEICMP5":  0b10101,
		},
	},
	{ // Set 2
		pins: "A1 B5 B1 B11 A8 C8 B12 A12 D6 G7 G0 E1 A14",
		signals: map[string]uint{
			//"Off":      0b00000,
			"U3RTS":    0b00001,
			"U4TX":     0b00010,
			"SDO1":     0b00011,
			"SDO2":     0b00100,
			"OC2":      0b00101,
			"OC8":      0b00110,
			"C3OUT":    0b00111,
			"OC9":      0b01000,
			"OC12":     0b01001,
			"OC16":     0b01010,
			"U6RTS":    0b01011,
			"C4TX":     0b01100,
			"SDO3":     0b01110,
			"SDO4":     0b01111,
			"SDO5":     0b10000,
			"SCK6":     0b10001,
			"REFCLKO3": 0b10010,
			"QEICMP2":  0b10100,
			"QEICMP6":  0b10101,
		},
	},
	{ // Set 3
		pins: "B6 C15 A4 B13 B2 C6 C1 A7 E14 G8 F0 D4",
		signals: map[string]uint{
			//"Off":      0b00000,
			"U3TX":     0b00001,
			"U4RTS":    0b00010,
			"SS1":      0b00011,
			"OC4":      0b00101,
			"OC5":      0b00110,
			"REFCLKO1": 0b00111,
			"C5OUT":    0b01000,
			"OC10":     0b01001,
			"OC14":     0b01010,
			"U6TX":     0b01011,
			"C3TX":     0b01100,
			"SS3":      0b01110,
			"SS4":      0b01111,
			"SS5":      0b10000,
			"SDO6":     0b10001,
			"REFCLKO2": 0b10010,
			"QEICMP3":  0b10100,
		},
	},
	{ // Set 4
		pins: "B14 C12 B0 B10 B9 C9 C2 E15 C10 G9 G12 G1 D3",
		signals: map[string]uint{
			"Off":     0b00000,
			"U1RTS":   0b00001,
			"U2TX":    0b00010,
			"SS2":     0b00100,
			"OC3":     0b00101,
			"OC6":     0b00110,
			"C1OUT":   0b00111,
			"OC11":    0b01001,
			"OC15":    0b01010,
			"U5TX":    0b01011,
			"C2TX":    0b01100,
			"SCK3":    0b01110,
			"SDO4":    0b01111,
			"SCK5":    0b10000,
			"SDO6":    0b10001,
			"CTPLS":   0b10010,
			"QEICMP4": 0b10100,
		},
	},
}
