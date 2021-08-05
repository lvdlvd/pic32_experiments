package main

/* PIC32MKxxxxGPC100  PIC32MKxxxxGPE100
PIC32MK GP/MC Family Datasheet DS60001402G  p 5-6
TABLE 5: PIN NAMES FOR 100-PIN GENERAL PURPOSE (GPD/GPE) DEVICES
*/
var PIC32MKxxxxGPE100 = []string{
	/*1*/ "AN23/CVD23/PMA23/RG15",
	/*2*/ "VDD",
	/*3*/ "TCK/RPA7/PMD5/RA7",
	/*4*/ "RPB14/VBUSON1/PMD6/RB14",
	/*5*/ "RPB15/PMD7/RB15",
	/*6*/ "RD1",
	/*7*/ "RD2",
	/*8*/ "RPD3/RD3",
	/*9*/ "RPD4/RD4",
	/*10*/ "AN19/CVD19/RPG6/VBUSON2/PMA5/RG6",
	/*11*/ "AN18/CVD18/RPG7/SCL1/PMA4/RG7",
	/*12*/ "AN17/CVD17/RPG8/SDA1/PMA3/RG8",
	/*13*/ "MCLR",
	/*14*/ "AN16/CVD16/RPG9/PMA2/RG9",
	/*15*/ "VSS",
	/*16*/ "VDD",
	/*17*/ "AN22/CVD22/RG10",
	/*18*/ "AN21/CVD21/RE8",
	/*19*/ "AN20/CVD20/RE9",
	/*20*/ "AN10/CVD10/RPA12/RA12",
	/*21*/ "AN9/CVD9/RPA11/RA11",
	/*22*/ "OA2OUT/AN0/C2IN4-/C4IN3-/RPA0/RA0",
	/*23*/ "OA2IN+/AN1/C2IN1+/RPA1/RA1",
	/*24*/ "PGD3/OA2IN-/AN2/C2IN1-/RPB0/CTED2/RB0",
	/*25*/ "PGC3/OA1OUT/AN3/C1IN4-/C4IN2-/RPB1/CTED1/RB1",
	/*26*/ "PGC1/OA1IN+/AN4/C1IN1+/C1IN3-/C2IN3-/RPB2/RB2",
	/*27*/ "PGD1/OA1IN-/AN5/CTCMP/C1IN1-/RTCC/RPB3/RB3",
	/*28*/ "VREF-/AN33/CVD33/PMA7/RF9",
	/*29*/ "VREF+/AN34/CVD34/PMA6/RF10",
	/*30*/ "AVDD",
	/*31*/ "AVSS",
	/*32*/ "OA3OUT/AN6/CVD6/C3IN4-/C4IN1+/C4IN4-/RPC0/RC0",
	/*33*/ "OA3IN-/AN7/CVD7/C3IN1-/C4IN1-/RPC1/RC1",
	/*34*/ "OA3IN+/AN8/CVD8/C3IN1+/C3IN3-/RPC2/PMA13/RC2",
	/*35*/ "AN11/CVD11/C1IN2-/PMA12/RC11",
	/*36*/ "VSS",
	/*37*/ "VDD",
	/*38*/ "AN35/CVD35/RG11",
	/*39*/ "AN36/CVD36/RF13",
	/*40*/ "AN37/CVD37/RF12",
	/*41*/ "AN12/CVD12/C2IN2-/C5IN2-/SDA4/PMA11/RE12",
	/*42*/ "AN13/CVD13/C3IN2-/SCL4/PMA10/RE13",
	/*43*/ "AN14/CVD14/RPE14/PMA1/RE14",
	/*44*/ "AN15/CVD15/RPE15/PMA0/RE15",
	/*45*/ "VSS",
	/*46*/ "VDD",
	/*47*/ "AN38/CVD38/RD14",
	/*48*/ "AN39/CVD39/RD15",
	/*49*/ "TDI/DAC3/AN26/CVD26/RPA8/SDA2/PMA9/RA8",
	/*50*/ "RPB4/SCL2/PMA8/RB4",
	/*51*/ "OA5IN+/DAC1/AN24/CVD24/C5IN1+/C5IN3-/RPA4/T1CK/RA4",
	/*52*/ "AN40/CVD40/RPE0/RE0",
	/*53*/ "AN41/CVD41/RPE1/RE1",
	/*54*/ "VBUS1",
	/*55*/ "VUSB3V3",
	/*56*/ "D1",
	/*57*/ "D1",
	/*58*/ "VBUS2",
	/*59*/ "D2",
	/*60*/ "D2",
	/*61*/ "AN45/CVD45/RF5",
	/*62*/ "VDD",
	/*63*/ "OSCI/CLKI/AN49/CVD49/RPC12/RC12",
	/*64*/ "OSCO/CLKO/RPC15/RC15",
	/*65*/ "VSS",
	/*66*/ "AN46/CVD46/RPA14/RA14",
	/*67*/ "AN47/CVD47/RPA15/RA15",
	/*68*/ "VBAT",
	/*69*/ "PGD2/RPB5/SDA3/USBID1/RB5",
	/*70*/ "PGC2/RPB6/SCL3/SCK2/PMA15/RB6",
	/*71*/ "DAC2/AN48/CVD48/RPC10/PMA14/PMCS/RC10",
	/*72*/ "OA5OUT/AN25/CVD25/C5IN4-/RPB7/SCK1/INT0/RB7",
	/*73*/ "SOSCI/RPC13/RC13",
	/*74*/ "SOSCO/RPB8/RB8",
	/*75*/ "VSS",
	/*76*/ "TMS/OA5IN-/AN27/CVD27/LVDIN/C5IN1-/RPB9/RB9",
	/*77*/ "RPC6/USBID2/PMA16/RC6",
	/*78*/ "RPC7/PMA17/RC7",
	/*79*/ "PMD12/RD12",
	/*80*/ "PMD13/RD13",
	/*81*/ "RPC8/PMWR/RC8",
	/*82*/ "RPD5/PMRD/RD5",
	/*83*/ "RPD6/PMD14/RD6",
	/*84*/ "RPC9/PMD15/RC9",
	/*85*/ "VSS",
	/*86*/ "VDD",
	/*87*/ "RPF0/PMD11/RF0",
	/*88*/ "RPF1/PMD10/RF1",
	/*89*/ "RPG1/PMD9/RG1",
	/*90*/ "RPG0/PMD8/RG0",
	/*91*/ "TRCLK/PMA18/RF6",
	/*92*/ "TRD3/PMA19/RF7",
	/*93*/ "RPB10/PMD0/RB10",
	/*94*/ "RPB11/PMD1/RB11",
	/*95*/ "TRD2/PMA20/RG14",
	/*96*/ "TRD1/RPG12/PMA21/RG12",
	/*97*/ "TRD0/PMA22/RG13",
	/*98*/ "RPB12/PMD2/RB12",
	/*99*/ "RPB13/CTPLS/PMD3/RB13",
	/*100*/ "TDO/PMD4/RA10",
}
