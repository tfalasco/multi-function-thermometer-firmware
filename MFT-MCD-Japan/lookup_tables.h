/*
 * lookup_tables.h
 *
 *  Created on: Feb 5, 2017
 *      Author: TFalasco
 */

#ifndef LOOKUP_TABLES_H_
#define LOOKUP_TABLES_H_

#include "bg_types.h"

// Declare tables
int16 TCLookupTable[64];
int16 TMLookupTable[81];

// This procedure initializes the values of the thermocouple lookup table
//-----------------------------------------------------------------------
void InitializeThermocoupleTable(void)
{
	// Create TC Lookup Table
	TCLookupTable[0]  = -2920;	//		# -80°C
	TCLookupTable[1]  = -2587;	//		# -70°C
	TCLookupTable[2]  = -2243;	//		# -60°C
	TCLookupTable[3]  = -1889;	//		# -50°C
	TCLookupTable[4]  = -1527;	//		# -40°C
	TCLookupTable[5]  = -1156;	//		# -30°C
	TCLookupTable[6]  =  -778;	//		# -20°C
	TCLookupTable[7]  =  -392;	//		# -10°C
	TCLookupTable[8]  =     0;	//		#   0°C
	TCLookupTable[9]  =   397;	//		#  10°C
	TCLookupTable[10] =   798;	//		#  20°C
	TCLookupTable[11] =  1203;	//		#  30°C
	TCLookupTable[12] =  1612;	//		#  40°C
	TCLookupTable[13] =  2023;	//		#  50°C
	TCLookupTable[14] =  2436;	//		#  60°C
	TCLookupTable[15] =  2851;	//		#  70°C
	TCLookupTable[16] =  3267;	//		#  80°C
	TCLookupTable[17] =  3682;	//		#  90°C
	TCLookupTable[18] =  4096;	//		# 100°C
	TCLookupTable[19] =  4509;	//		# 110°C
	TCLookupTable[20] =  4930;	//		# 120°C
	TCLookupTable[21] =  5328;	//		# 130°C
	TCLookupTable[22] =  5735;	//		# 140°C
	TCLookupTable[23] =  6138;	//		# 150°C
	TCLookupTable[24] =  6540;	//		# 160°C
	TCLookupTable[25] =  6941;	//		# 170°C
	TCLookupTable[26] =  7340;	//		# 180°C
	TCLookupTable[27] =  7739;	//		# 190°C
	TCLookupTable[28] =  8138;	//		# 200°C
	TCLookupTable[29] =  8539;	//		# 210°C
	TCLookupTable[30] =  8940;	//		# 220°C
	TCLookupTable[31] =  9343;	//		# 230°C
	TCLookupTable[32] =  9747;	//		# 240°C
	TCLookupTable[33] = 10153;	//		# 250°C
	TCLookupTable[34] = 10561;	//		# 260°C
	TCLookupTable[35] = 10971;	//		# 270°C
	TCLookupTable[36] = 11382;	//		# 280°C
	TCLookupTable[37] = 11795;	//		# 290°C
	TCLookupTable[38] = 12209;	//		# 300°C
	TCLookupTable[39] = 12624;	//		# 310°C
	TCLookupTable[40] = 13040;	//		# 320°C
	TCLookupTable[41] = 13457;	//		# 330°C
	TCLookupTable[42] = 13874;	//		# 340°C
	TCLookupTable[43] = 14293;	//		# 350°C
	TCLookupTable[44] = 14713;	//		# 360°C
	TCLookupTable[45] = 15133;	//		# 370°C
	TCLookupTable[46] = 15554;	//		# 380°C
	TCLookupTable[47] = 15975;	//		# 390°C
	TCLookupTable[48] = 16397;	//		# 400°C
	TCLookupTable[49] = 16820;	//		# 410°C
	TCLookupTable[50] = 17243;	//		# 420°C
	TCLookupTable[51] = 17667;	//		# 430°C
	TCLookupTable[52] = 18091;	//		# 440°C
	TCLookupTable[53] = 18516;	//		# 450°C
	TCLookupTable[54] = 18941;	//		# 460°C
	TCLookupTable[55] = 19366;	//		# 470°C
	TCLookupTable[56] = 19792;	//		# 480°C
	TCLookupTable[57] = 20218;	//		# 490°C
	TCLookupTable[58] = 20644;	//		# 500°C
	TCLookupTable[59] = 21071;	//		# 510°C
	TCLookupTable[60] = 21497;	//		# 520°C
	TCLookupTable[61] = 21924;	//		# 530°C
	TCLookupTable[62] = 22350;	//		# 540°C
	TCLookupTable[63] = 22776;	//		# 550°C
}

// This procedure initializes the values of the thermistor lookup table
//---------------------------------------------------------------------
void InitializeThermistorTable(void)
{
	// 131°F to -13°F every 1.8°F
	TMLookupTable[0] = 1775;	// 131
	TMLookupTable[1] = 1835;	// 129.2
	TMLookupTable[2] = 1898;	// 127.4
	TMLookupTable[3] = 1963;	// 125.6
	TMLookupTable[4] = 2031;	// 123.8
	TMLookupTable[5] = 2101;	// 122
	TMLookupTable[6] = 2174;	// 120.2
	TMLookupTable[7] = 2249;	// 118.4
	TMLookupTable[8] = 2327;	// 116.6
	TMLookupTable[9] = 2407;	// 114.8
	TMLookupTable[10] = 2491;	// 113
	TMLookupTable[11] = 2578;	// 111.2
	TMLookupTable[12] = 2667;	// 109.4
	TMLookupTable[13] = 2759;	// 107.6
	TMLookupTable[14] = 2855;	// 105.8
	TMLookupTable[15] = 2954;	// 104
	TMLookupTable[16] = 3057;	// 102.2
	TMLookupTable[17] = 3163;	// 100.4
	TMLookupTable[18] = 3273;	// 98.6
	TMLookupTable[19] = 3387;	// 96.8
	TMLookupTable[20] = 3504;	// 95
	TMLookupTable[21] = 3625;	// 93.2
	TMLookupTable[22] = 3750;	// 91.4
	TMLookupTable[23] = 3879;	// 89.6
	TMLookupTable[24] = 4012;	// 87.8
	TMLookupTable[25] = 4149;	// 86
	TMLookupTable[26] = 4291;	// 84.2
	TMLookupTable[27] = 4436;	// 82.4
	TMLookupTable[28] = 4586;	// 80.6
	TMLookupTable[29] = 4741;	// 78.8
	TMLookupTable[30] = 4900;	// 77
	TMLookupTable[31] = 5063;	// 75.2
	TMLookupTable[32] = 5230;	// 73.4
	TMLookupTable[33] = 5404;	// 71.6
	TMLookupTable[34] = 5580;	// 69.8
	TMLookupTable[35] = 5761;	// 68
	TMLookupTable[36] = 5948;	// 66.2
	TMLookupTable[37] = 6138;	// 64.4
	TMLookupTable[38] = 6333;	// 62.6
	TMLookupTable[39] = 6533;	// 60.8
	TMLookupTable[40] = 6736;	// 59
	TMLookupTable[41] = 6946;	// 57.2
	TMLookupTable[42] = 7158;	// 55.4
	TMLookupTable[43] = 7373;	// 53.6
	TMLookupTable[44] = 7592;	// 51.8
	TMLookupTable[45] = 7816;	// 50
	TMLookupTable[46] = 8045;	// 48.2
	TMLookupTable[47] = 8275;	// 46.4
	TMLookupTable[48] = 8510;	// 44.6
	TMLookupTable[49] = 8747;	// 42.8
	TMLookupTable[50] = 8986;	// 41
	TMLookupTable[51] = 9228;	// 39.2
	TMLookupTable[52] = 9472;	// 37.4
	TMLookupTable[53] = 9719;	// 35.6
	TMLookupTable[54] = 9967;	// 33.8
	TMLookupTable[55] = 10216;	// 32
	TMLookupTable[56] = 10466;	// 30.2
	TMLookupTable[57] = 10717;	// 28.4
	TMLookupTable[58] = 10969;	// 26.6
	TMLookupTable[59] = 11220;	// 24.8
	TMLookupTable[60] = 11471;	// 23
	TMLookupTable[61] = 11722;	// 21.2
	TMLookupTable[62] = 11971;	// 19.4
	TMLookupTable[63] = 12219;	// 17.6
	TMLookupTable[64] = 12465;	// 15.8
	TMLookupTable[65] = 12710;	// 14
	TMLookupTable[66] = 12952;	// 12.2
	TMLookupTable[67] = 13192;	// 10.4
	TMLookupTable[68] = 13428;	// 8.6
	TMLookupTable[69] = 13661;	// 6.8
	TMLookupTable[70] = 13890;	// 5
	TMLookupTable[71] = 14116;	// 3.2
	TMLookupTable[72] = 14337;	// 1.4
	TMLookupTable[73] = 14555;	// -0.4
	TMLookupTable[74] = 14767;	// -2.2
	TMLookupTable[75] = 14975;	// -4
	TMLookupTable[76] = 15178;	// -5.8
	TMLookupTable[77] = 15375;	// -7.6
	TMLookupTable[78] = 15568;	// -9.4
	TMLookupTable[79] = 15755;	// -11.2
	TMLookupTable[80] = 15937;	// -13



}

#endif /* LOOKUP_TABLES_H_ */
