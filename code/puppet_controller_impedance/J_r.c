/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * J_r.c
 *
 * Code generation for function 'J_r'
 *
 */

/* Include files */
#include "J_r.h"
#include "puppet_controller_4_cg_rtwutil.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Declarations */
static void ft_1(const double ct[317], double b_J_r[48]);

/* Function Definitions */
static void ft_1(const double ct[317], double b_J_r[48])
{
  double b_ct_tmp;
  double b_t1045_tmp;
  double b_t1069_tmp;
  double b_t1087_tmp;
  double b_t601_tmp;
  double b_t963_tmp;
  double c_ct_tmp;
  double c_t1045_tmp;
  double c_t1087_tmp;
  double ct_tmp;
  double d_ct_tmp;
  double t1011;
  double t1013;
  double t1015;
  double t1017;
  double t1024_tmp;
  double t1026;
  double t1031;
  double t1037;
  double t1037_tmp;
  double t1038;
  double t1039;
  double t1039_tmp;
  double t1041;
  double t1044;
  double t1045_tmp;
  double t1045_tmp_tmp;
  double t1050;
  double t1051;
  double t1052;
  double t1053;
  double t1059;
  double t1059_tmp;
  double t1060;
  double t1069_tmp;
  double t1070;
  double t1071_tmp;
  double t1079;
  double t1083;
  double t1087_tmp;
  double t1088;
  double t1088_tmp;
  double t1105;
  double t1106;
  double t1106_tmp;
  double t1107;
  double t1114;
  double t1117;
  double t1119;
  double t1121;
  double t1121_tmp;
  double t1123;
  double t1123_tmp;
  double t1124;
  double t1126;
  double t1126_tmp;
  double t1127;
  double t1127_tmp;
  double t1130;
  double t1131;
  double t1132;
  double t1132_tmp;
  double t1133;
  double t1134;
  double t1136;
  double t1136_tmp;
  double t1137;
  double t1140;
  double t1140_tmp;
  double t1141;
  double t1142;
  double t1143;
  double t1144;
  double t1144_tmp;
  double t1147;
  double t1148;
  double t1149;
  double t1150;
  double t1151;
  double t1154;
  double t1154_tmp;
  double t1160;
  double t1160_tmp;
  double t444;
  double t445;
  double t456;
  double t456_tmp;
  double t527;
  double t565;
  double t579;
  double t594;
  double t594_tmp;
  double t596;
  double t596_tmp;
  double t597;
  double t597_tmp;
  double t598;
  double t598_tmp;
  double t601_tmp;
  double t621;
  double t623;
  double t625;
  double t627;
  double t630;
  double t632;
  double t636;
  double t637;
  double t638;
  double t640;
  double t650_tmp;
  double t657_tmp;
  double t676;
  double t678;
  double t679;
  double t683;
  double t684;
  double t685;
  double t696;
  double t697;
  double t698;
  double t699;
  double t701;
  double t702;
  double t707_tmp;
  double t707_tmp_tmp;
  double t709;
  double t711;
  double t728;
  double t729;
  double t731;
  double t732;
  double t733;
  double t735;
  double t735_tmp;
  double t737;
  double t742;
  double t750;
  double t754;
  double t755;
  double t757;
  double t760;
  double t761;
  double t766;
  double t767;
  double t773;
  double t774;
  double t775;
  double t776;
  double t777;
  double t779;
  double t783;
  double t787_tmp;
  double t793;
  double t797;
  double t802;
  double t806_tmp;
  double t810;
  double t814;
  double t814_tmp;
  double t814_tmp_tmp;
  double t815;
  double t816;
  double t816_tmp;
  double t816_tmp_tmp;
  double t817;
  double t819;
  double t820;
  double t831;
  double t835;
  double t840;
  double t840_tmp;
  double t841;
  double t841_tmp;
  double t842;
  double t845;
  double t847;
  double t850;
  double t851;
  double t854;
  double t856;
  double t857;
  double t859;
  double t860;
  double t861_tmp;
  double t863;
  double t864;
  double t869_tmp;
  double t876_tmp;
  double t878_tmp;
  double t878_tmp_tmp;
  double t879;
  double t887_tmp;
  double t893;
  double t896_tmp;
  double t899;
  double t901;
  double t903;
  double t904;
  double t914;
  double t915;
  double t918;
  double t919;
  double t931;
  double t932;
  double t933;
  double t935;
  double t935_tmp;
  double t937;
  double t938;
  double t940;
  double t942;
  double t943;
  double t952;
  double t952_tmp;
  double t955;
  double t963;
  double t963_tmp;
  double t969_tmp;
  double t972;
  double t974;
  double t975;
  double t976;
  double t979;
  double t989;
  double t991;
  double t993;
  double t993_tmp;
  double t994;
  double t994_tmp;
  double t995;
  double t997;
  double t999;
  t625 = ct[216] + ct[237];
  t632 = ((ct[19] + ct[65]) + ct[75]) + ct[264];
  t640 = ((ct[14] + ct[69]) + ct[73]) + ct[272];
  t683 = (ct[153] + ct[184]) + ct[216];
  t742 = (((ct[80] + ct[92]) + ct[94]) + ct[178]) + ct[185];
  t754 = ((ct[84] + ct[89]) + ct[242]) + ct[250];
  t766 = ct[292] * ct[300];
  t819 = (((((ct[88] + ct[97]) + ct[102]) + ct[123]) + ct[125]) + ct[135]) +
         ct[243];
  t831 = (((((ct[119] + ct[123]) + ct[127]) + ct[128]) + ct[133]) + ct[136]) +
         ct[182];
  t444 = ct[2] * ct[190];
  t445 = ct[6] * ct[190];
  t456_tmp = ct[2] * ct[6];
  t456 = t456_tmp * ct[138] * ct[202] * ct[181];
  t527 = ct[98] * ct[239];
  t565 = ct[267] * ct[292];
  t579 = ct[98] * ct[276];
  t594_tmp = ct[287] * ct[298];
  t594 = t594_tmp * ct[277];
  t596_tmp = ct[3] * ct[217] * ct[306];
  t596 = t596_tmp * ct[277];
  t597_tmp = ct[7] * ct[217] * ct[306];
  t597 = t597_tmp * ct[277];
  t893 = ct[3] * ct[7];
  t598_tmp = t893 * ct[288] * ct[298];
  t598 = t598_tmp * ct[277];
  t601_tmp = t893 * ct[138] * ct[217];
  b_t601_tmp = t601_tmp * ct[294];
  t621 = ct[193] + ct[215];
  t627 = ct[292] * t625;
  t630 = ((ct[18] + ct[61]) + ct[66]) + ct[265];
  t638 = ((ct[19] + ct[60]) + ct[64]) + ct[273];
  t650_tmp = ct[2] * ct[284] * ct[297];
  t657_tmp = t456_tmp * ct[202] * ct[305];
  t676 = (ct[140] + ct[181]) + ct[193];
  t684 = ct[98] * t683;
  t685 = ct[292] * t683;
  t683 = ct[152] + ct[204];
  t696 = t683 + ct[245];
  t697 = (ct[166] + ct[205]) + ct[238];
  t698 = t683 + ct[246];
  t699 = (ct[173] + ct[205]) + ct[238];
  t728 = ((((ct[79] + ct[91]) + ct[93]) + ct[118]) + ct[122]) + ct[131];
  t750 = ct[98] * t742;
  t755 = ct[98] * t754;
  t760 = ((((ct[85] + ct[90]) + ct[114]) + ct[124]) + ct[126]) + ct[130];
  t773 = ((ct[100] + ct[104]) + ct[249]) + ct[253];
  t797 = ((((ct[87] + ct[96]) + ct[101]) + ct[121]) + ct[132]) + ct[180];
  t810 = ((((ct[120] + ct[121]) + ct[129]) + ct[132]) + ct[134]) + ct[142];
  t820 = ct[292] * t819;
  t878_tmp_tmp = ct[21] - ct[46];
  t878_tmp = ((((ct[171] + ct[174]) + ct[221]) + ct[224]) + ct[259]) -
             ct[2] * ct[202] * ct[305] * t878_tmp_tmp;
  t879 = ct[292] * t878_tmp;
  t995 = (((((ct[150] + ct[172]) + ct[175]) + ct[203]) + ct[225]) + ct[229]) +
         ct[275];
  t1013 = (((((ct[203] + ct[226]) + ct[230]) + ct[235]) + ct[240]) + ct[244]) +
          ct[256];
  t623 = ct[292] * t621;
  t636 = t527 + ct[255];
  t637 = ct[239] * ct[292] + ct[257];
  t678 = ct[98] * t676;
  t679 = ct[292] * t676;
  t701 = ct[292] * t696;
  t702 = ct[98] * t698;
  t707_tmp_tmp = ct[6] * ct[211];
  t707_tmp = (-ct[157] + t445) + t707_tmp_tmp;
  t709 = -ct[3] * ct[7] * ct[288] * ct[298] * t707_tmp;
  t711 = -ct[99] * ct[288] * ct[298] * t707_tmp;
  t729 = t594_tmp * t699;
  t731 = t596_tmp * t699;
  t732 = t597_tmp * t699;
  t733 = t598_tmp * t699;
  t735_tmp = ct[99] * ct[288] * ct[298];
  t735 = t735_tmp * t699;
  t737 = ct[98] * t728;
  t757 = ct[98] * ct[267] + ct[276] * ct[292];
  t761 = ct[98] * t760;
  t787_tmp = t565 - t579;
  t793 = t598_tmp * t787_tmp;
  t802 = ct[292] * t797;
  t806_tmp = ct[7] * ct[287] * ct[298];
  t816_tmp_tmp = ct[55] + ct[48] * (ct[22] - ct[45]);
  t816_tmp = ct[204] + ct[293] * t816_tmp_tmp;
  t816 = -ct[292] * t816_tmp + ct[98] * t625;
  t817 = ct[98] * t816_tmp + t627;
  t931 = ((((ct[197] + ct[212]) + ct[222]) - ct[221]) + ct[241]) + ct[247];
  t932 = ct[292] * t742 + ct[98] * ct[300];
  t935_tmp = ((ct[86] - ct[95]) + ct[248]) - ct[254];
  t935 = t755 + -ct[292] * t935_tmp;
  t937 = (((((ct[149] + ct[158]) + ct[169]) + t456) - ct[191]) + ct[233]) +
         ct[252];
  t963_tmp = ((ct[16] + ct[62]) - ct[70]) - ct[262];
  b_t963_tmp = (ct[282] - t650_tmp * t632) + ct[293] * t963_tmp;
  t963 = -ct[98] * b_t963_tmp;
  t993_tmp = ((ct[18] - ct[63]) + ct[72]) - ct[271];
  t993 = (ct[280] + t650_tmp * t640) + ct[28] * t993_tmp;
  t994_tmp = ct[6] * ct[284] * ct[297];
  t994 = (ct[281] + t994_tmp * t640) + t657_tmp * t993_tmp;
  t999 = ct[292] * t995;
  t1015 = ((((((ct[151] + ct[159]) + ct[170]) + ct[190]) + ct[206]) - ct[195]) +
           ct[232]) +
          ct[258];
  t1024_tmp = t893 * ct[217] * ct[306];
  t1026 = ((((((ct[168] + ct[189]) + ct[190]) + ct[207]) - ct[196]) + ct[211]) +
           ct[219]) +
          ct[236];
  t1045_tmp = ct[3] * ct[287] * ct[298];
  b_t1045_tmp = ct[284] * ct[297];
  t742 = ct[138] * ct[202];
  t893 = t456_tmp * ct[286] * ct[297];
  t1045_tmp_tmp = ((((ct[155] - ct[138] * ct[205]) + ct[231]) +
                    b_t1045_tmp * t878_tmp_tmp) +
                   t742 * ct[311] * t878_tmp_tmp) +
                  t893 * t816_tmp_tmp;
  c_t1045_tmp = t879 - ct[98] * t1045_tmp_tmp;
  t767 = ct[294] * t757;
  t774 = t594_tmp * t757;
  t777 = t598_tmp * t757;
  t676 = -ct[287] * ct[298];
  t779 = ct[3] * (t676 * t787_tmp);
  t814_tmp_tmp = ct[51] - ct[110];
  t814_tmp = t444 + ct[293] * t814_tmp_tmp;
  t814 = -ct[292] * t814_tmp + ct[98] * t621;
  t815 = ct[98] * t814_tmp + t623;
  t683 = -ct[144] + t444;
  t625 = ct[202] * ct[314];
  t840_tmp = t683 + t625 * t814_tmp_tmp;
  t840 = t678 + -ct[292] * t840_tmp;
  t841_tmp = t683 + ct[28] * t814_tmp_tmp;
  t841 = t678 + -ct[292] * t841_tmp;
  t842 = t679 + ct[98] * t840_tmp;
  t850 = t685 + ct[98] * t696;
  t851 = t685 + t702;
  t896_tmp = t684 - ct[292] * t698;
  t698 = -ct[294] * t896_tmp;
  t903 = t676 * t896_tmp;
  t914 = t1045_tmp * t896_tmp;
  t915 = t806_tmp * t896_tmp;
  t918 = t598_tmp * t896_tmp;
  t938 = ct[292] * t728 + ct[98] * t797;
  t942 = ct[292] * t937;
  t952_tmp = ((ct[14] - ct[53]) + ct[59]) - ct[266];
  t952 = (t994_tmp * ct[270] + -ct[293] * t952_tmp) + t650_tmp * t630;
  t969_tmp = ((ct[16] - ct[52]) + ct[58]) - ct[274];
  t696 = (t994_tmp * ct[261] + -ct[293] * t969_tmp) + t650_tmp * t638;
  t975 = (t657_tmp * ct[270] + t650_tmp * t952_tmp) + t625 * t630;
  t683 = ct[202] * ct[313];
  t976 = (t683 * ct[270] + t657_tmp * t630) + t994_tmp * t952_tmp;
  t621 = (t657_tmp * ct[261] + t650_tmp * t969_tmp) + t625 * t638;
  t989 = (t683 * ct[261] + t657_tmp * t638) + t994_tmp * t969_tmp;
  t997 = ct[292] * t993;
  t1011 = ct[292] * t760 + ct[98] * t819;
  t1017 = ct[292] * t1015;
  t683 = t742 * ct[308];
  t1059_tmp = ((((ct[160] + ct[227]) + ct[138] * ct[204]) +
                b_t1045_tmp * t816_tmp_tmp) +
               t683 * t816_tmp_tmp) +
              t893 * t878_tmp_tmp;
  t1059 = -ct[292] * t1059_tmp + ct[98] * t995;
  t1060 = ct[98] * t1059_tmp + t999;
  t1087_tmp = -ct[3] * ct[287] * ct[298];
  b_t1087_tmp = (-ct[283] + ct[28] * t632) + t650_tmp * t963_tmp;
  c_t1087_tmp = t963 + ct[292] * b_t1087_tmp;
  t1144_tmp = ct[98] * t878_tmp + ct[292] * t1045_tmp_tmp;
  t1144 = (t806_tmp * t1013 + ct[294] * c_t1045_tmp) + t1045_tmp * t1144_tmp;
  t775 = ct[3] * t774;
  t776 = ct[7] * t774;
  t783 = t601_tmp * t767;
  t676 =
      ((((ct[139] + ct[154]) + ct[156]) + ct[210]) + ct[213]) + ct[138] * -t445;
  t845 = ct[294] * t840;
  t847 = ct[294] * t842;
  t760 = t684 - t701;
  t854 = t594_tmp * t840;
  t857 = t598_tmp * t840;
  t859 = t594_tmp * t842;
  t861_tmp = ct[56] * ct[288] * ct[298];
  t864 = t598_tmp * t842;
  t742 = ct[138] * ct[217];
  t869_tmp = t742 * ct[309];
  t876_tmp = ct[217] * ct[316];
  t887_tmp = ct[3] * ct[220];
  t893 = ct[294] * t851;
  t899 = t594_tmp * t851;
  t901 = t598_tmp * t851;
  t904 = t861_tmp * t851;
  t919 = t601_tmp * t698;
  t940 = t737 - t802;
  t943 = (t1024_tmp * ct[251] + t1045_tmp * t636) + ct[29] * t637;
  t955 = ct[98] * t952;
  t972 = ct[98] * t696;
  t974 = t779 + ct[29] * t757;
  t979 = ct[292] * t975;
  t991 = ct[292] * t621;
  t625 = (ct[7] * t594 + t767) + t779;
  t1039_tmp = ct[218] - t994_tmp * t816_tmp_tmp;
  t1039 = (t806_tmp * t1039_tmp + ct[294] * t817) + t1045_tmp * t816;
  t1069_tmp = ((((ct[143] - ct[148]) + ct[208]) + ct[138] * t444) +
               b_t1045_tmp * t814_tmp_tmp) +
              t683 * t814_tmp_tmp;
  b_t1069_tmp = t1017 + ct[98] * t1069_tmp;
  t1071_tmp = ct[98] * t1015 - ct[292] * t1069_tmp;
  t1088_tmp = (ct[278] - ct[293] * t640) + t650_tmp * t993_tmp;
  t1088 = ct[98] * t1088_tmp + t997;
  t1123_tmp = t750 - t766;
  t1123 = (t806_tmp * ct[301] + -ct[294] * t1123_tmp) + t1045_tmp * t932;
  t1124 = (t1024_tmp * ct[301] + t1045_tmp * t1123_tmp) + ct[29] * t932;
  t1126_tmp = ct[292] * t754 + ct[98] * t935_tmp;
  t1126 = (t1024_tmp * t773 + t1045_tmp * t935) + ct[29] * t1126_tmp;
  t835 = ct[98] * t676;
  t728 = ct[3] * t854;
  t856 = ct[7] * t854;
  t860 = ct[3] * t859;
  t863 = t601_tmp * t845;
  t683 = ct[3] * t899;
  t933 = (t806_tmp * ct[251] - ct[294] * t636) + t1045_tmp * t637;
  t1031 = ct[304] * t625;
  t1037_tmp = ct[198] - t994_tmp * t814_tmp_tmp;
  t1037 = (t806_tmp * t1037_tmp + ct[294] * t815) + t1045_tmp * t814;
  t1041 = (t1024_tmp * t1039_tmp - t1045_tmp * t817) + ct[29] * t816;
  t1050 = t893 + t914;
  t1070 = t955 + t979;
  t1079 = t972 + t991;
  t797 = (ct[7] * t731 + t914) + ct[29] * t851;
  t1117 = (ct[31] * t699 + t1024_tmp * t851) + t915;
  t1132_tmp = t761 - t820;
  t1132 = (t806_tmp * t810 + -ct[294] * t1132_tmp) + t1045_tmp * t1011;
  t1133 = (t1024_tmp * t810 + t1045_tmp * t1132_tmp) + ct[29] * t1011;
  t1134 = (((((-t597 + ct[34] * ct[277]) + ct[44] * ct[277]) + t774) +
            t869_tmp * t757) -
           t861_tmp * t757) +
          -ct[145] * t787_tmp;
  t1137 = ((((-t731 + ct[33] * t699) + ct[43] * t699) + t918) + t919) +
          ct[109] * t851;
  t1149 = (-(t806_tmp * t931) + ct[294] * t1060) + t1045_tmp * t1059;
  t1151 = (((((-t732 + ct[34] * t699) + ct[44] * t699) + t903) +
            t861_tmp * t896_tmp) +
           -ct[138] * ct[217] * ct[309] * t896_tmp) +
          ct[145] * t851;
  t1038 = (t1024_tmp * t1037_tmp - t1045_tmp * t815) + t876_tmp * t814;
  t1044 = t847 + t728;
  t1051 = t683 + -ct[29] * t896_tmp;
  t1052 = ct[292] * t676 + ct[98] * t937;
  t1053 = t835 - t942;
  t1083 = ct[292] * t696 - ct[98] * t621;
  t1105 = (ct[7] * (t596_tmp * t707_tmp) + t728) + t876_tmp * t842;
  t1106_tmp = ct[217] * ct[315];
  t1106 = (t1106_tmp * t707_tmp + t856) + t1024_tmp * t842;
  t683 += ct[7] * t729 + t698;
  t621 = (t1024_tmp * t697 + t1045_tmp * t760) + t876_tmp * t850;
  t1119 = ct[161] * t797;
  t1127_tmp = (-(ct[7] * t596) + t775) + ct[29] * t787_tmp;
  t1127 = ct[161] * t625 + -ct[304] * t1127_tmp;
  t1131 = (t1024_tmp * t831 + t1045_tmp * t940) + ct[29] * t938;
  t1136_tmp = t742 * ct[312];
  t1136 = ((((t729 - t735) + t1136_tmp * t699) - t901) + t601_tmp * t893) +
          ct[138] * t915;
  t1143 = ((((-t733 + b_t601_tmp * t699) + t899) - t904) + t869_tmp * t851) +
          ct[138] * t914;
  t1150 = (t1024_tmp * t931 + t1045_tmp * t1060) - ct[29] * t1059;
  t1160_tmp = ct[98] * t993 - ct[292] * t1088_tmp;
  t1160 = (t806_tmp * t994 + ct[294] * t1088) + t1045_tmp * t1160_tmp;
  t1107 = ct[161] * t1105;
  t676 = (t806_tmp * t697 - ct[294] * t760) + t1045_tmp * t850;
  t1114 = ct[304] * t683;
  t1121_tmp = t735_tmp * ct[277];
  t1121 = ((((t594 + t1136_tmp * ct[277]) - t1121_tmp) +
            t601_tmp * (-ct[294] * t787_tmp)) +
           t793) +
          ct[138] * -t776;
  t1130 = (t806_tmp * t831 - ct[294] * t940) + t1045_tmp * t938;
  t1140_tmp = t861_tmp * t842;
  t1140 = ((((t709 + b_t601_tmp * t707_tmp) + t859) + ct[138] * t728) +
           t869_tmp * t842) -
          t1140_tmp;
  t1154_tmp = ct[292] * t952 - ct[98] * t975;
  t1154 = (t806_tmp * t976 + ct[294] * t1070) + t1087_tmp * t1154_tmp;
  t742 = (t1024_tmp * t994 - t1045_tmp * t1088) + ct[29] * t1160_tmp;
  t1141 = ct[161] * t683 + ct[304] * t797;
  t1142 = t1114 - t1119;
  t1148 = (t1024_tmp * t1026 + t1045_tmp * t1053) + t876_tmp * t1052;
  t683 = (-(t1024_tmp * t989) + t1045_tmp * t1079) + t876_tmp * t1083;
  t1147 = (t806_tmp * t1026 - ct[294] * t1053) + t1045_tmp * t1052;
  t625 = (t806_tmp * t989 + ct[294] * t1079) - t1045_tmp * t1083;
  ct_tmp = t679 + ct[98] * t841_tmp;
  t779 = (-ct[165] + t445) + t707_tmp_tmp;
  b_t1045_tmp = ct[0] * ct[13];
  t1015 = (-(ct[294] * t841) + t1045_tmp * ct_tmp) + t806_tmp * t779;
  t444 = (t1045_tmp * t841 + ct[29] * ct_tmp) + t1024_tmp * t779;
  t650_tmp = ct[9] * ct[161];
  t754 = ct[9] * ct[304];
  t731 = ct[0] * ct[23];
  t893 = t731 * ct[289] * ct[299];
  t937 = ct[0] * ct[10];
  t915 = ct[0] * ct[17];
  t914 = t915 * ct[287] * ct[298];
  t729 = ct[0] * ct[8];
  t993 = t729 * ct[23];
  t952 = t993 * ct[228] * ct[307];
  t975 = ct[0] * ct[4];
  t594 = t975 * ct[23];
  t707_tmp_tmp = t594 * ct[228] * ct[307];
  t445 = ct[0] * ct[7] * ct[17];
  b_ct_tmp = t445 * ct[217] * ct[306];
  c_ct_tmp = ct[9] * ct[292];
  d_ct_tmp = -ct[0] * ct[3] * ct[17] * ct[217] * ct[306];
  b_J_r[0] = (((((((((((-ct[9] * ct[50] - ct[9] * ct[111]) - ct[9] * t678) +
                      c_ct_tmp * t841_tmp) +
                     t650_tmp * t1015) +
                    t754 * t444) +
                   b_t1045_tmp * ct[141]) -
                  b_t1045_tmp * ct[190]) +
                 b_t1045_tmp * ct[199]) +
                t937 * ct[183] * ct[187] * ct[303]) -
               t914 * t841) +
              (t893 * (ct[161] * t1015 + ct[304] * t444) +
               t707_tmp_tmp * (ct[304] * t1015 - ct[161] * t444))) +
             ((d_ct_tmp * ct_tmp - b_ct_tmp * t779) -
              t952 * ((ct[31] * t779 + t806_tmp * t841) + t1024_tmp * ct_tmp));
  t878_tmp = ct[0] * ct[3];
  t598_tmp = t878_tmp * ct[17];
  ct_tmp = t598_tmp * ct[217] * ct[306];
  b_J_r[1] =
      ((((((((((((((ct[9] * ct[54] + ct[9] * ct[113]) + ct[9] * t684) +
                  ct[9] * -t701) +
                 b_t1045_tmp * ct[150]) +
                b_t1045_tmp * ct[203]) +
               b_t1045_tmp * ct[235]) -
              t650_tmp * t676) -
             t754 * t621) -
            t893 * (ct[161] * t676 + ct[304] * t621)) -
           t937 * ct[187] * ct[263] * ct[303]) +
          t914 * t760) +
         t952 * ((t1106_tmp * t697 + t806_tmp * t760) + t1024_tmp * t850)) +
        t707_tmp_tmp * (ct[161] * t621 - ct[304] * t676)) +
       ct_tmp * t850) +
      b_ct_tmp * t697;
  b_J_r[2] = 0.0;
  t779 = b_t1045_tmp * ct[284] * ct[297];
  t1015 = ct[1] * ct[10];
  t444 = t937 * ct[26];
  t696 = ct[0] * ct[2];
  t698 = t696 * ct[13];
  t728 = t698 * ct[202] * ct[305];
  t797 = ct[0] * ct[6] * ct[13];
  t760 = t797 * ct[202];
  t819 = t760 * ct[305];
  t995 = ct[24] + ct[47];
  b_J_r[3] = (((((((((((((((((t995 + ct[9] * ct[69]) + ct[9] * ct[73]) +
                            ct[9] * ct[272]) +
                           ct[9] * (-ct[98] * t1088_tmp)) -
                          ct[9] * t997) -
                         t650_tmp * t1160) -
                        t754 * t742) -
                       t893 * (ct[161] * t1160 + ct[304] * t742)) +
                      t779 * t640) -
                     t914 * t1088) +
                    t1015 * ct[263] * ct[279] * ct[296]) +
                   t444 * ct[194] * ct[263] * ct[303]) +
                  t952 * ((ct[31] * t994 - t806_tmp * t1088) +
                          t1024_tmp * t1160_tmp)) +
                 ct_tmp * t1160_tmp) +
                t728 * t993_tmp) +
               t707_tmp_tmp * (ct[161] * t742 - ct[304] * t1160)) +
              t819 * ct[260]) +
             b_ct_tmp * t994;
  t621 = -ct[217] * ct[315];
  b_J_r[4] =
      ((((((((((((((((((ct[25] - ct[36]) - ct[9] * ct[52]) + ct[9] * ct[58]) -
                     ct[9] * ct[274]) -
                    ct[9] * t972) -
                   ct[9] * t991) -
                  t650_tmp * t625) +
                 t754 * t683) +
                t779 * t969_tmp) -
               t893 * (ct[161] * t625 - ct[304] * t683)) -
              t914 * t1079) +
             t1015 * ct[183] * ct[279] * ct[296]) +
            t444 * ct[183] * ct[194] * ct[303]) -
           t952 * ((t621 * t989 + t806_tmp * t1079) + t1024_tmp * t1083)) -
          t707_tmp_tmp * (ct[161] * t683 + ct[304] * t625)) +
         t728 * t638) -
        ct_tmp * t1083) +
       t819 * ct[261]) +
      b_ct_tmp * t989;
  t444 = (t806_tmp * t773 - ct[294] * t935) + t1045_tmp * t1126_tmp;
  t625 = ct[0] * ct[1];
  t742 = -ct[0] * ct[8] * ct[23] * ct[228] * ct[307];
  b_J_r[5] =
      ((((((((((((((-ct[9] * ct[11] + ct[9] * ct[20]) - ct[9] * ct[37]) -
                  ct[9] * ct[40]) -
                 ct[9] * t755) +
                t650_tmp * t444) +
               c_ct_tmp * t935_tmp) +
              t754 * t1126) +
             t1015 * ct[187] * ct[291]) +
            t893 * (ct[304] * t1126 + ct[161] * t444)) -
           t625 * ct[10] * ct[279] * ct[296]) -
          t779 * ct[186]) -
         t914 * t935) -
        ct_tmp * t1126_tmp) -
       t707_tmp_tmp * (ct[161] * t1126 - ct[304] * t444)) +
      ((((t742 * ((ct[31] * t773 + t806_tmp * t935) + t1024_tmp * t1126_tmp) +
          t728 * ct[188]) -
         b_ct_tmp * t773) +
        ct[1] * ct[5] * ct[6] * ct[13] * ct[187] * ct[202] * ct[291] *
            ct[305]) -
       t625 * ct[5] * ct[6] * ct[13] * ct[202] * ct[279] * ct[296] * ct[305]);
  t1015 = ct[98] * b_t1087_tmp + ct[292] * b_t963_tmp;
  t444 = (-ct[285] + t657_tmp * t632) + t994_tmp * t963_tmp;
  t625 = (ct[294] * c_t1087_tmp + t1045_tmp * t1015) + t806_tmp * t444;
  t683 = (t1087_tmp * c_t1087_tmp + ct[29] * t1015) + t1024_tmp * t444;
  t676 = ct[5] * ct[10];
  t937 *= ct[35];
  b_J_r[6] =
      (((((((((((-ct[25] + ct[36]) + ct[9] * ct[70]) + ct[9] * ct[74]) +
              ct[9] * ct[262]) +
             ct[9] * t963) +
            c_ct_tmp * b_t1087_tmp) +
           t650_tmp * t625) +
          ((t754 * t683 + t914 * c_t1087_tmp) - t779 * t963_tmp)) +
         ((t893 * (ct[161] * t625 + ct[304] * t683) +
           t676 * ct[263] * ct[279] * ct[296]) +
          t937 * ct[194] * ct[263] * ct[303])) +
        (d_ct_tmp * t1015 - t952 * ((ct[31] * t444 - t806_tmp * c_t1087_tmp) +
                                    t1024_tmp * t1015))) +
       t707_tmp_tmp * (ct[304] * t625 - ct[161] * t683)) +
      ((-ct[0] * ct[7] * ct[17] * ct[217] * ct[306] * t444 - t728 * t632) +
       t819 * ct[268]);
  t1015 = (-(t1024_tmp * t976) + t1045_tmp * t1070) + t876_tmp * t1154_tmp;
  b_J_r[7] =
      ((((((((((((((((t995 - ct[9] * ct[53]) + ct[9] * ct[59]) -
                    ct[9] * ct[266]) -
                   ct[9] * t955) -
                  ct[9] * t979) -
                 t650_tmp * t1154) +
                t754 * t1015) +
               t779 * t952_tmp) -
              t893 * (ct[161] * t1154 - ct[304] * t1015)) -
             t914 * t1070) +
            t676 * ct[183] * ct[279] * ct[296]) +
           t937 * ct[183] * ct[194] * ct[303]) -
          ct_tmp * t1154_tmp) -
         t952 * ((t621 * t976 + t806_tmp * t1070) + t1024_tmp * t1154_tmp)) -
        t707_tmp_tmp * (ct[304] * t1154 + ct[161] * t1015)) +
       t728 * t630) +
      (t819 * ct[270] + b_ct_tmp * t976);
  t1015 = ct[6] * ct[13];
  t444 = ct[0] * ct[5];
  b_J_r[8] =
      ((((((((((((((((((ct[9] * ct[15] - ct[9] * ct[27]) - ct[9] * ct[39]) +
                      ct[9] * -t750) +
                     ct[9] * t766) +
                    t650_tmp * t1123) +
                   t754 * t1124) +
                  t676 * ct[187] * ct[291]) +
                 t893 * (ct[161] * t1123 + ct[304] * t1124)) -
                t444 * ct[10] * ct[279] * ct[296]) -
               t779 * ct[163]) -
              t914 * t1123_tmp) -
             t952 * ((ct[31] * ct[301] + t806_tmp * t1123_tmp) +
                     t1024_tmp * t932)) +
            t1015 * ct[187] * ct[202] * ct[305] * ct[310]) -
           t707_tmp_tmp * (ct[161] * t1124 - ct[304] * t1123)) -
          t728 * ct[162]) -
         ct_tmp * t932) -
        b_ct_tmp * ct[301]) +
       t760 * ct[269] * ct[296] * ct[305]) -
      t797 * ct[78] * ct[202] * ct[279] * ct[296] * ct[305];
  b_J_r[9] = -ct[0] * ct[187] * ct[263] * ct[303];
  b_J_r[10] = -ct[0] * ct[183] * ct[187] * ct[303];
  b_J_r[11] = ct[0] * ct[269] * ct[296];
  b_J_r[12] =
      ((((((((((((ct[9] * ct[68] + ct[9] * ct[112]) +
                 ct[9] * (-ct[98] * t816_tmp)) -
                ct[9] * t627) +
               b_t1045_tmp * ct[214]) -
              t650_tmp * t1039) -
             t754 * t1041) -
            t893 * (ct[161] * t1039 + ct[304] * t1041)) -
           t914 * t817) +
          t952 * ((ct[31] * t1039_tmp - t806_tmp * t817) + t1024_tmp * t816)) +
         b_ct_tmp * t1039_tmp) +
        t707_tmp_tmp * (ct[161] * t1041 - ct[304] * t1039)) +
       t728 * ct[177]) +
      ct_tmp * t816;
  b_J_r[13] =
      ((((((((((((ct[9] * ct[57] + ct[9] * ct[110]) +
                 ct[9] * (-ct[98] * t814_tmp)) -
                ct[9] * t623) +
               b_t1045_tmp * ct[192]) -
              t650_tmp * t1037) -
             t754 * t1038) -
            t893 * (ct[161] * t1037 + ct[304] * t1038)) -
           t914 * t815) +
          t707_tmp_tmp * (ct[161] * t1038 - ct[304] * t1037)) +
         b_ct_tmp * t1037_tmp) +
        t952 * ((t1106_tmp * t1037_tmp - t806_tmp * t815) + t1024_tmp * t814)) +
       t728 * ct[176]) +
      ct_tmp * t814;
  b_J_r[14] =
      ((((((((((((-ct[9] * ct[302] - ct[9] * ct[12]) - ct[9] * t527) -
                b_t1045_tmp * ct[114]) +
               t650_tmp * t933) +
              t754 * t943) +
             c_ct_tmp * (ct[115] - ct[137])) +
            t893 * (ct[161] * t933 + ct[304] * t943)) -
           t914 * t636) +
          t707_tmp_tmp * (ct[304] * t933 - ct[161] * t943)) -
         t952 * ((ct[31] * ct[251] + t806_tmp * t636) + t1024_tmp * t637)) +
        t728 * ct[77]) -
       ct_tmp * t637) -
      b_ct_tmp * ct[251];
  t779 = ct[2] * ct[13] * ct[202];
  t937 = b_t1045_tmp * ct[202] * ct[305];
  c_ct_tmp = t698 * ct[286] * ct[297];
  d_ct_tmp = t698 * ct[30] * ct[209] * ct[305];
  t728 = t696 * ct[6] * ct[13] * ct[209] * ct[305];
  b_J_r[15] =
      (((((((((((((((ct[9] * (-ct[98] * t1059_tmp) - ct[9] * t999) +
                    ct[13] * ct[227]) -
                   t650_tmp * t1149) +
                  t754 * t1150) +
                 t779 * ct[184]) -
                t893 * (ct[161] * t1149 - ct[304] * t1150)) +
               t937 * t816_tmp_tmp) -
              t914 * t1060) +
             t456_tmp * ct[13] * ct[286] * ct[297] * t878_tmp_tmp) -
            t707_tmp_tmp * (ct[161] * t1150 + ct[304] * t1149)) -
           t952 * ((ct[31] * t931 + t806_tmp * t1060) - t1024_tmp * t1059)) -
          c_ct_tmp * ct[177]) +
         ct_tmp * t1059) -
        b_ct_tmp * t931) -
       d_ct_tmp * t816_tmp_tmp) +
      t728 * t878_tmp_tmp * 2.0;
  t819 = (((((-ct[167] + ct[179]) + ct[191]) - t456) + ct[200]) +
          ct[32] * t814_tmp_tmp) +
         ct[42] * t814_tmp_tmp;
  t995 = (ct[294] * b_t1069_tmp + t1045_tmp * t1071_tmp) - t806_tmp * t819;
  t621 = (-(t876_tmp * t1071_tmp) + t1045_tmp * b_t1069_tmp) + t1024_tmp * t819;
  t625 = -ct[0] * ct[4] * ct[23] * ct[228] * ct[307];
  t683 = t797 * ct[30] * ct[209] * ct[305];
  t676 = -ct[0] * ct[23];
  b_J_r[16] = ((((((((ct[9] * (-ct[98] * t1069_tmp) + ct[9] * -t1017) +
                     ct[13] * ct[143]) +
                    ct[13] * ct[208]) +
                   t754 * t621) -
                  t650_tmp * t995) +
                 t779 * ct[181]) +
                ((t676 * ct[289] * ct[299] * (ct[161] * t995 - ct[304] * t621) -
                  t914 * b_t1069_tmp) +
                 t937 * t814_tmp_tmp)) +
               (ct_tmp * t1071_tmp -
                t952 * ((t1106_tmp * t819 + t806_tmp * b_t1069_tmp) -
                        t1024_tmp * t1071_tmp))) +
              ((((t625 * (ct[304] * t995 + ct[161] * t621) - b_ct_tmp * t819) -
                 c_ct_tmp * ct[176]) +
                t683 * ct[106]) -
               d_ct_tmp * t814_tmp_tmp);
  b_J_r[17] = (((((((((((((((ct[9] * -t761 + ct[9] * t820) + ct[13] * ct[90]) +
                           ct[13] * ct[126]) +
                          t650_tmp * t1132) +
                         t754 * t1133) +
                        t779 * ct[105]) +
                       t893 * (ct[161] * t1132 + ct[304] * t1133)) +
                      t937 * ct[76]) -
                     t914 * t1132_tmp) -
                    t952 * ((ct[31] * t810 + t806_tmp * t1132_tmp) +
                            t1024_tmp * t1011)) -
                   b_t1045_tmp * ct[49] * ct[209] * ct[305] * ct[76] * 2.0) -
                  t707_tmp_tmp * (ct[161] * t1133 - ct[304] * t1132)) -
                 c_ct_tmp * ct[77]) -
                ct_tmp * t1011) -
               b_ct_tmp * t810) -
              t696 * ct[5] * ct[6] * ct[13] * ct[209] * ct[269] * ct[296] *
                  ct[305] * 2.0;
  t779 = (t1024_tmp * t1013 - t1045_tmp * c_t1045_tmp) + ct[29] * t1144_tmp;
  t1015 *= ct[202];
  c_ct_tmp = t797 * ct[286] * ct[297];
  b_J_r[18] =
      ((((((((-ct[9] * t879 + ct[13] * ct[164]) + ct[13] * ct[223]) +
            ct[9] * ct[98] * t1045_tmp_tmp) -
           t754 * t779) -
          t650_tmp * t1144) +
         t1015 * ct[184]) -
        t893 * (ct[161] * t1144 + ct[304] * t779)) +
       (((-ct[0] * ct[17] * ct[287] * ct[298] * c_t1045_tmp -
          t937 * t878_tmp_tmp) +
         b_t1045_tmp * ct[83] * ct[209] * ct[305] * t878_tmp_tmp * 2.0) +
        t952 * ((ct[31] * t1013 - t806_tmp * c_t1045_tmp) +
                t1024_tmp * t1144_tmp))) +
      ((((t625 * (ct[304] * t1144 - ct[161] * t779) + ct_tmp * t1144_tmp) -
         c_ct_tmp * ct[177]) +
        b_ct_tmp * t1013) -
       t683 * t816_tmp_tmp);
  b_J_r[19] = (((((((((((((((ct[9] * t835 + ct[9] * -t942) + ct[13] * ct[147]) +
                           ct[13] * ct[201]) -
                          t650_tmp * t1147) -
                         t754 * t1148) +
                        t1015 * ct[181]) -
                       t893 * (ct[161] * t1147 + ct[304] * t1148)) -
                      t937 * ct[106]) +
                     t914 * t1053) +
                    t952 * ((t1106_tmp * t1026 + t806_tmp * t1053) +
                            t1024_tmp * t1052)) +
                   t707_tmp_tmp * (ct[161] * t1148 - ct[304] * t1147)) +
                  ct_tmp * t1052) -
                 c_ct_tmp * ct[176]) +
                b_ct_tmp * t1026) +
               t797 * ct[38] * ct[209] * ct[305] * ct[106]) -
              t683 * t814_tmp_tmp;
  t779 = t444 * ct[13];
  b_J_r[20] =
      (((((((((((((((-ct[9] * t737 + ct[9] * t802) + ct[13] * ct[93]) +
                   ct[13] * ct[122]) +
                  t650_tmp * t1130) +
                 t754 * t1131) +
                t1015 * ct[105]) +
               t893 * (ct[161] * t1130 + ct[304] * t1131)) -
              t914 * t940) -
             t707_tmp_tmp * (ct[161] * t1131 - ct[304] * t1130)) -
            t952 * ((ct[31] * t831 + t806_tmp * t940) + t1024_tmp * t938)) -
           ct_tmp * t938) -
          c_ct_tmp * ct[77]) -
         b_ct_tmp * t831) -
        t728 * ct[76] * 2.0) +
       t779 * ct[202] * ct[269] * ct[296] * ct[305]) -
      t779 * ct[83] * ct[209] * ct[269] * ct[296] * ct[305] * 2.0;
  b_J_r[21] = (ct[0] * ct[150] + ct[0] * ct[203]) + ct[0] * ct[235];
  b_J_r[22] = (ct[0] * ct[151] + ct[0] * ct[190]) + ct[0] * ct[211];
  b_J_r[23] = (ct[0] * ct[82] + ct[0] * ct[116]) + ct[0] * ct[117];
  b_J_r[24] =
      (((((((-ct[9] * t685 - ct[9] * t702) - t915 * t899) - t650_tmp * t1050) +
          t754 * t1051) -
         t893 * (ct[161] * t1050 - ct[304] * t1051)) +
        ct_tmp * t896_tmp) -
       t707_tmp_tmp * (ct[161] * t1051 + ct[304] * t1050)) -
      t952 * (ct[7] * t899 - t1024_tmp * t896_tmp);
  t779 = t860 - t876_tmp * t840;
  b_J_r[25] =
      (((((((ct[9] * -t679 + ct[9] * (-ct[98] * t840_tmp)) - t915 * t859) -
           t650_tmp * t1044) +
          t754 * t779) -
         t893 * (ct[161] * t1044 - ct[304] * t779)) -
        t952 * (ct[7] * t859 - t1024_tmp * t840)) -
       t707_tmp_tmp * (ct[304] * t1044 + ct[161] * t779)) +
      ct_tmp * t840;
  t779 = t775 + ct[294] * t787_tmp;
  b_J_r[26] =
      (((((((ct[9] * t565 - ct[9] * t579) + t650_tmp * t779) + t754 * t974) +
          t914 * t787_tmp) +
         t893 * (ct[161] * t779 + ct[304] * t974)) +
        t707_tmp_tmp * (ct[304] * t779 - ct[161] * t974)) +
       t952 * (t806_tmp * t787_tmp - t1024_tmp * t757)) -
      ct_tmp * t757;
  ct_tmp = ct[3] * ct[17] * ct[217];
  t779 = t915 * ct[217] * ct[306];
  b_t1045_tmp = t915 * ct[56] * ct[220] * ct[306];
  t1015 = t598_tmp * ct[288] * ct[298];
  t444 = t878_tmp * ct[7] * ct[17] * ct[220] * ct[306];
  b_J_r[27] = ((((((((((ct[17] * -t733 + ct[17] * -t904) - t650_tmp * t1143) +
                      t754 * t1151) -
                     t893 * (ct[161] * t1143 - ct[304] * t1151)) +
                    t779 * t851) +
                   ct_tmp * ct[294] * t896_tmp) -
                  b_t1045_tmp * t851 * 2.0) -
                 t1015 * t896_tmp) -
                t707_tmp_tmp * (ct[304] * t1143 + ct[161] * t1151)) -
               t952 * (((((t918 + t919) + ct[34] * t851) + ct[108] * t699) +
                        ct[44] * t851) -
                       t597_tmp * t851)) -
              t444 * t699 * 2.0;
  t937 =
      ((((((-ct[7] * ct[217] * ct[306] * t707_tmp - t854) + t861_tmp * t840) -
          t869_tmp * t840) +
         t887_tmp * ct[316] * t842 * 2.0) -
        ct[81] * ct[217] * t842) +
       ct[34] * t707_tmp) +
      ct[44] * t707_tmp;
  t914 = t598_tmp * ct[41] * ct[220] * ct[306];
  b_J_r[28] = ((((((((ct[17] * t709 + ct[17] * -t1140_tmp) - t650_tmp * t1140) +
                    t754 * t937) +
                   ct_tmp * t845) -
                  t893 * (ct[161] * t1140 - ct[304] * t937)) +
                 t779 * t842) -
                b_t1045_tmp * t842 * 2.0) -
               t707_tmp_tmp * (ct[304] * t1140 + ct[161] * t937)) +
              ((t742 * ((((((t857 - t863) + ct[34] * t842) + ct[44] * t842) -
                          ct[217] *
                              (ct[3] * 2.0 -
                               ct[3] * ct[99] * ct[138] * ct[287] * ct[298]) *
                              t707_tmp) -
                         t597_tmp * t842) +
                        t887_tmp * ct[315] * t707_tmp * 2.0) -
                t1015 * t840) -
               t914 * t707_tmp);
  t937 = ((((t598 - b_t601_tmp * ct[277]) +
            -ct[56] * ct[288] * ct[298] * t787_tmp) +
           ct[138] * t775) +
          t594_tmp * t787_tmp) +
         t869_tmp * t787_tmp;
  b_J_r[29] =
      (((((((((ct[17] * -t598 + ct[17] * (t861_tmp * t787_tmp)) +
              t650_tmp * t937) +
             t754 * t1134) -
            ct_tmp * t767) +
           t893 * (ct[304] * t1134 + ct[161] * t937)) -
          t779 * t787_tmp) +
         b_t1045_tmp * t787_tmp * 2.0) +
        t952 * (((((t777 - t783) - ct[108] * ct[277]) + ct[34] * t787_tmp) +
                 ct[44] * t787_tmp) -
                t597_tmp * t787_tmp)) -
       t707_tmp_tmp * (ct[161] * t1134 - ct[304] * t937)) +
      (t1015 * t757 - t444 * ct[277] * 2.0);
  ct_tmp = ct[7] * ct[17] * ct[217];
  b_t1045_tmp = t445 * ct[288] * ct[298];
  t1015 = t915 * ct[99] * ct[220] * ct[306];
  b_J_r[30] =
      ((((((((((ct[17] * -t735 + ct[17] * -t901) - t650_tmp * t1136) +
              t754 * t1137) -
             t893 * (ct[161] * t1136 - ct[304] * t1137)) +
            t779 * t699) +
           ct_tmp * ct[294] * t896_tmp) -
          t1015 * t699 * 2.0) -
         t952 * ((((((t903 + ct[33] * t851) + ct[43] * t851) + ct[146] * t699) -
                   t596_tmp * t851) +
                  t735_tmp * t896_tmp) -
                 t1136_tmp * t896_tmp)) -
        b_t1045_tmp * t896_tmp) -
       t707_tmp_tmp * (ct[161] * t1137 + ct[304] * t1136)) -
      t444 * t851 * 2.0;
  t937 = ((((t711 - t864) + ct[138] * t856) + t601_tmp * t847) +
          t594_tmp * t707_tmp) +
         t1136_tmp * t707_tmp;
  t915 = ct[41] * ct[220];
  b_ct_tmp = (((((-ct[3] * ct[217] * ct[306] * t707_tmp + t857) - t863) -
                ct[67] * ct[217] * t842) +
               ct[33] * t707_tmp) +
              ct[43] * t707_tmp) +
             t915 * ct[316] * t842;
  b_J_r[31] = ((((((ct[17] * t711 - ct[17] * t864) - t650_tmp * t937) +
                  t754 * b_ct_tmp) +
                 ct_tmp * t845) -
                t893 * (ct[161] * t937 - ct[304] * b_ct_tmp)) +
               t779 * t707_tmp) +
              ((((t625 * (ct[304] * t937 + ct[161] * b_ct_tmp) +
                  t952 * (((((((t854 - ct[33] * t842) - ct[43] * t842) +
                              ct[217] *
                                  (ct[41] * ct[294] -
                                   ct[103] * ct[138] * ct[287] * ct[298]) *
                                  t707_tmp) +
                             t596_tmp * t842) -
                            t735_tmp * t840) +
                           t1136_tmp * t840) -
                          t915 * ct[315] * t707_tmp)) -
                 b_t1045_tmp * t840) -
                t914 * t842) -
               t445 * ct[41] * ct[220] * ct[306] * t707_tmp);
  t937 = ((((t596 - ct[33] * ct[277]) - ct[43] * ct[277]) + t777) - t783) +
         ct[109] * t787_tmp;
  b_J_r[32] = ((((((((((ct[17] * -t1121_tmp + ct[17] * t793) - t754 * t937) -
                      t650_tmp * t1121) -
                     ct_tmp * t767) +
                    t779 * ct[277]) -
                   t893 * (ct[304] * t937 + ct[161] * t1121)) -
                  t1015 * ct[277] * 2.0) +
                 t707_tmp_tmp * (ct[161] * t937 - ct[304] * t1121)) -
                t952 * ((((((t774 + ct[146] * ct[277]) - ct[33] * t787_tmp) -
                           ct[43] * t787_tmp) -
                          t735_tmp * t757) +
                         t1136_tmp * t757) +
                        t596_tmp * t787_tmp)) +
               b_t1045_tmp * t757) +
              t444 * t787_tmp * 2.0;
  ct_tmp = t878_tmp * ct[217] * ct[306];
  b_J_r[33] =
      (ct[0] * t732 + ct[0] * ct[287] * ct[298] * t896_tmp) + ct_tmp * t851;
  b_J_r[34] = (ct[0] * (t597_tmp * t707_tmp) + ct[0] * t854) + ct_tmp * t842;
  b_J_r[35] = (ct[0] * t597 - ct[0] * t774) - ct_tmp * t787_tmp;
  b_J_r[36] =
      ((ct[9] * t1114 + ct[9] * -t1119) + t893 * t1142) - t707_tmp_tmp * t1141;
  ct_tmp = (-t845 + t860) + t806_tmp * t707_tmp;
  t779 = t1107 - ct[304] * ct_tmp;
  b_t1045_tmp = ct[304] * t1105 + ct[161] * ct_tmp;
  b_J_r[37] = ((-ct[9] * t1107 + t754 * ct_tmp) - t893 * t779) -
              t707_tmp_tmp * b_t1045_tmp;
  ct_tmp = t1031 + ct[161] * t1127_tmp;
  b_J_r[38] = ((ct[9] * t1031 + t650_tmp * t1127_tmp) + t893 * ct_tmp) -
              t707_tmp_tmp * t1127;
  t1015 = ct[4] * ct[23] * ct[228] * ct[295];
  t444 = t594 * ct[290] * ct[299];
  t650_tmp = t731 * ct[71] * ct[234] * ct[307];
  t754 = ct[4] * ct[8] * ct[23] * ct[290] * ct[299];
  t893 = t975 * ct[8] * ct[23] * ct[234] * ct[307];
  t937 = ct[23] * ct[71] * ct[290] * ct[299];
  t914 = t676 * ct[228] * ct[307];
  b_J_r[39] = (((((t914 * t1142 - t1015 * t1141) + t937 * t1142) +
                 t650_tmp * t1142 * 2.0) +
                t444 * t1141) -
               t754 * t1117) -
              t893 * t1117 * 2.0;
  t952 = t731 * ct[228] * ct[307];
  b_J_r[40] = (((((-ct[23] * ct[71] * ct[290] * ct[299] * t779 + t952 * t779) -
                  t1015 * b_t1045_tmp) +
                 t444 * b_t1045_tmp) -
                t650_tmp * t779 * 2.0) -
               t754 * t1106) -
              t893 * t1106 * 2.0;
  t1015 = (-(ct[31] * ct[277]) + t776) + t1024_tmp * t787_tmp;
  b_J_r[41] =
      (((((-ct[4] * ct[23] * ct[228] * ct[295] * t1127 - t952 * ct_tmp) +
          t937 * ct_tmp) +
         t650_tmp * ct_tmp * 2.0) +
        t754 * t1015) +
       t444 * t1127) +
      t893 * t1015 * 2.0;
  t444 = ct[23] * ct[107] * ct[290] * ct[299];
  t650_tmp = ct[8] * ct[23] * ct[228] * ct[295];
  t937 = t731 * ct[107] * ct[234] * ct[307];
  t707_tmp_tmp = t993 * ct[290] * ct[299];
  b_J_r[42] = (((((t952 * t1117 - t650_tmp * t1141) - t444 * t1117) -
                 t937 * t1117 * 2.0) +
                t707_tmp_tmp * t1141) +
               t754 * t1142) +
              t893 * t1142 * 2.0;
  b_J_r[43] = (((((t952 * t1106 - t444 * t1106) - t650_tmp * b_t1045_tmp) -
                 t937 * t1106 * 2.0) +
                t707_tmp_tmp * b_t1045_tmp) -
               t754 * t779) -
              t893 * t779 * 2.0;
  b_J_r[44] =
      (((((t914 * t1015 + t444 * t1015) - t650_tmp * t1127) + t754 * ct_tmp) +
        t937 * t1015 * 2.0) +
       t707_tmp_tmp * t1127) +
      t893 * ct_tmp * 2.0;
  t444 = -ct[0] * ct[289] * ct[299];
  t650_tmp = t729 * ct[228] * ct[307];
  t754 = t975 * ct[228] * ct[307];
  b_J_r[45] = (t444 * t1141 - t754 * t1142) + t650_tmp * t1117;
  b_J_r[46] = (t444 * b_t1045_tmp + t650_tmp * t1106) + t754 * t779;
  b_J_r[47] = (t444 * t1127 - t650_tmp * t1015) - t754 * ct_tmp;
}

void J_r(double q[16], double L0, double d, double hm, double b_J_r[48],
         double x_r[3])
{
  double b_d[317];
  double b_d_tmp;
  double b_t180_tmp;
  double b_t181_tmp;
  double b_t186_tmp;
  double b_t378_tmp;
  double b_t380_tmp;
  double c_d_tmp;
  double d_d_tmp;
  double d_tmp;
  double e_d_tmp;
  double f_d_tmp;
  double g_d_tmp;
  double t10;
  double t100;
  double t101;
  double t105;
  double t106;
  double t108;
  double t110;
  double t113;
  double t114;
  double t114_tmp;
  double t119;
  double t122_tmp;
  double t124_tmp;
  double t127;
  double t128;
  double t129;
  double t14;
  double t146;
  double t146_tmp;
  double t15;
  double t151;
  double t152;
  double t152_tmp;
  double t153;
  double t153_tmp;
  double t154;
  double t156;
  double t16;
  double t166;
  double t169;
  double t17;
  double t170;
  double t170_tmp;
  double t175;
  double t177;
  double t18;
  double t180;
  double t180_tmp;
  double t181;
  double t181_tmp;
  double t183;
  double t184;
  double t186_tmp;
  double t187_tmp;
  double t189_tmp;
  double t198;
  double t201;
  double t202;
  double t205;
  double t206;
  double t218;
  double t21_tmp;
  double t221;
  double t228;
  double t229;
  double t236;
  double t23_tmp;
  double t24;
  double t241;
  double t243;
  double t25;
  double t257;
  double t258;
  double t265;
  double t265_tmp;
  double t267;
  double t26_tmp;
  double t270;
  double t273;
  double t273_tmp;
  double t275;
  double t275_tmp;
  double t28_tmp;
  double t295;
  double t2_tmp;
  double t309;
  double t30_tmp;
  double t31;
  double t316;
  double t317;
  double t317_tmp;
  double t318;
  double t318_tmp;
  double t32;
  double t321;
  double t322;
  double t328;
  double t331;
  double t331_tmp;
  double t332;
  double t332_tmp;
  double t333;
  double t335;
  double t338;
  double t339;
  double t33_tmp;
  double t34;
  double t341;
  double t35_tmp;
  double t36;
  double t369;
  double t370;
  double t372;
  double t373;
  double t378;
  double t378_tmp;
  double t37_tmp;
  double t380;
  double t380_tmp;
  double t383;
  double t383_tmp;
  double t3_tmp;
  double t400;
  double t401;
  double t420;
  double t422;
  double t431;
  double t438;
  double t43_tmp;
  double t440;
  double t447_tmp;
  double t44_tmp;
  double t45;
  double t46_tmp;
  double t47;
  double t475;
  double t477;
  double t477_tmp;
  double t48_tmp;
  double t49;
  double t4_tmp;
  double t50_tmp;
  double t510;
  double t555;
  double t556;
  double t556_tmp;
  double t567;
  double t568;
  double t568_tmp;
  double t569;
  double t56_tmp;
  double t57_tmp;
  double t58;
  double t59_tmp;
  double t5_tmp;
  double t60;
  double t61_tmp;
  double t62;
  double t63_tmp;
  double t68_tmp;
  double t69_tmp;
  double t6_tmp;
  double t70_tmp;
  double t71_tmp;
  double t72_tmp;
  double t73_tmp;
  double t74_tmp;
  double t75_tmp;
  double t77_tmp;
  double t78_tmp;
  double t7_tmp;
  double t81;
  double t89;
  double t8_tmp;
  double t90;
  double t91;
  double t92;
  double t93;
  double t94_tmp;
  double t95;
  double t96;
  double t97_tmp;
  double t9_tmp;
  /* J_r */
  /*     J_r =
   * J_r(L0,D,dL1,dL2,dL3,dL4,DX1,DX2,DX3,DX4,DY1,DY2,DY3,DY4,HM,TH1,TH2,TH3,TH4)
   */
  /*     This function was generated by the Symbolic Math Toolbox version 23.2.
   */
  /*     04-Dec-2023 15:47:32 */
  if ((q[1] == 0.0) && (q[2] == 0.0)) {
    q[1] = 1.0E-20;
    q[2] = 1.0E-20;
  }
  if ((q[5] == 0.0) && (q[6] == 0.0)) {
    q[5] = 1.0E-20;
    q[6] = 1.0E-20;
  }
  if ((q[9] == 0.0) && (q[10] == 0.0)) {
    q[9] = 1.0E-20;
    q[10] = 1.0E-20;
  }
  if ((q[13] == 0.0) && (q[14] == 0.0)) {
    q[13] = 1.0E-20;
    q[14] = 1.0E-20;
  }
  t2_tmp = cos(q[4]);
  t3_tmp = cos(q[8]);
  t4_tmp = cos(q[12]);
  t5_tmp = cos(q[0]);
  t6_tmp = sin(q[4]);
  t7_tmp = sin(q[8]);
  t8_tmp = sin(q[12]);
  t9_tmp = sin(q[0]);
  t10 = L0 + q[3];
  t14 = q[1] * 2.0;
  t15 = q[5] * 2.0;
  t16 = q[2] * 2.0;
  t17 = q[6] * 2.0;
  t18 = q[10] * 2.0;
  t198 = q[1] * q[1];
  t21_tmp = q[5] * q[5];
  t23_tmp = q[9] * q[9];
  t24 = rt_powd_snf(q[9], 3.0);
  t25 = q[13] * q[13];
  t26_tmp = q[2] * q[2];
  t28_tmp = q[6] * q[6];
  t30_tmp = q[10] * q[10];
  t31 = rt_powd_snf(q[10], 3.0);
  t32 = q[14] * q[14];
  t37_tmp = 1.0 / d;
  t33_tmp = q[1] * t5_tmp;
  t34 = q[2] * t5_tmp;
  t35_tmp = q[1] * t9_tmp;
  t36 = q[2] * t9_tmp;
  t81 = t198 + t26_tmp;
  t318 = t21_tmp + t28_tmp;
  t156 = t23_tmp + t30_tmp;
  t110 = t25 + t32;
  t43_tmp = t34 + t35_tmp;
  t44_tmp = 1.0 / t81;
  t46_tmp = 1.0 / t318;
  t48_tmp = 1.0 / t156;
  t50_tmp = 1.0 / t110;
  t81 = sqrt(t81);
  t318 = sqrt(t318);
  t156 = sqrt(t156);
  t110 = sqrt(t110);
  t45 = t44_tmp * t44_tmp;
  t47 = t46_tmp * t46_tmp;
  t49 = t48_tmp * t48_tmp;
  t56_tmp = t33_tmp - t36;
  t57_tmp = 1.0 / t81;
  t59_tmp = 1.0 / t318;
  t61_tmp = 1.0 / t156;
  t63_tmp = 1.0 / t110;
  t81 *= t37_tmp;
  t318 *= t37_tmp;
  t156 *= t37_tmp;
  t68_tmp = t37_tmp * t110;
  t58 = rt_powd_snf(t57_tmp, 3.0);
  t60 = rt_powd_snf(t59_tmp, 3.0);
  t62 = rt_powd_snf(t61_tmp, 3.0);
  t69_tmp = cos(t81);
  t70_tmp = cos(t318);
  t71_tmp = cos(t156);
  t72_tmp = cos(t68_tmp);
  t73_tmp = sin(t81);
  t74_tmp = sin(t318);
  t75_tmp = sin(t156);
  t77_tmp = t2_tmp * t69_tmp;
  t78_tmp = t6_tmp * t69_tmp;
  t89 = t21_tmp * t70_tmp;
  t90 = t23_tmp * t71_tmp;
  t91 = t26_tmp * t69_tmp;
  t92 = t28_tmp * t70_tmp;
  t93 = t30_tmp * t71_tmp;
  t100 = t2_tmp * t57_tmp * t73_tmp;
  t101 = t6_tmp * t57_tmp * t73_tmp;
  t114_tmp = q[1] * q[2];
  t114 = t114_tmp * t6_tmp * t58 * t73_tmp;
  t127 = t6_tmp * t198 * t58 * t73_tmp;
  t180_tmp = q[5] * t28_tmp;
  b_t180_tmp = t180_tmp * t37_tmp;
  t180 = b_t180_tmp * t60 * t74_tmp;
  t181_tmp = q[6] * t21_tmp;
  b_t181_tmp = t181_tmp * t37_tmp;
  t181 = b_t181_tmp * t60 * t74_tmp;
  t183 = q[9] * t30_tmp * t37_tmp * t62 * t75_tmp;
  t95 = q[10] * t23_tmp * t37_tmp;
  t184 = t95 * t62 * t75_tmp;
  t205 = t26_tmp * t33_tmp * t37_tmp * t58 * t73_tmp;
  t206 = t198 * t34 * t37_tmp * t58 * t73_tmp;
  t105 = q[2] * t2_tmp;
  t236 = t105 * t56_tmp * t58 * t73_tmp;
  t265_tmp = q[2] * t57_tmp;
  t265 = t265_tmp * t59_tmp * t73_tmp * t74_tmp;
  t94_tmp = t198 + t91;
  t569 = t26_tmp + t198 * t69_tmp;
  t96 = t21_tmp + t92;
  t97_tmp = t28_tmp + t89;
  t106 = q[1] * t100;
  t108 = t33_tmp * t44_tmp * (t69_tmp - 1.0);
  t110 = t35_tmp * t44_tmp * (t69_tmp - 1.0);
  t113 = t5_tmp * t100;
  t119 = t9_tmp * t100;
  t122_tmp = t21_tmp * t46_tmp * (t70_tmp - 1.0);
  t124_tmp = t28_tmp * t46_tmp * (t70_tmp - 1.0);
  t146_tmp = t114_tmp * t37_tmp * t44_tmp;
  t146 = t146_tmp * t78_tmp;
  t151 = t15 * t28_tmp * t47 * (t70_tmp - 1.0);
  t152_tmp = q[5] * q[6];
  t152 = t152_tmp * t15 * t47 * (t70_tmp - 1.0);
  t153_tmp = q[9] * q[10];
  t153 = t153_tmp * t18 * t49 * (t71_tmp - 1.0);
  t154 = t18 * t23_tmp * t49 * (t71_tmp - 1.0);
  t170_tmp = q[2] * t37_tmp;
  t170 = t170_tmp * t100;
  t81 = t198 * t37_tmp * t44_tmp;
  t175 = t81 * t78_tmp;
  t189_tmp = t5_tmp * t14;
  t201 = q[2] * t14;
  t166 = t9_tmp * t14;
  t218 = t43_tmp * t100;
  t221 = t43_tmp * t101;
  t228 = t56_tmp * t100;
  t229 = t56_tmp * t101;
  t273_tmp = q[2] * q[5];
  t273 = t273_tmp * t46_tmp * t57_tmp * t73_tmp * (t70_tmp - 1.0);
  t275_tmp = q[2] * q[6];
  t275 = t275_tmp * t46_tmp * t57_tmp * t73_tmp * (t70_tmp - 1.0);
  t128 = q[2] * t108;
  t129 = q[2] * t110;
  t169 = t37_tmp * t106;
  t186_tmp = t5_tmp * t44_tmp;
  b_t186_tmp = t186_tmp * t569;
  t177 = t9_tmp * t44_tmp;
  t187_tmp = t177 * t94_tmp;
  t241 = t14 - q[1] * t26_tmp * t37_tmp * t57_tmp * t73_tmp;
  t243 = t16 - q[2] * t198 * t37_tmp * t57_tmp * t73_tmp;
  t257 = t78_tmp + t106;
  t267 = t14 * t69_tmp - rt_powd_snf(q[1], 3.0) * t37_tmp * t57_tmp * t73_tmp;
  t270 = t16 * t69_tmp - rt_powd_snf(q[2], 3.0) * t37_tmp * t57_tmp * t73_tmp;
  t321 = t151 + t180;
  t322 = t152 + t181;
  t378_tmp = q[5] * t46_tmp * (t70_tmp - 1.0);
  b_t378_tmp = rt_powd_snf(q[5], 3.0) * t37_tmp;
  t378 = (t378_tmp * -2.0 + t15 * t21_tmp * t47 * (t70_tmp - 1.0)) +
         b_t378_tmp * t60 * t74_tmp;
  t380_tmp = q[6] * t46_tmp * (t70_tmp - 1.0);
  b_t380_tmp = rt_powd_snf(q[6], 3.0) * t37_tmp;
  t380 = (t380_tmp * -2.0 + t17 * t28_tmp * t47 * (t70_tmp - 1.0)) +
         b_t380_tmp * t60 * t74_tmp;
  t401 = (-t114 + t146) + t170;
  t156 = q[1] * t101;
  t258 = t77_tmp - t156;
  t295 = t57_tmp * t73_tmp * (t124_tmp + 1.0);
  t309 = t70_tmp * t257;
  t317_tmp = t186_tmp * t94_tmp;
  t317 = t129 + t317_tmp;
  t318_tmp = t177 * t569;
  t318 = t128 + t318_tmp;
  t331_tmp = t129 - b_t186_tmp;
  t331 = -t6_tmp * t331_tmp;
  t332_tmp = t59_tmp * t74_tmp;
  t332 = t332_tmp * t257;
  t335 = t378_tmp * t257;
  t400 = (t114_tmp * t2_tmp * t58 * t73_tmp + t170_tmp * t101) -
         t146_tmp * t77_tmp;
  t438 = ((t101 - t127) + t169) + t175;
  t477_tmp = t229 + t2_tmp * t331_tmp;
  t202 = -t59_tmp * t74_tmp;
  t477 = t202 * t477_tmp;
  t510 = t378_tmp * t477_tmp;
  t316 = t70_tmp * t258;
  t328 = t2_tmp * t318;
  t318 *= t6_tmp;
  t333 = q[5] * t332;
  t338 = t332_tmp * t258;
  t369 = (t122_tmp + 1.0) * t258;
  t370 = t332_tmp * t317;
  t372 = t378_tmp * t317;
  t373 = t380_tmp * t317;
  t383_tmp = t28_tmp * t60 * t74_tmp;
  t383 = t383_tmp * t317;
  t422 = t228 + t331;
  t440 = ((t100 - t2_tmp * t198 * t58 * t73_tmp) + t81 * t77_tmp) +
         t37_tmp * -t156;
  t156 = -(t34 * t44_tmp * (t69_tmp - 1.0)) +
         t201 * t33_tmp * t45 * (t69_tmp - 1.0);
  t555 = ((t156 + t206) + t35_tmp * t45 * t94_tmp * -2.0) + t177 * t241;
  t556_tmp = (-t110 + t166 * t26_tmp * t45 * (t69_tmp - 1.0)) +
             t26_tmp * t35_tmp * t37_tmp * t58 * t73_tmp;
  t556 = (t556_tmp + t34 * t45 * t569 * -2.0) + t186_tmp * t243;
  t81 = -t108 + t189_tmp * t26_tmp * t45 * (t69_tmp - 1.0);
  t567 = ((t81 + t205) + t36 * t45 * t94_tmp * -2.0) + t177 * t270;
  t568_tmp = (-t36 * t44_tmp * (t69_tmp - 1.0) +
              t201 * t35_tmp * t45 * (t69_tmp - 1.0)) +
             t198 * t36 * t37_tmp * t58 * t73_tmp;
  t568 = (t568_tmp + t33_tmp * t45 * t569 * -2.0) + t186_tmp * t267;
  t339 = q[5] * t338;
  t341 = q[6] * t338;
  t420 = t218 + t318;
  t431 = t70_tmp * t422;
  t447_tmp = t221 - t328;
  t108 = t202 * t447_tmp;
  t36 = t332_tmp * t422;
  t100 = t152_tmp * t60 * t74_tmp;
  t475 = t100 * t422;
  t34 = ((t81 + t9_tmp * t16 * t45 * t569) + t205) - t177 * t243;
  t569 = ((t156 + t166 * t45 * t569) + t206) - t177 * t267;
  b_d[0] = d;
  b_d[1] = q[1];
  b_d[2] = q[5];
  b_d[3] = q[9];
  b_d[4] = q[13];
  b_d[5] = q[2];
  b_d[6] = q[6];
  b_d[7] = q[10];
  b_d[8] = q[14];
  b_d[9] = hm;
  b_d[10] = t10;
  b_d[11] = t101;
  b_d[12] = t106;
  d_tmp = L0 + q[7];
  b_d[13] = d_tmp;
  b_d[14] = t113;
  b_d[15] = t114;
  b_d[16] = t119;
  b_d_tmp = L0 + q[11];
  b_d[17] = b_d_tmp;
  b_d[18] = t5_tmp * t101;
  b_d[19] = t9_tmp * t101;
  b_d[20] = t127;
  b_d[21] = t128;
  b_d[22] = t129;
  c_d_tmp = L0 + q[15];
  b_d[23] = c_d_tmp;
  b_d[24] = hm * t113;
  b_d[25] = hm * t119;
  b_d[26] = t14;
  b_d[27] = t146;
  b_d[28] = t122_tmp + 1.0;
  d_d_tmp = t23_tmp * t48_tmp * (t71_tmp - 1.0) + 1.0;
  b_d[29] = d_d_tmp;
  b_d[30] = t15;
  e_d_tmp = t30_tmp * t48_tmp * (t71_tmp - 1.0) + 1.0;
  b_d[31] = e_d_tmp;
  b_d[32] = t152;
  b_d[33] = t153;
  b_d[34] = t154;
  b_d[35] = t16;
  b_d[36] = d * t9_tmp * t10 * t44_tmp * (t69_tmp - 1.0);
  b_d[37] = t169;
  b_d[38] = t17;
  b_d[39] = t170;
  b_d[40] = t175;
  b_d[41] = t18;
  b_d[42] = t181;
  b_d[43] = t183;
  b_d[44] = t184;
  b_d[45] = b_t186_tmp;
  b_d[46] = t187_tmp;
  b_d[47] = -(d * t5_tmp * t10 * t44_tmp * (t69_tmp - 1.0));
  b_d[48] = t2_tmp;
  b_d[49] = t21_tmp;
  b_d[50] = t218;
  b_d[51] = t221;
  t202 = q[1] * t2_tmp;
  b_d[52] = t202 * t43_tmp * t58 * t73_tmp;
  b_d[53] = t105 * t43_tmp * t58 * t73_tmp;
  b_d[54] = t228;
  b_d[55] = t229;
  b_d[56] = t23_tmp;
  b_d[57] = -t221;
  t14 = q[1] * t37_tmp;
  t9_tmp = t14 * t43_tmp * t44_tmp;
  b_d[58] = t9_tmp * t77_tmp;
  f_d_tmp = t170_tmp * t43_tmp * t44_tmp;
  b_d[59] = f_d_tmp * t77_tmp;
  b_d[60] = t9_tmp * t78_tmp;
  b_d[61] = f_d_tmp * t78_tmp;
  b_d[62] = t236;
  t9_tmp = q[1] * t6_tmp;
  b_d[63] = t9_tmp * t56_tmp * t58 * t73_tmp;
  b_d[64] = -(t9_tmp * t43_tmp * t58 * t73_tmp);
  f_d_tmp = q[2] * t6_tmp;
  b_d[65] = f_d_tmp * t56_tmp * t58 * t73_tmp;
  b_d[66] = -(f_d_tmp * t43_tmp * t58 * t73_tmp);
  b_d[67] = t18 - t95 * t61_tmp * t75_tmp;
  b_d[68] = -t229;
  f_d_tmp = t14 * t44_tmp * t56_tmp;
  b_d[69] = f_d_tmp * t77_tmp;
  t198 = t170_tmp * t44_tmp * t56_tmp;
  b_d[70] = t198 * t77_tmp;
  b_d[71] = t25;
  b_d[72] = f_d_tmp * t78_tmp;
  b_d[73] = -(t202 * t56_tmp * t58 * t73_tmp);
  b_d[74] = -t236;
  b_d[75] = -(t198 * t78_tmp);
  b_d[76] = t257;
  b_d[77] = t258;
  b_d[78] = t26_tmp;
  b_d[79] = t265;
  b_d[80] = q[6] * t57_tmp * t59_tmp * t73_tmp * t74_tmp;
  f_d_tmp = t24 * t37_tmp;
  b_d[81] = q[9] * t71_tmp * 2.0 - f_d_tmp * t61_tmp * t75_tmp;
  b_d[82] = t275;
  b_d[83] = t28_tmp;
  t198 = t114_tmp * q[6];
  b_d[84] = t198 * t37_tmp * t44_tmp * t59_tmp * t69_tmp * t74_tmp;
  t25 = t273_tmp * q[6];
  b_d[85] = t25 * t37_tmp * t46_tmp * t57_tmp * t70_tmp * t73_tmp;
  g_d_tmp = q[1] * q[5] * q[2] * q[6];
  b_d[86] = g_d_tmp * t46_tmp * t58 * t73_tmp * (t70_tmp - 1.0);
  b_d[87] = -t273;
  b_d[88] = -t275;
  b_d[89] = -(t198 * t58 * t59_tmp * t73_tmp * t74_tmp);
  b_d[90] = -(t25 * t57_tmp * t60 * t73_tmp * t74_tmp);
  b_d[91] = t170_tmp * t46_tmp * t57_tmp * t73_tmp * t92;
  b_d[92] = q[6] * t37_tmp * t44_tmp * t59_tmp * t74_tmp * t91;
  b_d[93] = -(q[2] * t28_tmp * t57_tmp * t60 * t73_tmp * t74_tmp);
  b_d[94] = -(q[6] * t26_tmp * t58 * t59_tmp * t73_tmp * t74_tmp);
  b_d[95] = g_d_tmp * t37_tmp * t44_tmp * t46_tmp * t69_tmp * (t70_tmp - 1.0);
  t198 = t265_tmp * t73_tmp;
  b_d[96] = t198 * t151;
  b_d[97] = t198 * t152;
  b_d[98] = t3_tmp;
  b_d[99] = t30_tmp;
  b_d[100] = t114_tmp * t58 * t73_tmp * (t124_tmp + 1.0);
  b_d[101] = t198 * t180;
  b_d[102] = t198 * t181;
  b_d[103] = t31;
  b_d[104] = -(t146_tmp * t69_tmp * (t124_tmp + 1.0));
  b_d[105] = t316;
  b_d[106] = t317;
  b_d[107] = t32;
  b_d[108] = t153 + t183;
  b_d[109] = t154 + t184;
  b_d[110] = t328;
  b_d[111] = t318;
  b_d[112] = -t2_tmp * t331_tmp;
  b_d[113] = t331;
  b_d[114] = t332;
  b_d[115] = t333;
  b_d[116] = t335;
  b_d[117] = t338;
  t14 = t152_tmp * t37_tmp;
  g_d_tmp = t14 * t46_tmp;
  b_d[118] = g_d_tmp * t309;
  b_d[119] = -t335;
  b_d[120] = -(t380_tmp * t257);
  b_d[121] = t100 * t258;
  b_d[122] = -(t100 * t257);
  b_d[123] = -t338;
  t236 = t37_tmp * t46_tmp;
  t154 = t236 * t89;
  b_d[124] = t154 * t257;
  t110 = t21_tmp * t60 * t74_tmp;
  b_d[125] = t110 * t258;
  b_d[126] = -(t110 * t257);
  b_d[127] = t383_tmp * t258;
  b_d[128] = t151 * t257;
  b_d[129] = t152 * t257;
  b_d[130] = t37_tmp * t339;
  b_d[131] = t37_tmp * t341;
  b_d[132] = g_d_tmp * -t316;
  b_d[133] = t180 * t257;
  b_d[134] = t181 * t257;
  b_d[135] = -(t154 * t258);
  t273_tmp = t236 * t92;
  b_d[136] = -(t273_tmp * t258);
  b_d[137] = t369;
  b_d[138] = t37_tmp;
  b_d[139] = t370;
  b_d[140] = q[6] * t370;
  b_d[141] = t373;
  b_d[142] = t198 * t321;
  b_d[143] = t100 * t317;
  b_d[144] = q[6] * t372;
  b_d[145] = (-(q[9] * t48_tmp * (t71_tmp - 1.0) * 2.0) +
              t24 * t49 * (t71_tmp - 1.0) * 2.0) +
             f_d_tmp * t62 * t75_tmp;
  b_d[146] = (q[10] * t48_tmp * (t71_tmp - 1.0) * -2.0 +
              t18 * t30_tmp * t49 * (t71_tmp - 1.0)) +
             t31 * t37_tmp * t62 * t75_tmp;
  b_d[147] = t383;
  b_d[148] = g_d_tmp * t70_tmp * t317;
  b_d[149] = -t372;
  f_d_tmp = t128 - t187_tmp;
  t229 = -q[6] * t46_tmp * (t70_tmp - 1.0);
  b_d[150] = t229 * f_d_tmp;
  b_d[151] = -t373;
  t156 = -q[5] * t46_tmp * (t70_tmp - 1.0);
  b_d[152] = q[6] * (t156 * f_d_tmp);
  t146_tmp = q[6] * t59_tmp * t74_tmp;
  b_d[153] = t146_tmp * f_d_tmp;
  b_d[154] = t273_tmp * t317;
  b_d[155] = -t28_tmp * t60 * t74_tmp * f_d_tmp;
  b_d[156] = -t383;
  t265_tmp = t46_tmp * t96;
  b_d[157] = t265_tmp * t317;
  b_d[158] = t151 * t317;
  b_d[159] = t152 * t317;
  t114_tmp = -q[5] * q[6];
  b_d[160] = t114_tmp * t37_tmp * t46_tmp * t70_tmp * f_d_tmp;
  b_d[161] = t4_tmp;
  b_d[162] = t400;
  b_d[163] = t401;
  b_d[164] = t383_tmp * f_d_tmp;
  b_d[165] = (t124_tmp + 1.0) * t317;
  b_d[166] = -t46_tmp * t96 * f_d_tmp;
  b_d[167] = t15 * t47 * t96 * t317;
  b_d[168] = t17 * t47 * t96 * t317;
  b_d[169] = t180 * t317;
  b_d[170] = t181 * t317;
  b_d[171] = t180_tmp * t47 * (t70_tmp - 1.0) * f_d_tmp * 2.0;
  t81 = t181_tmp * t47 * (t70_tmp - 1.0);
  b_d[172] = t81 * f_d_tmp * 2.0;
  b_d[173] = -(t124_tmp + 1.0) * f_d_tmp;
  b_d[174] = t180 * f_d_tmp;
  b_d[175] = t181 * f_d_tmp;
  b_d[176] = t420;
  b_d[177] = t422;
  b_d[178] = t70_tmp * t401;
  b_d[179] = t46_tmp * (t15 - b_t180_tmp * t59_tmp * t74_tmp) * t317;
  b_d[180] = t257 * t322;
  b_d[181] = t70_tmp * t420;
  b_d[182] = t198 * t380;
  b_d[183] = t43_tmp;
  b_d[184] = t431;
  t201 = q[5] * t59_tmp * t74_tmp;
  b_d[185] = -(t201 * t400);
  b_d[186] = t438;
  b_d[187] = t44_tmp;
  b_d[188] = t440;
  b_d[189] =
      -(t46_tmp * (t17 * t70_tmp - b_t380_tmp * t59_tmp * t74_tmp) * t317);
  b_d[190] = t332_tmp * t420;
  b_d[191] = t100 * t420;
  b_d[192] = t108;
  b_d[193] = q[5] * t108;
  b_d[194] = t45;
  b_d[195] = t110 * t420;
  b_d[196] = t383_tmp * t420;
  b_d[197] = -t321 * f_d_tmp;
  t236 = t152_tmp * t46_tmp;
  t183 = t236 * (t70_tmp - 1.0);
  b_d[198] = t183 * t420;
  b_d[199] = t156 * t447_tmp;
  b_d[200] = t229 * t447_tmp;
  t156 = t114_tmp * t60 * t74_tmp;
  b_d[201] = t156 * t447_tmp;
  b_d[202] = t46_tmp;
  b_d[203] = t36;
  b_d[204] = q[5] * t36;
  b_d[205] = q[6] * t36;
  b_d[206] = t154 * t420;
  b_d[207] = t273_tmp * t420;
  t318 = -t21_tmp * t60 * t74_tmp;
  b_d[208] = t318 * t447_tmp;
  b_d[209] = t47;
  b_d[210] = g_d_tmp * (-t70_tmp * t447_tmp);
  b_d[211] = t378_tmp * t447_tmp;
  b_d[212] = t475;
  b_d[213] = t100 * t447_tmp;
  b_d[214] = t477;
  t338 = t46_tmp * t97_tmp;
  b_d[215] = t338 * t420;
  b_d[216] = q[5] * t477;
  b_d[217] = t48_tmp;
  b_d[218] = t183 * t422;
  b_d[219] = -t151 * t447_tmp;
  b_d[220] = t49;
  b_d[221] = g_d_tmp * t431;
  b_d[222] = t229 * t477_tmp;
  b_d[223] = t156 * t477_tmp;
  b_d[224] = -t475;
  b_d[225] = t154 * t422;
  b_d[226] = t273_tmp * t422;
  b_d[227] = t318 * t477_tmp;
  b_d[228] = t50_tmp;
  b_d[229] = -(t110 * t422);
  b_d[230] = -(t383_tmp * t422);
  b_d[231] = g_d_tmp * (-t70_tmp * t477_tmp);
  b_d[232] = -t15 * t47 * t97_tmp * t447_tmp;
  b_d[233] = -t17 * t47 * t97_tmp * t447_tmp;
  b_d[234] = t50_tmp * t50_tmp;
  b_d[235] = t510;
  b_d[236] = -t180 * t447_tmp;
  b_d[237] = (t122_tmp + 1.0) * t422;
  b_d[238] = q[6] * t510;
  b_d[239] = t309 + t339;
  b_d[240] = -t151 * t477_tmp;
  b_d[241] = t81 * t477_tmp * 2.0;
  b_d[242] = t70_tmp * t438;
  b_d[243] = t257 * t378;
  b_d[244] = -t180 * t477_tmp;
  b_d[245] = t338 * t477_tmp;
  b_d[246] = (t122_tmp + 1.0) * t477_tmp;
  b_d[247] = t181 * t477_tmp;
  b_d[248] = t201 * t438;
  b_d[249] = t146_tmp * t438;
  b_d[250] = t201 * t440;
  b_d[251] = q[6] * t332 - t183 * t258;
  b_d[252] = t46_tmp * (t17 - b_t181_tmp * t59_tmp * t74_tmp) * t447_tmp;
  b_d[253] = -(t183 * t440);
  b_d[254] = (t122_tmp + 1.0) * t440;
  g_d_tmp = t333 - t369;
  b_d[255] = -t7_tmp * g_d_tmp;
  b_d[256] = t380 * f_d_tmp;
  b_d[257] = t3_tmp * g_d_tmp;
  b_d[258] =
      t46_tmp * (t15 * t70_tmp - b_t378_tmp * t59_tmp * t74_tmp) * t447_tmp;
  b_d[259] = -t322 * t477_tmp;
  b_d[260] = t555;
  b_d[261] = (t568_tmp + t189_tmp * t45 * t94_tmp) - t186_tmp * t241;
  b_d[262] = t6_tmp * t556;
  b_d[263] = t56_tmp;
  b_d[264] = t2_tmp * t556;
  b_d[265] = t2_tmp * t34;
  b_d[266] = t6_tmp * t34;
  b_d[267] = (q[6] * t265 - t316) + t333;
  b_d[268] = t567;
  b_d[269] = t57_tmp;
  b_d[270] = (t556_tmp + t5_tmp * t16 * t45 * t94_tmp) - t186_tmp * t270;
  b_d[271] = t2_tmp * t568;
  b_d[272] = t6_tmp * t568;
  b_d[273] = t2_tmp * t569;
  b_d[274] = t6_tmp * t569;
  b_d[275] = -t378 * t477_tmp;
  b_d[276] = (q[6] * t273 + t339) + (t122_tmp + 1.0) * t257;
  b_d[277] = (q[2] * t295 + t341) + q[6] * t335;
  b_d[278] = t146_tmp * t555;
  b_d[279] = t58;
  b_d[280] = t183 * t555;
  b_d[281] = (t124_tmp + 1.0) * t555;
  b_d[282] = t146_tmp * t567;
  b_d[283] = t183 * t567;
  b_d[284] = t59_tmp;
  b_d[285] = (t124_tmp + 1.0) * t567;
  b_d[286] = t60;
  b_d[287] = t61_tmp;
  b_d[288] = t62;
  b_d[289] = t63_tmp;
  b_d[290] = rt_powd_snf(t63_tmp, 3.0);
  b_d[291] = t69_tmp;
  b_d[292] = t7_tmp;
  b_d[293] = t70_tmp;
  b_d[294] = t71_tmp;
  b_d[295] = t72_tmp;
  b_d[296] = t73_tmp;
  b_d[297] = t74_tmp;
  b_d[298] = t75_tmp;
  f_d_tmp = sin(t68_tmp);
  b_d[299] = f_d_tmp;
  b_d[300] =
      (((-(t236 * t57_tmp * t73_tmp * (t70_tmp - 1.0)) +
         t152_tmp * t26_tmp * t46_tmp * t58 * t73_tmp * (t70_tmp - 1.0)) -
        t14 * t44_tmp * t46_tmp * (t70_tmp - 1.0) * t91) +
       t201 * t401) +
      (t122_tmp + 1.0) * t400;
  b_d[301] = (((-t295 + t26_tmp * t58 * t73_tmp * (t124_tmp + 1.0)) -
               t37_tmp * t44_tmp * t91 * (t124_tmp + 1.0)) +
              t146_tmp * t401) +
             t183 * t400;
  b_d[302] = t78_tmp;
  b_d[303] = t69_tmp - 1.0;
  b_d[304] = t8_tmp;
  b_d[305] = t70_tmp - 1.0;
  b_d[306] = t71_tmp - 1.0;
  b_d[307] = t72_tmp - 1.0;
  b_d[308] = t89;
  b_d[309] = t90;
  b_d[310] = t91;
  b_d[311] = t92;
  b_d[312] = t93;
  b_d[313] = t96;
  b_d[314] = t97_tmp;
  g_d_tmp = t23_tmp + t93;
  b_d[315] = g_d_tmp;
  t154 = t30_tmp + t90;
  b_d[316] = t154;
  ft_1(b_d, b_J_r);
  /*  calculate x_r */
  t93 = t2_tmp * t43_tmp * t57_tmp * t73_tmp;
  t95 = t2_tmp * t56_tmp * t57_tmp * t73_tmp;
  t108 = q[2] * t33_tmp * t44_tmp * (t69_tmp - 1.0);
  t81 = q[2] * t35_tmp * t44_tmp * (t69_tmp - 1.0);
  t228 = t78_tmp + t202 * t57_tmp * t73_tmp;
  t221 = t9_tmp * t57_tmp * t73_tmp;
  t100 = t77_tmp - t221;
  t105 = t81 + t317_tmp;
  t106 = t108 + t318_tmp;
  t318 = t81 - b_t186_tmp;
  t113 = -t6_tmp * t318;
  t218 = t6_tmp * t106;
  t128 = t95 + t113;
  t36 = (t25 * t46_tmp * t57_tmp * t73_tmp * (t70_tmp - 1.0) + t201 * t100) +
        (t122_tmp + 1.0) * t228;
  t151 = (t198 * (t124_tmp + 1.0) + t146_tmp * t100) + t183 * t228;
  t175 = t93 + t218;
  t146 = (t275_tmp * t57_tmp * t59_tmp * t73_tmp * t74_tmp - t70_tmp * t100) +
         t201 * t228;
  t153 = t7_tmp * t36;
  t119 = t153_tmp * t48_tmp * (t71_tmp - 1.0);
  t170 = t3_tmp * t146;
  t129 = t108 - t187_tmp;
  t127 = t6_tmp * t56_tmp * t57_tmp * t73_tmp + t2_tmp * t318;
  t318 = -q[5] * t59_tmp * t74_tmp;
  t81 = (t146_tmp * t129 + t70_tmp * t128) + t318 * t127;
  t110 = (t114_tmp * t46_tmp * (t70_tmp - 1.0) * t129 + t201 * t128) +
         (t122_tmp + 1.0) * t127;
  t169 = (-(t124_tmp + 1.0) * t129 + t146_tmp * t128) + t183 * t127;
  t101 = t6_tmp * t43_tmp * t57_tmp * t73_tmp - t2_tmp * t106;
  t156 = (t146_tmp * t105 + t70_tmp * t175) + t318 * t101;
  t114 = t3_tmp * t81;
  t318 = (-(t183 * t105) + t201 * t175) + t338 * t101;
  t166 = -t7_tmp * t318;
  t569 = t7_tmp * t110;
  t177 = t170 + t153;
  t267 = t3_tmp * t156;
  t180_tmp = q[9] * t61_tmp * t75_tmp;
  t206 = t7_tmp * t81 + t3_tmp * t110;
  t184 = t267 + t166;
  t205 = t7_tmp * t156 + t3_tmp * t318;
  t243 = q[10] * t61_tmp * t75_tmp;
  t25 = t7_tmp * t146 - t3_tmp * t36;
  t198 = (t243 * t151 + t71_tmp * t177) + -q[9] * t61_tmp * t75_tmp * t25;
  t81 = t114 - t569;
  t202 = (t243 * t169 + -t71_tmp * t81) + t180_tmp * t206;
  t34 = (t119 * t169 + t180_tmp * t81) + d_d_tmp * t206;
  t9_tmp = (-(t265_tmp * t105) + t146_tmp * t175) + t183 * t101;
  t201 = (t119 * t9_tmp + t180_tmp * t184) + t48_tmp * t154 * t205;
  t14 = hm * t8_tmp;
  t183 = hm * t4_tmp;
  t114_tmp = d * c_d_tmp * t63_tmp * f_d_tmp;
  t338 = d * t10;
  t273_tmp = d * d_tmp * t59_tmp * t74_tmp;
  t36 = d * b_d_tmp * t61_tmp * t75_tmp;
  t229 = d * q[5] * d_tmp * t46_tmp * (t70_tmp - 1.0);
  t236 = d * q[13] * c_d_tmp * t50_tmp * (t72_tmp - 1.0);
  t110 = d * q[14] * c_d_tmp * t50_tmp * (t72_tmp - 1.0);
  t108 = d * q[10] * b_d_tmp * t48_tmp * (t71_tmp - 1.0);
  t318 = d * q[9] * b_d_tmp * t48_tmp * (t71_tmp - 1.0);
  t156 = d * q[6] * d_tmp * t46_tmp * (t70_tmp - 1.0);
  x_r[0] = ((((((((((((((hm * t95 + hm * t113) + hm * t114) - hm * t569) -
                      t183 * t202) -
                     t14 * t34) -
                    t114_tmp * (t4_tmp * t202 + t8_tmp * t34)) -
                   t338 * t44_tmp * t56_tmp * (t69_tmp - 1.0)) +
                  t273_tmp * t128) +
                 t36 * t81) +
                t110 * ((e_d_tmp * t169 + t243 * t81) + t119 * t206)) -
               t156 * t129) +
              t236 * (t4_tmp * t34 - t8_tmp * t202)) +
             t229 * t127) +
            t318 * t206) +
           t108 * t169;
  t81 = (-(t71_tmp * t184) + t180_tmp * t205) + t243 * t9_tmp;
  x_r[1] =
      (((((((((((hm * t93 + hm * t218) + hm * t267) + hm * t166) - t14 * t201) -
             t183 * t81) -
            t114_tmp * (t8_tmp * t201 + t4_tmp * t81)) -
           t338 * t43_tmp * t44_tmp * (t69_tmp - 1.0)) +
          t273_tmp * t175) +
         t36 * t184) +
        t229 * t101) +
       t236 * (t4_tmp * t201 - t8_tmp * t81)) +
      (((t110 * ((t48_tmp * g_d_tmp * t9_tmp + t243 * t184) + t119 * t205) +
         t108 * t9_tmp) +
        t318 * t205) -
       t156 * t105);
  t156 = (-(t119 * t151) + t180_tmp * t177) + d_d_tmp * t25;
  x_r[2] =
      ((((((((((((((hm * t77_tmp + hm * -t221) - hm * t170) - hm * t153) +
                 t14 * t156) -
                t183 * t198) +
               t338 * t57_tmp * t73_tmp) +
              t273_tmp * t100) -
             t36 * t177) -
            t114_tmp * (t4_tmp * t198 - t8_tmp * t156)) -
           t110 * ((-e_d_tmp * t151 + t243 * t177) + t119 * t25)) -
          t236 * (t8_tmp * t198 + t4_tmp * t156)) -
         t318 * t25) +
        t229 * t228) +
       t108 * t151) +
      d * q[2] * q[6] * d_tmp * t46_tmp * t57_tmp * t73_tmp * (t70_tmp - 1.0);
}

/* End of code generation (J_r.c) */
