//**************************************************************************//
//******************************2014µç´Å³ÌÐò*****************************//
//                        Copyright by ÑîÁ£&²ñÁÁÁÁ   2014                             //
//*************************************************************************//
//*************************************************************************//

#include "MPC5605B.h" /* Use proper include file */
#include "MyHeader.h"
//===============================Íâ²¿º¯Êý=====================================//
extern core_config();
extern enableIrq();
extern disableIrq();
extern initSTM();
extern initSCI();
extern SCI0_SendChar();
extern SCI0_RecvChar();
extern initADC();
extern initPIT();
extern initEMIOS();


 //==========================µ÷ÓÃµÄº¯Êý======================================//
uint16_t servo_Fuzzy_Kp_chu_shi_hua( void ) ;
uint16_t servo_Fuzzy_Kd_chu_shi_hua( void ) ;
void Cai_yang(void);
void zhuanxiang(void);
uint16_t ABS(int16_t x,int16_t y);
uint16_t diangan_ADC(uint8_t tongdao);
void PD_servo( void);
void PID_Init( void );
void zhuan_xiang_control( void );
void wandao_check( void );
void pid_change_right( void );
void pid_change_left( void );
void noise_protect_first( void );
void noise_protect_second( void );
void lost_signal( void );
void distance_calculate( void );
void guiyi_chushihua( void ) ;
// =======CCD=======//
void StartInte1(void) ; // ÆØ¹â
void ImageCapture1(uint8_t *Data1); //²ÉÑù
void SendImageData(uint8_t *ImageData); // ÉÏÎ»»ú
void SendHex(uint8_t hex);//
void CCD_bizhang( void ) ;
//*********************************²ÉÑù±äÁ¿************************************//
#define TSL_SI1              SIU.GPDO[PCR63_PD15].R    //¶¨ÒåÏßÐÔ´«¸ÐÆ÷µÄ¶Ë¿Ú SI  27
#define TSL_CLK1           SIU.GPDO[PCR31_PB15].R    //¶¨ÒåÏßÐÔ´«¸ÐÆ÷µÄ¶Ë¿Ú CLK  7
#define TSL_CLK_DDR1  SIU.PCR[PCR31_PB15].R
#define TSL_SI_DDR1     SIU.PCR[PCR63_PD15].R

uint8_t  CCD_sample[128]; 
uint8_t  CCD_sample_now[128]; 
uint8_t CCD_lefttimes[10] ={0} ;
uint8_t CCD_righttimes[10] ={0} ;
uint8_t CCD_bizhang_flag=0; 
uint16_t CCD_yuzhi=0;
uint8_t CCD_value=0 ;
uint8_t CCD_zuobiao[2];
int8_t CCD_Error[5]= {0} ;
uint8_t Inteflag=0;
uint8_t sendcnt=0;
uint8_t CCD_min=0;
uint16_t CCD_max[8]={0};
uint8_t CCD_xuanze_flag=0;
uint8_t CCD_HUAT=0;
uint8_t CCD_HUAT_dis ;
uint32_t stop_times = 0 , shijian_ting_che=0 , shi_jian_ceshi_she_ding_shijian=0  ;     //PIT¼ÆÊýÆ÷
uint32_t CCD_times=0;
uint32_t CCD_stoptimes=0;
//uint32_t ccd_times=0; //   =1 ËµÃ÷ÒÑ¾­¹ýÕÏ°­
uint32_t ccd_stoptimes=0;
uint32_t distance[10]={0};
uint32_t zhijiao_charge[5]={0};
uint32_t chuizhi_point[2]={0};
uint32_t xiezhi_point[2]={0};
uint32_t zhijaio_point_k[10]={0};
uint32_t CCD_juli_zuo[5]={0} , CCD_juli_you[5]={0} , CCD_juli[5]={0} , CCD_juli_dm_she_ding_juli[10]={0}  ;     //PIT¼ÆÊýÆ÷
uint8_t  CCD_juli_flag=0 , CCD_baoguang_shijian=10 , CCD_jiance_you_zhangai_flag=0 , podao_flag=0 , podao_flag_dingju=0 ;
//*********************************×ªÏò±äÁ¿********************************//
int16_t Mid1[10]={0};
uint16_t variance1=0,variance2=0,variance=0;

uint8_t Fazhi_R11,Fazhi_R12,Fazhi_L11,Fazhi_L12;
uint8_t Max_pixel1,maxi1,Min_pixel1,mini1;
int16_t left11,right11,mid11,left12,right12,mid12,mid1=64,mid_x1;
uint8_t midcnt1=0  ; // 0 ÎÞÕÏ°­     1 ×ó  2 ÓÒ
uint16_t exce_pixel1;
//********************************ÑÓÊ±***********************************//
#define Delay1us() \
{ __asm(nop); __asm(nop); __asm(nop); __asm(nop);\
       __asm(nop); __asm(nop); __asm(nop); __asm(nop);\
       __asm(nop); __asm(nop); __asm(nop); __asm(nop);\
       __asm(nop); __asm(nop); __asm(nop); __asm(nop);\
       __asm(nop); __asm(nop); __asm(nop); __asm(nop);\
       __asm(nop); __asm(nop); __asm(nop); __asm(nop);\
       __asm(nop); __asm(nop); __asm(nop); __asm(nop);\
       __asm(nop); __asm(nop); __asm(nop); __asm(nop);\
       __asm(nop); __asm(nop); __asm(nop); __asm(nop);\
       __asm(nop); __asm(nop); __asm(nop); __asm(nop);\
       __asm(nop); __asm(nop); __asm(nop); __asm(nop);\
       __asm(nop); __asm(nop); __asm(nop); __asm(nop);\
       __asm(nop); __asm(nop); __asm(nop); __asm(nop);\
       __asm(nop); __asm(nop); __asm(nop); __asm(nop);\
       __asm(nop); __asm(nop); __asm(nop); __asm(nop);\
       __asm(nop); __asm(nop); __asm(nop); __asm(nop);\
}
#define SamplingDelay() \
{  __asm(nop); __asm(nop); __asm(nop); __asm(nop);\
       __asm(nop); __asm(nop); __asm(nop); __asm(nop);\
       __asm(nop); __asm(nop); __asm(nop); __asm(nop);\
       __asm(nop); __asm(nop); __asm(nop); __asm(nop);\
}

//==========================  dian_gan  ==   µç¸Ð       ===============================//
#define   DIAN    42     //  ³õÊ¼²ÉÑù    Ò»¸öÖÜÆÚÒ»ÐÐ²É¼¯µÄµã
#define   guiyi_DIAN    80      //   ¹éÒ»»¯ÖÐ  Ò»¸öµç¸Ð²ÉµÄÖµ
uint8_t flag_2;
int16_t array_one[50]={0};    // 1   Îå¸öµç¸Ð²É»ØÀ´µÄÖµ  
int16_t array_two[50]={0};    // 2
int16_t array_three[50]={0};  // 3
int16_t array_four[50]={0};   //  4
int16_t array_five[50]={0};    //  5  
int16_t array_six[50]={0};    //  5 
int16_t array_seven[50]={0};    //  5                 
int16_t array_current[8]={0};//  ´æ´¢µç¸ÐµÄÖµ±ãÓÚ¼ÆËã
int16_t guiyi_wugediangan_MAX = 0 ;
int16_t speed_Fuzzy_tiaosu_flag = 0 ;
uint8_t array_lvbo_flag[6][8] = { { 0 } , { 0 } , { 0 } , { 0 } , { 0 } , { 0 } } ; //ÂË²¨±êÖ¾Î»  ¡ª¡ªarray_lvbo_flag[7] ¡ª¡ª È·¶¨ÊÇÄÄ¸öµç¸Ð  
                                                           //                 ¡ª¡ª array_lvbo_flag[7][2] ¡ª¡ªÑ¡ÔñÏÞ·ùµÄ´óÐ¡

//=================  pian_cha___xinhao  ==   Æ«²î¡ª¡ªÐÅºÅ        ===============================//
#define   Error_lishudu_Max   200      // Ä£ºý¿ØÖÆÖÐ¡ª¡ªÆ«²î¡ª¡ªÁ¥Êô¶È×î´óÖµ   ==> Á¥Êô¶È  0---200
#define   Error_c_lishudu_Max   200  //  Ä£ºý¿ØÖÆÖÐ¡ª¡ªÆ«²î±ä»¯ÂÊ¡ª¡ªÁ¥Êô¶È×î´óÖµ
uint8_t  diangan_biaoding_xuhao = 1 ;  // ¹éÒ»»¯ ±ê¶¨µç¸ÐÊ± £¬ Ñ¡Ôñ±ê¶¨µÚ¼¸¸öµç¸Ð
uint16_t guiyi_MAX[8] ={ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0};  // ¹éÒ»»¯±ê¶¨ºó£¬Ã¿¸öµç¸ÐµÄ×î´óÖµ
int16_t servo_Error[100] = {0} ;  //  ´æ´¢Ã¿´ÎÓÃ¹ýµÄ Æ«²îÖµ
int16_t  servo_aaaaa[2];
int16_t servo_Error_c[50] = {0} ; // ´æ´¢Ã¿´ÎÓÃ¹ýµÄÆ«²î±ä»¯ÂÊµÄÖµ
uint8_t servo_Error_flag[10]={    0 , 0 , 0 , 0 , 0  };  // ÉÏ´Î´¹Ö±µç¸ÐµÄÆ«²î
uint8_t servo_output_flag[10]={    0 , 0 , 0 , 0 , 0  };  // ÉÏ´Î´¹Ö±µç¸ÐµÄÆ«²î
                    
int16_t abs_sum_[10]={0}; // ´æ´¢  Ê®×éÆ«²î  µÄ  ¾ø¶ÔÖµ  µÄ   ºÍ
int16_t abs_sum=0;   //¾ø¶ÔÖµºÍ   
int16_t daishu_sum=0;//´úÊýºÍ
uint16_t diangan_min[20]={0} ;
int16_t  middle_piancha[20]={0} ;
uint16_t diangan_maxfour[30]={0} ;  // ÌáÈ¡Èý¸öË®Æ½µç¸ÐÖÐµÄ×î´óÖµuint8_t lost_signal_flag=0;
uint16_t diangan_max[30]={0} ;  // ÌáÈ¡Èý¸öË®Æ½µç¸ÐÖÐµÄ×î´óÖµ
uint16_t chuizhidiangan_max[20]={0} ;
uint16_t  total_diangan_sum[20]={  0 , 0 , 0 , 0 , 0  }  ;  
uint16_t  Verticaldiangan_sum[20]={  0 , 0 , 0 , 0 , 0  }  ;  
uint16_t  Leveltaldiangan_sum[20]={  0 , 0 , 0 , 0 , 0  }  ;
uint16_t  xiezhi_diangan_sum[20]={  0 , 0 , 0 , 0 , 0  }  ;  
 
uint8_t dian_gan_cixu = 0 ; // ÅÐ¶ÏÈý¸öË®Æ½µç¸ÐÖÐÄÄ¸öµç¸ÐµÄÖµ×î´ó
int16_t servo_output_last_max = 0 ; //  ÕÒ³öÉÏ´Î¶æ»úÊä³öµÄ×î´óÖµ
uint8_t cixu[10][3]={0};  // ½«Îå¸öµç¸ÐµÄÖµ½øÐÐÅÅÐò
int16_t wandao_sppch[12]={0};
int16_t shuiping_piancha[12] = {    0 , 0 , 0 , 0 , 0  };   // Ã¿´Î²ÉÍêÑù£¬¼ÆËã³öÀ´µÄË®Æ½µç¸ÐµÄ  Æ«²î
uint8_t shuiping_piancha_flag[10] = {    0 , 0 , 0 , 0 , 0  };  //  ÉÏ´ÎË®Æ½µç¸ÐµÄÆ«²î 
int16_t xiezhi_piancha[12] = {    0 , 0 , 0 , 0 , 0  };
int16_t chuizhi_piancha_fs[12]= {    0 , 0 , 0 , 0 , 0  };
int16_t chuizhi_piancha[12]= {    0 , 0 , 0 , 0 , 0  };  // Ã¿´Î²ÉÍêÑù£¬¼ÆËã³öÀ´µÄ´¹Ö±µç¸ÐµÄ  Æ«²î
uint8_t chuizhi_piancha_flag[10]={    0 , 0 , 0 , 0 , 0  };  // ÉÏ´Î´¹Ö±µç¸ÐµÄÆ«²î
int16_t chuizhi_jiaquan[12];
int16_t shuiping_piancha_xishu=0 , chuizhi_piancha_xishu=0 , xiezhi_piancha_xishu=0 ;
uint8_t piancha_E_qiujie_flag=7 ;
//=======================servo ¡ª¡ª mohu¡ª¡ª¶æ»ú¡ª¡ª===============================//
#define   servo_Upper     830   // ¶æ»úÊä³öÖµ  ÉÏÆ«²î
#define   servo_Lower    -830   // ¶æ»úÊä³öÖµ  ÉÏÆ«²î
uint16_t  duoji_zhongzhi =6420 ,  duoji_zhongzhi_value =6420;// 6915 //      
uint16_t  servo_kp = 15 ;   // ¶æ»ú PD ¿ØÖÆÖÐµÄ P Öµ
uint16_t  servo_kd = 0 ;     // ¶æ»ú PD ¿ØÖÆÖÐµÄ D Öµ
uint16_t  Fuzzy_Kp_cunchu[7] ={ 0,0,0,0,0,0,0 } ;  // ´æ´¢Ä£ºý¿ØÖÆÖÐ Kp ¼ÆËã¹ý³ÌÖÐµÄÖµ£¬²¢´æ´¢ÉÏÒ»´ÎµÄÖµ
int32_t  Fuzzy_Kd_cunchu[7] ={ 0,0,0,0,0,0,0 } ;  // ´æ´¢Ä£ºý¿ØÖÆÖÐ Kd ¼ÆËã¹ý³ÌÖÐµÄÖµ£¬²¢´æ´¢ÉÏÒ»´ÎµÄÖµ
int16_t servo_Fuzzy_Error[7] = { -87 ,  -65 , -27 ,  0 , 27 ,  65 , 87 } ; // Æ«²î·ÖÇø     Ä¿Ç°¾õµÃÁ½Í·Ð¡ÖÐ¼ä´ó£¬Ð§¹û½ÏºÃ //   Ö±µÀ²»ÎÈ  kp 80 , 69 , 42 , 0 , 42 , 62 , 80 
//int16_t servo_Fuzzy_Error[7] = { -87 ,  -50 , -25 ,  0 , 25 ,  50 ,  87 } ; // Æ«²î·ÖÇø     Ä¿Ç°¾õµÃÁ½Í·Ð¡ÖÐ¼ä´ó£¬Ð§¹û½ÏºÃ //   Ö±µÀ²»ÎÈ  kp 80 , 69 , 42 , 0 , 42 , 62 , 80 

 //int8_t servo_Fuzzy_Error_c[7] = { -9 , -6 , -3 , 0 , 3 , 6 , 9 } ;     // Æ«²î±ä»¯ÂÊ   ·ÖÇø
int8_t servo_Fuzzy_Error_c[7] = { -15 , -11 , -7 , 0 , 7 , 11 , 15 } ;     // Æ«²î±ä»¯ÂÊ   ·ÖÇø
                                        //  -87 ,  -51 , -20 ,  0 , 20 ,  51 ,  87  // 110 , 95 , 80 , 40 , 80 , 95 , 110  // ¿ÉÒÔ
                                                             //  -90 ,  -59 , -27 ,  0 , 27 ,  59 ,  90    //120 , 100 , 55 , 30 , 55 , 100 , 120  ÍäµÀÓÐµãÍí
const uint16_t servo_Fuzzy_Kp_dan[7] = { 75 , 65 ,  40 , 0 , 40,65 , 75} ;  // KpÇøÓò´óÐ¡   70 , 50 , 20 , 10 , 20 , 50 , 70
//76 , 63 ,  45 , 0 , 45,63 , 76
/*const uint8_t servo_Fuzzy_Kp_rule[7][7] =       // Kp ¹æÔò±í
{///  0--1--2--3--4--5--6                                                                                                                                //
	6 , 6 , 5 , 5 , 4 , 4 , 3 ,//0     //
	6 , 6 , 5 , 5 , 4 , 3 , 3 ,//1     //
	5 , 5 , 5 , 4 , 3 , 2 , 1 ,//2     //
	5 , 5 , 4 , 3 , 2 , 1 , 1 ,//3     //
	5 , 4 , 3 , 2 , 1 , 1 , 1 ,//4     //
	3 , 3 , 2 , 1 , 1 , 0 , 0 ,//5     //
	3 , 2 , 2 , 1 , 1 , 0 , 0 //6      //
};*/

const uint8_t servo_Fuzzy_Kp_rule[7][7] =       // Kp ¹æÔò±í
{  ///  0---1---2---3---4---5---6                                          
	0 , 0     , 0 , 0 , 1 ,     2 , 3 ,//0     //
	0 , 0     , 1 , 1 , 2 ,     3 , 3 ,//1     //      
	
	0 , 1     , 1 , 2 , 3 ,     3 , 4 ,//2     //
	1 , 2     , 2 , 3 , 4 ,     4 , 5 ,//3     //
       2 , 3     , 3 , 4 , 5 ,     5 , 6 ,//4     //
        
	3 , 3     , 4 , 5 , 5 ,     6 , 6 ,//5     //
	3 , 4     , 5 , 6 , 6 ,     6 , 6  //6     //
};



/*
const uint8_t servo_Fuzzy_Kp_rule[7][7] =       // Kp ¹æÔò±í
{///  0--1--2--3--4--5--6                                                                                                                                //
	6 , 6 , 5 , 5 , 4 , 4 , 3 ,//0     //
	6 , 6 , 5 , 5 , 4 , 3 , 3 ,//1     //
	5 , 5 , 5 , 4 , 3 , 2 , 1 ,//2     //
	5 , 4 , 4 , 3 , 2 , 1 , 1 ,//3     //
	4 , 4 , 3 , 2 , 2 , 1 , 1 ,//4     //
	3 , 3 , 2 , 1 , 1 , 1 , 0 ,//5     //
	3 , 2 , 2 , 1 , 1 , 0 , 0 //6      //
};*/




 //   const uint16_t servo_Fuzzy_Kd_dan[7] = { 1 , 1 , 1 , 1 , 1 , 1 , 1 } ;    // Kd ÇøÓò´óÐ¡
 //  const uint32_t servo_Fuzzy_Kd_dan[7] = { 5000 , 4000 , 3000 , 1000 , 3000 , 4000 , 5000 } ;
   const uint16_t servo_Fuzzy_Kd_dan[7] = { 90 , 70 , 30 , 10 , 30 , 70 , 90 } ;

 //   const uint16_t servo_Fuzzy_Kd_dan[7] = { 2900 , 2500 , 2300 , 2000 , 2300 , 2500 , 2900 } ;
/*const uint8_t servo_Fuzzy_Kd_rule[7][7] =      //  Kd  ¹æÔò±í
{ //   0--1--2--3--4--5--6
	4 , 4 , 3 , 3 , 3 , 6 , 6 ,  //  0
       2 , 2 , 3 , 4 , 4 , 4 , 5 ,  //  1
       6 , 6 , 1 , 2 , 3 , 4 , 5 ,  //  2
       0 , 1 , 2 , 3 , 4 , 5 , 6 ,  //  3   
       1 , 2 , 3 , 4 , 5 , 0 , 0 ,  //  4
       1 , 2 , 2 , 2 , 3 , 4 , 4 ,  //  5
       4 , 4 , 3 , 3 , 3 , 6 , 6    //  6
}; */


const uint8_t servo_Fuzzy_Kd_rule[7][7] =      //  Kd  ¹æÔò±í
{  ///  0---1---2---3---4---5---6                                          
	0 , 0     , 0 , 0 , 1 ,     2 , 3 ,//0     //
	0 , 0     , 1 , 1 , 2 ,     3 , 3 ,//1     //      
	
	0 , 1     , 1 , 2 , 3 ,     3 , 4 ,//2     //
	1 , 2     , 2 , 3 , 4 ,     4 , 5 ,//3     //
       2 , 3  , 3 , 4 , 5 ,     5 , 6 ,//4     //
        
	3 , 3     , 4 , 5 , 5 ,     6 , 6 ,//5     //
	3 , 4     , 5 , 6 , 6 ,     6 , 6  //6     //
};



/*
const uint8_t servo_Fuzzy_Kd_rule[7][7] =      //  Kd  ¹æÔò±í
{ //   0--1--2--3--4--5--6
	4 , 4 , 3 , 3 , 3 , 6 , 6 ,  //  0
       2 , 2 , 2 , 2 , 3 , 2 , 5 ,  //  1
       6 , 6 , 1 , 2 , 3 , 4 , 5 ,  //  2
       0 , 1 , 1 , 2 , 3 , 4 , 5 ,  //  3
       0 , 1 , 2 , 2 , 3 , 4 , 4 ,  //  4
       1 , 2 , 2 , 2 , 3 , 4 , 4 ,  //  5
       4 , 3 , 3 , 3 , 3 , 6 , 6    //  6
}; */

uint32_t  Fuzzy_speed_cunchu_zuo[7] ={ 0,0,0,0,0,0,0 } ;  // ´æ´¢Ä£ºý¿ØÖÆÖÐ Kp ¼ÆËã¹ý³ÌÖÐµÄÖµ£¬²¢´æ´¢ÉÏÒ»´ÎµÄÖµ
uint32_t  Fuzzy_speed_cunchu_you[7] ={ 0,0,0,0,0,0,0 } ;  // ´æ´¢Ä£ºý¿ØÖÆÖÐ Kd ¼ÆËã¹ý³ÌÖÐµÄÖµ£¬²¢´æ´¢ÉÏÒ»´ÎµÄÖµ
                                                                 //   -3      -2       -1        0         1       2          3   
const uint16_t speed_Fuzzy_zuo_dan[7] = { 2350 , 2250 , 2100 , 2300 , 2100 , 1950 , 1800 } ; // 
const uint16_t speed_Fuzzy_you_dan[7] = { 1800 , 1950 , 2100 , 2300 , 2100 , 2250 , 2350 } ; 

//const uint16_t speed_Fuzzy_zuo_dan[7] = { 2450 , 2350 , 2200 , 2350 , 2150 , 2050 , 1900 } ; // 
//const uint16_t speed_Fuzzy_you_dan[7] = { 1900 , 2050 , 2150 , 2350 , 2200 , 2350 , 2450 } ; 

//const uint16_t speed_Fuzzy_zuo_dan[7] = { 2300 , 2250 , 2100 , 2200 , 2100 , 1950 , 1800 } ; // ÎÈ
//const uint16_t speed_Fuzzy_you_dan[7] = { 1800 , 1950 , 2100 , 2200 , 2100 , 2250 , 2300 } ; 

//const uint16_t speed_Fuzzy_zuo_dan[7] = { 2550 , 2520 , 2500 , 2800 , 2400 , 2250 , 2150 } ; // 37.26m----15.719s----2.370m/s
//const uint16_t speed_Fuzzy_you_dan[7] = { 2150 , 2250 , 2400 , 2800 , 2500 , 2520 , 2550 } ; 

//const uint16_t speed_Fuzzy_zuo_dan[7] =  { 2650 , 2620 , 2600 , 2800 , 2500 , 2350 , 2250 } ; // 37.26m----15.23s----m/s
//const uint16_t speed_Fuzzy_you_dan[7] =  { 2250 , 2350 , 2500 , 2800 , 2600 , 2620 , 2650 } ; 

//const uint16_t speed_Fuzzy_zuo_dan[7] =  { 2750 , 2720 , 2700 , 3200 , 2600 , 2450 , 2350 } ; // 14.78s--15.2m/s
//const uint16_t speed_Fuzzy_you_dan[7] =  { 2350 , 2450 , 2600 , 3200 , 2700 , 2720 , 2750 } ; 

uint16_t  speed_temp_11[4] = { 0,0,0,0 } , speed_temp_12[4] = { 0,0,0,0 } , speed_temp_21[4] = { 0,0,0,0 } , speed_temp_22[4] = { 0,0,0,0 } ;
 
//======================================================================//
//==========================È«¾Ö±äÁ¿======================================//
int32_t  i_quan , j_quan , k_quan ;
uint8_t xianshi_flag = 0 ;     //Ñ¡ÔñÏÔÊ¾µÄÊý¾Ý±êÖ¾Î»
uint32_t motor_time = 0 ;     //PIT¼ÆÊýÆ÷
 
//====================================================================//
//======================== Ëã·¨ ¡ª¡ª±êÖ¾Î»  =================================== //
uint8_t po_dao_flag = 0 , daoda_podao_flag = 0 ;  //ÆÂµÀ
uint8_t zhi_jiao_flag =0;  // ÅÐ¶ÏÊÇ·ñµ½´ï  Ö±½Ç´¦
uint8_t zhi_jiao_charge =0;
uint8_t zhi_jiao_check=0;
int8_t  zhijiao_zhuanxiang = 0 ; //   Ö±½Ç×ªÏò
uint8_t ru_shi_zi_flag = 0 ;  //   ½øÈëÊ®×Ö
uint8_t chu_shi_zi_flag = 0 ; // Àë¿ªÊ®×Ö
uint8_t zhi_dao_flag = 0 ;  // ½øÈëÖ±µÀ   
uint8_t little_S_flag = 0 ;  // ½øÈëÐ¡  S
uint8_t bigbig_S_flag = 0 ; // ½øÈë´ó  S 

int8_t K_one[50] ;  // 1  Ð±ÂÊ
int8_t K_two[50] ;  // 2 
int8_t K_three[50] ;// 3
int8_t K_four[50] ; // 4
int8_t K_five[50] ; //  5
//wandao_come_in =1 ×ó½øÍä wandao_come_in =2  ÓÒ½øÍä
uint8_t  wandao_come_in =0;

//wandao_go_out =1 ×ó³öÍä wandao_go_out =2  ÓÒ³öÍä 
uint8_t  wandao_go_out = 0 ;  

uint8_t pian_yi_zuo_flag = 0 ; // 
uint8_t pian_yi_zhong_flag = 0 ; // 
uint8_t pian_yi_you_flag = 0 ; //
uint8_t shangpo_flag = 0 ; // ÉÏÆÂ
uint32_t shangpo_flag_distance;
uint32_t lost_time = 0 ; //¶ªÐÅºÅ´ÎÊý
uint8_t stop_smartcar_flag=0;
uint8_t stop_smartcar_timesflag=0;

uint8_t chu_zuo_wandao_flag=0;
uint8_t zuo_wandao_flag=0;
uint8_t zhi_jiao_times=0;
int16_t  speed_ave;
int16_t servo_max;
int16_t servo_min;
int16_t jiao_biao_max;
int16_t jiao_biao_min;
uint8_t Road_Type_last;
uint8_t Road_Type[10]={0,0,0,0,0};
uint8_t special_Road_Type[10]={0 };   
 
uint8_t car_pid=0; 
uint8_t protect_Road_Type[10]={0,0,0,0,0};
uint8_t protect_Road_Type_Times[10]={0 };
  
uint8_t Cross_flag=0;                
uint16_t Road_Type_Times[10]={0 }; 
int16_t chui_zhi_daingan[50]={0};
int16_t KK_shuiping[10]={0 };
int16_t KK_chuizhi[10]={0 };
int16_t KK_xiezhi[10]={0 };
int16_t position_flag[10]={0};
int16_t position_flag_times[10]={0};
int16_t debug_value;

//=======================================================================//
#define   motor_pwmR    EMIOS_0.CH[13].CBDR.R //ÓÒ±ß  Maximal value is 8000
#define   motor_pwmL    EMIOS_0.CH[14].CBDR.R //×ó±ß  Maximal value is 8000
//#define   servo_pwm     EMIOS_0.CH[21].CBDR.R     //Maximal value is 53333

#define   servo_pwm     EMIOS_0.CH[22].CBDR.R     //Maximal value is 53333


//============================²âËÙ±äÁ¿=================================//

uint32_t smartcar_speed_L ; // ×óµç»úËÙ¶È
uint32_t smartcar_speed_R ; //  ÓÒµç»úËÙ¶È
uint8_t fashu;
uint32_t per_pulse_R=0;//¾àÀë
uint32_t per_pulse_L=0;
uint32_t average_distance;
//ÓÒ±ß
uint16_t R_count_now=0;
uint16_t R_count_last=0;
uint32_t R_count;
uint32_t R_period;

uint8_t  R_period_sum_times = 1;
uint16_t R_smartcar_speed_0=0;
uint16_t R_smartcar_speed_1=0;
uint32_t R_smartcar_speed_temp;
uint32_t R_smartcar_speed_time = 0;
uint32_t R_period_sum = 0;
int32_t zuo_shu_chu;
int32_t you_shu_chu; 
//×ó±ß
uint16_t L_count_now=0;
uint16_t L_count_last=0;
uint32_t L_count;
uint32_t L_period;

uint8_t  L_period_sum_times = 1;
uint16_t L_smartcar_speed_0=0;
uint16_t L_smartcar_speed_1=0;
uint32_t L_smartcar_speed_temp;
uint32_t L_smartcar_speed_time = 0;
uint32_t L_period_sum = 0;
uint32_t piancha_E_jisuan;
uint8_t EMIOS0_CH0_overflow_times_L;
uint8_t EMIOS0_CH0_overflow_times_R;

uint16_t CarSpeed_SET_R;
uint16_t CarSpeed_SET_L;
uint16_t last_CarSpeed_SET_R;
uint16_t last_CarSpeed_SET_L;

uint16_t zhidao_speed;
uint16_t wandao_speed;
uint16_t zhijiao_speed;
uint16_t k1;
uint16_t wandao_CarSpeed_SET_L;
uint16_t wandao_CarSpeed_SET_R;
 
uint16_t flage_tiaosu_r=0;
uint16_t flage_tiaosu_l=0;
uint8_t speed_dangwei;
//=====================================================================//
//=====================================================================//
//============================×ªÏò±äÁ¿=================================//
uint32_t per_distance_R=0;//¾àÀë
uint32_t per_distance_L=0;

uint32_t sum_distance_R=0;
uint32_t sum_distance_L=0;

uint32_t average_distance_L=0;
uint8_t zhuanxaing;
uint8_t protect_flag;

int16_t  speed_difference;
uint16_t  speed_difference_flag;

 int32_t servo_output;
 int32_t servo_output_last[20]={0} ;  //  ´æ´¢ÉÏ¼¸´Î  servo_output  Êä³öÁ¿
 int32_t cha_su_value;
int16_t Duoji_value;

int16_t start_pid=0;
int16_t pid_times[5]={0} ;
int16_t pid_flag[5]={0} ;
int16_t start_pid_time;
int16_t stop_flag=0;
int16_t smart_speed;
//**************************   µç»ú  PID  ½á¹¹Ìå  ¶¨Òå  ********************************//
struct motor_PID 
{
          int32_t  Proportion ;   // ±ÈÀý³£Êý  
          int32_t  Integral ;   // »ý·Ö³£Êý 
          int32_t  Derivative ;   // Î¢·Ö³£Êý
           
          int32_t  SumError;         //Îó²îÀÛ¼Æ 
                   // Éè¶¨Ä¿±ê Desired Value 
          int32_t  iError ;   // Æ«²î
          int32_t  iIncpid ;  // ÔöÁ¿             
          int32_t  output ; // µç»úÊä³öÖµ
          int32_t  last_output ; // µç»úÉÏ´ÎÊä³öÖµ
          int32_t  LastError;           //Error[-1] 
          int32_t  PrevError;           //Error[-2]

          int32_t  Max_iError ; //ÏÞÖÆiError£¬·À³¬µ÷
          int32_t  Min_iError ;
          int32_t  Max_iIncpid ;  //ÏÞÖÆÔöÁ¿
          int32_t  Min_iIncpid ;
          int32_t  Max_otput ;  //ÏÞÖÆÊä³öÁ¿
          int32_t  Min_otput ;
             
//=============  ÓÒÂÖ  ============//        
          int32_t  SumError_other;         //Îó²îÀÛ¼Æ 
          int32_t  iError_other; 
          int32_t  iIncpid_other;             
          int32_t  output_other;
          int32_t  last_output_other;
          int32_t  LastError_other;           //Error[-1] 
          int32_t  PrevError_other;           //Error[-2]

} PID;



//=====================================================================//
////////////////////////////ÊýÂë¹Ü³õÊ¼»¯///////////////////////////////
//====================================================================//
struct {
        uint8_t digist[7];
        uint8_t addr[7];
}Digist;

#define   load_port_config            SIU.PCR[PCR76_PE12].R 
#define   din_port_config             SIU.PCR[PCR34_PC2].R  
#define   clk_port_config             SIU.PCR[PCR5_PA5].R 
#define   ok_port_config              SIU.PCR[PCR35_PC3].R
#define   increase_port_config        SIU.PCR[PCR34_PC2].R  
#define   decrease_port_config        SIU.PCR[PCR5_PA5].R  
#define   load_port_out               SIU.GPDO[PCR76_PE12].R
#define   din_port_out                SIU.GPDO[PCR34_PC2].R
#define   clk_port_out                SIU.GPDO[PCR5_PA5].R
#define   ok_port_out                 SIU.GPDO[PCR35_PC3].R 
#define   increase_port_out           SIU.GPDO[PCR34_PC2].R
#define   decrease_port_out           SIU.GPDO[PCR5_PA5].R
#define   load_port_in                SIU.GPDI[PCR76_PE12].R
#define   din_port_in                 SIU.GPDI[PCR34_PC2].R
#define   clk_port_in                 SIU.GPDI[PCR5_PA5].R
#define   ok_port_in                  SIU.GPDI[PCR35_PC3].R 
#define   increase_port_in            SIU.GPDI[PCR34_PC2].R
#define   decrease_port_in            SIU.GPDI[PCR5_PA5].R


#define  port_output   0x0200                                                           //*
#define  port_input    0x0103   
#define   in_put                     0
#define   out_put                    1
#define   low                       0
#define   high                      1
#define  write_data       1                                                             //*
#define  write_command    0
//Register Address Map              REGISTER                   HEX CODE 
#define                             No_Op_addr                 0x00
#define                             Decode_Mode_addr           0x09
#define                             Intensity_addr             0x0A
#define                             Scan_Limit_addr            0x0B
#define                             Shutdown_addr              0x0C
#define                             Display_Test_addr          0x0F
                                    //REGISTER DATA            HEX CODE
#define                             Shutdown_Mode              0x00
#define                             Normal_Mode                0x01
#define                             BCD_Mode                   0xFF
#define                             nodecode_Mode              0x00
#define                             Scan_Limit                 0x07
#define                             Intensity_duty             0x00
#define                             Normal_Operation           0x00
#define                             Display_Test_Mode          0x01
#define                             DIG_1                0x01        // ¶¨ÒåÊýÂë¹Ü1 register 
#define                             DIG_2                0x02        // ¶¨ÒåÊýÂë¹Ü2 register 
#define                             DIG_3                0x03        // ¶¨ÒåÊýÂë¹Ü3 register 
#define                             DIG_4                0x04        // ¶¨ÒåÊýÂë¹Ü4 register 
#define                             DIG_5                0x05        // ¶¨ÒåÊýÂë¹Ü5 register 
#define                             DIG_6                0x06        // ¶¨ÒåÊýÂë¹Ü6 register 
#define                             DIG_7                0x07        // ¶¨ÒåÊýÂë¹Ü7 register 
#define                             DIG_8                0x08        // ¶¨ÒåÊýÂë¹Ü8 register 
/*---------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------*/
/*------------------------------------------------------------------------*/
                         /*ÊýÂë¹Ü*/
/********************************************************************************
*  1     2       3         4        5       6
* load   ok    din(+)    clk(-)    gnd     v++
* PTM5   PTM4  PTM3      PTM2      GND      5V
/********************************************************************************/
void Write7219(unsigned char Register_addr,unsigned char date)
{
  unsigned char i,j=0,addr=0,d=0;
 load_port_config = port_output;
  din_port_config = port_output;
  clk_port_config = port_output;
  addr = Register_addr;
  d = date;
  load_port_out = low;
  for(i=0;i<8;i++) 
  {
     clk_port_out = low;
     j = (0x80&addr);
     if(j==0x80)
         din_port_out = 1;
     else
         din_port_out = 0;
     addr = (addr<<1);
     
     clk_port_out = high;
  }
  for(i=0;i<8;i++) 
  {
     clk_port_out = low;
     j = (0x80&d);
     if(j==0x80)
         din_port_out = 1;
     else
         din_port_out = 0;
     d = (d<<1);
      
     clk_port_out = high;
  }
  load_port_out = high;
}
/*******************************************************************/
void init_max7219(void)
{
  unsigned char i;
  
  Write7219(Shutdown_addr,0X00);       //Í£»ú
  Write7219(Decode_Mode_addr,0XFF);    //ÒëÂëÄ£Ê½
  Write7219(Intensity_addr,0X00);     //ÁÁ¶È
  Write7219(Scan_Limit_addr,0X07);        //ÏÔÊ¾Î»Êý
  Write7219(Shutdown_addr,0X01);         //Õý³£ÏÔÊ¾Ä£Ê½
  Write7219(Display_Test_addr,0X00);     //²âÊÔ
   for(i=0;i<8;i++)
  {
    Digist.digist[i]=0;
    Digist.addr[i+1] = i+1;
	  Write7219(Digist.addr[i+1],Digist.digist[i]);
   }
 }
/*------------------------------------------------------*/
  
/*****************************************************************/
uint32_t LED_write( uint16_t A,  uint16_t i)   //  609
{
uint16_t  k,y ,x ,m , n;
switch(A)
        {   
            case 0:  
        	Write7219(DIG_8,0);  Write7219(DIG_7,0);
        	Write7219(DIG_6,0); Write7219(DIG_5,0);break;
        	
        	case 1:  
        	Write7219(DIG_8,0x0A);  Write7219(DIG_7,0x0B);
        	Write7219(DIG_6,0x0C); Write7219(DIG_5,0x0A);break;
        	
           	case 2: 
           	Write7219(DIG_8,0x0A);  Write7219(DIG_7,0x0B);
        	Write7219(DIG_6,0x0D); Write7219(DIG_5,0x0A);break; 
        	    
      	    case 3: 
      	    Write7219(DIG_8,0x0A);  Write7219(DIG_7,0x0E);
        	Write7219(DIG_6,0x0C); Write7219(DIG_5,0x0A);break;
        	
     default:	
   	 break;
        	
        }
      k=i;
      n = k/1000 - k /10000 *10  ;   //»ñµÃÇ§Î»
      m = k /100 - k /1000 *10    ;   //»ñµÃ°ÙÎ»
      x = k /10 - k /100 *10      ;   //»ñµÃ¸öÎ»
      y = k % 10                     ;   //»ñµÃ¸öÎ»
           Write7219(DIG_4,y);
            Write7219(DIG_3,x);
            Write7219(DIG_2,m);
            Write7219(DIG_1,n);   
     }
void Show_Data(uint16_t i)
{
	uint16_t  k,y ,x ,m , n;
	k=i;
    n = k/1000 - k /10000 *10  ;   //»ñµÃÇ§Î»
    m = k /100 - k /1000 *10    ;   //»ñµÃ°ÙÎ»
    x = k /10 - k /100 *10      ;   //»ñµÃ¸öÎ»
    y = k % 10                   ;   //»ñµÃ¸öÎ»
    Write7219(DIG_4,y);
    Write7219(DIG_3,x);
    Write7219(DIG_2,m);
    Write7219(DIG_1,n);
}


    
void   Show_Me_Data(uint16_t number_test,uint8_t change,uint8_t flag)
{ 
  uint8_t k=flag;

   while (k==flag)
   {   
       increase_port_config = port_input;
       decrease_port_config = port_input;
       ok_port_config = port_input; 
 
	   while(STM.CH[1].CIR.B.CIF == 0){}
       STM.CH[1].CMP.R = STM.CH[1].CMP.R + 360000;		 // Set STM CH1 compare register
       STM.CH[1].CIR.B.CIF = 1;
	   
       if (decrease_port_in == low)  //¼õÉÙ
           number_test-=change;
       
       if (increase_port_in == low)  // Ôö¼Ó
           number_test+=change;
       
       if(ok_port_in==low)       //OK
       {
	  	k+=1;
	  	switch(flag)
	  	{
 		   
//================================================//		   
		    case 0:
		         CarSpeed_SET_L=number_test;
 	  		 break;
//==============================================//	  		 
	  		case 1:
	  		     speed_dangwei=number_test;
	  		    
  	  		 break;
//=============================================//	  		 
	  		case 2 :
	  		      start_pid_time=number_test;
 	  		break;		  
//=============================================//	  		  
	  		case 3:
	  		      CCD_HUAT_dis=number_test;
 	  		  break;
//=============================================//	  		  
	        case 4 :
	      		    guiyi_wugediangan_MAX=number_test;   
 	       	 break;
//==============================================//	       	 
	        case 5 :
	                 piancha_E_qiujie_flag=number_test ;   
 	       	 break;
 //==============================================//	       	 
 	       	  case 6 :
	       	      diangan_biaoding_xuhao=number_test;
	       	      flag_2=number_test;
 	       	 break;
//===============================================// 
            case   13://    ÊÇ·ñÓÐÕÏ°­ 
   	                  CCD_xuanze_flag = number_test;    
   	        break;   	        
//===============================================// 
            case   14://    ÊÇ·ñÓÐÕÏ°­ 
   	                  duoji_zhongzhi = number_test;    
   	        break;   	      
//===============================================// 
             case   15://    ÊÇ·ñÓÐÕÏ°­ 
   	                  CCD_juli_dm_she_ding_juli[1] = number_test;    
   	        break;   	 
//===============================================// 
            case   16://    ¾àÀë²âËÙ
   	                  CCD_juli_dm_she_ding_juli[0] = number_test;    
   	        break;   	   	              
//===============================================// 
             case   17://    µ½´ï¾àÀëºó¿ªÆô  CCD
   	                  CCD_juli_dm_she_ding_juli[2] = number_test;    
   	        break;   	 
//===============================================// 
            case   18://    µ½´ï¾àÀëºó¹Ø±Õ  CCD
   	                  CCD_juli_dm_she_ding_juli[3] = number_test;    
   	        break;   	   	              
//===============================================// 
            case   19://    ¾àÀë²âËÙ
   	                  CCD_baoguang_shijian = number_test;    
   	        break;
//===============================================// 
            case   20://    µ½´ï¾àÀëºó¹Ø±Õ  CCD
   	                  piancha_E_jisuan = number_test;    
   	        break;   	   	              
//===============================================// 
 
   	         case   22://    ¾àÀë²âËÙ
   	                  k1 = number_test;    
   	        break;   
   	        	   	       	 
//==============================================//	       	 
       
       	default:									   
   	     break;										   
	  	}  											   
       }											   
      LED_write(flag,number_test);  
   }
}	
//================ ×îÐ¡¶þ³Ë·¨ ÄâºÏ ================= //
int16_t er_cheng_fa_ni_he( uint8_t N , int16_t er_cheng_shuju[50] )
{
	uint8_t i ;
	int32_t Sum_xy=0 , Sum_x=0 , Sum_y=0 , Sum_xx=0 ;
	int32_t K_fenzi=0 , K_fenmu=0 , K=0 ;
	for( i=1 ; i<=N ; i++ )
	{
		Sum_xy=Sum_xy+i*er_cheng_shuju[N-i] ;
		Sum_x = Sum_x+i ;
		Sum_y = Sum_y+er_cheng_shuju[i] ;
		Sum_xx=Sum_xx+i*i ;
	}
	
	K_fenzi=N*Sum_xy - Sum_x*Sum_y ;
	K_fenmu=N*Sum_xx - Sum_x*Sum_x ;
	if( K_fenmu==0 )  K_fenmu=1 ;
	
	K=N*K_fenzi/K_fenmu ;
	
	return K ;
	
}

//==============================  ÇóÁ½¸öÊý¾ÝÖ®²îµÄ¾ø¶ÔÖµ ============================//
uint16_t ABS(int16_t x,int16_t y)
{
	if(x>y)         return ( x-y ) ;
	else   	    return ( y-x ) ;
}
//=================  ÅÐ¶ÏÁ½¸öÊýÖ®²îµÄ  ·ûºÅ  ================//
int16_t fu_hao(int16_t xx)
{
      if(xx>0)	    return(1) ;
      if(xx<0)      return(-1) ;
}
//================  ½«Êý¾ÝÈ¡Õý  ====================//
int16_t qu_zheng(int16_t xxx)
{
	if(  xxx >= 0  )    return(xxx) ;
	else                   return(-xxx) ;
}
 
//**********************************  STM ¡ª¡ª ÑÓÊ±  *************************//
void delay500ms( void )
{
    while(STM.CH[1].CIR.B.CIF == 0){}
    STM.CH[1].CMP.R = STM.CH[1].CMP.R + 1000000; // Set STM CH1 compare register 
    STM.CH[1].CIR.B.CIF = 1;/**/
}
void delay400ms( void )
{
    while(STM.CH[1].CIR.B.CIF == 0){}
    STM.CH[1].CMP.R = STM.CH[1].CMP.R + 800000; // Set STM CH1 compare register 
    STM.CH[1].CIR.B.CIF = 1;/**/
}
void delay10ms( void )
{
    while(STM.CH[1].CIR.B.CIF == 0){}
    STM.CH[1].CMP.R = STM.CH[1].CMP.R + 20000; // Set STM CH01 compare register 
    STM.CH[1].CIR.B.CIF = 1;/**/
}
void delay5ms( void )
{
    while(STM.CH[1].CIR.B.CIF == 0){}
    STM.CH[1].CMP.R = STM.CH[1].CMP.R + 10000; // Set STM CH01 compare register 
    STM.CH[1].CIR.B.CIF = 1;/**/
}
void delay1ms( void )
{
    while(STM.CH[1].CIR.B.CIF == 0){}
    STM.CH[1].CMP.R = STM.CH[1].CMP.R + 2000; // Set STM CH01 compare register 
    STM.CH[1].CIR.B.CIF = 1;/**/
}




//==================================== ÖÐ¶Ï¡ª¡ªÑÓÊ± ===================================//
void STM_CH0_isr()
{
       STM.CH[0].CMP.R = STM.CH[0].CMP.R + 1000000;  /* Set STM CH0 compare register */
      STM.CH[0].CIR.B.CIF = 1; 	
}  
void EMIOS0_CH16_17_isr(void)
{
	EMIOS_0.CH[16].CSR.B.FLAG = 1;
	EMIOS0_CH0_overflow_times_L++;
    EMIOS0_CH0_overflow_times_R++;
    
    
    
    if(EMIOS0_CH0_overflow_times_R>10)
    {
        smartcar_speed_R = 0;
        EMIOS0_CH0_overflow_times_R = 0;
        R_smartcar_speed_time = 0;
        R_period_sum_times = 1;
        R_smartcar_speed_0=0;
        R_period_sum=0;
        R_smartcar_speed_temp=0;
    
     } 
     
    
     if(EMIOS0_CH0_overflow_times_L>10)
    {
         
        smartcar_speed_L = 0;
        EMIOS0_CH0_overflow_times_L = 0;
        L_smartcar_speed_time = 0;
        L_period_sum_times = 1; 
        L_period_sum=0;
        L_smartcar_speed_0=0;
        L_smartcar_speed_temp=0;
                 
     }
     
  //SIU.GPDO[PCR38_PC6].R =!SIU.GPDO[PCR38_PC6].R;
}



 

//==========================ÖÜÆÚÖÐ¶Ï===================================//

void PIT_CH1_isr(void)
{

    motor_time++;
    PIT.CH[1].TFLG.B.TIF = 1;
 	
	if(motor_time==CCD_baoguang_shijian+CCD_stoptimes)
	          CCD_times=1 ;//CCDÆØ¹âÊ±¼ä         
 
    
    if(motor_time==start_pid_time) 
      start_pid=1;        //ÂúPºó¿ªÊ¼PIDµ÷ËÙ
    
    
    
   // if(motor_time>1500) stop_smartcar_flag=1;  
    

}
//=============================¼ÇÂö³åÖÐ¶Ï=============================//
//================================×ó±ß================================//
void EMIOS1_GFR_F20_F21_ISR(void)
{  
    CCD_juli_zuo[0]++ ;
    L_smartcar_speed_time++;
    
    L_count_last=L_count_now;
    L_count_now = EMIOS_0.CH[20].CADR.R;
    
    if(L_count_last<50&&EMIOS0_CH0_overflow_times_L>0)
    {
    	  EMIOS0_CH0_overflow_times_L-=1;

    }   
   
    if(L_count_now<L_count_last&&EMIOS0_CH0_overflow_times_L==0)
    {
    	  EMIOS0_CH0_overflow_times_L+=1;

    }
    
    L_count = L_count_now+EMIOS0_CH0_overflow_times_L*65535-L_count_last;
    if(L_count<370)
    {
    	L_count = 370;	
    }
    
    L_period_sum = L_period_sum+L_count;//Âö³åÀÛ¼Ó

    if(L_period_sum_times==L_smartcar_speed_time)
    {
        L_period = L_period_sum/L_period_sum_times;
        //±àÂëÆ÷26T,µç»ú9T,³µÂÖ24T,Í¬Öá³ÝÂÖ´óµÄ28TÐ¡µÄ10T
    	smartcar_speed_L =43*157*200000/L_period/84;
    	// 10*25*3142*100/L_period/7; 
       ////43*157*100000/L_period/84 ´ó±àÂëÆ÷ 
        if(ABS(smartcar_speed_L,L_smartcar_speed_0)>2000) 
           {  
              smartcar_speed_L= L_smartcar_speed_0  ;
         
           }
 
          smartcar_speed_L=(L_smartcar_speed_1
                                +L_smartcar_speed_0
                                +smartcar_speed_L)/3;
           L_smartcar_speed_1=L_smartcar_speed_0;
           L_smartcar_speed_0=smartcar_speed_L;

     	
          L_smartcar_speed_time = 0;
          L_period_sum = 0;
          L_period_sum_times = 1+smartcar_speed_L/300;
                  	 
          flage_tiaosu_l=1;
            	 
    }

    EMIOS0_CH0_overflow_times_L = 0;
    EMIOS_0.CH[20].CSR.B.FLAG = 1;
 
    	
 
	
}
//===============================ÓÒ±ß=================================//
 void EMIOS0_GFR_F18_F19_ISR(void)
{

    CCD_juli_you[0]++ ;
    R_smartcar_speed_time++;
    
    R_count_last=R_count_now;
    R_count_now = EMIOS_0.CH[19].CADR.R;
    
    if(R_count_last<50&&EMIOS0_CH0_overflow_times_R>0)
    {
    	EMIOS0_CH0_overflow_times_R-=1;
    }
 
   if(R_count_now<R_count_last&&EMIOS0_CH0_overflow_times_R==0)
   {
       EMIOS0_CH0_overflow_times_R+=1;
    
   }
 
    R_count = R_count_now+EMIOS0_CH0_overflow_times_R*65535-R_count_last;
    if(R_count<370) 
    {
    	R_count = 370;	
    }
    
    R_period_sum = R_period_sum+R_count;//Âö³åÀÛ¼Ó
    
    if(R_period_sum_times==R_smartcar_speed_time)
    {
        R_period = R_period_sum/R_period_sum_times;
        //±àÂëÆ÷26T,µç»ú9T,³µÂÖ24T,Í¬Öá³ÝÂÖ´óµÄ28TÐ¡µÄ10T
    	smartcar_speed_R = 43*157*200000/R_period/84 ; 
         //32M=>100£¬8M=>25
       if(ABS(smartcar_speed_R,R_smartcar_speed_0)>2000) 
           {  
              smartcar_speed_R= R_smartcar_speed_0;
         
           }
      
          smartcar_speed_R=(R_smartcar_speed_1
                            +R_smartcar_speed_0
                            +smartcar_speed_R)/3;
                               
           R_smartcar_speed_1=R_smartcar_speed_0;
           R_smartcar_speed_0=smartcar_speed_R;

 
    	  R_smartcar_speed_time = 0;
          R_period_sum = 0;
          R_period_sum_times = 1+smartcar_speed_R/300;
    	  
          flage_tiaosu_r=1;

    	             	 
                      	 
    }
    
    EMIOS0_CH0_overflow_times_R = 0;
    EMIOS_0.CH[19].CSR.B.FLAG = 1;
}
 


void EMIOS1_GFR_F22_F23_ISR()
{  
    EMIOS_0.CH[22].CSR.B.FLAG = 1;
     	
}
//=============================µç»ú³õÊ¼»¯==================================//
void Dianji_Init( void )
{
    SIU.PCR[PCR40_PC8].R = 0x0200;  //GPIO[40] is output
    SIU.PCR[PCR44_PC12].R = 0x0200; //GPIO[44] is output 
    SIU.PCR[PCR41_PC9].R = 0x0200;  //GPIO[41] is output
    SIU.PCR[PCR47_PC15].R = 0x0200; //GPIO[47] is output  
        
    SIU.GPDO[PCR40_PC8].R = 0 ;       //0-1Õý×ª
    SIU.GPDO[PCR44_PC12].R =1 ;
    motor_pwmR = 0 ;
    
    SIU.GPDO[PCR41_PC9].R = 0 ;
    SIU.GPDO[PCR47_PC15].R = 1 ;
    motor_pwmL = 0 ; 	
}

void start_smartcar()
{
   motor_pwmL=7600;
   motor_pwmR=7600;	
}


void stop_smartcar()
{  
  
   if(stop_smartcar_flag ==1)
   {    
    
             CarSpeed_SET_L=0;
             CarSpeed_SET_R=0;
             smart_speed=(smartcar_speed_L+smartcar_speed_R)/2;
	              if(smart_speed>1200)
			      {
			          SIU.GPDO[PCR40_PC8].R = 1 ;       //1-0·´×ª
                      SIU.GPDO[PCR44_PC12].R = 0 ;
                      motor_pwmR=7600;
                      SIU.GPDO[PCR41_PC9].R = 1;
                      SIU.GPDO[PCR47_PC15].R = 0;
	                  motor_pwmL=7600;
 			      }
 			      
 			      else if(smart_speed>600)
 			      {
			          SIU.GPDO[PCR40_PC8].R = 1 ;       //1-0·´×ª
                      SIU.GPDO[PCR44_PC12].R = 0 ;
                      motor_pwmR= 3500;
                      
                      SIU.GPDO[PCR41_PC9].R = 1;
                      SIU.GPDO[PCR47_PC15].R = 0;
                      motor_pwmL= 3500;
 			      }
 			       
 			      else 
 			      {
			          SIU.GPDO[PCR40_PC8].R = 1 ;       //1-0·´×ª
                      SIU.GPDO[PCR44_PC12].R = 1 ;
                      motor_pwmR= 0;
                      
                      SIU.GPDO[PCR41_PC9].R = 1;
                      SIU.GPDO[PCR47_PC15].R = 1;
                      motor_pwmL= 0;
 			      }
   }
}

//=========================================================//
void stop_smartcar_check()
{  	
    if(EMIOS_0.CH[26].CSR.B.FLAG == 1) 
     {               
      	stop_smartcar_flag=1;
      	
      	//CCD_xuanze_flag=0;
      	EMIOS_0.CH[26].CSR.B.FLAG = 1;
     }//¼ì²âÊÇ·ñÓ¦¸ÃÍ£³µ
    if(motor_time<5000&&stop_smartcar_flag==1)
          stop_smartcar_flag=0;
   
}
//=========================PID³õÊ¼»¯=============================//
void PID_Init() 
{
            PID.SumError=0;          //Îó²îÀÛ¼Æ
           
            PID.Proportion=5600;        
            PID.Integral= 600;         
            PID.Derivative=50 ;       
            
            PID.iError=0;  
            PID.iIncpid=0;           
            
            PID.output=0;            
            PID.last_output=0;            

            PID.LastError=0;         //Error[-1]
            PID.PrevError=0;  

            PID.Max_iError=3500;      //ÏÞÖÆiError£¬·À³¬µ÷
            PID.Min_iError=-3500;
            
            PID.Max_iIncpid=8000;     //ÏÞÖÆÔöÁ¿
            PID.Min_iIncpid=-8000;
            
            PID.Max_otput=7600;         //ÏÞÖÆÊä³öÁ¿
            PID.Min_otput=-7600;

         //=========================//
             PID.SumError_other=0;         //Îó²îÀÛ¼Æ 
            
             PID.iError_other=0; 
             PID.iIncpid_other=0;             
             PID.output_other=0;
             PID.last_output_other=0;

             PID.LastError_other=0;           //Error[-1] 
             PID.PrevError_other=0;           //Error[-2]
}
 
//======================================================================//
//============================PIDµ÷ËÙ====================================//
int PID_change_L(int Current_Speed ,int SetPoint )
{ 
       //µ±Ç°Îó²î 
       PID.iError =  SetPoint - Current_Speed ;  //ÔöÁ¿¼ÆËã 
        
       //************ÏÞÖÆÌõ¼þ**************//     
          if( PID.iError >= PID.Max_iError )
          {    
              PID.iError = PID.Max_iError ;
          }                
          else if( PID.iError <= PID.Min_iError )
          {   
              PID.iError = PID.Min_iError ;  
          } 
          
          //**************************************************//
        PID.iIncpid = (PID.Derivative * (PID.PrevError + PID.iError - 2*PID.LastError )
                    + PID.Proportion * (PID.iError - PID.LastError ) 
                    + PID.Integral * PID.iError )/1000 ;
       
        //´æ´¢Îó²î£¬ÓÃÓÚÏÂ´Î¼ÆËã 
        PID.PrevError =  PID.LastError ; 
        PID.LastError =  PID.iError ; 
       
        //****** ********ÏÞÖÆÅÐ¶Ï*** ************************//            
      if( PID.iIncpid>= PID.Max_iIncpid)
      {     
         PID.iIncpid = PID.Max_iIncpid ; 
      }    //ÏÞÖÆÔöÁ¿  
      else if( PID.iIncpid<= PID.Min_iIncpid) 
      {    
          PID.iIncpid = PID.Min_iIncpid ; 
      } 
  
      //*******************************//       
        PID.output = PID.last_output + PID.iIncpid ;  
         
      //********************************// 
       if( PID.output >= PID.Max_otput ) 
       {   
          PID.output = PID.Max_otput ;
       }        
       else if( PID.output <= PID.Min_otput )
       {  
          PID.output = PID.Min_otput ; 
       }  //ÏÞÖÆÊä³ö
       zuo_shu_chu=PID.output ;
       PID.output=motor_judge_left( PID.output ) ; //ÊÇ·ñ·´ÍÏÅÐ¶Ï
        
	   PID.last_output = PID.output ;     
       return PID.output ;
       
           
		          
  }    
      
//=======================  ÓÒÂÖ  ==========================//          
int PID_change_R(int Current_Speed ,int SetPoint )          
 {
         //µ±Ç°Îó²î 
         PID.iError_other = SetPoint - Current_Speed;  //ÔöÁ¿¼ÆËã 
        
           //*********ÏÞÖÆÌõ¼þ**************/     
          if( PID.iError_other >= PID.Max_iError )
          {     
            PID.iError_other = PID.Max_iError ;
          }       
          else if( PID.iError_other<= PID.Min_iError)
          {
             PID.iError_other = PID.Min_iError ;  
          } 
          
            //****************************//
        PID.iIncpid_other = ( PID.Derivative * ( PID.PrevError_other + PID.iError_other - 2*PID.LastError_other )
                          + PID.Proportion * ( PID.iError_other - PID.LastError_other ) 
                          + PID.Integral * PID.iError_other )/1000 ;
       
        //´æ´¢Îó²î£¬ÓÃÓÚÏÂ´Î¼ÆËã 
        PID.PrevError_other =  PID.LastError_other ; 
        PID.LastError_other =  PID.iError_other ; 
       
          //*************ÏÞÖÆÅÐ¶Ï************//           
      if( PID.iIncpid_other >= PID.Max_iIncpid )
      {  
          PID.iIncpid_other = PID.Max_iIncpid ;
       }      //ÏÞÖÆÔöÁ¿
      else if( PID.iIncpid_other<= PID.Min_iIncpid ) 
      {    
          PID.iIncpid_other = PID.Min_iIncpid ; 
      } 
  
     //********************************************//       
       PID.output_other = PID.last_output_other + PID.iIncpid_other ;  
           
      //************************************//
        if(PID.output_other>= PID.Max_otput) 
       {  
         PID.output_other = PID.Max_otput ;
        }          
      else if( PID.output_other <=  PID.Min_otput )
       {
           PID.output_other = PID.Min_otput ;  
       }    //ÏÞÖÆÊä³ö
          you_shu_chu=PID.output_other;
	    PID.output_other = motor_judge_right( PID.output_other ) ;
	    PID.last_output_other = PID.output_other ;   
        return PID.output_other ;
}
//================  µ÷ËÙÅÐ¶Ï£¬ÊÇ·ñ·´ÍÏ£¿=====================//
int motor_judge_right(int right)
{
   	if(right>0)
   	{  	    
              SIU.PCR[PCR40_PC8].R = 0x0200 ;  //GPIO[40] is output
              SIU.PCR[PCR44_PC12].R = 0x0200 ; //GPIO[44] is output 

              SIU.GPDO[PCR40_PC8].R = 0 ;       //0-1Õý×ª
              SIU.GPDO[PCR44_PC12].R =1 ;
              return right;
   	}	
 //========================== //  	
   else	if(right<0)
   	{	 
              SIU.PCR[PCR40_PC8].R = 0x0200 ;  //GPIO[40] is output
              SIU.PCR[PCR44_PC12].R = 0x0200 ; //GPIO[44] is output 
        
              SIU.GPDO[PCR40_PC8].R = 1 ;       //1-0·´×ª
              SIU.GPDO[PCR44_PC12].R = 0 ;
              
             if( smartcar_speed_R > 800)
                return  (-right);
              else if( smartcar_speed_R<500)
                return  (0);  
        
   	}	
//===========================//	
    else	 
   	{ 	
              SIU.PCR[PCR40_PC8].R = 0x0200;  //GPIO[40] is output
              SIU.PCR[PCR44_PC12].R = 0x0200; //GPIO[44] is output 
         
              SIU.GPDO[PCR40_PC8].R = 0 ;       //0-1Õý×ª
              SIU.GPDO[PCR44_PC12].R =0 ;   
              return  0;
   	}	
 }
//======================================================//
int motor_judge_left(int left)
{
   	if( left>0 )
   	{
              SIU.PCR[PCR41_PC9].R = 0x0200;  //GPIO[41] is output
              SIU.PCR[PCR47_PC15].R = 0x0200; //GPIO[47] is output 

              SIU.GPDO[PCR41_PC9].R = 0;
              SIU.GPDO[PCR47_PC15].R = 1; 
              return left; 
   	}	
//-===============================//
   else	if( left<0 )
   	{ 
              SIU.PCR[PCR41_PC9].R = 0x0200;  //GPIO[41] is output
              SIU.PCR[PCR47_PC15].R = 0x0200; //GPIO[47] is output  
 
              SIU.GPDO[PCR41_PC9].R = 1;
              SIU.GPDO[PCR47_PC15].R = 0;
             if( smartcar_speed_L > 800)
                return  (-left);
             else if( smartcar_speed_L<500)
                return  (0);
        
   	}	
//================================//
    else
   	{ 
              SIU.PCR[PCR41_PC9].R = 0x0200;  //GPIO[41] is output
              SIU.PCR[PCR47_PC15].R = 0x0200; //GPIO[47] is output  

              SIU.GPDO[PCR41_PC9].R = 0;
              SIU.GPDO[PCR47_PC15].R = 0;
          return  0;
   	}
}
             

             
uint16_t AD_Measure10_1(void)
{
	   uint16_t data1;                         //       SIU.PCR[PCR49_PD1].R = 0x2000;  // Use PD[1] for ANP5 pin CCD
	   ADC_0.NCMR0.R = 0x00000020;             // Select ANP10 inputs for conversion
	   ADC_0.MCR.B.NSTART=1;         	
	   while (ADC_0.CDR[5].B.VALID == 0) {};  /* Wait for last scan to complete */ 
	     data1=ADC_0.CDR[5].B.CDATA;             /* Read ANS0 conversion result data */
	   ADC_0.MCR.B.NSTART=0;         	
	   return data1;
}
/********************************************ÆØ¹â**************************************************/
void StartInte1(void) 
{
	    uint8_t i;
	    TSL_SI1 = 1;         /* SI  = 1 */
	    SamplingDelay();
	    TSL_CLK1 = 1;        /* CLK = 1 */
	    SamplingDelay();
	    TSL_SI1 = 0;         /* SI  = 0 */
	    SamplingDelay();
	    TSL_CLK1 = 0;        /* CLK = 0 */

	    for(i=0; i<127; i++) 
	    {
		        SamplingDelay();
		        TSL_CLK1 = 1;    /* CLK = 1 */
		        SamplingDelay();
		        TSL_CLK1 = 0;    /* CLK = 0 */
	    }
	    
	    SamplingDelay();
	    TSL_CLK1 = 1;        /* CLK = 1 */
	    SamplingDelay();
	    TSL_CLK1 = 0;        /* CLK = 0 */
}
/*--------------------------------------------------------------------------------------------------------*/
void ImageCapture1(uint8_t *Data1)
{      
	    uint8_t i,ii;
	    uint16_t image_value1;
	       
	    TSL_SI1 = 1;         
	    SamplingDelay();
	    TSL_CLK1 = 1;       
	    SamplingDelay();
	    TSL_SI1 = 0;        
	    SamplingDelay();

	    for(i = 0; i < 20; i++)
	        Delay1us();
	    image_value1 = AD_Measure10_1();
	    *Data1++ = (uint8_t)(image_value1>>2) ;
	    TSL_CLK1 = 0;        

	    for(ii=0; ii<127; ii++) 
	    {
		        SamplingDelay();
		        TSL_CLK1 = 1;   
		        
		        image_value1= AD_Measure10_1();
		        *Data1++ = (uint8_t)(image_value1>>2) ;
		        TSL_CLK1 = 0;    
	    }
	       
	    SamplingDelay();
	    TSL_CLK1 = 1;        
	    SamplingDelay();
	    TSL_CLK1 = 0;        
}
  //********************************CCD_IO¿Ú³õÊ¼»¯*****************************************//
void CCD_IO_Init(void) 
{
  	  TSL_CLK_DDR1 =0x0203;
	  TSL_SI_DDR1  =0x0203;
  	  TSL_CLK1 = 0;
  	  TSL_SI1  = 0;
  
}
//************************************ÉÏÎ»»ú·¢Êý*******************************************//
void SendHex(uint8_t hex) 
{
	  unsigned char temp;
	  temp = hex>>4;
	  if(temp < 10) 
	  {
	    SCI0_SendChar(temp + '0');
	  } 
	  else 
	  {
	    SCI0_SendChar(temp - 10 + 'A');
	  }
	  temp = hex & 0x0F;
	  if(temp < 10) 
	  {
	    SCI0_SendChar(temp + '0');
	  } 
	  else 
	  {
	    SCI0_SendChar(temp - 10 + 'A');
	  }
}

/*--------------------------------------------------------------------------------*/
void SendImageData(uint8_t *ImageData) 
{
    unsigned char i;
    unsigned char crc = 0;

    SCI0_SendChar('*');
    SCI0_SendChar('L');
    SCI0_SendChar('D');

    SendHex(0);
    SendHex(0);
    SendHex(0);
    SendHex(0);

    for(i=0; i<128; i++) 
      SendHex(*ImageData++);

    SendHex(crc);
    SCI0_SendChar('#');
}

void CCD_tiaobian_erzhihua( void )
{	
	uint8_t i=0 , j=0 ;
	CCD_yuzhi=2  ;  
	//===================   ÕÒµ½×î´óÖµ     ==================================	
	CCD_max[0]=( CCD_sample[23]+CCD_sample[24]+CCD_sample[101]+CCD_sample[102] ) / 4 ;
	CCD_max[1]=( CCD_sample[57]+CCD_sample[58]+CCD_sample[59]+CCD_sample[60] ) / 4 ;
	CCD_max[2]=( CCD_sample[64]+CCD_sample[65]+CCD_sample[66]+CCD_sample[67] ) / 4 ;
	if( CCD_max[1]<CCD_max[2] )  CCD_max[1]=CCD_max[2] ;
	// ==============      min   =========================//
	CCD_min=( CCD_sample[30]+CCD_sample[31]+CCD_sample[32]+CCD_sample[33]+
	                    CCD_sample[92]+CCD_sample[93]+CCD_sample[94]+CCD_sample[95] ) / 8 ;
	 CCD_max[3]=CCD_max[0] ;
	 CCD_max[4]=( CCD_max[0]-CCD_min )/4+CCD_min ;  // ãÐÖµ 
	 
	 if( CCD_max[1]<CCD_max[0] )   CCD_max[0]=CCD_max[1] ; 
	 if( CCD_max[0]<CCD_min )    CCD_max[0]=CCD_min ;

	 CCD_yuzhi=( CCD_max[0]-CCD_min )/3 ;  
	 CCD_value= CCD_max[0] - CCD_yuzhi ;
	//=====================  Ìø±äµÄ²îÖµ  ===========================
        for(i=40;i<89;i++)
        {
		          if( CCD_sample[i]<=CCD_min )  CCD_sample_now[i]=0 ;
	        	  if( CCD_sample[i]>=CCD_max[0] )  CCD_sample_now[i]=70 ;  
	        
	        	  if( CCD_sample[i]<CCD_value )  CCD_sample_now[i]=0 ;
	        	  else CCD_sample_now[i]=70 ;
	        	  
	        	  if( CCD_sample[i]<=CCD_max[4] )  CCD_sample_now[i]=0 ;
        }
        
        /*
        CCD_yuzhi=( CCD_max-CCD_min )/3 ;            
	
	//=====================  Ìø±äµÄÖµ  ===========================
       CCD_value= CCD_max - CCD_yuzhi ;
       
     //     CCD_value= CCD_min + CCD_yuzhi ;
       
             for(i=40;i<89;i++)
	      {
		          if( CCD_value<=CCD_sample[i] )     CCD_sample_now[i]=70; 
		          else   CCD_sample_now[i]=0;
		          if( CCD_sample_now[i]<=CCD_min )  CCD_sample_now[i]=0 ;
	        	  else if( CCD_sample_now[i]>=CCD_max )  CCD_sample_now[i]=70 ;
	      }
           */       
             for(i=38;i<89;i++)  //  40 ----- 64 ------ 88
	      {
	      		if(CCD_sample_now[i]==0&&CCD_sample_now[i+1]==70&&CCD_sample_now[i+2]==0)
	      	   		CCD_sample_now[i+1]=0 ;
	        	else if(CCD_sample_now[i]==70&&CCD_sample_now[i+1]==0&&CCD_sample_now[i+2]==70)
	      	   		CCD_sample_now[i+1]=70 ;
	      }
             CCD_sample_now[63]=240 ;
             
         //=============================±ÜÕÏ´¦Àí==========================//
	     CCD_lefttimes[0]=0 ;
	     CCD_righttimes[0]=0 ;
	     for(i=38;i<63;i++) // 40--65
	     {
	             if(CCD_sample_now[i]==0)
	                   CCD_lefttimes[0]++ ;
	     }
	     for(i=65;i<89;i++) // 65--88
	     {
	             if(CCD_sample_now[i]==0)
	                   CCD_righttimes[0]++ ;
	     }
	     
	     if(CCD_bizhang_flag==0)
            {
		      if( (CCD_lefttimes[0]-CCD_righttimes[0])>4 && CCD_lefttimes[0]>5 && CCD_righttimes[0]<3 )
		      {
		      		if( CCD_lefttimes[1]>4 && CCD_lefttimes[2]>4 ) // È·¶¨µãÊÇÍ»±äµÄ
		      			CCD_bizhang_flag=1 ;//×ó±ßÕÏ°­
		      }
		      else if( (CCD_righttimes[0]-CCD_lefttimes[0])>4 && CCD_righttimes[0]>5 && CCD_lefttimes[0]<3 )
		      {
		      		if( CCD_righttimes[1]>4 && CCD_righttimes[2]>4 ) // È·¶¨µãÊÇÍ»±äµÄ
		      			CCD_bizhang_flag=2 ;//ÓÒ±ßÕÏ°­
		      }  
            }
}



//===========================±ÜÕÏCCD³ÌÐò==================================//
void CCD_bizhang( void )
{
                 if(CCD_times==1)
		  {
				    CCD_times=0 ;
				    CCD_stoptimes=motor_time ;
				    ImageCapture1(CCD_sample) ;//²ÉÑù 
				    StartInte1( ) ;//ÆØ¹â
				    CCD_tiaobian_erzhihua( ) ; //¶þÖµ»¯ ¡¢ÅÐ¶ÏÊÇ·ñÓÐÕÏ°­
				   
				    if( CCD_bizhang_flag==1 || CCD_bizhang_flag==2 )
				    {
						SIU.GPDO[PCR38_PC6].R =0 ;//¸ø0²ÅÁÁ
						CCD_juli_flag++ ;
						if( CCD_juli_flag==1 )
						{
						        CCD_juli_zuo[1]=CCD_juli_zuo[0] ;
			                              CCD_juli_you[1]=CCD_juli_you[0] ;
						}
				    }
				    else    SIU.GPDO[PCR38_PC6].R=1 ;
				    
				    if( CCD_bizhang_flag==1 ){    CCD_sample_now[1]=240 ; CCD_sample_now[126]=0 ; }
				    else if( CCD_bizhang_flag==2 ){    CCD_sample_now[1]=0 ; CCD_sample_now[126]=240 ; }
				    else    {    CCD_sample_now[1]=0 ; CCD_sample_now[126]=0 ; }

	/*  
		                   sendcnt++;
	  			            if(sendcnt>=25)    
				            {   
				                   if(sendcnt<=100) 
								SendImageData(CCD_sample) ;
			                          else if(sendcnt<=200) 
				                  	       SendImageData(CCD_sample_now) ;
			                       
				                    if(sendcnt>210)    sendcnt=0 ;
				                    
				             //      SIU.GPDO[PCR38_PC6].R =!SIU.GPDO[PCR38_PC6].R ;//¸ø0²ÅÁÁ
				           }
	 */	/*	*/
		  }
}




//13===3====2===1
//         11===10
//          8====9
//========================================================================//
uint16_t  diangan_ADC(uint8_t tongdao) // ²É¼¯µç¸ÐµÄÖµ
{
    uint16_t i=0 , j=0 , temp=0 ;
    uint16_t a[DIAN]={0 };
    uint16_t ADC_tongdao ;
    uint32_t average=0 ;
    
    if(tongdao == 13)     ///Ò»ÅÅ  1////
        ADC_tongdao = 0x00002000;
    else if(tongdao == 3)       ///Ò»ÅÅ  2////
        ADC_tongdao = 0x00000008;
    
    else if(tongdao == 2)       ///Ò»ÅÅ  3////
        ADC_tongdao = 0x00000004;
    else if(tongdao == 1)       ///Ò»ÅÅ  4////
        ADC_tongdao = 0x00000002; 
    
    else if(tongdao == 11)     ///¶þÅÅ  1////
        ADC_tongdao = 0x00000800;
    else if(tongdao == 10)       ///¶þÅÅ  2////
        ADC_tongdao = 0x00000400;
    
    else if(tongdao == 8)       ///ÈýÅÅ  1////
        ADC_tongdao = 0x00000100;
    else if(tongdao == 9)       ///ÈýÅÅ  2////
        ADC_tongdao = 0x00000200; 
    
    ADC_0.NCMR0.R = ADC_tongdao;     // Select Channel 14 inputs for conversion
    ADC_0.MCR.B.NSTART=1;             /* Trigger normal conversions for ADC0 */ 

   	    while (ADC_0.CDR[tongdao].B.VALID == 0) ; //1
	    a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	    average+=a[i] ;     
	            while (ADC_0.CDR[tongdao].B.VALID == 0) ;//2
	            a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	            average+=a[i] ;   
	    while (ADC_0.CDR[tongdao].B.VALID == 0) ;  //3
	    a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	    average+=a[i] ;    
	            while (ADC_0.CDR[tongdao].B.VALID == 0) ;//4
	            a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	            average+=a[i] ;   
	    while (ADC_0.CDR[tongdao].B.VALID == 0) ;//5
	    a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	    average+=a[i] ;   
	            while (ADC_0.CDR[tongdao].B.VALID == 0) ;//6
	            a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	            average+=a[i] ;    
	   while (ADC_0.CDR[tongdao].B.VALID == 0) ;  //7
	    a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	    average+=a[i] ;     
	            while (ADC_0.CDR[tongdao].B.VALID == 0) ; //8
	            a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	            average+=a[i] ;   
	    while (ADC_0.CDR[tongdao].B.VALID == 0) ; //9
	    a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	    average+=a[i] ;    
	            while (ADC_0.CDR[tongdao].B.VALID == 0) ; //10
	            a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	            average+=a[i] ;   
	            
	            
	    while (ADC_0.CDR[tongdao].B.VALID == 0) ;  //11
	    a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	    average+=a[i] ;   
	            while (ADC_0.CDR[tongdao].B.VALID == 0) ;//12
	            a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	            average+=a[i] ;   
	   while (ADC_0.CDR[tongdao].B.VALID == 0) ; // 13
	    a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	    average+=a[i] ;     
	            while (ADC_0.CDR[tongdao].B.VALID == 0) ; //14
	            a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	            average+=a[i] ;   
	    while (ADC_0.CDR[tongdao].B.VALID == 0) ; //15
	    a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	    average+=a[i] ;    
	            while (ADC_0.CDR[tongdao].B.VALID == 0) ; //16
	            a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	            average+=a[i] ;   
	    while (ADC_0.CDR[tongdao].B.VALID == 0) ; //17
	    a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	    average+=a[i] ;   
	            while (ADC_0.CDR[tongdao].B.VALID == 0) ; //18
	            a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	            average+=a[i] ;                     
	  while (ADC_0.CDR[tongdao].B.VALID == 0) ;//19
	    a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	    average+=a[i] ;     
	            while (ADC_0.CDR[tongdao].B.VALID == 0) ; //20
	            a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	            average+=a[i] ;   
	            
	            
	    while (ADC_0.CDR[tongdao].B.VALID == 0) ; //21
	    a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	    average+=a[i] ;    
	            while (ADC_0.CDR[tongdao].B.VALID == 0) ;//22
	            a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	            average+=a[i] ;   
	    while (ADC_0.CDR[tongdao].B.VALID == 0) ; //23
	    a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	    average+=a[i] ;   
	            while (ADC_0.CDR[tongdao].B.VALID == 0) ;//24
	            a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	            average+=a[i] ;   
	   while (ADC_0.CDR[tongdao].B.VALID == 0) ; //25
	    a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	    average+=a[i] ;     
	            while (ADC_0.CDR[tongdao].B.VALID == 0) ;//26
	            a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	            average+=a[i] ;   
	    while (ADC_0.CDR[tongdao].B.VALID == 0) ;//27
	    a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	    average+=a[i] ;    
	            while (ADC_0.CDR[tongdao].B.VALID == 0) ;//28
	            a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	            average+=a[i] ;   
	    while (ADC_0.CDR[tongdao].B.VALID == 0) ;//29
	    a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	    average+=a[i] ;   
	            while (ADC_0.CDR[tongdao].B.VALID == 0) ;//30
	            a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	            average+=a[i] ;            
	            
	  while (ADC_0.CDR[tongdao].B.VALID == 0) ; //31
	    a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	    average+=a[i] ;     
	            while (ADC_0.CDR[tongdao].B.VALID == 0) ;//32
	            a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	            average+=a[i] ;   
	    while (ADC_0.CDR[tongdao].B.VALID == 0) ;//33
	    a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	    average+=a[i] ;    
	            while (ADC_0.CDR[tongdao].B.VALID == 0) ;//34
	            a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	            average+=a[i] ;   
	    while (ADC_0.CDR[tongdao].B.VALID == 0) ;//35
	    a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	    average+=a[i] ;   
	            while (ADC_0.CDR[tongdao].B.VALID == 0) ;//36
	            a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	            average+=a[i] ;             
	            
	     while (ADC_0.CDR[tongdao].B.VALID == 0) ; //37
	    a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	    average+=a[i] ;     
	            while (ADC_0.CDR[tongdao].B.VALID == 0) ;//38
	            a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	            average+=a[i] ;   
	    while (ADC_0.CDR[tongdao].B.VALID == 0) ;  //39
	    a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	    average+=a[i] ;    
	            while (ADC_0.CDR[tongdao].B.VALID == 0) ;//40
	            a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	            average+=a[i] ;   
	    while (ADC_0.CDR[tongdao].B.VALID == 0) ;//41
	    a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	    average+=a[i] ;   
	            while (ADC_0.CDR[tongdao].B.VALID == 0) ;//42
	            a[i] = ADC_0.CDR[tongdao].B.CDATA;	 
	            average+=a[i] ;    
	     
	  
	            	
    ADC_0.MCR.B.NSTART = 0;
       
     average = average / ( DIAN ) ;
 
   	//SCI0_SendChar_16( average ) ; 	
        return average ;       
}
//13===3====2===1
//         11===10
//          8====9
//*******************************************************************//
//****************************  ¹éÒ»»¯  ´¦Àí  ************************//
uint16_t diangan_guiyihua( uint8_t guiyi_diangan_hao)
{
        uint16_t  guiyi[guiyi_DIAN] ;
        
        SIU.GPDO[PCR38_PC6].R = 1 ;    // Ãð   
        delay500ms() ; delay500ms() ;
        SIU.GPDO[PCR38_PC6].R = 0 ;   // ÁÁ     
       //=======================================//
        if( guiyi_diangan_hao == 1)    // Ë®Æ½Ò»ºÅµç¸Ð=======13¿Ú
        {
        	for( i_quan=0 ; i_quan <guiyi_DIAN ; i_quan++ )
              { 
                   guiyi[i_quan] = diangan_ADC(13) ;
       	            delay5ms() ;   	    
              } 
       }
       
       //==========================================//
       else if( guiyi_diangan_hao == 2)  // Ë®Æ½¶þºÅµç¸Ð=====3¿Ú
        {
      	      for( i_quan=0 ; i_quan <guiyi_DIAN ; i_quan++ )
             {
      	            guiyi[i_quan] = diangan_ADC(3) ;
      	      //      LCD5110_number_dis( guiyi[i_quan] );
      	            delay5ms() ; 	    
             }
      }
      
      //==========================================//
      else if( guiyi_diangan_hao == 3)  // Ë®Æ½ÈýºÅµç¸Ð======2¿Ú
      {
             for( i_quan=0 ; i_quan <guiyi_DIAN ; i_quan++ )
             {
    	           guiyi[i_quan] = diangan_ADC(2) ;
    	       //   LCD5110_number_dis( guiyi[i_quan] );
      	           delay5ms() ; 	    
             }
      }
        else if( guiyi_diangan_hao == 4)  // Ë®Æ½ÈýºÅµç¸Ð======2¿Ú
      {
             for( i_quan=0 ; i_quan <guiyi_DIAN ; i_quan++ )
             {
    	           guiyi[i_quan] = diangan_ADC(1) ;
    	       //   LCD5110_number_dis( guiyi[i_quan] );
      	           delay5ms() ; 	    
             }
      }
      //==========================================//
     else if( guiyi_diangan_hao == 5) //  =======11¿Ú
      {
      	      for( i_quan=0 ; i_quan <guiyi_DIAN ; i_quan++ )
             {
      	            guiyi[i_quan] = diangan_ADC(11) ;
       	            delay1ms() ; 	    
             }
      }
      
       //==========================================//
     else if( guiyi_diangan_hao == 6) //    =========1¿Ú
      {
      	      for( i_quan=0 ; i_quan <guiyi_DIAN ; i_quan++ )
             {
      	            guiyi[i_quan] = diangan_ADC(10) ;
       	            delay1ms() ;	    
             }
      }
      
    
  
      
      SIU.GPDO[PCR38_PC6].R = 1 ;    // Ãð   
      
      for( i_quan=1 ; i_quan <guiyi_DIAN ; i_quan++ )
      {       if( guiyi[0] < guiyi[i_quan])     guiyi[0] = guiyi[i_quan] ;  }
    
      return  guiyi[0] ;
}
//13===3====2===1
//         11===10
//          8====9
//=========================  ²É¼¯µç¸ÐÐÅºÅ  ====================//
void Cai_yang(void)  // ²É¼¯Îå¸öµç¸ÐµÄ×î´óÖµ£¬Ë®Æ½µç¸ÐÖ®ºÍ£¬´¹Ö±µç¸ÐÖµºÍ
{ 
       int i;
      uint16_t  array_temp[11] = { 0 } ;
      array_one[0]=200*diangan_ADC(13)/guiyi_MAX[0];//Ð±ÖÃµç¸Ð1ºÅ
	  array_two[0]=200*diangan_ADC(3)/guiyi_MAX[1]; //Ë®Æ½µç¸Ð1ºÅ
	  array_three[0]=200*diangan_ADC(2)/guiyi_MAX[2];//Ë®Æ½µç¸Ð2ºÅ
	  array_four[0]=200*diangan_ADC( 1)/guiyi_MAX[3];//Ð±ÖÃµç¸Ð2ºÅ
	  array_five[0]=200*diangan_ADC(11)/guiyi_MAX[4];//´¹Ö±µç¸Ð1ºÅ  
      array_six[0]=200*diangan_ADC(10)/guiyi_MAX[5];//´¹Ö±µç¸Ð1ºÅ 
	                        //SCI0_SendChar_16(array_one[0]);
 		                    //SCI0_SendCh[]ar_16(array_two[0]);
 		                    //SCI0_SendChar_16(array_three[0]);
 		                    //SCI0_SendChar_16(array_four[0]);
		                    //SCI0_SendChar_16(array_five[0]); 
        
	  //  ÏÞ·ù ¡ª¡ª ÂË²¨
	  //================== 1 ºÅ µç¸Ð ============== //
	  if( (array_one[0]-array_one[1]) > 5 )    //======= ²¨¶¯ÏÞ·ù´óÐ¡¡ª¡ª5£¬11 ¡ª¡ª +3 £¬+7=====//
	  {
	        array_lvbo_flag[0][2] = 0 ;
	        array_lvbo_flag[0][3] = 0 ;
	        if( array_lvbo_flag[0][0] < 2 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 5 ÒÔÄÚ
	        {
	        	array_one[0]=array_one[1]+3 ; // 1
	        	array_lvbo_flag[0][0]++;
	        }
	  	else  if( array_lvbo_flag[0][1] < 3 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		array_one[0]=array_one[1]+8 ; // 1
	  		array_lvbo_flag[0][1]++;
	  	}
	  	else if( (array_one[0]-array_one[5]) < 60 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 6 ´ÎÒÔÄÚ»¹Î´»Ø¹é
	  	       array_one[0]=array_one[5] + 40 ;
	  	else  
	  	       array_one[0]=array_one[1]+15 ; // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÔÚ 7 ´ÎÒÔÄÚ»¹Î´»Ø¹é
	  } 
 	  else if( (array_one[0]-array_one[1]) < -5 )   //======= ²¨¶¯ÏÞ·ù´óÐ¡¡ª¡ª -5£¬-11 ¡ª¡ª -3 £¬-7=====//
 	  {
	        array_lvbo_flag[0][0] = 0 ;
	        array_lvbo_flag[0][1] = 0 ;
	        if( array_lvbo_flag[0][2] < 2 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 5 ÒÔÄÚ
	        {
	        	array_one[0]=array_one[1]-3 ; // 1
	        	if( array_one[0]<0 )   array_one[0]=0;
	        	array_lvbo_flag[0][2]++;
	        }
	  	else  if( array_lvbo_flag[0][3] < 3 ) //// array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		array_one[0]=array_one[1]-8 ; // 1
	  		if( array_one[0]<0 )   array_one[0]=0;
	  		array_lvbo_flag[0][3]++;
	  	}
	  	else if( (array_one[0]-array_one[5]) > -60 ) // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 6 ´ÎÒÔÄÚ»¹Î´»Ø¹é
	  	     {
	  	         	array_one[0]=array_one[5] - 40 ;
	  	     		if( array_one[0]<0 )   array_one[0]=0;
	  	     }  
	  	else  
	  	     {
	  	         	array_one[0]=array_one[1]-15 ; 
	  	     		if( array_one[0]<0 )   array_one[0]=0;
	  	     }  
 	  }
 	  else
 	  {
 	     	array_lvbo_flag[0][0] = 0 ;
	        array_lvbo_flag[0][1] = 0 ;
	        array_lvbo_flag[0][2] = 0 ;
	        array_lvbo_flag[0][3] = 0 ;
 	  }
 	  // ========    ÏòÏÂÏÞ·ù    =========//
	   if( (array_one[0]-array_one[1]) > -3 && (array_one[0]-array_one[1]) < 3  )    //======= ²¨¶¯ÏÞ·ù´óÐ¡¡ª¡ª5£¬11 ¡ª¡ª +3 £¬+7=====//
	  {
	        array_lvbo_flag[0][5] ++ ;
	        if( array_lvbo_flag[0][5] == 10 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 5 ÒÔÄÚ
	    	 {
	    	  	array_temp[0]=( array_one[0]+array_one[1]+array_one[2]+array_one[3]+array_one[4]+array_one[5]+array_one[6]+array_one[7]+array_one[8]+array_one[9]+5 ) /10 ; // 1
	   		if( (array_temp[0]-array_one[0]) > -3 && (array_temp[0]-array_one[0]) < 3  )    
	   		  	array_one[0] = array_temp[0] ;
	   		else 
	   		        array_lvbo_flag[0][5] = 0 ;
	  	}
	  	else if( array_lvbo_flag[0][5] > 10 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		array_one[0] = array_one[1] ;
	  		array_lvbo_flag[0][5] -- ;
	  	}	
	  }
	  else
	  	array_lvbo_flag[0][5] = 0 ;
   
   
   
   
   

 	    
 	   //================== 2 ºÅ µç¸Ð ============== //   
 	  if( (array_two[0]-array_two[1]) > 5 )    //======= ²¨¶¯ÏÞ·ù´óÐ¡¡ª¡ª5£¬11 ¡ª¡ª +3 £¬+7=====//
	  {
	        array_lvbo_flag[1][2] = 0 ;
	        array_lvbo_flag[1][3] = 0 ;
	        if( array_lvbo_flag[1][0] < 2 )  // array_lvbo_flag[1][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 5 ÒÔÄÚ
	        {
	        	array_two[0]=array_two[1]+3 ; // 1
	        	array_lvbo_flag[1][0]++;
	        }
	  	else  if( array_lvbo_flag[1][1] < 3 )  // array_lvbo_flag[1][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		array_two[0]=array_two[1]+8 ; // 1
	  		array_lvbo_flag[1][1]++;
	  	}
	  	else if( (array_two[0]-array_two[5]) < 60 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 6 ´ÎÒÔÄÚ»¹Î´»Ø¹é
	  	       array_two[0]=array_two[5] + 40 ;
	  	else  
	  	       array_two[0]=array_two[1]+15 ; // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÔÚ 7 ´ÎÒÔÄÚ»¹Î´»Ø¹é
	  } 
 	  else if( (array_two[0]-array_two[1]) < -5 )   //======= ²¨¶¯ÏÞ·ù´óÐ¡¡ª¡ª -5£¬-11 ¡ª¡ª -3 £¬-7=====//
 	  {
	        array_lvbo_flag[1][0] = 0 ;
	        array_lvbo_flag[1][1] = 0 ;
	        if( array_lvbo_flag[1][2] < 2 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 5 ÒÔÄÚ
	        {
	        	array_two[0]=array_two[1]-3 ; // 1
	        		if( array_two[0]<0 )   array_two[0]=0;
	        	array_lvbo_flag[1][2]++;
	        }
	  	else  if( array_lvbo_flag[1][3] < 3 ) //// array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		array_two[0]=array_two[1]-8 ; // 1
	  			if( array_two[0]<0 )   array_two[0]=0;
	  		array_lvbo_flag[1][3]++;
	  	}
	  	else if( (array_two[0]-array_two[5]) > -60 ) // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 6 ´ÎÒÔÄÚ»¹Î´»Ø¹é
	  	     {
	  	        	array_two[0]=array_two[5] - 40 ;
	  	     		if( array_two[0]<0 )   array_two[0]=0;
	  	     }  
	  	else  
	  	    {
	  	        	array_two[0]=array_two[1]-15 ; 
	  	    		if( array_two[0]<0 )   array_two[0]=0;
	  	    }   
 	  }
 	  else
 	  {
 	     	array_lvbo_flag[1][0] = 0 ;
	        array_lvbo_flag[1][1] = 0 ;
	        array_lvbo_flag[1][2] = 0 ;
	        array_lvbo_flag[1][3] = 0 ;
 	  }
 	   	   // ========    ÏòÏÂÏÞ·ù    =========//
	   if( (array_two[0]-array_two[1]) > -3 && (array_two[0]-array_two[1]) < 3  )    //======= ²¨¶¯ÏÞ·ù´óÐ¡¡ª¡ª5£¬11 ¡ª¡ª +3 £¬+7=====//
	  {
	        array_lvbo_flag[1][5] ++ ;
	        if( array_lvbo_flag[1][5] == 10 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 5 ÒÔÄÚ
	    	 {
	    	  	array_temp[1]=( array_two[0]+array_two[1]+array_two[2]+array_two[3]+array_two[4]+array_two[5]+array_two[6]+array_two[7]+array_two[8]+array_two[9]+5 ) /10 ; // 1
                	if( (array_temp[1]-array_two[0]) > -3 && (array_temp[1]-array_two[0]) < 3  )    
	   		  	array_two[0] = array_temp[1] ;
	   		else 
	   		        array_lvbo_flag[1][5] = 0 ;
		 }
	  	else if( array_lvbo_flag[1][5] > 10 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		array_two[0] = array_two[1] ;
	  		array_lvbo_flag[1][5] -- ;
	  	}	
	  }
	  else
	  	array_lvbo_flag[1][5] = 0 ;
	  
	  
	  
	  

   
 	  //================== 3 ºÅ µç¸Ð ============== //   
 	  if( (array_three[0]-array_three[1]) > 5 )    //======= ²¨¶¯ÏÞ·ù´óÐ¡¡ª¡ª5£¬11 ¡ª¡ª +3 £¬+7=====//
	  {
	        array_lvbo_flag[2][2] = 0 ;
	        array_lvbo_flag[2][3] = 0 ;
	        if( array_lvbo_flag[2][0] < 2 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 5 ÒÔÄÚ
	        {
	        	array_three[0]=array_three[1]+3 ; // 1
	        	array_lvbo_flag[2][0]++;
	        }
	  	else  if( array_lvbo_flag[2][1] < 3 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		array_three[0]=array_three[1]+8 ; // 1
	  		array_lvbo_flag[2][1]++;
	  	}
	  	else if( (array_three[0]-array_three[5]) < 60 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 6 ´ÎÒÔÄÚ»¹Î´»Ø¹é
	  	       array_three[0]=array_three[5] + 40 ;
	  	else  
	  	     {
	  	     	array_three[0]=array_three[1]+15 ; // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÔÚ 7 ´ÎÒÔÄÚ»¹Î´»Ø¹é
	  	     }  
	  } 
 	  else if( (array_three[0]-array_three[1]) < -5 )   //======= ²¨¶¯ÏÞ·ù´óÐ¡¡ª¡ª -5£¬-11 ¡ª¡ª -3 £¬-7=====//
 	  {
	        array_lvbo_flag[2][0] = 0 ;
	        array_lvbo_flag[2][1] = 0 ;
	        if( array_lvbo_flag[2][2] < 2 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 5 ÒÔÄÚ
	        {
	        	array_three[0]=array_three[1]-3 ; // 1
	        		if( array_three[0]<0 )   array_three[0]=0;
	        	array_lvbo_flag[2][2]++;
	        }
	  	else  if( array_lvbo_flag[2][3] < 3 ) //// array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		array_three[0]=array_three[1]-8 ; // 1
	  			if( array_three[0]<0 )   array_three[0]=0;
	  		array_lvbo_flag[2][3]++;
	  	}
	  	else if( (array_three[0]-array_three[5]) > -60 ) // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 6 ´ÎÒÔÄÚ»¹Î´»Ø¹é
	  	       {
	  	       	    array_three[0]=array_three[5] - 40 ;
	  	       	    	if( array_three[0]<0 )   array_three[0]=0;
	  	       }
	  	else  
	  	    {
	  	        	array_three[0]=array_three[1]-15 ; 
	  	    		if( array_three[0]<0 )   array_three[0]=0;
	  	    }   
 	  }
 	  else
 	  {
 	     	array_lvbo_flag[2][0] = 0 ;
	        array_lvbo_flag[2][1] = 0 ;
	        array_lvbo_flag[2][2] = 0 ;
	        array_lvbo_flag[2][3] = 0 ;
 	  }
 	  	  // ========    ÏòÏÂÏÞ·ù    =========//
	   if( (array_three[0]-array_three[1]) > -3 && (array_three[0]-array_three[1]) < 3  )    //======= ²¨¶¯ÏÞ·ù´óÐ¡¡ª¡ª5£¬11 ¡ª¡ª +3 £¬+7=====//
	  {
	        array_lvbo_flag[2][5] ++ ;
	        if( array_lvbo_flag[2][5] == 10 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 5 ÒÔÄÚ
	    	 {
	    	  	array_temp[2]=( array_three[0]+array_three[1]+array_three[2]+array_three[3]+array_three[4]+array_three[5]+array_three[6]+array_three[7]+array_three[8]+array_three[9]+5 ) /10 ; // 1
                	if( (array_temp[2]-array_three[0]) > -3 && (array_temp[2]-array_three[0]) < 3  )    
	   		  	array_three[0] = array_temp[2] ;
	   		else 
	   		        array_lvbo_flag[2][5] = 0 ;
		 }
	  	else if( array_lvbo_flag[2][5] > 10 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		array_three[0] = array_three[1] ;
	  		array_lvbo_flag[2][5] -- ;
	  	}	
	  }
	  else
	  	array_lvbo_flag[2][5] = 0 ;
	  
	  
	  

 	  //================== 4 ºÅ µç¸Ð ============== //   
 	  if( (array_four[0]-array_four[1]) > 5 )    //======= ²¨¶¯ÏÞ·ù´óÐ¡¡ª¡ª5£¬11 ¡ª¡ª +3 £¬+7=====//
	  {
	        array_lvbo_flag[3][2] = 0 ;
	        array_lvbo_flag[3][3] = 0 ;
	        if( array_lvbo_flag[3][0] < 2 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 5 ÒÔÄÚ
	        {
	        	array_four[0]=array_four[1]+3 ; // 1
	        	array_lvbo_flag[3][0]++;
	        }
	  	else  if( array_lvbo_flag[3][1] < 3 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		array_four[0]=array_four[1]+8 ; // 1
	  		array_lvbo_flag[3][1]++;
	  	}
	  	else if( (array_four[0]-array_four[5]) < 60 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 6 ´ÎÒÔÄÚ»¹Î´»Ø¹é
	  	       array_four[0]=array_four[5] + 40 ;
	  	else  
	  	    {
	  	    	array_four[0]=array_four[1]+15 ; // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÔÚ 7 ´ÎÒÔÄÚ»¹Î´»Ø¹é
	  	    }   	 
	  }
 	  else if( (array_four[0]-array_four[1]) < -5 )   //======= ²¨¶¯ÏÞ·ù´óÐ¡¡ª¡ª -5£¬-11 ¡ª¡ª -3 £¬-7=====//
 	  {
	        array_lvbo_flag[3][0] = 0 ;
	        array_lvbo_flag[3][1] = 0 ;
	        if( array_lvbo_flag[3][2] < 2 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 5 ÒÔÄÚ
	        {
	        	array_four[0]=array_four[1]-3 ; // 1
	        		if( array_four[0]<0 )   array_four[0]=0;
	        	array_lvbo_flag[3][2]++;
	        }
	  	else  if( array_lvbo_flag[3][3] < 3 ) //// array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		array_four[0]=array_four[1]-8 ; // 1
	  			if( array_four[0]<0 )   array_four[0]=0;
	  		array_lvbo_flag[3][3]++;
	  	}
	  	else if( (array_four[0]-array_four[5]) > -60 ) // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 6 ´ÎÒÔÄÚ»¹Î´»Ø¹é
	  	     {
	  	     	       array_four[0]=array_four[5] - 40 ;
	  	     	   	if( array_four[0]<0 )   array_four[0]=0;
	  	     }  
	  	else  
	  	     {
	  	        	array_four[0]=array_four[1]-15 ; 
	  	        	if( array_four[0]<0 )   array_four[0]=0; 
	  	     }  
 	  }
 	  else
 	  {
 	     	array_lvbo_flag[3][0] = 0 ;
	        array_lvbo_flag[3][1] = 0 ;
	        array_lvbo_flag[3][2] = 0 ;
	        array_lvbo_flag[3][3] = 0 ;
 	  }
 	  	  // ========    ÏòÏÂÏÞ·ù    =========//
	   if( (array_four[0]-array_four[1]) > -3 && (array_four[0]-array_four[1]) < 3  )    //======= ²¨¶¯ÏÞ·ù´óÐ¡¡ª¡ª5£¬11 ¡ª¡ª +3 £¬+7=====//
	  {
	        array_lvbo_flag[3][5] ++ ;
	        if( array_lvbo_flag[3][5] == 10 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 5 ÒÔÄÚ
	    	 {
	    	  	array_temp[3]=( array_four[0]+array_four[1]+array_four[2]+array_four[3]+array_four[4]+array_four[5]+array_four[6]+array_four[7]+array_four[8]+array_four[9]+5 ) /10 ; // 1
                	if( (array_temp[3]-array_four[0]) > -3 && (array_temp[3]-array_four[0]) < 3  )    
	   		  	array_four[0] = array_temp[3] ;
	   		else 
	   		        array_lvbo_flag[3][5] = 0 ;
		 }
	  	else if( array_lvbo_flag[3][5] > 10 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		array_four[0] = array_four[1] ;
	  		array_lvbo_flag[3][5] -- ;
	  	}	
	  }
	  else
	  	array_lvbo_flag[3][5] = 0 ;
	  
	  
	  
	  
	  

 	  //================== 5 ºÅ µç¸Ð ============== //   
 	  if( (array_five[0]-array_five[1]) > 3 )    //======= ²¨¶¯ÏÞ·ù´óÐ¡¡ª¡ª5£¬11 ¡ª¡ª +3 £¬+7=====//
	  {
	        array_lvbo_flag[4][3] = 0 ;
	        array_lvbo_flag[4][4] = 0 ;
	        array_lvbo_flag[4][5] = 0 ;
	        if( array_lvbo_flag[4][0] < 1 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 5 ÒÔÄÚ
	        {
	        	array_five[0]=array_five[1]+2 ; // 1
	        	array_lvbo_flag[4][0]++;
	        }
	  	else  if( array_lvbo_flag[4][1] < 2 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		array_five[0]=array_five[1]+4 ; // 1
	  		array_lvbo_flag[4][1]++;
	  	}
	       else  if( array_lvbo_flag[4][2] < 3 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		array_five[0]=array_five[1]+8 ; // 1
	  		array_lvbo_flag[4][2]++;
	  	}
	  	else if( (array_five[0]-array_five[6]) < 60 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 6 ´ÎÒÔÄÚ»¹Î´»Ø¹é
	  	       array_five[0]=array_five[6] + 40 ;
	  	else  
	  	   {
	  	   	 array_five[0]=array_five[1]+15 ; // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÔÚ 7 ´ÎÒÔÄÚ»¹Î´»Ø¹é
	  	   }   
	  } 
 	  else if( (array_five[0]-array_five[1]) < -3 )   //======= ²¨¶¯ÏÞ·ù´óÐ¡¡ª¡ª -5£¬-11 ¡ª¡ª -3 £¬-7=====//
 	  {
	        array_lvbo_flag[4][0] = 0 ;
	        array_lvbo_flag[4][1] = 0 ;
	        array_lvbo_flag[4][2] = 0 ;
	        if( array_lvbo_flag[4][3] < 1 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 5 ÒÔÄÚ
	        {
	        	array_five[0]=array_five[1]-2 ; // 1
	        		if( array_five[0]<0 )   array_five[0]=0;
	        	array_lvbo_flag[4][3]++;
	        }
	  	else  if( array_lvbo_flag[4][4] < 2 ) //// array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		array_five[0]=array_five[1]-4 ; // 1
	  			if( array_five[0]<0 )   array_five[0]=0;
	  		array_lvbo_flag[4][4]++;
	  	}
	  	else  if( array_lvbo_flag[4][5] < 3 ) //// array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		array_five[0]=array_five[1]-8 ; // 1
	  			if( array_five[0]<0 )   array_five[0]=0;
	  		array_lvbo_flag[4][5]++;
	  	}
	  	else if( (array_five[0]-array_five[6]) > -60 ) // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 6 ´ÎÒÔÄÚ»¹Î´»Ø¹é
	  	    {
	  	    	        array_five[0]=array_five[6] - 40 ;
	  	    	 	if( array_five[0]<0 )   array_five[0]=0;
	  	    }  
	  	else  
	  	   {
	  	   	array_five[0]=array_five[1]-15 ; 
	  	   	if( array_five[0]<0 )   array_five[0]=0;	
	  	   }    
 	  }
 	  else
 	  {
 	     	array_lvbo_flag[4][0] = 0 ;
	        array_lvbo_flag[4][1] = 0 ;
	        array_lvbo_flag[4][2] = 0 ;
	        array_lvbo_flag[4][3] = 0 ;
	        array_lvbo_flag[4][4] = 0 ;
	        array_lvbo_flag[4][5] = 0 ;
 	  }
     	  // ========    ÏòÏÂÏÞ·ù    =========//
	   if( (array_five[0]-array_five[1]) > -3 && (array_five[0]-array_five[1]) < 3  )    //======= ²¨¶¯ÏÞ·ù´óÐ¡¡ª¡ª5£¬11 ¡ª¡ª +3 £¬+7=====//
	  {
	        array_lvbo_flag[4][6] ++ ;
	        if( array_lvbo_flag[4][6] == 10 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 5 ÒÔÄÚ
	    	 {
	    	  	array_temp[4]=( array_five[0]+array_five[1]+array_five[2]+array_five[3]+array_five[4]+array_five[5]+array_five[6]+array_five[7]+array_five[8]+array_five[9]+5 ) /10 ; // 1
                	if( (array_temp[4]-array_five[0]) > -3 && (array_temp[4]-array_five[0]) < 3  )    
	   		  	array_five[0] = array_temp[4] ;
	   		else 
	   		        array_lvbo_flag[4][6] = 0 ;
		 }
	  	else if( array_lvbo_flag[4][6] > 10 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		array_five[0] = array_five[1] ;
	  		array_lvbo_flag[4][6] -- ;
	  	}	
	  }
	  else
	  	array_lvbo_flag[4][6] = 0 ;
	  
	  
	   //========================== Æ«²î´¦Àí ===========================//
        if( array_one[0] < 2 )    // ·ÀÖ¹   ·ÖÄ¸ µç¸ÐÖµ   ÎªÁã
                array_one[0] = 1 ;
       if( array_two[0] < 2 )    // ·ÀÖ¹   ·ÖÄ¸ µç¸ÐÖµ   ÎªÁã
                array_two[0] = 1 ;
        if( array_three[0] < 2 )    // ·ÀÖ¹   ·ÖÄ¸ µç¸ÐÖµ   ÎªÁã
                array_three[0] = 1 ;
       if( array_four[0] < 2 )    // ·ÀÖ¹   ·ÖÄ¸ µç¸ÐÖµ   ÎªÁã
                array_four[0] = 1 ;
        if( array_five[0] < 2 )    // ·ÀÖ¹   ·ÖÄ¸ µç¸ÐÖµ   ÎªÁã
                array_five[0] = 1 ;

 	  	if( array_six[0] < 2 )    // ·ÀÖ¹   ·ÖÄ¸ µç¸ÐÖµ   ÎªÁã
                array_six[0] = 1 ;    
 	  
 //=================================================//
 //ÓÉÓÚ²ÉÑùÊÇÒÑ¾­¶à´Î²É¼¯ÁË£¬²¢ÇÒÓÃÁËÆ½¾ùÂË²¨£¬ËùÒÔ²»ÔÙÂË²¨£¬ºóÐøÔÙ¿´
 //ÊµÊ±½«Í¬Ò»Ê±¿ÌµÄµç¸ÐÖµ·ÅÔÚÍ¬Ò»¸öÊý×éÀï
	  	array_current[0] = array_one[0] ;//     ¶ÔÓ¦µç¸Ð1
	  	array_current[1] = array_two[0] ;//	     ¶ÔÓ¦µç¸Ð2
	  	array_current[2] = array_three[0] ;//     ¶ÔÓ¦µç¸Ð3
	  	array_current[3] = array_four[0];//       ¶ÔÓ¦µç¸Ð4
        array_current[4] = array_five[0] ;//       ¶ÔÓ¦µç¸Ð5 		
        array_current[5]=array_six[0];
  
    // noise_protect_first();
    
    //µç¸ÐÅÅ²¼
    ///4========5
    // 0==1==2==3 
    
     
       chui_zhi_daingan[0]=array_current[4]-array_current[5]; //´¹Ö±µç¸ÐÖ®²î               
       Verticaldiangan_sum[0]=array_current[4]+array_current[5]; //´¹Ö±µç¸ÐÖ®ºÍ
       Leveltaldiangan_sum[0]=array_current[1]+array_current[2];//Ë®Æ½µç¸ÐÖ®ºÍ
       xiezhi_diangan_sum[0]= array_current[0]+array_current[3];//Ð±ÖÃµç¸ÐÖ®ºó
       total_diangan_sum[0]=xiezhi_diangan_sum[0]+Leveltaldiangan_sum[0];//¶þÅÅµç¸Ð×ÜºÍ 
                             //SCI0_SendChar_16(three_diangan_sum[0]);
 		                     //SCI0_SendChar_16(two_diangan_sum[0]);
 		                    //SCI0_SendChar_16(array_three[0]);
 		                    //SCI0_SendChar_16(array_four[0]);
		                    //SCI0_SendChar_16(array_five[0]); 
          
}
//´æÏÂÆ«²îÔÚ¸÷¸ö·¶Î§Ê±µÄ´ò½Ç
//½¨Á¢Ä£ºýmap±í¸ñ£¬ÓÃÓÚ²éÑ¯×ªÏò£¿£¿£¿
//===================================Æ«ÒÆÁ¿º¯Êý==========================//
void smartcar_position()
{
 
    if(Leveltaldiangan_sum[0]>320)
    {
       if(diangan_max[0]>210 )
       {
       	     
       	  if(shuiping_piancha[0]*xiezhi_piancha[0]>0)
       	        	  servo_Error[0]= 100*shuiping_piancha[0]/100;
                      //+xiezhi_piancha[0]/10 ;	
       	  
       	  else
       	   	   
       	                servo_Error[0]= 100*shuiping_piancha[0]/100;
                       //-xiezhi_piancha[0]/10 ;	

       }
       else
       {
       
       
           if(xiezhi_piancha[0]*chuizhi_piancha[0]>0)
            {   
           
               if( Road_Type[0]==3)
                   servo_Error[0]= 100*shuiping_piancha[0]/100 ;
               else
                   servo_Error[0]= 70*xiezhi_piancha[0]/100 
                                  +70*chuizhi_piancha[0]/100 ;
                       
            }
         else
         { 
            if( Road_Type[0]==3)
            servo_Error[0]= 100*shuiping_piancha[0]/100;
                                 
         	else
            servo_Error[0]= 70*xiezhi_piancha[0]/100 
                            +70*chuizhi_piancha[0]/100 ;
                       
         }
    	
    }
       
     
    }
    else
    {
       if(xiezhi_piancha[0]*chuizhi_piancha[0]>0)
       {              
            if( Road_Type[0]==3) 
	        { 
	           if(shuiping_piancha[0]*chuizhi_piancha[0]>0)
	       	       servo_Error[0]= 100*shuiping_piancha[0]/100;
	                             // +chuizhi_piancha[0]/10 ;	  
	       	   else
	       	     	   
	       	       servo_Error[0]= 100*shuiping_piancha[0]/100;
	                              //- chuizhi_piancha[0]/10 ;	
	        }
	        
	        else  servo_Error[0]= 70*xiezhi_piancha[0]/100
                          +70*chuizhi_piancha[0]/100 ;          
           
       }
       
       else
         {  
                      
            if( Road_Type[0]==3) 
	        {
 
	           if(shuiping_piancha[0]*chuizhi_piancha[0]>0)
	       	   { 	   
	       	       servo_Error[0]= 100*shuiping_piancha[0]/100
	                              + chuizhi_piancha[0]/10 ;	
	       	   }
	       	   else
	       	   { 	   
	       	       servo_Error[0]= 100*shuiping_piancha[0]/100
	                              - chuizhi_piancha[0]/10 ;	
	       	   }          
	           
	           
	       }
	       
	       
	       
	       else  servo_Error[0]= 70*xiezhi_piancha[0]/100
                     +70*chuizhi_piancha[0]/100 ;
       }
    	
    }
 
  	
}


  
//²Î¿¼
//if( chuizhi_piancha[0]>=0)   chuizhi_piancha_xishu=chuizhi_piancha[0]*100/6+400 ;
//else chuizhi_piancha_xishu=-chuizhi_piancha[0]*100/6+400 ;
//if(chuizhi_piancha_xishu<500)    chuizhi_piancha_xishu=500 ;
//if(chuizhi_piancha_xishu>1300)    chuizhi_piancha_xishu=1300 ;
//xiezhi_piancha_xishu=1300-chuizhi_piancha_xishu ;
//if(xiezhi_piancha_xishu<500)    xiezhi_piancha_xishu=500 ;
//servo_Error[0]= xiezhi_piancha_xishu*xiezhi_piancha[0]/1000+chuizhi_piancha_xishu*chuizhi_piancha[0]/1000;

 
//==========================²âÊÔ³ÌÐò==========================//
void smartcar_position_NEW()
{
            //===============================================
            if( piancha_E_qiujie_flag==7 )
            {SIU.GPDO[PCR38_PC6].R =0;
	 /*
	 
	*/	    if( chuizhi_piancha[0]>=0 )   chuizhi_piancha_xishu=chuizhi_piancha[0]*100/6+400 ;
		    else    chuizhi_piancha_xishu=-chuizhi_piancha[0]*100/6+400 ;
		    if( chuizhi_piancha_xishu<500 )    chuizhi_piancha_xishu=500 ;
		    if( chuizhi_piancha_xishu>1000 )    chuizhi_piancha_xishu=1000 ;
		     
		     
		   ///  Verticaldiangan_sum[0]
		     xiezhi_piancha_xishu=1000-chuizhi_piancha_xishu ;
		     
		     
		     
		     if( xiezhi_piancha_xishu<500 )    xiezhi_piancha_xishu=500 ;
		     
		       //  if( ABS(array_current[4],array_current[5])<40 && ABS(array_current[0],array_current[3])>200 )
		     //	    xiezhi_piancha_xishu=-xiezhi_piancha_xishu/20 ;
		     //    if( array_current[4]>80 && array_current[5]>80 )
		     ///	    xiezhi_piancha_xishu=-xiezhi_piancha_xishu/20 ;
		         
		     servo_Error[0]= xiezhi_piancha_xishu*xiezhi_piancha[0]/1200+chuizhi_piancha_xishu*chuizhi_piancha[0]/1200;
	 
	  

	 /*  */
		     if(Leveltaldiangan_sum[0]>320)
	            {
			       if((diangan_max[0]>210)&&(array_current[4]>50)&&(array_current[5])>50)
			       {
				       	   if(shuiping_piancha[0]*xiezhi_piancha[0]>0)	   
				                   servo_Error[0]= 120*shuiping_piancha[0]/100 +  xiezhi_piancha[0]/7;	
				          else 
				                   servo_Error[0]= 120*shuiping_piancha[0]/100 -  xiezhi_piancha[0]/7 ;
			       }
			       else   if( Road_Type[0]==3)
	                   servo_Error[0]= 100*shuiping_piancha[0]/100 ;
	            }
	            else if( Road_Type[0]==3)
	                   servo_Error[0]= 100*shuiping_piancha[0]/100 ;

            }
            
            
           //============================================ 
            else
            {SIU.GPDO[PCR38_PC6].R =1;
		            		         //ÇóÈ¡¸÷¸öÆ«ÒÆÁ¿µÄÂÛÓò£¬ÔÚÇø¼ä½øÐÐ
		         KK_chuizhi[0]=qu_zheng(chuizhi_piancha[0])/6; 
		         KK_shuiping[0]=qu_zheng(shuiping_piancha[0])/6;  
		         KK_xiezhi[0]= qu_zheng(xiezhi_piancha[0])/6;
		  
		          if(KK_shuiping[0]>16)  KK_shuiping[0]=16;
		          if(KK_chuizhi[0]>16)  KK_chuizhi[0]=16;
		          if(KK_xiezhi[0]>16)  KK_xiezhi[0]=16;
		 
		      
		      //=======ÐÞÕýÐ±ÖÃµç¸ÐÆ«ÒÆÁ¿=======================//
		      if( Road_Type[0]==3)
		      {
		      	  if(KK_xiezhi[0]<2&&KK_chuizhi[0]>12)     
		           xiezhi_piancha[0]= xiezhi_piancha[0]/10;
		      }
		     //=================================================//
		      if(Leveltaldiangan_sum[0]>320)
		      {
		         //¼ì²âµ½Ê®×Ö£¬²»ÓÃÐ±ÖÃµç¸ÐµÄÆ«ÒÆÁ¿
		         if( Road_Type[0]==3)
		         {
		             if(shuiping_piancha[0]*chuizhi_piancha[0]>0)
		       	            	 servo_Error[0]= 110*shuiping_piancha[0]/100
		                                          +chuizhi_piancha[0]/7 ;	
		       	     else
		       	  	       	     servo_Error[0]= 120*shuiping_piancha[0]/100
		                                          -chuizhi_piancha[0]/7 ;	
		         	
		         }
		        //×î´óµç¸ÐºÜ´óÊ±£¬ºÜ¿ÉÄÜÐ±×ÅÈëÊ®×Ö£¬ÓÃË®Æ½Æ«ÒÆÁ¿
		        else if((diangan_max[0]>210)&&(array_current[4]>40)&&(array_current[5])>40)
		        {
		             if(shuiping_piancha[0]*chuizhi_piancha[0]>0)
		       	            	 servo_Error[0]= 110*shuiping_piancha[0]/100
		                                          +chuizhi_piancha[0]/7 ;	
		       	     else
		       	  	       	     servo_Error[0]= 120*shuiping_piancha[0]/100
		                                          -chuizhi_piancha[0]/7 ;	
		         	
		         }
		       else  
		       { 
		           servo_Error[0]= 70*shuiping_piancha[0]/100
		                           +70*chuizhi_piancha[0]/100 ;
		       }
		    }
		     //==================================//  
		    else
		    { 
		    
		         //¼ì²âµ½Ê®×Ö£¬²»ÓÃÐ±ÖÃµç¸ÐµÄÆ«ÒÆÁ¿
		         if( Road_Type[0]==3)
		         {
		             if(shuiping_piancha[0]*chuizhi_piancha[0]>0)
		       	            	 servo_Error[0]= 110*shuiping_piancha[0]/100
		                                          +chuizhi_piancha[0]/7 ;	
		       	     else
		       	  	       	     servo_Error[0]= 120*shuiping_piancha[0]/100
		                                          -chuizhi_piancha[0]/7 ;	
		         	
		         }
		        //×î´óµç¸ÐºÜ´óÊ±£¬ºÜ¿ÉÄÜÐ±×ÅÈëÊ®×Ö£¬ÓÃË®Æ½Æ«ÒÆÁ¿
		        else if((diangan_max[0]>210)&&(array_current[4]>40)&&(array_current[5])>40)
		        {
		             if(shuiping_piancha[0]*chuizhi_piancha[0]>0)
		       	            	 servo_Error[0]= 110*shuiping_piancha[0]/100
		                                          +chuizhi_piancha[0]/7 ;	
		       	     else
		       	  	       	     servo_Error[0]= 120*shuiping_piancha[0]/100
		                                          -chuizhi_piancha[0]/7 ;	
		         	
		         }
		    
		    
		        else 
		           {
		          
		         	 servo_Error[0]= 70*chuizhi_piancha[0]/100
		                               +70*xiezhi_piancha[0]/100 ;
		            }
		    
		      }
		 
            }
 
 
}


//======================================================================//
 
 //==================================================================//
void  yu_chu_li( void )
{
       int i;
       diangan_max[0]=0;//ÇåÁã´¦Àí
 diangan_maxfour[0]=0;
       for(i=0;i<4;i++)
       {
        
          if( diangan_max [0] < array_current[i] )//ÕÒ³öËÄ¸öµç¸ÐµÄ×î´óÖµ£¬
          {
         	 diangan_max [0]=array_current[i]  ; //ÒÔ¼°ÕÒ³öÊÇµÚ¼¸¸öµç¸ÐÊÇ×î´óÖµ 
       	     cixu[0][0]=i ;
          }
      
       }
       
       
 
       //=================´¹Ö±µç¸Ð×î´óÖµ
       if(array_current[5]>array_current[6])
       chuizhidiangan_max[0]=array_current[5];
       else chuizhidiangan_max[0]=array_current[6];
      /*
       if(cixu[0][0]==0)     //×ó±ß×î´ó:0
       {
      	 if(array_current[1]>array_current[2]) 
      	 {
      	 	cixu[0][1]=1;
      	 	cixu[0][2]=2;
      	 }
      	 else
      	 {   
      	     cixu[0][1]=2;
      	 	 cixu[0][2]=1;	
      	 }
       }
       else if(cixu[0][0]==1)//ÖÐ¼ä×î´ó:1
       {
          if(array_current[0]>array_current[2]) 
      	 {
      	 	cixu[0][1]=0;
      	 	cixu[0][2]=2;
      	 }
      	 else
      	 {   
      	     cixu[0][1]=2;
      	 	 cixu[0][2]=0;	
      	 }
      }
          
      else                 //ÓÒ±ß×î´ó:2
      {
         if(array_current[0]>array_current[1]) 
      	 {
      	 	cixu[0][1]=0;
      	 	cixu[0][2]=1;
      	 }
      	 else
      	 {   
      	     cixu[0][1]=1;
      	 	 cixu[0][2]=0;	
      	 }
	
      }
     diangan_min[0]=array_current[cixu[0][2]];//´¢´æ×îÐ¡Öµ
     */
  //Ð±ÖÃµç¸Ð´óÐ¡   
     /*if(array_current[3]>array_current[4])
         xiediangan_max[0]=array_current[3];
     else 
         xiediangan_max[0]=array_current[4];
     
     */
     //ÖÐ¼äµç¸ÐÓëÁ½±ßµç¸ÐÖ®²î
     
    /* 
     if(cixu[0][0]==1)//ÖÐ¼ä×î´ó
         middle_piancha[0]==0;
     else if(cixu[0][0]==0)//×ó±ß×î´ó
     {
        middle_piancha[0]=array_current[0]-array_current[1];
        	
     }
     else if(cixu[0][0]==2)
     {
     	 middle_piancha[0]=array_current[1]-array_current[2];
     }
     */
      
     
     
     
      //========================== Æ«²î´¦Àí ===========================//
        if( array_current[0] < 2 )    // ·ÀÖ¹   ·ÖÄ¸ µç¸ÐÖµ   ÎªÁã
                array_current[0] = 1 ;
       if( array_current[1] < 2 )    // ·ÀÖ¹   ·ÖÄ¸ µç¸ÐÖµ   ÎªÁã
                array_current[1] = 1 ;
        if( array_current[2] < 2 )    // ·ÀÖ¹   ·ÖÄ¸ µç¸ÐÖµ   ÎªÁã
                array_current[2] = 1 ;
       if( array_current[3] < 2 )    // ·ÀÖ¹   ·ÖÄ¸ µç¸ÐÖµ   ÎªÁã
                array_current[3] = 1 ;
       if( array_current[4] < 2 )    // ·ÀÖ¹   ·ÖÄ¸ µç¸ÐÖµ   ÎªÁã
                array_current[4] = 1 ;
        
      if( array_current[5] < 2 )    // ·ÀÖ¹   ·ÖÄ¸ µç¸ÐÖµ   ÎªÁã
                array_current[5] = 1 ;
      if( array_current[6] < 2 )    // ·ÀÖ¹   ·ÖÄ¸ µç¸ÐÖµ   ÎªÁã
                array_current[6] = 1 ;
        
        
      //Ë®Æ½µç¸ÐÆ«²î
      shuiping_piancha[0] = 250* ( array_current[1] - array_current[2] ) /( array_current[1] + array_current[2] )  ;
      //shuiping_piancha[0] = 40 * ( array_current[0] - array_current[2] ) /( array_current[0]+array_current[2] )*3 ;
        //´¹Ö±µç¸ÐÆ«²î  
      chuizhi_piancha[0] =135 * ( array_current[4] - array_current[5] )/( array_current[1] + array_current[2] )  ;
   
       xiezhi_piancha[0]=135*(array_current[0] - array_current[3] )/(array_current[0] + array_current[3] ); 
    
    
      //chuizhi_piancha_fs[0]=80*( array_current[3] - array_current[4] )/( array_current[1]);
      if( shuiping_piancha[0]>100)         shuiping_piancha[0]=100;
      else if(shuiping_piancha[0]<-100)    shuiping_piancha[0]=-100;
      
      if(chuizhi_piancha[0]>100)           chuizhi_piancha[0]=100;
      else if(chuizhi_piancha[0]<-100)     chuizhi_piancha[0]=-100;
     
     if(xiezhi_piancha[0]>100)             xiezhi_piancha[0]=100;
       else if(xiezhi_piancha[0]<-100)     xiezhi_piancha[0]=-100;
    
    
    
    if(Road_Type[0]==3)
    {
       if( shuiping_piancha[1]*shuiping_piancha[0]<0&&qu_zheng(shuiping_piancha[1])>50)
       {
       	   shuiping_piancha[0]=shuiping_piancha[1];
       }
    }
    
    
    
   //SCI0_SendChar_16(shuiping_piancha[0]);
   //SCI0_SendChar_16(chuizhi_piancha[0]);
         
   //================== Ë®Æ½µç¸ÐÏÞ·ù´¦Àí  ============== //   
 	  if( (shuiping_piancha[0]-shuiping_piancha[1]) > 3 )    //======= ²¨¶¯ÏÞ·ù´óÐ¡¡ª¡ª5£¬11 ¡ª¡ª +3 £¬+7=====//
	  {
	        shuiping_piancha_flag[3] = 0 ;
	        shuiping_piancha_flag[4] = 0 ;
	        shuiping_piancha_flag[5] = 0 ;
	        if( shuiping_piancha_flag[0] < 1 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 5 ÒÔÄÚ
	        {
	        	shuiping_piancha[0]=shuiping_piancha[1]+2 ; // 1
	        	shuiping_piancha_flag[0]++;
	        }
	  	else  if( shuiping_piancha_flag[1] < 1 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		shuiping_piancha[0]=shuiping_piancha[1]+4 ; // 1
	  		shuiping_piancha_flag[1]++;
	  	}
	       else  if( shuiping_piancha_flag[2] < 2 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		shuiping_piancha[0]=shuiping_piancha[1]+7 ; // 1
	  		shuiping_piancha_flag[2]++;
	  	}
	  	else if( (shuiping_piancha[0]-shuiping_piancha[5]) < 50 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 6 ´ÎÒÔÄÚ»¹Î´»Ø¹é
	  	       shuiping_piancha[0]=shuiping_piancha[5] + 32 ;
	  	else  
	  	       shuiping_piancha[0]=shuiping_piancha[1]+12 ; // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÔÚ 7 ´ÎÒÔÄÚ»¹Î´»Ø¹é
	  } 
 	  else if( (shuiping_piancha[0]-shuiping_piancha[1]) < -3 )   //======= ²¨¶¯ÏÞ·ù´óÐ¡¡ª¡ª -5£¬-11 ¡ª¡ª -3 £¬-7=====//
 	  {
	        shuiping_piancha_flag[0] = 0 ;
	        shuiping_piancha_flag[1] = 0 ;
	        shuiping_piancha_flag[2] = 0 ;
	        if( shuiping_piancha_flag[3] < 1 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 5 ÒÔÄÚ
	        {
	        	shuiping_piancha[0]=shuiping_piancha[1]-2 ; // 1
	        	shuiping_piancha_flag[3]++;
	        }
	  	else  if( shuiping_piancha_flag[4] < 1 ) //// array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		shuiping_piancha[0]=shuiping_piancha[1]-4 ; // 1
	  		shuiping_piancha_flag[4]++;
	  	}
	  	else  if( shuiping_piancha_flag[5] < 2 ) //// array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		shuiping_piancha[0]=shuiping_piancha[1]-7 ; // 1
	  		shuiping_piancha_flag[5]++;
	  	}
	  	else if( (shuiping_piancha[0]-shuiping_piancha[7]) > -50 ) // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 6 ´ÎÒÔÄÚ»¹Î´»Ø¹é
	  	       shuiping_piancha[0]=shuiping_piancha[7] - 32 ;
	  	else  
	  	    {
	  	    	shuiping_piancha[0]=shuiping_piancha[1]-12 ; 
	  	    	
	  	    }   
 	  }
 	  else
 	  {
 	     	 shuiping_piancha_flag[0] = 0 ;
	        shuiping_piancha_flag[1] = 0 ;
	        shuiping_piancha_flag[2] = 0 ;
	        shuiping_piancha_flag[3] = 0 ;
	        shuiping_piancha_flag[4] = 0 ;
	        shuiping_piancha_flag[5] = 0 ;
 	  }

   
   //================== ´¹Ö±µç¸ÐÏÞ·ù´¦Àí  ============== //   
 	  if( (chuizhi_piancha[0]-chuizhi_piancha[1]) > 3 )    //======= ²¨¶¯ÏÞ·ù´óÐ¡¡ª¡ª5£¬11 ¡ª¡ª +3 £¬+7=====//
	  {
	        chuizhi_piancha_flag[3] = 0 ;
	        chuizhi_piancha_flag[4] = 0 ;
	        chuizhi_piancha_flag[5] = 0 ;
	        if( chuizhi_piancha_flag[0] < 1 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 5 ÒÔÄÚ
	        {
	        	chuizhi_piancha[0]=chuizhi_piancha[1]+2 ; // 1
	        	chuizhi_piancha_flag[0]++;
	        }
	  	else  if( chuizhi_piancha_flag[1] < 1 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		chuizhi_piancha[0]=chuizhi_piancha[1]+4 ; // 1
	  		chuizhi_piancha_flag[1]++;
	  	}
	       else  if( chuizhi_piancha_flag[2] < 2 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		chuizhi_piancha[0]=chuizhi_piancha[1]+8 ; // 1
	  		chuizhi_piancha_flag[2]++;
	  	}
	  	else if( (chuizhi_piancha[0]-chuizhi_piancha[5]) < 50 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 6 ´ÎÒÔÄÚ»¹Î´»Ø¹é
	  	       chuizhi_piancha[0]=chuizhi_piancha[5] + 35 ;
	  	else  
	  	       chuizhi_piancha[0]=chuizhi_piancha[1]+15 ; // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÔÚ 7 ´ÎÒÔÄÚ»¹Î´»Ø¹é
	  } 
 	  else if( (chuizhi_piancha[0]-chuizhi_piancha[1]) < -3 )   //======= ²¨¶¯ÏÞ·ù´óÐ¡¡ª¡ª -5£¬-11 ¡ª¡ª -3 £¬-7=====//
 	  {
	        chuizhi_piancha_flag[0] = 0 ;
	        chuizhi_piancha_flag[1] = 0 ;
	        chuizhi_piancha_flag[2] = 0 ;
	        if( chuizhi_piancha_flag[3] < 1 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 5 ÒÔÄÚ
	        {
	        	chuizhi_piancha[0]=chuizhi_piancha[1]-2 ; // 1
	        	chuizhi_piancha_flag[3]++;
	        }
	  	else  if( chuizhi_piancha_flag[4] < 1 ) //// array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		chuizhi_piancha[0]=chuizhi_piancha[1]-4 ; // 1
	  		chuizhi_piancha_flag[4]++;
	  	}
	  	else  if( chuizhi_piancha_flag[5] < 2 ) //// array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		chuizhi_piancha[0]=chuizhi_piancha[1]-8 ; // 1
	  		chuizhi_piancha_flag[5]++;
	  	}
	  	else if( (chuizhi_piancha[0]-chuizhi_piancha[5]) > -50 ) // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 6 ´ÎÒÔÄÚ»¹Î´»Ø¹é
	  	       chuizhi_piancha[0]=chuizhi_piancha[5] - 35 ;
	  	else  
	  	       chuizhi_piancha[0]=chuizhi_piancha[1]-15 ; 
 	  }
 	  else
 	  {
 	     	 chuizhi_piancha_flag[0] = 0 ;
	         chuizhi_piancha_flag[1] = 0 ;
	         chuizhi_piancha_flag[2] = 0 ;
	         chuizhi_piancha_flag[3] = 0 ;
	         chuizhi_piancha_flag[4] = 0 ;
	         chuizhi_piancha_flag[5] = 0 ;
 	  }
 
         
         
         shuiping_piancha[0] =( 90*shuiping_piancha[0] + 7*shuiping_piancha[1] + 2*shuiping_piancha[2] + 1*shuiping_piancha[3] )/100 ;
         chuizhi_piancha[0] = ( 90*chuizhi_piancha[0] + 7*chuizhi_piancha[1] + 2*chuizhi_piancha[2] + 1*chuizhi_piancha[3] )/100 ; 
         
         
   
    
//=================================Æ«ÒÆÁ¿µÄ´¦Àí===========================//
  // smartcar_position();
  // servo_Error[0]= 0*shuiping_piancha[0]/20+20*chuizhi_piancha[0]/20;
     
    smartcar_position_NEW();   
                           
//========================================================================//   
    //°ÑÆ«²î±êÔÚÕý¸º100ÒÔÄÚ
     if(servo_Error[0]>100)    servo_Error[0]=100;
     if(servo_Error[0]<-100)   servo_Error[0]=-100;
    
        //SCI0_SendChar_16(servo_Error[0]);
      //==================   Æ«²î  ÏÞ·ù´¦Àí  ============== //   
   if( (servo_Error[0]-servo_Error[1]) > 6 )    //======= ²¨¶¯ÏÞ·ù´óÐ¡¡ª¡ª5£¬11 ¡ª¡ª +3 £¬+7=====//
	  {
	        servo_Error_flag[3] = 0 ;
	        servo_Error_flag[4] = 0 ;
	        servo_Error_flag[5] = 0 ;
	        if( servo_Error_flag[0] < 1 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 5 ÒÔÄÚ
	        {
	        	servo_Error[0]=servo_Error[1]+4 ; // 1
	        	servo_Error_flag[0]++;
	        }
	  	else  if( servo_Error_flag[1] < 1 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		servo_Error[0]=servo_Error[1]+8 ; // 1
	  		servo_Error_flag[1]++;
	  	}
	       else  if( servo_Error_flag[2] < 2 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		servo_Error[0]=servo_Error[1]+12 ; // 1
	  		servo_Error_flag[2]++;
	  	}
	  	else if( (servo_Error[0]-servo_Error[5]) < 60 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 6 ´ÎÒÔÄÚ»¹Î´»Ø¹é
	  	       servo_Error[0]=servo_Error[5] + 45 ;
	  	else  
	  	       servo_Error[0]=servo_Error[1]+15 ; // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÔÚ 7 ´ÎÒÔÄÚ»¹Î´»Ø¹é
	  } 
 	  else if( (servo_Error[0]-servo_Error[1]) < -6 )   //======= ²¨¶¯ÏÞ·ù´óÐ¡¡ª¡ª -5£¬-11 ¡ª¡ª -3 £¬-7=====//
 	  {
	        servo_Error_flag[0] = 0 ;
	        servo_Error_flag[1] = 0 ;
	        servo_Error_flag[2] = 0 ;
	        if( servo_Error_flag[3] < 1 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 5 ÒÔÄÚ
	        {
	        	servo_Error[0]=servo_Error[1]-4 ; // 1
	        	servo_Error_flag[3]++;
	        }
	  	else  if( servo_Error_flag[4] < 1 ) //// array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		servo_Error[0]=servo_Error[1]-8 ; // 1
	  		servo_Error_flag[4]++;
	  	}
	  	else  if( servo_Error_flag[5] < 2 ) //// array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		servo_Error[0]=servo_Error[1]-12 ; // 1
	  		servo_Error_flag[5]++;
	  	}
	  	else if( (servo_Error[0]-servo_Error[5]) > -60 ) // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 6 ´ÎÒÔÄÚ»¹Î´»Ø¹é
	  	       servo_Error[0]=servo_Error[5] - 45 ;
	  	else  
	  	       servo_Error[0]=servo_Error[1]-15 ; 
 	  }
 	  else
 	  {
 	     	servo_Error_flag[0] = 0 ;
	        servo_Error_flag[1] = 0 ;
	        servo_Error_flag[2] = 0 ;
	        servo_Error_flag[3] = 0 ;
	        servo_Error_flag[4] = 0 ;
	        servo_Error_flag[5] = 0 ;
 	  } 

   
       servo_Error[0]=90*servo_Error[0]/100+7*servo_Error[1]/100+3*servo_Error[2]/100;
       servo_Error_c[0] =( (servo_Error[0]+servo_Error[1]+servo_Error[2]) - (servo_Error[3]+servo_Error[4]+servo_Error[5]) )/3 ;
            
       servo_Error_c[0]=er_cheng_fa_ni_he( 30 ,  servo_Error) ;
        servo_Error_c[0]=servo_Error_c[0]/3 ;                
                      //SCI0_SendChar_16(servo_Error[0]);     
                   // SCI0_SendChar_16(servo_Error_c[0]) ;
                   
                   
                   
     //servo_Error_c[0]= er_cheng_fa_ni_he( 20 ,  diangan_max) ;              

}


//ÈýÌõÊý¾Ý±£»¤´ëÊ©£¬ÒÔÊ¶±ð²¢´¦Àí´íÎóÊý¾Ý¡£
//(1)ÔÚ²àÏòµç¸ÐL1¡¢L2ºÍL3µÄ¸ÐÓ¦µç¶¯ÊÆ¾ùºÜÐ¡µÄÇé¿öÏÂ£¬
//¼´¿ÉÈ·¶¨´ËÊ±¿Ï¶¨´¦ÓÚÍäµÀ£¬´ËÊ±²»ÄÜÈÃ×î´ó¸ÐÓ¦µç¶¯ÊÆµÄµç¸ÐÐòºÅ·¢Éú±ä»¯£¬
//¼´½øÈë×ó×ªÍä£¬Ó¦µ±±£³Öµç¸ÐL1µÄ¸ÐÓ¦µç¶¯ÊÆ×î´ó£¬
//½øÈëÓÒ×ªÍä£¬Ó¦µ±³Öµç¸ÐL3µÄ¸ÐÓ¦µç¶¯ÊÆ×î´ó
//Èô´ËÊ±×î´óµç¸Ð·¢Éú±ä»¯£¬¿ÉÈÏÎªÊý¾Ý·¢Éú´íÎó,µ±½øÐÐÇ¿ÖÆµ÷Õû¡
//(2)´ÓÀíÂÛÉÏ·ÖÎö£¬L2µÄ¸ÐÓ¦µç¶¯ÊÆÊ¼ÖÕÎª×î´óÖµ»òÖÐ¼äÖµ£¬
//ÈôÎª×îÐ¡Öµ£¬¿ÉÈÏÎªÊý¾Ý´íÎó£¬Í¬Ñù½øÐÐÇ¿ÖÆµ÷Õû¡£
//(3)µ±L2µÄ¸ÐÓ¦µç¶¯ÊÆ·Ç³£Ð¡µÄÇé¿öÏÂ£¬
//²ÉÈ¡·ÀÖ¹Çø¼ä±ê¶¨½á¹û·¢Éú´ó·ù¶ÈÌø±ä´ëÊ©¡£
 void noise_protect_first()
 {
 
    if(protect_Road_Type[0]==1&&total_diangan_sum[0]<240)//×óÍäµÀ
    {  //Èç¹û²»ÊÇ×î×ó±ßµÄ×î´ó Ôò½«Ç¿ÖÆµ÷ÕûÎª×î´óÖµ
       if(cixu[0][0]!= 0)
       {
       
       
         if(array_current[0]<array_current[2]&&array_current[3]<array_current[4])
         {
          	array_current[0]=array_current[cixu[1][0]];
          	array_current[1]=array_current[cixu[1][1]];
            array_current[2]=array_current[cixu[1][2]];
        
        }	
       }
       
    }	
    
    if(protect_Road_Type[0]==2&&total_diangan_sum[0]<240)//ÓÒÍäµÀ
	{//Èç¹û²»ÊÇ×îÓÒ±ßµÄ×î´ó Ôò½«Ç¿ÖÆµ÷ÕûÎª×î´óÖµ
	    if(cixu[0][0]!= 2)
	    {
	       if(array_current[0]>array_current[2]&&array_current[3]>array_current[4])
	    	array_current[2]=array_current[cixu[1][0]];
	    	array_current[1]=array_current[cixu[1][1]];
	    	array_current[0]=array_current[cixu[1][2]];
	    	
	    }
    }	
 	//Èç¹û×îÐ¡µÄµç¸Ð´ÎÐòÎª1£¬¼´ÖÐ¼äµç¸ÐÎª×îÐ¡£¬ÔòÈÏÎª·¢Éú´íÎó£¬Ç¿ÖÆµ÷Õû
 	if(cixu[0][2]==1) 
 	{
 	    array_current[1]=array_current[cixu[1][1]];
   		
 	}
 	    
 	
 }
//===================================================// 
 void noise_protect_second()
 {
 
      if(protect_Road_Type[0]==1)//×óÍäµÀ
      {
      
      
        	
        if(total_diangan_sum[0]<270&&servo_output<0 )  servo_output= servo_output_last[jiao_biao_max];
      	 
      }
      
      else if(protect_Road_Type[0]==2)
      {
      
          
      	  if(total_diangan_sum[0]<270&&servo_output>0 )  servo_output= servo_output_last[jiao_biao_max];   
      }
   
      else if(Road_Type[0]==3)
     {
     	//if(servo_output>100)  servo_output= 100;
     	
     	//else if(servo_output<-100)  servo_output= -100;
     }
 	
 }


//¸ù¾ÝÊµ¼ÊÇé¿ö£¬½«ÈüµÀÀàÐÍ·ÖÎª:
//Ö±µÀ¡¢×óÍäµÀ¡¢ÓÒÍäµÀ¡¢Ê®×Ö½»²æµãÒÔ¼°ÆÂµÀÎåÀà
//¶ÔÈüµÀ½øÐÐ·ÖÀàµÄÖ÷Òª×÷ÓÃÊÇ£¬ÔÚ²»Í¬ÀàÐÍµÄÈüµÀÉè¶¨²»Í¬µÄÆÚÍûËÙ¶È¡£
//²»Í¬ÀàÐÍÈü´ïÊ¶±ð·½·¨ÈçÏÂ£º
//(1)Ö±µÀ£º²àÏòµç¸ÐL1¡¢L2ºÍL3µÄ¸ÐÓ¦µç¶¯ÊÆºÍÖµ´óÓÚÒ»¶¨Öµ£¬
//×ÝÏòµç¸ÐV1ºÍ V2µÄ¸ÐÓ¦µç¶¯ÊÆ²îÖµµÄ¾ø¶ÔÖµÐ¡ÓÚÒ»¶¨Öµ
//=================================================================//
//(2)ÍäµÀ£º²àÏòµç¸ÐL1¡¢L2ºÍL3µÄ¸ÐÓ¦µç¶¯ÊÆºÍÖµÐ¡ÓÚÒ»¶¨Öµ¡£
//¶ÔÓÚ×óÍäµÀ£¬×ÝÏòµç¸ÐV1ºÍ V2µÄ¸ÐÓ¦µç¶¯ÊÆÖ®²î´óÓÚÒ»¶¨Öµ£»
//¶ÔÓÚÓÒÍäµÀ£¬×ÝÏòµç¸ÐV2ºÍ V1µÄ¸ÐÓ¦µç¶¯ÊÆÖ®²î´óÓÚÒ»¶¨Öµ£
//=================================================================//
//(3)Ê®×Ö½»²æµã£º×ÝÏòµç¸ÐV1ºÍ V2µÄ¸ÐÓ¦µç¶¯ÊÆ¾ù·Ö±ð´óÓÚÒ»¶¨Öµ
//=================================================================//
//(4)ÆÂµÀ¿É»®·ÖÎªÉÏÆÂ¡¢ÆÂ¶¥¡¢ÏÂÆÂ¡¢ÆÂºóËÄ¸öÇøÓò£¬
//¸÷ÇøÓò¶¼ÓÐºÜÃ÷ÏÔµÄÌØÕ÷£¬Éèµç¸ÐL1¡¢L2ºÍL3µÄ¸ÐÓ¦µç¶¯ÊÆºÍÖµÎªEsum
//ÉÏÆÂ£ºEsumºÜ´ó£»
//ÆÂ¶¥£ºEsumºÜÐ¡£»
//ÏÂÆÂ£ºEsumºÜ´ó£»
//ÆÂºó£ºEsum¼õÐ¡£¬»Ö¸´Õý³£Öµ¡
////0:Ö±µÀ  4:´ó»¡Íä 5:  1:×ó×ªÍä  2:ÓÒ×ªÍä 3:Ê®×Ö½»²æµã 
//=================================================================//
void sai_dao_position()
{ 
    
     //uint16_t two_cut_one;
     
//======== Ö±µÀ¼ì²â ===========//
   if(Road_Type[0] !=0)
   {

	    if(Leveltaldiangan_sum[0]>320 && qu_zheng(chui_zhi_daingan[0] )<30)
	    {
	         
	         if(++Road_Type_Times[0]>20)
	         {     
	              
 	             Road_Type[0] = 0;//Ö±µÀ
 	             special_Road_Type[0] = 0; 	             
	             Road_Type_Times[0]=0;
	             Road_Type_Times[1]=0;
	             Road_Type_Times[2]=0;
	             Road_Type_Times[3]=0;
	            // Road_Type_Times[4]=0; 
 	             
 	         }
	    }
	    else
	    {
	         Road_Type_Times[0]=0;
	         
	    }
    }  
     
   else  
   {
     if(Road_Type_Times[0] != 0 )
        Road_Type_Times[0]=0;
  
   }
   
    
//====================Ê®×Ö½»²æµã¼ì²â=============================//
  if( array_current[4]>100&&array_current[5]>100 )
  {
    if( Road_Type[0] !=3 )
    {
      if(++Road_Type_Times[3]>=2)
      {
        Road_Type[0] = 3;
        Road_Type_Times[3]=0;  
       }    
    }
    
     Road_Type_Times[0] = 0;
     Road_Type_Times[1] = 0;
     Road_Type_Times[2] = 0;
     
     
   }
  	
  else
  {
//=======================×ó×ªÍä¼ì²â=============================//
    Road_Type_Times[3]=0;
    if(Leveltaldiangan_sum[0] < 350)
    {
      if(Road_Type[0] != 1 && chui_zhi_daingan[0] >30 )
      {
         Road_Type_Times[0] = 0;  
         Road_Type_Times[2] = 0;
         Road_Type_Times[4] = 0;
         if(++Road_Type_Times[1] >= 5)
         {
           
          Road_Type[0] = 1;         
          Road_Type_Times[1]=0;
  
         }
      }
//======================ÓÒ×ªÍä¼ì²â==============================//
      else if(Road_Type[0] != 2 && chui_zhi_daingan[0] <-30 )
      {
         Road_Type_Times[0] = 0;
         Road_Type_Times[1] = 0;
         Road_Type_Times[4] = 0;
        if(++Road_Type_Times[2] >= 5)
        {    
           Road_Type[0] = 2;
           
           Road_Type_Times[2] = 0;
        }
      
      }
    
    }
  }
 
 
 
 if(Road_Type[0]==1&&Road_Type[1]==2)
 {
    if(diangan_max[0]<60) Road_Type[0]=2; 	 
 }
 else if(Road_Type[0]==2&&Road_Type[1]==1)
 {
    if(diangan_max[0]<60) Road_Type[0]=1; 	 
 
 	
 }
 
//============================ÈüµÀ¼ì²âÂË²¨==========================// 
 //protect_
 
 if(protect_Road_Type[0] !=0)
   {

	    if(Leveltaldiangan_sum[0]>320 && qu_zheng(chui_zhi_daingan[0] )<30)
	    {
	         
	         if(++protect_Road_Type_Times[0]>50)
	         {     
	              
 	             protect_Road_Type[0] = 0;//Ö±µÀ
 	                           
	             protect_Road_Type_Times[0]=0;
	             protect_Road_Type_Times[1]=0;
	             protect_Road_Type_Times[2]=0;
	             protect_Road_Type_Times[3]=0;
	            // Road_Type_Times[4]=0; 
 	             
 	         }
	    }
	    else
	    {
	         protect_Road_Type_Times[0]=0;
	         
	    }
    }  
     
   else  
   {
     if(protect_Road_Type_Times[0] != 0 )
        protect_Road_Type_Times[0]=0;
  
   }
   
    
//====================Ê®×Ö½»²æµã¼ì²â=============================//
  if(array_current[4]>100&&array_current[5]>100)
  {
    if( protect_Road_Type[0] !=3 )
    {
      if(++protect_Road_Type_Times[3]>=2)
      {
        protect_Road_Type[0] = 3;
        protect_Road_Type_Times[3]=0;  
       }    
    }
    
     protect_Road_Type_Times[0] = 0;
     protect_Road_Type_Times[1] = 0;
     protect_Road_Type_Times[2] = 0;
     
     
   }
  	
  else
  {
//=======================×ó×ªÍä¼ì²â=============================//
    protect_Road_Type_Times[3]=0;
    if(Leveltaldiangan_sum[0] < 330)
    {
      if(protect_Road_Type[0] != 1 && chui_zhi_daingan[0] >60 )
      {
         protect_Road_Type_Times[0] = 0;  
         protect_Road_Type_Times[2] = 0;
         protect_Road_Type_Times[4] = 0;
         if(++protect_Road_Type_Times[1] >= 2)
         {
           
          protect_Road_Type[0] = 1;         
          protect_Road_Type_Times[1]=0;
  
         }
      }
//======================ÓÒ×ªÍä¼ì²â==============================//
      else if(protect_Road_Type[0] != 2 && chui_zhi_daingan[0] <-60 )
      {
         protect_Road_Type_Times[0] = 0;
         protect_Road_Type_Times[1] = 0;
         protect_Road_Type_Times[4] = 0;
        if(++protect_Road_Type_Times[2] >= 2)
        {    
           protect_Road_Type[0] = 2;
           
           protect_Road_Type_Times[2] = 0;
        }
      
      }
    
    }
  }
 
  
 }
//==================================================================//
///////////////////////////////ÍäµÀ¼ì²âº¯Êý///////////////////////////
//¹¦ÄÜÃèÊö;Ñ°ÕÒÆ«ÒÆÁ¿µÄ±ä»¯Ç÷ÊÆ£¬È·¶¨³öÍä»¹ÊÇ½øÍä
void wandao_check()
{
     
    uint8_t i;
    uint8_t static ruwan_times;
    uint8_t static chuwan_times;

   if(Road_Type[0]==1)//×óÍäµÀÎ»ÖÃÅÐ¶Ï
   {
	    
	   chuwan_times=0;
	   ruwan_times=0;
	   for(i=0;i<50;i++)
	   {
	   	     if((servo_Error[i]-servo_Error[i+15])>0) 
	           ruwan_times++ ;//ÈëÍä´ÎÊýÅÐ¶Ï
	           
	    	 if((servo_Error[i]-servo_Error[i+15])<0) 
	    	   chuwan_times++ ;//³öÍä´ÎÊýÅÐ¶Ï

	   }
	    
	    
	   if(ruwan_times>15) wandao_come_in=1;//wandao_come_inÖÃ1ÊÇ×óÈëÍä
	   else wandao_come_in=0;
	   
	   if(chuwan_times>15) wandao_go_out=1;//wandao_go_outÖÃ1ÊÇ×ó³öÍä
	   else wandao_go_out=0;
    }
    
   else if(Road_Type[0]==2)//ÓÒÍäµÀÎ»ÖÃÅÐ¶Ï
   {
	    
	   chuwan_times=0;
	   ruwan_times=0;
	   for(i=0;i<50;i++)
	   {
	   	     if((servo_Error[i]-servo_Error[i+15])<0) 
	           ruwan_times++ ;//ÈëÍä´ÎÊýÅÐ¶Ï
	           
	    	 if((servo_Error[i]-servo_Error[i+15])>0) 
	    	   chuwan_times++ ;//³öÍä´ÎÊýÅÐ¶Ï

	   }
	    
	    
	   if(ruwan_times> 15) wandao_come_in=2;//wandao_come_inÖÃ2ÊÇÓÒÈëÍä
	   else wandao_come_in=0;
	   
	   if(chuwan_times> 15) wandao_go_out=2;//wandao_go_outÖÃ2ÊÇÓÒ³öÍä
	   else wandao_go_out=0;
    }
    
    else
    {
       	wandao_come_in=0;//Èç¹ûÌõ¼þ¶¼²»Âú×ã£¬±êÖ¾Î»ÇåÁã
       	wandao_go_out=0;//
    }
}
 
 
 
 
 
//======================void servo_Fuzzy_Kp_chu_shi_hua( void ) ======================================//
//                                  kp_e (kp_e_lishudu )      ( kp_e+1) ( Error_lishudu_Max - kp_e_lishudu )              
//                                --------------------------------------------------------------------------------
//                                  kp_ec (kp_ec_lishudu ) 
//                               (kp_ec+1) (Error_lishudu_Max - kp_ec_lishudu ) 
//                                   ÊäÈëÆ«²î  Æ«²î±ä»¯ÂÊ £¬µÃ³ö Kp µÄÖµ
//=================================================================================//
uint16_t servo_Fuzzy_Kp_chu_shi_hua( void )
{
        uint8_t    kp_e = 0 , kp_ec = 0 ;  //  Kp ¹æÔò±íÖÐµÚ¼¸ÇøÓò
                    // È¡³öÐ¡ÇøÓòµÄºÅ E£¬¼°ÆäÁ¥Êô¶È L£»Ôò´óÇøÓòµÄºÅ E+1 £¬Error_lishudu_Max - L
        uint16_t  kp_e_lishudu , kp_ec_lishudu  ; //  Á¥Êô¶È
        uint8_t  temp_11[4] = { 0,0,0,0 } , temp_12[4] = { 0,0,0,0 } , temp_21[4] = { 0,0,0,0 } , temp_22[4] = { 0,0,0,0 } ;
       
        Fuzzy_Kp_cunchu[5] = Fuzzy_Kp_cunchu[0] ;  // ´æ´¢ÉÏÒ»´Î KpÓÃ¹ýµÄÖµ £¬ ÓÃÓÚÓë±¾´Î±È½Ï £¬ Ä¿µÄÊÇÏÞ·ùÂË²¨
  
               ////**-----Kp  ----E----- ·ÖÅäÇø¼ä----- ****/////
              ///***    Çó³ö      kp_e      kp_e_lishudu   ***///
        if( servo_Error[0] >= servo_Fuzzy_Error[0] && servo_Error[0] < servo_Fuzzy_Error[6] )
        {       ////////////////////////////////////////////////////////////////
        	if( servo_Error[0] < ( servo_Fuzzy_Error[1] + servo_Fuzzy_Error[0] )/2 )    //-70 ------- -40       
        	{    kp_e=0 ;  //1
        	     kp_e_lishudu =Error_lishudu_Max - Error_lishudu_Max * ( servo_Error[0] - servo_Fuzzy_Error[0] ) / ( servo_Fuzzy_Error[1] - servo_Fuzzy_Error[0] ) ;
        	}
              else if( servo_Error[0] < ( servo_Fuzzy_Error[1]  ) )    
              {     kp_e=0 ; 
                    kp_e_lishudu = Error_lishudu_Max * ( servo_Fuzzy_Error[1] -  servo_Error[0]) / ( servo_Fuzzy_Error[1] - servo_Fuzzy_Error[0] ) ;
              }
             /////////-------------------------------------------------------/////////
              else if( servo_Error[0] < ( servo_Fuzzy_Error[2] + servo_Fuzzy_Error[1] )/2 )    // -40 ---- -10
              {     kp_e=1 ; // 2
                    kp_e_lishudu = Error_lishudu_Max - Error_lishudu_Max * ( servo_Error[0] - servo_Fuzzy_Error[1] ) / ( servo_Fuzzy_Error[2] - servo_Fuzzy_Error[1] ) ;
              } 
             else if( servo_Error[0] < servo_Fuzzy_Error[2] )    
              {     kp_e=1 ; 
                    kp_e_lishudu = Error_lishudu_Max * ( servo_Fuzzy_Error[2] - servo_Error[0] ) / ( servo_Fuzzy_Error[2] - servo_Fuzzy_Error[1] ) ;
              }
             ///////--------------------------------------------------//////
              else if( servo_Error[0] < ( servo_Fuzzy_Error[3] + servo_Fuzzy_Error[2] )/2 )    //-10 ------ 0
              {     kp_e=2 ; // 3
                    kp_e_lishudu =Error_lishudu_Max - Error_lishudu_Max * ( servo_Error[0] - servo_Fuzzy_Error[2] ) / ( servo_Fuzzy_Error[3] - servo_Fuzzy_Error[2] ) ;
              }
             else if( servo_Error[0] < servo_Fuzzy_Error[3] )    
              {     kp_e=2 ;
                    kp_e_lishudu = Error_lishudu_Max * ( servo_Fuzzy_Error[3] - servo_Error[0] ) / ( servo_Fuzzy_Error[3] - servo_Fuzzy_Error[2] ) ;
              }
             //////--------------------------------------------------//// 
              else if( servo_Error[0] < ( servo_Fuzzy_Error[4] + servo_Fuzzy_Error[3] )/2 )    // 0 --------10
              {     kp_e=3 ;  // 4
                    kp_e_lishudu =Error_lishudu_Max - Error_lishudu_Max * ( servo_Error[0] - servo_Fuzzy_Error[3] ) / ( servo_Fuzzy_Error[4] - servo_Fuzzy_Error[3] ) ;
              } 
             else if( servo_Error[0] < servo_Fuzzy_Error[4] )    
              {     kp_e=3 ; 
                    kp_e_lishudu = Error_lishudu_Max * ( servo_Fuzzy_Error[4] - servo_Error[0] ) / ( servo_Fuzzy_Error[4] - servo_Fuzzy_Error[3] ) ;
              }
             //////--------------------------------------------------//////
              else if( servo_Error[0] < ( servo_Fuzzy_Error[5] + servo_Fuzzy_Error[4] )/2 )    // 10 -------- 40
              {     kp_e=4 ; //5
                    kp_e_lishudu =Error_lishudu_Max - Error_lishudu_Max * ( servo_Error[0] - servo_Fuzzy_Error[4] ) / ( servo_Fuzzy_Error[5] - servo_Fuzzy_Error[4] ) ;
              } 
             else if( servo_Error[0] < servo_Fuzzy_Error[5] )    
              {     kp_e=4 ; 
                    kp_e_lishudu = Error_lishudu_Max * ( servo_Fuzzy_Error[5] - servo_Error[0] ) / ( servo_Fuzzy_Error[5] - servo_Fuzzy_Error[4] ) ;
              }
             ///////-------------------------------------------------------////////
             else if( servo_Error[0] < ( servo_Fuzzy_Error[6] + servo_Fuzzy_Error[5] )/2 )    // 40 -------- 70
              {     kp_e=5 ; //6
                    kp_e_lishudu =Error_lishudu_Max - Error_lishudu_Max * ( servo_Error[0] - servo_Fuzzy_Error[5] ) / ( servo_Fuzzy_Error[6] - servo_Fuzzy_Error[5] ) ;
             } 
             else if( servo_Error[0] < servo_Fuzzy_Error[6] )
              {     kp_e=5 ;
                    kp_e_lishudu = Error_lishudu_Max * ( servo_Fuzzy_Error[6] - servo_Error[0] ) / ( servo_Fuzzy_Error[6] - servo_Fuzzy_Error[5] ) ;
             }
        }
        ///////--------------------------------------------------------//////////////
        else if (  servo_Error[0] < servo_Fuzzy_Error[0] )  // ----- < -70
        {      kp_e=0 ; 
               kp_e_lishudu = Error_lishudu_Max ; 
        }
        else                                                                          //  ------ > 70
        {     kp_e=5 ;
              kp_e_lishudu = 0 ; 
       }

                       ////***----Kp  -----EC----- ·ÖÅäÇø¼ä---****/////
                      ///****    Çó³ö      kp_ec      kp_ec_lishudu    ****///
        if( servo_Error_c[0] >= servo_Fuzzy_Error_c[0] && servo_Error_c[0] < servo_Fuzzy_Error_c[6] )
        {       ////////////////////////////////////////////////////////////////
        	if( servo_Error_c[0] < ( servo_Fuzzy_Error_c[1] + servo_Fuzzy_Error_c[0] )/2 )    //-70 ------- -40       
        	{    kp_ec=0 ; //1
        	     kp_ec_lishudu =Error_c_lishudu_Max - Error_c_lishudu_Max * ( servo_Error_c[0] - servo_Fuzzy_Error_c[0] ) / ( servo_Fuzzy_Error_c[1] - servo_Fuzzy_Error_c[0] ) ;
        	}
              else if( servo_Error_c[0] < ( servo_Fuzzy_Error_c[1]  ) )    
              {     kp_ec=0 ; 
                    kp_ec_lishudu = Error_c_lishudu_Max * ( servo_Fuzzy_Error_c[1] -  servo_Error_c[0]) / ( servo_Fuzzy_Error_c[1] - servo_Fuzzy_Error_c[0] ) ;
              }
             /////////-------------------------------------------------------/////////
              else if( servo_Error_c[0] < ( servo_Fuzzy_Error_c[2] + servo_Fuzzy_Error_c[1] )/2 )    // -40 ---- -10
              {     kp_ec=1 ; //2
                    kp_ec_lishudu =Error_c_lishudu_Max - Error_c_lishudu_Max * ( servo_Error_c[0] - servo_Fuzzy_Error_c[1] ) / ( servo_Fuzzy_Error_c[2] - servo_Fuzzy_Error_c[1] ) ;
              } 
             else if( servo_Error_c[0] < servo_Fuzzy_Error_c[2] )    
              {     kp_ec=1 ; 
                    kp_ec_lishudu = Error_c_lishudu_Max * ( servo_Fuzzy_Error_c[2] - servo_Error_c[0] ) / ( servo_Fuzzy_Error_c[2] - servo_Fuzzy_Error_c[1] ) ;
              }
             ///////--------------------------------------------------//////
              else if( servo_Error_c[0] < ( servo_Fuzzy_Error_c[3] + servo_Fuzzy_Error_c[2] )/2 )    //-10 ------ 0
              {     kp_ec=2 ; // 3
                    kp_ec_lishudu =Error_c_lishudu_Max - Error_c_lishudu_Max * ( servo_Error_c[0] - servo_Fuzzy_Error_c[2] ) / ( servo_Fuzzy_Error_c[3] - servo_Fuzzy_Error_c[2] ) ;
            } 
             else if( servo_Error_c[0] < servo_Fuzzy_Error_c[3] )    
              {     kp_ec=2 ;
                    kp_ec_lishudu = Error_c_lishudu_Max * ( servo_Fuzzy_Error_c[3] - servo_Error_c[0] ) / ( servo_Fuzzy_Error_c[3] - servo_Fuzzy_Error_c[2] ) ;
             }
             //////--------------------------------------------------//// 
              else if( servo_Error_c[0] < ( servo_Fuzzy_Error_c[4] + servo_Fuzzy_Error_c[3] )/2 )    // 0 --------10
              {     kp_ec=3 ; //4
                    kp_ec_lishudu =Error_c_lishudu_Max - Error_c_lishudu_Max * ( servo_Error_c[0] - servo_Fuzzy_Error_c[3] ) / ( servo_Fuzzy_Error_c[4] - servo_Fuzzy_Error_c[3] ) ;
             } 
             else if( servo_Error_c[0] < servo_Fuzzy_Error_c[4] )    
              {     kp_ec=3 ; 
                    kp_ec_lishudu = Error_c_lishudu_Max * ( servo_Fuzzy_Error_c[4] - servo_Error_c[0] ) / ( servo_Fuzzy_Error_c[4] - servo_Fuzzy_Error_c[3] ) ;
              }
             //////--------------------------------------------------//////
              else if( servo_Error_c[0] < ( servo_Fuzzy_Error_c[5] + servo_Fuzzy_Error_c[4] )/2 )    // 10 -------- 40
              {     kp_ec=4 ; //5
                    kp_ec_lishudu =Error_c_lishudu_Max - Error_c_lishudu_Max * ( servo_Error_c[0] - servo_Fuzzy_Error_c[4] ) / ( servo_Fuzzy_Error_c[5] - servo_Fuzzy_Error_c[4] ) ;
              } 
             else if( servo_Error_c[0] < servo_Fuzzy_Error_c[5] )    
              {     kp_ec=4 ; 
                    kp_ec_lishudu = Error_c_lishudu_Max * ( servo_Fuzzy_Error_c[5] - servo_Error_c[0] ) / ( servo_Fuzzy_Error_c[5] - servo_Fuzzy_Error_c[4] ) ;
              }
             ///////-------------------------------------------------------////////
             else if( servo_Error_c[0] < ( servo_Fuzzy_Error_c[6] + servo_Fuzzy_Error_c[5] )/2 )    // 40 -------- 70
              {     kp_ec=5 ; //6
                    kp_ec_lishudu =Error_c_lishudu_Max - Error_c_lishudu_Max * ( servo_Error_c[0] - servo_Fuzzy_Error_c[5] ) / ( servo_Fuzzy_Error_c[6] - servo_Fuzzy_Error_c[5] ) ;
             } 
             else if( servo_Error_c[0] < servo_Fuzzy_Error_c[6] )   
              {     kp_ec=5 ;
                    kp_ec_lishudu = Error_c_lishudu_Max * ( servo_Fuzzy_Error_c[6] - servo_Error_c[0] ) / ( servo_Fuzzy_Error_c[6] - servo_Fuzzy_Error_c[5] ) ;
             }
        }
        ///////--------------------------------------------------------//////////////
        else if (  servo_Error_c[0] < servo_Fuzzy_Error_c[0] )  // ----- < -70
        {      kp_ec=0 ; 
               kp_ec_lishudu = Error_c_lishudu_Max ; 
        }
        else                                                                          //  ------ > 70
        {     kp_ec=5 ; //6
              kp_ec_lishudu = 0 ; 
       }
   //    kp_ec = 3 ;
       /////      ÕÒµ½¹æÔò±íÖÐµÚ¼¸ÇøÓò       ////
       temp_11[0] = servo_Fuzzy_Kp_rule[ kp_ec ][ kp_e ] ;
       temp_12[0] = servo_Fuzzy_Kp_rule[ kp_ec][ kp_e +1 ] ;
       temp_21[0] = servo_Fuzzy_Kp_rule[ kp_ec+1 ][ kp_e ] ;
       temp_22[0] = servo_Fuzzy_Kp_rule[ kp_ec+1 ][ kp_e+1 ] ;
       
       ///          ÕÒµ½°´ servo_Error ÕÒµ½µÄÁ¥Êô¶È   /////
       temp_11[1] = kp_e_lishudu ;
       temp_12[1] = Error_lishudu_Max - kp_e_lishudu ;
       temp_21[1] = kp_e_lishudu ;
       temp_22[1] = Error_lishudu_Max - kp_e_lishudu ;
       
        ///          ÕÒµ½°´ servo_Error_c ÕÒµ½µÄÁ¥Êô¶È   /////
       temp_11[2] = kp_ec_lishudu ;
       temp_21[2] = Error_c_lishudu_Max - kp_ec_lishudu ;
       temp_12[2] = kp_ec_lishudu ;
       temp_22[2] = Error_c_lishudu_Max - kp_ec_lishudu ;
       
       ///                  È·¶¨ËÄ¸öµãµÄÁ¥Êô¶È            /////
       if( temp_11[1] > temp_11[2] )  temp_11[1] = temp_11[2] ;
       if( temp_12[1] > temp_12[2] )  temp_12[1] = temp_12[2] ;
       if( temp_21[1] > temp_21[2] )  temp_21[1] = temp_21[2] ;
       if( temp_22[1] > temp_22[2] )  temp_22[1] = temp_22[2] ;
       
       //===================================//
       // Ð¡ÖÐÈ¡´ó  ¹æÔò±íÖÐÍ¬µÈÇøÓò¡ª¡ªÁ¥Êô¶ÈÈ¡´ó //
       if( temp_11[0]==temp_12[0] ) // 11--12
       {
       	     if( temp_11[1] > temp_12[1] )    temp_12[1] = 0 ;
       	     else     temp_11[1] = 0 ;                                   
       }
       if( temp_11[0]==temp_21[0] ) //11--21
       {
       	     if( temp_11[1] > temp_21[1] )    temp_21[1] = 0 ;
       	     else     temp_11[1] = 0 ;                                   
       }
       if( temp_11[0]==temp_22[0] ) //11==22
       {
       	     if( temp_11[1] > temp_22[1] )    temp_22[1] = 0 ;
       	     else     temp_11[1] = 0 ;                                   
       }
       if( temp_12[0]==temp_21[0] )  //12--21
       {
       	     if( temp_12[1] > temp_21[1] )    temp_21[1] = 0 ;
       	     else     temp_12[1] = 0 ;                                   
       }
       if( temp_12[0]==temp_22[0] ) //12--22
       {
       	     if( temp_12[1] > temp_22[1] )    temp_22[1] = 0 ;
       	     else     temp_12[1] = 0 ;                                   
       }
       if( temp_21[0]==temp_22[0] ) // 21 --22
       {
       	     if( temp_21[1] > temp_22[1] )    temp_22[1] = 0 ;
       	     else     temp_21[1] = 0 ;                                   
       }
       //====================================//
    
      //====================================//
      speed_temp_11[0] = temp_11[0] ;  speed_temp_11[1] = temp_11[1] ; 
      speed_temp_12[0] = temp_12[0] ;  speed_temp_12[1] = temp_12[1] ; 
      speed_temp_21[0] = temp_21[0] ;  speed_temp_21[1] = temp_21[1] ; 
      speed_temp_22[0] = temp_22[0] ;  speed_temp_22[1] = temp_22[1] ; 
     //====================================//
      Fuzzy_Kp_cunchu[1] =  servo_Fuzzy_Kp_dan[ temp_11[0] ] * temp_11[1]   // ·Ö×Ó
                                       + servo_Fuzzy_Kp_dan[ temp_12[0] ] * temp_12[1]
                                       + servo_Fuzzy_Kp_dan[ temp_21[0] ] * temp_21[1]
                                       + servo_Fuzzy_Kp_dan[ temp_22[0] ] * temp_22[1] ;
       Fuzzy_Kp_cunchu[2] = temp_11[1] + temp_12[1] + temp_21[1] + temp_22[1] ;  // ·ÖÄ¸             
       if( Fuzzy_Kp_cunchu[2] < 2 )  Fuzzy_Kp_cunchu[2]=1 ;
       
       Fuzzy_Kp_cunchu[0] = Fuzzy_Kp_cunchu[1] / Fuzzy_Kp_cunchu[2] ;
       
       
       
      

       //========================= ÉÏÏÞÂË²¨   ===================================//
       if( ( Fuzzy_Kp_cunchu[0] - Fuzzy_Kp_cunchu[5] ) > 27  )      // ¶Ô Kp ½øÐÐ ¡ª¡ª ÏÞ·ù ¡ª¡ª ÂË²¨
             Fuzzy_Kp_cunchu[0] = Fuzzy_Kp_cunchu[5] + 19 ;         // [0] ±¾´ÎµÄ Kp  ,
       else if( (Fuzzy_Kp_cunchu[0] - Fuzzy_Kp_cunchu[5])<-27 )  // [5] ÉÏ´ÎµÄKp
             Fuzzy_Kp_cunchu[0] = Fuzzy_Kp_cunchu[5] - 19 ;
     //========================  ÏÂÏÞÂË²¨    ==================================//
     if( ( Fuzzy_Kp_cunchu[0] - Fuzzy_Kp_cunchu[5] ) < 3 )
     {
     	   if( ( Fuzzy_Kp_cunchu[0] - Fuzzy_Kp_cunchu[5] ) > -3 )
     	        Fuzzy_Kp_cunchu[0] = Fuzzy_Kp_cunchu[5] ;
     }

       Fuzzy_Kp_cunchu[0] = ( 87*Fuzzy_Kp_cunchu[0] + 13*Fuzzy_Kp_cunchu[5]) / 100 ; // ÇóÆ½¾ùÖµ¡ª¡ª ÂË²¨
        
         
                return Fuzzy_Kp_cunchu[0]  ;    
}

//=========================void servo_Fuzzy_Kd_chu_shi_hua( void ) =================================//
//                                  kp_e (kp_e_lishudu )      ( kp_e+1) ( Error_lishudu_Max - kp_e_lishudu )              
//                                --------------------------------------------------------------------------------
//                                  kp_ec (kp_ec_lishudu ) 
//                               (kp_ec+1) (Error_lishudu_Max - kp_ec_lishudu ) 
//                               ÊäÈëÆ«²î  Æ«²î±ä»¯ÂÊ £¬µÃ³ö Kd µÄÖµ
//=======================================================================================//
uint16_t servo_Fuzzy_Kd_chu_shi_hua( void )
{
        uint8_t    kp_e = 0 , kp_ec = 0 ;  //  Kp ¹æÔò±íÖÐµÚ¼¸ÇøÓò
                    // È¡³öÐ¡ÇøÓòµÄºÅ E£¬¼°ÆäÁ¥Êô¶È L£»Ôò´óÇøÓòµÄºÅ E+1 £¬Error_lishudu_Max - L
        uint16_t  kp_e_lishudu=0 , kp_ec_lishudu =0  ; //  Á¥Êô¶È
        uint8_t  temp_11[4] = { 0,0,0,0 } , temp_12[4] = { 0,0,0,0 } , temp_21[4] = { 0,0,0,0 } , temp_22[4] = { 0,0,0,0 } ;
       
        Fuzzy_Kd_cunchu[5] = Fuzzy_Kd_cunchu[0] ;  // ´æ´¢ÉÏÒ»´Î KdÓÃ¹ýµÄÖµ £¬ ÓÃÓÚÓë±¾´Î±È½Ï £¬ Ä¿µÄÊÇÏÞ·ùÂË²¨
       
        ////***********    -----------Kp  ------E------- ·ÖÅäÇø¼ä---------   **************/////
        ///******************         Çó³ö      kp_e      kp_e_lishudu               **************///
        if( servo_Error[0] >= servo_Fuzzy_Error[0] && servo_Error[0] < servo_Fuzzy_Error[6] )
        {       ////////////////////////////////////////////////////////////////
        	if( servo_Error[0] < ( servo_Fuzzy_Error[1] + servo_Fuzzy_Error[0] )/2 )    //-70 ------- -40       
        	{    kp_e=0 ;  //1
        	     kp_e_lishudu =Error_lishudu_Max - Error_lishudu_Max * ( servo_Error[0] - servo_Fuzzy_Error[0] ) / ( servo_Fuzzy_Error[1] - servo_Fuzzy_Error[0] ) ;
        	}
              else if( servo_Error[0] < ( servo_Fuzzy_Error[1]  ) )    
              {     kp_e=0 ; 
                    kp_e_lishudu = Error_lishudu_Max * ( servo_Fuzzy_Error[1] -  servo_Error[0]) / ( servo_Fuzzy_Error[1] - servo_Fuzzy_Error[0] ) ;
              }
             /////////-------------------------------------------------------/////////
              else if( servo_Error[0] < ( servo_Fuzzy_Error[2] + servo_Fuzzy_Error[1] )/2 )    // -40 ---- -10
              {     kp_e=1 ; // 2
                    kp_e_lishudu = Error_lishudu_Max - Error_lishudu_Max * ( servo_Error[0] - servo_Fuzzy_Error[1] ) / ( servo_Fuzzy_Error[2] - servo_Fuzzy_Error[1] ) ;
              } 
             else if( servo_Error[0] < servo_Fuzzy_Error[2] )    
              {     kp_e=1 ; 
                    kp_e_lishudu = Error_lishudu_Max * ( servo_Fuzzy_Error[2] - servo_Error[0] ) / ( servo_Fuzzy_Error[2] - servo_Fuzzy_Error[1] ) ;
              }
             ///////--------------------------------------------------//////
              else if( servo_Error[0] < ( servo_Fuzzy_Error[3] + servo_Fuzzy_Error[2] )/2 )    //-10 ------ 0
              {     kp_e=2 ; // 3
                    kp_e_lishudu =Error_lishudu_Max - Error_lishudu_Max * ( servo_Error[0] - servo_Fuzzy_Error[2] ) / ( servo_Fuzzy_Error[3] - servo_Fuzzy_Error[2] ) ;
            } 
             else if( servo_Error[0] < servo_Fuzzy_Error[3] )    
              {     kp_e=2 ;
                    kp_e_lishudu = Error_lishudu_Max * ( servo_Fuzzy_Error[3] - servo_Error[0] ) / ( servo_Fuzzy_Error[3] - servo_Fuzzy_Error[2] ) ;
             }
             //////--------------------------------------------------//// 
              else if( servo_Error[0] < ( servo_Fuzzy_Error[4] + servo_Fuzzy_Error[3] )/2 )    // 0 --------10
              {     kp_e=3 ;  // 4
                    kp_e_lishudu =Error_lishudu_Max - Error_lishudu_Max * ( servo_Error[0] - servo_Fuzzy_Error[3] ) / ( servo_Fuzzy_Error[4] - servo_Fuzzy_Error[3] ) ;
             } 
             else if( servo_Error[0] < servo_Fuzzy_Error[4] )    
              {     kp_e=3 ; 
                    kp_e_lishudu = Error_lishudu_Max * ( servo_Fuzzy_Error[4] - servo_Error[0] ) / ( servo_Fuzzy_Error[4] - servo_Fuzzy_Error[3] ) ;
              }
             //////--------------------------------------------------//////
              else if( servo_Error[0] < ( servo_Fuzzy_Error[5] + servo_Fuzzy_Error[4] )/2 )    // 10 -------- 40
              {     kp_e=4 ; //5
                    kp_e_lishudu =Error_lishudu_Max - Error_lishudu_Max * ( servo_Error[0] - servo_Fuzzy_Error[4] ) / ( servo_Fuzzy_Error[5] - servo_Fuzzy_Error[4] ) ;
              } 
             else if( servo_Error[0] < servo_Fuzzy_Error[5] )    
              {     kp_e=4 ; 
                    kp_e_lishudu = Error_lishudu_Max * ( servo_Fuzzy_Error[5] - servo_Error[0] ) / ( servo_Fuzzy_Error[5] - servo_Fuzzy_Error[4] ) ;
              }
             ///////-------------------------------------------------------////////
             else if( servo_Error[0] < ( servo_Fuzzy_Error[6] + servo_Fuzzy_Error[5] )/2 )    // 40 -------- 70
              {     kp_e=5 ; //6
                    kp_e_lishudu =Error_lishudu_Max - Error_lishudu_Max * ( servo_Error[0] - servo_Fuzzy_Error[5] ) / ( servo_Fuzzy_Error[6] - servo_Fuzzy_Error[5] ) ;
             } 
             else if( servo_Error[0] < servo_Fuzzy_Error[6] )
              {     kp_e=5 ;
                    kp_e_lishudu = Error_lishudu_Max * ( servo_Fuzzy_Error[6] - servo_Error[0] ) / ( servo_Fuzzy_Error[6] - servo_Fuzzy_Error[5] ) ;
             }
        }
        ///////--------------------------------------------------------//////////////
        else if (  servo_Error[0] < servo_Fuzzy_Error[0] )  // ----- < -70
        {      kp_e=0 ; 
               kp_e_lishudu = Error_lishudu_Max ; 
        }
        else                                                                          //  ------ > 70
        {     kp_e=5 ;
              kp_e_lishudu = 0 ; 
       }

         ////***********    -----------Kp  ------EC------- ·ÖÅäÇø¼ä---------   **************/////
        ///******************         Çó³ö      kp_ec      kp_ec_lishudu               **************///
        //***********************************************************************************//
        if( servo_Error_c[0] >= servo_Fuzzy_Error_c[0] && servo_Error_c[0] < servo_Fuzzy_Error_c[6] )
        {       ////////////////////////////////////////////////////////////////
        	if( servo_Error_c[0] < ( servo_Fuzzy_Error_c[1] + servo_Fuzzy_Error_c[0] )/2 )    //-70 ------- -40       
        	{    kp_ec=0 ; //1
        	     kp_ec_lishudu =Error_c_lishudu_Max - Error_c_lishudu_Max * ( servo_Error_c[0] - servo_Fuzzy_Error_c[0] ) / ( servo_Fuzzy_Error_c[1] - servo_Fuzzy_Error_c[0] ) ;
        	}
              else if( servo_Error_c[0] < ( servo_Fuzzy_Error_c[1]  ) )    
              {     kp_ec=0 ; 
                    kp_ec_lishudu = Error_c_lishudu_Max * ( servo_Fuzzy_Error_c[1] -  servo_Error_c[0]) / ( servo_Fuzzy_Error_c[1] - servo_Fuzzy_Error_c[0] ) ;
              }
             /////////-------------------------------------------------------/////////
              else if( servo_Error_c[0] < ( servo_Fuzzy_Error_c[2] + servo_Fuzzy_Error_c[1] )/2 )    // -40 ---- -10
              {     kp_ec=1 ; //2
                    kp_ec_lishudu =Error_c_lishudu_Max - Error_c_lishudu_Max * ( servo_Error_c[0] - servo_Fuzzy_Error_c[1] ) / ( servo_Fuzzy_Error_c[2] - servo_Fuzzy_Error_c[1] ) ;
              } 
             else if( servo_Error_c[0] < servo_Fuzzy_Error_c[2] )    
              {     kp_ec=1 ; 
                    kp_ec_lishudu = Error_c_lishudu_Max * ( servo_Fuzzy_Error_c[2] - servo_Error_c[0] ) / ( servo_Fuzzy_Error_c[2] - servo_Fuzzy_Error_c[1] ) ;
              }
             ///////--------------------------------------------------//////
              else if( servo_Error_c[0] < ( servo_Fuzzy_Error_c[3] + servo_Fuzzy_Error_c[2] )/2 )    //-10 ------ 0
              {     kp_ec=2 ; // 3
                    kp_ec_lishudu =Error_c_lishudu_Max - Error_c_lishudu_Max * ( servo_Error_c[0] - servo_Fuzzy_Error_c[2] ) / ( servo_Fuzzy_Error_c[3] - servo_Fuzzy_Error_c[2] ) ;
            } 
             else if( servo_Error_c[0] < servo_Fuzzy_Error_c[3] )    
              {     kp_ec=2 ;
                    kp_ec_lishudu = Error_c_lishudu_Max * ( servo_Fuzzy_Error_c[3] - servo_Error_c[0] ) / ( servo_Fuzzy_Error_c[3] - servo_Fuzzy_Error_c[2] ) ;
             }
             //////--------------------------------------------------//// 
              else if( servo_Error_c[0] < ( servo_Fuzzy_Error_c[4] + servo_Fuzzy_Error_c[3] )/2 )    // 0 --------10
              {     kp_ec=3 ; //4
                    kp_ec_lishudu =Error_c_lishudu_Max - Error_c_lishudu_Max * ( servo_Error_c[0] - servo_Fuzzy_Error_c[3] ) / ( servo_Fuzzy_Error_c[4] - servo_Fuzzy_Error_c[3] ) ;
             } 
             else if( servo_Error_c[0] < servo_Fuzzy_Error_c[4] )    
              {     kp_ec=3 ; 
                    kp_ec_lishudu = Error_c_lishudu_Max * ( servo_Fuzzy_Error_c[4] - servo_Error_c[0] ) / ( servo_Fuzzy_Error_c[4] - servo_Fuzzy_Error_c[3] ) ;
              }
             //////--------------------------------------------------//////
              else if( servo_Error_c[0] < ( servo_Fuzzy_Error_c[5] + servo_Fuzzy_Error_c[4] )/2 )    // 10 -------- 40
              {     kp_ec=4 ; //5
                    kp_ec_lishudu =Error_c_lishudu_Max - Error_c_lishudu_Max * ( servo_Error_c[0] - servo_Fuzzy_Error_c[4] ) / ( servo_Fuzzy_Error_c[5] - servo_Fuzzy_Error_c[4] ) ;
              } 
             else if( servo_Error_c[0] < servo_Fuzzy_Error_c[5] )    
              {     kp_ec=4 ; 
                    kp_ec_lishudu = Error_c_lishudu_Max * ( servo_Fuzzy_Error_c[5] - servo_Error_c[0] ) / ( servo_Fuzzy_Error_c[5] - servo_Fuzzy_Error_c[4] ) ;
              }
             ///////-------------------------------------------------------////////
             else if( servo_Error_c[0] < ( servo_Fuzzy_Error_c[6] + servo_Fuzzy_Error_c[5] )/2 )    // 40 -------- 70
              {     kp_ec=5 ; //6
                    kp_ec_lishudu =Error_c_lishudu_Max - Error_c_lishudu_Max * ( servo_Error_c[0] - servo_Fuzzy_Error_c[5] ) / ( servo_Fuzzy_Error_c[6] - servo_Fuzzy_Error_c[5] ) ;
             } 
             else if( servo_Error_c[0] < servo_Fuzzy_Error_c[6] )   
              {     kp_ec=5 ;
                    kp_ec_lishudu = Error_c_lishudu_Max * ( servo_Fuzzy_Error_c[6] - servo_Error_c[0] ) / ( servo_Fuzzy_Error_c[6] - servo_Fuzzy_Error_c[5] ) ;
             }
        }
        ///////--------------------------------------------------------//////////////
        else if (  servo_Error_c[0] < servo_Fuzzy_Error_c[0] )  // ----- < -70
        {      kp_ec=0 ; 
               kp_ec_lishudu = Error_c_lishudu_Max ; 
        }
        else                                                                          //  ------ > 70
        {     kp_ec=5 ; //6
              kp_ec_lishudu = 0 ; 
       }
       /////      ÕÒµ½¹æÔò±íÖÐµÚ¼¸ÇøÓò       ////
       temp_11[0] = servo_Fuzzy_Kd_rule[ kp_ec ][ kp_e ] ;
       temp_12[0] = servo_Fuzzy_Kd_rule[ kp_ec][ kp_e +1 ] ;
       temp_21[0] = servo_Fuzzy_Kd_rule[ kp_ec+1 ][ kp_e ] ;
       temp_22[0] = servo_Fuzzy_Kd_rule[ kp_ec+1 ][ kp_e+1 ] ;
       
       temp_11[1] = kp_e_lishudu ;
       temp_12[1] = Error_lishudu_Max - kp_e_lishudu ;
       temp_21[1] = kp_e_lishudu ;
       temp_22[1] = Error_lishudu_Max - kp_e_lishudu ;

       temp_11[2] = kp_ec_lishudu ;
       temp_21[2] = Error_c_lishudu_Max - kp_ec_lishudu ;
       temp_12[2] = kp_ec_lishudu ;
       temp_22[2] = Error_c_lishudu_Max - kp_ec_lishudu ;
       
       ///                  È·¶¨ËÄ¸öµãµÄÁ¥Êô¶È            /////
       if( temp_11[1] > temp_11[2] )  temp_11[1] = temp_11[2] ;
       if( temp_12[1] > temp_12[2] )  temp_12[1] = temp_12[2] ;
       if( temp_21[1] > temp_21[2] )  temp_21[1] = temp_21[2] ;
       if( temp_22[1] > temp_22[2] )  temp_22[1] = temp_22[2] ;
       
       //===================================//
       // Ð¡ÖÐÈ¡´ó  ¹æÔò±íÖÐÍ¬µÈÇøÓò¡ª¡ªÁ¥Êô¶ÈÈ¡´ó //
       if( temp_11[0]==temp_12[0] ) // 11--12
       {
       	     if( temp_11[1] > temp_12[1] )    temp_12[1] = 0 ;
       	     else     temp_11[1] = 0 ;                                   
       }
       if( temp_11[0]==temp_21[0] ) //11--21
       {
       	     if( temp_11[1] > temp_21[1] )    temp_21[1] = 0 ;
       	     else     temp_11[1] = 0 ;                                   
       }
       if( temp_11[0]==temp_22[0] ) //11==22
       {
       	     if( temp_11[1] > temp_22[1] )    temp_22[1] = 0 ;
       	     else     temp_11[1] = 0 ;                                   
       }
       if( temp_12[0]==temp_21[0] )  //12--21
       {
       	     if( temp_12[1] > temp_21[1] )    temp_21[1] = 0 ;
       	     else     temp_12[1] = 0 ;                                   
       }
       if( temp_12[0]==temp_22[0] ) //12--22
       {
       	     if( temp_12[1] > temp_22[1] )    temp_22[1] = 0 ;
       	     else     temp_12[1] = 0 ;                                   
       }
       if( temp_21[0]==temp_22[0] ) // 21 --22
       {
       	     if( temp_21[1] > temp_22[1] )    temp_22[1] = 0 ;
       	     else     temp_21[1] = 0 ;                                   
       }
       //=================================//
   
       Fuzzy_Kd_cunchu[1]   =  servo_Fuzzy_Kd_dan[ temp_11[0] ] * temp_11[1]  //·Ö×Ó
                                          + servo_Fuzzy_Kd_dan[ temp_12[0] ] * temp_12[1]
                                          + servo_Fuzzy_Kd_dan[ temp_21[0] ] * temp_21[1]
                                          + servo_Fuzzy_Kd_dan[ temp_22[0] ] * temp_22[1] ;
       Fuzzy_Kd_cunchu[2] = temp_11[1] + temp_12[1] + temp_21[1] + temp_22[1] ;   // ·ÖÄ¸           
       if( Fuzzy_Kd_cunchu[2] <2 )    Fuzzy_Kd_cunchu[2] = 1;
           
       Fuzzy_Kd_cunchu[0] = Fuzzy_Kd_cunchu[1] / Fuzzy_Kd_cunchu[2] ;  //  ±¾´Î¼ÆËã³öµÄ Kd Öµ
    
       //==================  ÉÏÏÞÂË²¨ ============================//
         if( ( Fuzzy_Kd_cunchu[0] - Fuzzy_Kd_cunchu[5] ) > 27 )  //  ¶Ô¼ÆËã³öµÄ Kd Öµ½øÐÐ ¡ª¡ª ÏÞ·ù¡ª¡ªÂË²¨
              Fuzzy_Kd_cunchu[0] = Fuzzy_Kd_cunchu[5] + 19 ;  
        else if( ( Fuzzy_Kd_cunchu[0] - Fuzzy_Kd_cunchu[5] ) < -27 )
        {
       	        Fuzzy_Kd_cunchu[0] = Fuzzy_Kd_cunchu[5] - 19 ;
        	if( Fuzzy_Kd_cunchu[0] < 0)
        	    Fuzzy_Kd_cunchu[0] = 0;
        }
       //==================  ÏÂÏÞÂË²¨  ======================//
      if( ( Fuzzy_Kd_cunchu[0] - Fuzzy_Kd_cunchu[5] ) < 2 )  //  
       {
       	      if( ( Fuzzy_Kd_cunchu[0] - Fuzzy_Kd_cunchu[5] ) > -2 )
       	          Fuzzy_Kd_cunchu[0] = Fuzzy_Kd_cunchu[5] ;
       }

       Fuzzy_Kd_cunchu[0] = ( 87*Fuzzy_Kd_cunchu[0] + 13*Fuzzy_Kd_cunchu[5] ) / 100 ; // ÇóÆ½¾ù ¡ª¡ª ÂË²¨
                     
       
        //       SCI0_SendChar_16( Fuzzy_Kd_cunchu[0] ) ;
    
        return Fuzzy_Kd_cunchu[0]  ;
}


void Fuzzy_Speed_chushihua( void )
{
	//=============== Fuzzy_speed_control ===============//
        Fuzzy_speed_cunchu_zuo[1] =  speed_Fuzzy_zuo_dan[ speed_temp_11[0] ] * speed_temp_11[1]   // ·Ö×Ó
                                                     + speed_Fuzzy_zuo_dan[ speed_temp_12[0] ] * speed_temp_12[1]
                                                     + speed_Fuzzy_zuo_dan[ speed_temp_21[0] ] * speed_temp_21[1]
                                                     + speed_Fuzzy_zuo_dan[ speed_temp_22[0] ] * speed_temp_22[1] ;
       Fuzzy_speed_cunchu_zuo[2] = speed_temp_11[1] + speed_temp_12[1] + speed_temp_21[1] + speed_temp_22[1] ;  // ·ÖÄ¸             
     
       Fuzzy_speed_cunchu_zuo[0] = Fuzzy_speed_cunchu_zuo[1] / Fuzzy_speed_cunchu_zuo[2] ;
       
        Fuzzy_speed_cunchu_you[1] =  speed_Fuzzy_you_dan[ speed_temp_11[0] ] * speed_temp_11[1]   // ·Ö×Ó
                                                     + speed_Fuzzy_you_dan[ speed_temp_12[0] ] * speed_temp_12[1]
                                                     + speed_Fuzzy_you_dan[ speed_temp_21[0] ] * speed_temp_21[1]
                                                     + speed_Fuzzy_you_dan[ speed_temp_22[0] ] * speed_temp_22[1] ;
       Fuzzy_speed_cunchu_you[2] = speed_temp_11[1] + speed_temp_12[1] + speed_temp_21[1] + speed_temp_22[1] ;  // ·ÖÄ¸             
     
       Fuzzy_speed_cunchu_you[0] = Fuzzy_speed_cunchu_you[1] / Fuzzy_speed_cunchu_you[2] ;
       
     //  Fuzzy_speed_cunchu_you[0] = ( Fuzzy_speed_cunchu_you[0] + Fuzzy_speed_cunchu_you[5]) / 2 ; // ÇóÆ½¾ùÖµ¡ª¡ª ÂË²¨
        
        
        Fuzzy_speed_cunchu_zuo[5] = Fuzzy_speed_cunchu_zuo[0] ;
        Fuzzy_speed_cunchu_you[5] = Fuzzy_speed_cunchu_you[0] ;
        
        if( zhi_jiao_flag == 1)
        {
               if( servo_output < 0)
               {
               	CarSpeed_SET_L = 2100 ;	
                      CarSpeed_SET_R = 1800 ;
               }
        	else
        	{
        		CarSpeed_SET_L = 1800 ;	
                      CarSpeed_SET_R = 2100 ;
        	}
        }
        else
        {
        	CarSpeed_SET_L = Fuzzy_speed_cunchu_you[0] ;	
            CarSpeed_SET_R = Fuzzy_speed_cunchu_zuo[0] ;
        }
      
      //  SCI0_SendChar_16( CarSpeed_SET_L ) ;
     //   SCI0_SendChar_16( CarSpeed_SET_R ) ;
}
///================================================================================//
void PD_servo( void )
{	 
       uint16_t speed = 0 ;  
       speed= (smartcar_speed_L+smartcar_speed_R)/2;  	 
      //distance_calculate();
        
     
     //servo_Error[0]=80*fu_hao(servo_Error[0]);
     
     
	 servo_kp = servo_Fuzzy_Kp_chu_shi_hua( ) ;
	 servo_kd = servo_Fuzzy_Kd_chu_shi_hua( ) ;
		   
	   //SCI0_SendChar_16( servo_kp );
	   // SCI0_SendChar_16( servo_kd );
	  servo_output = servo_kp * servo_Error[0]  + servo_kd * servo_Error_c[0] ;
	  servo_output = servo_output/10 ;  
//      
                
                           
//==================   Æ«²î  ÏÞ·ù´¦Àí  ============== //   
/* */
    
          servo_output = ( 80*servo_output + 16*servo_output_last[0] + 3*servo_output_last[1] + 1*servo_output_last[2] ) / 100 ;
 
   //SCI0_SendChar_16( servo_output );
         
}



//========================    zhuanxiang  ¡ª¡ª  ×ªÏò      ======================================//
void zhuanxiang()
{

      uint8_t i;
      int16_t duoji_value;
      
              servo_max=0;
        
      for(i=0;i<12;i++)//Ñ°ÕÒÉÏÒ»´ÎµÄ×î´óÊä³öÁ¿
      {   
         duoji_value=qu_zheng(servo_output_last[i]);
         
         if(servo_max<duoji_value)
         {
         	servo_max=duoji_value;
         	jiao_biao_max=i;	
         }
       }
       
       servo_min=servo_max; 
       for(i=0;i<10;i++)
       {
         duoji_value=qu_zheng(servo_output_last[i]);
         if(servo_min>duoji_value)
         {
         	servo_min=duoji_value;
         	jiao_biao_min=i;//Ñ°ÕÒÉÏÒ»´ÎµÄ×îÐ¡Êä³öÁ¿	
         } 
       	
       }
     
   //============================================================//
		     switch(Road_Type[0])
		     {
		   	     case 0:
				        servo_output=8*servo_output/10;	
		 	     break;
		 //======================================================//  	
		   	     case 1:
				        if(wandao_go_out==1)
				        servo_output=7*servo_output/10;
				        else servo_output=10*servo_output/10;         	
		 	      break;
		 //=======================================================//   	
		   	      case 2:
		   	            if(wandao_go_out==2)
				        servo_output=7*servo_output/10;
				       else	servo_output=10*servo_output/10;
		 	      break;
		 //²ÎÊýµ÷ÊÔ 	
         //=======================================================//
           }
         	
		 
	      lost_signal();
 	     //noise_protect_second(); 
        
 
          zhuan_xiang_control() ;
           
   /*        
           
          if( (servo_output - servo_output_last[0]) > 50 )    //======= ²¨¶¯ÏÞ·ù´óÐ¡¡ª¡ª5£¬11 ¡ª¡ª +3 £¬+7=====//
	  {
	        servo_output_flag[3] = 0 ;
	        servo_output_flag[4] = 0 ;
	        servo_output_flag[5] = 0 ;
	        if( servo_output_flag[0] < 1 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 5 ÒÔÄÚ
	        {
	        	servo_output=servo_output_last[0]+37 ; // 1
	        	servo_output_flag[0]++;
	        }
	  	else  if( servo_output_flag[1] < 2 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		servo_output=servo_output_last[0]+54 ; // 1
	  		servo_output_flag[1]++;
	  	}
	       else  if( servo_output_flag[2] < 3 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		servo_output=servo_output_last[0]+67 ; // 1
	  		servo_output_flag[2]++;
	  	}
	  	else if( (servo_output-servo_output_last[5]) < 400 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 6 ´ÎÒÔÄÚ»¹Î´»Ø¹é
	  	       servo_output=servo_output_last[5] + 320 ;
	  	else  
	  	       servo_output=servo_output_last[0]+60 ; // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÔÚ 7 ´ÎÒÔÄÚ»¹Î´»Ø¹é
	  } 
 	  else if( (servo_output - servo_output_last[0]) < -50 )   //======= ²¨¶¯ÏÞ·ù´óÐ¡¡ª¡ª -5£¬-11 ¡ª¡ª -3 £¬-7=====//
 	  {
	        servo_output_flag[0] = 0 ;
	        servo_output_flag[1] = 0 ;
	        servo_output_flag[2] = 0 ;
	        if( servo_output_flag[3] < 1 )  // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 5 ÒÔÄÚ
	        {
	        	servo_output=servo_output_last[0]-37 ; // 1
	        	servo_output_flag[3]++;
	        }
	  	else  if( servo_output_flag[4] < 2 ) //// array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		servo_output=servo_output_last[0]-54 ; // 1
	  		servo_output_flag[4]++;
	  	}
	  	else  if( servo_output_flag[5] < 3 ) //// array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 11 ÒÔÄÚ
	  	{
	  		servo_output=servo_output_last[0]-67 ; // 1
	  		servo_output_flag[5]++;
	  	}
	  	else if( (servo_output-servo_output_last[5]) > -400 ) // array_lvbo_flag[0][0]  Ò»ºÅµç¸ÐµÄÆ«²îÊÇ·ñÔÚ 6 ´ÎÒÔÄÚ»¹Î´»Ø¹é
	  	       servo_output=servo_output_last[5] - 320 ;
	  	else  
	  	       servo_output=servo_output_last[0]-60 ; 
 	  }
 	  else
 	  {
 	     	 servo_output_flag[0] = 0 ;
	         servo_output_flag[1] = 0 ;
	         servo_output_flag[2] = 0 ;
	         servo_output_flag[3] = 0 ;
	         servo_output_flag[4] = 0 ;
	         servo_output_flag[5] = 0 ;
 	  }
 
*/
 
          
        // if(zhijiao_charge[1]==1) 
         
            //servo_output=600*fu_hao(servo_output);
          
          if(servo_output>700)  //ÏÞ·ù´¦Àí
          {   
             servo_output=700 ; 
          } 
          if(servo_output<-700)
          {  
             servo_output=-700 ;	
          }
          
          
        /*  if(total_diangan_sum[0]>560&&total_diangan_sum[0]<100)
	      {
	            if(servo_output>300)   
	          {   
	             servo_output=300 ; 
	          } 
	          if(servo_output<-300)
	          {  
	             servo_output=-300 ;	
	          }	
	     } */
          
         servo_pwm  = duoji_zhongzhi+servo_output;
 
}
//============================= ¶ªÐÅºÅ´¦Àíº¯Êý==========================//
//Ï¸»¯´¦Àí!!!!!
//²½Öè£º
//1¡¢²ÉÑù·ÖÎö
//2¡¢È·¶¨³µÉíÎ»ÖÃÓë×ªÏòÒÔ¼°µç¸ÐÖµµÄÄÚÔÚÁªÏµ
//3¡¢±àÐ´Ö´ÐÐº¯Êý£¬ÒÔ²éÑ¯µÄ·½Ê½¸ø³ö¾­ÑéÖµ
void lost_signal()
{

//diangan_min[0] lost_signal_flag 
 if(   Road_Type[0]==1)//×óÍäµÀ
{
      if( diangan_max[0]<100)
      {
       
        if(diangan_max[0]<60&& servo_output<590)
    	 servo_output = 590;
       else if(diangan_max[0]<85&&servo_output<585)
    	 servo_output = 585;
       else if(diangan_max[0]<100&&servo_output<580)
    	  servo_output = 580;
       
      } 
      
        
}
else if(  Road_Type[0]==2)//ÓÒÍäµÀ
{
      if(diangan_max[0]<100)
      {
       
        if(diangan_max[0]<60&& servo_output>-590)
    	 servo_output =-590;
       else if(diangan_max[0]<85&& servo_output>-585)
    	 servo_output = -585;
       else if(diangan_max[0]<100&&servo_output>-580)
    	  servo_output = -580;
       
      } 
      
        
}  
  /*if((diangan_max[0]<50)&&(qu_zheng(servo_output)<560))
     {
     
         if(cixu[0][0]==0)//L1  
            servo_output = 560;
         else if(cixu[0][0]==2)//L3
            servo_output = -560;
    }*/  	
}

//====================================================================//
//==========================×ªÏò×ÔÊÊÓ¦===============================//
//(103+speed_ave/k)*servo_output/100;
//ÓÉÓÚ³µËÙÓë×ªÏò·Çµ¥Ò»µÄÒ»´Îº¯Êý±ä»¯£¬
//ËùÒÔÔÚ×ªÏòµÄ²¹³¥ÉÏÃæ£¬²»ÄÜ¹»ÓÃµ¥´¿µÄÒ»´Îº¯Êý¼ÆËã
//·Ö¶Îµ÷ÊÔ£¬½«³µËÙÏ¸»¯£¬ÔÚ²»Í¬µÄ³µËÙÉÏÓÃ²»Í¬µÄÏµÊý£¬·´µ÷ÊÔ×îÖÕÈ·¶¨ÏµÊý
void zhuan_xiang_control()
{

 speed_ave=(smartcar_speed_R+smartcar_speed_L)/2;

 if((Road_Type[0]==1)||(Road_Type[0]==2))
{
	 	
	 
	if(speed_ave>3000)  
	{
			servo_output=109*servo_output/100;
		
	}
	else if (speed_ave>2800) 
	{
	        servo_output=108*servo_output/100;
	}
	else if (speed_ave>2600) 
	{
		    servo_output=107*servo_output/100;
	}
	else if (speed_ave>2400) 
	{
		    servo_output=106*servo_output/100;
	}
	else if (speed_ave>2200)
	{
		   servo_output=105*servo_output/100;
	}
	else if (speed_ave>2000)
	{
		   servo_output=100*servo_output/100;
	}
	else if (speed_ave>1800)
	{
		servo_output=100*servo_output/100;
	}
	else if (speed_ave>1600)
	{
		servo_output=100*servo_output/100;
	}
	else if (speed_ave>1200)
	{
		servo_output=100*servo_output/100;
		
	}
	else if (speed_ave>800)
	{
		servo_output=100*servo_output/100;
	}
	 

}
}




//=======================================================================//
//=============================  ¸üÐÂ    Êý¾Ý  ==========================//
//=======================================================================//
void geng_xin_shuju()
{
      
     // µç¸Ð1¸üÐÂÊý×é
       array_one[10]= array_one[9];
       array_one[9] = array_one[8];
       array_one[8] = array_one[7];
       array_one[7] = array_one[6];
       array_one[6] = array_one[5];
       array_one[5] = array_one[4];
       array_one[4] = array_one[3];
       array_one[3] = array_one[2];
       array_one[2] = array_one[1];
       array_one[1] = array_one[0];

      // µç¸Ð2¸üÐÂÊý×é
       array_two[10]= array_two[9];
       array_two[9] = array_two[8];
       array_two[8] = array_two[7];
       array_two[7] = array_two[6];
       array_two[6] = array_two[5];
       array_two[5] = array_two[4];
       array_two[4] = array_two[3];
       array_two[3] = array_two[2];
       array_two[2] = array_two[1];
       array_two[1] = array_two[0];

      // µç¸Ð3¸üÐÂÊý×é
       array_three[10]= array_three[9];
       array_three[9] = array_three[8];
       array_three[8] = array_three[7];
       array_three[7] = array_three[6];
       array_three[6] = array_three[5];
       array_three[5] = array_three[4];
       array_three[4] = array_three[3];
       array_three[3] = array_three[2];
       array_three[2] = array_three[1];
       array_three[1] = array_three[0];
       
      // µç¸Ð4¸üÐÂÊý×é//´¹Ö±1ºÅ
       array_four[10]= array_four[9];
       array_four[9] = array_four[8];
       array_four[8] = array_four[7];
       array_four[7] = array_four[6];
       array_four[6] = array_four[5];
       array_four[5] = array_four[4];
       array_four[4] = array_four[3];
       array_four[3] = array_four[2];
       array_four[2] = array_four[1];
       array_four[1] = array_four[0];
       
       // µç¸Ð5¸üÐÂÊý×é//´¹Ö±2ºÅ
       array_five[10]= array_five[9];
       array_five[9] = array_five[8];
       array_five[8] = array_five[7];
       array_five[7] = array_five[6];
       array_five[6] = array_five[5];
       array_five[5] = array_five[4];
       array_five[4] = array_five[3];
       array_five[3] = array_five[2];
       array_five[2] = array_five[1];
       array_five[1] = array_five[0];
       
        //¸üÐÂÆ«²îÖµ
        
       /*servo_Error[79]  = servo_Error[78];
       servo_Error[78]  = servo_Error[77];
       servo_Error[77]  = servo_Error[76];
       servo_Error[26]  = servo_Error[75];
       servo_Error[75]  = servo_Error[74];
       servo_Error[74]  = servo_Error[73];
       servo_Error[73]  = servo_Error[72];
       servo_Error[72]  = servo_Error[71];
       servo_Error[71]  = servo_Error[70];
       servo_Error[70]  = servo_Error[69];
       servo_Error[69]  = servo_Error[68];
       servo_Error[68]  = servo_Error[67];
       servo_Error[67]  = servo_Error[66];
       servo_Error[66]  = servo_Error[65];
       servo_Error[65]  = servo_Error[64];
       servo_Error[64]  = servo_Error[63];
       servo_Error[63]  = servo_Error[62];
       servo_Error[62]  = servo_Error[61];
       servo_Error[61]  = servo_Error[60];
       servo_Error[60]  = servo_Error[59];
       servo_Error[59]  = servo_Error[58];
       servo_Error[58]  = servo_Error[57];
       servo_Error[57]  = servo_Error[56];
       servo_Error[56]  = servo_Error[55];
       servo_Error[55]  = servo_Error[54];
       servo_Error[54]  = servo_Error[53];
       servo_Error[53]  = servo_Error[52];
       servo_Error[52]  = servo_Error[51];
       servo_Error[51]  = servo_Error[50];
       servo_Error[50]  = servo_Error[49];  
       servo_Error[49]  = servo_Error[48];
       servo_Error[48]  = servo_Error[47];
       servo_Error[47]  = servo_Error[46];
       servo_Error[46]  = servo_Error[45];
       servo_Error[45]  = servo_Error[44];
       servo_Error[44]  = servo_Error[43];
       servo_Error[43]  = servo_Error[42];
       servo_Error[42]  = servo_Error[41];
       servo_Error[41]  = servo_Error[40];*/
       servo_Error[40]  = servo_Error[39];
       servo_Error[39]  = servo_Error[38];
       servo_Error[38]  = servo_Error[37];
       servo_Error[37]  = servo_Error[36];
       servo_Error[36]  = servo_Error[35];
       servo_Error[35]  = servo_Error[34];
       servo_Error[34]  = servo_Error[33];
       servo_Error[33]  = servo_Error[32];
       servo_Error[32]  = servo_Error[31];
       servo_Error[31]  = servo_Error[30];
       servo_Error[30]  = servo_Error[29];
       servo_Error[29]  = servo_Error[28];
       servo_Error[28]  = servo_Error[27];
       servo_Error[27]  = servo_Error[26];
       servo_Error[26]  = servo_Error[25];
       servo_Error[25]  = servo_Error[24];
       servo_Error[24]  = servo_Error[23];
       servo_Error[23]  = servo_Error[22];
       servo_Error[22]  = servo_Error[21];
       servo_Error[21]  = servo_Error[20];
       servo_Error[20]  = servo_Error[19];
       servo_Error[19]  = servo_Error[18];
       servo_Error[18]  = servo_Error[17];
       servo_Error[17]  = servo_Error[16];
       servo_Error[16]  = servo_Error[15];
       servo_Error[15]  = servo_Error[14];
       servo_Error[14]  = servo_Error[13];
       servo_Error[13]  = servo_Error[12];
       servo_Error[12]  = servo_Error[11];
       servo_Error[11]  = servo_Error[10];
       servo_Error[10]  = servo_Error[9];
       servo_Error[9]   = servo_Error[8];
       servo_Error[8]   = servo_Error[7];
       servo_Error[7]   = servo_Error[6];
       servo_Error[6]   = servo_Error[5];
       servo_Error[5]   = servo_Error[4];
       servo_Error[4]   = servo_Error[3];
       servo_Error[3]   = servo_Error[2];
       servo_Error[2]   = servo_Error[1];
       servo_Error[1]   = servo_Error[0];
       
    //=================¸üÐÂÆ«²îÂÊ=================//
       servo_Error_c[10]  = servo_Error_c[9];
       servo_Error_c[9]   = servo_Error_c[8];
       servo_Error_c[8]   = servo_Error_c[7];
       servo_Error_c[7]   = servo_Error_c[6];
       servo_Error_c[6]   = servo_Error_c[5];
       servo_Error_c[5]   = servo_Error_c[4];
       servo_Error_c[4]   = servo_Error_c[3];
       servo_Error_c[3]   = servo_Error_c[2];
       servo_Error_c[2]   = servo_Error_c[1];
       servo_Error_c[1]   = servo_Error_c[0];
       
          //´¢´æÉÏÒ»´ÎµÄÆ«²î£¬ÓÃÀ´ÂË²¨
       shuiping_piancha[9]=shuiping_piancha[8] ;
       shuiping_piancha[8]=shuiping_piancha[7] ;
       shuiping_piancha[7]=shuiping_piancha[6] ;
       shuiping_piancha[6]=shuiping_piancha[5] ;
       shuiping_piancha[5]=shuiping_piancha[4] ;
       shuiping_piancha[4]=shuiping_piancha[3] ;
       shuiping_piancha[3]=shuiping_piancha[2] ;
       shuiping_piancha[2]=shuiping_piancha[1] ;
       shuiping_piancha[1]=shuiping_piancha[0] ;
       
       chuizhi_piancha[9]=chuizhi_piancha[8] ;
       chuizhi_piancha[8]=chuizhi_piancha[7] ;
       chuizhi_piancha[7]=chuizhi_piancha[6] ;
       chuizhi_piancha[6]=chuizhi_piancha[5] ;
       chuizhi_piancha[5]=chuizhi_piancha[4] ;
       chuizhi_piancha[4]=chuizhi_piancha[3] ;
       chuizhi_piancha[3]=chuizhi_piancha[2] ;
       chuizhi_piancha[2]=chuizhi_piancha[1] ;
       chuizhi_piancha[1]=chuizhi_piancha[0] ;
       
       //¸üÐÂÊä³öÁ¿
         servo_output_last[19]=servo_output_last[18];     
         servo_output_last[18]=servo_output_last[17];     
         servo_output_last[17]=servo_output_last[16];      
         servo_output_last[16]=servo_output_last[15];      
         servo_output_last[15]=servo_output_last[14];       
         servo_output_last[14]=servo_output_last[13];
         servo_output_last[13]=servo_output_last[12];
         servo_output_last[12]=servo_output_last[11];
         servo_output_last[11]=servo_output_last[10];
         servo_output_last[10]=servo_output_last[9];
         servo_output_last[9]=servo_output_last[8];
         servo_output_last[8]=servo_output_last[7];
         servo_output_last[7]=servo_output_last[6];
         servo_output_last[6]=servo_output_last[5];
         servo_output_last[5]=servo_output_last[4];
         servo_output_last[4]=servo_output_last[3];
         servo_output_last[3]=servo_output_last[2];
         servo_output_last[2]=servo_output_last[1];
         servo_output_last[1]=servo_output_last[0];
         servo_output_last[0]=servo_output;
   
         // ¸üÐÂÈýË®Æ½µç¸ÐÖµÖ®ºÍ
         /*total_diangan_sum[10]= total_diangan_sum[9] ;
         total_diangan_sum[9] = total_diangan_sum[8] ;
         total_diangan_sum[8] = total_diangan_sum[7] ;
         total_diangan_sum[7] = total_diangan_sum[6] ;
         total_diangan_sum[6] = total_diangan_sum[5] ;
         total_diangan_sum[5] = total_diangan_sum[4] ;*/
         total_diangan_sum[4] = total_diangan_sum[3] ;
         total_diangan_sum[3] = total_diangan_sum[2] ;
         total_diangan_sum[2] = total_diangan_sum[1] ;
         total_diangan_sum[1] = total_diangan_sum[0] ;
         // ¸üÐÂÁ½  ´¹Ö±µç¸ÐÖµÖ®ºÍ
 
         //¸üÐÂ×î´óµç¸ÐµÄ´ÎÐò
         cixu[3][0]=cixu[2][0];
	     cixu[2][0]=cixu[1][0];
	 	 cixu[1][0]=cixu[0][0];
		//¸üÐÂ´Î´óµç¸ÐµÄ´ÎÐò
		 cixu[3][1]=cixu[2][1];
		 cixu[2][1]=cixu[1][1];
		 cixu[1][1]=cixu[0][1];
		//¸üÐÂ×îÐ¡µç¸ÐµÄ´ÎÐò
		 cixu[3][2]=cixu[2][2];
		 cixu[2][2]=cixu[1][2];
		 cixu[1][2]=cixu[0][2];
		 
		 //¸üÐÂ×î´óµç¸ÐÖµ
		 
		 
		 diangan_max[20]=diangan_max[19];
         diangan_max[19]=diangan_max[18];
         diangan_max[18]=diangan_max[17];
         diangan_max[17]=diangan_max[16];
         diangan_max[16]=diangan_max[15];
         diangan_max[15]=diangan_max[14];
         diangan_max[14]=diangan_max[13];
         diangan_max[13]=diangan_max[12];
		 diangan_max[12]=diangan_max[11];
		 diangan_max[11]=diangan_max[10];
         diangan_max[10]=diangan_max[9];
         diangan_max[9]=diangan_max[8];
         diangan_max[8]=diangan_max[7];
         diangan_max[7]=diangan_max[6];
         diangan_max[6]=diangan_max[5];
         diangan_max[5]=diangan_max[4];
         diangan_max[4]=diangan_max[3];
         diangan_max[3]=diangan_max[2]; 
         diangan_max[2]=diangan_max[1];
         diangan_max[1]=diangan_max[0];
         
         //¸üÐÂ×îÐ¡µç¸ÐÖµ
 		 /*diangan_min[12]=diangan_min[11];
         diangan_min[11]=diangan_min[10];
         diangan_min[10]=diangan_min[9];
         diangan_min[9]=diangan_min[8];
         diangan_min[8]=diangan_min[7];
         diangan_min[7]=diangan_min[6];
         diangan_min[6]=diangan_min[5];
         diangan_min[5]=diangan_min[4];
         diangan_min[4]=diangan_min[3];
         diangan_min[3]=diangan_min[2];
         diangan_min[2]=diangan_min[1];
         diangan_min[1]=diangan_min[0];*/
         
		 //¸üÐÂÈüµÀÀàÐÍ
		 Road_Type[6]=Road_Type[5];
         Road_Type[5]=Road_Type[4];
         Road_Type[4]=Road_Type[3];
	  	 Road_Type[3]=Road_Type[2];
		 Road_Type[2]=Road_Type[1];
		 Road_Type[1]=Road_Type[0];
		 
		 
		chui_zhi_daingan[12]=chui_zhi_daingan[11];
		chui_zhi_daingan[11]=chui_zhi_daingan[10];
        chui_zhi_daingan[10]=chui_zhi_daingan[9];
        chui_zhi_daingan[9]=chui_zhi_daingan[8];
        chui_zhi_daingan[8]=chui_zhi_daingan[7];
        chui_zhi_daingan[7]=chui_zhi_daingan[6];
        chui_zhi_daingan[6]=chui_zhi_daingan[5];
        chui_zhi_daingan[5]=chui_zhi_daingan[4];
        chui_zhi_daingan[4]=chui_zhi_daingan[3];
        chui_zhi_daingan[3]=chui_zhi_daingan[2];
        chui_zhi_daingan[2]=chui_zhi_daingan[1];
        chui_zhi_daingan[1]=chui_zhi_daingan[0];
        
        
        wandao_sppch[6]=wandao_sppch[5];
        wandao_sppch[5]=wandao_sppch[4];
        wandao_sppch[4]=wandao_sppch[3];
        wandao_sppch[3]=wandao_sppch[2];
        wandao_sppch[2]=wandao_sppch[1];
        wandao_sppch[1]=wandao_sppch[0];
        
        // CCD ×ó ÓÒ ºÚµã¸öÊý ¸üÐÂ
		 CCD_righttimes[8]=CCD_righttimes[7] ;
		 CCD_righttimes[7]=CCD_righttimes[6] ;
		 CCD_righttimes[6]=CCD_righttimes[5] ;
		 CCD_righttimes[5]=CCD_righttimes[4] ;
		 CCD_righttimes[4]=CCD_righttimes[3] ;
		 CCD_righttimes[3]=CCD_righttimes[2] ;
		 CCD_righttimes[2]=CCD_righttimes[1] ;
		 CCD_righttimes[1]=CCD_righttimes[0] ;
		 
		 CCD_lefttimes[8]=CCD_lefttimes[7] ;
		 CCD_lefttimes[7]=CCD_lefttimes[6] ;
		 CCD_lefttimes[6]=CCD_lefttimes[5] ;
		 CCD_lefttimes[5]=CCD_lefttimes[4] ;
		 CCD_lefttimes[4]=CCD_lefttimes[3] ;
		 CCD_lefttimes[3]=CCD_lefttimes[2] ;
		 CCD_lefttimes[2]=CCD_lefttimes[1] ;
		 CCD_lefttimes[1]=CCD_lefttimes[0] ;
 
  
}



 
//º¯Êý¹¦ÄÜ
//Ð¡³µÃ¿×ß100cm·¢³ö1000.18¸öÂö³å£¬Ê¡ÂÔÎª1000¸ö
//------------------------------------------------------------------//
 
void distance_calculate()
{ 
    //uint32_t zhijiao_charge[5]={0};
    //uint32_t xiezhi_point[2]={0};
    //uint32_t chuizhi_point[2]={0};
    //uint32_t zhijaio_point_k[10]={0};
    //chui_zhi_daingan[0]  
    //shuiping_piancha[0]
  	//chuizhi_piancha[0] 
  	//xiezhi_piancha[0]
   //º¯Êý¹¦ÄÜ£º¼ÇÂ¼¸Õ½øÈëÍäµÀµÄÎ»ÒÆ
   if(Road_Type[1]!=1&&Road_Type[0]==1)
   {
   	 distance[0]=(CCD_juli_zuo[0]+CCD_juli_you[0])/2;//¶ÁÈ¡µ±Ç°Î»ÒÆ
   	 zhijiao_charge[0]=1;                //½øÈë¼ì²â³ÌÐò±êÖ¾Î»
   	 chuizhi_point[0]=qu_zheng(chuizhi_piancha[0]) ;//¶ÁÈ¡µ±Ç°´¹Ö±Î»ÒÆÁ¿£¬È¡ÕýÖµ
   	 xiezhi_point[0]=qu_zheng(xiezhi_piancha[0]) ;//¶ÁÈ¡µ±Ç°Ð±ÖÃÎ»ÒÆÁ¿£¬È¡ÕýÖµ
  	 
   	
   	
   }
   else if(Road_Type[1]!=2&&Road_Type[0]==2)
   {
   	 distance[0]=(CCD_juli_zuo[0]+CCD_juli_you[0])/2;//¶ÁÈ¡µ±Ç°Î»ÒÆ
   	 zhijiao_charge[0]=1;                //½øÈë¼ì²â³ÌÐò±êÖ¾Î»
   	 chuizhi_point[0]=qu_zheng(chuizhi_piancha[0]) ;//¶ÁÈ¡µ±Ç°´¹Ö±Î»ÒÆÁ¿£¬È¡ÕýÖµ
   	 xiezhi_point[0]=qu_zheng(xiezhi_piancha[0]) ;//¶ÁÈ¡µ±Ç°Ð±ÖÃÎ»ÒÆÁ¿£¬È¡ÕýÖµ
  	
   }
    
 //=========================Ö±½ÇÅÐ¶Ï=========================//   
  if((zhijiao_charge[0]==1)&&(Road_Type[0]==1||Road_Type[0]==2))
  {
         //µ±Ð¡³µ×ß¹ý15CMÊ±£¬½øÈë¼ì²â³ÌÐò
  	     if((CCD_juli_zuo[0]+CCD_juli_you[0])/2>distance[0]+150)
  	     { 
   	        chuizhi_point[1]=qu_zheng(chuizhi_piancha[0]) ;//¶ÁÈ¡µ±Ç°´¹Ö±Î»ÒÆÁ¿£¬È¡ÕýÖµ
   	        xiezhi_point[1]=qu_zheng(xiezhi_piancha[0]) ;
  	        zhijiao_charge[0]=0;//±êÖ¾Î»ÇåÁã£¬²»ÔÙ½øÈë´Ë³ÌÐò£¬µÈ´ýÏÂÒ»´Î¡£¡£¡£ 
  	        //¼ÆËãÐ±ÂÊ
  	        distance[1]=(CCD_juli_zuo[0]+CCD_juli_you[0])/2;
  	        zhijaio_point_k[0]=20*qu_zheng(chuizhi_point[1]-chuizhi_point[0])/
  	                               (xiezhi_point[1]+xiezhi_point[0])  ;
  	                               
  	         
  	         if(zhijaio_point_k[0]>10) zhijiao_charge[1]=1;//Ö±½Ç¼±Íä
  	         else zhijiao_charge[1]=0;             
  	     }
  	   
  	
        
     
  }
  
  
 if(zhijiao_charge[1]==1) 
 {
     if((CCD_juli_zuo[0]+CCD_juli_you[0])/2>distance[1]+400)
      
       zhijiao_charge[1]=0;	
 }
 
   
   if(zhijiao_charge[1]==1) SIU.GPDO[PCR38_PC6].R =0;
   else SIU.GPDO[PCR38_PC6].R =1;     
	  
}
//===================================================================//
void protect()
{
      
      if(diangan_max[0]<3)
      {
           lost_time++;
           
           if(lost_time>1500)
           {
	           CarSpeed_SET_L=0;//½ô¼±É²³µ£¬±£»¤Ð¡³µ
	           CarSpeed_SET_R=0;
	            
	           
           }
      }
      else  lost_time=0;
      
          
}

 
 
//====================================================================//
uint16_t Difference_new(int chasu)
{


 //2700  550 520 500 450 400 350 300 250 200
  
  
   /*if(Duoji_value>700)      cha_su_value=350;
   else if(Duoji_value>650) cha_su_value=320;
   else if(Duoji_value>600) cha_su_value=300;
   else if(Duoji_value>550) cha_su_value=270;
   else if(Duoji_value>500) cha_su_value=250;
   else if(Duoji_value>450) cha_su_value=200;
   else if(Duoji_value>400) cha_su_value=150;
   else if(Duoji_value>350) cha_su_value=120;
   else if(Duoji_value>300) cha_su_value=80;
    
   else  cha_su_value=0;*/
   
   
  cha_su_value=7*Duoji_value*wandao_CarSpeed_SET_L/600/50;
   
  //========================×ó±ß³µÂÖ==============================//   
   if(chasu==1)
   {      if(servo_output>0)
		   {          //Ïò×ó×ª,×óÂÖ×ªµÄÂý
		      CarSpeed_SET_L=wandao_CarSpeed_SET_L-70*cha_su_value/100 ;              
		   }     
	       else  if(servo_output<0)
		   {       //ÏòÓÒ×ª,×óÂÖ×ªµ½µÄ¿ì
		       CarSpeed_SET_L=wandao_CarSpeed_SET_L+k1*cha_su_value/100;                                 
		   }
           return CarSpeed_SET_L;  
   }
   
   
   
   
 //=========================ÓÒ±ß³µÂÖ============================//  
  else if(chasu==2)
  {
  	if(servo_output>0)
    {        //Ïò×ó×ª,ÓÒÂÖ×ªµ½¿ì
		   CarSpeed_SET_R=wandao_CarSpeed_SET_R+k1*cha_su_value/100;
    }     
    else if(servo_output<0)
    {        //ÏòÓÒ×ª,ÓÒÂÖ×ªµÄÂý 
		    CarSpeed_SET_R=wandao_CarSpeed_SET_R-70*cha_su_value/100  ;                               
    }
		    return CarSpeed_SET_R;
  	
  }
 
}
 
 
 
void speed_control_new( void )
{
	
	Duoji_value=qu_zheng(servo_output);
   
   if(Duoji_value>650) Duoji_value=650; 
 
 
 
 switch(speed_dangwei)
 {
 	 
   case 0:
 	
   	 zhidao_speed= 0;
     wandao_speed= 0;
      
   break; 
 	 
 	 
 	case 1:
 	
   	 zhidao_speed=600;
     wandao_speed=100;
      
   break;
   
 	case 2:
 	
   	  zhidao_speed=800;
      wandao_speed=100;
       
      break;
 	case 3:
 	
   	 zhidao_speed=1200;
     wandao_speed=100;
      
   
 	break;
 	
	case 4:
 	
   	 zhidao_speed=1500;
     wandao_speed=100;
 
 	break;
 	case 5:
 	
   	 zhidao_speed=2000;
     wandao_speed=100;
 
 	break;
 	
 }
     
 //===============================ËÙ¶È¿ØÖÆ=========================//    
  // pid_times[0]    
	if(Road_Type[0]==0)  
	{
			
		   
			CarSpeed_SET_R=last_CarSpeed_SET_R+zhidao_speed; 
	        CarSpeed_SET_L=last_CarSpeed_SET_L+zhidao_speed;
	        
 
	     // pid_times[0]++ ; 
            
   }
    //pid_flag[0];
 	    
	else if(Road_Type[0]==1||Road_Type[0]==2)
	{           
	            
	        /*if(qu_zheng(chui_zhi_daingan[0])<60)
	        {
	            if(++Road_Type_Times[5]>100)
	            {
  	
	            	CarSpeed_SET_R=last_CarSpeed_SET_R+zhidao_speed; 
	                CarSpeed_SET_L=last_CarSpeed_SET_L+zhidao_speed;
	                 	
	            }
	            else Road_Type_Times[5]=0;
	        	
	        }*/
   
	            wandao_CarSpeed_SET_L=last_CarSpeed_SET_L+wandao_speed;
                wandao_CarSpeed_SET_R=last_CarSpeed_SET_R+wandao_speed;
                
               if(Duoji_value>260)
               {
               	CarSpeed_SET_L=Difference_new(1);
                CarSpeed_SET_R=Difference_new(2);
               }
               else
               {
               	CarSpeed_SET_L=wandao_CarSpeed_SET_L;
                CarSpeed_SET_R=wandao_CarSpeed_SET_R;
               }
 
	}
	
	else
	{
	      CarSpeed_SET_R=last_CarSpeed_SET_R; 
          CarSpeed_SET_L=last_CarSpeed_SET_L;
       	
	}
	
 
 
 
 
 
 
 
 if(speed_ave> (CarSpeed_SET_R+ CarSpeed_SET_R)/2+1000)
 {	
 	
 	 	
	SIU.GPDO[PCR40_PC8].R = 1 ;       //1-0·´×ª
    SIU.GPDO[PCR44_PC12].R = 0 ;
    motor_pwmR= 7000;
                      
    SIU.GPDO[PCR41_PC9].R = 1;
    SIU.GPDO[PCR47_PC15].R = 0;
    motor_pwmL= 7000;
 			      
 	
	
 }
 else if(speed_ave> (CarSpeed_SET_R+ CarSpeed_SET_R)/2+700)
 {	
 	
 	 	
	SIU.GPDO[PCR40_PC8].R = 1 ;       //1-0·´×ª
    SIU.GPDO[PCR44_PC12].R = 0 ;
    motor_pwmR= 5500;
                      
    SIU.GPDO[PCR41_PC9].R = 1;
    SIU.GPDO[PCR47_PC15].R = 0;
    motor_pwmL= 5500;
 			      
 	
	
 }
 
 else if(speed_ave> (CarSpeed_SET_R+ CarSpeed_SET_R)/2+300)
 {	
 	
 	 	
	SIU.GPDO[PCR40_PC8].R = 1 ;       //1-0·´×ª
    SIU.GPDO[PCR44_PC12].R = 0 ;
    motor_pwmR= 4000;
                      
    SIU.GPDO[PCR41_PC9].R = 1;
    SIU.GPDO[PCR47_PC15].R = 0;
    motor_pwmL= 4000;
 			      
 	
	
 }
 else start_pid=1;
 
 
 
 
 if((CarSpeed_SET_R+ CarSpeed_SET_R)/2<1000)  start_pid=1;
 	
}

 
//=================================================================//
////////////////////////////PID²ÎÊýÑ¡Ôñ//////////////////////////////
//===============================×ó±ß================================
void pid_change_right()
{
  
  
  	if(qu_zheng(smartcar_speed_R-CarSpeed_SET_R)>1500)//2000
  	{
  	
  	        if(smartcar_speed_R<CarSpeed_SET_R)
  	        {
  	        	PID.Proportion=5900;         
                PID.Integral= 400;            
                PID.Derivative= 330 ;
  	        }
  	        else  
  	        {
  	        	PID.Proportion=6700;         
                PID.Integral= 1200;            
                PID.Derivative= 330 ;
  	        }
  	    	  	
  	}
  
  
  else 	if(qu_zheng(smartcar_speed_R-CarSpeed_SET_R)>1000)//2000
  	{
  	        if(smartcar_speed_R<CarSpeed_SET_R)
  	        {
  	        	PID.Proportion=5800;         
                PID.Integral= 380;            
                PID.Derivative= 300 ;  
  	        }
  	        else
  	        {
  	        	PID.Proportion=6500;         
                PID.Integral= 1100;            
                PID.Derivative= 300 ;  
  	        
  	        	
  	        }
  	    		
  	}
  	
  else	if(qu_zheng(smartcar_speed_R-CarSpeed_SET_R)>700)//1500
  	{
  	       if(smartcar_speed_R<CarSpeed_SET_R)
  	       {
  	
  	     	PID.Proportion=5600;         
            PID.Integral= 340;            
            PID.Derivative= 260 ;
            }
            else
	        {
  	            PID.Proportion=6300;         
	            PID.Integral=  900;            
	            PID.Derivative= 260 ;
	        }
  	}
    else	if(qu_zheng(smartcar_speed_R-CarSpeed_SET_R)>400)//1000
  	{
  	         if(smartcar_speed_R<CarSpeed_SET_R)
  	         {
  	            PID.Proportion=5400;         
                PID.Integral= 320;            
                PID.Derivative= 220 ;  
             }
             else
             {
  	            PID.Proportion=6100;         
                PID.Integral=  800;            
                PID.Derivative= 220 ;  
             }
  	}
    
       else	if(qu_zheng(smartcar_speed_R-CarSpeed_SET_R)>200)//1000
  	{
  	         if(smartcar_speed_R<CarSpeed_SET_R)
  	         {
  	            PID.Proportion=5400;         
                PID.Integral= 320;            
                PID.Derivative= 220 ;  
             }
             else
             {
  	            PID.Proportion=6000;         
                PID.Integral=  650;            
                PID.Derivative= 220 ;  
             }
  	}
    
    
  	else
  	{
  	         if(smartcar_speed_R<CarSpeed_SET_R)
  	         {
 
  	 	      PID.Proportion=5400;         
              PID.Integral=310;            
              PID.Derivative=210 ;
             }
             else
             {
 
  	 	       PID.Proportion=5700;         
               PID.Integral=400;            
               PID.Derivative=210 ;
             }
  	}
          
}
void pid_change_left()
{
  
  
  	if(qu_zheng(smartcar_speed_L-CarSpeed_SET_L)>1500)//2000
  	{
  	
  	        if(smartcar_speed_L<CarSpeed_SET_L)
  	        {
  	        	PID.Proportion=5900;         
                PID.Integral= 400;            
                PID.Derivative= 330 ;
  	        }
  	        else  
  	        {
  	        	PID.Proportion=6700;         
                PID.Integral= 1200;            
                PID.Derivative= 330 ;
  	        }
  	    	  	
  	}
  
  
  else 	if(qu_zheng(smartcar_speed_L-CarSpeed_SET_L)>1000)//2000
  	{
  	        if(smartcar_speed_L<CarSpeed_SET_L)
  	        {
  	        	PID.Proportion=5800;         
                PID.Integral= 380;            
                PID.Derivative= 300 ;  
  	        }
  	        else
  	        {
  	        	PID.Proportion=6500;         
                PID.Integral= 1100;            
                PID.Derivative= 300 ;  
  	        
  	        	
  	        }
  	    		
  	}
  	
  else	if(qu_zheng(smartcar_speed_L-CarSpeed_SET_L)>700)//1500
  	{
  	       if(smartcar_speed_L<CarSpeed_SET_L)
  	       {
  	
  	     	PID.Proportion=5600;         
            PID.Integral= 340;            
            PID.Derivative= 260 ;
            }
            else
	        {
  	            PID.Proportion=6300;         
	            PID.Integral=  900;            
	            PID.Derivative= 260 ;
	        }
  	}
  	
  else	if(qu_zheng(smartcar_speed_L-CarSpeed_SET_L)>400)//1500
  	{
  	       if(smartcar_speed_L<CarSpeed_SET_L)
  	       {
  	
  	     	PID.Proportion=5600;         
            PID.Integral= 340;            
            PID.Derivative= 260 ;
            }
            else
	        {
  	            PID.Proportion=6100;         
	            PID.Integral=  800;            
	            PID.Derivative= 260 ;
	        }
  	}	
  	
    else	if(qu_zheng(smartcar_speed_L-CarSpeed_SET_L)>200)//1000
  	{
  	         if(smartcar_speed_L<CarSpeed_SET_L)
  	         {
  	            PID.Proportion=5400;         
                PID.Integral= 320;            
                PID.Derivative= 220 ;  
             }
             else
             {
  	            PID.Proportion=6000;         
                PID.Integral=  650;            
                PID.Derivative= 220 ;  
             }
  	}
    
    
  	else
  	{
  	         if(smartcar_speed_L<CarSpeed_SET_L)
  	         {
 
  	 	      PID.Proportion=5400;         
              PID.Integral=310;            
              PID.Derivative=210 ;
            }
             else
            {
 
  	 	      PID.Proportion=5700;         
              PID.Integral=400;            
              PID.Derivative=210 ;
             }
  	}
          
}
void yejingping_xuanze( void )
{
        
        Show_Me_Data(7,1,20);   
        if( piancha_E_jisuan==8)  // ·¢³µ³ÌÐò
        {
        	  //============== ¹éÒ»»¯ ========================
                guiyi_MAX[0] = 200 ;//195  238
		        guiyi_MAX[1] = 240 ;//183  220
			    guiyi_MAX[2] = 240 ;//183   225
		        guiyi_MAX[3] = 235 ;//181   223
		        guiyi_MAX[4] = 245 ;//179   220
		        guiyi_MAX[5] = 245 ;//179   220
		        
		               Show_Me_Data(2500,50,0);  //Éè¶¨Æ½¾ùËÙ¶È
                       Show_Me_Data(1,1,1);      //ËÙ¶ÈµµÎ»
                       Show_Me_Data(400,50,2);  //ÂúP¼ÓËÙÊ±¼äÉè¶¨ 
                       
                       Show_Me_Data(3,1,13) ;       //Ñ¡ÔñÊÇ·ñÕÏ°­     3¡ª¡ª±ÜÕÏ   CCD_xuanze_flag
                       if( CCD_xuanze_flag==3 )
	                   {
	             		Show_Me_Data(1,1,19) ;    //CCD ÆØ¹âÊ±¼ä    /cm   
	             		Show_Me_Data(5,1,16) ;    //±ÜÕÏ ¡ª¡ª ¾àÀë ¡ª¡ª Âö³å     /cm  
	             		Show_Me_Data(101,20,17) ;    //   µ½´ï¾àÀëºó   ¿ªÆô  CCD    CCD_juli_dm_she_ding_juli[2]>2 ¿ªÆô
	             		if( CCD_juli_dm_she_ding_juli[2]>2 )	//    µ½´ï¾àÀëºó   ¹Ø±Õ  CCD
	             			Show_Me_Data(401,20,18) ;   //
	                    }
	               
	               Show_Me_Data(50,10,22) ; 
	               Show_Me_Data(7,1,5) ;       //  ¼ÆËã E Ñ¡Ôñ
        
        
        }
        else   // µ÷ÊÔ³ÌÐò
        {
        	  //============== ¹éÒ»»¯ ========================
                 guiyi_chushihua() ;  
                 Show_Me_Data(2500,50,0);  //Éè¶¨Æ½¾ùËÙ¶È
	             Show_Me_Data(1,1,1);      //ËÙ¶ÈµµÎ»
	             Show_Me_Data(400,50,2);  //ÂúP¼ÓËÙÊ±¼äÉè¶¨ 
	 	         SIU.GPDO[PCR38_PC6].R =0 ;//¸ø0²ÅÁÁ  
	 	   
	                 //==============================================        		       		
	             Show_Me_Data(3,1,13) ;       //Ñ¡ÔñÊÇ·ñÕÏ°­     3¡ª¡ª±ÜÕÏ   CCD_xuanze_flag
	             if( CCD_xuanze_flag==3 )
	             {
	             		Show_Me_Data(1,1,19) ;    //CCD ÆØ¹âÊ±¼ä    /cm   
	             		Show_Me_Data(5,1,16) ;    //±ÜÕÏ ¡ª¡ª ¾àÀë ¡ª¡ª Âö³å     /cm  
	             		Show_Me_Data(101,20,17) ;    //   µ½´ï¾àÀëºó   ¿ªÆô  CCD    CCD_juli_dm_she_ding_juli[2]>2 ¿ªÆô
	             		Show_Me_Data(401,20,18) ;   //
	             }
	             
	              Show_Me_Data(50,10,22) ;
	               Show_Me_Data(7,1,5) ;       //¼ÆËã E Ñ¡Ôñ

	               //Show_Me_Data(277,20,15) ;   //µ÷ÊÔ¾àÀë Í£³µ     /cm      <27 ²»¶¨¾àÀëÍ£³µ
	                                                                                 //    Show_Me_Data(5900,2,14) ;    //¼ì²é¶æ»úÖÐÖµ   /cm  

        }

}


void CCD_xuanze( void )
{
	    //=============      ÅÐ¶¨ÔÚÖ±µÀ   ¡ª¡ª   ½øÐÐ±ÜÕÏ   =======================
          if( CCD_bizhang_flag==0 && podao_flag==0)
           {
	       //  if( Road_Type[0]==0 )
	          if( CCD_xuanze_flag==3 && array_current[4]<20 && array_current[5]<20 && chuizhi_piancha[0]<4 && chuizhi_piancha[0]>-4 )  // 
	  	    {        // ÎªÁËÈ·¶¨ÊÇ·ñÔÚÖ±µÀ array_current[1]>array_current[0] && array_current[2]>array_current[3] &&
		           	if( ( array_current[1]>40 &&array_current[1]<300 ) )   // 
		          	{                                                                                                                     //      Î´¶ªÐÅºÅ                            Î´ÉÏÆÂµÀ          
		                          CCD_jiance_you_zhangai_flag++; 
		                          if( CCD_jiance_you_zhangai_flag>20 )
/*  */	 		                         CCD_bizhang( ) ;  
/*  	  */               }
		          	else
		                         CCD_jiance_you_zhangai_flag=0 ;
	  	    }
		   else
	         	   CCD_jiance_you_zhangai_flag=0 ;
                 }
	    
	   //================             °´¾àÀë±ÚÕÏ     ===================	
	      if( CCD_bizhang_flag!=0 )
	      {         //  !=0 ËµÃ÷ ¿´ µ½ÕÏ°­ÁË
			//   	CarSpeed_SET_R=0 ;    CarSpeed_SET_L=0 ;
	         	SIU.GPDO[PCR38_PC6].R=0 ;
			
			    CCD_juli[0]=( CCD_juli_zuo[0]+ CCD_juli_you[0]-CCD_juli_zuo[1] -CCD_juli_you[1] )/2 ;
			    if( CCD_juli[0]>CCD_juli_dm_she_ding_juli[0] ) //  ±ÜÕÏÑÓÊ± ½áÊø
			    {
				     CCD_bizhang_flag=0 ; //  =0 ËµÃ÷ Àë¿ªÕÏ°­ÁË
				     CCD_juli_flag=0 ; // ±£Ö¤ CCD_juli_zuo[1]  Ö»½øÐÐÒ»´Î¸³Öµ
			    }
	      }
	      else
		        SIU.GPDO[PCR38_PC6].R=1 ;      
			      //=====================================================       
		 if( CCD_bizhang_flag==1 )    //   ¶æ»úÖÐÖµ
			    duoji_zhongzhi=duoji_zhongzhi_value-300 ;  //   5443  5843    6243
		 else if( CCD_bizhang_flag==2 )
			    duoji_zhongzhi=duoji_zhongzhi_value+300 ;  // 
		 else duoji_zhongzhi=duoji_zhongzhi_value ;  //6915      
		 
	     //=============      ÅÐ¶¨ÊÇ·ñÔÚÆÂµÀ   ¡ª¡ª   ½øÐÐ±ÜÕÏ   =======================	 
	     if( array_current[1]>250 && array_current[2]>250 )  podao_flag=1 ; 
	     if( podao_flag==1&&podao_flag_dingju==0 )    
	     {
	     		podao_flag_dingju=1 ;
	     		CCD_juli_zuo[2]=CCD_juli_zuo[0] ;   CCD_juli_you[2]=CCD_juli_you[0] ; 
	     }
	     if( podao_flag_dingju==1 ) 
	     {  
	     		CCD_juli[2]=( CCD_juli_zuo[0]+ CCD_juli_you[0]-CCD_juli_zuo[2] -CCD_juli_you[2] )/2 ;
	     		if( CCD_juli[2]>2000 ) //  ±ÜÕÏÑÓÊ± ½áÊø
			{
			         podao_flag_dingju=0 ; //  =0 ËµÃ÷ Àë¿ªÆÂµÀÁË
				  podao_flag=0 ;    
			}
	     }
	    
	    
	       
        //    Show_Data(CCD_bizhang_flag) ; 
         	 /*          sendcnt++;
	  			            if(sendcnt>=25)    
				            {   
				                   if(sendcnt<=100) 
								SendImageData(CCD_sample) ;
			                          else if(sendcnt<=200) 
				                  	       SendImageData(CCD_sample_now) ;
			                       
				                    if(sendcnt>210)    sendcnt=0 ;
				                    
				             //      SIU.GPDO[PCR38_PC6].R =!SIU.GPDO[PCR38_PC6].R ;//¸ø0²ÅÁÁ
				           }   
			           
		      */ 
		      
		      
}

void ding_ju_tingche( void )
{
	 //=================          ÊÇ·ñ¶¨¾àÀëÍ£³µ      ==============================       
			     
            if(  (CCD_juli_zuo[0]+CCD_juli_you[0])/2 > CCD_juli_dm_she_ding_juli[1]  )
	    { 	    CarSpeed_SET_R=0 ;
	           CarSpeed_SET_L=0 ;
	           if(  (CCD_juli_zuo[0]+CCD_juli_you[0])/2 > CCD_juli_dm_she_ding_juli[1]+3000  )
	           {
	                    CCD_juli_zuo[0]=0 ;
	                    CCD_juli_you[0]=0 ;
	           }
	            //  break ;
	            Show_Data(   (CCD_juli_zuo[0]+CCD_juli_you[0])/2/100   ) ;
	    }
}
//======================= gui_yi¡ª¡ª ¹éÒ»³õÊ¼»¯ =======================//
void guiyi_chushihua( void )
{
	uint8_t  guiyi_xuhao_flage_1 = 0 , guiyi_xuhao_flage_2 = 0 , guiyi_xuhao_flage_3 = 0 ;
	uint8_t  guiyi_xuhao_flage_4 = 0 , guiyi_xuhao_flage_5 = 0 , guiyi_xuhao_flage_6 = 0 ; 
	uint8_t  guiyi_xuhao_flage_7 = 0 , guiyi_xuhao_flage_8 = 0 , guiyi_xuhao_flage_9 = 0 ; 
	uint8_t  guiyi_xuhao_flage_10 = 0 , guiyi_xuhao_flage_11 = 1 , guiyi_xuhao_flage_12 = 0 ; 

	while( guiyi_xuhao_flage_11 )
	{
	       Show_Me_Data(7,1,6) ;//ÓÃÓÚ×¼±¸    
	       // ====================================================  
	       if( diangan_biaoding_xuhao==9 ) //  1
		{
		        while( 1 )
		        {
		        	Show_Me_Data(1,1,6);//ÓÃÓÚ×¼±¸ 
		        	guiyi_MAX[ diangan_biaoding_xuhao -1 ] = diangan_guiyihua( diangan_biaoding_xuhao);  // ÕÒ³öË®Æ½1ºÅµç¸ÐµÄ×î´óÖµ   
                              Show_Data(guiyi_MAX[diangan_biaoding_xuhao-1]);
                              delay400ms();
                              if( diangan_biaoding_xuhao == 8)
                              {
                              	guiyi_xuhao_flage_11=0 ;  
                              	diangan_biaoding_xuhao = 0 ; 
                              	break ;
                              }
		        }              
		}
              //========================================================        
		if( diangan_biaoding_xuhao==7 ) //  1
		{
		        Show_Me_Data(1,1,6);//ÓÃÓÚ×¼±¸ 
			guiyi_MAX[ diangan_biaoding_xuhao -1 ] = diangan_guiyihua( diangan_biaoding_xuhao);  // ÕÒ³öË®Æ½1ºÅµç¸ÐµÄ×î´óÖµ   
                       Show_Data(guiyi_MAX[diangan_biaoding_xuhao-1]) ;
                   //   delay400ms();
                      guiyi_xuhao_flage_1 = 1 ;
                
                   	diangan_biaoding_xuhao=2 ;   //	Show_Me_Data(2,1,6);//ÓÃÓÚ×¼±¸ 
			guiyi_MAX[ diangan_biaoding_xuhao -1 ] = diangan_guiyihua( diangan_biaoding_xuhao);  // ÕÒ³öË®Æ½1ºÅµç¸ÐµÄ×î´óÖµ   
                      Show_Data(guiyi_MAX[diangan_biaoding_xuhao-1]) ;
                  //    delay400ms();
                      guiyi_xuhao_flage_2 = 1 ;
               
                       diangan_biaoding_xuhao=3 ; 	//Show_Me_Data(3,1,6);//ÓÃÓÚ×¼±¸
			guiyi_MAX[ diangan_biaoding_xuhao -1 ] = diangan_guiyihua( diangan_biaoding_xuhao);  // ÕÒ³öË®Æ½1ºÅµç¸ÐµÄ×î´óÖµ   
                      Show_Data(guiyi_MAX[diangan_biaoding_xuhao-1]);
               //       delay400ms();
                      guiyi_xuhao_flage_3 = 1 ;
		
		       diangan_biaoding_xuhao=4 ;	       //Show_Me_Data(4,1,6);//ÓÃÓÚ×¼±¸ 
			guiyi_MAX[ diangan_biaoding_xuhao -1 ] = diangan_guiyihua( diangan_biaoding_xuhao);  // ÕÒ³öË®Æ½1ºÅµç¸ÐµÄ×î´óÖµ   
                      Show_Data(guiyi_MAX[diangan_biaoding_xuhao-1]);
                      delay400ms();
                      guiyi_xuhao_flage_4 = 1 ;
		
		        Show_Me_Data(5,1,6);//ÓÃÓÚ×¼±¸ 
			guiyi_MAX[ diangan_biaoding_xuhao -1 ] = diangan_guiyihua( diangan_biaoding_xuhao);  // ÕÒ³öË®Æ½1ºÅµç¸ÐµÄ×î´óÖµ   
                      Show_Data(guiyi_MAX[diangan_biaoding_xuhao-1]);
                 //     delay400ms();
                      guiyi_xuhao_flage_5 = 1 ;
                      
                      diangan_biaoding_xuhao=6 ;     //   Show_Me_Data(6,1,6);//ÓÃÓÚ×¼±¸ 
			guiyi_MAX[ diangan_biaoding_xuhao -1 ] = diangan_guiyihua( diangan_biaoding_xuhao);  // ÕÒ³öË®Æ½1ºÅµç¸ÐµÄ×î´óÖµ   
                      Show_Data(guiyi_MAX[diangan_biaoding_xuhao-1]);
                      delay400ms();
                      guiyi_xuhao_flage_6 = 1 ;
		}
		if( guiyi_xuhao_flage_1 && guiyi_xuhao_flage_2 && guiyi_xuhao_flage_3 && guiyi_xuhao_flage_4 && guiyi_xuhao_flage_5 && guiyi_xuhao_flage_6 )
		{	guiyi_xuhao_flage_11=0 ; diangan_biaoding_xuhao = 0 ; 	}
	      //============================================================
		if( diangan_biaoding_xuhao == 6 )
		{
			guiyi_xuhao_flage_11=0 ; 	
		        	guiyi_MAX[0] = 200 ;//195  238
		        	guiyi_MAX[1] = 200 ;//183  220
			        guiyi_MAX[2] = 200 ;//183   225
		        	guiyi_MAX[3] = 200 ;//181   223
		        	guiyi_MAX[4] = 200 ;//179   220
		        	guiyi_MAX[5] = 200 ;//179   220
		        	diangan_biaoding_xuhao = 0 ; 
		}
		//============================================================

		if( diangan_biaoding_xuhao == 11 ) while(1){  array_one[0]    =  diangan_ADC(13) ; Show_Data(array_one[0] ) ;  delay500ms();	  }
		if( diangan_biaoding_xuhao == 12 ) while(1){  array_two[0]    =  diangan_ADC(3)   ; Show_Data(array_two[0] ) ;  delay500ms();   }
		if( diangan_biaoding_xuhao == 13 ) while(1){  array_three[0]  = diangan_ADC(2)    ; Show_Data(array_three[0] ) ;delay500ms();  }
		if( diangan_biaoding_xuhao == 14 ) while(1){  array_four[0]   = diangan_ADC(1)   ; Show_Data(array_four[0] ) ;  delay500ms();  }
		if( diangan_biaoding_xuhao == 15 ) while(1){  array_five[0]    = diangan_ADC(11)     ; Show_Data(array_five[0] ) ;   delay500ms();  }
		if( diangan_biaoding_xuhao == 16 ) while(1){  array_five[0]    = diangan_ADC(10)     ; Show_Data(array_five[0] ) ;   delay500ms();  }
	}
}




//========================================================================//
///////////////////////////////////Ö÷³ÌÐò/////////////////////////////////
//========================================================================//
void main (void)       
{
    core_config();
    initEMIOS();
    initSCI();                     
    initSTM();
    initADC(0x20000000);
    initPIT();
    PID_Init();
    disableIrq();
    init_max7219();
    Dianji_Init();
    CCD_IO_Init();
      //while(motor_time<10000);//ÑÓÊ±5s
      // motor_time=0;
      
      //==========µÆ=========//          
      SIU.PCR[PCR38_PC6].R =  0x0200;  //GPIO[38] is output
      SIU.GPDO[PCR38_PC6].R =1;//¸ø0²ÅÁÁ
//==============================±ê¶¨====================================//       
    

       	//===============  Ñ¡ÔñËÙ¶È ¡ª¡ª ¶¨ËÙ ¡¢²îËÙ ================= 
        yejingping_xuanze( ) ;   

                                    
//===================================================================//       
        SIU.GPDO[PCR38_PC6].R =1;//¸ø0²ÅÁÁ       
        last_CarSpeed_SET_R=CarSpeed_SET_R; //±£´æ³õÉèËÙ¶È
        last_CarSpeed_SET_L=CarSpeed_SET_L;
      
        Write7219(Shutdown_addr,0X00);       //¹Ø±ÕÊýÂë¹ÜÏÔÊ¾//Í£»ú
      //ÑÓÊ±Á½Ãë·¢³µ
      delay500ms();
      delay500ms();
      delay500ms();
      delay500ms();
      
      
      start_smartcar();
      motor_time=0;
      stop_smartcar_flag=0;
      
      duoji_zhongzhi_value=duoji_zhongzhi ;
       
      lost_time=0;
      stop_times=0 ;
      CCD_times=0 ;  
      CCD_juli_flag=0 ;
      CCD_juli_zuo[0]=0 ;
      CCD_juli_you[0]=0 ;
      CCD_juli_dm_she_ding_juli[0]=CCD_juli_dm_she_ding_juli[0]*100 ;  //  1dm  ¿´µ½ÕÏ°­ÎïÊ±Òª×ßµÄ¾àÀë
      CCD_juli_dm_she_ding_juli[1]=CCD_juli_dm_she_ding_juli[1]*100 ;  //  1dm  ÊÇ·ñ¶¨¾àÀëÍ£³µ  ×ßÒ»¶Î¾àÀëÍ£³µ£¬Áï³µ3mºó½Ó×Å×ß
      CCD_juli_dm_she_ding_juli[2]=CCD_juli_dm_she_ding_juli[2]*100 ;  //  1dm  17    µ½´ï¾àÀëºó   ¿ªÆô  CCD
      CCD_juli_dm_she_ding_juli[3]=CCD_juli_dm_she_ding_juli[3]*100 ;  //  1dm  18    µ½´ï¾àÀëºó   ¹Ø±Õ  CCD

      enableIrq();
      while(1)
      { 
             
               stop_smartcar_check();  
               Cai_yang();
               sai_dao_position();
               yu_chu_li();
               
               	if(  (CCD_juli_zuo[0]+CCD_juli_you[0])/2 > CCD_juli_dm_she_ding_juli[2] && (CCD_juli_zuo[0]+CCD_juli_you[0])/2 < CCD_juli_dm_she_ding_juli[3] )
                	      CCD_xuanze( ) ;
               
               
               PD_servo();
               zhuanxiang( );
               geng_xin_shuju();//  ¸üÐÂÊý¾Ý
               speed_control_new(); //ËÙ¶È¿ØÖÆ 
               protect();
        
               stop_smartcar();    
                 
                 
                 if( CCD_juli_dm_she_ding_juli[1]/100 > 27 )    // Ñ¡ÔñÊÇ·ñÐèÒªÌø³ö³ÌÐò     > 100 cm
                             ding_ju_tingche( ) ;
                 
               if(start_pid==1&&stop_smartcar_flag==0  )//
	           {
	             	
	                 if(flage_tiaosu_r==1)
			        {
			          	flage_tiaosu_r=0;
			         	pid_change_right();
			            motor_pwmR=PID_change_R(smartcar_speed_R ,CarSpeed_SET_R);    
			         }
	                 if(flage_tiaosu_l==1)
			         {
			         	flage_tiaosu_l=0;
			        	pid_change_left();
			            motor_pwmL=PID_change_L(smartcar_speed_L ,CarSpeed_SET_L  );
			           }
	       }
               //150   450
               //140   580
 
                    /*if( SCI0_RecvChar(&fashu))
		            { 
		                   
	                //SCI0_SendChar_16(array_current[0]);
 		             //SCI0_SendChar_16(array_current[1]);
 		            // SCI0_SendChar_16(array_current[2]);
 		              //SCI0_SendChar_16(array_current[3]);
		                 SCI0_SendChar_16(array_current[4]);
		                SCI0_SendChar_16(array_current[5]);
 	                     SCI0_SendChar_16(diangan_max[0]);
 	                 
 	                 //SCI0_SendChar_16(diangan_maxfour[0]);
 	                 //SCI0_SendChar_16(Verticaldiangan_sum[0]);
 	                 //SCI0_SendChar_16(xiezhi_diangan_sum[0]);
                      //SCI0_SendChar_16(Leveltaldiangan_sum[0]);
                    // SCI0_SendChar_16(chui_zhi_daingan[0]);
                     //SCI0_SendChar_16(total_diangan_sum[0]); 
  		             //SCI0_SendChar_16(xiediangan_max[0]);
  		                  SCI0_SendChar_16(shuiping_piancha[0]);
  	               	     SCI0_SendChar_16(chuizhi_piancha[0]); 
  	               	       SCI0_SendChar_16(xiezhi_piancha[0]);
  	               	 //SCI0_SendChar_16(zhijaio_point_k[0]);
  	               	 //SCI0_SendChar_16( (CCD_juli_zuo[0]+CCD_juli_you[0])/20) ;   
  		             //SCI0_SendChar_16(smartcar_speed_R-smartcar_speed_L)  ;
		             //SCI0_SendChar_16(smartcar_speed_L);
		             //SCI0_SendChar_16(smartcar_speed_R);
                     //SCI0_SendChar_16(motor_pwmL);
                     //SCI0_SendChar_16(motor_pwmR);          
		             //SCI0_SendChar_16(servo_Error[0]);
		             //SCI0_SendChar_16(servo_output );
                    
 		              } */    
 		          
 		     
           
         
         } 
           
          
               
     
     
}
 
 
//======================================================================//
/////////////////////////////µ÷ÊÔ¼ÇÂ¼////////////////////////////////////
//======================================================================//
//2.28-------±íÑÝ³É¹¦
//3.1--------32M²ÉËÙ³É¹¦£¬½øÒ»²½ÓÅ»¯´¦Àí
//3.3--------PIDµ÷ÊÔ
//3.9--------PID³É¹¦£¬ÏìÓ¦ºÜ¿ì£¬
//3.10-------¿ÉÅÜµÄ³ÌÐòµÄÇ¶ºÏ£¬·¢ÏÖ²ÉËÙÓë¶æ»úµÄÆµÂÊ³åÍ»
//3.15-------½â¾öÎÊÌâ£¬¸ÄÓÃ16M²ÉËÙ²¢¼ÓPIDµ÷ËÙ£¬×´¿öÁ¼ºÃ
//-----------ÊµÏÖ¿ÉÒÔ´øPIDµÄ¼òµ¥¿ÉÅÜ
//3.16-------³õ²½ÊµÏÖÂ·³Ì¼ÆËã»ý·Ö£¬×ªÏò¸ÄÎªÃ¿×ß2CMµ÷Õû
//3.17-------²îËÙÆ÷µÄ³õ²½ÊµÏÖ£¬ºóÐøÓÅ»¯
//3.18-------Ð±ÖÃµç¸ÐÓëË®Æ½µç¸ÐµÄÁªºÏÊ¹ÓÃ£¬µ½Ä¿Ç°ÎªÖ¹³ÌÐòµÄÈÚºÏ
//-----------ÊµÏÖ¿ÉÅÜ£¬×´Ì¬Á¼ºÃ
//3.19-------Õâ¶ÎÊ±¼äµÄµ÷ÊÔ×Ü½á£º
//1.¶ÀÁ¢µ÷ËÙÒÑ¾­±È½Ï³ÉÊì£¬µ«¶ÔÓÚÕûÌå»¹ÓÐ´ýÌá¸ß£¬¶øÕâÒ»¿éÓë¶æ»úºÍÐÅºÅ
//´¦ÀíÏ¢Ï¢Ïà¹Ø£¬ºóÆÚ¼ÓÇ¿ÐÅºÅµÄ´¦Àí£¬ÈçºÎ½«¼òµ¥µÄµç¸ÐÖµ×ª»¯³É¿ÉÒÔ°ÑÎÕ
//È«¾ÖµÄµÄÒ»ÕÅÍø£¬ÕâºÜÖØÒª.
//2.´Ó3.20¿ªÊ¼µ½3.29£¬×öÐÅÏ¢µÄ²É¼¯ÒÔ¼°´¦Àí¹¤×÷£¬²¢Óë¶æ»ú½áºÏÆðÀ´£¬
//ÑÐ¾¿Ëã·¨£¬Í¬Ê±×¢ÒâÍÅ¶ÓÐ­×÷£¬·Ö¹¤Ã÷È·£¬Òª´ïµ½µÄÄ¿µÄ³õ²½½¨Á¢ÕûÌå
//µÄÐÅÏ¢ÍøÂçÄ£ÐÍ
//3.µÚ¶þÁ¾³µÄ£µÄ´î½¨£¬·½°¸°´×îÔ¶Ç°Õ°À´¶¨
//3.21-------ÃÔÃ££¬²»ÖªµÀ¸ÃÈçºÎ´¦ÀíÊý¾Ý¡£¡£¡£½ø¶ÈÍ£ÖÍ²»Ç°£¡£¡£¡£¡£¡£¡£¡£¡
//3.22-------¼ÆËã×î´óµç¸ÐÎ»ÖÃ£¬´ÖÂÔÈ·¶¨´Åµ¼ÏßËùÔÚÏóÏÞ
//3.23Ïë·¨£ºÐ¡³µÃ¿×ßÒ»¶Î²ÉÑù£¬¾àÀëÊÇ¶àÉÙ£---5£¿10£¿¾àÀë»¹Òª¾«È·¼ÆËã2014
//3.24½ñÌìÍê³ÉÐ¡³µµÄ¸Ä×°£¬ÓÃ5¸öµç¸Ð£¬3¸öË®Æ½2¸ö´¹Ö±£¬ÏÂÒ»²½½øÐÐÆ«ÒÆÁ¿µÄ
//²âÊÔ.ÍíÉÏ¿´ÁËÍù½ìµÄÊÓÆµ£¬ÉîÓÐ´¥¶¯£¬ÎÒÃÇ¸ÃÈ«Á¦ÒÔ¸°ÁË£¡£¡£¡£¡£¡£¡£¡£¡
//3.25Ð¡³µÖÕÓÚ¿ÉÒÔÅÜÆðÀ´ÁË£¬Ö±µÀ¿ÉÒÔ¹ýÈ¥£¬µ«ÊÇ»¹Ðè½øÒ»²½ÍêÉÆ¡£
//3ÔÂµ×Ò»¶¨ÈÃ×ËÌ¬ºÃÆðÀ´,ÃÎÖ®Òí£¬¼ÓÓÍ£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡
//³öÏÖÎÊÌâ£ºÔÚÍäµÀÇÐ³öÈ¥£¬Æ«ÒÆÁ¿¿ÉÄÜÓÐÎÊÌâ£¬Ì«Ð¡£¬×ªÏò²»¹»
//3.27³öÍäµÄÇé¿öÉÙÁËºÜ¶à£¬µ«ÊÇ»¹Ã»ÓÐ×öµ½µÎË®²»Â©£¬Ê®×ÖÍä¿ÉÒÔÅÐ¶Ï³öÀ´
//»¹Òª½øÒ»²½ÌáÈ¡£¬ÓÃºÃÐÅÏ¢µÄÇ°ÌáÊÇÐÅÏ¢ÊÇ×¼µÄ£¬ËùÒÔÐÅÏ¢µÄÌáÈ¡ÈÔÈ»ÊÇÊ×Î»µÄ
//Í¬Ê±Ìá¸ß³ÌÐòµÄÖÊÁ¿£¬Èç¹ûÓÐÏë·¨¾ÍÒªÁ¢Âí¸¶ÖîÊµÊ©£¬Ò»¿ÌÒ²²»ÄÜÍÏÑÓ£¡£¡
//3.28½ñÌìÍêÕûµÄÅÜÍêÒ»È¦£¬ÉÏÆÂµÄÎÊÌâ½â¾öÁË£¬ÊÇÒòÎª²ÎÊý¸øÐ¡ÁË£¬ÒÔºó²»Òª
//°ÑÎÊÌâÁôµ½ºóÃæ£¬¼°Ê±½â¾ö£¬Ê±¼ä²»¶àÁË£¬ºóÆÚ×¢ÖØÍÅ¶ÓºÏ×÷£¬¼Ó¿ì½ø¶È
//Òª¿ìÒª¿ì.ÏÂÒ»²½×öºÃÑ­¼£(°üº¬Æ«²îºÍ×ªÏò),Í¬Ê±Ìá¸ß»ù´¡£¬Ìá¸ß¿ØÖÆ²ßÂÔ£¡£¡£¡£
//3.29-4.8ÕâÊ®ÌìÀ§ÈÅÁËÃÎÖ®ÒíÒ»¸öÎÊÌâ£¬Í¬ÑùµÄ³ÌÐòÔÚÁ½Á¾³µÉÏÓÐ²»ÓÃµÄÌåÏÖ£¬¾­¼ì²é
//·¢ÏÖ£ºÎÊÌâ³µ²ÉËÙ²»×¼£¬¿ÉÊÇÔÚÀÏÊ¦ÄÇÀïÓÐÃ»ÓÐÎÊÌâ£¬ºóÀ´»¹ÊÇÃ»ÓÐ²é³öÎÊÌâËùÔÚ
//ÓÉÓÚÊ±¼ä½ôÆÈ£¬ËùÒÔ»»ÁË±àÂëÆ÷£¬ÏÖÔÚ×´¿öÁ¼ºÃ£¬Í¬Ê±½â¾öÁËÈüµÀÅÜ·ÉµÄÎÊÌâ£¬Ô­ÒòÔÚÓÚ
//ÏÞ·ùÌ«´ó£¬²»ÄÜ¹»·´Ó³ËÙ¶ÈµÄÍ»±ä
//4.9ÉÏÎçºÍÏÂÎç·Ö±ðÓÐÂÞÓÀ¸ïÔº³¤ºÍ¹ùÒ»Ãù¸±Ôº³¤´øÁìÁìµ¼²Î¹Û£¬±íÑÝ³É¹¦£¬ÏÂÒ»²½¿ªÊ¼×öÂË²¨
//ºÍÈüµÀµÄÐÅÏ¢´¦Àí
//4.11½ñÌì²âÍêÈüµÀ×Ü³¤34.5m£¬È«³Ì×î¶ÌÓÃÊ±14.984£¬Æ½¾ùËÙ¶È2.302m/s
//4.15½ñÌì×öÍêÐÂÈüµÀ£¬È«³¤37.26m£¬×î¶ÌÓÃÊ±16.122s,Æ½¾ùËÙ¶È2.32£¬Ê®×ÖÍäÓÐÎÊÌâ
//ÔÚÀïÃæ³ö²»À´£¬ÃÎÖ®ÒíµÄ¡±ËÀÍöÊ®×Ö¡°£¬Í¨¹ý¼õÐ¡Ä£ºý¿ØÖÆµÄÊä³ö£¬×´¿öºÃÁËºÜ¶à
//ÏÂÒ»²½³¹µ×½â¾öÊ®×ÖµÀ£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡
//4.23ÖØÐÂÐ´ÈüµÀÊ¶±ð£¬Ð§¹ûºÃÁËºÜ¶à£¬ÅÐ¶ÏÎÈ¶¨ÁËºÜ¶à
//ÏÖÔÚÍêÉÆÂË²¨¡¢×ªÏò¡¢Ê®×ÖÍä£¬Ò»¶¨ÒªÊ¹³µµÄ×ËÌ¬ÎÈ¶¨£¡£
//5.3Íê³ÉÐÂËã·¨£¬Æ«²îÖØÐÂÈ·¶¨£¬PIDµ÷ËÙ½øÒ»²½¼ÓÇ¿
//ºóÐøÈüµÀÊ¶±ð£¬¿¹¸ÉÈÅ£¬ÎÈ¶¨+ÌáËÙ
//========================================================================//
////////////////////////////±¸Õ½»ªÄÏÈü//////////////////////////////////////
//7.6-7.7----------È·¶¨¼Ü·¨
//7.8-7.10---------Æ«ÒÆÁ¿ÒÔ¼°×ªÏò£¨PD£©
//7.11-12----±ÜÕÏ
//7.13-17----´óÁ¿µ÷ÊÔ
