//**************************************************************************//
//******************************2014��ų���*****************************//
//                        Copyright by ����&������   2014                             //
//*************************************************************************//
//*************************************************************************//

#include "MPC5605B.h" /* Use proper include file */
#include "MyHeader.h"
//===============================�ⲿ����=====================================//
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


 //==========================���õĺ���======================================//
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
void StartInte1(void) ; // �ع�
void ImageCapture1(uint8_t *Data1); //����
void SendImageData(uint8_t *ImageData); // ��λ��
void SendHex(uint8_t hex);//
void CCD_bizhang( void ) ;
//*********************************��������************************************//
#define TSL_SI1              SIU.GPDO[PCR63_PD15].R    //�������Դ������Ķ˿� SI  27
#define TSL_CLK1           SIU.GPDO[PCR31_PB15].R    //�������Դ������Ķ˿� CLK  7
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
uint32_t stop_times = 0 , shijian_ting_che=0 , shi_jian_ceshi_she_ding_shijian=0  ;     //PIT������
uint32_t CCD_times=0;
uint32_t CCD_stoptimes=0;
//uint32_t ccd_times=0; //   =1 ˵���Ѿ����ϰ�
uint32_t ccd_stoptimes=0;
uint32_t distance[10]={0};
uint32_t zhijiao_charge[5]={0};
uint32_t chuizhi_point[2]={0};
uint32_t xiezhi_point[2]={0};
uint32_t zhijaio_point_k[10]={0};
uint32_t CCD_juli_zuo[5]={0} , CCD_juli_you[5]={0} , CCD_juli[5]={0} , CCD_juli_dm_she_ding_juli[10]={0}  ;     //PIT������
uint8_t  CCD_juli_flag=0 , CCD_baoguang_shijian=10 , CCD_jiance_you_zhangai_flag=0 , podao_flag=0 , podao_flag_dingju=0 ;
//*********************************ת�����********************************//
int16_t Mid1[10]={0};
uint16_t variance1=0,variance2=0,variance=0;

uint8_t Fazhi_R11,Fazhi_R12,Fazhi_L11,Fazhi_L12;
uint8_t Max_pixel1,maxi1,Min_pixel1,mini1;
int16_t left11,right11,mid11,left12,right12,mid12,mid1=64,mid_x1;
uint8_t midcnt1=0  ; // 0 ���ϰ�     1 ��  2 ��
uint16_t exce_pixel1;
//********************************��ʱ***********************************//
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

//==========================  dian_gan  ==   ���       ===============================//
#define   DIAN    42     //  ��ʼ����    һ������һ�вɼ��ĵ�
#define   guiyi_DIAN    80      //   ��һ����  һ����вɵ�ֵ
uint8_t flag_2;
int16_t array_one[50]={0};    // 1   �����вɻ�����ֵ  
int16_t array_two[50]={0};    // 2
int16_t array_three[50]={0};  // 3
int16_t array_four[50]={0};   //  4
int16_t array_five[50]={0};    //  5  
int16_t array_six[50]={0};    //  5 
int16_t array_seven[50]={0};    //  5                 
int16_t array_current[8]={0};//  �洢��е�ֵ���ڼ���
int16_t guiyi_wugediangan_MAX = 0 ;
int16_t speed_Fuzzy_tiaosu_flag = 0 ;
uint8_t array_lvbo_flag[6][8] = { { 0 } , { 0 } , { 0 } , { 0 } , { 0 } , { 0 } } ; //�˲���־λ  ����array_lvbo_flag[7] ���� ȷ�����ĸ����  
                                                           //                 ���� array_lvbo_flag[7][2] ����ѡ���޷��Ĵ�С

//=================  pian_cha___xinhao  ==   ƫ����ź�        ===============================//
#define   Error_lishudu_Max   200      // ģ�������С���ƫ������������ֵ   ==> ������  0---200
#define   Error_c_lishudu_Max   200  //  ģ�������С���ƫ��仯�ʡ������������ֵ
uint8_t  diangan_biaoding_xuhao = 1 ;  // ��һ�� �궨���ʱ �� ѡ��궨�ڼ������
uint16_t guiyi_MAX[8] ={ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0};  // ��һ���궨��ÿ����е����ֵ
int16_t servo_Error[100] = {0} ;  //  �洢ÿ���ù��� ƫ��ֵ
int16_t  servo_aaaaa[2];
int16_t servo_Error_c[50] = {0} ; // �洢ÿ���ù���ƫ��仯�ʵ�ֵ
uint8_t servo_Error_flag[10]={    0 , 0 , 0 , 0 , 0  };  // �ϴδ�ֱ��е�ƫ��
uint8_t servo_output_flag[10]={    0 , 0 , 0 , 0 , 0  };  // �ϴδ�ֱ��е�ƫ��
                    
int16_t abs_sum_[10]={0}; // �洢  ʮ��ƫ��  ��  ����ֵ  ��   ��
int16_t abs_sum=0;   //����ֵ��   
int16_t daishu_sum=0;//������
uint16_t diangan_min[20]={0} ;
int16_t  middle_piancha[20]={0} ;
uint16_t diangan_maxfour[30]={0} ;  // ��ȡ����ˮƽ����е����ֵuint8_t lost_signal_flag=0;
uint16_t diangan_max[30]={0} ;  // ��ȡ����ˮƽ����е����ֵ
uint16_t chuizhidiangan_max[20]={0} ;
uint16_t  total_diangan_sum[20]={  0 , 0 , 0 , 0 , 0  }  ;  
uint16_t  Verticaldiangan_sum[20]={  0 , 0 , 0 , 0 , 0  }  ;  
uint16_t  Leveltaldiangan_sum[20]={  0 , 0 , 0 , 0 , 0  }  ;
uint16_t  xiezhi_diangan_sum[20]={  0 , 0 , 0 , 0 , 0  }  ;  
 
uint8_t dian_gan_cixu = 0 ; // �ж�����ˮƽ������ĸ���е�ֵ���
int16_t servo_output_last_max = 0 ; //  �ҳ��ϴζ����������ֵ
uint8_t cixu[10][3]={0};  // �������е�ֵ��������
int16_t wandao_sppch[12]={0};
int16_t shuiping_piancha[12] = {    0 , 0 , 0 , 0 , 0  };   // ÿ�β����������������ˮƽ��е�  ƫ��
uint8_t shuiping_piancha_flag[10] = {    0 , 0 , 0 , 0 , 0  };  //  �ϴ�ˮƽ��е�ƫ�� 
int16_t xiezhi_piancha[12] = {    0 , 0 , 0 , 0 , 0  };
int16_t chuizhi_piancha_fs[12]= {    0 , 0 , 0 , 0 , 0  };
int16_t chuizhi_piancha[12]= {    0 , 0 , 0 , 0 , 0  };  // ÿ�β���������������Ĵ�ֱ��е�  ƫ��
uint8_t chuizhi_piancha_flag[10]={    0 , 0 , 0 , 0 , 0  };  // �ϴδ�ֱ��е�ƫ��
int16_t chuizhi_jiaquan[12];
int16_t shuiping_piancha_xishu=0 , chuizhi_piancha_xishu=0 , xiezhi_piancha_xishu=0 ;
uint8_t piancha_E_qiujie_flag=7 ;
//=======================servo ���� mohu�����������===============================//
#define   servo_Upper     830   // ������ֵ  ��ƫ��
#define   servo_Lower    -830   // ������ֵ  ��ƫ��
uint16_t  duoji_zhongzhi =6420 ,  duoji_zhongzhi_value =6420;// 6915 //      
uint16_t  servo_kp = 15 ;   // ��� PD �����е� P ֵ
uint16_t  servo_kd = 0 ;     // ��� PD �����е� D ֵ
uint16_t  Fuzzy_Kp_cunchu[7] ={ 0,0,0,0,0,0,0 } ;  // �洢ģ�������� Kp ��������е�ֵ�����洢��һ�ε�ֵ
int32_t  Fuzzy_Kd_cunchu[7] ={ 0,0,0,0,0,0,0 } ;  // �洢ģ�������� Kd ��������е�ֵ�����洢��һ�ε�ֵ
int16_t servo_Fuzzy_Error[7] = { -87 ,  -65 , -27 ,  0 , 27 ,  65 , 87 } ; // ƫ�����     Ŀǰ������ͷС�м��Ч���Ϻ� //   ֱ������  kp 80 , 69 , 42 , 0 , 42 , 62 , 80 
//int16_t servo_Fuzzy_Error[7] = { -87 ,  -50 , -25 ,  0 , 25 ,  50 ,  87 } ; // ƫ�����     Ŀǰ������ͷС�м��Ч���Ϻ� //   ֱ������  kp 80 , 69 , 42 , 0 , 42 , 62 , 80 

 //int8_t servo_Fuzzy_Error_c[7] = { -9 , -6 , -3 , 0 , 3 , 6 , 9 } ;     // ƫ��仯��   ����
int8_t servo_Fuzzy_Error_c[7] = { -15 , -11 , -7 , 0 , 7 , 11 , 15 } ;     // ƫ��仯��   ����
                                        //  -87 ,  -51 , -20 ,  0 , 20 ,  51 ,  87  // 110 , 95 , 80 , 40 , 80 , 95 , 110  // ����
                                                             //  -90 ,  -59 , -27 ,  0 , 27 ,  59 ,  90    //120 , 100 , 55 , 30 , 55 , 100 , 120  ����е���
const uint16_t servo_Fuzzy_Kp_dan[7] = { 75 , 65 ,  40 , 0 , 40,65 , 75} ;  // Kp�����С   70 , 50 , 20 , 10 , 20 , 50 , 70
//76 , 63 ,  45 , 0 , 45,63 , 76
/*const uint8_t servo_Fuzzy_Kp_rule[7][7] =       // Kp �����
{///  0--1--2--3--4--5--6                                                                                                                                //
	6 , 6 , 5 , 5 , 4 , 4 , 3 ,//0     //
	6 , 6 , 5 , 5 , 4 , 3 , 3 ,//1     //
	5 , 5 , 5 , 4 , 3 , 2 , 1 ,//2     //
	5 , 5 , 4 , 3 , 2 , 1 , 1 ,//3     //
	5 , 4 , 3 , 2 , 1 , 1 , 1 ,//4     //
	3 , 3 , 2 , 1 , 1 , 0 , 0 ,//5     //
	3 , 2 , 2 , 1 , 1 , 0 , 0 //6      //
};*/

const uint8_t servo_Fuzzy_Kp_rule[7][7] =       // Kp �����
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
const uint8_t servo_Fuzzy_Kp_rule[7][7] =       // Kp �����
{///  0--1--2--3--4--5--6                                                                                                                                //
	6 , 6 , 5 , 5 , 4 , 4 , 3 ,//0     //
	6 , 6 , 5 , 5 , 4 , 3 , 3 ,//1     //
	5 , 5 , 5 , 4 , 3 , 2 , 1 ,//2     //
	5 , 4 , 4 , 3 , 2 , 1 , 1 ,//3     //
	4 , 4 , 3 , 2 , 2 , 1 , 1 ,//4     //
	3 , 3 , 2 , 1 , 1 , 1 , 0 ,//5     //
	3 , 2 , 2 , 1 , 1 , 0 , 0 //6      //
};*/




 //   const uint16_t servo_Fuzzy_Kd_dan[7] = { 1 , 1 , 1 , 1 , 1 , 1 , 1 } ;    // Kd �����С
 //  const uint32_t servo_Fuzzy_Kd_dan[7] = { 5000 , 4000 , 3000 , 1000 , 3000 , 4000 , 5000 } ;
   const uint16_t servo_Fuzzy_Kd_dan[7] = { 90 , 70 , 30 , 10 , 30 , 70 , 90 } ;

 //   const uint16_t servo_Fuzzy_Kd_dan[7] = { 2900 , 2500 , 2300 , 2000 , 2300 , 2500 , 2900 } ;
/*const uint8_t servo_Fuzzy_Kd_rule[7][7] =      //  Kd  �����
{ //   0--1--2--3--4--5--6
	4 , 4 , 3 , 3 , 3 , 6 , 6 ,  //  0
       2 , 2 , 3 , 4 , 4 , 4 , 5 ,  //  1
       6 , 6 , 1 , 2 , 3 , 4 , 5 ,  //  2
       0 , 1 , 2 , 3 , 4 , 5 , 6 ,  //  3   
       1 , 2 , 3 , 4 , 5 , 0 , 0 ,  //  4
       1 , 2 , 2 , 2 , 3 , 4 , 4 ,  //  5
       4 , 4 , 3 , 3 , 3 , 6 , 6    //  6
}; */


const uint8_t servo_Fuzzy_Kd_rule[7][7] =      //  Kd  �����
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
const uint8_t servo_Fuzzy_Kd_rule[7][7] =      //  Kd  �����
{ //   0--1--2--3--4--5--6
	4 , 4 , 3 , 3 , 3 , 6 , 6 ,  //  0
       2 , 2 , 2 , 2 , 3 , 2 , 5 ,  //  1
       6 , 6 , 1 , 2 , 3 , 4 , 5 ,  //  2
       0 , 1 , 1 , 2 , 3 , 4 , 5 ,  //  3
       0 , 1 , 2 , 2 , 3 , 4 , 4 ,  //  4
       1 , 2 , 2 , 2 , 3 , 4 , 4 ,  //  5
       4 , 3 , 3 , 3 , 3 , 6 , 6    //  6
}; */

uint32_t  Fuzzy_speed_cunchu_zuo[7] ={ 0,0,0,0,0,0,0 } ;  // �洢ģ�������� Kp ��������е�ֵ�����洢��һ�ε�ֵ
uint32_t  Fuzzy_speed_cunchu_you[7] ={ 0,0,0,0,0,0,0 } ;  // �洢ģ�������� Kd ��������е�ֵ�����洢��һ�ε�ֵ
                                                                 //   -3      -2       -1        0         1       2          3   
const uint16_t speed_Fuzzy_zuo_dan[7] = { 2350 , 2250 , 2100 , 2300 , 2100 , 1950 , 1800 } ; // 
const uint16_t speed_Fuzzy_you_dan[7] = { 1800 , 1950 , 2100 , 2300 , 2100 , 2250 , 2350 } ; 

//const uint16_t speed_Fuzzy_zuo_dan[7] = { 2450 , 2350 , 2200 , 2350 , 2150 , 2050 , 1900 } ; // 
//const uint16_t speed_Fuzzy_you_dan[7] = { 1900 , 2050 , 2150 , 2350 , 2200 , 2350 , 2450 } ; 

//const uint16_t speed_Fuzzy_zuo_dan[7] = { 2300 , 2250 , 2100 , 2200 , 2100 , 1950 , 1800 } ; // ��
//const uint16_t speed_Fuzzy_you_dan[7] = { 1800 , 1950 , 2100 , 2200 , 2100 , 2250 , 2300 } ; 

//const uint16_t speed_Fuzzy_zuo_dan[7] = { 2550 , 2520 , 2500 , 2800 , 2400 , 2250 , 2150 } ; // 37.26m----15.719s----2.370m/s
//const uint16_t speed_Fuzzy_you_dan[7] = { 2150 , 2250 , 2400 , 2800 , 2500 , 2520 , 2550 } ; 

//const uint16_t speed_Fuzzy_zuo_dan[7] =  { 2650 , 2620 , 2600 , 2800 , 2500 , 2350 , 2250 } ; // 37.26m----15.23s----m/s
//const uint16_t speed_Fuzzy_you_dan[7] =  { 2250 , 2350 , 2500 , 2800 , 2600 , 2620 , 2650 } ; 

//const uint16_t speed_Fuzzy_zuo_dan[7] =  { 2750 , 2720 , 2700 , 3200 , 2600 , 2450 , 2350 } ; // 14.78s--15.2m/s
//const uint16_t speed_Fuzzy_you_dan[7] =  { 2350 , 2450 , 2600 , 3200 , 2700 , 2720 , 2750 } ; 

uint16_t  speed_temp_11[4] = { 0,0,0,0 } , speed_temp_12[4] = { 0,0,0,0 } , speed_temp_21[4] = { 0,0,0,0 } , speed_temp_22[4] = { 0,0,0,0 } ;
 
//======================================================================//
//==========================ȫ�ֱ���======================================//
int32_t  i_quan , j_quan , k_quan ;
uint8_t xianshi_flag = 0 ;     //ѡ����ʾ�����ݱ�־λ
uint32_t motor_time = 0 ;     //PIT������
 
//====================================================================//
//======================== �㷨 ������־λ  =================================== //
uint8_t po_dao_flag = 0 , daoda_podao_flag = 0 ;  //�µ�
uint8_t zhi_jiao_flag =0;  // �ж��Ƿ񵽴�  ֱ�Ǵ�
uint8_t zhi_jiao_charge =0;
uint8_t zhi_jiao_check=0;
int8_t  zhijiao_zhuanxiang = 0 ; //   ֱ��ת��
uint8_t ru_shi_zi_flag = 0 ;  //   ����ʮ��
uint8_t chu_shi_zi_flag = 0 ; // �뿪ʮ��
uint8_t zhi_dao_flag = 0 ;  // ����ֱ��   
uint8_t little_S_flag = 0 ;  // ����С  S
uint8_t bigbig_S_flag = 0 ; // �����  S 

int8_t K_one[50] ;  // 1  б��
int8_t K_two[50] ;  // 2 
int8_t K_three[50] ;// 3
int8_t K_four[50] ; // 4
int8_t K_five[50] ; //  5
//wandao_come_in =1 ����� wandao_come_in =2  �ҽ���
uint8_t  wandao_come_in =0;

//wandao_go_out =1 ����� wandao_go_out =2  �ҳ��� 
uint8_t  wandao_go_out = 0 ;  

uint8_t pian_yi_zuo_flag = 0 ; // 
uint8_t pian_yi_zhong_flag = 0 ; // 
uint8_t pian_yi_you_flag = 0 ; //
uint8_t shangpo_flag = 0 ; // ����
uint32_t shangpo_flag_distance;
uint32_t lost_time = 0 ; //���źŴ���
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
#define   motor_pwmR    EMIOS_0.CH[13].CBDR.R //�ұ�  Maximal value is 8000
#define   motor_pwmL    EMIOS_0.CH[14].CBDR.R //���  Maximal value is 8000
//#define   servo_pwm     EMIOS_0.CH[21].CBDR.R     //Maximal value is 53333

#define   servo_pwm     EMIOS_0.CH[22].CBDR.R     //Maximal value is 53333


//============================���ٱ���=================================//

uint32_t smartcar_speed_L ; // �����ٶ�
uint32_t smartcar_speed_R ; //  �ҵ���ٶ�
uint8_t fashu;
uint32_t per_pulse_R=0;//����
uint32_t per_pulse_L=0;
uint32_t average_distance;
//�ұ�
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
//���
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
//============================ת�����=================================//
uint32_t per_distance_R=0;//����
uint32_t per_distance_L=0;

uint32_t sum_distance_R=0;
uint32_t sum_distance_L=0;

uint32_t average_distance_L=0;
uint8_t zhuanxaing;
uint8_t protect_flag;

int16_t  speed_difference;
uint16_t  speed_difference_flag;

 int32_t servo_output;
 int32_t servo_output_last[20]={0} ;  //  �洢�ϼ���  servo_output  �����
 int32_t cha_su_value;
int16_t Duoji_value;

int16_t start_pid=0;
int16_t pid_times[5]={0} ;
int16_t pid_flag[5]={0} ;
int16_t start_pid_time;
int16_t stop_flag=0;
int16_t smart_speed;
//**************************   ���  PID  �ṹ��  ����  ********************************//
struct motor_PID 
{
          int32_t  Proportion ;   // ��������  
          int32_t  Integral ;   // ���ֳ��� 
          int32_t  Derivative ;   // ΢�ֳ���
           
          int32_t  SumError;         //����ۼ� 
                   // �趨Ŀ�� Desired Value 
          int32_t  iError ;   // ƫ��
          int32_t  iIncpid ;  // ����             
          int32_t  output ; // ������ֵ
          int32_t  last_output ; // ����ϴ����ֵ
          int32_t  LastError;           //Error[-1] 
          int32_t  PrevError;           //Error[-2]

          int32_t  Max_iError ; //����iError��������
          int32_t  Min_iError ;
          int32_t  Max_iIncpid ;  //��������
          int32_t  Min_iIncpid ;
          int32_t  Max_otput ;  //���������
          int32_t  Min_otput ;
             
//=============  ����  ============//        
          int32_t  SumError_other;         //����ۼ� 
          int32_t  iError_other; 
          int32_t  iIncpid_other;             
          int32_t  output_other;
          int32_t  last_output_other;
          int32_t  LastError_other;           //Error[-1] 
          int32_t  PrevError_other;           //Error[-2]

} PID;



//=====================================================================//
////////////////////////////����ܳ�ʼ��///////////////////////////////
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
#define                             DIG_1                0x01        // ���������1 register 
#define                             DIG_2                0x02        // ���������2 register 
#define                             DIG_3                0x03        // ���������3 register 
#define                             DIG_4                0x04        // ���������4 register 
#define                             DIG_5                0x05        // ���������5 register 
#define                             DIG_6                0x06        // ���������6 register 
#define                             DIG_7                0x07        // ���������7 register 
#define                             DIG_8                0x08        // ���������8 register 
/*---------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------*/
/*------------------------------------------------------------------------*/
                         /*�����*/
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
  
  Write7219(Shutdown_addr,0X00);       //ͣ��
  Write7219(Decode_Mode_addr,0XFF);    //����ģʽ
  Write7219(Intensity_addr,0X00);     //����
  Write7219(Scan_Limit_addr,0X07);        //��ʾλ��
  Write7219(Shutdown_addr,0X01);         //������ʾģʽ
  Write7219(Display_Test_addr,0X00);     //����
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
      n = k/1000 - k /10000 *10  ;   //���ǧλ
      m = k /100 - k /1000 *10    ;   //��ð�λ
      x = k /10 - k /100 *10      ;   //��ø�λ
      y = k % 10                     ;   //��ø�λ
           Write7219(DIG_4,y);
            Write7219(DIG_3,x);
            Write7219(DIG_2,m);
            Write7219(DIG_1,n);   
     }
void Show_Data(uint16_t i)
{
	uint16_t  k,y ,x ,m , n;
	k=i;
    n = k/1000 - k /10000 *10  ;   //���ǧλ
    m = k /100 - k /1000 *10    ;   //��ð�λ
    x = k /10 - k /100 *10      ;   //��ø�λ
    y = k % 10                   ;   //��ø�λ
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
	   
       if (decrease_port_in == low)  //����
           number_test-=change;
       
       if (increase_port_in == low)  // ����
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
            case   13://    �Ƿ����ϰ� 
   	                  CCD_xuanze_flag = number_test;    
   	        break;   	        
//===============================================// 
            case   14://    �Ƿ����ϰ� 
   	                  duoji_zhongzhi = number_test;    
   	        break;   	      
//===============================================// 
             case   15://    �Ƿ����ϰ� 
   	                  CCD_juli_dm_she_ding_juli[1] = number_test;    
   	        break;   	 
//===============================================// 
            case   16://    �������
   	                  CCD_juli_dm_she_ding_juli[0] = number_test;    
   	        break;   	   	              
//===============================================// 
             case   17://    ����������  CCD
   	                  CCD_juli_dm_she_ding_juli[2] = number_test;    
   	        break;   	 
//===============================================// 
            case   18://    ��������ر�  CCD
   	                  CCD_juli_dm_she_ding_juli[3] = number_test;    
   	        break;   	   	              
//===============================================// 
            case   19://    �������
   	                  CCD_baoguang_shijian = number_test;    
   	        break;
//===============================================// 
            case   20://    ��������ر�  CCD
   	                  piancha_E_jisuan = number_test;    
   	        break;   	   	              
//===============================================// 
 
   	         case   22://    �������
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
//================ ��С���˷� ��� ================= //
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

//==============================  ����������֮��ľ���ֵ ============================//
uint16_t ABS(int16_t x,int16_t y)
{
	if(x>y)         return ( x-y ) ;
	else   	    return ( y-x ) ;
}
//=================  �ж�������֮���  ����  ================//
int16_t fu_hao(int16_t xx)
{
      if(xx>0)	    return(1) ;
      if(xx<0)      return(-1) ;
}
//================  ������ȡ��  ====================//
int16_t qu_zheng(int16_t xxx)
{
	if(  xxx >= 0  )    return(xxx) ;
	else                   return(-xxx) ;
}
 
//**********************************  STM ���� ��ʱ  *************************//
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




//==================================== �жϡ�����ʱ ===================================//
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



 

//==========================�����ж�===================================//

void PIT_CH1_isr(void)
{

    motor_time++;
    PIT.CH[1].TFLG.B.TIF = 1;
 	
	if(motor_time==CCD_baoguang_shijian+CCD_stoptimes)
	          CCD_times=1 ;//CCD�ع�ʱ��         
 
    
    if(motor_time==start_pid_time) 
      start_pid=1;        //��P��ʼPID����
    
    
    
   // if(motor_time>1500) stop_smartcar_flag=1;  
    

}
//=============================�������ж�=============================//
//================================���================================//
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
    
    L_period_sum = L_period_sum+L_count;//�����ۼ�

    if(L_period_sum_times==L_smartcar_speed_time)
    {
        L_period = L_period_sum/L_period_sum_times;
        //������26T,���9T,����24T,ͬ����ִ��28TС��10T
    	smartcar_speed_L =43*157*200000/L_period/84;
    	// 10*25*3142*100/L_period/7; 
       ////43*157*100000/L_period/84 ������� 
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
//===============================�ұ�=================================//
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
    
    R_period_sum = R_period_sum+R_count;//�����ۼ�
    
    if(R_period_sum_times==R_smartcar_speed_time)
    {
        R_period = R_period_sum/R_period_sum_times;
        //������26T,���9T,����24T,ͬ����ִ��28TС��10T
    	smartcar_speed_R = 43*157*200000/R_period/84 ; 
         //32M=>100��8M=>25
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
//=============================�����ʼ��==================================//
void Dianji_Init( void )
{
    SIU.PCR[PCR40_PC8].R = 0x0200;  //GPIO[40] is output
    SIU.PCR[PCR44_PC12].R = 0x0200; //GPIO[44] is output 
    SIU.PCR[PCR41_PC9].R = 0x0200;  //GPIO[41] is output
    SIU.PCR[PCR47_PC15].R = 0x0200; //GPIO[47] is output  
        
    SIU.GPDO[PCR40_PC8].R = 0 ;       //0-1��ת
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
			          SIU.GPDO[PCR40_PC8].R = 1 ;       //1-0��ת
                      SIU.GPDO[PCR44_PC12].R = 0 ;
                      motor_pwmR=7600;
                      SIU.GPDO[PCR41_PC9].R = 1;
                      SIU.GPDO[PCR47_PC15].R = 0;
	                  motor_pwmL=7600;
 			      }
 			      
 			      else if(smart_speed>600)
 			      {
			          SIU.GPDO[PCR40_PC8].R = 1 ;       //1-0��ת
                      SIU.GPDO[PCR44_PC12].R = 0 ;
                      motor_pwmR= 3500;
                      
                      SIU.GPDO[PCR41_PC9].R = 1;
                      SIU.GPDO[PCR47_PC15].R = 0;
                      motor_pwmL= 3500;
 			      }
 			       
 			      else 
 			      {
			          SIU.GPDO[PCR40_PC8].R = 1 ;       //1-0��ת
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
     }//����Ƿ�Ӧ��ͣ��
    if(motor_time<5000&&stop_smartcar_flag==1)
          stop_smartcar_flag=0;
   
}
//=========================PID��ʼ��=============================//
void PID_Init() 
{
            PID.SumError=0;          //����ۼ�
           
            PID.Proportion=5600;        
            PID.Integral= 600;         
            PID.Derivative=50 ;       
            
            PID.iError=0;  
            PID.iIncpid=0;           
            
            PID.output=0;            
            PID.last_output=0;            

            PID.LastError=0;         //Error[-1]
            PID.PrevError=0;  

            PID.Max_iError=3500;      //����iError��������
            PID.Min_iError=-3500;
            
            PID.Max_iIncpid=8000;     //��������
            PID.Min_iIncpid=-8000;
            
            PID.Max_otput=7600;         //���������
            PID.Min_otput=-7600;

         //=========================//
             PID.SumError_other=0;         //����ۼ� 
            
             PID.iError_other=0; 
             PID.iIncpid_other=0;             
             PID.output_other=0;
             PID.last_output_other=0;

             PID.LastError_other=0;           //Error[-1] 
             PID.PrevError_other=0;           //Error[-2]
}
 
//======================================================================//
//============================PID����====================================//
int PID_change_L(int Current_Speed ,int SetPoint )
{ 
       //��ǰ��� 
       PID.iError =  SetPoint - Current_Speed ;  //�������� 
        
       //************��������**************//     
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
       
        //�洢�������´μ��� 
        PID.PrevError =  PID.LastError ; 
        PID.LastError =  PID.iError ; 
       
        //****** ********�����ж�*** ************************//            
      if( PID.iIncpid>= PID.Max_iIncpid)
      {     
         PID.iIncpid = PID.Max_iIncpid ; 
      }    //��������  
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
       }  //�������
       zuo_shu_chu=PID.output ;
       PID.output=motor_judge_left( PID.output ) ; //�Ƿ����ж�
        
	   PID.last_output = PID.output ;     
       return PID.output ;
       
           
		          
  }    
      
//=======================  ����  ==========================//          
int PID_change_R(int Current_Speed ,int SetPoint )          
 {
         //��ǰ��� 
         PID.iError_other = SetPoint - Current_Speed;  //�������� 
        
           //*********��������**************/     
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
       
        //�洢�������´μ��� 
        PID.PrevError_other =  PID.LastError_other ; 
        PID.LastError_other =  PID.iError_other ; 
       
          //*************�����ж�************//           
      if( PID.iIncpid_other >= PID.Max_iIncpid )
      {  
          PID.iIncpid_other = PID.Max_iIncpid ;
       }      //��������
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
       }    //�������
          you_shu_chu=PID.output_other;
	    PID.output_other = motor_judge_right( PID.output_other ) ;
	    PID.last_output_other = PID.output_other ;   
        return PID.output_other ;
}
//================  �����жϣ��Ƿ��ϣ�=====================//
int motor_judge_right(int right)
{
   	if(right>0)
   	{  	    
              SIU.PCR[PCR40_PC8].R = 0x0200 ;  //GPIO[40] is output
              SIU.PCR[PCR44_PC12].R = 0x0200 ; //GPIO[44] is output 

              SIU.GPDO[PCR40_PC8].R = 0 ;       //0-1��ת
              SIU.GPDO[PCR44_PC12].R =1 ;
              return right;
   	}	
 //========================== //  	
   else	if(right<0)
   	{	 
              SIU.PCR[PCR40_PC8].R = 0x0200 ;  //GPIO[40] is output
              SIU.PCR[PCR44_PC12].R = 0x0200 ; //GPIO[44] is output 
        
              SIU.GPDO[PCR40_PC8].R = 1 ;       //1-0��ת
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
         
              SIU.GPDO[PCR40_PC8].R = 0 ;       //0-1��ת
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
/********************************************�ع�**************************************************/
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
  //********************************CCD_IO�ڳ�ʼ��*****************************************//
void CCD_IO_Init(void) 
{
  	  TSL_CLK_DDR1 =0x0203;
	  TSL_SI_DDR1  =0x0203;
  	  TSL_CLK1 = 0;
  	  TSL_SI1  = 0;
  
}
//************************************��λ������*******************************************//
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
	//===================   �ҵ����ֵ     ==================================	
	CCD_max[0]=( CCD_sample[23]+CCD_sample[24]+CCD_sample[101]+CCD_sample[102] ) / 4 ;
	CCD_max[1]=( CCD_sample[57]+CCD_sample[58]+CCD_sample[59]+CCD_sample[60] ) / 4 ;
	CCD_max[2]=( CCD_sample[64]+CCD_sample[65]+CCD_sample[66]+CCD_sample[67] ) / 4 ;
	if( CCD_max[1]<CCD_max[2] )  CCD_max[1]=CCD_max[2] ;
	// ==============      min   =========================//
	CCD_min=( CCD_sample[30]+CCD_sample[31]+CCD_sample[32]+CCD_sample[33]+
	                    CCD_sample[92]+CCD_sample[93]+CCD_sample[94]+CCD_sample[95] ) / 8 ;
	 CCD_max[3]=CCD_max[0] ;
	 CCD_max[4]=( CCD_max[0]-CCD_min )/4+CCD_min ;  // ��ֵ 
	 
	 if( CCD_max[1]<CCD_max[0] )   CCD_max[0]=CCD_max[1] ; 
	 if( CCD_max[0]<CCD_min )    CCD_max[0]=CCD_min ;

	 CCD_yuzhi=( CCD_max[0]-CCD_min )/3 ;  
	 CCD_value= CCD_max[0] - CCD_yuzhi ;
	//=====================  ����Ĳ�ֵ  ===========================
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
	
	//=====================  �����ֵ  ===========================
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
             
         //=============================���ϴ���==========================//
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
		      		if( CCD_lefttimes[1]>4 && CCD_lefttimes[2]>4 ) // ȷ������ͻ���
		      			CCD_bizhang_flag=1 ;//����ϰ�
		      }
		      else if( (CCD_righttimes[0]-CCD_lefttimes[0])>4 && CCD_righttimes[0]>5 && CCD_lefttimes[0]<3 )
		      {
		      		if( CCD_righttimes[1]>4 && CCD_righttimes[2]>4 ) // ȷ������ͻ���
		      			CCD_bizhang_flag=2 ;//�ұ��ϰ�
		      }  
            }
}



//===========================����CCD����==================================//
void CCD_bizhang( void )
{
                 if(CCD_times==1)
		  {
				    CCD_times=0 ;
				    CCD_stoptimes=motor_time ;
				    ImageCapture1(CCD_sample) ;//���� 
				    StartInte1( ) ;//�ع�
				    CCD_tiaobian_erzhihua( ) ; //��ֵ�� ���ж��Ƿ����ϰ�
				   
				    if( CCD_bizhang_flag==1 || CCD_bizhang_flag==2 )
				    {
						SIU.GPDO[PCR38_PC6].R =0 ;//��0����
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
				                    
				             //      SIU.GPDO[PCR38_PC6].R =!SIU.GPDO[PCR38_PC6].R ;//��0����
				           }
	 */	/*	*/
		  }
}




//13===3====2===1
//         11===10
//          8====9
//========================================================================//
uint16_t  diangan_ADC(uint8_t tongdao) // �ɼ���е�ֵ
{
    uint16_t i=0 , j=0 , temp=0 ;
    uint16_t a[DIAN]={0 };
    uint16_t ADC_tongdao ;
    uint32_t average=0 ;
    
    if(tongdao == 13)     ///һ��  1////
        ADC_tongdao = 0x00002000;
    else if(tongdao == 3)       ///һ��  2////
        ADC_tongdao = 0x00000008;
    
    else if(tongdao == 2)       ///һ��  3////
        ADC_tongdao = 0x00000004;
    else if(tongdao == 1)       ///һ��  4////
        ADC_tongdao = 0x00000002; 
    
    else if(tongdao == 11)     ///����  1////
        ADC_tongdao = 0x00000800;
    else if(tongdao == 10)       ///����  2////
        ADC_tongdao = 0x00000400;
    
    else if(tongdao == 8)       ///����  1////
        ADC_tongdao = 0x00000100;
    else if(tongdao == 9)       ///����  2////
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
//****************************  ��һ��  ����  ************************//
uint16_t diangan_guiyihua( uint8_t guiyi_diangan_hao)
{
        uint16_t  guiyi[guiyi_DIAN] ;
        
        SIU.GPDO[PCR38_PC6].R = 1 ;    // ��   
        delay500ms() ; delay500ms() ;
        SIU.GPDO[PCR38_PC6].R = 0 ;   // ��     
       //=======================================//
        if( guiyi_diangan_hao == 1)    // ˮƽһ�ŵ��=======13��
        {
        	for( i_quan=0 ; i_quan <guiyi_DIAN ; i_quan++ )
              { 
                   guiyi[i_quan] = diangan_ADC(13) ;
       	            delay5ms() ;   	    
              } 
       }
       
       //==========================================//
       else if( guiyi_diangan_hao == 2)  // ˮƽ���ŵ��=====3��
        {
      	      for( i_quan=0 ; i_quan <guiyi_DIAN ; i_quan++ )
             {
      	            guiyi[i_quan] = diangan_ADC(3) ;
      	      //      LCD5110_number_dis( guiyi[i_quan] );
      	            delay5ms() ; 	    
             }
      }
      
      //==========================================//
      else if( guiyi_diangan_hao == 3)  // ˮƽ���ŵ��======2��
      {
             for( i_quan=0 ; i_quan <guiyi_DIAN ; i_quan++ )
             {
    	           guiyi[i_quan] = diangan_ADC(2) ;
    	       //   LCD5110_number_dis( guiyi[i_quan] );
      	           delay5ms() ; 	    
             }
      }
        else if( guiyi_diangan_hao == 4)  // ˮƽ���ŵ��======2��
      {
             for( i_quan=0 ; i_quan <guiyi_DIAN ; i_quan++ )
             {
    	           guiyi[i_quan] = diangan_ADC(1) ;
    	       //   LCD5110_number_dis( guiyi[i_quan] );
      	           delay5ms() ; 	    
             }
      }
      //==========================================//
     else if( guiyi_diangan_hao == 5) //  =======11��
      {
      	      for( i_quan=0 ; i_quan <guiyi_DIAN ; i_quan++ )
             {
      	            guiyi[i_quan] = diangan_ADC(11) ;
       	            delay1ms() ; 	    
             }
      }
      
       //==========================================//
     else if( guiyi_diangan_hao == 6) //    =========1��
      {
      	      for( i_quan=0 ; i_quan <guiyi_DIAN ; i_quan++ )
             {
      	            guiyi[i_quan] = diangan_ADC(10) ;
       	            delay1ms() ;	    
             }
      }
      
    
  
      
      SIU.GPDO[PCR38_PC6].R = 1 ;    // ��   
      
      for( i_quan=1 ; i_quan <guiyi_DIAN ; i_quan++ )
      {       if( guiyi[0] < guiyi[i_quan])     guiyi[0] = guiyi[i_quan] ;  }
    
      return  guiyi[0] ;
}
//13===3====2===1
//         11===10
//          8====9
//=========================  �ɼ�����ź�  ====================//
void Cai_yang(void)  // �ɼ������е����ֵ��ˮƽ���֮�ͣ���ֱ���ֵ��
{ 
       int i;
      uint16_t  array_temp[11] = { 0 } ;
      array_one[0]=200*diangan_ADC(13)/guiyi_MAX[0];//б�õ��1��
	  array_two[0]=200*diangan_ADC(3)/guiyi_MAX[1]; //ˮƽ���1��
	  array_three[0]=200*diangan_ADC(2)/guiyi_MAX[2];//ˮƽ���2��
	  array_four[0]=200*diangan_ADC( 1)/guiyi_MAX[3];//б�õ��2��
	  array_five[0]=200*diangan_ADC(11)/guiyi_MAX[4];//��ֱ���1��  
      array_six[0]=200*diangan_ADC(10)/guiyi_MAX[5];//��ֱ���1�� 
	                        //SCI0_SendChar_16(array_one[0]);
 		                    //SCI0_SendCh[]ar_16(array_two[0]);
 		                    //SCI0_SendChar_16(array_three[0]);
 		                    //SCI0_SendChar_16(array_four[0]);
		                    //SCI0_SendChar_16(array_five[0]); 
        
	  //  �޷� ���� �˲�
	  //================== 1 �� ��� ============== //
	  if( (array_one[0]-array_one[1]) > 5 )    //======= �����޷���С����5��11 ���� +3 ��+7=====//
	  {
	        array_lvbo_flag[0][2] = 0 ;
	        array_lvbo_flag[0][3] = 0 ;
	        if( array_lvbo_flag[0][0] < 2 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 5 ����
	        {
	        	array_one[0]=array_one[1]+3 ; // 1
	        	array_lvbo_flag[0][0]++;
	        }
	  	else  if( array_lvbo_flag[0][1] < 3 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		array_one[0]=array_one[1]+8 ; // 1
	  		array_lvbo_flag[0][1]++;
	  	}
	  	else if( (array_one[0]-array_one[5]) < 60 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 6 �����ڻ�δ�ع�
	  	       array_one[0]=array_one[5] + 40 ;
	  	else  
	  	       array_one[0]=array_one[1]+15 ; // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���� 7 �����ڻ�δ�ع�
	  } 
 	  else if( (array_one[0]-array_one[1]) < -5 )   //======= �����޷���С���� -5��-11 ���� -3 ��-7=====//
 	  {
	        array_lvbo_flag[0][0] = 0 ;
	        array_lvbo_flag[0][1] = 0 ;
	        if( array_lvbo_flag[0][2] < 2 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 5 ����
	        {
	        	array_one[0]=array_one[1]-3 ; // 1
	        	if( array_one[0]<0 )   array_one[0]=0;
	        	array_lvbo_flag[0][2]++;
	        }
	  	else  if( array_lvbo_flag[0][3] < 3 ) //// array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		array_one[0]=array_one[1]-8 ; // 1
	  		if( array_one[0]<0 )   array_one[0]=0;
	  		array_lvbo_flag[0][3]++;
	  	}
	  	else if( (array_one[0]-array_one[5]) > -60 ) // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 6 �����ڻ�δ�ع�
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
 	  // ========    �����޷�    =========//
	   if( (array_one[0]-array_one[1]) > -3 && (array_one[0]-array_one[1]) < 3  )    //======= �����޷���С����5��11 ���� +3 ��+7=====//
	  {
	        array_lvbo_flag[0][5] ++ ;
	        if( array_lvbo_flag[0][5] == 10 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 5 ����
	    	 {
	    	  	array_temp[0]=( array_one[0]+array_one[1]+array_one[2]+array_one[3]+array_one[4]+array_one[5]+array_one[6]+array_one[7]+array_one[8]+array_one[9]+5 ) /10 ; // 1
	   		if( (array_temp[0]-array_one[0]) > -3 && (array_temp[0]-array_one[0]) < 3  )    
	   		  	array_one[0] = array_temp[0] ;
	   		else 
	   		        array_lvbo_flag[0][5] = 0 ;
	  	}
	  	else if( array_lvbo_flag[0][5] > 10 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		array_one[0] = array_one[1] ;
	  		array_lvbo_flag[0][5] -- ;
	  	}	
	  }
	  else
	  	array_lvbo_flag[0][5] = 0 ;
   
   
   
   
   

 	    
 	   //================== 2 �� ��� ============== //   
 	  if( (array_two[0]-array_two[1]) > 5 )    //======= �����޷���С����5��11 ���� +3 ��+7=====//
	  {
	        array_lvbo_flag[1][2] = 0 ;
	        array_lvbo_flag[1][3] = 0 ;
	        if( array_lvbo_flag[1][0] < 2 )  // array_lvbo_flag[1][0]  һ�ŵ�е�ƫ���Ƿ��� 5 ����
	        {
	        	array_two[0]=array_two[1]+3 ; // 1
	        	array_lvbo_flag[1][0]++;
	        }
	  	else  if( array_lvbo_flag[1][1] < 3 )  // array_lvbo_flag[1][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		array_two[0]=array_two[1]+8 ; // 1
	  		array_lvbo_flag[1][1]++;
	  	}
	  	else if( (array_two[0]-array_two[5]) < 60 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 6 �����ڻ�δ�ع�
	  	       array_two[0]=array_two[5] + 40 ;
	  	else  
	  	       array_two[0]=array_two[1]+15 ; // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���� 7 �����ڻ�δ�ع�
	  } 
 	  else if( (array_two[0]-array_two[1]) < -5 )   //======= �����޷���С���� -5��-11 ���� -3 ��-7=====//
 	  {
	        array_lvbo_flag[1][0] = 0 ;
	        array_lvbo_flag[1][1] = 0 ;
	        if( array_lvbo_flag[1][2] < 2 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 5 ����
	        {
	        	array_two[0]=array_two[1]-3 ; // 1
	        		if( array_two[0]<0 )   array_two[0]=0;
	        	array_lvbo_flag[1][2]++;
	        }
	  	else  if( array_lvbo_flag[1][3] < 3 ) //// array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		array_two[0]=array_two[1]-8 ; // 1
	  			if( array_two[0]<0 )   array_two[0]=0;
	  		array_lvbo_flag[1][3]++;
	  	}
	  	else if( (array_two[0]-array_two[5]) > -60 ) // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 6 �����ڻ�δ�ع�
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
 	   	   // ========    �����޷�    =========//
	   if( (array_two[0]-array_two[1]) > -3 && (array_two[0]-array_two[1]) < 3  )    //======= �����޷���С����5��11 ���� +3 ��+7=====//
	  {
	        array_lvbo_flag[1][5] ++ ;
	        if( array_lvbo_flag[1][5] == 10 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 5 ����
	    	 {
	    	  	array_temp[1]=( array_two[0]+array_two[1]+array_two[2]+array_two[3]+array_two[4]+array_two[5]+array_two[6]+array_two[7]+array_two[8]+array_two[9]+5 ) /10 ; // 1
                	if( (array_temp[1]-array_two[0]) > -3 && (array_temp[1]-array_two[0]) < 3  )    
	   		  	array_two[0] = array_temp[1] ;
	   		else 
	   		        array_lvbo_flag[1][5] = 0 ;
		 }
	  	else if( array_lvbo_flag[1][5] > 10 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		array_two[0] = array_two[1] ;
	  		array_lvbo_flag[1][5] -- ;
	  	}	
	  }
	  else
	  	array_lvbo_flag[1][5] = 0 ;
	  
	  
	  
	  

   
 	  //================== 3 �� ��� ============== //   
 	  if( (array_three[0]-array_three[1]) > 5 )    //======= �����޷���С����5��11 ���� +3 ��+7=====//
	  {
	        array_lvbo_flag[2][2] = 0 ;
	        array_lvbo_flag[2][3] = 0 ;
	        if( array_lvbo_flag[2][0] < 2 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 5 ����
	        {
	        	array_three[0]=array_three[1]+3 ; // 1
	        	array_lvbo_flag[2][0]++;
	        }
	  	else  if( array_lvbo_flag[2][1] < 3 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		array_three[0]=array_three[1]+8 ; // 1
	  		array_lvbo_flag[2][1]++;
	  	}
	  	else if( (array_three[0]-array_three[5]) < 60 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 6 �����ڻ�δ�ع�
	  	       array_three[0]=array_three[5] + 40 ;
	  	else  
	  	     {
	  	     	array_three[0]=array_three[1]+15 ; // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���� 7 �����ڻ�δ�ع�
	  	     }  
	  } 
 	  else if( (array_three[0]-array_three[1]) < -5 )   //======= �����޷���С���� -5��-11 ���� -3 ��-7=====//
 	  {
	        array_lvbo_flag[2][0] = 0 ;
	        array_lvbo_flag[2][1] = 0 ;
	        if( array_lvbo_flag[2][2] < 2 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 5 ����
	        {
	        	array_three[0]=array_three[1]-3 ; // 1
	        		if( array_three[0]<0 )   array_three[0]=0;
	        	array_lvbo_flag[2][2]++;
	        }
	  	else  if( array_lvbo_flag[2][3] < 3 ) //// array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		array_three[0]=array_three[1]-8 ; // 1
	  			if( array_three[0]<0 )   array_three[0]=0;
	  		array_lvbo_flag[2][3]++;
	  	}
	  	else if( (array_three[0]-array_three[5]) > -60 ) // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 6 �����ڻ�δ�ع�
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
 	  	  // ========    �����޷�    =========//
	   if( (array_three[0]-array_three[1]) > -3 && (array_three[0]-array_three[1]) < 3  )    //======= �����޷���С����5��11 ���� +3 ��+7=====//
	  {
	        array_lvbo_flag[2][5] ++ ;
	        if( array_lvbo_flag[2][5] == 10 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 5 ����
	    	 {
	    	  	array_temp[2]=( array_three[0]+array_three[1]+array_three[2]+array_three[3]+array_three[4]+array_three[5]+array_three[6]+array_three[7]+array_three[8]+array_three[9]+5 ) /10 ; // 1
                	if( (array_temp[2]-array_three[0]) > -3 && (array_temp[2]-array_three[0]) < 3  )    
	   		  	array_three[0] = array_temp[2] ;
	   		else 
	   		        array_lvbo_flag[2][5] = 0 ;
		 }
	  	else if( array_lvbo_flag[2][5] > 10 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		array_three[0] = array_three[1] ;
	  		array_lvbo_flag[2][5] -- ;
	  	}	
	  }
	  else
	  	array_lvbo_flag[2][5] = 0 ;
	  
	  
	  

 	  //================== 4 �� ��� ============== //   
 	  if( (array_four[0]-array_four[1]) > 5 )    //======= �����޷���С����5��11 ���� +3 ��+7=====//
	  {
	        array_lvbo_flag[3][2] = 0 ;
	        array_lvbo_flag[3][3] = 0 ;
	        if( array_lvbo_flag[3][0] < 2 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 5 ����
	        {
	        	array_four[0]=array_four[1]+3 ; // 1
	        	array_lvbo_flag[3][0]++;
	        }
	  	else  if( array_lvbo_flag[3][1] < 3 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		array_four[0]=array_four[1]+8 ; // 1
	  		array_lvbo_flag[3][1]++;
	  	}
	  	else if( (array_four[0]-array_four[5]) < 60 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 6 �����ڻ�δ�ع�
	  	       array_four[0]=array_four[5] + 40 ;
	  	else  
	  	    {
	  	    	array_four[0]=array_four[1]+15 ; // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���� 7 �����ڻ�δ�ع�
	  	    }   	 
	  }
 	  else if( (array_four[0]-array_four[1]) < -5 )   //======= �����޷���С���� -5��-11 ���� -3 ��-7=====//
 	  {
	        array_lvbo_flag[3][0] = 0 ;
	        array_lvbo_flag[3][1] = 0 ;
	        if( array_lvbo_flag[3][2] < 2 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 5 ����
	        {
	        	array_four[0]=array_four[1]-3 ; // 1
	        		if( array_four[0]<0 )   array_four[0]=0;
	        	array_lvbo_flag[3][2]++;
	        }
	  	else  if( array_lvbo_flag[3][3] < 3 ) //// array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		array_four[0]=array_four[1]-8 ; // 1
	  			if( array_four[0]<0 )   array_four[0]=0;
	  		array_lvbo_flag[3][3]++;
	  	}
	  	else if( (array_four[0]-array_four[5]) > -60 ) // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 6 �����ڻ�δ�ع�
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
 	  	  // ========    �����޷�    =========//
	   if( (array_four[0]-array_four[1]) > -3 && (array_four[0]-array_four[1]) < 3  )    //======= �����޷���С����5��11 ���� +3 ��+7=====//
	  {
	        array_lvbo_flag[3][5] ++ ;
	        if( array_lvbo_flag[3][5] == 10 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 5 ����
	    	 {
	    	  	array_temp[3]=( array_four[0]+array_four[1]+array_four[2]+array_four[3]+array_four[4]+array_four[5]+array_four[6]+array_four[7]+array_four[8]+array_four[9]+5 ) /10 ; // 1
                	if( (array_temp[3]-array_four[0]) > -3 && (array_temp[3]-array_four[0]) < 3  )    
	   		  	array_four[0] = array_temp[3] ;
	   		else 
	   		        array_lvbo_flag[3][5] = 0 ;
		 }
	  	else if( array_lvbo_flag[3][5] > 10 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		array_four[0] = array_four[1] ;
	  		array_lvbo_flag[3][5] -- ;
	  	}	
	  }
	  else
	  	array_lvbo_flag[3][5] = 0 ;
	  
	  
	  
	  
	  

 	  //================== 5 �� ��� ============== //   
 	  if( (array_five[0]-array_five[1]) > 3 )    //======= �����޷���С����5��11 ���� +3 ��+7=====//
	  {
	        array_lvbo_flag[4][3] = 0 ;
	        array_lvbo_flag[4][4] = 0 ;
	        array_lvbo_flag[4][5] = 0 ;
	        if( array_lvbo_flag[4][0] < 1 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 5 ����
	        {
	        	array_five[0]=array_five[1]+2 ; // 1
	        	array_lvbo_flag[4][0]++;
	        }
	  	else  if( array_lvbo_flag[4][1] < 2 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		array_five[0]=array_five[1]+4 ; // 1
	  		array_lvbo_flag[4][1]++;
	  	}
	       else  if( array_lvbo_flag[4][2] < 3 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		array_five[0]=array_five[1]+8 ; // 1
	  		array_lvbo_flag[4][2]++;
	  	}
	  	else if( (array_five[0]-array_five[6]) < 60 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 6 �����ڻ�δ�ع�
	  	       array_five[0]=array_five[6] + 40 ;
	  	else  
	  	   {
	  	   	 array_five[0]=array_five[1]+15 ; // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���� 7 �����ڻ�δ�ع�
	  	   }   
	  } 
 	  else if( (array_five[0]-array_five[1]) < -3 )   //======= �����޷���С���� -5��-11 ���� -3 ��-7=====//
 	  {
	        array_lvbo_flag[4][0] = 0 ;
	        array_lvbo_flag[4][1] = 0 ;
	        array_lvbo_flag[4][2] = 0 ;
	        if( array_lvbo_flag[4][3] < 1 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 5 ����
	        {
	        	array_five[0]=array_five[1]-2 ; // 1
	        		if( array_five[0]<0 )   array_five[0]=0;
	        	array_lvbo_flag[4][3]++;
	        }
	  	else  if( array_lvbo_flag[4][4] < 2 ) //// array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		array_five[0]=array_five[1]-4 ; // 1
	  			if( array_five[0]<0 )   array_five[0]=0;
	  		array_lvbo_flag[4][4]++;
	  	}
	  	else  if( array_lvbo_flag[4][5] < 3 ) //// array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		array_five[0]=array_five[1]-8 ; // 1
	  			if( array_five[0]<0 )   array_five[0]=0;
	  		array_lvbo_flag[4][5]++;
	  	}
	  	else if( (array_five[0]-array_five[6]) > -60 ) // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 6 �����ڻ�δ�ع�
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
     	  // ========    �����޷�    =========//
	   if( (array_five[0]-array_five[1]) > -3 && (array_five[0]-array_five[1]) < 3  )    //======= �����޷���С����5��11 ���� +3 ��+7=====//
	  {
	        array_lvbo_flag[4][6] ++ ;
	        if( array_lvbo_flag[4][6] == 10 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 5 ����
	    	 {
	    	  	array_temp[4]=( array_five[0]+array_five[1]+array_five[2]+array_five[3]+array_five[4]+array_five[5]+array_five[6]+array_five[7]+array_five[8]+array_five[9]+5 ) /10 ; // 1
                	if( (array_temp[4]-array_five[0]) > -3 && (array_temp[4]-array_five[0]) < 3  )    
	   		  	array_five[0] = array_temp[4] ;
	   		else 
	   		        array_lvbo_flag[4][6] = 0 ;
		 }
	  	else if( array_lvbo_flag[4][6] > 10 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		array_five[0] = array_five[1] ;
	  		array_lvbo_flag[4][6] -- ;
	  	}	
	  }
	  else
	  	array_lvbo_flag[4][6] = 0 ;
	  
	  
	   //========================== ƫ��� ===========================//
        if( array_one[0] < 2 )    // ��ֹ   ��ĸ ���ֵ   Ϊ��
                array_one[0] = 1 ;
       if( array_two[0] < 2 )    // ��ֹ   ��ĸ ���ֵ   Ϊ��
                array_two[0] = 1 ;
        if( array_three[0] < 2 )    // ��ֹ   ��ĸ ���ֵ   Ϊ��
                array_three[0] = 1 ;
       if( array_four[0] < 2 )    // ��ֹ   ��ĸ ���ֵ   Ϊ��
                array_four[0] = 1 ;
        if( array_five[0] < 2 )    // ��ֹ   ��ĸ ���ֵ   Ϊ��
                array_five[0] = 1 ;

 	  	if( array_six[0] < 2 )    // ��ֹ   ��ĸ ���ֵ   Ϊ��
                array_six[0] = 1 ;    
 	  
 //=================================================//
 //���ڲ������Ѿ���βɼ��ˣ���������ƽ���˲������Բ����˲��������ٿ�
 //ʵʱ��ͬһʱ�̵ĵ��ֵ����ͬһ��������
	  	array_current[0] = array_one[0] ;//     ��Ӧ���1
	  	array_current[1] = array_two[0] ;//	     ��Ӧ���2
	  	array_current[2] = array_three[0] ;//     ��Ӧ���3
	  	array_current[3] = array_four[0];//       ��Ӧ���4
        array_current[4] = array_five[0] ;//       ��Ӧ���5 		
        array_current[5]=array_six[0];
  
    // noise_protect_first();
    
    //����Ų�
    ///4========5
    // 0==1==2==3 
    
     
       chui_zhi_daingan[0]=array_current[4]-array_current[5]; //��ֱ���֮��               
       Verticaldiangan_sum[0]=array_current[4]+array_current[5]; //��ֱ���֮��
       Leveltaldiangan_sum[0]=array_current[1]+array_current[2];//ˮƽ���֮��
       xiezhi_diangan_sum[0]= array_current[0]+array_current[3];//б�õ��֮��
       total_diangan_sum[0]=xiezhi_diangan_sum[0]+Leveltaldiangan_sum[0];//���ŵ���ܺ� 
                             //SCI0_SendChar_16(three_diangan_sum[0]);
 		                     //SCI0_SendChar_16(two_diangan_sum[0]);
 		                    //SCI0_SendChar_16(array_three[0]);
 		                    //SCI0_SendChar_16(array_four[0]);
		                    //SCI0_SendChar_16(array_five[0]); 
          
}
//����ƫ���ڸ�����Χʱ�Ĵ��
//����ģ��map������ڲ�ѯת�򣿣���
//===================================ƫ��������==========================//
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


  
//�ο�
//if( chuizhi_piancha[0]>=0)   chuizhi_piancha_xishu=chuizhi_piancha[0]*100/6+400 ;
//else chuizhi_piancha_xishu=-chuizhi_piancha[0]*100/6+400 ;
//if(chuizhi_piancha_xishu<500)    chuizhi_piancha_xishu=500 ;
//if(chuizhi_piancha_xishu>1300)    chuizhi_piancha_xishu=1300 ;
//xiezhi_piancha_xishu=1300-chuizhi_piancha_xishu ;
//if(xiezhi_piancha_xishu<500)    xiezhi_piancha_xishu=500 ;
//servo_Error[0]= xiezhi_piancha_xishu*xiezhi_piancha[0]/1000+chuizhi_piancha_xishu*chuizhi_piancha[0]/1000;

 
//==========================���Գ���==========================//
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
		            		         //��ȡ����ƫ�������������������
		         KK_chuizhi[0]=qu_zheng(chuizhi_piancha[0])/6; 
		         KK_shuiping[0]=qu_zheng(shuiping_piancha[0])/6;  
		         KK_xiezhi[0]= qu_zheng(xiezhi_piancha[0])/6;
		  
		          if(KK_shuiping[0]>16)  KK_shuiping[0]=16;
		          if(KK_chuizhi[0]>16)  KK_chuizhi[0]=16;
		          if(KK_xiezhi[0]>16)  KK_xiezhi[0]=16;
		 
		      
		      //=======����б�õ��ƫ����=======================//
		      if( Road_Type[0]==3)
		      {
		      	  if(KK_xiezhi[0]<2&&KK_chuizhi[0]>12)     
		           xiezhi_piancha[0]= xiezhi_piancha[0]/10;
		      }
		     //=================================================//
		      if(Leveltaldiangan_sum[0]>320)
		      {
		         //��⵽ʮ�֣�����б�õ�е�ƫ����
		         if( Road_Type[0]==3)
		         {
		             if(shuiping_piancha[0]*chuizhi_piancha[0]>0)
		       	            	 servo_Error[0]= 110*shuiping_piancha[0]/100
		                                          +chuizhi_piancha[0]/7 ;	
		       	     else
		       	  	       	     servo_Error[0]= 120*shuiping_piancha[0]/100
		                                          -chuizhi_piancha[0]/7 ;	
		         	
		         }
		        //����кܴ�ʱ���ܿ���б����ʮ�֣���ˮƽƫ����
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
		    
		         //��⵽ʮ�֣�����б�õ�е�ƫ����
		         if( Road_Type[0]==3)
		         {
		             if(shuiping_piancha[0]*chuizhi_piancha[0]>0)
		       	            	 servo_Error[0]= 110*shuiping_piancha[0]/100
		                                          +chuizhi_piancha[0]/7 ;	
		       	     else
		       	  	       	     servo_Error[0]= 120*shuiping_piancha[0]/100
		                                          -chuizhi_piancha[0]/7 ;	
		         	
		         }
		        //����кܴ�ʱ���ܿ���б����ʮ�֣���ˮƽƫ����
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
       diangan_max[0]=0;//���㴦��
 diangan_maxfour[0]=0;
       for(i=0;i<4;i++)
       {
        
          if( diangan_max [0] < array_current[i] )//�ҳ��ĸ���е����ֵ��
          {
         	 diangan_max [0]=array_current[i]  ; //�Լ��ҳ��ǵڼ�����������ֵ 
       	     cixu[0][0]=i ;
          }
      
       }
       
       
 
       //=================��ֱ������ֵ
       if(array_current[5]>array_current[6])
       chuizhidiangan_max[0]=array_current[5];
       else chuizhidiangan_max[0]=array_current[6];
      /*
       if(cixu[0][0]==0)     //������:0
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
       else if(cixu[0][0]==1)//�м����:1
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
          
      else                 //�ұ����:2
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
     diangan_min[0]=array_current[cixu[0][2]];//������Сֵ
     */
  //б�õ�д�С   
     /*if(array_current[3]>array_current[4])
         xiediangan_max[0]=array_current[3];
     else 
         xiediangan_max[0]=array_current[4];
     
     */
     //�м��������ߵ��֮��
     
    /* 
     if(cixu[0][0]==1)//�м����
         middle_piancha[0]==0;
     else if(cixu[0][0]==0)//������
     {
        middle_piancha[0]=array_current[0]-array_current[1];
        	
     }
     else if(cixu[0][0]==2)
     {
     	 middle_piancha[0]=array_current[1]-array_current[2];
     }
     */
      
     
     
     
      //========================== ƫ��� ===========================//
        if( array_current[0] < 2 )    // ��ֹ   ��ĸ ���ֵ   Ϊ��
                array_current[0] = 1 ;
       if( array_current[1] < 2 )    // ��ֹ   ��ĸ ���ֵ   Ϊ��
                array_current[1] = 1 ;
        if( array_current[2] < 2 )    // ��ֹ   ��ĸ ���ֵ   Ϊ��
                array_current[2] = 1 ;
       if( array_current[3] < 2 )    // ��ֹ   ��ĸ ���ֵ   Ϊ��
                array_current[3] = 1 ;
       if( array_current[4] < 2 )    // ��ֹ   ��ĸ ���ֵ   Ϊ��
                array_current[4] = 1 ;
        
      if( array_current[5] < 2 )    // ��ֹ   ��ĸ ���ֵ   Ϊ��
                array_current[5] = 1 ;
      if( array_current[6] < 2 )    // ��ֹ   ��ĸ ���ֵ   Ϊ��
                array_current[6] = 1 ;
        
        
      //ˮƽ���ƫ��
      shuiping_piancha[0] = 250* ( array_current[1] - array_current[2] ) /( array_current[1] + array_current[2] )  ;
      //shuiping_piancha[0] = 40 * ( array_current[0] - array_current[2] ) /( array_current[0]+array_current[2] )*3 ;
        //��ֱ���ƫ��  
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
         
   //================== ˮƽ����޷�����  ============== //   
 	  if( (shuiping_piancha[0]-shuiping_piancha[1]) > 3 )    //======= �����޷���С����5��11 ���� +3 ��+7=====//
	  {
	        shuiping_piancha_flag[3] = 0 ;
	        shuiping_piancha_flag[4] = 0 ;
	        shuiping_piancha_flag[5] = 0 ;
	        if( shuiping_piancha_flag[0] < 1 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 5 ����
	        {
	        	shuiping_piancha[0]=shuiping_piancha[1]+2 ; // 1
	        	shuiping_piancha_flag[0]++;
	        }
	  	else  if( shuiping_piancha_flag[1] < 1 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		shuiping_piancha[0]=shuiping_piancha[1]+4 ; // 1
	  		shuiping_piancha_flag[1]++;
	  	}
	       else  if( shuiping_piancha_flag[2] < 2 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		shuiping_piancha[0]=shuiping_piancha[1]+7 ; // 1
	  		shuiping_piancha_flag[2]++;
	  	}
	  	else if( (shuiping_piancha[0]-shuiping_piancha[5]) < 50 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 6 �����ڻ�δ�ع�
	  	       shuiping_piancha[0]=shuiping_piancha[5] + 32 ;
	  	else  
	  	       shuiping_piancha[0]=shuiping_piancha[1]+12 ; // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���� 7 �����ڻ�δ�ع�
	  } 
 	  else if( (shuiping_piancha[0]-shuiping_piancha[1]) < -3 )   //======= �����޷���С���� -5��-11 ���� -3 ��-7=====//
 	  {
	        shuiping_piancha_flag[0] = 0 ;
	        shuiping_piancha_flag[1] = 0 ;
	        shuiping_piancha_flag[2] = 0 ;
	        if( shuiping_piancha_flag[3] < 1 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 5 ����
	        {
	        	shuiping_piancha[0]=shuiping_piancha[1]-2 ; // 1
	        	shuiping_piancha_flag[3]++;
	        }
	  	else  if( shuiping_piancha_flag[4] < 1 ) //// array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		shuiping_piancha[0]=shuiping_piancha[1]-4 ; // 1
	  		shuiping_piancha_flag[4]++;
	  	}
	  	else  if( shuiping_piancha_flag[5] < 2 ) //// array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		shuiping_piancha[0]=shuiping_piancha[1]-7 ; // 1
	  		shuiping_piancha_flag[5]++;
	  	}
	  	else if( (shuiping_piancha[0]-shuiping_piancha[7]) > -50 ) // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 6 �����ڻ�δ�ع�
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

   
   //================== ��ֱ����޷�����  ============== //   
 	  if( (chuizhi_piancha[0]-chuizhi_piancha[1]) > 3 )    //======= �����޷���С����5��11 ���� +3 ��+7=====//
	  {
	        chuizhi_piancha_flag[3] = 0 ;
	        chuizhi_piancha_flag[4] = 0 ;
	        chuizhi_piancha_flag[5] = 0 ;
	        if( chuizhi_piancha_flag[0] < 1 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 5 ����
	        {
	        	chuizhi_piancha[0]=chuizhi_piancha[1]+2 ; // 1
	        	chuizhi_piancha_flag[0]++;
	        }
	  	else  if( chuizhi_piancha_flag[1] < 1 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		chuizhi_piancha[0]=chuizhi_piancha[1]+4 ; // 1
	  		chuizhi_piancha_flag[1]++;
	  	}
	       else  if( chuizhi_piancha_flag[2] < 2 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		chuizhi_piancha[0]=chuizhi_piancha[1]+8 ; // 1
	  		chuizhi_piancha_flag[2]++;
	  	}
	  	else if( (chuizhi_piancha[0]-chuizhi_piancha[5]) < 50 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 6 �����ڻ�δ�ع�
	  	       chuizhi_piancha[0]=chuizhi_piancha[5] + 35 ;
	  	else  
	  	       chuizhi_piancha[0]=chuizhi_piancha[1]+15 ; // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���� 7 �����ڻ�δ�ع�
	  } 
 	  else if( (chuizhi_piancha[0]-chuizhi_piancha[1]) < -3 )   //======= �����޷���С���� -5��-11 ���� -3 ��-7=====//
 	  {
	        chuizhi_piancha_flag[0] = 0 ;
	        chuizhi_piancha_flag[1] = 0 ;
	        chuizhi_piancha_flag[2] = 0 ;
	        if( chuizhi_piancha_flag[3] < 1 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 5 ����
	        {
	        	chuizhi_piancha[0]=chuizhi_piancha[1]-2 ; // 1
	        	chuizhi_piancha_flag[3]++;
	        }
	  	else  if( chuizhi_piancha_flag[4] < 1 ) //// array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		chuizhi_piancha[0]=chuizhi_piancha[1]-4 ; // 1
	  		chuizhi_piancha_flag[4]++;
	  	}
	  	else  if( chuizhi_piancha_flag[5] < 2 ) //// array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		chuizhi_piancha[0]=chuizhi_piancha[1]-8 ; // 1
	  		chuizhi_piancha_flag[5]++;
	  	}
	  	else if( (chuizhi_piancha[0]-chuizhi_piancha[5]) > -50 ) // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 6 �����ڻ�δ�ع�
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
         
         
   
    
//=================================ƫ�����Ĵ���===========================//
  // smartcar_position();
  // servo_Error[0]= 0*shuiping_piancha[0]/20+20*chuizhi_piancha[0]/20;
     
    smartcar_position_NEW();   
                           
//========================================================================//   
    //��ƫ���������100����
     if(servo_Error[0]>100)    servo_Error[0]=100;
     if(servo_Error[0]<-100)   servo_Error[0]=-100;
    
        //SCI0_SendChar_16(servo_Error[0]);
      //==================   ƫ��  �޷�����  ============== //   
   if( (servo_Error[0]-servo_Error[1]) > 6 )    //======= �����޷���С����5��11 ���� +3 ��+7=====//
	  {
	        servo_Error_flag[3] = 0 ;
	        servo_Error_flag[4] = 0 ;
	        servo_Error_flag[5] = 0 ;
	        if( servo_Error_flag[0] < 1 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 5 ����
	        {
	        	servo_Error[0]=servo_Error[1]+4 ; // 1
	        	servo_Error_flag[0]++;
	        }
	  	else  if( servo_Error_flag[1] < 1 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		servo_Error[0]=servo_Error[1]+8 ; // 1
	  		servo_Error_flag[1]++;
	  	}
	       else  if( servo_Error_flag[2] < 2 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		servo_Error[0]=servo_Error[1]+12 ; // 1
	  		servo_Error_flag[2]++;
	  	}
	  	else if( (servo_Error[0]-servo_Error[5]) < 60 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 6 �����ڻ�δ�ع�
	  	       servo_Error[0]=servo_Error[5] + 45 ;
	  	else  
	  	       servo_Error[0]=servo_Error[1]+15 ; // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���� 7 �����ڻ�δ�ع�
	  } 
 	  else if( (servo_Error[0]-servo_Error[1]) < -6 )   //======= �����޷���С���� -5��-11 ���� -3 ��-7=====//
 	  {
	        servo_Error_flag[0] = 0 ;
	        servo_Error_flag[1] = 0 ;
	        servo_Error_flag[2] = 0 ;
	        if( servo_Error_flag[3] < 1 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 5 ����
	        {
	        	servo_Error[0]=servo_Error[1]-4 ; // 1
	        	servo_Error_flag[3]++;
	        }
	  	else  if( servo_Error_flag[4] < 1 ) //// array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		servo_Error[0]=servo_Error[1]-8 ; // 1
	  		servo_Error_flag[4]++;
	  	}
	  	else  if( servo_Error_flag[5] < 2 ) //// array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		servo_Error[0]=servo_Error[1]-12 ; // 1
	  		servo_Error_flag[5]++;
	  	}
	  	else if( (servo_Error[0]-servo_Error[5]) > -60 ) // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 6 �����ڻ�δ�ع�
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


//�������ݱ�����ʩ����ʶ�𲢴���������ݡ�
//(1)�ڲ�����L1��L2��L3�ĸ�Ӧ�綯�ƾ���С������£�
//����ȷ����ʱ�϶������������ʱ����������Ӧ�綯�Ƶĵ����ŷ����仯��
//��������ת�䣬Ӧ�����ֵ��L1�ĸ�Ӧ�綯�����
//������ת�䣬Ӧ���ֵ��L3�ĸ�Ӧ�綯�����
//����ʱ����з����仯������Ϊ���ݷ�������,������ǿ�Ƶ����
//(2)�������Ϸ�����L2�ĸ�Ӧ�綯��ʼ��Ϊ���ֵ���м�ֵ��
//��Ϊ��Сֵ������Ϊ���ݴ���ͬ������ǿ�Ƶ�����
//(3)��L2�ĸ�Ӧ�綯�Ʒǳ�С������£�
//��ȡ��ֹ����궨�����������������ʩ��
 void noise_protect_first()
 {
 
    if(protect_Road_Type[0]==1&&total_diangan_sum[0]<240)//�����
    {  //�����������ߵ���� ��ǿ�Ƶ���Ϊ���ֵ
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
    
    if(protect_Road_Type[0]==2&&total_diangan_sum[0]<240)//�����
	{//����������ұߵ���� ��ǿ�Ƶ���Ϊ���ֵ
	    if(cixu[0][0]!= 2)
	    {
	       if(array_current[0]>array_current[2]&&array_current[3]>array_current[4])
	    	array_current[2]=array_current[cixu[1][0]];
	    	array_current[1]=array_current[cixu[1][1]];
	    	array_current[0]=array_current[cixu[1][2]];
	    	
	    }
    }	
 	//�����С�ĵ�д���Ϊ1�����м���Ϊ��С������Ϊ��������ǿ�Ƶ���
 	if(cixu[0][2]==1) 
 	{
 	    array_current[1]=array_current[cixu[1][1]];
   		
 	}
 	    
 	
 }
//===================================================// 
 void noise_protect_second()
 {
 
      if(protect_Road_Type[0]==1)//�����
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


//����ʵ����������������ͷ�Ϊ:
//ֱ������������������ʮ�ֽ�����Լ��µ�����
//���������з������Ҫ�����ǣ��ڲ�ͬ���͵������趨��ͬ�������ٶȡ�
//��ͬ��������ʶ�𷽷����£�
//(1)ֱ����������L1��L2��L3�ĸ�Ӧ�綯�ƺ�ֵ����һ��ֵ��
//������V1�� V2�ĸ�Ӧ�綯�Ʋ�ֵ�ľ���ֵС��һ��ֵ
//=================================================================//
//(2)�����������L1��L2��L3�ĸ�Ӧ�綯�ƺ�ֵС��һ��ֵ��
//�����������������V1�� V2�ĸ�Ӧ�綯��֮�����һ��ֵ��
//�����������������V2�� V1�ĸ�Ӧ�綯��֮�����һ��ֵ�
//=================================================================//
//(3)ʮ�ֽ���㣺������V1�� V2�ĸ�Ӧ�綯�ƾ��ֱ����һ��ֵ
//=================================================================//
//(4)�µ��ɻ���Ϊ���¡��¶������¡��º��ĸ�����
//�������к����Ե�����������L1��L2��L3�ĸ�Ӧ�綯�ƺ�ֵΪEsum
//���£�Esum�ܴ�
//�¶���Esum��С��
//���£�Esum�ܴ�
//�º�Esum��С���ָ�����ֵ�
////0:ֱ��  4:���� 5:  1:��ת��  2:��ת�� 3:ʮ�ֽ���� 
//=================================================================//
void sai_dao_position()
{ 
    
     //uint16_t two_cut_one;
     
//======== ֱ����� ===========//
   if(Road_Type[0] !=0)
   {

	    if(Leveltaldiangan_sum[0]>320 && qu_zheng(chui_zhi_daingan[0] )<30)
	    {
	         
	         if(++Road_Type_Times[0]>20)
	         {     
	              
 	             Road_Type[0] = 0;//ֱ��
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
   
    
//====================ʮ�ֽ������=============================//
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
//=======================��ת����=============================//
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
//======================��ת����==============================//
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
 
//============================��������˲�==========================// 
 //protect_
 
 if(protect_Road_Type[0] !=0)
   {

	    if(Leveltaldiangan_sum[0]>320 && qu_zheng(chui_zhi_daingan[0] )<30)
	    {
	         
	         if(++protect_Road_Type_Times[0]>50)
	         {     
	              
 	             protect_Road_Type[0] = 0;//ֱ��
 	                           
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
   
    
//====================ʮ�ֽ������=============================//
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
//=======================��ת����=============================//
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
//======================��ת����==============================//
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
///////////////////////////////�����⺯��///////////////////////////
//��������;Ѱ��ƫ�����ı仯���ƣ�ȷ�����仹�ǽ���
void wandao_check()
{
     
    uint8_t i;
    uint8_t static ruwan_times;
    uint8_t static chuwan_times;

   if(Road_Type[0]==1)//�����λ���ж�
   {
	    
	   chuwan_times=0;
	   ruwan_times=0;
	   for(i=0;i<50;i++)
	   {
	   	     if((servo_Error[i]-servo_Error[i+15])>0) 
	           ruwan_times++ ;//��������ж�
	           
	    	 if((servo_Error[i]-servo_Error[i+15])<0) 
	    	   chuwan_times++ ;//��������ж�

	   }
	    
	    
	   if(ruwan_times>15) wandao_come_in=1;//wandao_come_in��1��������
	   else wandao_come_in=0;
	   
	   if(chuwan_times>15) wandao_go_out=1;//wandao_go_out��1�������
	   else wandao_go_out=0;
    }
    
   else if(Road_Type[0]==2)//�����λ���ж�
   {
	    
	   chuwan_times=0;
	   ruwan_times=0;
	   for(i=0;i<50;i++)
	   {
	   	     if((servo_Error[i]-servo_Error[i+15])<0) 
	           ruwan_times++ ;//��������ж�
	           
	    	 if((servo_Error[i]-servo_Error[i+15])>0) 
	    	   chuwan_times++ ;//��������ж�

	   }
	    
	    
	   if(ruwan_times> 15) wandao_come_in=2;//wandao_come_in��2��������
	   else wandao_come_in=0;
	   
	   if(chuwan_times> 15) wandao_go_out=2;//wandao_go_out��2���ҳ���
	   else wandao_go_out=0;
    }
    
    else
    {
       	wandao_come_in=0;//��������������㣬��־λ����
       	wandao_go_out=0;//
    }
}
 
 
 
 
 
//======================void servo_Fuzzy_Kp_chu_shi_hua( void ) ======================================//
//                                  kp_e (kp_e_lishudu )      ( kp_e+1) ( Error_lishudu_Max - kp_e_lishudu )              
//                                --------------------------------------------------------------------------------
//                                  kp_ec (kp_ec_lishudu ) 
//                               (kp_ec+1) (Error_lishudu_Max - kp_ec_lishudu ) 
//                                   ����ƫ��  ƫ��仯�� ���ó� Kp ��ֵ
//=================================================================================//
uint16_t servo_Fuzzy_Kp_chu_shi_hua( void )
{
        uint8_t    kp_e = 0 , kp_ec = 0 ;  //  Kp ������еڼ�����
                    // ȡ��С����ĺ� E������������ L���������ĺ� E+1 ��Error_lishudu_Max - L
        uint16_t  kp_e_lishudu , kp_ec_lishudu  ; //  ������
        uint8_t  temp_11[4] = { 0,0,0,0 } , temp_12[4] = { 0,0,0,0 } , temp_21[4] = { 0,0,0,0 } , temp_22[4] = { 0,0,0,0 } ;
       
        Fuzzy_Kp_cunchu[5] = Fuzzy_Kp_cunchu[0] ;  // �洢��һ�� Kp�ù���ֵ �� �����뱾�αȽ� �� Ŀ�����޷��˲�
  
               ////**-----Kp  ----E----- ��������----- ****/////
              ///***    ���      kp_e      kp_e_lishudu   ***///
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

                       ////***----Kp  -----EC----- ��������---****/////
                      ///****    ���      kp_ec      kp_ec_lishudu    ****///
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
       /////      �ҵ�������еڼ�����       ////
       temp_11[0] = servo_Fuzzy_Kp_rule[ kp_ec ][ kp_e ] ;
       temp_12[0] = servo_Fuzzy_Kp_rule[ kp_ec][ kp_e +1 ] ;
       temp_21[0] = servo_Fuzzy_Kp_rule[ kp_ec+1 ][ kp_e ] ;
       temp_22[0] = servo_Fuzzy_Kp_rule[ kp_ec+1 ][ kp_e+1 ] ;
       
       ///          �ҵ��� servo_Error �ҵ���������   /////
       temp_11[1] = kp_e_lishudu ;
       temp_12[1] = Error_lishudu_Max - kp_e_lishudu ;
       temp_21[1] = kp_e_lishudu ;
       temp_22[1] = Error_lishudu_Max - kp_e_lishudu ;
       
        ///          �ҵ��� servo_Error_c �ҵ���������   /////
       temp_11[2] = kp_ec_lishudu ;
       temp_21[2] = Error_c_lishudu_Max - kp_ec_lishudu ;
       temp_12[2] = kp_ec_lishudu ;
       temp_22[2] = Error_c_lishudu_Max - kp_ec_lishudu ;
       
       ///                  ȷ���ĸ����������            /////
       if( temp_11[1] > temp_11[2] )  temp_11[1] = temp_11[2] ;
       if( temp_12[1] > temp_12[2] )  temp_12[1] = temp_12[2] ;
       if( temp_21[1] > temp_21[2] )  temp_21[1] = temp_21[2] ;
       if( temp_22[1] > temp_22[2] )  temp_22[1] = temp_22[2] ;
       
       //===================================//
       // С��ȡ��  �������ͬ�����򡪡�������ȡ�� //
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
      Fuzzy_Kp_cunchu[1] =  servo_Fuzzy_Kp_dan[ temp_11[0] ] * temp_11[1]   // ����
                                       + servo_Fuzzy_Kp_dan[ temp_12[0] ] * temp_12[1]
                                       + servo_Fuzzy_Kp_dan[ temp_21[0] ] * temp_21[1]
                                       + servo_Fuzzy_Kp_dan[ temp_22[0] ] * temp_22[1] ;
       Fuzzy_Kp_cunchu[2] = temp_11[1] + temp_12[1] + temp_21[1] + temp_22[1] ;  // ��ĸ             
       if( Fuzzy_Kp_cunchu[2] < 2 )  Fuzzy_Kp_cunchu[2]=1 ;
       
       Fuzzy_Kp_cunchu[0] = Fuzzy_Kp_cunchu[1] / Fuzzy_Kp_cunchu[2] ;
       
       
       
      

       //========================= �����˲�   ===================================//
       if( ( Fuzzy_Kp_cunchu[0] - Fuzzy_Kp_cunchu[5] ) > 27  )      // �� Kp ���� ���� �޷� ���� �˲�
             Fuzzy_Kp_cunchu[0] = Fuzzy_Kp_cunchu[5] + 19 ;         // [0] ���ε� Kp  ,
       else if( (Fuzzy_Kp_cunchu[0] - Fuzzy_Kp_cunchu[5])<-27 )  // [5] �ϴε�Kp
             Fuzzy_Kp_cunchu[0] = Fuzzy_Kp_cunchu[5] - 19 ;
     //========================  �����˲�    ==================================//
     if( ( Fuzzy_Kp_cunchu[0] - Fuzzy_Kp_cunchu[5] ) < 3 )
     {
     	   if( ( Fuzzy_Kp_cunchu[0] - Fuzzy_Kp_cunchu[5] ) > -3 )
     	        Fuzzy_Kp_cunchu[0] = Fuzzy_Kp_cunchu[5] ;
     }

       Fuzzy_Kp_cunchu[0] = ( 87*Fuzzy_Kp_cunchu[0] + 13*Fuzzy_Kp_cunchu[5]) / 100 ; // ��ƽ��ֵ���� �˲�
        
         
                return Fuzzy_Kp_cunchu[0]  ;    
}

//=========================void servo_Fuzzy_Kd_chu_shi_hua( void ) =================================//
//                                  kp_e (kp_e_lishudu )      ( kp_e+1) ( Error_lishudu_Max - kp_e_lishudu )              
//                                --------------------------------------------------------------------------------
//                                  kp_ec (kp_ec_lishudu ) 
//                               (kp_ec+1) (Error_lishudu_Max - kp_ec_lishudu ) 
//                               ����ƫ��  ƫ��仯�� ���ó� Kd ��ֵ
//=======================================================================================//
uint16_t servo_Fuzzy_Kd_chu_shi_hua( void )
{
        uint8_t    kp_e = 0 , kp_ec = 0 ;  //  Kp ������еڼ�����
                    // ȡ��С����ĺ� E������������ L���������ĺ� E+1 ��Error_lishudu_Max - L
        uint16_t  kp_e_lishudu=0 , kp_ec_lishudu =0  ; //  ������
        uint8_t  temp_11[4] = { 0,0,0,0 } , temp_12[4] = { 0,0,0,0 } , temp_21[4] = { 0,0,0,0 } , temp_22[4] = { 0,0,0,0 } ;
       
        Fuzzy_Kd_cunchu[5] = Fuzzy_Kd_cunchu[0] ;  // �洢��һ�� Kd�ù���ֵ �� �����뱾�αȽ� �� Ŀ�����޷��˲�
       
        ////***********    -----------Kp  ------E------- ��������---------   **************/////
        ///******************         ���      kp_e      kp_e_lishudu               **************///
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

         ////***********    -----------Kp  ------EC------- ��������---------   **************/////
        ///******************         ���      kp_ec      kp_ec_lishudu               **************///
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
       /////      �ҵ�������еڼ�����       ////
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
       
       ///                  ȷ���ĸ����������            /////
       if( temp_11[1] > temp_11[2] )  temp_11[1] = temp_11[2] ;
       if( temp_12[1] > temp_12[2] )  temp_12[1] = temp_12[2] ;
       if( temp_21[1] > temp_21[2] )  temp_21[1] = temp_21[2] ;
       if( temp_22[1] > temp_22[2] )  temp_22[1] = temp_22[2] ;
       
       //===================================//
       // С��ȡ��  �������ͬ�����򡪡�������ȡ�� //
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
   
       Fuzzy_Kd_cunchu[1]   =  servo_Fuzzy_Kd_dan[ temp_11[0] ] * temp_11[1]  //����
                                          + servo_Fuzzy_Kd_dan[ temp_12[0] ] * temp_12[1]
                                          + servo_Fuzzy_Kd_dan[ temp_21[0] ] * temp_21[1]
                                          + servo_Fuzzy_Kd_dan[ temp_22[0] ] * temp_22[1] ;
       Fuzzy_Kd_cunchu[2] = temp_11[1] + temp_12[1] + temp_21[1] + temp_22[1] ;   // ��ĸ           
       if( Fuzzy_Kd_cunchu[2] <2 )    Fuzzy_Kd_cunchu[2] = 1;
           
       Fuzzy_Kd_cunchu[0] = Fuzzy_Kd_cunchu[1] / Fuzzy_Kd_cunchu[2] ;  //  ���μ������ Kd ֵ
    
       //==================  �����˲� ============================//
         if( ( Fuzzy_Kd_cunchu[0] - Fuzzy_Kd_cunchu[5] ) > 27 )  //  �Լ������ Kd ֵ���� ���� �޷������˲�
              Fuzzy_Kd_cunchu[0] = Fuzzy_Kd_cunchu[5] + 19 ;  
        else if( ( Fuzzy_Kd_cunchu[0] - Fuzzy_Kd_cunchu[5] ) < -27 )
        {
       	        Fuzzy_Kd_cunchu[0] = Fuzzy_Kd_cunchu[5] - 19 ;
        	if( Fuzzy_Kd_cunchu[0] < 0)
        	    Fuzzy_Kd_cunchu[0] = 0;
        }
       //==================  �����˲�  ======================//
      if( ( Fuzzy_Kd_cunchu[0] - Fuzzy_Kd_cunchu[5] ) < 2 )  //  
       {
       	      if( ( Fuzzy_Kd_cunchu[0] - Fuzzy_Kd_cunchu[5] ) > -2 )
       	          Fuzzy_Kd_cunchu[0] = Fuzzy_Kd_cunchu[5] ;
       }

       Fuzzy_Kd_cunchu[0] = ( 87*Fuzzy_Kd_cunchu[0] + 13*Fuzzy_Kd_cunchu[5] ) / 100 ; // ��ƽ�� ���� �˲�
                     
       
        //       SCI0_SendChar_16( Fuzzy_Kd_cunchu[0] ) ;
    
        return Fuzzy_Kd_cunchu[0]  ;
}


void Fuzzy_Speed_chushihua( void )
{
	//=============== Fuzzy_speed_control ===============//
        Fuzzy_speed_cunchu_zuo[1] =  speed_Fuzzy_zuo_dan[ speed_temp_11[0] ] * speed_temp_11[1]   // ����
                                                     + speed_Fuzzy_zuo_dan[ speed_temp_12[0] ] * speed_temp_12[1]
                                                     + speed_Fuzzy_zuo_dan[ speed_temp_21[0] ] * speed_temp_21[1]
                                                     + speed_Fuzzy_zuo_dan[ speed_temp_22[0] ] * speed_temp_22[1] ;
       Fuzzy_speed_cunchu_zuo[2] = speed_temp_11[1] + speed_temp_12[1] + speed_temp_21[1] + speed_temp_22[1] ;  // ��ĸ             
     
       Fuzzy_speed_cunchu_zuo[0] = Fuzzy_speed_cunchu_zuo[1] / Fuzzy_speed_cunchu_zuo[2] ;
       
        Fuzzy_speed_cunchu_you[1] =  speed_Fuzzy_you_dan[ speed_temp_11[0] ] * speed_temp_11[1]   // ����
                                                     + speed_Fuzzy_you_dan[ speed_temp_12[0] ] * speed_temp_12[1]
                                                     + speed_Fuzzy_you_dan[ speed_temp_21[0] ] * speed_temp_21[1]
                                                     + speed_Fuzzy_you_dan[ speed_temp_22[0] ] * speed_temp_22[1] ;
       Fuzzy_speed_cunchu_you[2] = speed_temp_11[1] + speed_temp_12[1] + speed_temp_21[1] + speed_temp_22[1] ;  // ��ĸ             
     
       Fuzzy_speed_cunchu_you[0] = Fuzzy_speed_cunchu_you[1] / Fuzzy_speed_cunchu_you[2] ;
       
     //  Fuzzy_speed_cunchu_you[0] = ( Fuzzy_speed_cunchu_you[0] + Fuzzy_speed_cunchu_you[5]) / 2 ; // ��ƽ��ֵ���� �˲�
        
        
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
                
                           
//==================   ƫ��  �޷�����  ============== //   
/* */
    
          servo_output = ( 80*servo_output + 16*servo_output_last[0] + 3*servo_output_last[1] + 1*servo_output_last[2] ) / 100 ;
 
   //SCI0_SendChar_16( servo_output );
         
}



//========================    zhuanxiang  ����  ת��      ======================================//
void zhuanxiang()
{

      uint8_t i;
      int16_t duoji_value;
      
              servo_max=0;
        
      for(i=0;i<12;i++)//Ѱ����һ�ε���������
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
         	jiao_biao_min=i;//Ѱ����һ�ε���С�����	
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
		 //�������� 	
         //=======================================================//
           }
         	
		 
	      lost_signal();
 	     //noise_protect_second(); 
        
 
          zhuan_xiang_control() ;
           
   /*        
           
          if( (servo_output - servo_output_last[0]) > 50 )    //======= �����޷���С����5��11 ���� +3 ��+7=====//
	  {
	        servo_output_flag[3] = 0 ;
	        servo_output_flag[4] = 0 ;
	        servo_output_flag[5] = 0 ;
	        if( servo_output_flag[0] < 1 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 5 ����
	        {
	        	servo_output=servo_output_last[0]+37 ; // 1
	        	servo_output_flag[0]++;
	        }
	  	else  if( servo_output_flag[1] < 2 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		servo_output=servo_output_last[0]+54 ; // 1
	  		servo_output_flag[1]++;
	  	}
	       else  if( servo_output_flag[2] < 3 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		servo_output=servo_output_last[0]+67 ; // 1
	  		servo_output_flag[2]++;
	  	}
	  	else if( (servo_output-servo_output_last[5]) < 400 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 6 �����ڻ�δ�ع�
	  	       servo_output=servo_output_last[5] + 320 ;
	  	else  
	  	       servo_output=servo_output_last[0]+60 ; // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���� 7 �����ڻ�δ�ع�
	  } 
 	  else if( (servo_output - servo_output_last[0]) < -50 )   //======= �����޷���С���� -5��-11 ���� -3 ��-7=====//
 	  {
	        servo_output_flag[0] = 0 ;
	        servo_output_flag[1] = 0 ;
	        servo_output_flag[2] = 0 ;
	        if( servo_output_flag[3] < 1 )  // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 5 ����
	        {
	        	servo_output=servo_output_last[0]-37 ; // 1
	        	servo_output_flag[3]++;
	        }
	  	else  if( servo_output_flag[4] < 2 ) //// array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		servo_output=servo_output_last[0]-54 ; // 1
	  		servo_output_flag[4]++;
	  	}
	  	else  if( servo_output_flag[5] < 3 ) //// array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 11 ����
	  	{
	  		servo_output=servo_output_last[0]-67 ; // 1
	  		servo_output_flag[5]++;
	  	}
	  	else if( (servo_output-servo_output_last[5]) > -400 ) // array_lvbo_flag[0][0]  һ�ŵ�е�ƫ���Ƿ��� 6 �����ڻ�δ�ع�
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
          
          if(servo_output>700)  //�޷�����
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
//============================= ���źŴ�����==========================//
//ϸ������!!!!!
//���裺
//1����������
//2��ȷ������λ����ת���Լ����ֵ��������ϵ
//3����дִ�к������Բ�ѯ�ķ�ʽ��������ֵ
void lost_signal()
{

//diangan_min[0] lost_signal_flag 
 if(   Road_Type[0]==1)//�����
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
else if(  Road_Type[0]==2)//�����
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
//==========================ת������Ӧ===============================//
//(103+speed_ave/k)*servo_output/100;
//���ڳ�����ת��ǵ�һ��һ�κ����仯��
//������ת��Ĳ������棬���ܹ��õ�����һ�κ�������
//�ֶε��ԣ�������ϸ�����ڲ�ͬ�ĳ������ò�ͬ��ϵ��������������ȷ��ϵ��
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
//=============================  ����    ����  ==========================//
//=======================================================================//
void geng_xin_shuju()
{
      
     // ���1��������
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

      // ���2��������
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

      // ���3��������
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
       
      // ���4��������//��ֱ1��
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
       
       // ���5��������//��ֱ2��
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
       
        //����ƫ��ֵ
        
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
       
    //=================����ƫ����=================//
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
       
          //������һ�ε�ƫ������˲�
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
       
       //���������
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
   
         // ������ˮƽ���ֵ֮��
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
         // ������  ��ֱ���ֵ֮��
 
         //��������еĴ���
         cixu[3][0]=cixu[2][0];
	     cixu[2][0]=cixu[1][0];
	 	 cixu[1][0]=cixu[0][0];
		//���´δ��еĴ���
		 cixu[3][1]=cixu[2][1];
		 cixu[2][1]=cixu[1][1];
		 cixu[1][1]=cixu[0][1];
		//������С��еĴ���
		 cixu[3][2]=cixu[2][2];
		 cixu[2][2]=cixu[1][2];
		 cixu[1][2]=cixu[0][2];
		 
		 //���������ֵ
		 
		 
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
         
         //������С���ֵ
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
         
		 //������������
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
        
        // CCD �� �� �ڵ���� ����
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



 
//��������
//С��ÿ��100cm����1000.18�����壬ʡ��Ϊ1000��
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
   //�������ܣ���¼�ս��������λ��
   if(Road_Type[1]!=1&&Road_Type[0]==1)
   {
   	 distance[0]=(CCD_juli_zuo[0]+CCD_juli_you[0])/2;//��ȡ��ǰλ��
   	 zhijiao_charge[0]=1;                //����������־λ
   	 chuizhi_point[0]=qu_zheng(chuizhi_piancha[0]) ;//��ȡ��ǰ��ֱλ������ȡ��ֵ
   	 xiezhi_point[0]=qu_zheng(xiezhi_piancha[0]) ;//��ȡ��ǰб��λ������ȡ��ֵ
  	 
   	
   	
   }
   else if(Road_Type[1]!=2&&Road_Type[0]==2)
   {
   	 distance[0]=(CCD_juli_zuo[0]+CCD_juli_you[0])/2;//��ȡ��ǰλ��
   	 zhijiao_charge[0]=1;                //����������־λ
   	 chuizhi_point[0]=qu_zheng(chuizhi_piancha[0]) ;//��ȡ��ǰ��ֱλ������ȡ��ֵ
   	 xiezhi_point[0]=qu_zheng(xiezhi_piancha[0]) ;//��ȡ��ǰб��λ������ȡ��ֵ
  	
   }
    
 //=========================ֱ���ж�=========================//   
  if((zhijiao_charge[0]==1)&&(Road_Type[0]==1||Road_Type[0]==2))
  {
         //��С���߹�15CMʱ�����������
  	     if((CCD_juli_zuo[0]+CCD_juli_you[0])/2>distance[0]+150)
  	     { 
   	        chuizhi_point[1]=qu_zheng(chuizhi_piancha[0]) ;//��ȡ��ǰ��ֱλ������ȡ��ֵ
   	        xiezhi_point[1]=qu_zheng(xiezhi_piancha[0]) ;
  	        zhijiao_charge[0]=0;//��־λ���㣬���ٽ���˳��򣬵ȴ���һ�Ρ����� 
  	        //����б��
  	        distance[1]=(CCD_juli_zuo[0]+CCD_juli_you[0])/2;
  	        zhijaio_point_k[0]=20*qu_zheng(chuizhi_point[1]-chuizhi_point[0])/
  	                               (xiezhi_point[1]+xiezhi_point[0])  ;
  	                               
  	         
  	         if(zhijaio_point_k[0]>10) zhijiao_charge[1]=1;//ֱ�Ǽ���
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
	           CarSpeed_SET_L=0;//����ɲ��������С��
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
   
  //========================��߳���==============================//   
   if(chasu==1)
   {      if(servo_output>0)
		   {          //����ת,����ת����
		      CarSpeed_SET_L=wandao_CarSpeed_SET_L-70*cha_su_value/100 ;              
		   }     
	       else  if(servo_output<0)
		   {       //����ת,����ת���Ŀ�
		       CarSpeed_SET_L=wandao_CarSpeed_SET_L+k1*cha_su_value/100;                                 
		   }
           return CarSpeed_SET_L;  
   }
   
   
   
   
 //=========================�ұ߳���============================//  
  else if(chasu==2)
  {
  	if(servo_output>0)
    {        //����ת,����ת����
		   CarSpeed_SET_R=wandao_CarSpeed_SET_R+k1*cha_su_value/100;
    }     
    else if(servo_output<0)
    {        //����ת,����ת���� 
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
     
 //===============================�ٶȿ���=========================//    
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
 	
 	 	
	SIU.GPDO[PCR40_PC8].R = 1 ;       //1-0��ת
    SIU.GPDO[PCR44_PC12].R = 0 ;
    motor_pwmR= 7000;
                      
    SIU.GPDO[PCR41_PC9].R = 1;
    SIU.GPDO[PCR47_PC15].R = 0;
    motor_pwmL= 7000;
 			      
 	
	
 }
 else if(speed_ave> (CarSpeed_SET_R+ CarSpeed_SET_R)/2+700)
 {	
 	
 	 	
	SIU.GPDO[PCR40_PC8].R = 1 ;       //1-0��ת
    SIU.GPDO[PCR44_PC12].R = 0 ;
    motor_pwmR= 5500;
                      
    SIU.GPDO[PCR41_PC9].R = 1;
    SIU.GPDO[PCR47_PC15].R = 0;
    motor_pwmL= 5500;
 			      
 	
	
 }
 
 else if(speed_ave> (CarSpeed_SET_R+ CarSpeed_SET_R)/2+300)
 {	
 	
 	 	
	SIU.GPDO[PCR40_PC8].R = 1 ;       //1-0��ת
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
////////////////////////////PID����ѡ��//////////////////////////////
//===============================���================================
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
        if( piancha_E_jisuan==8)  // ��������
        {
        	  //============== ��һ�� ========================
                guiyi_MAX[0] = 200 ;//195  238
		        guiyi_MAX[1] = 240 ;//183  220
			    guiyi_MAX[2] = 240 ;//183   225
		        guiyi_MAX[3] = 235 ;//181   223
		        guiyi_MAX[4] = 245 ;//179   220
		        guiyi_MAX[5] = 245 ;//179   220
		        
		               Show_Me_Data(2500,50,0);  //�趨ƽ���ٶ�
                       Show_Me_Data(1,1,1);      //�ٶȵ�λ
                       Show_Me_Data(400,50,2);  //��P����ʱ���趨 
                       
                       Show_Me_Data(3,1,13) ;       //ѡ���Ƿ��ϰ�     3��������   CCD_xuanze_flag
                       if( CCD_xuanze_flag==3 )
	                   {
	             		Show_Me_Data(1,1,19) ;    //CCD �ع�ʱ��    /cm   
	             		Show_Me_Data(5,1,16) ;    //���� ���� ���� ���� ����     /cm  
	             		Show_Me_Data(101,20,17) ;    //   ��������   ����  CCD    CCD_juli_dm_she_ding_juli[2]>2 ����
	             		if( CCD_juli_dm_she_ding_juli[2]>2 )	//    ��������   �ر�  CCD
	             			Show_Me_Data(401,20,18) ;   //
	                    }
	               
	               Show_Me_Data(50,10,22) ; 
	               Show_Me_Data(7,1,5) ;       //  ���� E ѡ��
        
        
        }
        else   // ���Գ���
        {
        	  //============== ��һ�� ========================
                 guiyi_chushihua() ;  
                 Show_Me_Data(2500,50,0);  //�趨ƽ���ٶ�
	             Show_Me_Data(1,1,1);      //�ٶȵ�λ
	             Show_Me_Data(400,50,2);  //��P����ʱ���趨 
	 	         SIU.GPDO[PCR38_PC6].R =0 ;//��0����  
	 	   
	                 //==============================================        		       		
	             Show_Me_Data(3,1,13) ;       //ѡ���Ƿ��ϰ�     3��������   CCD_xuanze_flag
	             if( CCD_xuanze_flag==3 )
	             {
	             		Show_Me_Data(1,1,19) ;    //CCD �ع�ʱ��    /cm   
	             		Show_Me_Data(5,1,16) ;    //���� ���� ���� ���� ����     /cm  
	             		Show_Me_Data(101,20,17) ;    //   ��������   ����  CCD    CCD_juli_dm_she_ding_juli[2]>2 ����
	             		Show_Me_Data(401,20,18) ;   //
	             }
	             
	              Show_Me_Data(50,10,22) ;
	               Show_Me_Data(7,1,5) ;       //���� E ѡ��

	               //Show_Me_Data(277,20,15) ;   //���Ծ��� ͣ��     /cm      <27 ��������ͣ��
	                                                                                 //    Show_Me_Data(5900,2,14) ;    //�������ֵ   /cm  

        }

}


void CCD_xuanze( void )
{
	    //=============      �ж���ֱ��   ����   ���б���   =======================
          if( CCD_bizhang_flag==0 && podao_flag==0)
           {
	       //  if( Road_Type[0]==0 )
	          if( CCD_xuanze_flag==3 && array_current[4]<20 && array_current[5]<20 && chuizhi_piancha[0]<4 && chuizhi_piancha[0]>-4 )  // 
	  	    {        // Ϊ��ȷ���Ƿ���ֱ�� array_current[1]>array_current[0] && array_current[2]>array_current[3] &&
		           	if( ( array_current[1]>40 &&array_current[1]<300 ) )   // 
		          	{                                                                                                                     //      δ���ź�                            δ���µ�          
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
	    
	   //================             ���������     ===================	
	      if( CCD_bizhang_flag!=0 )
	      {         //  !=0 ˵�� �� ���ϰ���
			//   	CarSpeed_SET_R=0 ;    CarSpeed_SET_L=0 ;
	         	SIU.GPDO[PCR38_PC6].R=0 ;
			
			    CCD_juli[0]=( CCD_juli_zuo[0]+ CCD_juli_you[0]-CCD_juli_zuo[1] -CCD_juli_you[1] )/2 ;
			    if( CCD_juli[0]>CCD_juli_dm_she_ding_juli[0] ) //  ������ʱ ����
			    {
				     CCD_bizhang_flag=0 ; //  =0 ˵�� �뿪�ϰ���
				     CCD_juli_flag=0 ; // ��֤ CCD_juli_zuo[1]  ֻ����һ�θ�ֵ
			    }
	      }
	      else
		        SIU.GPDO[PCR38_PC6].R=1 ;      
			      //=====================================================       
		 if( CCD_bizhang_flag==1 )    //   �����ֵ
			    duoji_zhongzhi=duoji_zhongzhi_value-300 ;  //   5443  5843    6243
		 else if( CCD_bizhang_flag==2 )
			    duoji_zhongzhi=duoji_zhongzhi_value+300 ;  // 
		 else duoji_zhongzhi=duoji_zhongzhi_value ;  //6915      
		 
	     //=============      �ж��Ƿ����µ�   ����   ���б���   =======================	 
	     if( array_current[1]>250 && array_current[2]>250 )  podao_flag=1 ; 
	     if( podao_flag==1&&podao_flag_dingju==0 )    
	     {
	     		podao_flag_dingju=1 ;
	     		CCD_juli_zuo[2]=CCD_juli_zuo[0] ;   CCD_juli_you[2]=CCD_juli_you[0] ; 
	     }
	     if( podao_flag_dingju==1 ) 
	     {  
	     		CCD_juli[2]=( CCD_juli_zuo[0]+ CCD_juli_you[0]-CCD_juli_zuo[2] -CCD_juli_you[2] )/2 ;
	     		if( CCD_juli[2]>2000 ) //  ������ʱ ����
			{
			         podao_flag_dingju=0 ; //  =0 ˵�� �뿪�µ���
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
				                    
				             //      SIU.GPDO[PCR38_PC6].R =!SIU.GPDO[PCR38_PC6].R ;//��0����
				           }   
			           
		      */ 
		      
		      
}

void ding_ju_tingche( void )
{
	 //=================          �Ƿ񶨾���ͣ��      ==============================       
			     
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
//======================= gui_yi���� ��һ��ʼ�� =======================//
void guiyi_chushihua( void )
{
	uint8_t  guiyi_xuhao_flage_1 = 0 , guiyi_xuhao_flage_2 = 0 , guiyi_xuhao_flage_3 = 0 ;
	uint8_t  guiyi_xuhao_flage_4 = 0 , guiyi_xuhao_flage_5 = 0 , guiyi_xuhao_flage_6 = 0 ; 
	uint8_t  guiyi_xuhao_flage_7 = 0 , guiyi_xuhao_flage_8 = 0 , guiyi_xuhao_flage_9 = 0 ; 
	uint8_t  guiyi_xuhao_flage_10 = 0 , guiyi_xuhao_flage_11 = 1 , guiyi_xuhao_flage_12 = 0 ; 

	while( guiyi_xuhao_flage_11 )
	{
	       Show_Me_Data(7,1,6) ;//����׼��    
	       // ====================================================  
	       if( diangan_biaoding_xuhao==9 ) //  1
		{
		        while( 1 )
		        {
		        	Show_Me_Data(1,1,6);//����׼�� 
		        	guiyi_MAX[ diangan_biaoding_xuhao -1 ] = diangan_guiyihua( diangan_biaoding_xuhao);  // �ҳ�ˮƽ1�ŵ�е����ֵ   
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
		        Show_Me_Data(1,1,6);//����׼�� 
			guiyi_MAX[ diangan_biaoding_xuhao -1 ] = diangan_guiyihua( diangan_biaoding_xuhao);  // �ҳ�ˮƽ1�ŵ�е����ֵ   
                       Show_Data(guiyi_MAX[diangan_biaoding_xuhao-1]) ;
                   //   delay400ms();
                      guiyi_xuhao_flage_1 = 1 ;
                
                   	diangan_biaoding_xuhao=2 ;   //	Show_Me_Data(2,1,6);//����׼�� 
			guiyi_MAX[ diangan_biaoding_xuhao -1 ] = diangan_guiyihua( diangan_biaoding_xuhao);  // �ҳ�ˮƽ1�ŵ�е����ֵ   
                      Show_Data(guiyi_MAX[diangan_biaoding_xuhao-1]) ;
                  //    delay400ms();
                      guiyi_xuhao_flage_2 = 1 ;
               
                       diangan_biaoding_xuhao=3 ; 	//Show_Me_Data(3,1,6);//����׼��
			guiyi_MAX[ diangan_biaoding_xuhao -1 ] = diangan_guiyihua( diangan_biaoding_xuhao);  // �ҳ�ˮƽ1�ŵ�е����ֵ   
                      Show_Data(guiyi_MAX[diangan_biaoding_xuhao-1]);
               //       delay400ms();
                      guiyi_xuhao_flage_3 = 1 ;
		
		       diangan_biaoding_xuhao=4 ;	       //Show_Me_Data(4,1,6);//����׼�� 
			guiyi_MAX[ diangan_biaoding_xuhao -1 ] = diangan_guiyihua( diangan_biaoding_xuhao);  // �ҳ�ˮƽ1�ŵ�е����ֵ   
                      Show_Data(guiyi_MAX[diangan_biaoding_xuhao-1]);
                      delay400ms();
                      guiyi_xuhao_flage_4 = 1 ;
		
		        Show_Me_Data(5,1,6);//����׼�� 
			guiyi_MAX[ diangan_biaoding_xuhao -1 ] = diangan_guiyihua( diangan_biaoding_xuhao);  // �ҳ�ˮƽ1�ŵ�е����ֵ   
                      Show_Data(guiyi_MAX[diangan_biaoding_xuhao-1]);
                 //     delay400ms();
                      guiyi_xuhao_flage_5 = 1 ;
                      
                      diangan_biaoding_xuhao=6 ;     //   Show_Me_Data(6,1,6);//����׼�� 
			guiyi_MAX[ diangan_biaoding_xuhao -1 ] = diangan_guiyihua( diangan_biaoding_xuhao);  // �ҳ�ˮƽ1�ŵ�е����ֵ   
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
///////////////////////////////////������/////////////////////////////////
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
      //while(motor_time<10000);//��ʱ5s
      // motor_time=0;
      
      //==========��=========//          
      SIU.PCR[PCR38_PC6].R =  0x0200;  //GPIO[38] is output
      SIU.GPDO[PCR38_PC6].R =1;//��0����
//==============================�궨====================================//       
    

       	//===============  ѡ���ٶ� ���� ���� ������ ================= 
        yejingping_xuanze( ) ;   

                                    
//===================================================================//       
        SIU.GPDO[PCR38_PC6].R =1;//��0����       
        last_CarSpeed_SET_R=CarSpeed_SET_R; //��������ٶ�
        last_CarSpeed_SET_L=CarSpeed_SET_L;
      
        Write7219(Shutdown_addr,0X00);       //�ر��������ʾ//ͣ��
      //��ʱ���뷢��
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
      CCD_juli_dm_she_ding_juli[0]=CCD_juli_dm_she_ding_juli[0]*100 ;  //  1dm  �����ϰ���ʱҪ�ߵľ���
      CCD_juli_dm_she_ding_juli[1]=CCD_juli_dm_she_ding_juli[1]*100 ;  //  1dm  �Ƿ񶨾���ͣ��  ��һ�ξ���ͣ�����ﳵ3m�������
      CCD_juli_dm_she_ding_juli[2]=CCD_juli_dm_she_ding_juli[2]*100 ;  //  1dm  17    ��������   ����  CCD
      CCD_juli_dm_she_ding_juli[3]=CCD_juli_dm_she_ding_juli[3]*100 ;  //  1dm  18    ��������   �ر�  CCD

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
               geng_xin_shuju();//  ��������
               speed_control_new(); //�ٶȿ��� 
               protect();
        
               stop_smartcar();    
                 
                 
                 if( CCD_juli_dm_she_ding_juli[1]/100 > 27 )    // ѡ���Ƿ���Ҫ��������     > 100 cm
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
/////////////////////////////���Լ�¼////////////////////////////////////
//======================================================================//
//2.28-------���ݳɹ�
//3.1--------32M���ٳɹ�����һ���Ż�����
//3.3--------PID����
//3.9--------PID�ɹ�����Ӧ�ܿ죬
//3.10-------���ܵĳ����Ƕ�ϣ����ֲ���������Ƶ�ʳ�ͻ
//3.15-------������⣬����16M���ٲ���PID���٣�״������
//-----------ʵ�ֿ��Դ�PID�ļ򵥿���
//3.16-------����ʵ��·�̼�����֣�ת���Ϊÿ��2CM����
//3.17-------�������ĳ���ʵ�֣������Ż�
//3.18-------б�õ����ˮƽ��е�����ʹ�ã���ĿǰΪֹ������ں�
//-----------ʵ�ֿ��ܣ�״̬����
//3.19-------���ʱ��ĵ����ܽ᣺
//1.���������Ѿ��Ƚϳ��죬���������廹�д���ߣ�����һ���������ź�
//����ϢϢ��أ����ڼ�ǿ�źŵĴ�����ν��򵥵ĵ��ֵת���ɿ��԰���
//ȫ�ֵĵ�һ�����������Ҫ.
//2.��3.20��ʼ��3.29������Ϣ�Ĳɼ��Լ������������������������
//�о��㷨��ͬʱע���Ŷ�Э�����ֹ���ȷ��Ҫ�ﵽ��Ŀ�ĳ�����������
//����Ϣ����ģ��
//3.�ڶ�����ģ�Ĵ����������Զǰհ����
//3.21-------��ã����֪������δ������ݡ���������ͣ�Ͳ�ǰ����������������
//3.22-------���������λ�ã�����ȷ���ŵ�����������
//3.23�뷨��С��ÿ��һ�β����������Ƕ��٣---5��10�����뻹Ҫ��ȷ����2014
//3.24�������С���ĸ�װ����5����У�3��ˮƽ2����ֱ����һ������ƫ������
//����.���Ͽ����������Ƶ�����д��������Ǹ�ȫ���Ը��ˣ���������������
//3.25С�����ڿ����������ˣ�ֱ�����Թ�ȥ�����ǻ����һ�����ơ�
//3�µ�һ������̬������,��֮�����ͣ���������������������������
//�������⣺������г�ȥ��ƫ�������������⣬̫С��ת�򲻹�
//3.27�����������˺ܶ࣬���ǻ�û��������ˮ��©��ʮ��������жϳ���
//��Ҫ��һ����ȡ���ú���Ϣ��ǰ������Ϣ��׼�ģ�������Ϣ����ȡ��Ȼ����λ��
//ͬʱ��߳����������������뷨��Ҫ������ʵʩ��һ��Ҳ�������ӣ���
//3.28��������������һȦ�����µ��������ˣ�����Ϊ������С�ˣ��Ժ�Ҫ
//�������������棬��ʱ�����ʱ�䲻���ˣ�����ע���ŶӺ������ӿ����
//Ҫ��Ҫ��.��һ������ѭ��(����ƫ���ת��),ͬʱ��߻�������߿��Ʋ��ԣ������
//3.29-4.8��ʮ����������֮��һ�����⣬ͬ���ĳ��������������в��õ����֣������
//���֣����⳵���ٲ�׼����������ʦ������û�����⣬��������û�в����������
//����ʱ����ȣ����Ի��˱�����������״�����ã�ͬʱ����������ܷɵ����⣬ԭ������
//�޷�̫�󣬲��ܹ���ӳ�ٶȵ�ͻ��
//4.9���������ֱ���������Ժ���͹�һ����Ժ�������쵼�ιۣ����ݳɹ�����һ����ʼ���˲�
//����������Ϣ����
//4.11������������ܳ�34.5m��ȫ�������ʱ14.984��ƽ���ٶ�2.302m/s
//4.15����������������ȫ��37.26m�������ʱ16.122s,ƽ���ٶ�2.32��ʮ����������
//���������������֮��ġ�����ʮ�֡���ͨ����Сģ�����Ƶ������״�����˺ܶ�
//��һ�����׽��ʮ�ֵ�������������������������������
//4.23����д����ʶ��Ч�����˺ܶ࣬�ж��ȶ��˺ܶ�
//���������˲���ת��ʮ���䣬һ��Ҫʹ������̬�ȶ����
//5.3������㷨��ƫ������ȷ����PID���ٽ�һ����ǿ
//��������ʶ�𣬿����ţ��ȶ�+����
//========================================================================//
////////////////////////////��ս������//////////////////////////////////////
//7.6-7.7----------ȷ���ܷ�
//7.8-7.10---------ƫ�����Լ�ת��PD��
//7.11-12----����
//7.13-17----��������
