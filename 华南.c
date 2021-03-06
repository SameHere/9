//**************************************************************************//
//******************************2014电磁程序*****************************//
//                        Copyright by 杨粒&柴亮亮   2014                             //
//*************************************************************************//
//*************************************************************************//

#include "MPC5605B.h" /* Use proper include file */
#include "MyHeader.h"
//===============================外部函数=====================================//
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


 //==========================调用的函数======================================//
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
void StartInte1(void) ; // 曝光
void ImageCapture1(uint8_t *Data1); //采样
void SendImageData(uint8_t *ImageData); // 上位机
void SendHex(uint8_t hex);//
void CCD_bizhang( void ) ;
//*********************************采样变量************************************//
#define TSL_SI1              SIU.GPDO[PCR63_PD15].R    //定义线性传感器的端口 SI  27
#define TSL_CLK1           SIU.GPDO[PCR31_PB15].R    //定义线性传感器的端口 CLK  7
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
uint32_t stop_times = 0 , shijian_ting_che=0 , shi_jian_ceshi_she_ding_shijian=0  ;     //PIT计数器
uint32_t CCD_times=0;
uint32_t CCD_stoptimes=0;
//uint32_t ccd_times=0; //   =1 说明已经过障碍
uint32_t ccd_stoptimes=0;
uint32_t distance[10]={0};
uint32_t zhijiao_charge[5]={0};
uint32_t chuizhi_point[2]={0};
uint32_t xiezhi_point[2]={0};
uint32_t zhijaio_point_k[10]={0};
uint32_t CCD_juli_zuo[5]={0} , CCD_juli_you[5]={0} , CCD_juli[5]={0} , CCD_juli_dm_she_ding_juli[10]={0}  ;     //PIT计数器
uint8_t  CCD_juli_flag=0 , CCD_baoguang_shijian=10 , CCD_jiance_you_zhangai_flag=0 , podao_flag=0 , podao_flag_dingju=0 ;
//*********************************转向变量********************************//
int16_t Mid1[10]={0};
uint16_t variance1=0,variance2=0,variance=0;

uint8_t Fazhi_R11,Fazhi_R12,Fazhi_L11,Fazhi_L12;
uint8_t Max_pixel1,maxi1,Min_pixel1,mini1;
int16_t left11,right11,mid11,left12,right12,mid12,mid1=64,mid_x1;
uint8_t midcnt1=0  ; // 0 无障碍     1 左  2 右
uint16_t exce_pixel1;
//********************************延时***********************************//
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

//==========================  dian_gan  ==   电感       ===============================//
#define   DIAN    42     //  初始采样    一个周期一行采集的点
#define   guiyi_DIAN    80      //   归一化中  一个电感采的值
uint8_t flag_2;
int16_t array_one[50]={0};    // 1   五个电感采回来的值  
int16_t array_two[50]={0};    // 2
int16_t array_three[50]={0};  // 3
int16_t array_four[50]={0};   //  4
int16_t array_five[50]={0};    //  5  
int16_t array_six[50]={0};    //  5 
int16_t array_seven[50]={0};    //  5                 
int16_t array_current[8]={0};//  存储电感的值便于计算
int16_t guiyi_wugediangan_MAX = 0 ;
int16_t speed_Fuzzy_tiaosu_flag = 0 ;
uint8_t array_lvbo_flag[6][8] = { { 0 } , { 0 } , { 0 } , { 0 } , { 0 } , { 0 } } ; //滤波标志位  ——array_lvbo_flag[7] —— 确定是哪个电感  
                                                           //                 —— array_lvbo_flag[7][2] ——选择限幅的大小

//=================  pian_cha___xinhao  ==   偏差——信号        ===============================//
#define   Error_lishudu_Max   200      // 模糊控制中——偏差——隶属度最大值   ==> 隶属度  0---200
#define   Error_c_lishudu_Max   200  //  模糊控制中——偏差变化率——隶属度最大值
uint8_t  diangan_biaoding_xuhao = 1 ;  // 归一化 标定电感时 ， 选择标定第几个电感
uint16_t guiyi_MAX[8] ={ 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0};  // 归一化标定后，每个电感的最大值
int16_t servo_Error[100] = {0} ;  //  存储每次用过的 偏差值
int16_t  servo_aaaaa[2];
int16_t servo_Error_c[50] = {0} ; // 存储每次用过的偏差变化率的值
uint8_t servo_Error_flag[10]={    0 , 0 , 0 , 0 , 0  };  // 上次垂直电感的偏差
uint8_t servo_output_flag[10]={    0 , 0 , 0 , 0 , 0  };  // 上次垂直电感的偏差
                    
int16_t abs_sum_[10]={0}; // 存储  十组偏差  的  绝对值  的   和
int16_t abs_sum=0;   //绝对值和   
int16_t daishu_sum=0;//代数和
uint16_t diangan_min[20]={0} ;
int16_t  middle_piancha[20]={0} ;
uint16_t diangan_maxfour[30]={0} ;  // 提取三个水平电感中的最大值uint8_t lost_signal_flag=0;
uint16_t diangan_max[30]={0} ;  // 提取三个水平电感中的最大值
uint16_t chuizhidiangan_max[20]={0} ;
uint16_t  total_diangan_sum[20]={  0 , 0 , 0 , 0 , 0  }  ;  
uint16_t  Verticaldiangan_sum[20]={  0 , 0 , 0 , 0 , 0  }  ;  
uint16_t  Leveltaldiangan_sum[20]={  0 , 0 , 0 , 0 , 0  }  ;
uint16_t  xiezhi_diangan_sum[20]={  0 , 0 , 0 , 0 , 0  }  ;  
 
uint8_t dian_gan_cixu = 0 ; // 判断三个水平电感中哪个电感的值最大
int16_t servo_output_last_max = 0 ; //  找出上次舵机输出的最大值
uint8_t cixu[10][3]={0};  // 将五个电感的值进行排序
int16_t wandao_sppch[12]={0};
int16_t shuiping_piancha[12] = {    0 , 0 , 0 , 0 , 0  };   // 每次采完样，计算出来的水平电感的  偏差
uint8_t shuiping_piancha_flag[10] = {    0 , 0 , 0 , 0 , 0  };  //  上次水平电感的偏差 
int16_t xiezhi_piancha[12] = {    0 , 0 , 0 , 0 , 0  };
int16_t chuizhi_piancha_fs[12]= {    0 , 0 , 0 , 0 , 0  };
int16_t chuizhi_piancha[12]= {    0 , 0 , 0 , 0 , 0  };  // 每次采完样，计算出来的垂直电感的  偏差
uint8_t chuizhi_piancha_flag[10]={    0 , 0 , 0 , 0 , 0  };  // 上次垂直电感的偏差
int16_t chuizhi_jiaquan[12];
int16_t shuiping_piancha_xishu=0 , chuizhi_piancha_xishu=0 , xiezhi_piancha_xishu=0 ;
uint8_t piancha_E_qiujie_flag=7 ;
//=======================servo —— mohu——舵机——===============================//
#define   servo_Upper     830   // 舵机输出值  上偏差
#define   servo_Lower    -830   // 舵机输出值  上偏差
uint16_t  duoji_zhongzhi =6420 ,  duoji_zhongzhi_value =6420;// 6915 //      
uint16_t  servo_kp = 15 ;   // 舵机 PD 控制中的 P 值
uint16_t  servo_kd = 0 ;     // 舵机 PD 控制中的 D 值
uint16_t  Fuzzy_Kp_cunchu[7] ={ 0,0,0,0,0,0,0 } ;  // 存储模糊控制中 Kp 计算过程中的值，并存储上一次的值
int32_t  Fuzzy_Kd_cunchu[7] ={ 0,0,0,0,0,0,0 } ;  // 存储模糊控制中 Kd 计算过程中的值，并存储上一次的值
int16_t servo_Fuzzy_Error[7] = { -87 ,  -65 , -27 ,  0 , 27 ,  65 , 87 } ; // 偏差分区     目前觉得两头小中间大，效果较好 //   直道不稳  kp 80 , 69 , 42 , 0 , 42 , 62 , 80 
//int16_t servo_Fuzzy_Error[7] = { -87 ,  -50 , -25 ,  0 , 25 ,  50 ,  87 } ; // 偏差分区     目前觉得两头小中间大，效果较好 //   直道不稳  kp 80 , 69 , 42 , 0 , 42 , 62 , 80 

 //int8_t servo_Fuzzy_Error_c[7] = { -9 , -6 , -3 , 0 , 3 , 6 , 9 } ;     // 偏差变化率   分区
int8_t servo_Fuzzy_Error_c[7] = { -15 , -11 , -7 , 0 , 7 , 11 , 15 } ;     // 偏差变化率   分区
                                        //  -87 ,  -51 , -20 ,  0 , 20 ,  51 ,  87  // 110 , 95 , 80 , 40 , 80 , 95 , 110  // 可以
                                                             //  -90 ,  -59 , -27 ,  0 , 27 ,  59 ,  90    //120 , 100 , 55 , 30 , 55 , 100 , 120  弯道有点晚
const uint16_t servo_Fuzzy_Kp_dan[7] = { 75 , 65 ,  40 , 0 , 40,65 , 75} ;  // Kp区域大小   70 , 50 , 20 , 10 , 20 , 50 , 70
//76 , 63 ,  45 , 0 , 45,63 , 76
/*const uint8_t servo_Fuzzy_Kp_rule[7][7] =       // Kp 规则表
{///  0--1--2--3--4--5--6                                                                                                                                //
	6 , 6 , 5 , 5 , 4 , 4 , 3 ,//0     //
	6 , 6 , 5 , 5 , 4 , 3 , 3 ,//1     //
	5 , 5 , 5 , 4 , 3 , 2 , 1 ,//2     //
	5 , 5 , 4 , 3 , 2 , 1 , 1 ,//3     //
	5 , 4 , 3 , 2 , 1 , 1 , 1 ,//4     //
	3 , 3 , 2 , 1 , 1 , 0 , 0 ,//5     //
	3 , 2 , 2 , 1 , 1 , 0 , 0 //6      //
};*/

const uint8_t servo_Fuzzy_Kp_rule[7][7] =       // Kp 规则表
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
const uint8_t servo_Fuzzy_Kp_rule[7][7] =       // Kp 规则表
{///  0--1--2--3--4--5--6                                                                                                                                //
	6 , 6 , 5 , 5 , 4 , 4 , 3 ,//0     //
	6 , 6 , 5 , 5 , 4 , 3 , 3 ,//1     //
	5 , 5 , 5 , 4 , 3 , 2 , 1 ,//2     //
	5 , 4 , 4 , 3 , 2 , 1 , 1 ,//3     //
	4 , 4 , 3 , 2 , 2 , 1 , 1 ,//4     //
	3 , 3 , 2 , 1 , 1 , 1 , 0 ,//5     //
	3 , 2 , 2 , 1 , 1 , 0 , 0 //6      //
};*/




 //   const uint16_t servo_Fuzzy_Kd_dan[7] = { 1 , 1 , 1 , 1 , 1 , 1 , 1 } ;    // Kd 区域大小
 //  const uint32_t servo_Fuzzy_Kd_dan[7] = { 5000 , 4000 , 3000 , 1000 , 3000 , 4000 , 5000 } ;
   const uint16_t servo_Fuzzy_Kd_dan[7] = { 90 , 70 , 30 , 10 , 30 , 70 , 90 } ;

 //   const uint16_t servo_Fuzzy_Kd_dan[7] = { 2900 , 2500 , 2300 , 2000 , 2300 , 2500 , 2900 } ;
/*const uint8_t servo_Fuzzy_Kd_rule[7][7] =      //  Kd  规则表
{ //   0--1--2--3--4--5--6
	4 , 4 , 3 , 3 , 3 , 6 , 6 ,  //  0
       2 , 2 , 3 , 4 , 4 , 4 , 5 ,  //  1
       6 , 6 , 1 , 2 , 3 , 4 , 5 ,  //  2
       0 , 1 , 2 , 3 , 4 , 5 , 6 ,  //  3   
       1 , 2 , 3 , 4 , 5 , 0 , 0 ,  //  4
       1 , 2 , 2 , 2 , 3 , 4 , 4 ,  //  5
       4 , 4 , 3 , 3 , 3 , 6 , 6    //  6
}; */


const uint8_t servo_Fuzzy_Kd_rule[7][7] =      //  Kd  规则表
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
const uint8_t servo_Fuzzy_Kd_rule[7][7] =      //  Kd  规则表
{ //   0--1--2--3--4--5--6
	4 , 4 , 3 , 3 , 3 , 6 , 6 ,  //  0
       2 , 2 , 2 , 2 , 3 , 2 , 5 ,  //  1
       6 , 6 , 1 , 2 , 3 , 4 , 5 ,  //  2
       0 , 1 , 1 , 2 , 3 , 4 , 5 ,  //  3
       0 , 1 , 2 , 2 , 3 , 4 , 4 ,  //  4
       1 , 2 , 2 , 2 , 3 , 4 , 4 ,  //  5
       4 , 3 , 3 , 3 , 3 , 6 , 6    //  6
}; */

uint32_t  Fuzzy_speed_cunchu_zuo[7] ={ 0,0,0,0,0,0,0 } ;  // 存储模糊控制中 Kp 计算过程中的值，并存储上一次的值
uint32_t  Fuzzy_speed_cunchu_you[7] ={ 0,0,0,0,0,0,0 } ;  // 存储模糊控制中 Kd 计算过程中的值，并存储上一次的值
                                                                 //   -3      -2       -1        0         1       2          3   
const uint16_t speed_Fuzzy_zuo_dan[7] = { 2350 , 2250 , 2100 , 2300 , 2100 , 1950 , 1800 } ; // 
const uint16_t speed_Fuzzy_you_dan[7] = { 1800 , 1950 , 2100 , 2300 , 2100 , 2250 , 2350 } ; 

//const uint16_t speed_Fuzzy_zuo_dan[7] = { 2450 , 2350 , 2200 , 2350 , 2150 , 2050 , 1900 } ; // 
//const uint16_t speed_Fuzzy_you_dan[7] = { 1900 , 2050 , 2150 , 2350 , 2200 , 2350 , 2450 } ; 

//const uint16_t speed_Fuzzy_zuo_dan[7] = { 2300 , 2250 , 2100 , 2200 , 2100 , 1950 , 1800 } ; // 稳
//const uint16_t speed_Fuzzy_you_dan[7] = { 1800 , 1950 , 2100 , 2200 , 2100 , 2250 , 2300 } ; 

//const uint16_t speed_Fuzzy_zuo_dan[7] = { 2550 , 2520 , 2500 , 2800 , 2400 , 2250 , 2150 } ; // 37.26m----15.719s----2.370m/s
//const uint16_t speed_Fuzzy_you_dan[7] = { 2150 , 2250 , 2400 , 2800 , 2500 , 2520 , 2550 } ; 

//const uint16_t speed_Fuzzy_zuo_dan[7] =  { 2650 , 2620 , 2600 , 2800 , 2500 , 2350 , 2250 } ; // 37.26m----15.23s----m/s
//const uint16_t speed_Fuzzy_you_dan[7] =  { 2250 , 2350 , 2500 , 2800 , 2600 , 2620 , 2650 } ; 

//const uint16_t speed_Fuzzy_zuo_dan[7] =  { 2750 , 2720 , 2700 , 3200 , 2600 , 2450 , 2350 } ; // 14.78s--15.2m/s
//const uint16_t speed_Fuzzy_you_dan[7] =  { 2350 , 2450 , 2600 , 3200 , 2700 , 2720 , 2750 } ; 

uint16_t  speed_temp_11[4] = { 0,0,0,0 } , speed_temp_12[4] = { 0,0,0,0 } , speed_temp_21[4] = { 0,0,0,0 } , speed_temp_22[4] = { 0,0,0,0 } ;
 
//======================================================================//
//==========================全局变量======================================//
int32_t  i_quan , j_quan , k_quan ;
uint8_t xianshi_flag = 0 ;     //选择显示的数据标志位
uint32_t motor_time = 0 ;     //PIT计数器
 
//====================================================================//
//======================== 算法 ——标志位  =================================== //
uint8_t po_dao_flag = 0 , daoda_podao_flag = 0 ;  //坡道
uint8_t zhi_jiao_flag =0;  // 判断是否到达  直角处
uint8_t zhi_jiao_charge =0;
uint8_t zhi_jiao_check=0;
int8_t  zhijiao_zhuanxiang = 0 ; //   直角转向
uint8_t ru_shi_zi_flag = 0 ;  //   进入十字
uint8_t chu_shi_zi_flag = 0 ; // 离开十字
uint8_t zhi_dao_flag = 0 ;  // 进入直道   
uint8_t little_S_flag = 0 ;  // 进入小  S
uint8_t bigbig_S_flag = 0 ; // 进入大  S 

int8_t K_one[50] ;  // 1  斜率
int8_t K_two[50] ;  // 2 
int8_t K_three[50] ;// 3
int8_t K_four[50] ; // 4
int8_t K_five[50] ; //  5
//wandao_come_in =1 左进弯 wandao_come_in =2  右进弯
uint8_t  wandao_come_in =0;

//wandao_go_out =1 左出弯 wandao_go_out =2  右出弯 
uint8_t  wandao_go_out = 0 ;  

uint8_t pian_yi_zuo_flag = 0 ; // 
uint8_t pian_yi_zhong_flag = 0 ; // 
uint8_t pian_yi_you_flag = 0 ; //
uint8_t shangpo_flag = 0 ; // 上坡
uint32_t shangpo_flag_distance;
uint32_t lost_time = 0 ; //丢信号次数
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
#define   motor_pwmR    EMIOS_0.CH[13].CBDR.R //右边  Maximal value is 8000
#define   motor_pwmL    EMIOS_0.CH[14].CBDR.R //左边  Maximal value is 8000
//#define   servo_pwm     EMIOS_0.CH[21].CBDR.R     //Maximal value is 53333

#define   servo_pwm     EMIOS_0.CH[22].CBDR.R     //Maximal value is 53333


//============================测速变量=================================//

uint32_t smartcar_speed_L ; // 左电机速度
uint32_t smartcar_speed_R ; //  右电机速度
uint8_t fashu;
uint32_t per_pulse_R=0;//距离
uint32_t per_pulse_L=0;
uint32_t average_distance;
//右边
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
//左边
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
//============================转向变量=================================//
uint32_t per_distance_R=0;//距离
uint32_t per_distance_L=0;

uint32_t sum_distance_R=0;
uint32_t sum_distance_L=0;

uint32_t average_distance_L=0;
uint8_t zhuanxaing;
uint8_t protect_flag;

int16_t  speed_difference;
uint16_t  speed_difference_flag;

 int32_t servo_output;
 int32_t servo_output_last[20]={0} ;  //  存储上几次  servo_output  输出量
 int32_t cha_su_value;
int16_t Duoji_value;

int16_t start_pid=0;
int16_t pid_times[5]={0} ;
int16_t pid_flag[5]={0} ;
int16_t start_pid_time;
int16_t stop_flag=0;
int16_t smart_speed;
//**************************   电机  PID  结构体  定义  ********************************//
struct motor_PID 
{
          int32_t  Proportion ;   // 比例常数  
          int32_t  Integral ;   // 积分常数 
          int32_t  Derivative ;   // 微分常数
           
          int32_t  SumError;         //误差累计 
                   // 设定目标 Desired Value 
          int32_t  iError ;   // 偏差
          int32_t  iIncpid ;  // 增量             
          int32_t  output ; // 电机输出值
          int32_t  last_output ; // 电机上次输出值
          int32_t  LastError;           //Error[-1] 
          int32_t  PrevError;           //Error[-2]

          int32_t  Max_iError ; //限制iError，防超调
          int32_t  Min_iError ;
          int32_t  Max_iIncpid ;  //限制增量
          int32_t  Min_iIncpid ;
          int32_t  Max_otput ;  //限制输出量
          int32_t  Min_otput ;
             
//=============  右轮  ============//        
          int32_t  SumError_other;         //误差累计 
          int32_t  iError_other; 
          int32_t  iIncpid_other;             
          int32_t  output_other;
          int32_t  last_output_other;
          int32_t  LastError_other;           //Error[-1] 
          int32_t  PrevError_other;           //Error[-2]

} PID;



//=====================================================================//
////////////////////////////数码管初始化///////////////////////////////
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
#define                             DIG_1                0x01        // 定义数码管1 register 
#define                             DIG_2                0x02        // 定义数码管2 register 
#define                             DIG_3                0x03        // 定义数码管3 register 
#define                             DIG_4                0x04        // 定义数码管4 register 
#define                             DIG_5                0x05        // 定义数码管5 register 
#define                             DIG_6                0x06        // 定义数码管6 register 
#define                             DIG_7                0x07        // 定义数码管7 register 
#define                             DIG_8                0x08        // 定义数码管8 register 
/*---------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------*/
/*------------------------------------------------------------------------*/
                         /*数码管*/
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
  
  Write7219(Shutdown_addr,0X00);       //停机
  Write7219(Decode_Mode_addr,0XFF);    //译码模式
  Write7219(Intensity_addr,0X00);     //亮度
  Write7219(Scan_Limit_addr,0X07);        //显示位数
  Write7219(Shutdown_addr,0X01);         //正常显示模式
  Write7219(Display_Test_addr,0X00);     //测试
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
      n = k/1000 - k /10000 *10  ;   //获得千位
      m = k /100 - k /1000 *10    ;   //获得百位
      x = k /10 - k /100 *10      ;   //获得个位
      y = k % 10                     ;   //获得个位
           Write7219(DIG_4,y);
            Write7219(DIG_3,x);
            Write7219(DIG_2,m);
            Write7219(DIG_1,n);   
     }
void Show_Data(uint16_t i)
{
	uint16_t  k,y ,x ,m , n;
	k=i;
    n = k/1000 - k /10000 *10  ;   //获得千位
    m = k /100 - k /1000 *10    ;   //获得百位
    x = k /10 - k /100 *10      ;   //获得个位
    y = k % 10                   ;   //获得个位
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
	   
       if (decrease_port_in == low)  //减少
           number_test-=change;
       
       if (increase_port_in == low)  // 增加
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
            case   13://    是否有障碍 
   	                  CCD_xuanze_flag = number_test;    
   	        break;   	        
//===============================================// 
            case   14://    是否有障碍 
   	                  duoji_zhongzhi = number_test;    
   	        break;   	      
//===============================================// 
             case   15://    是否有障碍 
   	                  CCD_juli_dm_she_ding_juli[1] = number_test;    
   	        break;   	 
//===============================================// 
            case   16://    距离测速
   	                  CCD_juli_dm_she_ding_juli[0] = number_test;    
   	        break;   	   	              
//===============================================// 
             case   17://    到达距离后开启  CCD
   	                  CCD_juli_dm_she_ding_juli[2] = number_test;    
   	        break;   	 
//===============================================// 
            case   18://    到达距离后关闭  CCD
   	                  CCD_juli_dm_she_ding_juli[3] = number_test;    
   	        break;   	   	              
//===============================================// 
            case   19://    距离测速
   	                  CCD_baoguang_shijian = number_test;    
   	        break;
//===============================================// 
            case   20://    到达距离后关闭  CCD
   	                  piancha_E_jisuan = number_test;    
   	        break;   	   	              
//===============================================// 
 
   	         case   22://    距离测速
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
//================ 最小二乘法 拟合 ================= //
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

//==============================  求两个数据之差的绝对值 ============================//
uint16_t ABS(int16_t x,int16_t y)
{
	if(x>y)         return ( x-y ) ;
	else   	    return ( y-x ) ;
}
//=================  判断两个数之差的  符号  ================//
int16_t fu_hao(int16_t xx)
{
      if(xx>0)	    return(1) ;
      if(xx<0)      return(-1) ;
}
//================  将数据取正  ====================//
int16_t qu_zheng(int16_t xxx)
{
	if(  xxx >= 0  )    return(xxx) ;
	else                   return(-xxx) ;
}
 
//**********************************  STM —— 延时  *************************//
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




//==================================== 中断——延时 ===================================//
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



 

//==========================周期中断===================================//

void PIT_CH1_isr(void)
{

    motor_time++;
    PIT.CH[1].TFLG.B.TIF = 1;
 	
	if(motor_time==CCD_baoguang_shijian+CCD_stoptimes)
	          CCD_times=1 ;//CCD曝光时间         
 
    
    if(motor_time==start_pid_time) 
      start_pid=1;        //满P后开始PID调速
    
    
    
   // if(motor_time>1500) stop_smartcar_flag=1;  
    

}
//=============================记脉冲中断=============================//
//================================左边================================//
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
    
    L_period_sum = L_period_sum+L_count;//脉冲累加

    if(L_period_sum_times==L_smartcar_speed_time)
    {
        L_period = L_period_sum/L_period_sum_times;
        //编码器26T,电机9T,车轮24T,同轴齿轮大的28T小的10T
    	smartcar_speed_L =43*157*200000/L_period/84;
    	// 10*25*3142*100/L_period/7; 
       ////43*157*100000/L_period/84 大编码器 
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
//===============================右边=================================//
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
    
    R_period_sum = R_period_sum+R_count;//脉冲累加
    
    if(R_period_sum_times==R_smartcar_speed_time)
    {
        R_period = R_period_sum/R_period_sum_times;
        //编码器26T,电机9T,车轮24T,同轴齿轮大的28T小的10T
    	smartcar_speed_R = 43*157*200000/R_period/84 ; 
         //32M=>100，8M=>25
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
//=============================电机初始化==================================//
void Dianji_Init( void )
{
    SIU.PCR[PCR40_PC8].R = 0x0200;  //GPIO[40] is output
    SIU.PCR[PCR44_PC12].R = 0x0200; //GPIO[44] is output 
    SIU.PCR[PCR41_PC9].R = 0x0200;  //GPIO[41] is output
    SIU.PCR[PCR47_PC15].R = 0x0200; //GPIO[47] is output  
        
    SIU.GPDO[PCR40_PC8].R = 0 ;       //0-1正转
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
			          SIU.GPDO[PCR40_PC8].R = 1 ;       //1-0反转
                      SIU.GPDO[PCR44_PC12].R = 0 ;
                      motor_pwmR=7600;
                      SIU.GPDO[PCR41_PC9].R = 1;
                      SIU.GPDO[PCR47_PC15].R = 0;
	                  motor_pwmL=7600;
 			      }
 			      
 			      else if(smart_speed>600)
 			      {
			          SIU.GPDO[PCR40_PC8].R = 1 ;       //1-0反转
                      SIU.GPDO[PCR44_PC12].R = 0 ;
                      motor_pwmR= 3500;
                      
                      SIU.GPDO[PCR41_PC9].R = 1;
                      SIU.GPDO[PCR47_PC15].R = 0;
                      motor_pwmL= 3500;
 			      }
 			       
 			      else 
 			      {
			          SIU.GPDO[PCR40_PC8].R = 1 ;       //1-0反转
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
     }//检测是否应该停车
    if(motor_time<5000&&stop_smartcar_flag==1)
          stop_smartcar_flag=0;
   
}
//=========================PID初始化=============================//
void PID_Init() 
{
            PID.SumError=0;          //误差累计
           
            PID.Proportion=5600;        
            PID.Integral= 600;         
            PID.Derivative=50 ;       
            
            PID.iError=0;  
            PID.iIncpid=0;           
            
            PID.output=0;            
            PID.last_output=0;            

            PID.LastError=0;         //Error[-1]
            PID.PrevError=0;  

            PID.Max_iError=3500;      //限制iError，防超调
            PID.Min_iError=-3500;
            
            PID.Max_iIncpid=8000;     //限制增量
            PID.Min_iIncpid=-8000;
            
            PID.Max_otput=7600;         //限制输出量
            PID.Min_otput=-7600;

         //=========================//
             PID.SumError_other=0;         //误差累计 
            
             PID.iError_other=0; 
             PID.iIncpid_other=0;             
             PID.output_other=0;
             PID.last_output_other=0;

             PID.LastError_other=0;           //Error[-1] 
             PID.PrevError_other=0;           //Error[-2]
}
 
//======================================================================//
//============================PID调速====================================//
int PID_change_L(int Current_Speed ,int SetPoint )
{ 
       //当前误差 
       PID.iError =  SetPoint - Current_Speed ;  //增量计算 
        
       //************限制条件**************//     
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
       
        //存储误差，用于下次计算 
        PID.PrevError =  PID.LastError ; 
        PID.LastError =  PID.iError ; 
       
        //****** ********限制判断*** ************************//            
      if( PID.iIncpid>= PID.Max_iIncpid)
      {     
         PID.iIncpid = PID.Max_iIncpid ; 
      }    //限制增量  
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
       }  //限制输出
       zuo_shu_chu=PID.output ;
       PID.output=motor_judge_left( PID.output ) ; //是否反拖判断
        
	   PID.last_output = PID.output ;     
       return PID.output ;
       
           
		          
  }    
      
//=======================  右轮  ==========================//          
int PID_change_R(int Current_Speed ,int SetPoint )          
 {
         //当前误差 
         PID.iError_other = SetPoint - Current_Speed;  //增量计算 
        
           //*********限制条件**************/     
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
       
        //存储误差，用于下次计算 
        PID.PrevError_other =  PID.LastError_other ; 
        PID.LastError_other =  PID.iError_other ; 
       
          //*************限制判断************//           
      if( PID.iIncpid_other >= PID.Max_iIncpid )
      {  
          PID.iIncpid_other = PID.Max_iIncpid ;
       }      //限制增量
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
       }    //限制输出
          you_shu_chu=PID.output_other;
	    PID.output_other = motor_judge_right( PID.output_other ) ;
	    PID.last_output_other = PID.output_other ;   
        return PID.output_other ;
}
//================  调速判断，是否反拖？=====================//
int motor_judge_right(int right)
{
   	if(right>0)
   	{  	    
              SIU.PCR[PCR40_PC8].R = 0x0200 ;  //GPIO[40] is output
              SIU.PCR[PCR44_PC12].R = 0x0200 ; //GPIO[44] is output 

              SIU.GPDO[PCR40_PC8].R = 0 ;       //0-1正转
              SIU.GPDO[PCR44_PC12].R =1 ;
              return right;
   	}	
 //========================== //  	
   else	if(right<0)
   	{	 
              SIU.PCR[PCR40_PC8].R = 0x0200 ;  //GPIO[40] is output
              SIU.PCR[PCR44_PC12].R = 0x0200 ; //GPIO[44] is output 
        
              SIU.GPDO[PCR40_PC8].R = 1 ;       //1-0反转
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
         
              SIU.GPDO[PCR40_PC8].R = 0 ;       //0-1正转
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
/********************************************曝光**************************************************/
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
  //********************************CCD_IO口初始化*****************************************//
void CCD_IO_Init(void) 
{
  	  TSL_CLK_DDR1 =0x0203;
	  TSL_SI_DDR1  =0x0203;
  	  TSL_CLK1 = 0;
  	  TSL_SI1  = 0;
  
}
//************************************上位机发数*******************************************//
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
	//===================   找到最大值     ==================================	
	CCD_max[0]=( CCD_sample[23]+CCD_sample[24]+CCD_sample[101]+CCD_sample[102] ) / 4 ;
	CCD_max[1]=( CCD_sample[57]+CCD_sample[58]+CCD_sample[59]+CCD_sample[60] ) / 4 ;
	CCD_max[2]=( CCD_sample[64]+CCD_sample[65]+CCD_sample[66]+CCD_sample[67] ) / 4 ;
	if( CCD_max[1]<CCD_max[2] )  CCD_max[1]=CCD_max[2] ;
	// ==============      min   =========================//
	CCD_min=( CCD_sample[30]+CCD_sample[31]+CCD_sample[32]+CCD_sample[33]+
	                    CCD_sample[92]+CCD_sample[93]+CCD_sample[94]+CCD_sample[95] ) / 8 ;
	 CCD_max[3]=CCD_max[0] ;
	 CCD_max[4]=( CCD_max[0]-CCD_min )/4+CCD_min ;  // 阈值 
	 
	 if( CCD_max[1]<CCD_max[0] )   CCD_max[0]=CCD_max[1] ; 
	 if( CCD_max[0]<CCD_min )    CCD_max[0]=CCD_min ;

	 CCD_yuzhi=( CCD_max[0]-CCD_min )/3 ;  
	 CCD_value= CCD_max[0] - CCD_yuzhi ;
	//=====================  跳变的差值  ===========================
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
	
	//=====================  跳变的值  ===========================
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
             
         //=============================避障处理==========================//
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
		      		if( CCD_lefttimes[1]>4 && CCD_lefttimes[2]>4 ) // 确定点是突变的
		      			CCD_bizhang_flag=1 ;//左边障碍
		      }
		      else if( (CCD_righttimes[0]-CCD_lefttimes[0])>4 && CCD_righttimes[0]>5 && CCD_lefttimes[0]<3 )
		      {
		      		if( CCD_righttimes[1]>4 && CCD_righttimes[2]>4 ) // 确定点是突变的
		      			CCD_bizhang_flag=2 ;//右边障碍
		      }  
            }
}



//===========================避障CCD程序==================================//
void CCD_bizhang( void )
{
                 if(CCD_times==1)
		  {
				    CCD_times=0 ;
				    CCD_stoptimes=motor_time ;
				    ImageCapture1(CCD_sample) ;//采样 
				    StartInte1( ) ;//曝光
				    CCD_tiaobian_erzhihua( ) ; //二值化 、判断是否有障碍
				   
				    if( CCD_bizhang_flag==1 || CCD_bizhang_flag==2 )
				    {
						SIU.GPDO[PCR38_PC6].R =0 ;//给0才亮
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
				                    
				             //      SIU.GPDO[PCR38_PC6].R =!SIU.GPDO[PCR38_PC6].R ;//给0才亮
				           }
	 */	/*	*/
		  }
}




//13===3====2===1
//         11===10
//          8====9
//========================================================================//
uint16_t  diangan_ADC(uint8_t tongdao) // 采集电感的值
{
    uint16_t i=0 , j=0 , temp=0 ;
    uint16_t a[DIAN]={0 };
    uint16_t ADC_tongdao ;
    uint32_t average=0 ;
    
    if(tongdao == 13)     ///一排  1////
        ADC_tongdao = 0x00002000;
    else if(tongdao == 3)       ///一排  2////
        ADC_tongdao = 0x00000008;
    
    else if(tongdao == 2)       ///一排  3////
        ADC_tongdao = 0x00000004;
    else if(tongdao == 1)       ///一排  4////
        ADC_tongdao = 0x00000002; 
    
    else if(tongdao == 11)     ///二排  1////
        ADC_tongdao = 0x00000800;
    else if(tongdao == 10)       ///二排  2////
        ADC_tongdao = 0x00000400;
    
    else if(tongdao == 8)       ///三排  1////
        ADC_tongdao = 0x00000100;
    else if(tongdao == 9)       ///三排  2////
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
//****************************  归一化  处理  ************************//
uint16_t diangan_guiyihua( uint8_t guiyi_diangan_hao)
{
        uint16_t  guiyi[guiyi_DIAN] ;
        
        SIU.GPDO[PCR38_PC6].R = 1 ;    // 灭   
        delay500ms() ; delay500ms() ;
        SIU.GPDO[PCR38_PC6].R = 0 ;   // 亮     
       //=======================================//
        if( guiyi_diangan_hao == 1)    // 水平一号电感=======13口
        {
        	for( i_quan=0 ; i_quan <guiyi_DIAN ; i_quan++ )
              { 
                   guiyi[i_quan] = diangan_ADC(13) ;
       	            delay5ms() ;   	    
              } 
       }
       
       //==========================================//
       else if( guiyi_diangan_hao == 2)  // 水平二号电感=====3口
        {
      	      for( i_quan=0 ; i_quan <guiyi_DIAN ; i_quan++ )
             {
      	            guiyi[i_quan] = diangan_ADC(3) ;
      	      //      LCD5110_number_dis( guiyi[i_quan] );
      	            delay5ms() ; 	    
             }
      }
      
      //==========================================//
      else if( guiyi_diangan_hao == 3)  // 水平三号电感======2口
      {
             for( i_quan=0 ; i_quan <guiyi_DIAN ; i_quan++ )
             {
    	           guiyi[i_quan] = diangan_ADC(2) ;
    	       //   LCD5110_number_dis( guiyi[i_quan] );
      	           delay5ms() ; 	    
             }
      }
        else if( guiyi_diangan_hao == 4)  // 水平三号电感======2口
      {
             for( i_quan=0 ; i_quan <guiyi_DIAN ; i_quan++ )
             {
    	           guiyi[i_quan] = diangan_ADC(1) ;
    	       //   LCD5110_number_dis( guiyi[i_quan] );
      	           delay5ms() ; 	    
             }
      }
      //==========================================//
     else if( guiyi_diangan_hao == 5) //  =======11口
      {
      	      for( i_quan=0 ; i_quan <guiyi_DIAN ; i_quan++ )
             {
      	            guiyi[i_quan] = diangan_ADC(11) ;
       	            delay1ms() ; 	    
             }
      }
      
       //==========================================//
     else if( guiyi_diangan_hao == 6) //    =========1口
      {
      	      for( i_quan=0 ; i_quan <guiyi_DIAN ; i_quan++ )
             {
      	            guiyi[i_quan] = diangan_ADC(10) ;
       	            delay1ms() ;	    
             }
      }
      
    
  
      
      SIU.GPDO[PCR38_PC6].R = 1 ;    // 灭   
      
      for( i_quan=1 ; i_quan <guiyi_DIAN ; i_quan++ )
      {       if( guiyi[0] < guiyi[i_quan])     guiyi[0] = guiyi[i_quan] ;  }
    
      return  guiyi[0] ;
}
//13===3====2===1
//         11===10
//          8====9
//=========================  采集电感信号  ====================//
void Cai_yang(void)  // 采集五个电感的最大值，水平电感之和，垂直电感值和
{ 
       int i;
      uint16_t  array_temp[11] = { 0 } ;
      array_one[0]=200*diangan_ADC(13)/guiyi_MAX[0];//斜置电感1号
	  array_two[0]=200*diangan_ADC(3)/guiyi_MAX[1]; //水平电感1号
	  array_three[0]=200*diangan_ADC(2)/guiyi_MAX[2];//水平电感2号
	  array_four[0]=200*diangan_ADC( 1)/guiyi_MAX[3];//斜置电感2号
	  array_five[0]=200*diangan_ADC(11)/guiyi_MAX[4];//垂直电感1号  
      array_six[0]=200*diangan_ADC(10)/guiyi_MAX[5];//垂直电感1号 
	                        //SCI0_SendChar_16(array_one[0]);
 		                    //SCI0_SendCh[]ar_16(array_two[0]);
 		                    //SCI0_SendChar_16(array_three[0]);
 		                    //SCI0_SendChar_16(array_four[0]);
		                    //SCI0_SendChar_16(array_five[0]); 
        
	  //  限幅 —— 滤波
	  //================== 1 号 电感 ============== //
	  if( (array_one[0]-array_one[1]) > 5 )    //======= 波动限幅大小——5，11 —— +3 ，+7=====//
	  {
	        array_lvbo_flag[0][2] = 0 ;
	        array_lvbo_flag[0][3] = 0 ;
	        if( array_lvbo_flag[0][0] < 2 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 5 以内
	        {
	        	array_one[0]=array_one[1]+3 ; // 1
	        	array_lvbo_flag[0][0]++;
	        }
	  	else  if( array_lvbo_flag[0][1] < 3 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		array_one[0]=array_one[1]+8 ; // 1
	  		array_lvbo_flag[0][1]++;
	  	}
	  	else if( (array_one[0]-array_one[5]) < 60 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 6 次以内还未回归
	  	       array_one[0]=array_one[5] + 40 ;
	  	else  
	  	       array_one[0]=array_one[1]+15 ; // array_lvbo_flag[0][0]  一号电感的偏差在 7 次以内还未回归
	  } 
 	  else if( (array_one[0]-array_one[1]) < -5 )   //======= 波动限幅大小—— -5，-11 —— -3 ，-7=====//
 	  {
	        array_lvbo_flag[0][0] = 0 ;
	        array_lvbo_flag[0][1] = 0 ;
	        if( array_lvbo_flag[0][2] < 2 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 5 以内
	        {
	        	array_one[0]=array_one[1]-3 ; // 1
	        	if( array_one[0]<0 )   array_one[0]=0;
	        	array_lvbo_flag[0][2]++;
	        }
	  	else  if( array_lvbo_flag[0][3] < 3 ) //// array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		array_one[0]=array_one[1]-8 ; // 1
	  		if( array_one[0]<0 )   array_one[0]=0;
	  		array_lvbo_flag[0][3]++;
	  	}
	  	else if( (array_one[0]-array_one[5]) > -60 ) // array_lvbo_flag[0][0]  一号电感的偏差是否在 6 次以内还未回归
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
 	  // ========    向下限幅    =========//
	   if( (array_one[0]-array_one[1]) > -3 && (array_one[0]-array_one[1]) < 3  )    //======= 波动限幅大小——5，11 —— +3 ，+7=====//
	  {
	        array_lvbo_flag[0][5] ++ ;
	        if( array_lvbo_flag[0][5] == 10 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 5 以内
	    	 {
	    	  	array_temp[0]=( array_one[0]+array_one[1]+array_one[2]+array_one[3]+array_one[4]+array_one[5]+array_one[6]+array_one[7]+array_one[8]+array_one[9]+5 ) /10 ; // 1
	   		if( (array_temp[0]-array_one[0]) > -3 && (array_temp[0]-array_one[0]) < 3  )    
	   		  	array_one[0] = array_temp[0] ;
	   		else 
	   		        array_lvbo_flag[0][5] = 0 ;
	  	}
	  	else if( array_lvbo_flag[0][5] > 10 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		array_one[0] = array_one[1] ;
	  		array_lvbo_flag[0][5] -- ;
	  	}	
	  }
	  else
	  	array_lvbo_flag[0][5] = 0 ;
   
   
   
   
   

 	    
 	   //================== 2 号 电感 ============== //   
 	  if( (array_two[0]-array_two[1]) > 5 )    //======= 波动限幅大小——5，11 —— +3 ，+7=====//
	  {
	        array_lvbo_flag[1][2] = 0 ;
	        array_lvbo_flag[1][3] = 0 ;
	        if( array_lvbo_flag[1][0] < 2 )  // array_lvbo_flag[1][0]  一号电感的偏差是否在 5 以内
	        {
	        	array_two[0]=array_two[1]+3 ; // 1
	        	array_lvbo_flag[1][0]++;
	        }
	  	else  if( array_lvbo_flag[1][1] < 3 )  // array_lvbo_flag[1][0]  一号电感的偏差是否在 11 以内
	  	{
	  		array_two[0]=array_two[1]+8 ; // 1
	  		array_lvbo_flag[1][1]++;
	  	}
	  	else if( (array_two[0]-array_two[5]) < 60 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 6 次以内还未回归
	  	       array_two[0]=array_two[5] + 40 ;
	  	else  
	  	       array_two[0]=array_two[1]+15 ; // array_lvbo_flag[0][0]  一号电感的偏差在 7 次以内还未回归
	  } 
 	  else if( (array_two[0]-array_two[1]) < -5 )   //======= 波动限幅大小—— -5，-11 —— -3 ，-7=====//
 	  {
	        array_lvbo_flag[1][0] = 0 ;
	        array_lvbo_flag[1][1] = 0 ;
	        if( array_lvbo_flag[1][2] < 2 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 5 以内
	        {
	        	array_two[0]=array_two[1]-3 ; // 1
	        		if( array_two[0]<0 )   array_two[0]=0;
	        	array_lvbo_flag[1][2]++;
	        }
	  	else  if( array_lvbo_flag[1][3] < 3 ) //// array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		array_two[0]=array_two[1]-8 ; // 1
	  			if( array_two[0]<0 )   array_two[0]=0;
	  		array_lvbo_flag[1][3]++;
	  	}
	  	else if( (array_two[0]-array_two[5]) > -60 ) // array_lvbo_flag[0][0]  一号电感的偏差是否在 6 次以内还未回归
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
 	   	   // ========    向下限幅    =========//
	   if( (array_two[0]-array_two[1]) > -3 && (array_two[0]-array_two[1]) < 3  )    //======= 波动限幅大小——5，11 —— +3 ，+7=====//
	  {
	        array_lvbo_flag[1][5] ++ ;
	        if( array_lvbo_flag[1][5] == 10 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 5 以内
	    	 {
	    	  	array_temp[1]=( array_two[0]+array_two[1]+array_two[2]+array_two[3]+array_two[4]+array_two[5]+array_two[6]+array_two[7]+array_two[8]+array_two[9]+5 ) /10 ; // 1
                	if( (array_temp[1]-array_two[0]) > -3 && (array_temp[1]-array_two[0]) < 3  )    
	   		  	array_two[0] = array_temp[1] ;
	   		else 
	   		        array_lvbo_flag[1][5] = 0 ;
		 }
	  	else if( array_lvbo_flag[1][5] > 10 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		array_two[0] = array_two[1] ;
	  		array_lvbo_flag[1][5] -- ;
	  	}	
	  }
	  else
	  	array_lvbo_flag[1][5] = 0 ;
	  
	  
	  
	  

   
 	  //================== 3 号 电感 ============== //   
 	  if( (array_three[0]-array_three[1]) > 5 )    //======= 波动限幅大小——5，11 —— +3 ，+7=====//
	  {
	        array_lvbo_flag[2][2] = 0 ;
	        array_lvbo_flag[2][3] = 0 ;
	        if( array_lvbo_flag[2][0] < 2 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 5 以内
	        {
	        	array_three[0]=array_three[1]+3 ; // 1
	        	array_lvbo_flag[2][0]++;
	        }
	  	else  if( array_lvbo_flag[2][1] < 3 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		array_three[0]=array_three[1]+8 ; // 1
	  		array_lvbo_flag[2][1]++;
	  	}
	  	else if( (array_three[0]-array_three[5]) < 60 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 6 次以内还未回归
	  	       array_three[0]=array_three[5] + 40 ;
	  	else  
	  	     {
	  	     	array_three[0]=array_three[1]+15 ; // array_lvbo_flag[0][0]  一号电感的偏差在 7 次以内还未回归
	  	     }  
	  } 
 	  else if( (array_three[0]-array_three[1]) < -5 )   //======= 波动限幅大小—— -5，-11 —— -3 ，-7=====//
 	  {
	        array_lvbo_flag[2][0] = 0 ;
	        array_lvbo_flag[2][1] = 0 ;
	        if( array_lvbo_flag[2][2] < 2 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 5 以内
	        {
	        	array_three[0]=array_three[1]-3 ; // 1
	        		if( array_three[0]<0 )   array_three[0]=0;
	        	array_lvbo_flag[2][2]++;
	        }
	  	else  if( array_lvbo_flag[2][3] < 3 ) //// array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		array_three[0]=array_three[1]-8 ; // 1
	  			if( array_three[0]<0 )   array_three[0]=0;
	  		array_lvbo_flag[2][3]++;
	  	}
	  	else if( (array_three[0]-array_three[5]) > -60 ) // array_lvbo_flag[0][0]  一号电感的偏差是否在 6 次以内还未回归
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
 	  	  // ========    向下限幅    =========//
	   if( (array_three[0]-array_three[1]) > -3 && (array_three[0]-array_three[1]) < 3  )    //======= 波动限幅大小——5，11 —— +3 ，+7=====//
	  {
	        array_lvbo_flag[2][5] ++ ;
	        if( array_lvbo_flag[2][5] == 10 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 5 以内
	    	 {
	    	  	array_temp[2]=( array_three[0]+array_three[1]+array_three[2]+array_three[3]+array_three[4]+array_three[5]+array_three[6]+array_three[7]+array_three[8]+array_three[9]+5 ) /10 ; // 1
                	if( (array_temp[2]-array_three[0]) > -3 && (array_temp[2]-array_three[0]) < 3  )    
	   		  	array_three[0] = array_temp[2] ;
	   		else 
	   		        array_lvbo_flag[2][5] = 0 ;
		 }
	  	else if( array_lvbo_flag[2][5] > 10 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		array_three[0] = array_three[1] ;
	  		array_lvbo_flag[2][5] -- ;
	  	}	
	  }
	  else
	  	array_lvbo_flag[2][5] = 0 ;
	  
	  
	  

 	  //================== 4 号 电感 ============== //   
 	  if( (array_four[0]-array_four[1]) > 5 )    //======= 波动限幅大小——5，11 —— +3 ，+7=====//
	  {
	        array_lvbo_flag[3][2] = 0 ;
	        array_lvbo_flag[3][3] = 0 ;
	        if( array_lvbo_flag[3][0] < 2 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 5 以内
	        {
	        	array_four[0]=array_four[1]+3 ; // 1
	        	array_lvbo_flag[3][0]++;
	        }
	  	else  if( array_lvbo_flag[3][1] < 3 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		array_four[0]=array_four[1]+8 ; // 1
	  		array_lvbo_flag[3][1]++;
	  	}
	  	else if( (array_four[0]-array_four[5]) < 60 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 6 次以内还未回归
	  	       array_four[0]=array_four[5] + 40 ;
	  	else  
	  	    {
	  	    	array_four[0]=array_four[1]+15 ; // array_lvbo_flag[0][0]  一号电感的偏差在 7 次以内还未回归
	  	    }   	 
	  }
 	  else if( (array_four[0]-array_four[1]) < -5 )   //======= 波动限幅大小—— -5，-11 —— -3 ，-7=====//
 	  {
	        array_lvbo_flag[3][0] = 0 ;
	        array_lvbo_flag[3][1] = 0 ;
	        if( array_lvbo_flag[3][2] < 2 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 5 以内
	        {
	        	array_four[0]=array_four[1]-3 ; // 1
	        		if( array_four[0]<0 )   array_four[0]=0;
	        	array_lvbo_flag[3][2]++;
	        }
	  	else  if( array_lvbo_flag[3][3] < 3 ) //// array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		array_four[0]=array_four[1]-8 ; // 1
	  			if( array_four[0]<0 )   array_four[0]=0;
	  		array_lvbo_flag[3][3]++;
	  	}
	  	else if( (array_four[0]-array_four[5]) > -60 ) // array_lvbo_flag[0][0]  一号电感的偏差是否在 6 次以内还未回归
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
 	  	  // ========    向下限幅    =========//
	   if( (array_four[0]-array_four[1]) > -3 && (array_four[0]-array_four[1]) < 3  )    //======= 波动限幅大小——5，11 —— +3 ，+7=====//
	  {
	        array_lvbo_flag[3][5] ++ ;
	        if( array_lvbo_flag[3][5] == 10 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 5 以内
	    	 {
	    	  	array_temp[3]=( array_four[0]+array_four[1]+array_four[2]+array_four[3]+array_four[4]+array_four[5]+array_four[6]+array_four[7]+array_four[8]+array_four[9]+5 ) /10 ; // 1
                	if( (array_temp[3]-array_four[0]) > -3 && (array_temp[3]-array_four[0]) < 3  )    
	   		  	array_four[0] = array_temp[3] ;
	   		else 
	   		        array_lvbo_flag[3][5] = 0 ;
		 }
	  	else if( array_lvbo_flag[3][5] > 10 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		array_four[0] = array_four[1] ;
	  		array_lvbo_flag[3][5] -- ;
	  	}	
	  }
	  else
	  	array_lvbo_flag[3][5] = 0 ;
	  
	  
	  
	  
	  

 	  //================== 5 号 电感 ============== //   
 	  if( (array_five[0]-array_five[1]) > 3 )    //======= 波动限幅大小——5，11 —— +3 ，+7=====//
	  {
	        array_lvbo_flag[4][3] = 0 ;
	        array_lvbo_flag[4][4] = 0 ;
	        array_lvbo_flag[4][5] = 0 ;
	        if( array_lvbo_flag[4][0] < 1 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 5 以内
	        {
	        	array_five[0]=array_five[1]+2 ; // 1
	        	array_lvbo_flag[4][0]++;
	        }
	  	else  if( array_lvbo_flag[4][1] < 2 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		array_five[0]=array_five[1]+4 ; // 1
	  		array_lvbo_flag[4][1]++;
	  	}
	       else  if( array_lvbo_flag[4][2] < 3 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		array_five[0]=array_five[1]+8 ; // 1
	  		array_lvbo_flag[4][2]++;
	  	}
	  	else if( (array_five[0]-array_five[6]) < 60 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 6 次以内还未回归
	  	       array_five[0]=array_five[6] + 40 ;
	  	else  
	  	   {
	  	   	 array_five[0]=array_five[1]+15 ; // array_lvbo_flag[0][0]  一号电感的偏差在 7 次以内还未回归
	  	   }   
	  } 
 	  else if( (array_five[0]-array_five[1]) < -3 )   //======= 波动限幅大小—— -5，-11 —— -3 ，-7=====//
 	  {
	        array_lvbo_flag[4][0] = 0 ;
	        array_lvbo_flag[4][1] = 0 ;
	        array_lvbo_flag[4][2] = 0 ;
	        if( array_lvbo_flag[4][3] < 1 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 5 以内
	        {
	        	array_five[0]=array_five[1]-2 ; // 1
	        		if( array_five[0]<0 )   array_five[0]=0;
	        	array_lvbo_flag[4][3]++;
	        }
	  	else  if( array_lvbo_flag[4][4] < 2 ) //// array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		array_five[0]=array_five[1]-4 ; // 1
	  			if( array_five[0]<0 )   array_five[0]=0;
	  		array_lvbo_flag[4][4]++;
	  	}
	  	else  if( array_lvbo_flag[4][5] < 3 ) //// array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		array_five[0]=array_five[1]-8 ; // 1
	  			if( array_five[0]<0 )   array_five[0]=0;
	  		array_lvbo_flag[4][5]++;
	  	}
	  	else if( (array_five[0]-array_five[6]) > -60 ) // array_lvbo_flag[0][0]  一号电感的偏差是否在 6 次以内还未回归
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
     	  // ========    向下限幅    =========//
	   if( (array_five[0]-array_five[1]) > -3 && (array_five[0]-array_five[1]) < 3  )    //======= 波动限幅大小——5，11 —— +3 ，+7=====//
	  {
	        array_lvbo_flag[4][6] ++ ;
	        if( array_lvbo_flag[4][6] == 10 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 5 以内
	    	 {
	    	  	array_temp[4]=( array_five[0]+array_five[1]+array_five[2]+array_five[3]+array_five[4]+array_five[5]+array_five[6]+array_five[7]+array_five[8]+array_five[9]+5 ) /10 ; // 1
                	if( (array_temp[4]-array_five[0]) > -3 && (array_temp[4]-array_five[0]) < 3  )    
	   		  	array_five[0] = array_temp[4] ;
	   		else 
	   		        array_lvbo_flag[4][6] = 0 ;
		 }
	  	else if( array_lvbo_flag[4][6] > 10 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		array_five[0] = array_five[1] ;
	  		array_lvbo_flag[4][6] -- ;
	  	}	
	  }
	  else
	  	array_lvbo_flag[4][6] = 0 ;
	  
	  
	   //========================== 偏差处理 ===========================//
        if( array_one[0] < 2 )    // 防止   分母 电感值   为零
                array_one[0] = 1 ;
       if( array_two[0] < 2 )    // 防止   分母 电感值   为零
                array_two[0] = 1 ;
        if( array_three[0] < 2 )    // 防止   分母 电感值   为零
                array_three[0] = 1 ;
       if( array_four[0] < 2 )    // 防止   分母 电感值   为零
                array_four[0] = 1 ;
        if( array_five[0] < 2 )    // 防止   分母 电感值   为零
                array_five[0] = 1 ;

 	  	if( array_six[0] < 2 )    // 防止   分母 电感值   为零
                array_six[0] = 1 ;    
 	  
 //=================================================//
 //由于采样是已经多次采集了，并且用了平均滤波，所以不再滤波，后续再看
 //实时将同一时刻的电感值放在同一个数组里
	  	array_current[0] = array_one[0] ;//     对应电感1
	  	array_current[1] = array_two[0] ;//	     对应电感2
	  	array_current[2] = array_three[0] ;//     对应电感3
	  	array_current[3] = array_four[0];//       对应电感4
        array_current[4] = array_five[0] ;//       对应电感5 		
        array_current[5]=array_six[0];
  
    // noise_protect_first();
    
    //电感排布
    ///4========5
    // 0==1==2==3 
    
     
       chui_zhi_daingan[0]=array_current[4]-array_current[5]; //垂直电感之差               
       Verticaldiangan_sum[0]=array_current[4]+array_current[5]; //垂直电感之和
       Leveltaldiangan_sum[0]=array_current[1]+array_current[2];//水平电感之和
       xiezhi_diangan_sum[0]= array_current[0]+array_current[3];//斜置电感之后
       total_diangan_sum[0]=xiezhi_diangan_sum[0]+Leveltaldiangan_sum[0];//二排电感总和 
                             //SCI0_SendChar_16(three_diangan_sum[0]);
 		                     //SCI0_SendChar_16(two_diangan_sum[0]);
 		                    //SCI0_SendChar_16(array_three[0]);
 		                    //SCI0_SendChar_16(array_four[0]);
		                    //SCI0_SendChar_16(array_five[0]); 
          
}
//存下偏差在各个范围时的打角
//建立模糊map表格，用于查询转向？？？
//===================================偏移量函数==========================//
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


  
//参考
//if( chuizhi_piancha[0]>=0)   chuizhi_piancha_xishu=chuizhi_piancha[0]*100/6+400 ;
//else chuizhi_piancha_xishu=-chuizhi_piancha[0]*100/6+400 ;
//if(chuizhi_piancha_xishu<500)    chuizhi_piancha_xishu=500 ;
//if(chuizhi_piancha_xishu>1300)    chuizhi_piancha_xishu=1300 ;
//xiezhi_piancha_xishu=1300-chuizhi_piancha_xishu ;
//if(xiezhi_piancha_xishu<500)    xiezhi_piancha_xishu=500 ;
//servo_Error[0]= xiezhi_piancha_xishu*xiezhi_piancha[0]/1000+chuizhi_piancha_xishu*chuizhi_piancha[0]/1000;

 
//==========================测试程序==========================//
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
		            		         //求取各个偏移量的论域，在区间进行
		         KK_chuizhi[0]=qu_zheng(chuizhi_piancha[0])/6; 
		         KK_shuiping[0]=qu_zheng(shuiping_piancha[0])/6;  
		         KK_xiezhi[0]= qu_zheng(xiezhi_piancha[0])/6;
		  
		          if(KK_shuiping[0]>16)  KK_shuiping[0]=16;
		          if(KK_chuizhi[0]>16)  KK_chuizhi[0]=16;
		          if(KK_xiezhi[0]>16)  KK_xiezhi[0]=16;
		 
		      
		      //=======修正斜置电感偏移量=======================//
		      if( Road_Type[0]==3)
		      {
		      	  if(KK_xiezhi[0]<2&&KK_chuizhi[0]>12)     
		           xiezhi_piancha[0]= xiezhi_piancha[0]/10;
		      }
		     //=================================================//
		      if(Leveltaldiangan_sum[0]>320)
		      {
		         //检测到十字，不用斜置电感的偏移量
		         if( Road_Type[0]==3)
		         {
		             if(shuiping_piancha[0]*chuizhi_piancha[0]>0)
		       	            	 servo_Error[0]= 110*shuiping_piancha[0]/100
		                                          +chuizhi_piancha[0]/7 ;	
		       	     else
		       	  	       	     servo_Error[0]= 120*shuiping_piancha[0]/100
		                                          -chuizhi_piancha[0]/7 ;	
		         	
		         }
		        //最大电感很大时，很可能斜着入十字，用水平偏移量
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
		    
		         //检测到十字，不用斜置电感的偏移量
		         if( Road_Type[0]==3)
		         {
		             if(shuiping_piancha[0]*chuizhi_piancha[0]>0)
		       	            	 servo_Error[0]= 110*shuiping_piancha[0]/100
		                                          +chuizhi_piancha[0]/7 ;	
		       	     else
		       	  	       	     servo_Error[0]= 120*shuiping_piancha[0]/100
		                                          -chuizhi_piancha[0]/7 ;	
		         	
		         }
		        //最大电感很大时，很可能斜着入十字，用水平偏移量
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
       diangan_max[0]=0;//清零处理
 diangan_maxfour[0]=0;
       for(i=0;i<4;i++)
       {
        
          if( diangan_max [0] < array_current[i] )//找出四个电感的最大值，
          {
         	 diangan_max [0]=array_current[i]  ; //以及找出是第几个电感是最大值 
       	     cixu[0][0]=i ;
          }
      
       }
       
       
 
       //=================垂直电感最大值
       if(array_current[5]>array_current[6])
       chuizhidiangan_max[0]=array_current[5];
       else chuizhidiangan_max[0]=array_current[6];
      /*
       if(cixu[0][0]==0)     //左边最大:0
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
       else if(cixu[0][0]==1)//中间最大:1
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
          
      else                 //右边最大:2
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
     diangan_min[0]=array_current[cixu[0][2]];//储存最小值
     */
  //斜置电感大小   
     /*if(array_current[3]>array_current[4])
         xiediangan_max[0]=array_current[3];
     else 
         xiediangan_max[0]=array_current[4];
     
     */
     //中间电感与两边电感之差
     
    /* 
     if(cixu[0][0]==1)//中间最大
         middle_piancha[0]==0;
     else if(cixu[0][0]==0)//左边最大
     {
        middle_piancha[0]=array_current[0]-array_current[1];
        	
     }
     else if(cixu[0][0]==2)
     {
     	 middle_piancha[0]=array_current[1]-array_current[2];
     }
     */
      
     
     
     
      //========================== 偏差处理 ===========================//
        if( array_current[0] < 2 )    // 防止   分母 电感值   为零
                array_current[0] = 1 ;
       if( array_current[1] < 2 )    // 防止   分母 电感值   为零
                array_current[1] = 1 ;
        if( array_current[2] < 2 )    // 防止   分母 电感值   为零
                array_current[2] = 1 ;
       if( array_current[3] < 2 )    // 防止   分母 电感值   为零
                array_current[3] = 1 ;
       if( array_current[4] < 2 )    // 防止   分母 电感值   为零
                array_current[4] = 1 ;
        
      if( array_current[5] < 2 )    // 防止   分母 电感值   为零
                array_current[5] = 1 ;
      if( array_current[6] < 2 )    // 防止   分母 电感值   为零
                array_current[6] = 1 ;
        
        
      //水平电感偏差
      shuiping_piancha[0] = 250* ( array_current[1] - array_current[2] ) /( array_current[1] + array_current[2] )  ;
      //shuiping_piancha[0] = 40 * ( array_current[0] - array_current[2] ) /( array_current[0]+array_current[2] )*3 ;
        //垂直电感偏差  
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
         
   //================== 水平电感限幅处理  ============== //   
 	  if( (shuiping_piancha[0]-shuiping_piancha[1]) > 3 )    //======= 波动限幅大小——5，11 —— +3 ，+7=====//
	  {
	        shuiping_piancha_flag[3] = 0 ;
	        shuiping_piancha_flag[4] = 0 ;
	        shuiping_piancha_flag[5] = 0 ;
	        if( shuiping_piancha_flag[0] < 1 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 5 以内
	        {
	        	shuiping_piancha[0]=shuiping_piancha[1]+2 ; // 1
	        	shuiping_piancha_flag[0]++;
	        }
	  	else  if( shuiping_piancha_flag[1] < 1 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		shuiping_piancha[0]=shuiping_piancha[1]+4 ; // 1
	  		shuiping_piancha_flag[1]++;
	  	}
	       else  if( shuiping_piancha_flag[2] < 2 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		shuiping_piancha[0]=shuiping_piancha[1]+7 ; // 1
	  		shuiping_piancha_flag[2]++;
	  	}
	  	else if( (shuiping_piancha[0]-shuiping_piancha[5]) < 50 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 6 次以内还未回归
	  	       shuiping_piancha[0]=shuiping_piancha[5] + 32 ;
	  	else  
	  	       shuiping_piancha[0]=shuiping_piancha[1]+12 ; // array_lvbo_flag[0][0]  一号电感的偏差在 7 次以内还未回归
	  } 
 	  else if( (shuiping_piancha[0]-shuiping_piancha[1]) < -3 )   //======= 波动限幅大小—— -5，-11 —— -3 ，-7=====//
 	  {
	        shuiping_piancha_flag[0] = 0 ;
	        shuiping_piancha_flag[1] = 0 ;
	        shuiping_piancha_flag[2] = 0 ;
	        if( shuiping_piancha_flag[3] < 1 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 5 以内
	        {
	        	shuiping_piancha[0]=shuiping_piancha[1]-2 ; // 1
	        	shuiping_piancha_flag[3]++;
	        }
	  	else  if( shuiping_piancha_flag[4] < 1 ) //// array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		shuiping_piancha[0]=shuiping_piancha[1]-4 ; // 1
	  		shuiping_piancha_flag[4]++;
	  	}
	  	else  if( shuiping_piancha_flag[5] < 2 ) //// array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		shuiping_piancha[0]=shuiping_piancha[1]-7 ; // 1
	  		shuiping_piancha_flag[5]++;
	  	}
	  	else if( (shuiping_piancha[0]-shuiping_piancha[7]) > -50 ) // array_lvbo_flag[0][0]  一号电感的偏差是否在 6 次以内还未回归
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

   
   //================== 垂直电感限幅处理  ============== //   
 	  if( (chuizhi_piancha[0]-chuizhi_piancha[1]) > 3 )    //======= 波动限幅大小——5，11 —— +3 ，+7=====//
	  {
	        chuizhi_piancha_flag[3] = 0 ;
	        chuizhi_piancha_flag[4] = 0 ;
	        chuizhi_piancha_flag[5] = 0 ;
	        if( chuizhi_piancha_flag[0] < 1 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 5 以内
	        {
	        	chuizhi_piancha[0]=chuizhi_piancha[1]+2 ; // 1
	        	chuizhi_piancha_flag[0]++;
	        }
	  	else  if( chuizhi_piancha_flag[1] < 1 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		chuizhi_piancha[0]=chuizhi_piancha[1]+4 ; // 1
	  		chuizhi_piancha_flag[1]++;
	  	}
	       else  if( chuizhi_piancha_flag[2] < 2 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		chuizhi_piancha[0]=chuizhi_piancha[1]+8 ; // 1
	  		chuizhi_piancha_flag[2]++;
	  	}
	  	else if( (chuizhi_piancha[0]-chuizhi_piancha[5]) < 50 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 6 次以内还未回归
	  	       chuizhi_piancha[0]=chuizhi_piancha[5] + 35 ;
	  	else  
	  	       chuizhi_piancha[0]=chuizhi_piancha[1]+15 ; // array_lvbo_flag[0][0]  一号电感的偏差在 7 次以内还未回归
	  } 
 	  else if( (chuizhi_piancha[0]-chuizhi_piancha[1]) < -3 )   //======= 波动限幅大小—— -5，-11 —— -3 ，-7=====//
 	  {
	        chuizhi_piancha_flag[0] = 0 ;
	        chuizhi_piancha_flag[1] = 0 ;
	        chuizhi_piancha_flag[2] = 0 ;
	        if( chuizhi_piancha_flag[3] < 1 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 5 以内
	        {
	        	chuizhi_piancha[0]=chuizhi_piancha[1]-2 ; // 1
	        	chuizhi_piancha_flag[3]++;
	        }
	  	else  if( chuizhi_piancha_flag[4] < 1 ) //// array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		chuizhi_piancha[0]=chuizhi_piancha[1]-4 ; // 1
	  		chuizhi_piancha_flag[4]++;
	  	}
	  	else  if( chuizhi_piancha_flag[5] < 2 ) //// array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		chuizhi_piancha[0]=chuizhi_piancha[1]-8 ; // 1
	  		chuizhi_piancha_flag[5]++;
	  	}
	  	else if( (chuizhi_piancha[0]-chuizhi_piancha[5]) > -50 ) // array_lvbo_flag[0][0]  一号电感的偏差是否在 6 次以内还未回归
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
         
         
   
    
//=================================偏移量的处理===========================//
  // smartcar_position();
  // servo_Error[0]= 0*shuiping_piancha[0]/20+20*chuizhi_piancha[0]/20;
     
    smartcar_position_NEW();   
                           
//========================================================================//   
    //把偏差标在正负100以内
     if(servo_Error[0]>100)    servo_Error[0]=100;
     if(servo_Error[0]<-100)   servo_Error[0]=-100;
    
        //SCI0_SendChar_16(servo_Error[0]);
      //==================   偏差  限幅处理  ============== //   
   if( (servo_Error[0]-servo_Error[1]) > 6 )    //======= 波动限幅大小——5，11 —— +3 ，+7=====//
	  {
	        servo_Error_flag[3] = 0 ;
	        servo_Error_flag[4] = 0 ;
	        servo_Error_flag[5] = 0 ;
	        if( servo_Error_flag[0] < 1 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 5 以内
	        {
	        	servo_Error[0]=servo_Error[1]+4 ; // 1
	        	servo_Error_flag[0]++;
	        }
	  	else  if( servo_Error_flag[1] < 1 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		servo_Error[0]=servo_Error[1]+8 ; // 1
	  		servo_Error_flag[1]++;
	  	}
	       else  if( servo_Error_flag[2] < 2 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		servo_Error[0]=servo_Error[1]+12 ; // 1
	  		servo_Error_flag[2]++;
	  	}
	  	else if( (servo_Error[0]-servo_Error[5]) < 60 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 6 次以内还未回归
	  	       servo_Error[0]=servo_Error[5] + 45 ;
	  	else  
	  	       servo_Error[0]=servo_Error[1]+15 ; // array_lvbo_flag[0][0]  一号电感的偏差在 7 次以内还未回归
	  } 
 	  else if( (servo_Error[0]-servo_Error[1]) < -6 )   //======= 波动限幅大小—— -5，-11 —— -3 ，-7=====//
 	  {
	        servo_Error_flag[0] = 0 ;
	        servo_Error_flag[1] = 0 ;
	        servo_Error_flag[2] = 0 ;
	        if( servo_Error_flag[3] < 1 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 5 以内
	        {
	        	servo_Error[0]=servo_Error[1]-4 ; // 1
	        	servo_Error_flag[3]++;
	        }
	  	else  if( servo_Error_flag[4] < 1 ) //// array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		servo_Error[0]=servo_Error[1]-8 ; // 1
	  		servo_Error_flag[4]++;
	  	}
	  	else  if( servo_Error_flag[5] < 2 ) //// array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		servo_Error[0]=servo_Error[1]-12 ; // 1
	  		servo_Error_flag[5]++;
	  	}
	  	else if( (servo_Error[0]-servo_Error[5]) > -60 ) // array_lvbo_flag[0][0]  一号电感的偏差是否在 6 次以内还未回归
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


//三条数据保护措施，以识别并处理错误数据。
//(1)在侧向电感L1、L2和L3的感应电动势均很小的情况下，
//即可确定此时肯定处于弯道，此时不能让最大感应电动势的电感序号发生变化，
//即进入左转弯，应当保持电感L1的感应电动势最大，
//进入右转弯，应当持电感L3的感应电动势最大
//若此时最大电感发生变化，可认为数据发生错误,当进行强制调整�
//(2)从理论上分析，L2的感应电动势始终为最大值或中间值，
//若为最小值，可认为数据错误，同样进行强制调整。
//(3)当L2的感应电动势非常小的情况下，
//采取防止区间标定结果发生大幅度跳变措施。
 void noise_protect_first()
 {
 
    if(protect_Road_Type[0]==1&&total_diangan_sum[0]<240)//左弯道
    {  //如果不是最左边的最大 则将强制调整为最大值
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
    
    if(protect_Road_Type[0]==2&&total_diangan_sum[0]<240)//右弯道
	{//如果不是最右边的最大 则将强制调整为最大值
	    if(cixu[0][0]!= 2)
	    {
	       if(array_current[0]>array_current[2]&&array_current[3]>array_current[4])
	    	array_current[2]=array_current[cixu[1][0]];
	    	array_current[1]=array_current[cixu[1][1]];
	    	array_current[0]=array_current[cixu[1][2]];
	    	
	    }
    }	
 	//如果最小的电感次序为1，即中间电感为最小，则认为发生错误，强制调整
 	if(cixu[0][2]==1) 
 	{
 	    array_current[1]=array_current[cixu[1][1]];
   		
 	}
 	    
 	
 }
//===================================================// 
 void noise_protect_second()
 {
 
      if(protect_Road_Type[0]==1)//左弯道
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


//根据实际情况，将赛道类型分为:
//直道、左弯道、右弯道、十字交叉点以及坡道五类
//对赛道进行分类的主要作用是，在不同类型的赛道设定不同的期望速度。
//不同类型赛达识别方法如下：
//(1)直道：侧向电感L1、L2和L3的感应电动势和值大于一定值，
//纵向电感V1和 V2的感应电动势差值的绝对值小于一定值
//=================================================================//
//(2)弯道：侧向电感L1、L2和L3的感应电动势和值小于一定值。
//对于左弯道，纵向电感V1和 V2的感应电动势之差大于一定值；
//对于右弯道，纵向电感V2和 V1的感应电动势之差大于一定值�
//=================================================================//
//(3)十字交叉点：纵向电感V1和 V2的感应电动势均分别大于一定值
//=================================================================//
//(4)坡道可划分为上坡、坡顶、下坡、坡后四个区域，
//各区域都有很明显的特征，设电感L1、L2和L3的感应电动势和值为Esum
//上坡：Esum很大；
//坡顶：Esum很小；
//下坡：Esum很大；
//坡后：Esum减小，恢复正常值�
////0:直道  4:大弧弯 5:  1:左转弯  2:右转弯 3:十字交叉点 
//=================================================================//
void sai_dao_position()
{ 
    
     //uint16_t two_cut_one;
     
//======== 直道检测 ===========//
   if(Road_Type[0] !=0)
   {

	    if(Leveltaldiangan_sum[0]>320 && qu_zheng(chui_zhi_daingan[0] )<30)
	    {
	         
	         if(++Road_Type_Times[0]>20)
	         {     
	              
 	             Road_Type[0] = 0;//直道
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
   
    
//====================十字交叉点检测=============================//
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
//=======================左转弯检测=============================//
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
//======================右转弯检测==============================//
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
 
//============================赛道检测滤波==========================// 
 //protect_
 
 if(protect_Road_Type[0] !=0)
   {

	    if(Leveltaldiangan_sum[0]>320 && qu_zheng(chui_zhi_daingan[0] )<30)
	    {
	         
	         if(++protect_Road_Type_Times[0]>50)
	         {     
	              
 	             protect_Road_Type[0] = 0;//直道
 	                           
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
   
    
//====================十字交叉点检测=============================//
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
//=======================左转弯检测=============================//
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
//======================右转弯检测==============================//
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
///////////////////////////////弯道检测函数///////////////////////////
//功能描述;寻找偏移量的变化趋势，确定出弯还是进弯
void wandao_check()
{
     
    uint8_t i;
    uint8_t static ruwan_times;
    uint8_t static chuwan_times;

   if(Road_Type[0]==1)//左弯道位置判断
   {
	    
	   chuwan_times=0;
	   ruwan_times=0;
	   for(i=0;i<50;i++)
	   {
	   	     if((servo_Error[i]-servo_Error[i+15])>0) 
	           ruwan_times++ ;//入弯次数判断
	           
	    	 if((servo_Error[i]-servo_Error[i+15])<0) 
	    	   chuwan_times++ ;//出弯次数判断

	   }
	    
	    
	   if(ruwan_times>15) wandao_come_in=1;//wandao_come_in置1是左入弯
	   else wandao_come_in=0;
	   
	   if(chuwan_times>15) wandao_go_out=1;//wandao_go_out置1是左出弯
	   else wandao_go_out=0;
    }
    
   else if(Road_Type[0]==2)//右弯道位置判断
   {
	    
	   chuwan_times=0;
	   ruwan_times=0;
	   for(i=0;i<50;i++)
	   {
	   	     if((servo_Error[i]-servo_Error[i+15])<0) 
	           ruwan_times++ ;//入弯次数判断
	           
	    	 if((servo_Error[i]-servo_Error[i+15])>0) 
	    	   chuwan_times++ ;//出弯次数判断

	   }
	    
	    
	   if(ruwan_times> 15) wandao_come_in=2;//wandao_come_in置2是右入弯
	   else wandao_come_in=0;
	   
	   if(chuwan_times> 15) wandao_go_out=2;//wandao_go_out置2是右出弯
	   else wandao_go_out=0;
    }
    
    else
    {
       	wandao_come_in=0;//如果条件都不满足，标志位清零
       	wandao_go_out=0;//
    }
}
 
 
 
 
 
//======================void servo_Fuzzy_Kp_chu_shi_hua( void ) ======================================//
//                                  kp_e (kp_e_lishudu )      ( kp_e+1) ( Error_lishudu_Max - kp_e_lishudu )              
//                                --------------------------------------------------------------------------------
//                                  kp_ec (kp_ec_lishudu ) 
//                               (kp_ec+1) (Error_lishudu_Max - kp_ec_lishudu ) 
//                                   输入偏差  偏差变化率 ，得出 Kp 的值
//=================================================================================//
uint16_t servo_Fuzzy_Kp_chu_shi_hua( void )
{
        uint8_t    kp_e = 0 , kp_ec = 0 ;  //  Kp 规则表中第几区域
                    // 取出小区域的号 E，及其隶属度 L；则大区域的号 E+1 ，Error_lishudu_Max - L
        uint16_t  kp_e_lishudu , kp_ec_lishudu  ; //  隶属度
        uint8_t  temp_11[4] = { 0,0,0,0 } , temp_12[4] = { 0,0,0,0 } , temp_21[4] = { 0,0,0,0 } , temp_22[4] = { 0,0,0,0 } ;
       
        Fuzzy_Kp_cunchu[5] = Fuzzy_Kp_cunchu[0] ;  // 存储上一次 Kp用过的值 ， 用于与本次比较 ， 目的是限幅滤波
  
               ////**-----Kp  ----E----- 分配区间----- ****/////
              ///***    求出      kp_e      kp_e_lishudu   ***///
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

                       ////***----Kp  -----EC----- 分配区间---****/////
                      ///****    求出      kp_ec      kp_ec_lishudu    ****///
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
       /////      找到规则表中第几区域       ////
       temp_11[0] = servo_Fuzzy_Kp_rule[ kp_ec ][ kp_e ] ;
       temp_12[0] = servo_Fuzzy_Kp_rule[ kp_ec][ kp_e +1 ] ;
       temp_21[0] = servo_Fuzzy_Kp_rule[ kp_ec+1 ][ kp_e ] ;
       temp_22[0] = servo_Fuzzy_Kp_rule[ kp_ec+1 ][ kp_e+1 ] ;
       
       ///          找到按 servo_Error 找到的隶属度   /////
       temp_11[1] = kp_e_lishudu ;
       temp_12[1] = Error_lishudu_Max - kp_e_lishudu ;
       temp_21[1] = kp_e_lishudu ;
       temp_22[1] = Error_lishudu_Max - kp_e_lishudu ;
       
        ///          找到按 servo_Error_c 找到的隶属度   /////
       temp_11[2] = kp_ec_lishudu ;
       temp_21[2] = Error_c_lishudu_Max - kp_ec_lishudu ;
       temp_12[2] = kp_ec_lishudu ;
       temp_22[2] = Error_c_lishudu_Max - kp_ec_lishudu ;
       
       ///                  确定四个点的隶属度            /////
       if( temp_11[1] > temp_11[2] )  temp_11[1] = temp_11[2] ;
       if( temp_12[1] > temp_12[2] )  temp_12[1] = temp_12[2] ;
       if( temp_21[1] > temp_21[2] )  temp_21[1] = temp_21[2] ;
       if( temp_22[1] > temp_22[2] )  temp_22[1] = temp_22[2] ;
       
       //===================================//
       // 小中取大  规则表中同等区域——隶属度取大 //
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
      Fuzzy_Kp_cunchu[1] =  servo_Fuzzy_Kp_dan[ temp_11[0] ] * temp_11[1]   // 分子
                                       + servo_Fuzzy_Kp_dan[ temp_12[0] ] * temp_12[1]
                                       + servo_Fuzzy_Kp_dan[ temp_21[0] ] * temp_21[1]
                                       + servo_Fuzzy_Kp_dan[ temp_22[0] ] * temp_22[1] ;
       Fuzzy_Kp_cunchu[2] = temp_11[1] + temp_12[1] + temp_21[1] + temp_22[1] ;  // 分母             
       if( Fuzzy_Kp_cunchu[2] < 2 )  Fuzzy_Kp_cunchu[2]=1 ;
       
       Fuzzy_Kp_cunchu[0] = Fuzzy_Kp_cunchu[1] / Fuzzy_Kp_cunchu[2] ;
       
       
       
      

       //========================= 上限滤波   ===================================//
       if( ( Fuzzy_Kp_cunchu[0] - Fuzzy_Kp_cunchu[5] ) > 27  )      // 对 Kp 进行 —— 限幅 —— 滤波
             Fuzzy_Kp_cunchu[0] = Fuzzy_Kp_cunchu[5] + 19 ;         // [0] 本次的 Kp  ,
       else if( (Fuzzy_Kp_cunchu[0] - Fuzzy_Kp_cunchu[5])<-27 )  // [5] 上次的Kp
             Fuzzy_Kp_cunchu[0] = Fuzzy_Kp_cunchu[5] - 19 ;
     //========================  下限滤波    ==================================//
     if( ( Fuzzy_Kp_cunchu[0] - Fuzzy_Kp_cunchu[5] ) < 3 )
     {
     	   if( ( Fuzzy_Kp_cunchu[0] - Fuzzy_Kp_cunchu[5] ) > -3 )
     	        Fuzzy_Kp_cunchu[0] = Fuzzy_Kp_cunchu[5] ;
     }

       Fuzzy_Kp_cunchu[0] = ( 87*Fuzzy_Kp_cunchu[0] + 13*Fuzzy_Kp_cunchu[5]) / 100 ; // 求平均值—— 滤波
        
         
                return Fuzzy_Kp_cunchu[0]  ;    
}

//=========================void servo_Fuzzy_Kd_chu_shi_hua( void ) =================================//
//                                  kp_e (kp_e_lishudu )      ( kp_e+1) ( Error_lishudu_Max - kp_e_lishudu )              
//                                --------------------------------------------------------------------------------
//                                  kp_ec (kp_ec_lishudu ) 
//                               (kp_ec+1) (Error_lishudu_Max - kp_ec_lishudu ) 
//                               输入偏差  偏差变化率 ，得出 Kd 的值
//=======================================================================================//
uint16_t servo_Fuzzy_Kd_chu_shi_hua( void )
{
        uint8_t    kp_e = 0 , kp_ec = 0 ;  //  Kp 规则表中第几区域
                    // 取出小区域的号 E，及其隶属度 L；则大区域的号 E+1 ，Error_lishudu_Max - L
        uint16_t  kp_e_lishudu=0 , kp_ec_lishudu =0  ; //  隶属度
        uint8_t  temp_11[4] = { 0,0,0,0 } , temp_12[4] = { 0,0,0,0 } , temp_21[4] = { 0,0,0,0 } , temp_22[4] = { 0,0,0,0 } ;
       
        Fuzzy_Kd_cunchu[5] = Fuzzy_Kd_cunchu[0] ;  // 存储上一次 Kd用过的值 ， 用于与本次比较 ， 目的是限幅滤波
       
        ////***********    -----------Kp  ------E------- 分配区间---------   **************/////
        ///******************         求出      kp_e      kp_e_lishudu               **************///
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

         ////***********    -----------Kp  ------EC------- 分配区间---------   **************/////
        ///******************         求出      kp_ec      kp_ec_lishudu               **************///
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
       /////      找到规则表中第几区域       ////
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
       
       ///                  确定四个点的隶属度            /////
       if( temp_11[1] > temp_11[2] )  temp_11[1] = temp_11[2] ;
       if( temp_12[1] > temp_12[2] )  temp_12[1] = temp_12[2] ;
       if( temp_21[1] > temp_21[2] )  temp_21[1] = temp_21[2] ;
       if( temp_22[1] > temp_22[2] )  temp_22[1] = temp_22[2] ;
       
       //===================================//
       // 小中取大  规则表中同等区域——隶属度取大 //
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
   
       Fuzzy_Kd_cunchu[1]   =  servo_Fuzzy_Kd_dan[ temp_11[0] ] * temp_11[1]  //分子
                                          + servo_Fuzzy_Kd_dan[ temp_12[0] ] * temp_12[1]
                                          + servo_Fuzzy_Kd_dan[ temp_21[0] ] * temp_21[1]
                                          + servo_Fuzzy_Kd_dan[ temp_22[0] ] * temp_22[1] ;
       Fuzzy_Kd_cunchu[2] = temp_11[1] + temp_12[1] + temp_21[1] + temp_22[1] ;   // 分母           
       if( Fuzzy_Kd_cunchu[2] <2 )    Fuzzy_Kd_cunchu[2] = 1;
           
       Fuzzy_Kd_cunchu[0] = Fuzzy_Kd_cunchu[1] / Fuzzy_Kd_cunchu[2] ;  //  本次计算出的 Kd 值
    
       //==================  上限滤波 ============================//
         if( ( Fuzzy_Kd_cunchu[0] - Fuzzy_Kd_cunchu[5] ) > 27 )  //  对计算出的 Kd 值进行 —— 限幅——滤波
              Fuzzy_Kd_cunchu[0] = Fuzzy_Kd_cunchu[5] + 19 ;  
        else if( ( Fuzzy_Kd_cunchu[0] - Fuzzy_Kd_cunchu[5] ) < -27 )
        {
       	        Fuzzy_Kd_cunchu[0] = Fuzzy_Kd_cunchu[5] - 19 ;
        	if( Fuzzy_Kd_cunchu[0] < 0)
        	    Fuzzy_Kd_cunchu[0] = 0;
        }
       //==================  下限滤波  ======================//
      if( ( Fuzzy_Kd_cunchu[0] - Fuzzy_Kd_cunchu[5] ) < 2 )  //  
       {
       	      if( ( Fuzzy_Kd_cunchu[0] - Fuzzy_Kd_cunchu[5] ) > -2 )
       	          Fuzzy_Kd_cunchu[0] = Fuzzy_Kd_cunchu[5] ;
       }

       Fuzzy_Kd_cunchu[0] = ( 87*Fuzzy_Kd_cunchu[0] + 13*Fuzzy_Kd_cunchu[5] ) / 100 ; // 求平均 —— 滤波
                     
       
        //       SCI0_SendChar_16( Fuzzy_Kd_cunchu[0] ) ;
    
        return Fuzzy_Kd_cunchu[0]  ;
}


void Fuzzy_Speed_chushihua( void )
{
	//=============== Fuzzy_speed_control ===============//
        Fuzzy_speed_cunchu_zuo[1] =  speed_Fuzzy_zuo_dan[ speed_temp_11[0] ] * speed_temp_11[1]   // 分子
                                                     + speed_Fuzzy_zuo_dan[ speed_temp_12[0] ] * speed_temp_12[1]
                                                     + speed_Fuzzy_zuo_dan[ speed_temp_21[0] ] * speed_temp_21[1]
                                                     + speed_Fuzzy_zuo_dan[ speed_temp_22[0] ] * speed_temp_22[1] ;
       Fuzzy_speed_cunchu_zuo[2] = speed_temp_11[1] + speed_temp_12[1] + speed_temp_21[1] + speed_temp_22[1] ;  // 分母             
     
       Fuzzy_speed_cunchu_zuo[0] = Fuzzy_speed_cunchu_zuo[1] / Fuzzy_speed_cunchu_zuo[2] ;
       
        Fuzzy_speed_cunchu_you[1] =  speed_Fuzzy_you_dan[ speed_temp_11[0] ] * speed_temp_11[1]   // 分子
                                                     + speed_Fuzzy_you_dan[ speed_temp_12[0] ] * speed_temp_12[1]
                                                     + speed_Fuzzy_you_dan[ speed_temp_21[0] ] * speed_temp_21[1]
                                                     + speed_Fuzzy_you_dan[ speed_temp_22[0] ] * speed_temp_22[1] ;
       Fuzzy_speed_cunchu_you[2] = speed_temp_11[1] + speed_temp_12[1] + speed_temp_21[1] + speed_temp_22[1] ;  // 分母             
     
       Fuzzy_speed_cunchu_you[0] = Fuzzy_speed_cunchu_you[1] / Fuzzy_speed_cunchu_you[2] ;
       
     //  Fuzzy_speed_cunchu_you[0] = ( Fuzzy_speed_cunchu_you[0] + Fuzzy_speed_cunchu_you[5]) / 2 ; // 求平均值—— 滤波
        
        
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
                
                           
//==================   偏差  限幅处理  ============== //   
/* */
    
          servo_output = ( 80*servo_output + 16*servo_output_last[0] + 3*servo_output_last[1] + 1*servo_output_last[2] ) / 100 ;
 
   //SCI0_SendChar_16( servo_output );
         
}



//========================    zhuanxiang  ——  转向      ======================================//
void zhuanxiang()
{

      uint8_t i;
      int16_t duoji_value;
      
              servo_max=0;
        
      for(i=0;i<12;i++)//寻找上一次的最大输出量
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
         	jiao_biao_min=i;//寻找上一次的最小输出量	
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
		 //参数调试 	
         //=======================================================//
           }
         	
		 
	      lost_signal();
 	     //noise_protect_second(); 
        
 
          zhuan_xiang_control() ;
           
   /*        
           
          if( (servo_output - servo_output_last[0]) > 50 )    //======= 波动限幅大小——5，11 —— +3 ，+7=====//
	  {
	        servo_output_flag[3] = 0 ;
	        servo_output_flag[4] = 0 ;
	        servo_output_flag[5] = 0 ;
	        if( servo_output_flag[0] < 1 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 5 以内
	        {
	        	servo_output=servo_output_last[0]+37 ; // 1
	        	servo_output_flag[0]++;
	        }
	  	else  if( servo_output_flag[1] < 2 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		servo_output=servo_output_last[0]+54 ; // 1
	  		servo_output_flag[1]++;
	  	}
	       else  if( servo_output_flag[2] < 3 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		servo_output=servo_output_last[0]+67 ; // 1
	  		servo_output_flag[2]++;
	  	}
	  	else if( (servo_output-servo_output_last[5]) < 400 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 6 次以内还未回归
	  	       servo_output=servo_output_last[5] + 320 ;
	  	else  
	  	       servo_output=servo_output_last[0]+60 ; // array_lvbo_flag[0][0]  一号电感的偏差在 7 次以内还未回归
	  } 
 	  else if( (servo_output - servo_output_last[0]) < -50 )   //======= 波动限幅大小—— -5，-11 —— -3 ，-7=====//
 	  {
	        servo_output_flag[0] = 0 ;
	        servo_output_flag[1] = 0 ;
	        servo_output_flag[2] = 0 ;
	        if( servo_output_flag[3] < 1 )  // array_lvbo_flag[0][0]  一号电感的偏差是否在 5 以内
	        {
	        	servo_output=servo_output_last[0]-37 ; // 1
	        	servo_output_flag[3]++;
	        }
	  	else  if( servo_output_flag[4] < 2 ) //// array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		servo_output=servo_output_last[0]-54 ; // 1
	  		servo_output_flag[4]++;
	  	}
	  	else  if( servo_output_flag[5] < 3 ) //// array_lvbo_flag[0][0]  一号电感的偏差是否在 11 以内
	  	{
	  		servo_output=servo_output_last[0]-67 ; // 1
	  		servo_output_flag[5]++;
	  	}
	  	else if( (servo_output-servo_output_last[5]) > -400 ) // array_lvbo_flag[0][0]  一号电感的偏差是否在 6 次以内还未回归
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
          
          if(servo_output>700)  //限幅处理
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
//============================= 丢信号处理函数==========================//
//细化处理!!!!!
//步骤：
//1、采样分析
//2、确定车身位置与转向以及电感值的内在联系
//3、编写执行函数，以查询的方式给出经验值
void lost_signal()
{

//diangan_min[0] lost_signal_flag 
 if(   Road_Type[0]==1)//左弯道
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
else if(  Road_Type[0]==2)//右弯道
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
//==========================转向自适应===============================//
//(103+speed_ave/k)*servo_output/100;
//由于车速与转向非单一的一次函数变化，
//所以在转向的补偿上面，不能够用单纯的一次函数计算
//分段调试，将车速细化，在不同的车速上用不同的系数，反调试最终确定系数
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
//=============================  更新    数据  ==========================//
//=======================================================================//
void geng_xin_shuju()
{
      
     // 电感1更新数组
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

      // 电感2更新数组
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

      // 电感3更新数组
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
       
      // 电感4更新数组//垂直1号
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
       
       // 电感5更新数组//垂直2号
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
       
        //更新偏差值
        
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
       
    //=================更新偏差率=================//
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
       
          //储存上一次的偏差，用来滤波
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
       
       //更新输出量
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
   
         // 更新三水平电感值之和
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
         // 更新两  垂直电感值之和
 
         //更新最大电感的次序
         cixu[3][0]=cixu[2][0];
	     cixu[2][0]=cixu[1][0];
	 	 cixu[1][0]=cixu[0][0];
		//更新次大电感的次序
		 cixu[3][1]=cixu[2][1];
		 cixu[2][1]=cixu[1][1];
		 cixu[1][1]=cixu[0][1];
		//更新最小电感的次序
		 cixu[3][2]=cixu[2][2];
		 cixu[2][2]=cixu[1][2];
		 cixu[1][2]=cixu[0][2];
		 
		 //更新最大电感值
		 
		 
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
         
         //更新最小电感值
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
         
		 //更新赛道类型
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
        
        // CCD 左 右 黑点个数 更新
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



 
//函数功能
//小车每走100cm发出1000.18个脉冲，省略为1000个
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
   //函数功能：记录刚进入弯道的位移
   if(Road_Type[1]!=1&&Road_Type[0]==1)
   {
   	 distance[0]=(CCD_juli_zuo[0]+CCD_juli_you[0])/2;//读取当前位移
   	 zhijiao_charge[0]=1;                //进入检测程序标志位
   	 chuizhi_point[0]=qu_zheng(chuizhi_piancha[0]) ;//读取当前垂直位移量，取正值
   	 xiezhi_point[0]=qu_zheng(xiezhi_piancha[0]) ;//读取当前斜置位移量，取正值
  	 
   	
   	
   }
   else if(Road_Type[1]!=2&&Road_Type[0]==2)
   {
   	 distance[0]=(CCD_juli_zuo[0]+CCD_juli_you[0])/2;//读取当前位移
   	 zhijiao_charge[0]=1;                //进入检测程序标志位
   	 chuizhi_point[0]=qu_zheng(chuizhi_piancha[0]) ;//读取当前垂直位移量，取正值
   	 xiezhi_point[0]=qu_zheng(xiezhi_piancha[0]) ;//读取当前斜置位移量，取正值
  	
   }
    
 //=========================直角判断=========================//   
  if((zhijiao_charge[0]==1)&&(Road_Type[0]==1||Road_Type[0]==2))
  {
         //当小车走过15CM时，进入检测程序
  	     if((CCD_juli_zuo[0]+CCD_juli_you[0])/2>distance[0]+150)
  	     { 
   	        chuizhi_point[1]=qu_zheng(chuizhi_piancha[0]) ;//读取当前垂直位移量，取正值
   	        xiezhi_point[1]=qu_zheng(xiezhi_piancha[0]) ;
  	        zhijiao_charge[0]=0;//标志位清零，不再进入此程序，等待下一次。。。 
  	        //计算斜率
  	        distance[1]=(CCD_juli_zuo[0]+CCD_juli_you[0])/2;
  	        zhijaio_point_k[0]=20*qu_zheng(chuizhi_point[1]-chuizhi_point[0])/
  	                               (xiezhi_point[1]+xiezhi_point[0])  ;
  	                               
  	         
  	         if(zhijaio_point_k[0]>10) zhijiao_charge[1]=1;//直角急弯
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
	           CarSpeed_SET_L=0;//紧急刹车，保护小车
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
   
  //========================左边车轮==============================//   
   if(chasu==1)
   {      if(servo_output>0)
		   {          //向左转,左轮转的慢
		      CarSpeed_SET_L=wandao_CarSpeed_SET_L-70*cha_su_value/100 ;              
		   }     
	       else  if(servo_output<0)
		   {       //向右转,左轮转到的快
		       CarSpeed_SET_L=wandao_CarSpeed_SET_L+k1*cha_su_value/100;                                 
		   }
           return CarSpeed_SET_L;  
   }
   
   
   
   
 //=========================右边车轮============================//  
  else if(chasu==2)
  {
  	if(servo_output>0)
    {        //向左转,右轮转到快
		   CarSpeed_SET_R=wandao_CarSpeed_SET_R+k1*cha_su_value/100;
    }     
    else if(servo_output<0)
    {        //向右转,右轮转的慢 
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
     
 //===============================速度控制=========================//    
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
 	
 	 	
	SIU.GPDO[PCR40_PC8].R = 1 ;       //1-0反转
    SIU.GPDO[PCR44_PC12].R = 0 ;
    motor_pwmR= 7000;
                      
    SIU.GPDO[PCR41_PC9].R = 1;
    SIU.GPDO[PCR47_PC15].R = 0;
    motor_pwmL= 7000;
 			      
 	
	
 }
 else if(speed_ave> (CarSpeed_SET_R+ CarSpeed_SET_R)/2+700)
 {	
 	
 	 	
	SIU.GPDO[PCR40_PC8].R = 1 ;       //1-0反转
    SIU.GPDO[PCR44_PC12].R = 0 ;
    motor_pwmR= 5500;
                      
    SIU.GPDO[PCR41_PC9].R = 1;
    SIU.GPDO[PCR47_PC15].R = 0;
    motor_pwmL= 5500;
 			      
 	
	
 }
 
 else if(speed_ave> (CarSpeed_SET_R+ CarSpeed_SET_R)/2+300)
 {	
 	
 	 	
	SIU.GPDO[PCR40_PC8].R = 1 ;       //1-0反转
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
////////////////////////////PID参数选择//////////////////////////////
//===============================左边================================
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
        if( piancha_E_jisuan==8)  // 发车程序
        {
        	  //============== 归一化 ========================
                guiyi_MAX[0] = 200 ;//195  238
		        guiyi_MAX[1] = 240 ;//183  220
			    guiyi_MAX[2] = 240 ;//183   225
		        guiyi_MAX[3] = 235 ;//181   223
		        guiyi_MAX[4] = 245 ;//179   220
		        guiyi_MAX[5] = 245 ;//179   220
		        
		               Show_Me_Data(2500,50,0);  //设定平均速度
                       Show_Me_Data(1,1,1);      //速度档位
                       Show_Me_Data(400,50,2);  //满P加速时间设定 
                       
                       Show_Me_Data(3,1,13) ;       //选择是否障碍     3——避障   CCD_xuanze_flag
                       if( CCD_xuanze_flag==3 )
	                   {
	             		Show_Me_Data(1,1,19) ;    //CCD 曝光时间    /cm   
	             		Show_Me_Data(5,1,16) ;    //避障 —— 距离 —— 脉冲     /cm  
	             		Show_Me_Data(101,20,17) ;    //   到达距离后   开启  CCD    CCD_juli_dm_she_ding_juli[2]>2 开启
	             		if( CCD_juli_dm_she_ding_juli[2]>2 )	//    到达距离后   关闭  CCD
	             			Show_Me_Data(401,20,18) ;   //
	                    }
	               
	               Show_Me_Data(50,10,22) ; 
	               Show_Me_Data(7,1,5) ;       //  计算 E 选择
        
        
        }
        else   // 调试程序
        {
        	  //============== 归一化 ========================
                 guiyi_chushihua() ;  
                 Show_Me_Data(2500,50,0);  //设定平均速度
	             Show_Me_Data(1,1,1);      //速度档位
	             Show_Me_Data(400,50,2);  //满P加速时间设定 
	 	         SIU.GPDO[PCR38_PC6].R =0 ;//给0才亮  
	 	   
	                 //==============================================        		       		
	             Show_Me_Data(3,1,13) ;       //选择是否障碍     3——避障   CCD_xuanze_flag
	             if( CCD_xuanze_flag==3 )
	             {
	             		Show_Me_Data(1,1,19) ;    //CCD 曝光时间    /cm   
	             		Show_Me_Data(5,1,16) ;    //避障 —— 距离 —— 脉冲     /cm  
	             		Show_Me_Data(101,20,17) ;    //   到达距离后   开启  CCD    CCD_juli_dm_she_ding_juli[2]>2 开启
	             		Show_Me_Data(401,20,18) ;   //
	             }
	             
	              Show_Me_Data(50,10,22) ;
	               Show_Me_Data(7,1,5) ;       //计算 E 选择

	               //Show_Me_Data(277,20,15) ;   //调试距离 停车     /cm      <27 不定距离停车
	                                                                                 //    Show_Me_Data(5900,2,14) ;    //检查舵机中值   /cm  

        }

}


void CCD_xuanze( void )
{
	    //=============      判定在直道   ——   进行避障   =======================
          if( CCD_bizhang_flag==0 && podao_flag==0)
           {
	       //  if( Road_Type[0]==0 )
	          if( CCD_xuanze_flag==3 && array_current[4]<20 && array_current[5]<20 && chuizhi_piancha[0]<4 && chuizhi_piancha[0]>-4 )  // 
	  	    {        // 为了确定是否在直道 array_current[1]>array_current[0] && array_current[2]>array_current[3] &&
		           	if( ( array_current[1]>40 &&array_current[1]<300 ) )   // 
		          	{                                                                                                                     //      未丢信号                            未上坡道          
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
	    
	   //================             按距离壁障     ===================	
	      if( CCD_bizhang_flag!=0 )
	      {         //  !=0 说明 看 到障碍了
			//   	CarSpeed_SET_R=0 ;    CarSpeed_SET_L=0 ;
	         	SIU.GPDO[PCR38_PC6].R=0 ;
			
			    CCD_juli[0]=( CCD_juli_zuo[0]+ CCD_juli_you[0]-CCD_juli_zuo[1] -CCD_juli_you[1] )/2 ;
			    if( CCD_juli[0]>CCD_juli_dm_she_ding_juli[0] ) //  避障延时 结束
			    {
				     CCD_bizhang_flag=0 ; //  =0 说明 离开障碍了
				     CCD_juli_flag=0 ; // 保证 CCD_juli_zuo[1]  只进行一次赋值
			    }
	      }
	      else
		        SIU.GPDO[PCR38_PC6].R=1 ;      
			      //=====================================================       
		 if( CCD_bizhang_flag==1 )    //   舵机中值
			    duoji_zhongzhi=duoji_zhongzhi_value-300 ;  //   5443  5843    6243
		 else if( CCD_bizhang_flag==2 )
			    duoji_zhongzhi=duoji_zhongzhi_value+300 ;  // 
		 else duoji_zhongzhi=duoji_zhongzhi_value ;  //6915      
		 
	     //=============      判定是否在坡道   ——   进行避障   =======================	 
	     if( array_current[1]>250 && array_current[2]>250 )  podao_flag=1 ; 
	     if( podao_flag==1&&podao_flag_dingju==0 )    
	     {
	     		podao_flag_dingju=1 ;
	     		CCD_juli_zuo[2]=CCD_juli_zuo[0] ;   CCD_juli_you[2]=CCD_juli_you[0] ; 
	     }
	     if( podao_flag_dingju==1 ) 
	     {  
	     		CCD_juli[2]=( CCD_juli_zuo[0]+ CCD_juli_you[0]-CCD_juli_zuo[2] -CCD_juli_you[2] )/2 ;
	     		if( CCD_juli[2]>2000 ) //  避障延时 结束
			{
			         podao_flag_dingju=0 ; //  =0 说明 离开坡道了
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
				                    
				             //      SIU.GPDO[PCR38_PC6].R =!SIU.GPDO[PCR38_PC6].R ;//给0才亮
				           }   
			           
		      */ 
		      
		      
}

void ding_ju_tingche( void )
{
	 //=================          是否定距离停车      ==============================       
			     
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
//======================= gui_yi—— 归一初始化 =======================//
void guiyi_chushihua( void )
{
	uint8_t  guiyi_xuhao_flage_1 = 0 , guiyi_xuhao_flage_2 = 0 , guiyi_xuhao_flage_3 = 0 ;
	uint8_t  guiyi_xuhao_flage_4 = 0 , guiyi_xuhao_flage_5 = 0 , guiyi_xuhao_flage_6 = 0 ; 
	uint8_t  guiyi_xuhao_flage_7 = 0 , guiyi_xuhao_flage_8 = 0 , guiyi_xuhao_flage_9 = 0 ; 
	uint8_t  guiyi_xuhao_flage_10 = 0 , guiyi_xuhao_flage_11 = 1 , guiyi_xuhao_flage_12 = 0 ; 

	while( guiyi_xuhao_flage_11 )
	{
	       Show_Me_Data(7,1,6) ;//用于准备    
	       // ====================================================  
	       if( diangan_biaoding_xuhao==9 ) //  1
		{
		        while( 1 )
		        {
		        	Show_Me_Data(1,1,6);//用于准备 
		        	guiyi_MAX[ diangan_biaoding_xuhao -1 ] = diangan_guiyihua( diangan_biaoding_xuhao);  // 找出水平1号电感的最大值   
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
		        Show_Me_Data(1,1,6);//用于准备 
			guiyi_MAX[ diangan_biaoding_xuhao -1 ] = diangan_guiyihua( diangan_biaoding_xuhao);  // 找出水平1号电感的最大值   
                       Show_Data(guiyi_MAX[diangan_biaoding_xuhao-1]) ;
                   //   delay400ms();
                      guiyi_xuhao_flage_1 = 1 ;
                
                   	diangan_biaoding_xuhao=2 ;   //	Show_Me_Data(2,1,6);//用于准备 
			guiyi_MAX[ diangan_biaoding_xuhao -1 ] = diangan_guiyihua( diangan_biaoding_xuhao);  // 找出水平1号电感的最大值   
                      Show_Data(guiyi_MAX[diangan_biaoding_xuhao-1]) ;
                  //    delay400ms();
                      guiyi_xuhao_flage_2 = 1 ;
               
                       diangan_biaoding_xuhao=3 ; 	//Show_Me_Data(3,1,6);//用于准备
			guiyi_MAX[ diangan_biaoding_xuhao -1 ] = diangan_guiyihua( diangan_biaoding_xuhao);  // 找出水平1号电感的最大值   
                      Show_Data(guiyi_MAX[diangan_biaoding_xuhao-1]);
               //       delay400ms();
                      guiyi_xuhao_flage_3 = 1 ;
		
		       diangan_biaoding_xuhao=4 ;	       //Show_Me_Data(4,1,6);//用于准备 
			guiyi_MAX[ diangan_biaoding_xuhao -1 ] = diangan_guiyihua( diangan_biaoding_xuhao);  // 找出水平1号电感的最大值   
                      Show_Data(guiyi_MAX[diangan_biaoding_xuhao-1]);
                      delay400ms();
                      guiyi_xuhao_flage_4 = 1 ;
		
		        Show_Me_Data(5,1,6);//用于准备 
			guiyi_MAX[ diangan_biaoding_xuhao -1 ] = diangan_guiyihua( diangan_biaoding_xuhao);  // 找出水平1号电感的最大值   
                      Show_Data(guiyi_MAX[diangan_biaoding_xuhao-1]);
                 //     delay400ms();
                      guiyi_xuhao_flage_5 = 1 ;
                      
                      diangan_biaoding_xuhao=6 ;     //   Show_Me_Data(6,1,6);//用于准备 
			guiyi_MAX[ diangan_biaoding_xuhao -1 ] = diangan_guiyihua( diangan_biaoding_xuhao);  // 找出水平1号电感的最大值   
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
///////////////////////////////////主程序/////////////////////////////////
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
      //while(motor_time<10000);//延时5s
      // motor_time=0;
      
      //==========灯=========//          
      SIU.PCR[PCR38_PC6].R =  0x0200;  //GPIO[38] is output
      SIU.GPDO[PCR38_PC6].R =1;//给0才亮
//==============================标定====================================//       
    

       	//===============  选择速度 —— 定速 、差速 ================= 
        yejingping_xuanze( ) ;   

                                    
//===================================================================//       
        SIU.GPDO[PCR38_PC6].R =1;//给0才亮       
        last_CarSpeed_SET_R=CarSpeed_SET_R; //保存初设速度
        last_CarSpeed_SET_L=CarSpeed_SET_L;
      
        Write7219(Shutdown_addr,0X00);       //关闭数码管显示//停机
      //延时两秒发车
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
      CCD_juli_dm_she_ding_juli[0]=CCD_juli_dm_she_ding_juli[0]*100 ;  //  1dm  看到障碍物时要走的距离
      CCD_juli_dm_she_ding_juli[1]=CCD_juli_dm_she_ding_juli[1]*100 ;  //  1dm  是否定距离停车  走一段距离停车，溜车3m后接着走
      CCD_juli_dm_she_ding_juli[2]=CCD_juli_dm_she_ding_juli[2]*100 ;  //  1dm  17    到达距离后   开启  CCD
      CCD_juli_dm_she_ding_juli[3]=CCD_juli_dm_she_ding_juli[3]*100 ;  //  1dm  18    到达距离后   关闭  CCD

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
               geng_xin_shuju();//  更新数据
               speed_control_new(); //速度控制 
               protect();
        
               stop_smartcar();    
                 
                 
                 if( CCD_juli_dm_she_ding_juli[1]/100 > 27 )    // 选择是否需要跳出程序     > 100 cm
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
/////////////////////////////调试记录////////////////////////////////////
//======================================================================//
//2.28-------表演成功
//3.1--------32M采速成功，进一步优化处理
//3.3--------PID调试
//3.9--------PID成功，响应很快，
//3.10-------可跑的程序的嵌合，发现采速与舵机的频率冲突
//3.15-------解决问题，改用16M采速并加PID调速，状况良好
//-----------实现可以带PID的简单可跑
//3.16-------初步实现路程计算积分，转向改为每走2CM调整
//3.17-------差速器的初步实现，后续优化
//3.18-------斜置电感与水平电感的联合使用，到目前为止程序的融合
//-----------实现可跑，状态良好
//3.19-------这段时间的调试总结：
//1.独立调速已经比较成熟，但对于整体还有待提高，而这一块与舵机和信号
//处理息息相关，后期加强信号的处理，如何将简单的电感值转化成可以把握
//全局的的一张网，这很重要.
//2.从3.20开始到3.29，做信息的采集以及处理工作，并与舵机结合起来，
//研究算法，同时注意团队协作，分工明确，要达到的目的初步建立整体
//的信息网络模型
//3.第二辆车模的搭建，方案按最远前瞻来定
//3.21-------迷茫，不知道该如何处理数据。。。进度停滞不前！！！！！！！！
//3.22-------计算最大电感位置，粗略确定磁导线所在象限
//3.23想法：小车每走一段采样，距离是多少�---5？10？距离还要精确计算2014
//3.24今天完成小车的改装，用5个电感，3个水平2个垂直，下一步进行偏移量的
//测试.晚上看了往届的视频，深有触动，我们该全力以赴了！！！！！！！！
//3.25小车终于可以跑起来了，直道可以过去，但是还需进一步完善。
//3月底一定让姿态好起来,梦之翼，加油！！！！！！！！！！！！！！
//出现问题：在弯道切出去，偏移量可能有问题，太小，转向不够
//3.27出弯的情况少了很多，但是还没有做到滴水不漏，十字弯可以判断出来
//还要进一步提取，用好信息的前提是信息是准的，所以信息的提取仍然是首位的
//同时提高程序的质量，如果有想法就要立马付诸实施，一刻也不能拖延！！
//3.28今天完整的跑完一圈，上坡的问题解决了，是因为参数给小了，以后不要
//把问题留到后面，及时解决，时间不多了，后期注重团队合作，加快进度
//要快要快.下一步做好循迹(包含偏差和转向),同时提高基础，提高控制策略！！！�
//3.29-4.8这十天困扰了梦之翼一个问题，同样的程序在两辆车上有不用的体现，经检查
//发现：问题车采速不准，可是在老师那里有没有问题，后来还是没有查出问题所在
//由于时间紧迫，所以换了编码器，现在状况良好，同时解决了赛道跑飞的问题，原因在于
//限幅太大，不能够反映速度的突变
//4.9上午和下午分别有罗永革院长和郭一鸣副院长带领领导参观，表演成功，下一步开始做滤波
//和赛道的信息处理
//4.11今天测完赛道总长34.5m，全程最短用时14.984，平均速度2.302m/s
//4.15今天做完新赛道，全长37.26m，最短用时16.122s,平均速度2.32，十字弯有问题
//在里面出不来，梦之翼的”死亡十字“，通过减小模糊控制的输出，状况好了很多
//下一步彻底解决十字道！！！！！！！！！！！！！！！
//4.23重新写赛道识别，效果好了很多，判断稳定了很多
//现在完善滤波、转向、十字弯，一定要使车的姿态稳定！�
//5.3完成新算法，偏差重新确定，PID调速进一步加强
//后续赛道识别，抗干扰，稳定+提速
//========================================================================//
////////////////////////////备战华南赛//////////////////////////////////////
//7.6-7.7----------确定架法
//7.8-7.10---------偏移量以及转向（PD）
//7.11-12----避障
//7.13-17----大量调试
