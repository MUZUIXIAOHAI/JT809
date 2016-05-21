#ifndef _center_h_
#define _center_h_
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include "../comm/MD5.h"
#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <openssl/ssl.h>
#include <openssl/err.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <math.h>

//守护进程需用到的头文件
#include <unistd.h> 
#include <signal.h> 
#include <sys/param.h> 
#include <sys/types.h> 
#include <sys/stat.h> 
//日志文件所用头文件
#include <syslog.h>

//线程所需头文件
#include<pthread.h>

//数据库所需头文件
#include "mysql.h"
//数据包标识
#define LOGIN_RANDOM_SERIAL 0X0000
#define LOGIN_REQ 0X1001
#define LOGIN_RSP 0X9001
#define LOGOUT_REQ 0X1002
#define LOGOUT_RSP 0X9002
#define APPLY_REQ 0X1004
#define APPLY_RSP 0X9004
#define DELIVER 0X0001
#define LINKTEST_REQ 0X1003
#define LINKTEST_RSP 0X9003
//序号	数据包名称	数据包类型	PACKET_ID
//1	登录随机序列	LOGIN_RANDOM_SERIAL	0x0000
//2	登录请求	LOGIN_REQ	0x1001
//3	登录回应	LOGIN_RSP	0x9001
//4	注销请求	LOGOUT_REQ	0x1002
//5	注销回应	LOGOUT_RSP	0x9002
//6	链路检测请求	LINKTEST_REQ	0x1003
//7	链路检测回应	LINKTEST_RSP	0x9003
//8	申请请求	APPLY_REQ	0x1004
//9	申请回应	APPLY_RSP	0x9004
//10	递交	DELIVER	0x0001

//申请命令类型Apply_Cmd_id
//序号	命令类型	CMD_ID
//1	车辆基本资料上报申请	0x0003
//2	车辆图像数据上报申请	0x0004
//3	车辆调度信息发送申请	0x0005
//4	车辆单向电话监听申请	0x0006

#define COMMAND_TYPE_GPS_REAL_FORWORD 0X0001
#define COMMAND_TYPE_GPS_HISTORY_FORWORD 0X0002
#define COMMAND_TYPE_CAR_INFO_UPLOAD 0X0003
#define COMMAND_TYPE_IMAGE_UPLOAD 0X0004
#define COMMAND_TYPE_TEXT_SEND 0X0005
#define COMMAND_TYPE_LISTEN 0X0006
//递交命令类型
#define DELIVER_CMD_ID_POSITION 0X0001
#define DELIVER_CMD_ID_CARINFO 0X0002
#define DELIVER_CMD_ID_IMAGE_DATA 0X0003
#define DELIVER_CMD_ID_PANIC_ALARM 0X0004
#define DELIVER_CMD_ID_SPEED_ALARM 0X0005
#define DELIVER_CMD_ID_PUSH_ALARM 0X0006
#define DELIVER_CMD_ID_POWER_ALARM 0X0007
#define DELIVER_CMD_ID_TIRE_DRIVE 0X0008
#define DELIVER_CMD_ID_CROSS_ALARM 0X0009
//车牌色
#define CAR_COLOR_BLUE 0X01
#define CAR_COLOR_YELLOW 0X02
#define CAR_COLOR_WHITE 0X03
#define CAR_COLOR_BLACK 0X04


//文件格式	编码值
#define IMG_FORMAT_JPEG 1
#define IMG_FORMAT_GIF 2
#define IMG_FORMAT_TIFF 3
#define IMG_FORMAT_PNG 4



//7E
//00-59
//00-01
//48
//00-01-30-31-38-39-30-32-33-36-34-37-02-00-20-10-01-15-11-43-04-02-30-49-07-11-22-65-81-00-30-03-50-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-01-03-20-00-00-00-00-00-00-00-00-00-00
//23

//00-01
//30-31-38-39-30-32-33-36-34-37
//02
//00-20
//10-01-15-11-43-04-02-30-49-07-11-22-65-81-00-30-03-50-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-01-03-20-00-00-00-00-00-00-00-00-00-00
#define APPLY_CMD_OK 0x00000000	//成功
#define APPLY_CMD_ERR_PACKET 0x00000001	//无效数据包
#define APPLY_CMD_ERR_PACKETTYPE 0x00000002	//无效数据包类型
#define APPLY_CMD_ERR_CENTERID 0x00000003	//无效卫星定位监控中心编号
#define APPLY_CMD_ERR_USERID 0x00000004	//无效用户编号
#define APPLY_CMD_ERR_PASSWORD 0x00000005	//密码错误
#define APPLY_CMD_REFUSE_APPLY 0x00000006	//申请拒绝
#define APPLY_CMD_REFUSE_LOGIN 0x00000007	//登录拒绝
#define APPLY_CMD_REFUSE_LOGOUT 0x00000008	//注销拒绝
#define APPLY_CMD_REFUSE_UPLOADIMAGE 0x00000009	//图像上传失败
#define APPLY_CMD_ERR_SEND_TEXT 0x00000010	//调度信息发送失败
#define APPLY_CMD_ERR_LISTEN 0x00000011	//监听失败


//登录回应
#define LOGIN_OK 0X0000
#define LOGIN_ERR_INVAL_PACKET 0X0001
#define LOGIN_ERR_PACKETID 0X0002
#define LOGIN_ERR_CENTERID 0X0003
#define LOGIN_ERR_USER_NO 0X0004
#define LOGIN_ERR_PASSWORD 0X0005
#define LOGIN_ERR_REQ_REFUSE 0X0006
#define LOGIN_ERR_LOGIN_REFUSE 0X0007
#define LOGIN_ERR_LOGOUT_REFUSE 0X0008

#define PACKETTOTALLEN 1024//数据名总长

#define MAXBUF 1024




//车载资料
typedef struct _CarInfo
{
	char CarID[20];//车载机ID
	char CarName[10];//车牌号
	int Color;//车牌色
	char DriverName[12];//司机
	char DriverID[18];//司机身份证号
	char CarPhone[12];//sim
	//char ManufacterID[1];//
    unsigned int ManufacterID;
//	char CenterID[11];//CenterID
	unsigned int CenterID;
	int ServiceStatu;//服务状态2，正常，3，新加入的车，要申请上传基本数据

}CarInfo;


//SOCK数据解析线程参数结构体
typedef struct DecodeArg
	{
	
	char buf[MAXBUF];//数据区
	int len;//长度
	int fd;//SOCK号
	}DecodeArg;
//省中心GPS数据包
typedef struct _CenterGPSPacket
{
char GPS_TIME[7];//yyymmddhhmmss bcd
char LATITUDE[4];//23 12.123 =02 31 21 23
char LONGTITUDE[4];//113 45.608 =11 34 56 08 
char SPEED[2];//120=0120;
char DIRECTION[2];//128=01 28
char ALTITUDE[2];//高度
char MILEAGE[4];//里程单位米
char DRIVER_NAME[12];//驾驶员姓名
char DRIVER_IDCARDNO[18];//驾驶员身份证号
unsigned char VALID;//定位有效性
unsigned char sta10_EW:1;//东西经
unsigned char sta11_NS:1;//南北纬
unsigned char sta12_PanicAlarm:1;//紧急报警
unsigned char sta13_OilOff:1;//断油
unsigned char sta14_SpeedAlarm:1;//超速报警
unsigned char sta15_ZhengdongAlarm:1;//震动报警
unsigned char sta16_PowerOff:1;//主电源断开
unsigned char sta17_PowerVol:1;//主电源电压
unsigned char sta20_ShaChe:1;//刹车
unsigned char sta21_DoorOpen:1;//车门
unsigned char sta22_LeftLigth:1;//左灯
unsigned char sta23_RigthLigth:1;//右灯
unsigned char sta24_FarLigth:1;//远灯
unsigned char sta25_ACC:1;//ACC
unsigned char sta26_NA:1;//NA
unsigned char sta27_NA:1;//NA
unsigned char sta30_GPSLock:1;//GPSLock
unsigned char sta31_GPSShort:1;//定位天线短路
unsigned char sta32_GPSOff:1;//定位天线开路
unsigned char sta33_GPSTruble:1;//定位模块故障
unsigned char sta34_GSMTruble:1;//通信模块故障
unsigned char sta35_CrossOut:1;//出区报警
unsigned char sta36_CrossIn:1;//入区报警
unsigned char sta37_NA:1;//保留
unsigned char sta40_BattTruble:1;//备用电池故障
unsigned char sta41_CrossAlarm:1;//电子围栏报警
unsigned char sta42_EnginStart:1;//发动机开
unsigned char sta43_TireDriveAlarm:1;//疲劳驾驶报警
unsigned char sta44_NA:1;//na
unsigned char sta45_NA:1;//na
unsigned char sta46_NA:1;//NA
unsigned char sta47_NA:1;//NA
unsigned char sta5;//第五状态字
unsigned char sta6;
unsigned char sta7;
unsigned char sta8;
}CenterGPSPacket;
int GetAPacket(char * InBuf,int inlen,char * out,char* res,int * leslen);
int SJT_Connected(int fd);//联上后发送的数据
int CGJ_Connected(int fd);
void * BSJ_UDP_function(void * arg);
void * DecodeCGJServerData_thread(DecodeArg * arg);
void * DecodeGPSServerData_thread(DecodeArg * arg);
unsigned char Val2BCD(unsigned char x) ;
void * 	Connect_GPS(void * sig);
void * DecodeCenterData_thread(DecodeArg * arg);
void * 	TimerCenter(void * sig);
int GetStr(char* RecvBuf,int nRecvLen,char  StartSyb,char  EndSyb,char* pOutBuf,int *pRestBuf,int * RestLen);
int GetGPS_CarInfo_Forword(char* CarName,char Color,int fd);
int ChangCGJ2JT808(char *Data,int DataLen,int fd,int ptype);
int DoDeliver(short DELIVER_CMD_ID,char* CarName,char Color,
			  unsigned short CMD_len,char * CMD_content,
			  int fd,char * ret);
unsigned char BCD2Val(unsigned char x);
int StrTime2BCD(char * str,char *BCD);
int lpt_show_login(int no);
int lpt_UpdateCarData(int fd,char * bookuserlist);
int GetGPS_Image_Forword(char* CarName,char Color,int fd);
int GetGPS_Real_Forword(char* CarName,char Color,int fd);
int GetCarHistory(char * CarName,char Color,char* StartDateTime,char* EndDateTime,int fd);
int GetGPS_Real_Forword(char* CarName,char Color,int fd);
int DoApply(short Apply_Cmd_id,char* CarName,char Color, unsigned short CMD_len,char * CMD_content,int fd);
int DoApplyRsp(char* Body,int BodyLen);
int lpt_login(char *username,char * password,int fd);
int DecodeCenterPacket(char * Data ,unsigned int DataLen,int fd);
int DoLogRep(char* Body,int BodyLen);
int lpt_upload_one_car(int fd,char * up_info);
int SendPacket2Center(char * Packet,int PacketLen ,int fd);
int DoLogOut(int fd);
int DoLogin(char * serial,char* userid,char * password,int fd);
int DecodePacket(char * Data ,int DataLen,char * Body,unsigned int*  PacketID);
int DoPacket(unsigned int PacketID,char GPScenter,char* DataBody ,int DataBodyLen,char* b);
//中心平台代码
int SendPing2GPS(int fd);
	int lpt_send_msg(int fd,char* Data,int contentlen);
int GPS_Connected(int fd);
char * ToUpper(char*in);
int lpt_decode_data(int fd,char * Data,int len);
int cb_send_test_link(int fd);
int lpt_send_test_link(int fd);
int SendPacket2GPS(char * Packet,int PacketLen ,int fd);
int Connect_Center_server(char * ServerIP,int Port,char* ServerName,
						  void *(*Data_Thread)( DecodeArg  * arg),
						  int(*Connected_Fun)(int fd),
						  int(*Close_Fun)(int fd)
						  );
int GetCarIndexFromCarID(char * CarID);
int GPSClose(int fd);
int CenterClose(int fd);
//int Connect_Center_server(char * ServerIP,int Port,char* ServerName,void *(*Data_Thread)( DecodeArg  * arg));
//int Connect_Center_server(char * ServerIP,int Port,char* ServerName);
int SendPing2Center(int fd);
int DeliveGPS_Position(char * data,char* CarName,char Color,int fd,int len);

int Chang2GPSPacket(char *Data,int DataLen,int fd,CenterGPSPacket* track,char * CarID);
int str2bcd(char * str,char * data,int len);

int Connect_Data_Server(MYSQL** cconn_ptr,char * host,char * username,char * mysqlpassword ,
						char * database,int dbport,char * Con_Prompt);
void * DecodeCenterData_thread1(DecodeArg * arg);
int CenterClose1(int fd);
void * 	Connect_JTXX_thread(void * sig);
int DecodeCenterPacket1(char * Data ,unsigned int DataLen,int fd);
int DeliveImage(char *CarName,char Color);//回应并递交图像数据;
int DoApplyRequest(char *Body,int BodyLen);
int UploadCarInfo(char *CarName,char Color);
int ChangCGJ2GPSPacket(char *Data,int DataLen,int fd,CenterGPSPacket * track,char * CarID);
int DoCarInfoUploadApply(char* CarName,char Color);
int ChangData2GPSPacket(char *Va,char * sCarID,char * strTime,double dx,double dy,int speed ,int dir,
						char * s1,char * s2,char* s3,int fd,CenterGPSPacket * track);
#endif


