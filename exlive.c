#include "exlive.h"
#include "../comm/MD5.h"
#include "../comm/comm.h"
#include "DB44dec.h"
#include <string.h>
#include <unistd.h>
#define MAXCARNUMBER 10000 //一万台车入网
//JT809协议
#define UP_CONNECT_REQ 0X1001 //主链路登录请求消息
#define UP_LINKTEST_REQ 0X1005 //主链路链接保持请求消息
#define UP_EXG_MSG_REAL_LOCATION 0X1202 //实时上传车辆定位信息
#define UP_EXG_MSG 0x1200 //主链路动信息交换
#define UP_CONNECT_RSP 0x1002 //
int JT809_up_gps_track(char * carname,char color,short date_type,char * data,int datalen,char * packetdata);
int JT809_GPS_packet(char * gpsdate,double lon,double lat,int speed,int speed2,int mile, int heading,int height,char* statu,char *alarm,unsigned char * packetdata);
int cb_lpt_sendto(int fd,char * packetdata,int len);
int JT809_DO_TESTLINK(int fd);
char RANDOM_SERIAL[10];//随机序列
char UserID[20];
char ocm_code[20];//运营商代码
char PassWord[50];
DecodeArg arg;
int DebugMode=0;//是否调试模式
int ProtocolID=0;//转发协议
int key=0x00002334;//Key
int Center_sock_fd,GPS_sock_fd,Center_sock_fd1;
int JT_GPS_sock=-1 ;//服务器TCP接入
int UDPTCP_mode=0;//以何种方式转发0:udp 1:tcp
char DataIP[20];//=127.0.0.1
char Booker[20];//订阅者
char dbuser[20];//=sjt
char dbpwd[20];//jhydz203
char dbname[20];//sjt
int LoadIMF(char * imf,char * imd);
char GPSLogInString[100];//GPS登录字符
char GPS_BeatHear_String[100];//GPS心调字符
char GPSIP[100];//GPS ip
int GPSPORT=8914;
char GPSCETNTER='E';
unsigned int DebugID;
static long jt809seq=0;
unsigned short packetSQE=0;//报文序号
//#define GPSCETNTER 'H'
//int DataPort=3306;
//省中心参数
int CENTERPORT=12501;
//12501-12510
char CENTERIP[20];//=10.0.64.186
char CENTERUSER[20];//=test
char CENTERPASSWORD[20];//=test
char SERVERNAME[20];//Shengiaotong
int UDP_Send_Sock=0;//转发的UDP
int cb_loginstatu=0;//长宝中心登录状态，1为已登录。
//交通信息中心参数
int ProtocolType=0;
int CENTERPORT1=12501;
//12501-12510
char CENTERIP1[20];//=10.0.64.186
char CENTERUSER1[20];//=test
char CENTERPASSWORD1[20];//=test
char SERVERNAME1[20];//Shengiaotong
int DataPort=3306;
int Center_Beat_Heart,Center_Beat_Heart1;
int GPS_Beat_Heart;
CarInfo CarData[MAXCARNUMBER];
int InitCarData();
int cb_decode_data(int fd,char * Data,int len);
int JT809_login_data(int userid,char * password,char * down_link_ip,short down_link_port,char * packetdata);
int cb_encode_data(char * funword,char * ocm_code,int datalen,char * data,char * packetdata);
int GetCarTrack();
void * 	Connect_Server(void * sig);
int JT809_encode_data(int  funword,int  ocm_code, char * data, int datalen,char * packet);
void * SockRecive_thread(DecodeArg * arg);
int SendDataUDP(int FromFD,char * buf,int len,char * udp_ip,int udp_port,int fd);
int SockClose_thread(int fd);
//int cd_decode_data(int fd,char * Data,int len);
int SockConect_thread(int fd);

int GetACWPacket(char * buff,int len,int fd);
int ReadConfigFile();
//unsigned char TestBit(char a,unsigned int no)
//{
//if ( (a & (1<<no))>0)
//return 1;
//else
//return 0;
//}

typedef struct _RoadXY
{
	double Lon;
	double Lat;
	int index;//点顺序号
	struct _RoadXY * NextRoadXY;//下一个点
}RoadXY;
typedef struct _Road
{
	int RoadNo;//道路编号
	int PoitsCounts;//总共多少坐标点
	double MinLon;
	double MaxLon;
	double MinLat;
	double MaxLat;
	char RoadName[100];//道路名称
	RoadXY XY;//点数据链表头
}Road;

/******该文件使用查表法计算CCITT 标准的CRC-16检验码，并附测试代码********/
//#include <stdio.h>
#define CRC_INIT 0xffff   //CCITT初始CRC为全1
#define GOOD_CRC 0xf0b8   //校验时计算出的固定结果值
/****下表是常用ccitt 16,生成式1021反转成8408后的查询表格****/
unsigned short crc16_ccitt_table[256] =
{
0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x77, 0x643e,
0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
/*****CRC计算函数,可将第一个参数reg_init简化掉********/
unsigned short do_crc(unsigned short reg_init, unsigned char *message, unsigned int len)
{
    unsigned short crc_reg = reg_init;

    while (len--)
        crc_reg = (crc_reg >> 8) ^ crc16_ccitt_table[(crc_reg ^ *message++) & 0xff];

    return crc_reg;
}


void * BSJ_UDP_function(void * arg)
{
	struct sockaddr_in s_addr;
	struct sockaddr c_addr;//客户端地址结构
	int sock;
	//pthread_t tidtcp;
	socklen_t addr_len;
	int len;
	char buff[MAXBUF+1];
	if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
	{
		syslog(LOG_ERR|LOG_LOCAL0,"\nCan not Create UDP sock!\n");
		exit(errno);
	}


	memset(&s_addr, 0, sizeof(struct sockaddr_in));

	/* 设置地址和端口信息 */
	s_addr.sin_family = AF_INET;
	s_addr.sin_port = htons(CENTERPORT1);
	s_addr.sin_addr.s_addr = INADDR_ANY;

	/* 绑定地址和端口信息 */
	if ((bind(sock, (struct sockaddr *) &s_addr, sizeof(s_addr))) == -1)
	{
		syslog(LOG_ERR|LOG_LOCAL0,"Can not BIND BSJ UDP sock! port:%d\n",ntohs(s_addr.sin_port));
		exit(errno);
	}
	syslog(LOG_ERR|LOG_LOCAL0,"BIND BSJ UDP sock Sucess! port:%d\n",ntohs(s_addr.sin_port));



	/* 循环接收数据 */
	addr_len = sizeof(c_addr);
	/*unsigned char	MainMsg, CRC,SecondMsg;*/
	/*	int ret,ACKLen;
	char  ACK[1024];*/
	// BSJ_UDP_LOOP
	/*	int i;
	pthread_t tidudp[10];
	for(i=0;i<3;i++)
	{

	if (pthread_create(&tidudp[i],NULL,BSJ_UDP_LOOP,(void *)&sock))
	{
	syslog(LOG_INFO|LOG_USER,"Can not Create BSJ-UDP_LOOP pthread!");
	exit(1);
	}
	}*/

	UDP_Send_Sock=sock;
	while (1)
	{
		//pthread_mutex_lock(&BSJUDPMutex);
		//pthread_mutex_unlock(&BSJUDPMutex);
		len = recvfrom(	sock, buff,MAXBUF, 0,(struct sockaddr *) &c_addr, &addr_len);
		//pthread_mutex_unlock(&BSJUDPMutex);
		if (len <= 0)
		{
			syslog(LOG_ERR|LOG_LOCAL0,"UDP recvfrom error,err:%s\n",strerror(errno));
			//UDP_Send_Sock=-1;
			continue;
		}
		else
		{
			UDP_Send_Sock=sock;
			syslog(LOG_ERR|LOG_LOCAL0,"%s%dUDP recv:%s\n",__FILE__,__LINE__,buff);

		}

	}
	return 0;
}


int SendDataUDP(int FromFD,char * buf,int len,char * udp_ip,int udp_port,int jtfd)
{
	struct sockaddr_in UDP_addr;
	int result=0;
	if (jtfd>0)
	{
		result=send(jtfd,buf,len,0);

	}
	else
	{

		memset(&UDP_addr, 0, sizeof(struct sockaddr_in));
		/* 设置转发的UDP地址和端口信息 */
		UDP_addr.sin_family = AF_INET;
		UDP_addr.sin_port = htons(udp_port);
		//UDP_addr.sin_addr.s_addr = INADDR_ANY;
		UDP_addr.sin_addr.s_addr=inet_addr(udp_ip);
		//result=sendto(udp_fd,ReplyData,result,0,(struct sockaddr *)&Clinet_addr,sizeof(struct sockaddr_in));
		result=sendto(FromFD,buf,len,0,(struct sockaddr *)&UDP_addr,sizeof(struct sockaddr_in));
	}

	if (result<=0)
		syslog(LOG_INFO|LOG_USER,"%s%dsend data udp error! %s jtfd:%d \n",__FILE__,__LINE__,strerror(errno),jtfd);
		//syslog(LOG_INFO|LOG_USER,"%s%dsend data udp error! %s fd:%d \n",__FILE__,__LINE__,strerror(errno),jtfd);
	return result;

}



int ReadConfigFile(char * ConfigFileName)
{
	char string[50];
	memset(string,0,50);
	//GPSCETNTER
	if (AnalizeConfFile(ConfigFileName,"Booker",string)!=0)
		strcpy(Booker,"(%fsjtj%)");
	else
		strcpy(Booker,string);
	if (AnalizeConfFile(ConfigFileName,"OMC",string)!=0)
		strcpy(ocm_code,"8888");
	else
		strcpy(ocm_code,string);

	if (AnalizeConfFile(ConfigFileName,"GPSIP",string)!=0)
		strcpy(GPSIP,"211.147.224.218");
	else
		strcpy(GPSIP,string);
	if (AnalizeConfFile(ConfigFileName,"GPS_BeatHear_String",string)!=0)
		strcpy(GPS_BeatHear_String,"[PING|fsjtj|1234]");
	else
		strcpy(GPS_BeatHear_String,string);
	strcpy(GPSLogInString,string);
	if (AnalizeConfFile(ConfigFileName,"GPSLogInString",string)!=0)
		strcpy(GPSLogInString,"[PING|fsjtj|1234]");
	else
		strcpy(GPSLogInString,string);
	if (AnalizeConfFile(ConfigFileName,"SERVERNAME",string)!=0)
		strcpy(SERVERNAME,"Shengiaotong");
	else
		strcpy(SERVERNAME,string);
	if (AnalizeConfFile(ConfigFileName,"SERVERNAME1",string)!=0)
		strcpy(SERVERNAME1,"jtxx");
	else
		strcpy(SERVERNAME1,string);
	if (AnalizeConfFile(ConfigFileName,"CENTERPASSWORD",string)!=0)
		strcpy(CENTERPASSWORD,"ICyX,S6i");
	else
		strcpy(CENTERPASSWORD,string);
	if (AnalizeConfFile(ConfigFileName,"CENTERPASSWORD1",string)!=0)
		strcpy(CENTERPASSWORD1,"4vvcdpx4mx5p2tsmxyvp1l");
	else
		strcpy(CENTERPASSWORD1,string);

	if (AnalizeConfFile(ConfigFileName,"CENTERUSER",string)!=0)
		strcpy(CENTERUSER,"FDrmf6ab");
	else
		strcpy(CENTERUSER,string);
	if (AnalizeConfFile(ConfigFileName,"CENTERUSER1",string)!=0)
		strcpy(CENTERUSER1,"ZQ00000001");
	else
		strcpy(CENTERUSER1,string);

	if (AnalizeConfFile(ConfigFileName,"CENTERIP",string)!=0)
		strcpy(CENTERIP,"10.0.64.186");
	else
		strcpy(CENTERIP,string);
	if (AnalizeConfFile(ConfigFileName,"CENTERIP1",string)!=0)
		strcpy(CENTERIP1,"121.33.200.103");
	else
		strcpy(CENTERIP1,string);

	if (AnalizeConfFile(ConfigFileName,"GPSPORT",string)!=0)
		GPSPORT=8914;
	else
		GPSPORT=atoi(string);
	if (AnalizeConfFile(ConfigFileName,"CENTERPORT",string)!=0)
		CENTERPORT=12501;
	else
		CENTERPORT=atoi(string);
	if (AnalizeConfFile(ConfigFileName,"CENTERPORT1",string)!=0)
		CENTERPORT1=10071;
	else
		CENTERPORT1=atoi(string);

	if (AnalizeConfFile(ConfigFileName,"DataIP",string)!=0)
		strcpy(DataIP,"127.0.0.1");
	else
		strcpy(DataIP,string);

	if (AnalizeConfFile(ConfigFileName,"dbuser",string)!=0)
		strcpy(dbuser,"lml");
	else
		strcpy(dbuser,string);

	if (AnalizeConfFile(ConfigFileName,"dbpwd",string)!=0)
		strcpy(dbpwd,"jhydz203");
	else
		strcpy(dbpwd,string);
	if (AnalizeConfFile(ConfigFileName,"dbname",string)!=0)
		strcpy(dbname,"gprs");
	else
		strcpy(dbname,string);

	if (AnalizeConfFile(ConfigFileName,"DataPort",string)!=0)
		DataPort=3306;
	else
		DataPort=atoi(string);
	if (AnalizeConfFile(ConfigFileName,"CenterID",string)!=0)
		GPSCETNTER='E';
	else
		GPSCETNTER=atoi(string);

	if (AnalizeConfFile(ConfigFileName,"DebugMode",string)!=0)
		DebugMode=1;
	else
		DebugMode=atoi(string);
	if (AnalizeConfFile(ConfigFileName,"ProtocolType",string)!=0)
		ProtocolType=0;
	else
		ProtocolType=atoi(string);
	if (AnalizeConfFile(ConfigFileName,"key",string)!=0)
		key=0x00002334;
	else
		key=atoi(string);
	if (AnalizeConfFile(ConfigFileName,"TCPUDP",string)!=0)
		UDPTCP_mode=0;
	else
		UDPTCP_mode=atoi(string);

	if (AnalizeConfFile(ConfigFileName,"OMC",string)!=0)
		strcpy(ocm_code,"1443233814");
	else
		strcpy(ocm_code,string);

	//
	////E=69 H=72 W=87
	//CenerID=69



	return 0;
}
int main(int argc,char **argv)
{

	char cnfilename[100];
	sprintf(cnfilename,"%s.conf",argv[0]);
	ReadConfigFile(cnfilename);
	printf("config_file name:%s Complie date:%s %s,DataIP:%s,dbuser:%s,dbpwd:%s,\n"
		"dbname:%s,DataPort:%d,CenterPort:%d,CenterIP:%s,CenterUser:%s,\nCenterPassWord:%s,ServerName:%s,CenterPort1:%d,CenterIP1:%s,CenterUser1:%s,CenterPassWord1:%s,ServerName1:%s,TCPIP:%d,OMC:%s\n",
		cnfilename,__DATE__,__TIME__,DataIP,dbuser,dbpwd,dbname,DataPort,CENTERPORT,CENTERIP,CENTERUSER,CENTERPASSWORD,SERVERNAME,CENTERPORT1,CENTERIP1,CENTERUSER1,CENTERPASSWORD1,SERVERNAME1,UDPTCP_mode,ocm_code);
	InitCarData();
#ifndef DEBUG
	daemon_init(argv[0],LOG_USER);//初始化为守护进程
#endif
	signal(SIGPIPE,SIG_IGN);//关闭信号响应
	signal(SIGALRM, SIG_IGN);//alarm(1);
	signal(SIGHUP, SIG_IGN);
	signal(SIGINT, SIG_IGN);
	InitSignal();

	pthread_t tidtimer;//定时器
	if (pthread_create(&tidtimer,NULL,TimerCenter,NULL))
	{
		syslog(LOG_INFO|LOG_USER,"Can not Create Center Timer pthread!");
		exit(1);
	}

	if (UDPTCP_mode>0)//TCP模式转发//TCPUDP=1;//UDP:0 TCP:1
	{
			syslog(LOG_INFO|LOG_USER,"%s %d Connect Center Use TCP\n",__FILE__,__LINE__);
		pthread_t tidtcp;//TCP
		if (pthread_create(&tidtcp,NULL,Connect_Server,NULL))
		{
			syslog(LOG_INFO|LOG_USER,"Can not Create JT-TCP_function pthread!");
			exit(1);
		}
	}
	else //UDP模式转发
	{
		syslog(LOG_INFO|LOG_USER,"%s %d Connect Center Use TCP\n",__FILE__,__LINE__);

		pthread_t tidudp;//UDP
		if (pthread_create(&tidudp,NULL,BSJ_UDP_function,NULL))
		{
			syslog(LOG_INFO|LOG_USER,"Can not Create UDP_function pthread!");
			exit(1);
		}
	}





	//Connect_Server
	Connect_GPS(NULL);
	//pthread_t tidGPS;//中心GPS数据
	//if (pthread_create(&tidGPS,NULL,Connect_GPS,NULL))
	//{
	//	syslog(LOG_INFO|LOG_USER,"Can not Create Connect GPS pthread!");
	//	exit(1);
	//}
	//pthread_t tidjtxx;//交通信息数据
	//if (pthread_create(&tidjtxx,NULL,Connect_JTXX_thread,NULL))
	//{
	//	syslog(LOG_INFO|LOG_USER,"Can not Create Connect JTXX pthread!");
	//	exit(1);
	//}
	//while(1)
	//	{
	//syslog(LOG_INFO|LOG_USER,"Ready Connect to Center IP:%s port:%d username:%s pasword:%s\n",CENTERIP,CENTERPORT,CENTERUSER,CENTERPASSWORD);
	////联接省厅

	//Connect_Center_server(CENTERIP,CENTERPORT,SERVERNAME,DecodeCenterData_thread,SJT_Connected,CenterClose);
	//Center_sock_fd=-1;
	//sleep(3);
	//	}
	/*

	char * a;
	Connect_GPS(a);*/

	return 0;
}

void * 	Connect_JTXX_thread(void * sig)
	//void Connect_GPS()//联接到GPS服务器
{
	Connect_Center_server(CENTERIP1,CENTERPORT1,SERVERNAME1,DecodeCenterData_thread1,NULL,CenterClose1);
	return 0;
}


int GPSClose(int fd)
{
	close(fd);
	GPS_sock_fd=-1;
	return 1;
}
int CGJClose(int fd)
{
	close(GPS_sock_fd);
	GPS_sock_fd=-1;
	return 1;
}

int CenterClose(int fd)
{
	close(fd);
	Center_sock_fd=-1;
	return 1;
}
int CenterClose1(int fd)
{
	close(fd);
	Center_sock_fd1=-1;
	return 1;
}
///收到定位数据包的处理
int DecodeCGJGPSServer(char * Data ,unsigned int DataLen,int fd)
{
	//[RETR|粤A17264|2000143054|2011-01-22 10:44:32|114.9268|21.8207|无效|0|0|0|0|车匙关|正常|.|1|1|0|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|0|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|114.9224|21.8235|jh10]
	//[RETR|粤A169PG|2000146023|2011-01-22 10:44:32|113.3205|23.1382|精确|0|0|0|0|车匙关|正常|广东省广州市天河区体育西路,|1|1|0|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|0|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|113.3151|23.1408|jh10]

	CenterGPSPacket track;
	char CarID[20];
	memset(&track,0,sizeof(CenterGPSPacket));
	int TrackCount;
	TrackCount=sizeof(CenterGPSPacket);
	TrackCount=ChangCGJ2GPSPacket(Data,DataLen,fd,&track,CarID);//产生递交数据体在TRACK中.
	if	(TrackCount<=0)
		return 0;

	int Carindex=0;
	Carindex=GetCarIndexFromCarID(CarID);
	if (Carindex<0)
	{
		syslog(LOG_INFO|LOG_USER,"%s%dUnknow Car CarID: %s\n",__FILE__,__LINE__,CarID);
		InitCarData();
		return -1;
	}

	char CarName[10];
	memset(CarName,0,10);
	sprintf(CarName,"%s",CarData[Carindex].CarName);

	char CarColor;
	CarColor=CarData[Carindex].Color;

	if(CarData[Carindex].ServiceStatu>=2 )   //新加入的车，或更改过资料的车，需要重新上传基本资料到省中心
	{
		syslog(LOG_INFO|LOG_USER,"%s%d ServiceStatu:%d CarName:%s\n",__FILE__,__LINE__,CarData[Carindex].ServiceStatu,CarName);

		DoCarInfoUploadApply(CarName,CarColor);//请求上传车辆资料
		UploadCarInfo(CarName,CarColor);
		CarData[Carindex].ServiceStatu=1;

	}
	char DeliverData[1024];
	memset(DeliverData,0,1024);

	char CenterData[1024];
	memset(CenterData,0,1024);
	int datalen,CenterDataLen;
	//产生递交数据包
	datalen=DoDeliver(DELIVER_CMD_ID_POSITION,CarName,CarColor,
		sizeof(CenterGPSPacket),(char*)&track,fd,DeliverData);

	CenterDataLen=DoPacket(DELIVER,GPSCETNTER,DeliverData,datalen,CenterData);//编码成数据包
	//发送
	TrackCount=SendPacket2Center(CenterData,CenterDataLen,Center_sock_fd);
	if (DebugMode<1)
		syslog(LOG_INFO|LOG_USER,"%s%d CarName:%s,Color:%d Track Send %dbytes OK,fd%d\n",__FILE__,__LINE__,CarName,CarColor,TrackCount,Center_sock_fd);


	//SendPacket2Center(CenterData,CenterDataLen,Center_sock_fd1);
	return 1;
}
//收到定位数据的处理
int DecodeGPSServer(char * Data ,unsigned int DataLen,int fd)
	//DecodeGPSServer()
{
	Data[DataLen]=0;
	syslog(LOG_INFO|LOG_USER,"%s%d fd:%d Comalie Data:%s %s Get GPS Packet %s\n",__FILE__,__LINE__,fd,__DATE__,__TIME__,Data);
	/*if ((unsigned char)Data[0]==0x4e)
	{
	char a[]={0x42,0x4D,0x4F,0x44,0x20,0x30,0x20,0x0,0x0,0xD,0xA};
	if (send(fd,a,11,0)>0)
	syslog(LOG_INFO|LOG_USER,"Send Packet to GPS:%s\n",a);
	}
	*/
	if (strstr(Data,"EINI")==Data)
	{//Client->Server[INFO C:\hjygps 5.0600 黄金眼GPS监控系统专业版]
		//	sleep(5);
		char B[]={0x49,0x4E,0x46,0x4F,0x20,0x43,0x3A,0x5C,0x68,0x6A,0x79,0x67,
			0x70,0x73,0x20,0x35,0x2E,0x30,0x36,0x30,0x30,0x20,0xBB,0xC6,0xBD,
			0xF0,0xD1,0xDB,0x47,0x50,0x53,0xBC,0xE0,0xBF,0xD8,0xCF,0xB5,0xCD,
			0xB3,0xD7,0xA8,0xD2,0xB5,0xB0,0xE6,0x20,0xD,0xA,0x42,0x4D,0x4F,0x44,
			0x20,0x30,0x20,0x0,0x0,0xD,0xA,0x41,0x44,0x44,0x54,0x20,0x73,0x6A,0x74,
			0x20,0x4A,0x37,0x35,0x35,0x20,0x32,0x30,0x30,0x30,0x31,0x36,0x31,0x35,
			0x30,0x36,0x20,0xD,0xA,0x41,0x44,0x44,0x54,0x20,0x73,0x6A,0x74,0x20,0x4D,
			0x4C,0x31,0x30,0x20,0x32,0x30,0x33,0x30,0x31,0x36,0x34,0x33,0x34,0x33,0x20,
			0xD,0xA,0x41,0x44,0x44,0x54,0x20,0x73,0x6A,0x74,0x20,0x4A,0x37,0x35,0x35,
			0x20,0x36,0x30,0x39,0x30,0x33,0x31,0x39,0x36,0x35,0x31,0x20,0xD,0xA,0x41,
			0x44,0x44,0x54,0x20,0x73,0x6A,0x74,0x20,0x4A,0x37,0x35,0x35,0x20,0x36,0x30,
			0x39,0x30,0x33,0x30,0x39,0x31,0x36,0x37,0x20,0xD,0xA,0x41,0x44,0x44,0x54,0x20,
			0x73,0x6A,0x74,0x20,0x4A,0x37,0x35,0x35,0x20,0x37,0x30,0x39,0x31,0x32,0x30,
			0x37,0x32,0x30,0x31,0x20,0xD,0xA,0x41,0x44,0x44,0x54,0x20,0x73,0x6A,0x74,0x20,
			0x4A,0x37,0x35,0x35,0x20,0x32,0x30,0x33,0x30,0x31,0x36,0x33,0x32,0x37,0x37,
			0x20,0xD,0xA,0x41,0x44,0x44,0x54,0x20,0x73,0x6A,0x74,0x20,0x4A,0x37,0x35,0x35,
			0x20,0x36,0x30,0x39,0x30,0x35,0x30,0x37,0x38,0x32,0x33,0x20,0xD,0xA};
		//	char B[]={0x42,0x4D,0x4F,0x44,0x20,0x30,0x20,0x0,0x0,0xD,0xA};
		if (send(GPS_sock_fd,B,248,0)>0)
			syslog(LOG_INFO|LOG_USER,"fd:%d Send Packet BOOK to GPS:%s\n",GPS_sock_fd,B);

	}


	if (strstr(Data,"NETD")==Data)
	{
		//Client->Server[INFO C:\hjygps 5.0600 黄金眼GPS监控系统专业版]
		char a[]={0x49,0x4E,0x46,0x4F,0x20,0x43,0x3A,0x5C,0x68,0x6A,0x79,0x67,0x70,0x73,0x20,0x35,0x2E,0x30,0x36,0x30,0x30,0x20,0xBB,0xC6,0xBD,0xF0,0xD1,0xDB,0x47,0x50,0x53,0xBC,0xE0,0xBF,0xD8,0xCF,0xB5,0xCD,0xB3,0xD7,0xA8,0xD2,0xB5,0xB0,0xE6,0x20,0xD,0xA};
		if (send(GPS_sock_fd,a,48,0)>0)
			syslog(LOG_INFO|LOG_USER,"fd:%dSend Packet to GPS:%s\n",GPS_sock_fd,a);
		//sleep(1);
	}



	if (strstr(Data,"TRAC")==Data)
	{
		//syslog(LOG_INFO|LOG_USER,"%s%dGet Track Packet %s\n",__FILE__,__LINE__,Data);
		//TRAC 0 013189340518 UDP ZQ10 67472625 13837714 2010-01-14/  :26:54 óDD§ 10 200 ?y3￡ μ??e|??3μ ?y3￡ ?¨ê±??±¨|1???ê?￡????ìêD￡????Y?? 0

		CenterGPSPacket track;
		char CarID[20];
		memset(&track,0,sizeof(CenterGPSPacket));
		int TrackCount;
		TrackCount=sizeof(CenterGPSPacket);
		//TrackCount=Chang2GPSPacket(Data,DataLen,fd,&track,CarID);//产生递交数据体在TRACK中.
		int Carindex=0;
		Carindex=GetCarIndexFromCarID(CarID);
		if (Carindex<0)
		{
			syslog(LOG_INFO|LOG_USER,"%s%dUnknow Car CarID: %s\n",__FILE__,__LINE__,CarID);
			return -1;
		}
		char CarName[10];
		memset(CarName,0,10);
		sprintf(CarName,"%s",CarData[Carindex].CarName);

		char CarColor;
		CarColor=CarData[Carindex].Color;
		char DeliverData[1024];
		memset(DeliverData,0,1024);

		char CenterData[1024];
		memset(CenterData,0,1024);
		int datalen,CenterDataLen;
		//产生递交数据包
		datalen=DoDeliver(DELIVER_CMD_ID_POSITION,CarName,CarColor,
			sizeof(CenterGPSPacket),(char*)&track,fd,DeliverData);

		CenterDataLen=DoPacket(DELIVER,GPSCETNTER,DeliverData,datalen,CenterData);//编码成数据包
		//发送
		SendPacket2Center(CenterData,CenterDataLen,Center_sock_fd);
		SendPacket2Center(CenterData,CenterDataLen,Center_sock_fd1);
	}
	return 0;
}


//TRAC 0 013189340518 UDP ZQ10 67472625 13837714 2010-01-14/  :26:54 óDD§ 10 200 ?y3￡ μ??e|??3μ ?y3￡ ?¨ê±??±¨|1???ê?￡????ìêD￡????Y?? 0
int ChangCGJ2GPSPacket(char *Data,int DataLen,int fd,CenterGPSPacket * track,char * CarID)
{
	char buff[2048];
	memcpy(buff,Data,DataLen);
	int ind=0;
	char *ptr=NULL;
	char* pa[50];
	char * p=NULL;
	p=(char*)buff;
	while((pa[ind]=strtok_r(p,"[|]",&ptr))!=NULL)
	{
		ind++;
		p=NULL;
		if (ind>49)
			break;
	}

	if( ind<15 )

		return -1;
	//[RETR|粤A17264|2000143054|2011-01-22 10:44:32|114.9268|21.8207|无效|0|0|0|0|车匙关|正常|.|1|1|0|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|0|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|114.9224|21.8235|jh10][RETR|粤A169PG|2000146023|2011-01-22 10:44:32|113.3205|23.1382|精确|0|0|0|0|车匙关|正常|广东省广州市天河区体育西路,|1|1|0|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|0|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|113.3151|23.1408|jh10]
	//
	//定位有效性
	track->VALID=(strstr(pa[6],"精确")!=NULL)?1:0;
	//车机机ID

	strcpy(CarID,pa[2]);

	unsigned int year,m,d,h,mm,s;
	sscanf(pa[3],"%04d-%02d-%02d %02d:%02d:%02d",&year,&m,&d,&h,&mm,&s);
	char strTime[40];
	sprintf(strTime,"%04d%02d%02d%02d%02d%02d",year,m,d,h,mm,s);
	str2bcd(strTime,track->GPS_TIME,7);
	double x,y;
	x=atof(pa[4]);
	y=atof(pa[5]);
	char sx[20];
	//经度	 113.2345
	int du,fen;
	du=x;  //整数度	113
	fen=(x-du)*60.0*1000;//整数分14.0700*1000=14070
	sprintf(sx,"%03d%05d",du,fen);//11314070
	str2bcd(sx,track->LONGTITUDE,4);
	du=y;  //整数度	23
	fen=(y-du)*60.0*1000;//整数分14.0700*1000=14070
	sprintf(sx,"%03d%05d",du,fen);//02314070
	str2bcd(sx,track->LATITUDE,4);
	char sy[100];
	//速度
	sprintf(sy,"%04d",atoi(pa[8]));
	str2bcd(sy,track->SPEED,2);
	//方向
	sprintf(sy,"%04d",atoi(pa[7]));
	str2bcd(sy,track->DIRECTION,2);

	//坐标方向,中国
	track->sta10_EW=1;
	track->sta11_NS=1;
	//报警状态在pa12
	track->sta41_CrossAlarm=(strstr(pa[12],"越")!=NULL)?1:0;
	track->sta32_GPSOff=(strstr(pa[12],"GPS开路")!=NULL)?1:0;


	//车辆状态在pa13
	track->sta25_ACC=(strstr(pa[13],"点火")!=NULL)?1:0;
	track->sta21_DoorOpen=(strstr(pa[13],"车门开")!=NULL)?1:0;
	int Carindex=0;
	Carindex=GetCarIndexFromCarID(CarID);
	if (Carindex>=0)
	{
		memcpy(track->DRIVER_NAME,CarData[Carindex].DriverName,12);
		memcpy(track->DRIVER_IDCARDNO,CarData[Carindex].DriverID,18);
	}

	return sizeof(CenterGPSPacket);
}
int Do_JT_Cmd(unsigned short CommandID,char * Phone,unsigned short seq,
	unsigned char * InData,unsigned short DataLen,unsigned char *Packet)
{//生成数据包
	unsigned char p[1024];
	memset(p,0,1024);
	p[0]=CommandID>>8;
	p[1]=CommandID &0x00ff;
	//消息体长度低8位
	p[2]=0;   //不加密
	p[2]=	(DataLen>>8) & 0x3;
	p[3]=	DataLen & 0x00ff;
	//终端手机号
	char ph[100];
	int i;
	sprintf(ph,"%12s",Phone);
	for(i=0;i<strlen(ph);i++)
	{
		if (ph[i]==' ')
			ph[i]='0';


	}


	// if (strlen(Phone)<12) return -1;
	sscanf(ph,"%02X%02X%02X%02X%02X%02X",(unsigned int*)&p[4],(unsigned int*)&p[5],(unsigned int*)&p[6],(unsigned int*)&p[7],(unsigned int*)&p[8],(unsigned int*)&p[9]);
	//消息流水号
	p[10]=seq >>8;
	p[11]=seq & 0x00ff;
	//消息体
	if (DataLen>1024-12)
		return -1;
	memcpy(p+12,InData,DataLen);

	unsigned char crc=0;
	for (i=0;i<DataLen+12;i++)
	{
		crc=crc ^p[i];
	}
	p[DataLen+12]=crc;
	//转议
	int PL=DataLen+13;//数据项长度
	unsigned char D[1024];
	int j=0;
	for (i=0;i<PL;i++)
	{
		if (p[i]==0x7e)
		{
			D[j]=0x7d;
			j++;
			D[j]=0x02;
		}
		else if(p[i]==0x7E)
		{
			D[j]=0x7d;
			j++;
			D[j]=0x01;
		}
		else
		{
			D[j]=p[i];
		}
		j++;
	}
	Packet[0]=0x7e;
	memcpy(Packet+1,D,j);
	Packet[j+1]=0x7e;
	return j+2;
}
int IsCLRChar(char ch)
{
	if ((ch==0x0d) ||(ch ==0x0a))
		return 1;
	else
		return 0;
}





//int LoadIMF(char * imf,char * imd)
//{
//	FILE * fpimd;
//	FILE * fpimf;
//
//	int j,i=0,pacount=0,pointcount=0;
//	char imfline[1000];
//	char imdline[1000];
//	size_t len = 0;
//	size_t read;
//	char* pa[100];
//	fpimf=fopen(imf, "r");
//	fpimd=fopen(imd,"r");
//	int roadno,pointno;
//	Road r[100];
//	RoadXY * pxy;
//	RoadXY *lastpxy;
//	RoadXY ** proadxy;
//
//
//
//
//	int roadindex=0;
//
//
//	if ((fpimf==NULL)||(fpimd==NULL))
//	{
//		perror(strerror(errno));
//		exit(0);
//	}
//	else
//	{
//		printf("%s %s open success!\n",imf,imd);
//	}
//	while (1!= ReadLine(fpimf,imfline))
//	{
//		//	printf("%s\n",imfline);
//		pacount=strSplit(imfline," ",pa);
//		for (i=0;i<pacount;i++)
//		{
//			printf("%s\n",pa[i]);
//		}
//
//		//			Pline Multiple 2
//		//  4
//		//112.622102 35.012869
//		//112.623475 35.012553
//		//112.624012 35.012429
//		//112.625015 35.012198
//		//  2
//		//112.62685 35.011586
//		//112.625015 35.012198
//		//    Pen (1,2,16755455)
//		//Pline 2
//		//112.62685 35.011586
//		//112.627907 35.011257
//		//    Pen (1,2,16755455)
//		if (strcmp(pa[0],"Pline")==0)
//		{
//			ReadLine(fpimd,r[roadindex].RoadName);//读道路名。=
//			pointcount=atoi(pa[1]);
//			proadxy=&r[roadindex].XY;
//
//			if (pointcount>1)//只有一段的路
//			{
//				i=0;
//				r[roadindex].PoitsCounts=pointcount;
//				for(i=0;i<pointcount;i++)//循环读取坐标列表
//				{
//					ReadLine(fpimf,imfline);//读一行坐标
//					pacount=strSplit(imfline," ",pa);
//					if (strcmp(pa[0],"Pen")==0)
//					{
//						continue;//结束了这个地物
//					}
//					pxy=(RoadXY*)malloc(sizeof(RoadXY));
//					pxy->index=i;
//					pxy->Lat=atof(pa[0]);
//					pxy->Lon=atof(pa[1]);
//					pxy->NextRoadXY=NULL;
//					*proadxy=pxy;
//					proadxy=&pxy->NextRoadXY;
//
//				}
//			}
//			roadindex++;
//		}
//
//
//	}
//	fclose(fpimf);
//	fclose(fpimd);
//	return 1;
//
//}
//
//车牌号
//车牌颜色 说明 数字格式字符串。白 0，黄 1，蓝 2，黑 3 长度 固定长度 1 字节
//数据字段 3 字段名称 号牌种类 说明 数字格式字符串值，符合GA 24.7的要求 长度 固定长度 2 字节，长度不够前面补 0
//数据字段 4 字段名称 卫星定位时间 说明 数字字符串，格式为 YYYYMMDDHHMMSS 长度 固定长度 6 字节。
//数据字段 5 字段名称 卫星定位经度 说明 数字格式字符串，格式为 xxx.xxxxxx  长度 固定长度 4 字节
//数据字段 6 字段名称 卫星定位纬度 说明 数字格式字符串，格式为 xx.xxxxxx  长度 固定长度 4 字节
//数据字段 7 字段名称 卫星定位高程 说明 数字格式字符串，为 xxxx，单位：米 长度 固定长度 2 字节，长度不够前面补 0
//数据字段 8 字段名称 卫星定位速度 说明 数字格式字符串，格式为 xxx.xx，单位：千米每小时 长度 固定长度 1 字节，整数长度不够前面补 0
//数据字段 9 字段名称 汽车行驶记录仪速度 说明 数字格式字符串，格式为 xxx.xx，单位：千米每小时 长度 固定长度 1 字节，整数长度不够前面补 0
//数据字段 10 字段名称 卫星定位方向 说明 单位：度，以正北方向为 0度，顺时针方向 长度 固定长度 1 字节，整数长度不够前面补 0
//数据字段 11 字段名称 机动车驾驶证号码 说明 字符串 长度 固定长度 18 字节
//数据字段 12 字段名称 基本车辆状态字 说明 16 个 Hex 码字符串，参考附加信息说明。 长度 固定长度 8 字节


int SendsdjtjTrack(int fd,char * CarNickName,char Color,short CarType,char * GPSDate,double Lon,double Lat,int Speed ,int Heading,unsigned long mile,short AV,short ACC)
{
       if (cb_loginstatu<=0)//未登录cb_loginstatu
       return 0;
	   int ReplayPacketLen;
	   char GPS[100];
		memset(GPS,0,100);
		//车牌,不定长。
		sprintf(GPS,"%s",CarNickName);
		//ReplayPacketLen
		ReplayPacketLen=strlen(CarNickName);//车牌号长度

		GPS[ReplayPacketLen]='|';//分隔符
		ReplayPacketLen++;

		//车牌颜色1字节
		GPS[ReplayPacketLen]=Color;
		ReplayPacketLen++;
		//分隔符
		GPS[ReplayPacketLen]='|';//分隔符

		//号牌种类两字节，不够长补0
		ReplayPacketLen++;
		GPS[ReplayPacketLen]='0';
		ReplayPacketLen++;
		GPS[ReplayPacketLen]='1';
		//分隔符
		ReplayPacketLen++;
		GPS[ReplayPacketLen]='|';//分隔符
		//卫星定位时间十六进制140522112233
		ReplayPacketLen++;
		int TimeP;
		TimeP=ReplayPacketLen;
		sscanf(GPSDate,"%02X%02X%02X%02X%02X%02X",(unsigned int *)&GPS[TimeP],(unsigned int *)&GPS[TimeP+1],(unsigned int *)&GPS[TimeP+2],(unsigned int *)&GPS[TimeP+3],(unsigned int *)&GPS[TimeP+4],(unsigned int *)&GPS[TimeP+5]);
		//分隔(unsigned int *)
		ReplayPacketLen=ReplayPacketLen+6;
		GPS[ReplayPacketLen]='|';//分隔符
		//卫星定位经度
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X11;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X30;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X14;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X16;
		//分隔符
		ReplayPacketLen++;
		GPS[ReplayPacketLen]='|';//分隔符
		//卫星定位纬度
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X02;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X30;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X14;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X16;
			//分隔符
		ReplayPacketLen++;
		GPS[ReplayPacketLen]='|';//分隔符
		//高程2字节
		ReplayPacketLen++;
		GPS[ReplayPacketLen]='0';
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=80;
				//分隔符
		ReplayPacketLen++;
		GPS[ReplayPacketLen]='|';//分隔符
		//GPS速度2字节
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X0;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X6;
				//分隔符
		ReplayPacketLen++;
		GPS[ReplayPacketLen]='|';//分隔符
		//仪表速度2字节
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X0;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X8;
				//分隔符
		ReplayPacketLen++;
		GPS[ReplayPacketLen]='|';//分隔符
		//方向1字节
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X10;
				//分隔符
		ReplayPacketLen++;
		GPS[ReplayPacketLen]='|';//分隔符
		//驾驶证号18字节，字符串
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X10;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X10;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X10;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X10;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X10;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X10;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X10;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X10;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X10;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X10;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X10;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X10;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X10;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X10;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X10;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X10;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X10;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X10;
				//分隔符
		ReplayPacketLen++;
		GPS[ReplayPacketLen]='|';//分隔符
		//状态字8字HEX字节
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X0;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X0;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X0;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X0;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X0;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X0;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X0;
		ReplayPacketLen++;
		GPS[ReplayPacketLen]=0X0;
		ReplayPacketLen++;
		int GPSPacketLen;
		GPSPacketLen=ReplayPacketLen;//定位数据长度
		char ReplayPacket[200];
		ReplayPacketLen=cb_encode_data("U01",ocm_code,GPSPacketLen,GPS,ReplayPacket);//
		char msg[200];
		if (DebugMode)
		{
			memset(msg,0,200);
			int i;
			for(i=0;i<ReplayPacketLen;i++)
			{
				sprintf(msg,"%s %02X", msg,ReplayPacket[i]);
			}
		}
		if (DebugMode)
			syslog(LOG_ERR|LOG_LOCAL0,"%s %d Generate CB CarID:%s-%u  ReplayPacketLen:%d CarFD:%d CommandPacket:%s ReplayPacket:%s \n",
			__FILE__,__LINE__,CarNickName,0,ReplayPacketLen,fd,msg,ReplayPacket);

		int result;


	result= SendDataUDP(  -1,  (char*)ReplayPacket,ReplayPacketLen,CENTERIP,CENTERPORT,JT_GPS_sock) ;
	if (result!= ReplayPacketLen)
		syslog(LOG_ERR|LOG_LOCAL0,"%s %d Send ReplayJT packet:%dbytes-%dbytes send JT_GPS_sock:%d %s\n",__FILE__,__LINE__,ReplayPacketLen,result,JT_GPS_sock,strerror(errno));


	return result;
}
int SendCBTrack(int fd,char * CarNickName,int CarColor,char * GPSDate,double Lon,double Lat,int Speed ,int Heading,unsigned long mile,short AV,short ACC)
{
if (cb_loginstatu<=0)//未登录cb_loginstatu
return 0;

		int ReplayPacketLen=30;
		char GPS[30];
		memset(GPS,0,30);
		//0时间6字节BCD码  YMD hms	 6
		time_t now = time(NULL);
		struct tm *t = localtime(&now);
		char strTime[200];
		sprintf(strTime,"%02d%02d%02d%02d%02d%02d", t->tm_year+1900,t->tm_mon, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
		str2bcd(strTime,(char*)GPS,6);
		//6经度DDDFF.FFF  BCD码	 4

		//53分组名称
	/*	fx=atof(pa[54]);
		fy=atof(pa[55]);*/
		double fx,fy,ffenx,ffeny;
		fx=Lon;
		fy=Lat;

		int ix,iy,i6,i7,i8,i9;
		ix=(int)fx;
		iy=(int)fy;
		ffenx=(fx-ix)*60000.0;
		ffeny=(fy-iy)*60000.0;
		//sprintf(strxy,"%03d%02.3f",ix,ffenx);
		char strxy[30];
		memset(strxy,0,30);
		sprintf(strxy,"%03d%05.0f",ix,ffenx);
		//11302123
		sscanf(strxy,"%02X%02X%02X%02X",&i6,&i7,&i8,&i9);
		GPS[6]=(unsigned char)i6;
		GPS[7]=(unsigned char)i7;
		GPS[8]=(unsigned char)i8;
		GPS[9]=(unsigned char)i9;

		//10纬度0DDFF.FFF BCD码	  4
		ffeny=(fy-iy)*60000.0;
		sprintf(strxy,"%03d%05.0f",iy,ffeny);
		sscanf(strxy,"%02X%02X%02X%02X",&i6,&i7,&i8,&i9);
		GPS[10]=(unsigned char)i6;
		GPS[11]=(unsigned char)i7;
		GPS[12]=(unsigned char)i8;
		GPS[13]=(unsigned char)i9;
		//&GPS[10],&GPS[11],&GPS[12],&GPS[13]
		//14速度km/h 1
		//GPS[14]=atoi(pa[8]);
		GPS[14]=Speed;
		//15方向2dec 1
		GPS[15]=Heading/2;

		//16高度2bytes

		//18里程4字节BCD码 0.1K  13 10 23 09 50 49    11 30 63 24    02 33 12 24   2E  AA  00 00  00 16 02 99  03 20 01 00 00 00 00 00
		sprintf(strxy,"%08u",(unsigned int)mile);
		sscanf(strxy,"%02X%02X%02X%02X",&i6,&i7,&i8,&i9);
		GPS[18]=(unsigned char)i6;
		GPS[19]=(unsigned char)i7;
		GPS[20]=(unsigned char)i8;
		GPS[21]=(unsigned char)i9;
		GPS[18]=0;
		GPS[19]=0;
		GPS[20]=0;
		GPS[21]=0;
		//22状态8字节
		GPS[22]=0x00;
		GPS[23]=0X00;
		GPS[24]=0x00;
		GPS[25]=0X00;
		GPS[26]=0x00;
		GPS[27]=0X00;
		GPS[28]=0x00;
		GPS[29]=0X00;

		GPS[22]=GPS[22] | 0X3;//东经北纬
		if (AV==1)
		GPS[24]=GPS[24] |0X1;


	/*	if (strcmp(exlive_av,"A")==0)
		{
			GPS[24]=GPS[24] |0X1;
		}*/
		if (ACC==0)
			GPS[23]=GPS[23]&0X6F;//第一5位置0
		else
			GPS[23]=GPS[23]|0X20;//第5位置1

		//if (strstr(pa[11],"车匙关")!=NULL)//第二状态字第一5位为ACC
		//	GPS[23]=GPS[23]&0X6F;//第一5位置0
		//else
		//	GPS[23]=GPS[23]|0X20;//第5位置1
		//[RETR|粤R43740|002909667436|2013-09-27 20:26:28|112.9912|23.7001|精确|1|0|0|0|当日里程605KM,车匙开空调开油箱电压0|TCP:601ip:120.197.206.200总里程1331公里,报文类型:200,轨迹号:37043|广东省清远市清新县回澜镇 ,江佳贤东南30米|0|48|1|0|0|0|0|0|未知司机|00000000|2|12|13|14|15|16|17|18|19|20|21|22|23|24|25|26|27|28|29|30|31|32|33|34|35|36|37|38|39|江佳贤|112.9860|23.7027|DB44]
	   char cb_data[1000];
		memset(cb_data,0,1000);
		sprintf(cb_data,"%s",CarNickName);
		int headlen=0;
		headlen=strlen(cb_data);
		cb_data[headlen]='|';//数据分隔符
		//CarNickName,CarColor
		cb_data[headlen+1]=CarColor;//车牌色
		cb_data[headlen+2]='|';//数据分隔符
		memcpy(cb_data+headlen+3,GPS,30);//30节节的定位数据
		char ReplayPacket[200];
		ReplayPacketLen=cb_encode_data("U01",ocm_code,headlen+30+3,cb_data,ReplayPacket);//


		char msg[200];
		if (DebugMode)
		{
			memset(msg,0,200);
			int i;
			for(i=0;i<ReplayPacketLen;i++)
			{
				sprintf(msg,"%s %02X", msg,ReplayPacket[i]);
			}
		}
		if (DebugMode)
			syslog(LOG_ERR|LOG_LOCAL0,"%s %d Generate CB CarID:%s-%u  ReplayPacketLen:%d CarFD:%d CommandPacket:%s ReplayPacket:%s \n",
			__FILE__,__LINE__,CarNickName,0,ReplayPacketLen,fd,msg,ReplayPacket);

		int result;


	result= SendDataUDP(  -1,  (char*)ReplayPacket,ReplayPacketLen,CENTERIP,CENTERPORT,JT_GPS_sock) ;
	if (result!= ReplayPacketLen)
		syslog(LOG_ERR|LOG_LOCAL0,"%s %d Send ReplayJT packet:%dbytes-%dbytes send JT_GPS_sock:%d %s\n",__FILE__,__LINE__,ReplayPacketLen,result,JT_GPS_sock,strerror(errno));

	return result;

}

int JT809_CGJ_GPS(char * buff)
{
//[RETR|粤R43740|002909667436|2013-09-27 20:26:28|112.9912|23.7001|精确|1|0|0|0|
//当日里程605KM,车匙开空调开油箱电压0|TCP:601ip:120.197.206.200总里程1331公里,报文类型:200,轨迹号:37043
//|广东省清远市清新县回澜镇 ,江佳贤东南30米|0|48|1|0|0|0|0|0|未知司机|00000000|2|12|13|14|15|16|17|18|19|20|21|22|23|24|25|26|27|28|29|30|31|32|33|34|35|36|37|38|39|江佳贤|54-112.9860|23.7027|DB44]
//  0      1		2				3					4		5	6	7

	int ind=0;
	char *ptr=NULL;
	char* pa[100];
	char * p=NULL;
	p=(char*)buff;
	while((pa[ind]=strtok_r(p,"[|]",&ptr))!=NULL)
	{
		ind++;
		p=NULL;
		if (ind>89)
			break;
	}

	if(( ind<55 )||(strcmp(pa[0],"RETR")!=0))
	{
		if (DebugMode)
			syslog(LOG_ERR|LOG_LOCAL0,"%s %d  Packet Erro string:%s \n",
			__FILE__,__LINE__,buff);

		return -1;
	}

	unsigned char gnss[36];//保存卫星定位数据
	char packdata[1000];//
	char trackdata[1000];
	//int JT809_GPS_packet(char * gpsdate,double lon,double lat,int speed,int speed2,int mile, int heading,int height,char* statu,char *alarm,char * packetdata)
	char gpsdate[100];
	strcpy(gpsdate,pa[3]);//2013-09-27 20:26:28 p[3]
	double lon;
	lon=atof(pa[54]);
	double lat;
	lat=atof(pa[55]);
	int speed;

	int heading;

	int len,gpslen,packetlen;
	int Carindex=0;
	Carindex=GetCarIndexFromCarID(pa[2]);
	char carname[100];
	sprintf(carname,"%s",CarData[Carindex].CarName);
	int color;
	color=CarData[Carindex].Color;
	speed=atoi(pa[8]);
	heading=atoi(pa[7]);
	unsigned int  mile=0;
	mile=atoi(pa[52])/1000;
	len=JT809_GPS_packet(gpsdate,lon,lat,speed,speed,mile,heading,0,"00008003","40000000",gnss);
	gpslen=JT809_up_gps_track(carname,color,UP_EXG_MSG_REAL_LOCATION,gnss,(unsigned int)len, trackdata);
	packetlen=JT809_encode_data(UP_EXG_MSG,atoi(ocm_code), trackdata, gpslen,packdata);
	if ((JT_GPS_sock>0) && (cb_loginstatu>0))//登录成功才发送定位
		speed=cb_lpt_sendto(JT_GPS_sock,packdata,packetlen);
		unsigned char msg[5000];
		int i;

		if ((atoi(pa[8]))>0)
		{
			memset(msg,0,5000);
			for(i=0;i<packetlen;i++)
			{
				sprintf(msg,"%s-%02X",msg,(unsigned char )packdata[i]);

			}
			syslog(LOG_ERR|LOG_LOCAL0,"%s %d   %s %s speed：%d:%s mile:%s %d %dbytes\n",
			__FILE__,__LINE__,carname,pa[8],atoi(pa[8]),msg,pa[52],mile,speed);

		}

	//exlive.c 1330   62 speed￡o62:-5B-00-00-00-5A-02-00-00-00-08-12-00-56-05-D5-1E-01-00-00-00-5A-02-5A-01-5C-5E-01-D4-C1-41-39-30-37-37-00-00-00-00-00-00-00-00-00-00-00-00-00-00-02-12-02-00-00-00-24-00-11-0C-07-DE-0F-30-22-06-BF-18-18-01-56-34-34-00-3E-00-3E-00-00-00-00-00-AF-00-00-00-00-80-03-40-00-00-00-7A-20-5D

	return speed;
}


int weixin_CGJ_GPS(char * buff)
{
//[RETR|粤R43740|002909667436|2013-09-27 20:26:28|112.9912|23.7001|精确|1|0|0|0|
//当日里程605KM,车匙开空调开油箱电压0|TCP:601ip:120.197.206.200总里程1331公里,报文类型:200,轨迹号:37043
//|广东省清远市清新县回澜镇 ,江佳贤东南30米|0|48|1|0|0|0|0|0|未知司机|00000000|2|12|13|14|15|16|17|18|19|20|21|22|23|24|25|26|27|28|29|30|31|32|33|34|35|36|37|38|39|江佳贤|54-112.9860|23.7027|DB44]
//  0      1		2				3					4		5	6	7

	int ind=0;
	char *ptr=NULL;
	char* pa[100];
	char * p=NULL;
	p=(char*)buff;
	while((pa[ind]=strtok_r(p,"[|]",&ptr))!=NULL)
	{
		ind++;
		p=NULL;
		if (ind>89)
			break;
	}

	if(( ind<55 )||(strcmp(pa[0],"RETR")!=0))
	{
		if (DebugMode)
			syslog(LOG_ERR|LOG_LOCAL0,"%s %d  Packet Erro string:%s \n",
			__FILE__,__LINE__,buff);

		return -1;
	}

	char* api="/hqweixin/hqwx.php";
	char * para="mode=gps";
	char msg[1000];
	sprintf(msg,"CarID=%s&Lon=%s&Lat=%s&gpsdate=%s&speed=%s&heading=%s",pa[2],pa[54],pa[55],pa[3],pa[8],pa[7]);
	int r;
	syslog(LOG_ERR|LOG_LOCAL0,"%s %d  weixin %s-%s-%s-%s\n",__FILE__,__LINE__,CENTERIP,api,para,msg);
	r=PostData(CENTERIP,api,para,msg);

return r;


}
//[RETR|粤AN6802|002916884011|2014-12-05 18:15:11|113.2095|21.9696|精确|1|0|0|0|
//当日238KM,车匙关油箱电压0|TCP:1241ip:172.24.19.13总里程53144公里,报文类型:200,轨迹号:6|广东省珠海市珠海西区Ｓ３６６,.|0|0|1|0|0|0|0
//|0|未知司机|00000000|2|12|13|14|15|16|17|18|19|20|21|22|23|24|25|26|27|28|29|30|31|32|33|34|35|36|37|38|53144800|广州市壹弘运输有限公司|113.2042|21.9727|DB44]
//	  ProtocloType=0 :exlive 1:JT808
int ChangCGJ2JT808(char *Data,int DataLen,int fd,int ProtocloType)
{
	//02 00		命令ID
	//00 26
	//00 29 02 42 12 14	   车机ID
	//D1 CD			   序号
	//消息体
	//40 00 00 00报警标志	0
	//00 00	 00 82 状态	  4
	//01 61	 B5 AE 纬度	 8
	//06 B9	B1 40 经度	12
	//02 25	 高度	  16
	//00 00 速度	 18
	//00 01 方向	  20
	//13 07 13 16 28 34	  时间	 22

	//01 04 00 02 9B 68 03 02 00 00 7E
	packetSQE++;//消息序号加1
	int pr=0;
	if (ProtocloType==6)
	{
	pr=JT809_CGJ_GPS(Data);
     return pr;
	}

	if (ProtocloType==7)
	{

	 pr=weixin_CGJ_GPS(Data);
	 return pr;
	}
	unsigned char st1,st2,st3,st4,st5,st6,st7,st8,est1,est2,est3,est4;
	unsigned char MsgBody[100];
	char sCarID[20];
	char cb_data[1000];//长宝数据段。
	memset(MsgBody,0,100);
	char msg[2048];
	memset(msg,0,2048);
	char buff[2048];
	memcpy(buff,Data,DataLen);
	int ind=0;
	char *ptr=NULL;
	char* pa[100];
	char * p=NULL;
	p=(char*)buff;
	while((pa[ind]=strtok_r(p,"[|]",&ptr))!=NULL)
	{
		ind++;
		p=NULL;
		if (ind>89)
			break;
	}

	if(( ind<55 )||(strcmp(pa[0],"RETR")!=0))
	{
		if (DebugMode)
			syslog(LOG_ERR|LOG_LOCAL0,"%s %d  Packet Erro string:%s \n",
			__FILE__,__LINE__,Data);

		return -1;
	}


	char sCarID12[20];

	memset(sCarID12,0,20);//临时的

	strcpy(sCarID12,pa[2]);

	int i=0,j=0,CarIDLen=0;
	CarIDLen=strlen(sCarID12);//车机号长度
	if (CarIDLen>10)//车机号大于10位
	{
		for (i=CarIDLen-10;i<CarIDLen;i++)
		{
			sCarID[j]=sCarID12[i];
			j++;

		}
	}
	else//车机号不足10位
	{
		sprintf(sCarID,"%s",sCarID12);
	}
	unsigned long a = strtoull(sCarID, NULL, 10);
	unsigned int iCarID;
	iCarID=(a & 0x0000FFFFFFFF);
	//	sprintf(sCarID,"%010u",iCarID);

	if (DebugMode)
		syslog(LOG_ERR|LOG_LOCAL0,"%s %d  sCarID12:%s->sCarID:%s -> %u-%u  \n",__FILE__,__LINE__,sCarID12,sCarID,a,iCarID);

	//[RETR|粤A17264|2000143054|2011-01-22 10:44:32|114.9268|21.8207|无效|0|0|0|0|车匙关|正常|.|1|1|0|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|0|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|114.9224|21.8235|jh10][RETR|粤A169PG|2000146023|2011-01-22 10:44:32|113.3205|23.1382|精确|0|0|0|0|车匙关|正常|广东省广州市天河区体育西路,|1|1|0|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|0|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|113.3151|23.1408|jh10]
	//
	//定位有效性
	//track->VALID=(strstr(pa[6],"精确")!=NULL)?1:0;
	//车机机ID

	//消息体
	//40 00 00 00报警标志	0
	//00 00	 00 82 状态	  4	  23180718
	//01 61	 B5 AE 纬度	 8
	//06 B9	B1 40 经度	12	  112832832
	//02 25	 高度	  16
	//00 00 速度	 18
	//00 01 方向	  20
	//13 07 13 16 28 34	  时间	 22
	st1=(unsigned char)atoi(pa[14]);
	st2=(unsigned char)atoi(pa[15]);
	st3=(unsigned char)atoi(pa[16]);
	st4=(unsigned char)atoi(pa[17]);
	st5=(unsigned char)atoi(pa[18]);
	st6=(unsigned char)atoi(pa[19]);
	st7=(unsigned char)atoi(pa[20]);
	st8=(unsigned char)atoi(pa[21]);
	est1=0xff;est2=0xff;est3=0xff;est4=0xff;

	//acc
	// if	 ( TestBit(st2,5)>0)
	if (strstr(pa[11],"车匙关")!=NULL)
		est3= est3 & 0xFB;	//   状态实3第二位置0
	//电源断  2，4
	// if	 ( TestBit(st1,6)>0)
	if (strstr(pa[11],"电源")!=NULL)
		est2= est2 & 0xEF;


	//超速	  3，7
	//if	 ( TestBit(st1,4)>0)
	if (strstr(pa[11],"超速")!=NULL)
		est3= est3 & 0x7F;
	//	 if	 ( TestBit(st2,6)>0)
	if ((strstr(pa[11],"空调开")!=NULL) ||(strstr(pa[11],"冷气开")!=NULL))
		//空调	  1，5
		est1=est1 & 0xDF;

	if ((strstr(pa[11],"按钮")!=NULL) ||(strstr(pa[11],"紧急")!=NULL))
		// if	 ( TestBit(st1,2)>0)
		//按钮  4，1
		est4=est4 & 0xFD;
	int	 ReplayPacketLen,MsgBodyLen,ReplayCommandID;
	unsigned char   ReplayPacket[200],Phone[13];
	memset(ReplayPacket,0,200);
	MsgBody[0]=0x40;
	MsgBody[7]=0x82;
	struct tm UTC;

	unsigned int year,mon,day,hour,minute,second;
	struct tm * tmptr;
	char strTime[40],strUTCDay[20],strUTCTime[20],XDDD[20],YDD[20];
	int ddd;
	float fff;
	char exlive_av[10];
	char Sexlive [200];
	//char s[4000];
	int exlive_heading;
	int ret;
	unsigned short int   speed;
	unsigned short int   heading;
	//int i=0;
	unsigned int x,y;
	unsigned long mile;
	int result;
	unsigned  char GPS[30];//基本卫星定位数据包
	int ix,iy;
	float fx,fy,ffenx,ffeny;
	char strxy[20];
	strncpy((char *)Phone,pa[2],12);
	sscanf(pa[3],"%04d-%02d-%02d %02d:%02d:%02d",&year,&mon,&day,&hour,&minute,&second);
	UTC.tm_year=year-2000;
	UTC.tm_mon=mon-1;
	UTC.tm_mday=day;
	UTC.tm_hour=hour;
	UTC.tm_min=minute;
	UTC.tm_sec=second;
	time_t t;
	t=mktime(&UTC);
	t=t-3600*8;

	tmptr=localtime(&t);
	memcpy(&UTC,tmptr,sizeof(struct tm));



	sprintf(strUTCDay,"%02d%02d%02d",UTC.tm_mday,UTC.tm_mon+1,UTC.tm_year);
	sprintf(strUTCTime,	"%02d%02d%02d",UTC.tm_hour,UTC.tm_min,UTC.tm_sec);

	ddd=atof(pa[54]);
	fff=(atof(pa[54])-ddd)*60.0;
	sprintf(XDDD,"%03d%07.4f",ddd,fff);
	ddd=atof(pa[55]);
	fff=(atof(pa[55])-ddd)*60.0;
	sprintf(YDD,"%02d%07.4f",ddd,fff);

	if (strstr(pa[6],"精确")!=NULL)
		sprintf(   exlive_av,"A");
	else
		sprintf(   exlive_av,"V");
	float exlive_speed;
	exlive_speed=atof(pa[8])/1.852;


	exlive_heading=atoi(pa[7]);

	int carindex=-1;
	int headlen=0;
	unsigned int i6,i7,i8,i9;
	char CarNickName[20];
	int CarColor=2;
	switch(ProtocloType)   //	根据要求传不同的协义格式数据出去。
	{
	case 6://珠海裕丰钢铁
		//[RETR|粤R43740|002909667436|2013-09-27 20:26:28|112.9912|23.7001|精确|1|0|0|0|当日里程605KM,车匙开空调开油箱电压0|TCP:601ip:120.197.206.200总里程1331公里,报文类型:200,轨迹号:37043|广东省清远市清新县回澜镇 ,江佳贤东南30米|0|48|1|0|0|0|0|0|未知司机|00000000|2|12|13|14|15|16|17|18|19|20|21|22|23|24|25|26|27|28|29|30|31|32|33|34|35|36|37|38|39|江佳贤|112.9860|23.7027|DB44]
		//int JT809_GPS_packet(char * gpsdate,double lon,double lat,int speed,int speed2,int mile, int heading,int height,char* statu,char *alarm,char * packetdata)

		break;
	case 5://顺德交通局，不用实时的定位数据
		break;
	case 3://长宝
		if (cb_loginstatu<=0)//未登录cb_loginstatu
			return 0;
		carindex=-1;
		for(i=0;i<MAXCARNUMBER;i++)
		{
			if (strcmp(CarData[i].CarID,pa[2])==0)
			{
				CarColor=CarData[i].Color;
				strcpy(CarNickName,CarData[i].CarName);
				if (DebugMode)
				syslog(LOG_ERR|LOG_LOCAL0,"%s %d CAR_ID:%s in CarData,CarNickName:%s,Color %d! \n",
				__FILE__,__LINE__,pa[2],CarNickName,CarColor);
				carindex=i;
				break;
			}
		}
		if (carindex<0)
		{
			syslog(LOG_ERR|LOG_LOCAL0,"%s %d Can NOT FIND THIS CAR_ID:%s in CarData,exit function! \n",
				__FILE__,__LINE__,pa[2]);
			return -1;
		}

		if (iCarID>=0xFFFFFFFF)
			iCarID=iCarID-(int)((iCarID/1000000000.0)*1000000000);

		//iCarID=a;
		ReplayPacketLen=30;
		memset(GPS,0,30);
		//0时间6字节BCD码  YMD hms	 6
		year=year-2000;
		sprintf(strTime,"%02d%02d%02d%02d%02d%02d",year,mon,day,hour,minute,second);
		str2bcd(strTime,(char*)GPS,6);

		//6经度DDDFF.FFF  BCD码	 4


		mile=strtoul(pa[52],NULL,10);//里程：米
		mile=mile/100;//DB44里以0.1KM算

		//53分组名称
		fx=atof(pa[54]);
		fy=atof(pa[55]);

		ix=(int)fx;
		iy=(int)fy;
		ffenx=(fx-ix)*60000.0;
		ffeny=(fy-iy)*60000.0;
		//sprintf(strxy,"%03d%02.3f",ix,ffenx);
		sprintf(strxy,"%03d%05.0f",ix,ffenx);
		//11302123
		sscanf(strxy,"%02X%02X%02X%02X",&i6,&i7,&i8,&i9);
		GPS[6]=(unsigned char)i6;
		GPS[7]=(unsigned char)i7;
		GPS[8]=(unsigned char)i8;
		GPS[9]=(unsigned char)i9;

		//10纬度0DDFF.FFF BCD码	  4
		ffeny=(fy-iy)*60000.0;
		sprintf(strxy,"%03d%05.0f",iy,ffeny);
		sscanf(strxy,"%02X%02X%02X%02X",&i6,&i7,&i8,&i9);
		GPS[10]=(unsigned char)i6;
		GPS[11]=(unsigned char)i7;
		GPS[12]=(unsigned char)i8;
		GPS[13]=(unsigned char)i9;
		//&GPS[10],&GPS[11],&GPS[12],&GPS[13]
		//14速度km/h 1
		GPS[14]=atoi(pa[8]);
		//15方向2dec 1
		GPS[15]=atoi(pa[7])/2;

		//16高度2bytes

		//18里程4字节BCD码 0.1K  13 10 23 09 50 49    11 30 63 24    02 33 12 24   2E  AA  00 00  00 16 02 99  03 20 01 00 00 00 00 00
		sprintf(strxy,"%08u",mile);
		sscanf(strxy,"%02X%02X%02X%02X",&i6,&i7,&i8,&i9);
		GPS[18]=(unsigned char)i6;
		GPS[19]=(unsigned char)i7;
		GPS[20]=(unsigned char)i8;
		GPS[21]=(unsigned char)i9;
		GPS[18]=0;
		GPS[19]=0;
		GPS[20]=0;
		GPS[21]=0;
		//22状态8字节
		GPS[22]=0x00;
		GPS[23]=0X00;
		GPS[24]=0x00;
		GPS[25]=0X00;
		GPS[26]=0x00;
		GPS[27]=0X00;
		GPS[28]=0x00;
		GPS[29]=0X00;

		GPS[22]=GPS[22] | 0X3;//东经北纬
		if (strcmp(exlive_av,"A")==0)
		{
			GPS[24]=GPS[24] |0X1;
		}

		if (strstr(pa[11],"车匙关")!=NULL)//第二状态字第一5位为ACC
			GPS[23]=GPS[23]&0X6F;//第一5位置0
		else
			GPS[23]=GPS[23]|0X20;//第5位置1
		//[RETR|粤R43740|002909667436|2013-09-27 20:26:28|112.9912|23.7001|精确|1|0|0|0|当日里程605KM,车匙开空调开油箱电压0|TCP:601ip:120.197.206.200总里程1331公里,报文类型:200,轨迹号:37043|广东省清远市清新县回澜镇 ,江佳贤东南30米|0|48|1|0|0|0|0|0|未知司机|00000000|2|12|13|14|15|16|17|18|19|20|21|22|23|24|25|26|27|28|29|30|31|32|33|34|35|36|37|38|39|江佳贤|112.9860|23.7027|DB44]
		memset(cb_data,0,1000);
		sprintf(cb_data,"%s",CarNickName);
		headlen=strlen(cb_data);
		cb_data[headlen]='|';//数据分隔符
		//CarNickName,CarColor
		cb_data[headlen+1]=CarColor;//车牌色
		cb_data[headlen+2]='|';//数据分隔符
		memcpy(cb_data+headlen+3,GPS,30);//30节节的定位数据

		ReplayPacketLen=cb_encode_data("U01",ocm_code,headlen+30+3,cb_data,ReplayPacket);//



		if (DebugMode)
		{
			memset(msg,0,200);
			for(i=0;i<ReplayPacketLen;i++)
			{
				sprintf(msg,"%s %02X", msg,ReplayPacket[i]);
			}
		}
		if (DebugMode)
			syslog(LOG_ERR|LOG_LOCAL0,"%s %d Generate CB CarID:%s-%u  ReplayPacketLen:%d CarFD:%d CommandPacket:%s ReplayPacket:%s \n",
			__FILE__,__LINE__,Phone,(unsigned int)iCarID,ReplayPacketLen,fd,msg,ReplayPacket);




	result= SendDataUDP(  -1,  (char*)ReplayPacket,ReplayPacketLen,CENTERIP,CENTERPORT,JT_GPS_sock) ;
	if (result!= ReplayPacketLen)
		syslog(LOG_ERR|LOG_LOCAL0,"%s %d Send ReplayJT packet:%dbytes-%dbytes send JT_GPS_sock:%d %s\n",__FILE__,__LINE__,ReplayPacketLen,result,JT_GPS_sock,strerror(errno));

	break;
		case 0://天琴


			//*EX,9090422215,MOVE,053651,A,2305.7672,N,11116.8198,E,0.00,000,180510,FBFFFFFF#
			memset(Sexlive,0,200);

			sprintf(Sexlive,"*HQ,%s,V1,%s,%s,%s,N,%s,E,%.2f,%d,%s,%02X%02X%02X%02X#",
				sCarID,strUTCTime,exlive_av,YDD,XDDD,exlive_speed,exlive_heading,strUTCDay,est1,est2,est3,est4);

			ret= SendDataUDP(  UDP_Send_Sock,  Sexlive,strlen(Sexlive),	CENTERIP,CENTERPORT,JT_GPS_sock) ;
			if (DebugMode)
				syslog(LOG_ERR|LOG_LOCAL0,"%s %d  send string:%s \n",
				__FILE__,__LINE__,Sexlive);
			if(ret<=0)
				syslog(LOG_ERR|LOG_LOCAL0,"%s %d  send string:%s error %d bytes %s \n",
				__FILE__,__LINE__,Sexlive,ret,strerror(errno));
			break;
		case 2:	//省标数据
			// 13 08 28 17 07 36 11 31 02 00 02 15 08  00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00
			if (iCarID>=0xFFFFFFFF)
				iCarID=iCarID-(int)((iCarID/1000000000.0)*1000000000);

			//iCarID=a;
			ReplayPacketLen=30;
			memset(GPS,0,30);
			//0时间6字节BCD码  YMD hms	 6
			year=year-2000;
			sprintf(strTime,"%02d%02d%02d%02d%02d%02d",year,mon,day,hour,minute,second);
			str2bcd(strTime,(char*)GPS,6);

			//6经度DDDFF.FFF  BCD码	 4

			//unsigned long mile;
			mile=strtoul(pa[52],NULL,10);//里程：米
			mile=mile/100;//DB44里以0.1KM算

			//53分组名称
			fx=atof(pa[54]);
			fy=atof(pa[55]);
		//	unsigned int i6,i7,i8,i9;
			ix=(int)fx;
			iy=(int)fy;
			ffenx=(fx-ix)*60000.0;
			ffeny=(fy-iy)*60000.0;
			//sprintf(strxy,"%03d%02.3f",ix,ffenx);
			sprintf(strxy,"%03d%05.0f",ix,ffenx);
			//11302123
			sscanf(strxy,"%02X%02X%02X%02X",&i6,&i7,&i8,&i9);
			GPS[6]=(unsigned char)i6;
			GPS[7]=(unsigned char)i7;
			GPS[8]=(unsigned char)i8;
			GPS[9]=(unsigned char)i9;

			//10纬度0DDFF.FFF BCD码	  4
			ffeny=(fy-iy)*60000.0;
			sprintf(strxy,"%03d%05.0f",iy,ffeny);
			sscanf(strxy,"%02X%02X%02X%02X",&i6,&i7,&i8,&i9);
			GPS[10]=(unsigned char)i6;
			GPS[11]=(unsigned char)i7;
			GPS[12]=(unsigned char)i8;
			GPS[13]=(unsigned char)i9;
			//&GPS[10],&GPS[11],&GPS[12],&GPS[13]
			//14速度km/h 1
			GPS[14]=atoi(pa[8]);
			//15方向2dec 1
			GPS[15]=atoi(pa[7])/2;

			//16高度2bytes

			//18里程4字节BCD码 0.1K  13 10 23 09 50 49    11 30 63 24    02 33 12 24   2E  AA  00 00  00 16 02 99  03 20 01 00 00 00 00 00
			sprintf(strxy,"%08u",mile);
			sscanf(strxy,"%02X%02X%02X%02X",&i6,&i7,&i8,&i9);
			GPS[18]=(unsigned char)i6;
			GPS[19]=(unsigned char)i7;
			GPS[20]=(unsigned char)i8;
			GPS[21]=(unsigned char)i9;
			GPS[18]=0;
			GPS[19]=0;
			GPS[20]=0;
			GPS[21]=0;
			//22状态8字节
			GPS[22]=0x00;
			GPS[23]=0X00;
			GPS[24]=0x00;
			GPS[25]=0X00;
			GPS[26]=0x00;
			GPS[27]=0X00;
			GPS[28]=0x00;
			GPS[29]=0X00;

			GPS[22]=GPS[22] | 0X3;//东经北纬
			if (strcmp(exlive_av,"A")==0)
			{
				GPS[24]=GPS[24] |0X1;
			}

			if (strstr(pa[11],"车匙关")!=NULL)//第二状态字第一5位为ACC
				GPS[23]=GPS[23]&0X6F;//第一5位置0
			else
				GPS[23]=GPS[23]|0X20;//第5位置1
			//[RETR|粤R43740|002909667436|2013-09-27 20:26:28|112.9912|23.7001|精确|1|0|0|0|当日里程605KM,车匙开空调开油箱电压0|TCP:601ip:120.197.206.200总里程1331公里,报文类型:200,轨迹号:37043|广东省清远市清新县回澜镇 ,江佳贤东南30米|0|48|1|0|0|0|0|0|未知司机|00000000|2|12|13|14|15|16|17|18|19|20|21|22|23|24|25|26|27|28|29|30|31|32|33|34|35|36|37|38|39|江佳贤|112.9860|23.7027|DB44]
			if (sCarID[0]>'4')
			{
				sCarID[0]='1';

			}

			if (DebugMode)
			{
				memset(msg,0,200);
				for(i=0;i<ReplayPacketLen;i++)
				{
					sprintf(msg,"%s %02X", msg,GPS[i]);
				}
				if (DebugMode)
					syslog(LOG_ERR|LOG_LOCAL0,"%s %d Generate DB44 CarID:%s-%u  ReplayPacketLen:%d CarFD:%d CommandPacket:%s \n",
					__FILE__,__LINE__,Phone,(unsigned int)iCarID,ReplayPacketLen,fd,msg);
			}
			unsigned char DBData[1000];
			int dblen=0;
			if	 (packetSQE %10==0)	   //每10条鉴权一次E0
				dblen=DoDBPacket((unsigned int)iCarID,0x56060016,packetSQE,5642,key,0xE0,GPS,0,DBData,'U');
			else
				dblen=DoDBPacket((unsigned int)iCarID,0x56060016,packetSQE,5642,key,00,GPS,ReplayPacketLen,DBData,'U');


			result= SendDataUDP(  UDP_Send_Sock,  (char*)DBData,dblen,CENTERIP,CENTERPORT,-1) ;
			/*
			memset(s,0,2000);
			PrintArray((const char*)DBData,dblen,s);
			syslog(LOG_ERR|LOG_LOCAL0,"%s %d  %s <%s> %d bytes\n",
			__FILE__,__LINE__,sCarID,s,dblen);*/

			if(result<=0)
				syslog(LOG_ERR|LOG_LOCAL0,"%s %d  send DB44 error %d bytes %s \n",
				__FILE__,__LINE__,dblen,strerror(errno));

			//if	 (packetSQE %10==0)	   //每10条鉴权一次E0
			//		 {
			//		 sprintf((char*)MsgBody,"123456");
			//	  ReplayPacketLen=Do_JT_Cmd(0x0102,(char*)sCarID,packetSQE,MsgBody,6,ReplayPacket); //生成数据到  ReplayPacket，长度为ReplayPacketLen
			//	  result= SendDataUDP(UDP_Send_Sock,  (char*)ReplayPacket,ReplayPacketLen,	CENTERIP,CENTERPORT,JT_GPS_sock) ;
			//	  }

			return result;
			break;
		case 1:	   //部标数据

			// 7E
			//02 00
			//00 1C
			//36 98 00 01 32 90
			//00 00
			//《40 00 00 00, 00 00 00 82, 01 5F 3F EC, 06 BE CF AC, 00 00, 00 02 ,00 02 ,13 08 05 18 25 27, 83 7E

			year=year-2000;
			sprintf(strTime,"%02d%02d%02d%02d%02d%02d",year,mon,day,hour,minute,second);
			str2bcd(strTime,(char*)(MsgBody+22),6);



			x=1000000*atof(pa[54]);
			y=1000000*atof(pa[55]);
			//经度12
			MsgBody[12]=x >>24;
			MsgBody[13]=(x & 0x00ff0000)>>16;
			MsgBody[14]=(x & 0x0000ff00)>>8;
			MsgBody[15]=x & 0x000000ff;
			//纬度8
			MsgBody[8]=y>>24;
			MsgBody[9]=(y & 0x00ff0000)>>16;
			MsgBody[10]=(y & 0x0000ff00)>>8;
			MsgBody[11]=y & 0x000000ff;
			//高度16 2
			MsgBody[16]= speed>>8;
			MsgBody[17]= speed & 0x00ff;

			//速度
			speed=atoi(pa[8]);
			MsgBody[18]= speed>>8;
			MsgBody[19]= speed & 0x00ff;
			//heading	20

			heading=atoi(pa[7]);
			MsgBody[20]= heading>>8;
			MsgBody[21]= heading & 0x00ff;
			//消息体长度27;
			MsgBodyLen=28;
			ReplayCommandID=0x200;
			//封包

			/*memset(msg,0,200);
			for(i=0;i<  MsgBodyLen;i++)
			{
			sprintf(msg,"%s %02X", msg,MsgBody[i]);
			}

			syslog(LOG_ERR|LOG_LOCAL0,"%s %d  CarID:%s  MsgBodyLen:%d CarFD:%d MsgBodyLen:%s \n",
			__FILE__,__LINE__,Phone,MsgBodyLen,fd,msg);*/

			if	(MsgBodyLen>=0)
			{
				ReplayPacketLen=Do_JT_Cmd((unsigned short)ReplayCommandID,(char*)sCarID,packetSQE,MsgBody,MsgBodyLen,ReplayPacket); //生成数据到  ReplayPacket，长度为ReplayPacketLen


				if  (DebugMode)
				{
					memset(msg,0,200);
					for(i=0;i< ReplayPacketLen;i++)
					{
						sprintf(msg,"%s %02X", msg,ReplayPacket[i]);
					}

					syslog(LOG_ERR|LOG_LOCAL0,"%s %d Reply JT  CarID:%s  ReplayPacketLen:%d CarFD:%d CommandPacket:%s \n",
						__FILE__,__LINE__,sCarID,ReplayPacketLen,fd,msg);
				}

				//	}

				//if(( ReplayPacketLen>15) && (fd>0))	//回复车机
				//	{
				//	result=send(fd,ReplayPacket,ReplayPacketLen,0);
				//	}
				//else if(( ReplayPacketLen>15) && (fd<=0))	//回复车机
				//	{
				//result=sendto(udp_fd,ReplayPacket,ReplayPacketLen,0,(struct sockaddr *)c_addr,sizeof(struct sockaddr));
				//	//result=sendto(fd,ReplayPacket,ReplayPacketLen,0);
				//	}
				result= SendDataUDP(  UDP_Send_Sock,  (char*)ReplayPacket,ReplayPacketLen,	CENTERIP,CENTERPORT,JT_GPS_sock) ;


				if	 (packetSQE %10==0)	   //每10条鉴权一次
				{
					sprintf((char*)MsgBody,"123456");
					ReplayPacketLen=Do_JT_Cmd(0x0102,(char*)sCarID,packetSQE,MsgBody,6,ReplayPacket); //生成数据到  ReplayPacket，长度为ReplayPacketLen
					result= SendDataUDP(UDP_Send_Sock,  (char*)ReplayPacket,ReplayPacketLen,	CENTERIP,CENTERPORT,JT_GPS_sock) ;
				}


				if (result!= ReplayPacketLen)
					syslog(LOG_ERR|LOG_LOCAL0,"%s %d Send ReplayJT packet:%dbytes-%dbytes send fd:%d %s\n",__FILE__,__LINE__,ReplayPacketLen,result,fd,strerror(errno));

			}
			}

return sizeof(ReplayPacketLen);
}
	/*
	00-01-30-31-33-31-38-39-33-34-30-35-02-00-43
	20-10-01-15-12-03-23
	02-30-81-80
	11-23-26-01
	00-00
	02-20
	00-00//高度
	00-00-00-00//里程
	00-00-00-00-00-00-00-00-00-00-00-00//司机姓名
	00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00-00身份证号
	01//定位有效
	03-20-00-00-00-00-00-00//状态位
	00-00-00
	*/



	int Chang2JT808Packet(char *Data,int DataLen,int fd,CenterGPSPacket * track,char * CarID)
	{
		//[RETR|AF2422|002967677751|2013-07-13 09:54:44|113.4543|23.1432|精确|351|0|1|0|当日里程191KM,车匙关车辆电路断主电源断油箱电压0|TCP:1836ip:172.24.31.24总里程3355公里,报文类型:200,轨迹号:4315|广东省广州市黄埔区 ,.|64|16|1|0|0|0|0|0|未知司机|00000000|2|12|13|14|15|16|17|18|19|20|21|22|23|24|25|26|27|28|29|30|31|32|33|34|35|36|37|38|39|40|113.4486|23.1456|DB44]
		//0      1      2             3                   4        5      6   7
		//[RETR|粤A17264|2000143054|2011-01-22 10:44:32|114.9268|21.8207|无效|0|0|0|0|车匙关|正常|.|1|1|0|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|0|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|114.9224|21.8235|jh10][RETR|粤A169PG|2000146023|2011-01-22 10:44:32|113.3205|23.1382|精确|0|0|0|0|车匙关|正常|广东省广州市天河区体育西路,|1|1|0|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|0|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|1|113.3151|23.1408|jh10]

		char  * pa[50];
		int ind=0;
		char  *p;
		p=strtok(Data,"[|]");
		pa[0]=p;
		while((p=strtok(NULL,"[|]")))
		{
			ind ++;
			if ((ind>0 )&& (ind<50))
			{
				pa[ind]=p;
			}
		}


		//定位有效性
		track->VALID=(strstr(pa[6],"精确")!=NULL)?1:0;
		//车机机ID

		strcpy(CarID,pa[2]);

		//pa[3]=2013-07-13 09:54:44
		char  * ddd[15];
		int indx=0;
		char  *a;
		a=strtok(pa[7],"- :");
		ddd[0]=a;
		while((a=strtok(NULL,"- :")))
		{
			indx ++;
			if ((indx>0 )&& (indx<5))
			{
				ddd[indx]=a;
			}
		}

		//char  * ttt[5];
		//indx=0;
		////char  *a;
		//a=strtok(pa[8],":");
		//ttt[0]=a;
		//while((a=strtok(NULL,":")))
		//{
		//	indx ++;
		//	if ((indx>0 )&& (indx<5))
		//	{
		//		ttt[indx]=a;
		//	}
		//}


		char strTime[40];
		sprintf(strTime,"%s%s%s%s%s%s",ddd[0],ddd[1],ddd[2],ddd[3],ddd[4],ddd[5]);

		str2bcd(strTime,track->GPS_TIME,7);
		syslog(LOG_INFO|LOG_USER,"gpstime %02x%02x%02x%02x%02x%02x%02x\n",
			track->GPS_TIME[0],
			track->GPS_TIME[1],
			track->GPS_TIME[2],
			track->GPS_TIME[3],
			track->GPS_TIME[4],
			track->GPS_TIME[5],
			track->GPS_TIME[6]);

		//	char aa[3];

		double x,y;
		x=atof(pa[4]);
		y=atof(pa[5]);

		char sx[20],sy[20];
		int du;
		int fen;
		int miao;
		double dfen;
		//经度
		du=x;
		fen=(x-du)*60.0;
		dfen=(x-du)*60.0;

		miao=(dfen-fen)*1000;
		memset(sx,0,20);
		sprintf(sx,"%03d%02d%d",du,fen,miao);
		str2bcd(sx,track->LONGTITUDE,4);
		syslog(LOG_INFO|LOG_USER,"lon: %s %02x%02x%02x%02x\n",sx,
			track->LONGTITUDE[0],
			track->LONGTITUDE[1],
			track->LONGTITUDE[2],
			track->LONGTITUDE[3]
		);

		//	str2bcd(sx,track->LONGTITUDE,4);

		//纬度
		du=y;
		fen=(y-du)*60.0;
		miao=((y-du)*60.0-fen)*1000;
		memset(sy,0,20);
		sprintf(sy,"%03d%02d%d",du,fen,miao);


		str2bcd(sy,track->LATITUDE,4);

		//速度
		sprintf(sy,"%04d",atoi(pa[10]));
		str2bcd(sy,track->SPEED,2);
		//方向
		sprintf(sy,"%04d",atoi(pa[11]));
		str2bcd(sy,track->DIRECTION,2);

		//坐标方向,中国
		track->sta10_EW=1;
		track->sta11_NS=1;
		//报警状态在pa12
		track->sta41_CrossAlarm=(strstr(pa[12],"越")!=NULL)?1:0;
		track->sta32_GPSOff=(strstr(pa[12],"GPS开路")!=NULL)?1:0;


		//车辆状态在pa13
		track->sta25_ACC=(strstr(pa[13],"点火")!=NULL)?1:0;
		track->sta21_DoorOpen=(strstr(pa[13],"车门开")!=NULL)?1:0;
		int Carindex=0;
		Carindex=GetCarIndexFromCarID(CarID);
		if (Carindex>=0)
		{
			memcpy(track->DRIVER_NAME,CarData[Carindex].DriverName,12);
			memcpy(track->DRIVER_IDCARDNO,CarData[Carindex].DriverID,18);
		}

		return sizeof(CenterGPSPacket);
	}
	void * 	Connect_GPS(void * sig)
	{
		while(1)
		{
			if (GPSPORT>0)
			{
			Connect_Center_server(GPSIP,GPSPORT,"GPS Server",
				DecodeCGJServerData_thread,CGJ_Connected,CGJClose);


			close(GPS_sock_fd);
			GPS_sock_fd=-1;
			}
			sleep(25);


		}

		return 0;
	}
	void * 	Connect_Server(void * sig)//返回联接的fd
	{

		while(1)
		{
			//		void * SockRecive_thread(DecodeArg * arg);
			//
			//void * SockClose_thread(DecodeArg * arg);
			//
			//void * SockConect_thread(DecodeArg * arg);
			syslog(LOG_ERR|LOG_LOCAL0,"%s %d ready connect to %s : %d\n",__FILE__,__LINE__,CENTERIP,CENTERPORT);

			JT_GPS_sock=Connect_Center_server(CENTERIP,CENTERPORT,"CB_Server",
				SockRecive_thread,SockConect_thread,SockClose_thread);


			close(JT_GPS_sock);
			JT_GPS_sock=-1;
			sleep(15);
		}

		return 0;
	}
	//产生数据包体,返回的数据在B地址中
	int DoPacket(unsigned int PacketID,char GPScenter,char* DataBody ,int DataBodyLen,char* b)
	{
		//char b[1024];
		memset(b,0,PACKETTOTALLEN);
		b[0]=0x7e;//头
		unsigned int totallen;
		totallen=DataBodyLen+7;
		b[totallen-1]=0x23;//尾
		/*	char slen[10];
		memset(slen,0,10);
		sprintf(slen,"%04d",totallen);
		str2bcd(slen,&b[1],2);*/
		b[1]=(totallen &0xff00 )>>8;//高位在前
		b[2]=(totallen & 0x00ff);//低位在后

		b[3]=(PacketID & 0XFF00)>>8;
		b[4]=(PacketID & 0x00ff);
		//memcpy(&b[3],&PacketID,2);//两字节
		b[5]=GPScenter;
		if (DataBodyLen>=1)
		{
			memcpy(&b[6],DataBody,DataBodyLen);
		}
		return totallen;
	}
	//解包省中心数据,返回数据体的长度和包类型
	int DecodePacket(char * Data ,int DataLen,char * Body,unsigned int*  PacketID)
	{
		unsigned int BodyLen;

		if ((Data[0]==0x7e) && (Data[DataLen-1]==0x23))
		{
			BodyLen=((unsigned char)Data[1]<<8) +(unsigned char)Data[2]-7;
			*PacketID=((unsigned char)Data[3]<<8)+(unsigned char)Data[4];
			memcpy(Body,&Data[6],BodyLen);
			return BodyLen;


		}
		else
		{
			return 0;//不符合格式.0长度
		}

	}
	//发送该订阅者轨迹
	int SendTrackCB()
	{
		char sql[300];
		sprintf(sql,"call sp_GetBookerTrack('%s')",Booker);
		int rscount;
		MYSQL_ROW getrow;
		MYSQL_RES *RS;
		MYSQL * conn_ptr;
		Connect_Data_Server(&conn_ptr,DataIP,dbuser,dbpwd,
			dbname,DataPort,"Get CarTrack DataBase Connected!");
		syslog(LOG_ERR|LOG_LOCAL0,"%s %d GetCarTrack sql:%s \n",__FILE__,__LINE__,sql);

		rscount=mysql_query(conn_ptr,sql);
		if (rscount)
		{
			syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d GetCarTrack Err:%s \n",
				__FILE__,__LINE__,mysql_error(conn_ptr));
			return -1;
		}
		if(!rscount)
		{
			RS=mysql_store_result(conn_ptr);
			rscount=mysql_affected_rows(conn_ptr);
			if ((RS))
			{
				while  ((getrow=mysql_fetch_row(RS))!=NULL)
				{
					//CarNickName,LastGPSDate,LastLon,LastLat,LastSpeed,LastHeading,LastAV,LastMile

					/*sscanf(getrow[0],"%s",CarData[i].CarID);
					sscanf(getrow[1],"%s",CarData[i].CarName);
					sscanf(getrow[2],"%d",&CarData[i].Color);
					sscanf(getrow[3],"%s",CarData[i].DriverName);
					sscanf(getrow[4],"%s",CarData[i].DriverID);
					sscanf(getrow[5],"%s",CarData[i].CarPhone);
					sscanf(getrow[6],"%d",&CarData[i].ManufacterID);
					sscanf(getrow[7],"%d",&CarData[i].CenterID);
					sscanf(getrow[8],"%d",&CarData[i].ServiceStatu);
					i++;*/
				}
				syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d Get %d CarTrack OK!\n",
					__FILE__,__LINE__,rscount);
				mysql_free_result(RS);

			}
		}
		else
		{
			syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d Read CarInfo Fail[%s]!\n",
				__FILE__,__LINE__,mysql_error(conn_ptr));

		}
		mysql_close(conn_ptr);
		return rscount;

	}
//车牌号
//车牌颜色 说明 数字格式字符串。白 0，黄 1，蓝 2，黑 3 长度 固定长度 1 字节
//数据字段 3 字段名称 号牌种类 说明 数字格式字符串值，符合GA 24.7的要求 长度 固定长度 2 字节，长度不够前面补 0
//数据字段 4 字段名称 卫星定位时间 说明 数字字符串，格式为 YYYYMMDDHHMMSS 长度 固定长度 6 字节。
//数据字段 5 字段名称 卫星定位经度 说明 数字格式字符串，格式为 xxx.xxxxxx  长度 固定长度 4 字节
//数据字段 6 字段名称 卫星定位纬度 说明 数字格式字符串，格式为 xx.xxxxxx  长度 固定长度 4 字节
//数据字段 7 字段名称 卫星定位高程 说明 数字格式字符串，为 xxxx，单位：米 长度 固定长度 2 字节，长度不够前面补 0
//数据字段 8 字段名称 卫星定位速度 说明 数字格式字符串，格式为 xxx.xx，单位：千米每小时 长度 固定长度 1 字节，整数长度不够前面补 0
//数据字段 9 字段名称 汽车行驶记录仪速度 说明 数字格式字符串，格式为 xxx.xx，单位：千米每小时 长度 固定长度 1 字节，整数长度不够前面补 0
//数据字段 10 字段名称 卫星定位方向 说明 单位：度，以正北方向为 0度，顺时针方向 长度 固定长度 1 字节，整数长度不够前面补 0
//数据字段 11 字段名称 机动车驾驶证号码 说明 字符串 长度 固定长度 18 字节
//数据字段 12 字段名称 基本车辆状态字 说明 16 个 Hex 码字符串，参考附加信息说明。 长度 固定长度 8 字节

	void * 	TimerCenter(void * sig)
	{
		//int i=0,j=0;
		int GPS_Beat_Heart=0;
		int Center_Beat_Heart=0;
		int Center_Beat_Heart1=0;
		int Track_heart=0;

		while(1)
		{
			sleep(1);
			//	i++;
			//	j++;
			Track_heart++;
			GPS_Beat_Heart++;
			Center_Beat_Heart++;
			Center_Beat_Heart1++;

			if ((ProtocolType==5) && (Track_heart==15))//每十五秒发送一次轨迹
				{
					 SendTrackCB(Booker);

			}
			if (GPS_Beat_Heart==60)
			{
				GPS_Beat_Heart=0;
				SendPing2GPS(GPS_sock_fd);
				//GetCarTrack();

			}
			if (Center_Beat_Heart1==60)
			{
				Center_Beat_Heart1=0;
				//SendPing2Center(Center_sock_fd);
				switch(ProtocolType)
				{
				case 3:
				case 5://顺德交通局
				if (cb_loginstatu==1)//登录
				{
				cb_send_test_link(JT_GPS_sock);
				}
			    break;
				case 4:

				  if (cb_loginstatu<=0)//未登录
			          return 0;
				lpt_send_test_link(JT_GPS_sock);
				break;
				case 6:
					if ((cb_loginstatu==1) &&(JT_GPS_sock>0)) //登录

					JT809_DO_TESTLINK(JT_GPS_sock);
					break;


				}

			}


		}
	}
	char * ToUpper(char*in)
	{
		int i;
		//	char a;
		char *b;
		b=malloc(strlen(in));

		for(i=0;i<=strlen(in);i++)
		{
			b[i]=toupper(in[i]);

		}
		memcpy(in,b,strlen(in));
		free(b);

		return in;
	}
	//登录
	int DoLogin(char * serial,char* userid,char * password,int fd)
	{
		if (fd<=0)
			return -1;

		char *ps;
		int len,result;
		char psstring[100],pack[PACKETTOTALLEN];
		memset(psstring,0,100);
		sprintf(psstring,"%s%s",password,serial);

		ps=MDString(psstring);
		//syslog(LOG_ERR|LOG_LOCAL0,"Send to Center[%s] %d Bytes\n",s,result);


		//ToUpper(ps);
		char Body[42];
		memset(Body,0,42);
		sprintf(Body,"%s",userid);
		memcpy(&Body[10],ps,32);
		syslog(LOG_ERR|LOG_LOCAL0,"Do lon in:%s %s-%s-%s fd:%d",userid,serial,psstring,ps,fd);
		len=DoPacket(LOGIN_REQ,GPSCETNTER,Body,42,pack);//编码成数据包
		result=SendPacket2Center(pack,len,fd);
		return result;
	}
	//注销请求
	int DoLogOut(int fd)
	{
		if (fd<=0)
			return -1;
		int len,result;
		char pack[PACKETTOTALLEN];
		len=DoPacket(LOGOUT_REQ,GPSCETNTER,0,0,pack);//编码成数据包
		result=SendPacket2Center(pack,len,fd);
		return result;
	}
	//发送数据包到gps中心
	int SendPacket2GPS(char * Packet,int PacketLen ,int fd)
	{
		//同步锁
		int result=0;
		if (fd>=0)
		{
			result= send(fd,Packet,PacketLen,0);
			if ((DebugMode) ||(result<=0))
			{
				int i;
				char s[1024];
				memset(s,0,1024);
				for(i=0;i<result;i++)
				{
					sprintf(s,"%s-%02X",s,Packet[i]);
				}

				//syslog(LOG_ERR|LOG_LOCAL0,"%s%d Send to GPS[%s] %d Bytes,fd:%d\n",__FILE__,__LINE__,s,result,fd);
			}

		}
		//解锁
		return result;

	}
	//发送数据包到省中心
	int SendPacket2Center(char * Packet,int PacketLen ,int fd)
	{
		//同步锁
		int result=0;

		//fd=Center_sock_fd;

		if (fd>0)
		{
			if (PacketLen==0)
			{
				PacketLen=(Packet[1]<<8)+Packet[2];
			}

			result= send(fd,Packet,PacketLen,0);
			int i;
			char s[1024];
			memset(s,0,1024);
			for(i=0;i<PacketLen;i++)
			{
				sprintf(s,"%s-%02X",s,(unsigned char)Packet[i]);
			}
			if	(result>0)
			{
				if (DebugMode)
					syslog(LOG_ERR|LOG_LOCAL0,"%s%d Send to Center[%s] %d Bytes OK,fd:%d\n",__FILE__,__LINE__,s,result,fd);
			}
			else
			{
				syslog(LOG_ERR|LOG_LOCAL0,"%s%d Send to Center[%s] %d Bytes Fail,fd:%d\n",__FILE__,__LINE__,s,result,fd);
			}

		}
		//解锁
		return result;

	}
	//登录回应
	int DoLogRep(char* Body,int BodyLen)
	{
		unsigned long Loginresult;
		Loginresult=((unsigned char)Body[0]*16777216 )+((unsigned char)Body[1]*65536) +((unsigned char)Body[2]*256) +(unsigned char)Body[3];
		//BeatHeart=0;
		switch(Loginresult)
		{
		case LOGIN_OK:
			syslog(LOG_ERR|LOG_LOCAL0,"Center Login ok\n");
			//BeatHeart=1;
			break;
		case LOGIN_ERR_INVAL_PACKET:
			syslog(LOG_ERR|LOG_LOCAL0,"Login LOGIN_ERR_INVAL_PACKET\n");
			break;
		case LOGIN_ERR_PACKETID:
			syslog(LOG_ERR|LOG_LOCAL0,"Login LOGIN_ERR_PACKETID\n");
			break;
		case LOGIN_ERR_CENTERID:
			syslog(LOG_ERR|LOG_LOCAL0,"Login LOGIN_ERR_CENTERID\n");
			break;
		case LOGIN_ERR_USER_NO:
			syslog(LOG_ERR|LOG_LOCAL0,"Login LOGIN_ERR_USER_NO\n");
			break;
		case LOGIN_ERR_PASSWORD:
			syslog(LOG_ERR|LOG_LOCAL0,"Login LOGIN_ERR_PASSWORD\n");
			break;
		case LOGIN_ERR_REQ_REFUSE:
			syslog(LOG_ERR|LOG_LOCAL0,"Login LOGIN_ERR_REQ_REFUSE\n");
			break;
		case LOGIN_ERR_LOGOUT_REFUSE:
			syslog(LOG_ERR|LOG_LOCAL0,"Login LOGIN_ERR_LOGOUT_REFUSE\n");
			break;
		default:
			syslog(LOG_ERR|LOG_LOCAL0,"Login err: unknow [%02X%02X%02X%02X]\n",Body[0],Body[1],Body[2],Body[3]);
			break;


		}
		return 1;
	}
	void * DecodeGPSServerData_thread(DecodeArg * arg)
	{
		/*	char s[MAXBUF];
		memset(s,0,MAXBUF);
		int i;
		for(i=0;i<(arg->len);i++)
		{
		sprintf(s,"%s %02X",s,(unsigned char)arg->buf[i]);
		}

		syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d Get Data [%s]\n",__FILE__,__LINE__,s);
		*/
		GPS_Beat_Heart=0;
		if ((arg->len>0) && (arg->len<MAXBUF))
		{
			DecodeGPSServer(arg->buf,arg->len,arg->fd);
		}
		free(arg);

		return 0;

	}

	//从一段字符段中截取开头为StartSyb，结束为EndSyb的一段字符串，返回到Out,长度为反回值
	int GetStr(char* RecvBuf,int nRecvLen,char  StartSyb,char  EndSyb,char* pOutBuf,int * RestPositon,int * RestLen)
	{
		int nPackFrom = 0;
		int nLen=-1;
		*RestPositon=nRecvLen;
		*RestLen=nRecvLen;


		int Position=0;
		//分析接收的数据，有可能会有多于一包的数据,或者一个数据包被分拆。

		while (nPackFrom < nRecvLen)
		{
			//包头,首先检测所有协议的报头
			if (RecvBuf[nPackFrom] != StartSyb)
			{
				nPackFrom ++;
				continue;
			}
			else
			{
				//包尾
				//长度
				nLen = 0;
				Position=nPackFrom;//从包头后开始找包尾
				while ((Position < nRecvLen) && (RecvBuf[Position] != EndSyb))
				{
					Position++;//指向一下个位置
					nLen ++;//有效数据报文加1
				}
				nLen ++;

				if (nLen < 5 || nLen > nRecvLen)
				{
					//szLogTemp2.Format ("地址(%s:%u)数据长度不对，丢掉该数据", rSocketAddress, rSocketPort);
					//theLog.WriteLogMsg (LOG_MSG, szLogTemp2, FALSE);
					break;	//包尾错误，丢掉该数据
				}

				if (nLen > nRecvLen)
				{
					//szLogTemp2.Format ("地址(%s:%u)包尾错误，丢掉该数据", rSocketAddress, rSocketPort);
					//theLog.WriteLogMsg (LOG_MSG, szLogTemp2, FALSE);
					break;	//包尾错误，丢掉该数据
				}

				//对数据进行接7E分析处理
				char* pInBuf = RecvBuf + nPackFrom;	 //报文的起始位置指针
				memset(pOutBuf,0,1000);
				memcpy(pOutBuf,pInBuf,nLen);//copy数据到输出区
				*RestPositon=Position++;//剩下的数据指针
				*RestLen=nRecvLen-Position;
				return nLen;
				break;
			}
		}
		return nLen;
	}
	int JT809_zhuanyi(char * in,int len,char * out)
	{
		int i=0,j=0;
		out[0]=in[0];//头不转
		j++;
		for(i=1;i<=len-1;i++)
		{
			if ((in[i]==0x5a) && (in[i+1]==0x01))//5a01=5b
			{
				out[j]=0x5b;
				i++;
			}
			else if ((in[i]==0x5a) && (in[i+1]==0x02))//5a02=5a
			{
				out[j]=0x5a;
				i++;
			}
			else if((in[i]==0x5e) && (in[i+1]==0x01)) //5e01=5d
			{
				out[j]=0x5d;
				i++;
			}
			else if ((in[i]==0x5e) && (in[i+1]==0x02))//5e02=5e
			{
				out[j]=0x5e;
				i++;
			}
			else
			{
			out[j]=in[i];
			}
			j++;

		}
		return j;

	}
	int JT809_Display_Login_result(int result,int fd)
	{
		switch(result)
		{
		case 0:
			syslog(LOG_ERR|LOG_LOCAL0,"%s%d JT809 Center Login OK!",__FILE__,__LINE__);
			JT_GPS_sock=fd;
			cb_loginstatu=1;
			break;
		case 1:
			syslog(LOG_ERR|LOG_LOCAL0,"%s%d JT809 Center Login FAIL IP ERROR!\n",__FILE__,__LINE__);
			break;
		case 2:
			syslog(LOG_ERR|LOG_LOCAL0,"%s%d JT809 Center Login FAIL! OMC CODE  ERROR !\n",__FILE__,__LINE__);
			break;
		case 3:
			syslog(LOG_ERR|LOG_LOCAL0,"%s%d JT809 Center Login FAIL! UNRIGISTED USER ERROR!\n",__FILE__,__LINE__);
			break;
		case 4:
			syslog(LOG_ERR|LOG_LOCAL0,"%s%d JT809 Center Login FAIL! PASSWORD ERROR!\n",__FILE__,__LINE__);
			break;
		case 5:
			syslog(LOG_ERR|LOG_LOCAL0,"%s%d JT809 Center Login FAIL! RESOURCE  ERROR!\n",__FILE__,__LINE__);
			break;
		default:
			syslog(LOG_ERR|LOG_LOCAL0,"%s%d JT809 Center Login FAIL! UNKNOW ERROR,ERROR NO:%d!\n",__FILE__,__LINE__,result);
		}
		return 1;
	}
	int JT809_REC_DATA(int fd,char *data ,int len)
	{
		// 5B
		//00 00 00 1F长度
		//00 00 00 00序列号
		//10 02 功能号
		//56 05 D5 1E 平台代码
		//01 02 0F  协议版本
		//00  加密位
		//00 00 00 00 KEY
		//00 00 00 00 00   一位登录结果加四位校验码
		//F1 5A 02  CRC
		//5D
		if (len<25)
			return -1;
		char out[1000];
		int packetlen;
		int packetno;
		if ((data[0]==0x5b) && (data[len-1]==0x5d))
		{
		//转义
		packetlen=JT809_zhuanyi(data,len,out);
		//根据保文号处理
		packetno=(out[9]<<8)+out[10];
		switch(packetno)
		{
		case UP_CONNECT_RSP://登陆回应
			JT809_Display_Login_result(out[23],fd);
			break;
		default:
			break;
		}

		}


	return len;
	}
	void * SockRecive_thread(DecodeArg * arg)
	{
		char buf[MAXBUF];//数据区
		int len;//长度
		int fd;//SOCK号
	    char a[1000];

		if (arg->len>9)
		{
			len=arg->len;
			memcpy(buf,arg->buf,arg->len);
			fd=arg->fd;
			switch(ProtocolType)
			{
			case 3://cb
			syslog(LOG_ERR|LOG_LOCAL0,"recive CHANGBAO format from fd:%d data %d bytes %s\n",fd,len,buf);
			cb_decode_data(fd,buf,len);
			break;
			case 4://ltp
			syslog(LOG_ERR|LOG_LOCAL0,"recive LPT format from fd:%d data %d bytes %s\n",fd,len,buf);
			lpt_decode_data(fd,buf,len);
				break;
			case 5://顺德交通
				syslog(LOG_ERR|LOG_LOCAL0,"recive JT809 format from fd:%d data %d bytes %s\n",fd,len,buf);
			cb_decode_data(fd,buf,len);
				break;
			case 6://粤裕丰
				syslog(LOG_ERR|LOG_LOCAL0,"recive JT809 format from fd:%d data %d bytes %s\n",fd,len,buf);
				if (DebugMode)
					{

					 memset(a,0,1000);
					 PrintArray(buf,len,a);
					syslog(LOG_ERR|LOG_LOCAL0,"%s %d recive from JT809:%d<%s>!\n",	__FILE__,__LINE__,fd,a);
					}
			JT809_REC_DATA(fd,buf,len);
				break;


			}

		}
		else
		{
			syslog(LOG_ERR|LOG_LOCAL0,"%s %d Get %d bytes Data [%s] fd:%d\n",__FILE__,__LINE__,arg->len,arg->buf,arg->fd);
			/*if (DebugMode)
					{
					 char a[1000];
					 memset(a,0,1000);
					 PrintArray(buf,len,a);
					syslog(LOG_ERR|LOG_LOCAL0,"%s %d send  to fd:%d<%s>!\n",	__FILE__,__LINE__,fd,a);
					}*/
		}
		return 0;
	}
	int SockClose_thread(int fd)
	{
		close(fd);
		JT_GPS_sock=-1;
       cb_loginstatu=0;//未登录

		return 1;
	}

	int JT809_DO_TESTLINK(int fd)
	{
	   char data[1000];
		char packetdata[1000];
		int packetlen=0;
		int len=0;
		//int JT809_login_data(int userid,char * password,char * down_link_ip,short down_link_port,char * packetdata)
		//len=JT809_login_data(atoi(username),password,"211.147.224.218",0000,data);

		//int JT809_encode_data(int  funword,int  ocm_code, char * data, int datalen,char * packet)
		packetlen=JT809_encode_data(UP_LINKTEST_REQ,atoi(ocm_code),data,0,packetdata);
		if (packetlen>0)
		len=cb_lpt_sendto(fd,packetdata,packetlen);
		return len;
	}
	int JT809_DO_LOGIN(char * username, char * password, int fd)
	{
		char data[1000];
		char packetdata[1000];
		int packetlen=0;
		int len=0;
		//int JT809_login_data(int userid,char * password,char * down_link_ip,short down_link_port,char * packetdata)
		len=JT809_login_data(atoi(username),password,"211.147.224.218",9999,data);

		//int JT809_encode_data(int  funword,int  ocm_code, char * data, int datalen,char * packet)
		packetlen=JT809_encode_data(UP_CONNECT_REQ,atoi(ocm_code),data,len,packetdata);
		if (packetlen>0)
		len=cb_lpt_sendto(fd,packetdata,packetlen);

		return len;
	}
	int SockConect_thread(int fd)
	{
		switch(ProtocolType)
		{
		case 4://丽普盾登录。
			lpt_login(CENTERUSER, CENTERPASSWORD, fd);
			break;
		case 5://顺德交通局
			break;
		case 6://裕丰钢铁
			syslog(LOG_ERR|LOG_LOCAL0,"%s %d ready to login yfgt fd:%d user:%s password:%s\n",__FILE__,__LINE__,fd,CENTERUSER,CENTERPASSWORD);
			JT809_DO_LOGIN(CENTERUSER,CENTERPASSWORD,fd);
			break;
		default:
			return 0;

		}

		   JT_GPS_sock=fd;
		return 1;
	}
	void * DecodeCGJServerData_thread(DecodeArg * arg)
	{

		GPS_Beat_Heart=0;
		int ret,inlen=0;
		char * p;
		char o[2048];
		//char Data[2048];
		int len=100;

		//char res[2048];
		ret=arg->len ;
		p=arg->buf ;

		len=GetStr(p,ret,'[',']',o,&inlen,&ret);
		if (len>=10)
		{
			//syslog(LOG_ERR|LOG_LOCAL0,"%s %d Get Data [%s]\n",__FILE__,__LINE__,o);
			ChangCGJ2JT808(o,len,arg->fd,ProtocolType);

		}


		//ChangCGJ2JT808(o,inlen,arg->fd,ProtocolType);
		//  restlen=restlen-len;

		//}

		//while(ret>1)
		//	 {
		//	 memset(o,0,2048);
		//	   ret=GetAPacket(p,inlen,o,res,&lesslen);
		//      if( ret>1)
		//	   {
		//	 // syslog(LOG_ERR|LOG_LOCAL0,"%s %d Get Data {%s}%dbytes\n",__FILE__,__LINE__,o,ret);
		//	 //  printf("ret=%d <%s>\n",ret,o);
		//    //   DecodeCGJGPSServer(o,ret,arg->fd);
		//	 // ChangCGJ2JT808(o,ret,arg->fd);
		//	   }
		//      p=res;
		//      inlen=lesslen;
		//	 }
		//DecodeCGJGPSServer(arg->buf,arg->len,arg->fd);


		return 0;

	}
	void * DecodeCenterData_thread1(DecodeArg * arg)
	{
		Center_Beat_Heart1=0;
		if ((arg->len>0) && (arg->len<MAXBUF))
		{
			char s[MAXBUF];
			memset(s,0,MAXBUF);
			int i;
			for(i=0;i<(arg->len);i++)
			{
				sprintf(s,"%s %02X",s,(unsigned char)arg->buf[i]);
			}

			syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d fd:%d Get Center Data [%s]\n",__FILE__,__LINE__,arg->fd,s);

			DecodeCenterPacket1(arg->buf,arg->len,arg->fd);
		}
		free(arg);

		return 0;

	}

	void * DecodeCenterData_thread(DecodeArg * arg)
	{
		Center_Beat_Heart=0;
		if ((arg->len>0) && (arg->len<MAXBUF))
		{
			char s[4098];
			memset(s,0,MAXBUF);
			int i;
			for(i=0;i<(arg->len);i++)
			{
				sprintf(s,"%s %02X",s,(unsigned char)arg->buf[i]);
			}
			if (DebugMode)
				syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d fd:%d Get Center Data [%s]\n",__FILE__,__LINE__,arg->fd,s);
			GetACWPacket(arg->buf,arg->len,arg->fd);
			//DecodeCenterPacket(arg->buf,arg->len,arg->fd);
		}
		//free(arg);

		return 0;

	}
	//从文分包程序
	int GetACWPacket(char * buff,int len,int fd)
	{
		//char buf[1024];
		char * st=NULL;
		char * en=NULL;
		int i;
		for(i=0;i<len;i++)
		{
			if (buff[i]==0x7e)
				st=buff+i;
			if (buff[i]==0x23)
				en=buff+i;
			if(en!=NULL && st!=NULL && en-st>6 && en-st<=len)
			{
				if (DebugMode)
					/*	syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d fd:%d Get Center Data Packet st:%d en:%d len:%d\n",
					__FILE__,__LINE__,fd,st,en,en-st+1);*/

					DecodeCenterPacket(st,en-st+1,fd);
				st=NULL;
				en=NULL;
			}
		}
		return 1;
	}

	// 7E 00 3F 00 01 45 00 02 D4 C1 45 31 31 39 38 35 00 00 02 00 29 86 06 00 22 00 00 00 00 00 00 16 0E 00 00 00 00 00 00 00 00 16 0E 00 00 00 00 00 00 00 00 31 33 39 32 34 38 31 33 31 37 34 23

	//#define CENTERUSER "test"
	//#define CENTERPASSWORD "test"
	//接收到中心数据的处理
	int DecodeCenterPacket(char * Data ,unsigned int DataLen,int fd)
	{

		char s[4098];
		memset(s,0,MAXBUF);
		int i;
		for(i=0;i<DataLen;i++)
		{
			sprintf(s,"%s %02X",s,(unsigned char)Data[i]);
		}
		if (DebugMode)
			syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d fd:%d Get Center DataPacket [%s]\n",__FILE__,__LINE__,fd,s);
		unsigned int PacketID=0,BodyLen=0;
		char Body[PACKETTOTALLEN];
		memset(Body,0,PACKETTOTALLEN);
		BodyLen=DecodePacket(Data,DataLen,Body,&PacketID);
		//fd=Center_sock_fd;
		switch (PacketID)
		{
		case LOGIN_RANDOM_SERIAL:
			DoLogin(Body,CENTERUSER,CENTERPASSWORD,fd);//登录
			break;
		case LOGIN_REQ://登录请求,主动发
			//DoLogOutRep(Body,BodyLen);
			syslog(LOG_ERR|LOG_LOCAL0,"Get a LOGIN_REQ from Center fd:%d!\n",fd);
			break;
		case LOGIN_RSP:
			DoLogRep(Body,BodyLen);//登录回应
			break;
		case LOGOUT_REQ:
			syslog(LOG_ERR|LOG_LOCAL0,"Get a LOGOUT_REQ from Center fd:%d!\n",fd);
			break;
		case LOGOUT_RSP:
			DoLogRep(Body,BodyLen);//注消回应
			break;
		case APPLY_REQ://申请请求
			DoApplyRequest(Body,BodyLen);//处理省中心的申请要求.
			break;
		case APPLY_RSP:
			DoApplyRsp(Body,BodyLen);//申请结果
			break;
		case DELIVER://递交
			syslog(LOG_ERR|LOG_LOCAL0,"Get a DELIVER from Center fd:%d\n",fd);
			break;
		case LINKTEST_REQ:
			syslog(LOG_ERR|LOG_LOCAL0,"Get a LINKTEST_REQ from Center fd:%d!\n",fd);
			break;
		case LINKTEST_RSP:
			syslog(LOG_ERR|LOG_LOCAL0,"Get a LINKTEST_RSP from Center fd:%d!\n",fd);
			break;
		default:
			syslog(LOG_ERR|LOG_LOCAL0,"Get a PacketID Data from Center[%d],fd:%d\n",PacketID,fd);
			return 0;
		}
		return 1;
	}

	int DecodeCenterPacket1(char * Data ,unsigned int DataLen,int fd)
	{
		unsigned int PacketID,BodyLen;
		char Body[PACKETTOTALLEN];
		memset(Body,0,PACKETTOTALLEN);
		BodyLen=DecodePacket(Data,DataLen,Body,&PacketID);   //获取数据体和数据类型标识
		//Center_sock_fd1=fd;
		fd=Center_sock_fd1;
		switch (PacketID)
		{
		case LOGIN_RANDOM_SERIAL:
			DoLogin(Body,CENTERUSER1,CENTERPASSWORD1,fd);//登录

			break;
		case LOGIN_REQ://登录请求,主动发
			//DoLogOutRep(Body,BodyLen);
			syslog(LOG_ERR|LOG_LOCAL0,"Get a LOGIN_REQ from Center!\n");
			break;
		case LOGIN_RSP:
			DoLogRep(Body,BodyLen);//登录回应
			break;
		case LOGOUT_RSP:
			DoLogRep(Body,BodyLen);//注消回应
			break;
		case APPLY_REQ://申请请求
			DoApplyRequest(Body,BodyLen);//处理省中心的申请要求.
			break;
		case APPLY_RSP:
			DoApplyRsp(Body,BodyLen);//申请结果
			break;
		case DELIVER://递交
			break;

			//case LINKTEST_REQ:
			//	break;
		case LINKTEST_RSP:
			syslog(LOG_ERR|LOG_LOCAL0,"Get a LINKTEST_RSP from Center1!\n");
			break;
		default:
			syslog(LOG_ERR|LOG_LOCAL0,"Get a PacketID Data from Center1[%d]!\n",PacketID);
			return 0;



		}

		return 1;
	}

	//发送申请,省中心允许后进行工作
	int ApplyScusess(int Appcmdid,char *CarName,char Color)
	{
		//#define COMMAND_TYPE_GPS_REAL_FORWORD 0X0001
		//#define COMMAND_TYPE_GPS_HISTORY_FORWORD 0X0002
		//#define COMMAND_TYPE_CAR_INFO_UPLOAD 0X0003
		//#define COMMAND_TYPE_IMAGE_UPLOAD 0X0004
		//#define COMMAND_TYPE_TEXT_SEND 0X0005
		//#define COMMAND_TYPE_LISTEN 0X0006
		switch (Appcmdid)
		{
		case COMMAND_TYPE_GPS_REAL_FORWORD://允许实时GPS定位转发申请
			syslog(LOG_ERR|LOG_LOCAL0,"Apply Response OK, APPLY COMMAND ID:%d,COMMAND_TYPE_GPS_REAL_FORWORD\n",Appcmdid);
			break;
		case COMMAND_TYPE_GPS_HISTORY_FORWORD://允许历史数据转发申请
			syslog(LOG_ERR|LOG_LOCAL0,"Apply Response OK, APPLY COMMAND ID:%d,COMMAND_TYPE_GPS_HISTORY_FORWORD\n",Appcmdid);
			break;
		case COMMAND_TYPE_CAR_INFO_UPLOAD://允许车辆资料上报申请
			syslog(LOG_ERR|LOG_LOCAL0,"Apply Response OK, APPLY COMMAND ID:%d,COMMAND_TYPE_CAR_INFO_UPLOAD\n",Appcmdid);

			UploadCarInfo(CarName,Color);//通过递交方式上传车辆资料
			break;
		case COMMAND_TYPE_IMAGE_UPLOAD://允许图像数据上报申请
			syslog(LOG_ERR|LOG_LOCAL0,"Apply Response OK, APPLY COMMAND ID:%d,COMMAND_TYPE_IMAGE_UPLOAD\n",Appcmdid);
			//	UploadImage(CarName,Color);//上传车辆图片
			break;
		case COMMAND_TYPE_TEXT_SEND:  //调度指令发送
			syslog(LOG_ERR|LOG_LOCAL0,"Apply Response OK, APPLY COMMAND ID:%d,COMMAND_TYPE_TEXT_SEND：\n",Appcmdid);

			break;
		case COMMAND_TYPE_LISTEN://请求监听

			syslog(LOG_ERR|LOG_LOCAL0,"Apply Response OK, APPLY COMMAND ID:%d,COMMAND_TYPE_LISTEN\n",Appcmdid);
			break;

		default:
			syslog(LOG_ERR|LOG_LOCAL0,"Apply Response OK,unknow APPLY COMMAND ID:%d\n",Appcmdid);

		}
		return 1;
	}
	//上传车辆资料
	int UploadCarInfo(char *CarName,char Color)
	{
		//7E 00 3F 00 01 45 00 02 D4 C1 48 4B 32 30 34 39 00 00 02 00 29 56 06 00 16 00 00 00 00 00 00 16 0D 00 00 00 00 00 00 00 00 16 0E 00 00 00 00 00 00 00 00 31 35 30 32 34 34 31 39 32 34 32 23
		// 7E 00 3F 00 01 45 00 02 D4 C1 59 41 38 38 31 31 00 00 02 00 29 86 06 00 22 00 00 00 00 00 00 16 0E 00 00 00 00 00 00 00 00 16 0E 00 00 00 00 00 00 00 00 31 33 38 32 33 34 36 39 35 30 39 23
		unsigned int VENDOR_ID;//	10 byte	STRING	运营商代码
		unsigned int FACTORY_ID;//10 byte	STRING	厂商代码
		char DEVICE_ID[11];//	10 byte	STRING	终端设备编号
		char COMMUNICATION_NO[12];//	11 byte	STRING	通信号（SIM/UIM卡号）*/
		char CarBaseInfo[41];
		memset(CarBaseInfo,0,41);
		int i,result=0;
		for(i=0;i<MAXCARNUMBER;i++)
		{

			if ((strcmp(CarData[i].CarName,CarName)==0) &&( CarData[i].Color==Color))
			{
				FACTORY_ID=CarData[i].ManufacterID;
				VENDOR_ID=CarData[i].CenterID;
				strcpy(DEVICE_ID,CarData[i].CarID);
				strcpy(COMMUNICATION_NO,CarData[i].CarPhone);
				CarData[i].ServiceStatu=2;
				result=1;
				break;
			}
			result=0;
		}
		if (result==0)   //没资料
			return 0;
		//运营商代码：佛山航旗 86.6.0.22
		CarBaseInfo[0]=86;
		CarBaseInfo[1]=6;
		CarBaseInfo[2]=0;
		CarBaseInfo[3]=22;

		/* CarBaseInfo[0]= ( VENDOR_ID & 0xff000000)>>24;
		CarBaseInfo[1]=( VENDOR_ID & 0x00ff0000)>>16;;
		CarBaseInfo[2]=( VENDOR_ID & 0x0000ff00)>>8;;
		CarBaseInfo[3]=( VENDOR_ID & 0x000000ff)>>0;;*/
		//制造商代码   ://华宝0x160E   160D

		CarBaseInfo[10]=0x16;
		CarBaseInfo[11]=0x0d;
		/*  CarBaseInfo[10]=(FACTORY_ID & 0xff00)>>8;
		CarBaseInfo[11]=FACTORY_ID & 0x00ff;*/

		//车机ID
		unsigned long lCarid;
		lCarid=atol(DEVICE_ID);
		/*  CarBaseInfo[20]=lCarid/0x1000000;
		CarBaseInfo[21]=lCarid/0x10000;
		CarBaseInfo[22]=lCarid/0x100;
		CarBaseInfo[23]=lCarid/0x1*/;
		CarBaseInfo[20]=0x16;
		CarBaseInfo[21]=0x0e;



		//strcpy(&CarBaseInfo[20],DEVICE_ID);

		//通信码
		strcpy(&CarBaseInfo[30],COMMUNICATION_NO);
		//产生递交数据包
		int  datalen,CenterDataLen;
		char DeliverData[1024];
		char CenterData[1024];

		datalen=DoDeliver(DELIVER_CMD_ID_CARINFO,CarName,Color,41,CarBaseInfo,0,DeliverData);
		CenterDataLen=DoPacket(DELIVER,GPSCETNTER,DeliverData,datalen,CenterData);//编码成数据包
		//发送
		SendPacket2Center(CenterData,CenterDataLen,Center_sock_fd);
		syslog(LOG_ERR|LOG_LOCAL0,"上传车辆基本资料:%s color:%d\n",CarName,Color);

		return 1;
	}
	int DoApplyRsp(char* Body,int BodyLen)//申请回应
	{
		int Appcmdid;
		char CarName[11];
		char Color;
		Appcmdid=(Body[0]<<8)+Body[1];
		memset(CarName,0,11);
		memcpy(CarName,&Body[2],10);//10字节车牌号
		Color=Body[12];//车牌色
		long result;
		result=(Body[13]<<24 )+ (Body[14]<<16) +(Body[15]<<8) + (Body[16]);
		switch (result)
		{
		case APPLY_CMD_OK:
			syslog(LOG_ERR|LOG_LOCAL0,"	%s%d ApplyID:%d AppResult:%ld Center ApplyCommandok CarName:%s colore:%d\n",__FILE__,__LINE__,Appcmdid,result,CarName,Color);
			ApplyScusess(Appcmdid,CarName,Color);//成功后进行下一步递交数据
			break;
		case LOGIN_ERR_INVAL_PACKET:
			syslog(LOG_ERR|LOG_LOCAL0,"	%s%d ApplyID:%d AppResult:%ld Center INVAL_PACKET CarName:%s colore:%d\n",__FILE__,__LINE__,Appcmdid,result,CarName,Color);
			break;
		case LOGIN_ERR_PACKETID:
			syslog(LOG_ERR|LOG_LOCAL0,"	%s%d ApplyID:%d AppResult:%ld Center ERR_PACKETID CarName:%s colore:%d\n",__FILE__,__LINE__,Appcmdid,result,CarName,Color);
			break;
		case LOGIN_ERR_CENTERID:
			syslog(LOG_ERR|LOG_LOCAL0,"	%s%d ApplyID:%d AppResult:%ld Center ERR_CENTERID CarName:%s colore:%d\n",__FILE__,__LINE__,Appcmdid,result,CarName,Color);
			break;
		case LOGIN_ERR_USER_NO:
			syslog(LOG_ERR|LOG_LOCAL0,"	%s%d ApplyID:%d AppResult:%ld Center ERR_USER_NO CarName:%s colore:%d\n",__FILE__,__LINE__,Appcmdid,result,CarName,Color);
			break;
		case LOGIN_ERR_PASSWORD:
			syslog(LOG_ERR|LOG_LOCAL0,"	%s%d ApplyID:%d AppResult:%ld Center ERR_PASSWORD CarName:%s colore:%d\n",__FILE__,__LINE__,Appcmdid,result,CarName,Color);
			break;
		case LOGIN_ERR_REQ_REFUSE:
			syslog(LOG_ERR|LOG_LOCAL0,"	%s%d ApplyID:%d AppResult:%ld Center ERR_REQ_REFUSE CarName:%s colore:%d\n",__FILE__,__LINE__,Appcmdid,result,CarName,Color);
			break;
		case LOGIN_ERR_LOGOUT_REFUSE:
			syslog(LOG_ERR|LOG_LOCAL0,"	%s%d ApplyID:%d AppResult:%ld Center ERR_LOGOUT_REFUSE CarName:%s colore:%d\n",__FILE__,__LINE__,Appcmdid,result,CarName,Color);
			break;
		default:
			syslog(LOG_ERR|LOG_LOCAL0,"	%s%d ApplyID:%d AppResult:%ld Center Unknow RequesResult CarName:%s colore:%d\n",__FILE__,__LINE__,Appcmdid,result,CarName,Color);
			break;
		}

		return 1;
	}




	//申请请求
	int DoApply(short  Apply_Cmd_id,char* CarName,char Color, unsigned short CMD_len,char * CMD_content,int fd)
	{
		if (fd<=0)
			return -1;
		unsigned char buf[PACKETTOTALLEN];
		char pack[PACKETTOTALLEN];
		int len,result;
		memset(buf,0,PACKETTOTALLEN);
		buf[0]=(Apply_Cmd_id & 0xff00)>>8;
		buf[1]=Apply_Cmd_id & 0x00ff;
		if (CarName!=NULL)
		{
			memcpy(&buf[2],CarName,10);
		}
		buf[12]=Color;
		buf[13]=(CMD_len & 0xff00)>>8;
		buf[14]=CMD_len & 0x00ff;
		if ((CMD_content!=NULL) && (CMD_len>0))
		{
			memcpy(&buf[15],CMD_content,CMD_len);
		}
		len=DoPacket((short)APPLY_REQ,GPSCETNTER,(char*)buf,CMD_len+15,pack);
		result=SendPacket2Center(pack,len,fd);
		syslog(LOG_INFO|LOG_USER,"%s%d Car%s,Color:%d DoApplyID:%d\n",__FILE__,__LINE__,CarName,Color,Apply_Cmd_id);
		return result;
	}


	//申请车辆实时位置
	int GetGPS_Real_Forword(char* CarName,char Color,int fd)
	{
		return DoApply(COMMAND_TYPE_GPS_REAL_FORWORD,CarName,Color,0,0,fd);


	}
	//申请车辆历史数据StartDateTime:YYYYMMDDHHMMSS
	int GetCarHistory(char * CarName,char Color,char* StartDateTime,char* EndDateTime,int fd)
	{
		char CMD[30];
		memset(CMD,0,30);
		return DoApply(COMMAND_TYPE_GPS_HISTORY_FORWORD,CarName,Color,14,CMD,fd);
	}
	//车辆资料上传申请
	int GetGPS_CarInfo_Forword(char* CarName,char Color,int fd)
	{
		return DoApply(COMMAND_TYPE_CAR_INFO_UPLOAD,0,0,0,0,fd);
	}

	//车辆图像上传申请
	int GetGPS_Image_Forword(char* CarName,char Color,int fd)
	{
		return DoApply(COMMAND_TYPE_IMAGE_UPLOAD,CarName,Color,0,0,fd);


	}

	//产生递交数据包,返回数据在地址ret指针中,长度为返回值
	int DoDeliver(short DELIVER_CMD_ID,char* CarName,char Color,
		unsigned short CMD_len,char * CMD_content,
		int fd,char * ret)
	{
		unsigned char buf[PACKETTOTALLEN];
		memset(buf,0,PACKETTOTALLEN);

		buf[0]=(DELIVER_CMD_ID & 0xff00)>>8;
		buf[1]=DELIVER_CMD_ID & 0x00ff;
		if (CarName!=NULL)
		{
			memcpy(&buf[2],CarName,10);
		}
		buf[12]=Color;
		/*	char slen[10];
		memset(slen,0,10);
		sprintf(slen,"%04d",CMD_len);
		str2bcd(slen,(unsigned char *)&buf[13],2);*/
		buf[13]=(CMD_len & 0xff00)>>8;
		buf[14]=CMD_len & 0x00ff;
		if ((CMD_content!=NULL) && (CMD_len>0))
		{
			memcpy(&buf[15],CMD_content,CMD_len);
		}
		memcpy(ret,buf,CMD_len+15);//返回数据
		return CMD_len+15;
	}

	//得到一段标准报文，返回本次取得报文的字节数。
	int GetAPacket(char * buf,int inlen,char * out,char* res,int * leslen)
	{
		char InBuf[2048];
		memcpy(InBuf,buf,inlen);
		int iFrom=0,iEnd=0;
		while(iFrom<inlen)
		{
			if (InBuf[iFrom]=='[')
				break;
			iFrom++;
		}
		iEnd=iFrom+1;
		while(iEnd<inlen)
		{
			if (InBuf[iEnd]==']')
				break;
			iEnd++;
		}
		if (iFrom<iEnd && iEnd<inlen)
		{
			memset(out,0,1024);
			memcpy(out,InBuf+iFrom,iEnd-iFrom+1);
			memset(res,0,1024);
			memcpy(res,InBuf+iEnd+1,inlen-(iEnd+1));
			*leslen=inlen-(iEnd+1);
			return iEnd-iFrom+1;
		}
		return 0;


	}
	unsigned char BCD2Val(unsigned char x) //将BCD格式的时间数据转为实际的值，如 29秒的BCD格式为 0x29 ，实际值为 0x1d
	{
		return (x>>4)*10+(x&0x0f);
	}

	unsigned char Val2BCD(unsigned char x) //将值转为BCD格式，如 23转为BCD格式为 0x23
	{
		return (x/10)*16+(x%10);
	}
	//YYYYMMDDHHMMSS转BCD码
	int StrTime2BCD(char * str,char *BCD)
	{
		if (strlen(str)<14)
			return -1;
		char y1[5];
		memset(y1,0,5);
		memcpy(y1,str,2);
		BCD[0]=Val2BCD((char)atoi(y1));

		memset(y1,0,5);
		memcpy(y1,str+2,2);
		BCD[1]=Val2BCD((char)atoi(y1));

		memset(y1,0,5);
		memcpy(y1,str+4,2);
		BCD[2]=Val2BCD((char)atoi(y1));

		memset(y1,0,5);
		memcpy(y1,str+6,2);
		BCD[3]=Val2BCD((char)atoi(y1));

		memset(y1,0,5);
		memcpy(y1,str+8,2);
		BCD[4]=Val2BCD((char)atoi(y1));

		memset(y1,0,5);
		memcpy(y1,str+10,2);
		BCD[5]=Val2BCD((char)atoi(y1));

		memset(y1,0,5);
		memcpy(y1,str+12,2);
		BCD[6]=Val2BCD((char)atoi(y1));
		return 1;
	}


	//发送心跳数据到车管家中心
	int SendPing2GPS(int fd)
	{

		if (fd<=0)
			return -1;
		int result;
		//char pack[]="[PING|fsjtj|123456";


		result=SendPacket2GPS(GPS_BeatHear_String,strlen(GPS_BeatHear_String),fd);


		return result;


	}
	int SendPing2CGJGPS(int fd)
	{

		if (fd<=0)
			return -1;
		int result;
		//char pack[]="[PING|fsjtj|200202121111]";


		result=SendPacket2GPS(GPS_BeatHear_String,strlen(GPS_BeatHear_String),fd);


		return result;


	}


	//发送心跳数据到省中心
	int SendPing2Center(int fd)
	{

		if (fd<=0)
			return -1;
		int len,result;
		char pack[PACKETTOTALLEN];

		len=DoPacket((short)LINKTEST_REQ,GPSCETNTER,NULL,0,pack);
		result=SendPacket2Center(pack,len,Center_sock_fd1);


		return result;


	}
	/*
	#define CENTERPORT 9000 //12501-12510
	#define CENTERIP "192.168.0.102"
	#define CENTERUSER "test"
	#define CENTERPASSWORD "test"
	*/

	int CGJGPS_Connected(int fd)//联上后发送的数据
	{

		GPS_sock_fd=fd;
		return 1;
	}
	int CGJ_Connected(int fd)//联上后发送的数据
	{

		send(fd,GPSLogInString,strlen(GPSLogInString),0);
		//send(fd,log,20,0);
		GPS_sock_fd=fd;
		return 1;
	}

	int SJT_Connected(int fd)//联上后发送的数据
	{
		Center_sock_fd=fd;
		return 1;
	}
	int GPS_Connected(int fd)//联上后发送的数据
	{

		//4C 4F 47 49 20 3C 4F 68 3F 59 69 4F 43 68 59 69 44 20 D A

		char log[]={0x4C,0x4F,0x47,0x49,0x20,0x3C,0x4F,0x66,0x3E,0x67,0x68,0x69,0x2E,0x6A,0x6B,0x4F,0x2E,0x59,0x69,0x6D,0x44,0x6D,0x67,0x2E,0x3E,0x20,0x0D,0x0A};
		//char log[]={0x4C,0x4F,0x47,0x49,0x20,0x3C,0x4F,0x68,0x3F,0x59,0x69,0x4F,0x43,0x68,0x59,0x69,0x44,0x20,0x0D,0x0A};
		send(fd,log,28,0);
		//send(fd,log,20,0);
		GPS_sock_fd=fd;
		return 1;
	}

	int Connect_Center_server(char * ServerIP,int Port,char* ServerName,
		void *(*Data_Thread)( DecodeArg  * arg),
		int(*Connected_Fun)(int fd),
		int(*Close_Fun)(int fd)
		)
	{


		int sockfd,recv_count=0;
		struct sockaddr_in  GprsServer;

		//int recv_count=0;
		sockfd=socket(AF_INET,SOCK_STREAM,0);

		if (sockfd<0)
		{
			syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d Can not Creat  %s Sock,err: %s\n",__FILE__,__LINE__,ServerName,strerror(errno));
			return -1;
		}
		bzero(&GprsServer,sizeof(GprsServer));
		GprsServer.sin_family=AF_INET;
		GprsServer.sin_port=htons(Port);
		GprsServer.sin_addr.s_addr=inet_addr(ServerIP);

	syslog(LOG_ERR|LOG_LOCAL0,"%s %d Connecteding to  %s ip:%s port:%d!\n",__FILE__,__LINE__,ServerName,ServerIP,Port);
	int connectresult=0;
	connectresult=connect(sockfd,(struct sockaddr *)&GprsServer,sizeof(GprsServer));

	syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d  %s Center Connect result:%d,err: %s\n",__FILE__,__LINE__,ServerName,connectresult,strerror(errno));
		if (connectresult!=-1)
		{
			syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d Connected to  %s scusess ip:%s port:%d! fd:%d\n",__FILE__,__LINE__,ServerName,ServerIP,Port,sockfd);
			if (Connected_Fun!=NULL)
			{
				Connected_Fun(sockfd);
			}
		}
		else
		{
			syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d Can not Connect to %s Server!,err: %s\n",__FILE__,__LINE__,ServerIP,strerror(errno));
			sockfd=-1;
			close(sockfd);
			return -1;
		}
		char buf[MAXBUF];
		while(1)
		{
			memset(buf,0,MAXBUF);
			recv_count=recv(sockfd,buf,MAXBUF,0);

			if (recv_count>0 )
			{
				arg.fd=sockfd;
				arg.len=recv_count ;

				memcpy(arg.buf,buf,recv_count);
				//syslog(LOG_ERR|LOG_LOCAL0,"%s %d fd:%d read:%s \n",__FILE__,__LINE__,arg.fd,arg.buf);
				/*if (DebugMode)
					syslog(LOG_ERR|LOG_LOCAL0,"%s %d fd:%d read:%s \n",__FILE__,__LINE__,arg.fd,arg.buf);*/
				Data_Thread((void *)&arg);
			}
			else if(recv_count==0)
			{
				syslog(LOG_ERR|LOG_LOCAL0,"%s %d %s fd:%d Recive %d bytes Closed  ! err: %s errno:%d\n",__FILE__,__LINE__,ServerName,sockfd,recv_count,strerror(errno),errno);
				if (Close_Fun!=NULL)
				{
					Close_Fun(sockfd);
				}
				close(sockfd);

				return -1;
			}
			else if  ((recv_count<0 )&& (errno == EINTR || errno == EWOULDBLOCK || errno == EAGAIN))
			{
			  continue;
			}
			else
			{
			close(sockfd);
			}
		}

		return sockfd;
	}

	int Connect_Data_Server(MYSQL** cconn_ptr,char * host,char * username,char * mysqlpassword ,
		char * database,int dbport,char * Con_Prompt)
	{
		if (strlen(mysqlpassword)==0)
			strcpy(mysqlpassword,"jhydz203");
		int result=1;
		syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d Connecting  %s Data_Server!\n",__FILE__,__LINE__,Con_Prompt);
		*cconn_ptr=mysql_init(NULL);
		if (!cconn_ptr)
		{
			syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d %s mysql init failed! err:%s\n",__FILE__,__LINE__,Con_Prompt,strerror(errno));
			return -1;
		}
		syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d %s mysql_connecting host:%s database:%s \nuser:%s password:%s\n",
			__FILE__,__LINE__,Con_Prompt,host,database,username,mysqlpassword);
		if(mysql_real_connect(*cconn_ptr,host,username,mysqlpassword,database,
			dbport,NULL,CLIENT_MULTI_RESULTS))
		{
			syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d %s database connect scusess!\n",__FILE__,__LINE__,Con_Prompt);
			//	mysql_query(*cconn_ptr,"set interactive_timeout=24*3600" );
			mysql_query(*cconn_ptr,"set names gb2312" );
			result=1;
		}
		else
		{
			syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d %s Database connect Fail[%s]!\n",
				__FILE__,__LINE__,Con_Prompt,mysql_error(*cconn_ptr));
			result=-1;
		}
		return result;

	}

	int SendCarInfoRequest()
	{
		return 0;
	}
	//发送最新车辆数据上省中心
	int SendNewCarInfo2Center()
	{
		/*	int Commanddbconnected,Result;
		MYSQL *pCommandConnection;
		MYSQL_ROW getrow;
		MYSQL_RES *RS;
		char sql[500];//查询指令字符
		sprintf(sql,"In Send to Center Database init Connect,Complie date:%s %s\n",__DATE__,__TIME__);
		Commanddbconnected=Connect_Data_Server(&pCommandConnection,DataIP,dbuser,dbpwd,dbname,DataPort,sql);
		sprintf(sql,"selec
		Result=mysql_real_query(pCommandConnection,sql,strlen(sql));

		if(!Result)
		{
		RS=mysql_store_result(pCommandConnection);
		if (RS!=NULL)
		{

		while  ((getrow=mysql_fetch_row(RS)))
		{
		CommandID=0;
		sscanf(getrow[0],"%d",&Channel);//取发送通道
		sscanf(getrow[1],"%s",Command);//取发送内容
		sscanf(getrow[2],"%s",Target);//取目标收机号或客户端号
		sscanf(getrow[3],"%d",&CommandID);//取命令ID
		//syslog(LOG_INFO|LOG_USER,"Get CH:%d ComID:%d Target:%s Command:%s!",Channel,CommandID,Target,Command);
		//sleep(1);//每条指令发送间隔1秒
		switch (Channel)//根据命令通道号进行命令处理
		{
		case 0://发往TCP通信服务器的指令
		//	syslog(LOG_INFO|LOG_USER,"Send CommandString:%s to GPRS Server!",Command);
		SendDataViaTCP(GPRS_sockfd,Command,strlen(Command));
		sleep(1);
		break;
		case 1://发短信命令
		syslog(LOG_INFO|LOG_USER,"SendCommandString:%s to SMS Server!",Command);
		//SendDataViaTCP(SMSfd,Command,strlen(Command));

		SendPostSMS(Target,Command);

		//sleep(1);

		break;
		case 2://发往客户端的命令
		syslog(LOG_INFO|LOG_USER,"SendCommandString:%s TO Client!",Command);
		//SendDataViaTCP(GPRS_sockfd,Command,strlen(Command));
		SendDataToClientByCompanyID_r(Target,Command,strlen(Command));//发送到公司名为TARGET的监控客户端

		break;
		case 3://用飞信发短信
		SendFetionSMS(Target,Command);
		break;

		default:
		syslog(LOG_INFO|LOG_USER,"Get Undefine Command From Database Chanel %d Command:%s!",Channel,Command);


		}


		}*/
		return 0;
	}





	int str2bcd(char * str,char * data,int len)
	{
		//char a[]="0498573";
		char b[3];
		unsigned char a;
		memset(b,0,3);
		int i;
		for (i=0;i<len;i++)
		{
			strncpy(b,str+i*2,2);
			sscanf(b,"%02x",(unsigned int*)&a);
			memset(&data[i],a,1);

		}
		return 0;
	}

	//丽普顿全部上传车辆资料,这么多资料上传，烦死了，自己不会查呀。直接一个字段搞好算了。memo:格式：车牌号码&车牌颜色（0、1、2、3）&联系方式&车辆用途&OMC代码&MDT类型&MDT代码&车辆类型（符合GA24.4要求）&车辆组织机构代码&车辆公司&车辆从业资格证号码&辖区省&辖区市&辖区县&号牌种类（符合GA24.7要求）
	//示例粤E31812&1&13812345678&旅游包车&47591&88123&2322&K24&440442123&测试公司&123432423&广东&佛山&禅城&01
	int lpt_UpdateCarData(int fd,char * bookuserlist)
	{
		char up_info[1000];
		int rscount,i=0;
		MYSQL_ROW getrow;
		MYSQL_RES *RS;
		MYSQL * conn_ptr;
		Connect_Data_Server(&conn_ptr,DataIP,dbuser,dbpwd,
			dbname,DataPort,"Init CarData DataBase Connected!");
		char sql[1024];
		sprintf(sql,"Select memo FROM Car where BookUserList like '%s' order by CarID",bookuserlist);
		syslog(LOG_ERR|LOG_LOCAL0,"%s %d InitCarData sql:%s \n",__FILE__,__LINE__,sql);

		rscount=mysql_query(conn_ptr,sql);
		if (rscount)
		{
			syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d GetCarData Err:%s \n",
				__FILE__,__LINE__,mysql_error(conn_ptr));
			//MysqlUnLock();
			return -1;
		}
		if(!rscount)
		{
			RS=mysql_store_result(conn_ptr);
			rscount=mysql_affected_rows(conn_ptr);
			//MysqlUnLock();
			if ((RS))
			{
				while  ((getrow=mysql_fetch_row(RS))!=NULL)
				{
					sscanf(getrow[0],"%s",up_info);
					lpt_upload_one_car(fd,up_info);
					i++;
				}
				//pthread_mutex_unlock(&CarDataMutex);
				syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d Upload %d CarInfo OK!\n",
					__FILE__,__LINE__,rscount);
				mysql_free_result(RS);

			}
		}
		else
		{
			syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d Read CarInfo Fail[%s]!\n",
				__FILE__,__LINE__,mysql_error(conn_ptr));

		}

		//	pthread_mutex_unlock(&conn_ptrMutex);


		return rscount;



	}
	//从数据库中取一个值。
	int GetValueFromDB(char* sql,char * result)
	{
		int rscount,i=0;
		MYSQL_ROW getrow;
		MYSQL_RES *RS;
		MYSQL * conn_ptr;

		Connect_Data_Server(&conn_ptr,DataIP,dbuser,dbpwd,
			dbname,DataPort,"Init CarData DataBase Connected!");

		rscount=mysql_query(conn_ptr,sql);
		if (rscount)
		{
			syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d InitCarData Err:%s \n",
				__FILE__,__LINE__,mysql_error(conn_ptr));
			//MysqlUnLock();
			mysql_close(conn_ptr);
			return -1;
		}
		if(!rscount)
		{
			RS=mysql_store_result(conn_ptr);
			rscount=mysql_affected_rows(conn_ptr);
			//MysqlUnLock();
			if ((RS))
			{
				while  ((getrow=mysql_fetch_row(RS))!=NULL)
				{
					sscanf(getrow[0],"%s",result);

					i++;
				}
				//pthread_mutex_unlock(&CarDataMutex);
				syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d Read %d CarInfo OK!\n",
					__FILE__,__LINE__,rscount);
				mysql_free_result(RS);

			}
		}
		else
		{
			syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d Read CarInfo Fail[%s]!\n",
				__FILE__,__LINE__,mysql_error(conn_ptr));

		}

		//	pthread_mutex_unlock(&conn_ptrMutex);

		mysql_close(conn_ptr);
		return rscount;



	}
	int InitCarData()
	{
		int rscount,i=0;
		MYSQL_ROW getrow;
		MYSQL_RES *RS;
		MYSQL * conn_ptr;

		/*int Connect_Data_Server(MYSQL** cconn_ptr,char * host,char * username,char * mysqlpassword ,
		char * database,int dbport,char * Con_Prompt)	*/

		Connect_Data_Server(&conn_ptr,DataIP,dbuser,dbpwd,
			dbname,DataPort,"Init CarData DataBase Connected!");

		//char sql[]="Select CarID,CarName,ColorType,DriverName,DriverID FROM CarInfo order by CarID";
		char sql[1024];
		sprintf(sql,"Select CarID,CarNickName,ColorType,DriverName,MainDriverID,CarPhone,ManufacterID,CenterID,ServiceStatu FROM Car where BookUserList like '%s' order by CarID",Booker);
		syslog(LOG_ERR|LOG_LOCAL0,"%s %d InitCarData sql:%s \n",__FILE__,__LINE__,sql);

		rscount=mysql_query(conn_ptr,sql);
		if (rscount)
		{
			syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d InitCarData Err:%s \n",
				__FILE__,__LINE__,mysql_error(conn_ptr));
			//MysqlUnLock();
			return -1;
		}
		if(!rscount)
		{
			RS=mysql_store_result(conn_ptr);
			rscount=mysql_affected_rows(conn_ptr);
			//MysqlUnLock();
			if ((RS))
			{
				while  ((getrow=mysql_fetch_row(RS))!=NULL)
				{
					sscanf(getrow[0],"%s",CarData[i].CarID);
					sscanf(getrow[1],"%s",CarData[i].CarName);
					sscanf(getrow[2],"%d",&CarData[i].Color);
					sscanf(getrow[3],"%s",CarData[i].DriverName);
					sscanf(getrow[4],"%s",CarData[i].DriverID);
					sscanf(getrow[5],"%s",CarData[i].CarPhone);
					sscanf(getrow[6],"%d",&CarData[i].ManufacterID);
					sscanf(getrow[7],"%d",&CarData[i].CenterID);
					sscanf(getrow[8],"%d",&CarData[i].ServiceStatu);
					i++;
				}
				//pthread_mutex_unlock(&CarDataMutex);
				syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d Read %d CarInfo OK!\n",
					__FILE__,__LINE__,rscount);
				mysql_free_result(RS);

			}
		}
		else
		{
			syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d Read CarInfo Fail[%s]!\n",
				__FILE__,__LINE__,mysql_error(conn_ptr));

		}

		//	pthread_mutex_unlock(&conn_ptrMutex);


		return rscount;



	}
	int GetCarIndexFromCarID(char * CarID)
	{
		int i;
		for(i=0;i<3000;i++)
		{
			if (strcmp(CarData[i].CarID,CarID)==0)
			{
				return i;
			}

		}

		return -1;
	}
	//处理省中心下发的申请
	int DoApplyRequest(char *Body,int BodyLen)
	{
		//序号	字段名	长度	数据类型	描述说明
		//1	APPLY_CMD_ID	2 byte	WORD	申请命令类型，见表13
		//2	REGISTRATION_NO	10 byte	STRING	车牌号
		//3	REGISTRATION_NO_COLOR	1 byte	BYTES	车牌颜色，编码对应关系见表3
		//4	CMD_LENGTH	2 byte	WORD	命令长度
		//5	CMD_CONTENT	N byte	BYTES	命令内容（长度由CMD_LENGTH决定）
		if (BodyLen<15)
		{
			syslog(LOG_ERR|LOG_LOCAL0,"%s %d Get Error ApplyRequest len %d!\n",__FILE__,__LINE__,BodyLen);
			return -1;
		}
		char CMD_CONTENT[1024];
		memset(CMD_CONTENT,0,1024);
		unsigned int AppCMD=0,CMD_LENGTH=0;
		AppCMD=(Body[0]<<8)+Body[1];
		char CarName[11];
		memset(CarName,0,11);
		memcpy(CarName,(char*)Body+2,10);
		char Color;
		Color=Body[12];
		CMD_LENGTH=(Body[13]<<8)+Body[14];
		memcpy(CMD_CONTENT,Body+15,CMD_LENGTH);//取内容
		/*#define COMMAND_TYPE_GPS_REAL_FORWORD 0X0001
		#define COMMAND_TYPE_GPS_HISTORY_FORWORD 0X0002

		#define COMMAND_TYPE_CAR_INFO_UPLOAD 0X0003	  //企业平台
		#define COMMAND_TYPE_IMAGE_UPLOAD 0X0004    //企业平台
		#define COMMAND_TYPE_TEXT_SEND 0X0005	 //企业平台
		#define COMMAND_TYPE_LISTEN 0X0006   //企业平台*/
		switch (AppCMD)
		{
		case COMMAND_TYPE_GPS_REAL_FORWORD:
			syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d Get A COMMAND_TYPE_GPS_REAL_FORWORD %d!\n",
				__FILE__,__LINE__,AppCMD);
			break;
		case COMMAND_TYPE_GPS_HISTORY_FORWORD:
			syslog(LOG_ERR|LOG_LOCAL0,
				"\n%s %d Get A COMMAND_TYPE_GPS_HISTORY_FORWORD AppCOMAND %d!\n",
				__FILE__,__LINE__,AppCMD);
			break;
		case COMMAND_TYPE_CAR_INFO_UPLOAD:
			syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d Get A COMMAND_TYPE_CAR_INFO_UPLOAD AppCOMAND %d!\n",
				__FILE__,__LINE__,AppCMD);

			// UploadCarInfo(CarName,Color);
			break;
			//省中心只有向市中心申请图像上传
		case COMMAND_TYPE_IMAGE_UPLOAD:
			DeliveImage(CarName,Color);//回应并递交图像数据
			break;
		case COMMAND_TYPE_TEXT_SEND:
			syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d Get A COMMAND_TYPE_TEXT_SEND AppCOMAND %d!\n",
				__FILE__,__LINE__,AppCMD);

			break;
		case COMMAND_TYPE_LISTEN:
			syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d Get A COMMAND_TYPE_LISTEN AppCOMAND %d!\n",
				__FILE__,__LINE__,AppCMD);
			break;
		default:
			syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d Get A Unknow AppCOMAND %d!\n",
				__FILE__,__LINE__,AppCMD);
		}
		return 1;
	}
	int DeliveImage(char *CarName,char Color)//回应并递交图像数据
	{

		char buf[100];
		memset(buf,0,100);
		//回应申请命令ID
		buf[0]=0;
		buf[1]=4;
		memcpy(&buf[2],CarName,10);
		buf[12]=Color;
		//取图像数据
		//成功
		buf[13]=0;
		buf[14]=0;
		buf[15]=0;
		buf[16]=0;
		char pack[PACKETTOTALLEN];
		DoPacket(APPLY_RSP,GPSCETNTER,buf ,17,pack);
		SendPacket2Center(pack,(pack[1]<<8)+pack[2],Center_sock_fd);
		//递交图像数据(待写)


		return 1;

	}




	int GetCarTrack()
	{
		CenterGPSPacket track;
		memset(&track,0,sizeof(CenterGPSPacket));
		int TrackCount;
		TrackCount=sizeof(CenterGPSPacket);
		int tracklen;
		int rscount,i=0;
		MYSQL_ROW getrow;
		MYSQL_RES *RS;
		MYSQL * conn_ptr;
		char mac_id[20];
		int x,y,speed;
		char gpstime[40];
		char bvalid[20];
		int dir;
		char s1[20],s2[20],s3[20];
		/*int Connect_Data_Server(MYSQL** cconn_ptr,char * host,char * username,char * mysqlpassword ,
		char * database,int dbport,char * Con_Prompt)	*/

		Connect_Data_Server(&conn_ptr,"211.139.255.99","gpsadmin","1qaz2wsx",
			"gps4",8639,"Init CarDataTrack DataBase Connected!");

		char sql[]="select mac_id,x,y,speed,DATE_FORMAT(gpstime,'%Y%m%d%H%i%s'),bvalid,dir,s1,s2,s3 from target where cp9='sjt' and imagefile='1'";
		//char sql[]="Select CarID,CarName,ColorType,DriverName,DriverID FROM CarInfo order by CarID";
		rscount=mysql_query(conn_ptr,sql);
		if (rscount)
		{
			syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d GetCarTrack Err:%s \n",
				__FILE__,__LINE__,mysql_error(conn_ptr));
			//MysqlUnLock();
			return -1;
		}
		if(!rscount)
		{
			RS=mysql_store_result(conn_ptr);
			rscount=mysql_affected_rows(conn_ptr);
			//MysqlUnLock();
			if ((RS))
			{
				i=0;
				while  ((getrow=mysql_fetch_row(RS))!=NULL)
				{//mac_id,x,y,speed,gpstime,bvalid,dir,s1,s2,s3

					//2030163277	67452050	13840147	0	2010-3-26 08:10:19	有效	0	正常	车门关|熄火	正常

					sscanf(getrow[0],"%s",mac_id);
					sscanf(getrow[1],"%d",&x);
					sscanf(getrow[2],"%d",&y);
					sscanf(getrow[3],"%d",&speed);
					sscanf(getrow[4],"%s",gpstime);
					sscanf(getrow[5],"%s",bvalid);
					sscanf(getrow[6],"%d",&dir);
					sscanf(getrow[7],"%s",s1);
					sscanf(getrow[8],"%s",s2);
					sscanf(getrow[9],"%s",s3);
					//生成报文
					syslog(LOG_INFO|LOG_USER,"%s%d no:%d Car GPSData: carid:%s-x:%d-y:%d-speed:%d-gpstime:%s-bvalid:%s-dir:%d-s1:%s-s2:%s-s3%s\n",
						__FILE__,__LINE__,i,mac_id,x,y,speed,gpstime,bvalid,dir,s1,s2,s3);

					tracklen=ChangData2GPSPacket(bvalid,mac_id,gpstime,x,y,speed,dir,s1,s2,s3,0,&track);
					int Carindex=0;
					Carindex=GetCarIndexFromCarID(mac_id);
					if (Carindex<0)
					{
						syslog(LOG_INFO|LOG_USER,"%s%dUnknow Car CarID: %s\n",__FILE__,__LINE__,mac_id);
						continue;
					}
					char CarName[10];
					memset(CarName,0,10);
					sprintf(CarName,"%s",CarData[Carindex].CarName);

					char CarColor;
					CarColor=CarData[Carindex].Color;
					char DeliverData[1024];
					memset(DeliverData,0,1024);

					char CenterData[1024];
					memset(CenterData,0,1024);
					int datalen,CenterDataLen;
					//产生递交数据包
					datalen=DoDeliver(DELIVER_CMD_ID_POSITION,CarName,CarColor,
						sizeof(CenterGPSPacket),(char*)&track,0,DeliverData);

					CenterDataLen=DoPacket(DELIVER,GPSCETNTER,DeliverData,datalen,CenterData);//编码成数据包
					//发送
					if (Center_sock_fd>0)
					{
						SendPacket2Center(CenterData,CenterDataLen,Center_sock_fd);
						syslog(LOG_INFO|LOG_USER,"%s%dSendCarDataTo fd:%d CarID:%s\n",__FILE__,__LINE__,Center_sock_fd,mac_id);

					}
					if (Center_sock_fd1>0)
					{
						SendPacket2Center(CenterData,CenterDataLen,Center_sock_fd1);
						syslog(LOG_INFO|LOG_USER,"%s%dSendCarDataTo fd:%d CarID:%s\n",__FILE__,__LINE__,Center_sock_fd1,mac_id);

					}



					i++;
				}
				sprintf(sql,"update target SET imagefile=0 where cp9='sjt' and imagefile='1'");
				rscount=mysql_query(conn_ptr,sql);
				//pthread_mutex_unlock(&CarDataMutex);
				syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d Read %d CarTrack OK!\n",
					__FILE__,__LINE__,i);
				mysql_free_result(RS);

			}
		}
		else
		{
			syslog(LOG_ERR|LOG_LOCAL0,"\n%s %d Read CarTrack Fail[%s]!\n",
				__FILE__,__LINE__,mysql_error(conn_ptr));

		}

		//	pthread_mutex_unlock(&conn_ptrMutex);

		mysql_close(conn_ptr);
		return rscount;



	}


	int ChangData2GPSPacket(char *Va,char * sCarID,char * strTime,double dx,double dy,
		int speed ,int dir,char * s1,char * s2,char* s3,int fd,CenterGPSPacket * track)
	{


		//定位有效性
		track->VALID=(strstr(Va,"有效")!=NULL)?1:0;
		//车机机ID
		//track->
		//strcpy(CarID,sCarID);
		//pa[7]=2010-01-14
		/*char strTime[40];

		sprintf(strTime,"%s%s%s%s%s%s",ddd[0],ddd[1],ddd[2],ttt[0],ttt[1],ttt[2]);
		(*/
		str2bcd(strTime,track->GPS_TIME,7);

		double x,y;
		x=dx/600000.0;
		y=dy/600000.0;

		char sx[20],sy[20];
		int du;
		int fen;
		int miao;
		double dfen;
		//经度
		du=x;
		fen=(x-du)*60.0;
		dfen=(x-du)*60.0;

		miao=(dfen-fen)*1000;
		memset(sx,0,20);
		sprintf(sx,"%03d%02d%d",du,fen,miao);
		str2bcd(sx,track->LONGTITUDE,4);
		//纬度
		du=y;
		fen=(y-du)*60.0;
		miao=((y-du)*60.0-fen)*1000;
		memset(sy,0,20);
		sprintf(sy,"%03d%02d%d",du,fen,miao);


		str2bcd(sy,track->LATITUDE,4);

		//速度
		sprintf(sy,"%04d",speed);
		str2bcd(sy,track->SPEED,2);
		//方向
		sprintf(sy,"%04d",dir);
		str2bcd(sy,track->DIRECTION,2);

		//坐标方向,中国
		track->sta10_EW=1;
		track->sta11_NS=1;
		//报警状态在pa12
		track->sta41_CrossAlarm=(strstr(s1,"越")!=NULL)?1:0;
		track->sta32_GPSOff=(strstr(s1,"GPS开路")!=NULL)?1:0;


		//车辆状态在pa13
		track->sta25_ACC=(strstr(s2,"点火")!=NULL)?1:0;
		track->sta21_DoorOpen=(strstr(s2,"车门开")!=NULL)?1:0;
		int Carindex=0;
		Carindex=GetCarIndexFromCarID(sCarID);
		if (Carindex>=0)
		{
			memcpy(track->DRIVER_NAME,CarData[Carindex].DriverName,12);
			memcpy(track->DRIVER_IDCARDNO,CarData[Carindex].DriverID,18);
		}

		return sizeof(CenterGPSPacket);
	}
	//车辆资料上报申请
	int DoCarInfoUploadApply(char* CarName,char Color)
	{
		char Data[100];
		DoApply(COMMAND_TYPE_CAR_INFO_UPLOAD,CarName,Color,0,Data,Center_sock_fd);
		return 0;

	}
		//丽普盾平台数据编码，funword
	int lpt_encode_data(char main_key_word,char main_key_code, char sec_key_code,int datalen,char * data,char * packetdata)
	{
		int resultlen=0;
		packetdata[0]=0x7e;
		packetdata[1]=0x2a;
		//序号packetSQE
		packetSQE++;
		packetdata[2]=packetSQE>>8;
		packetdata[3]=packetSQE &0xff;
		//数据长
		packetdata[4]=datalen>>8;
		packetdata[5]=datalen &0xff;
		//主命令
		packetdata[6]=main_key_word;
		//主关键码
		packetdata[7]=main_key_code;
		//次关键码
		packetdata[8]=sec_key_code;
		//预留
		packetdata[9]=0;
		//数据内容
		memcpy(packetdata+10,data,datalen);
		packetdata[datalen+10]=0x23;
		//协议尾[
		resultlen=datalen+11;
		return resultlen;

	}

	int cb_lpt_sendto(int fd,char * packetdata,int len)
	{

		int result;
	    result=send(fd,packetdata,len,0);
		if (result!=len )
			{
            syslog(LOG_ERR|LOG_LOCAL0,"%s %d err:send %s to fd:%d result:%d ,len:%d!system exit!\n",__FILE__,__LINE__,packetdata,fd,result,len);
			exit(0);
			}
        if (DebugMode)
		{
		  char a[1000];
		  memset(a,0,1000);
		  PrintArray(packetdata,len,a);
		  syslog(LOG_ERR|LOG_LOCAL0,"%s %d send %dbytes to fd:%d<%s> result:%d!\n",	__FILE__,__LINE__,len,fd,a,result);
		}

		return result;

	}

	//利普顿上传车辆资料
	//车牌号码&车牌颜色（0、1、2、3）&联系方式&车辆用途&OMC代码&MDT类型&MDT代码&车辆类型（符合GA24.4要求）&车辆组织机构代码&车辆公司&车辆从业资格证号码&辖区省&辖区市&辖区县&号牌种类
	int lpt_upload_one_car(int fd,char * up_info)
	{  //粤E31812&1&13812345678&旅游包车&47591&88123&2322&K24&440442123&测试公司&123432423&广东&佛山&禅城&01
		//车牌号码&车牌颜色（0、1、2、3）&联系方式&车辆用途&OMC代码&MDT类型&MDT代码&车辆类型（符合GA24.4要求）&车辆组织机构代码&车辆公司&车辆从业资格证号码&辖区省&辖区市&辖区县&号牌种类
		//char data[1000];
		int len;
		char packetdata[1000];
		//15
		//sprintf(data,"%s&%d&%s&%s&%s&%s&%s&%s&%s&%s&%s&%s&%s&%s&%s",CarName,color,contact,usefor,OMC,MDT_type,MDT_code,cartype,OG_No,CompanyName,licence_id,province,city,town,id_type);
		len=lpt_encode_data(0xa2,0x11,0,strlen(up_info),up_info,packetdata);


		return cb_lpt_sendto(fd,packetdata,len);
	}
	//利普顿登录
		int lpt_login(char *username,char * password,int fd)
	{
		//int result=0
		int len=0;
		char data[100];
		char packetdata[100];
		sprintf(data,"%s|%s",username,password);

		//int lpt_encode_data(char main_key_word,char main_key_code, char sec_key_code,int datalen,char * data,char * packetdata)
		len=lpt_encode_data(0xa1,1,0,strlen(data),data,packetdata);
		return cb_lpt_sendto(fd,packetdata,len);


	}
	//JT809
		//static unsigned int jt809seq=0;
		//typedef struct _JT809
		//{
		//char head;//头5B
		//char packetlenght[4];//4字节数据包长度
		//char seq[4];//报文序列号
		//char packet_code[2];//2字节数据类型代码0x1200表示上传数据
		//char omc_code[4];//下级平台代码
		//char ver[3];//3字节协议版本号
		//char en_flag;//一字节加密标志，0不加密
		//char key[4];//4字节密匙
		//struct child_msg data;//数据体，长度未知
		//char crc[2];//校验码，两字节
		//char tail;//尾5D
		//}JT809;
		////子业务数据报文
		//typedef struct _child_msg
		//{
		//	char carname[21];//21字节车牌号
		//	char color;//一字节车牌颜色，2表示黄色
		//	char packetcode[2];//2字节子业务类型，0x1202表示上传实时位置
		//	char datalen[4];//后续字节长度，位置是0x24字节
		//	struct GNSS_DATA data;//数据
		//}child_msg;
		////定位数据//36字节
		//typedef struct _GNSS_DATA
		//{
		//	char ecy;//加密标志，0未加密
		//	char day;//日
		//	char month;//月
		//	char year[2];//年
		//	char hour;//时
		//	char minute;//分
		//	char seconds;//秒
		//	char lon[4];//经度*1000000
		//	char lat[4];//纬度*1000000
		//	char gpsspeed[2];//GPS速度
		//	char carspeed[2];//机械速度
		//	char mile[4];//里程公里
		//	char heading[2];//方向
		//	char hight[2];//海拔
		//	char statu[4];//JT808状态位
		//	char alarm[4];//报警状态
		//}GNSS_DATA;
	//JT809数据编码
		/*
	13:29:06 【粤AL1338】ObjidGet fail 91字节
5B
0000005A02
00000054
1200
5605D51E
010000
00
5A025A015C5E01
D4C141483537323900000000000000000000000000
02
1202
00000024

00
050807DF
0A1633
06C2C8C8
01600000
001F001F00000000011F0000000080034000000024F45D
5B0000005A020000AD961200000004D20100000000201025D4C1414C313333380000000000000000000000000002120200000024000C0B07DE0D1E0B06C23880016082200033003300000000001F0000000080034000000056B75D

5B//报头
//以下为数据头
0000005A02//5a02为转义后的5A=90,4字节表示整个数据包长度90(转义前的长度)，实际长度为91
0000AD96//报文序列号4字节
1200//2字节业务数据类型0x1200表示UP_EXG_MSG
000004D2//4字节下级平台接入码
010000//3字节协议版本号1.0.0
00//1字节加密标识，0为不加密
00201025//数据加密密匙4字节

//以下为数据体
D4C1414C3133333800000000000000000000000000//21字节车牌号
02//1字节车牌颜色 02表示黄
1202//两字节子业务类型1202标识，表示实时上传车辆位置UP_EXG_MSG_REAL_LOCATIONG
00000024//后续数据字节长度0x24=36字节
**以下为车辆定位数据部份36字节
00//未加密定位
0C0B07DE//0xC=12日，0B=11月 07DE=2014
0D1E0B//时分秒0D=13 1E=30 0B=11秒
06C23880//113391744/1000000=113.391744
01608220//纬度23101984/1000000=23.101984
0033//卫星速度0x33=51
0033//机械速度
00000000//总里程数(km)
001F//方向31
0000//海拔米
00008003//车辆状态位JT808状态位
40000000//JT808报警状态

56B7//CRC校验码
5D//报尾
	*/



	int JT809_GPS_packet(char * gpsdate,double lon,double lat,int speed,int speed2,int mile, int heading,int height,char* statu,char *alarm,unsigned char * packetdata)
	{
	//	***gnss
	//00 //
	//04 0C 07 DE
	//09 38 36
	//06 C4 30 30
	//01 5B 01 0F
	//0F 00
	//00 00
	//00 00 00 00
	//00 00
	//00 00
	//00 30 30 30 30 30 30

		//GPS卫星定位数据
		packetdata[0]=0;//加密标志0，不加密
		int day,mon,year,hour,minute,second;

		sscanf(gpsdate,"%d-%d-%d %d:%d:%d",&year,&mon,&day,&hour,&minute,&second);
		//日月年四字节
		packetdata[1]=day&0x00ff;
		packetdata[2]=mon&0x00ff;
		packetdata[3]=(year>>8);
		packetdata[4]=(year &0x00FF);
		//时分秒三字节
		packetdata[5]=hour;
		packetdata[6]=minute;
		packetdata[7]=second;
		//4节经度*1000000
		unsigned long longlonlat;
		char hh[100];
		int int1,int2,int3,int4;
		longlonlat=lon*1000000;
		sprintf(hh,"%08X",longlonlat);

		sscanf(hh,"%02x%02x%02x%02x",&int1,&int2,&int3,&int4);
		packetdata[8]=int1;
		packetdata[9]=int2;
		packetdata[10]=int3;
		packetdata[11]=int4;

		/*packetdata[8]=((longlonlat&0xff000000)>>24);
		packetdata[9]=((longlonlat&0xff0000)>>16);
		packetdata[10]=(longlonlat&0xff00>>8);
		packetdata[11]=(longlonlat&0xFF);*/
		//4节纬度*1000000
		longlonlat=lat*1000000;
		/*packetdata[12]=((longlonlat&0xff000000)>>24);
		packetdata[13]=((longlonlat&0xff0000)>>16);
		packetdata[14]=(longlonlat&0xff00>>8);
		packetdata[15]=(longlonlat&0xFF);*/

		sprintf(hh,"%08X",longlonlat);

		sscanf(hh,"%02x%02x%02x%02x",&int1,&int2,&int3,&int4);
		packetdata[12]=int1;
		packetdata[13]=int2;
		packetdata[14]=int3;
		packetdata[15]=int4;
		//2字节GPS速度
		packetdata[16]=((speed&0x00ff00)>>8);
		packetdata[17]=(speed&0x00FF);
		//2字节车速
		packetdata[18]=((speed2&0x00ff00)>>8);
		packetdata[19]=(speed2&0xFF);
		//4字节里程公里
		packetdata[20]=(mile>>24);
		packetdata[21]=((mile&0x00ff0000)>>16);
		packetdata[22]=((mile&0x0000ff00)>>8);
		packetdata[23]=(mile&0xFF);
		//2字节方向
		packetdata[24]=((heading&0x00ff00)>>8);
		packetdata[25]=(heading&0xFF);
		//2字节高度
		packetdata[26]=(height&0x00ff00>>8);
		packetdata[27]=(height&0xFF);
		//四字节状态
	     unsigned int st1,st2,st3,st4;
		 sscanf(statu,"%02X%02X%02X%02X",&st1,&st2,&st3,&st4);
		packetdata[28]=st1;
		packetdata[29]=st2;
		packetdata[30]=st3;
		packetdata[31]=st4;

		 //四字节报警
		 sscanf(alarm,"%02X%02X%02X%02X",&st1,&st2,&st3,&st4);
		packetdata[32]=st1;
		packetdata[33]=st2;
		packetdata[34]=st3;
		packetdata[35]=st4;
		//memcpy(packetdata+28+4,alarm,4);
		return 36;



	}

	/*5B
	00 00 00 58
	00 00 00 01
	12 00
	00 00 00 00
	01 00 00
	00
	5A 02 5B 01 5C 5D 01
	--------gps
	41 44 35 32 39 35 00 32 35 32 31 34 33 36 33 32 30 00 32 30 31 //车牌
	02
	12 02
	00 00 00 24 =36字节长度
	***gnss
	00 //
	04 0C 07 DE
	09 38 36
	06 C4 30 30
	01 5B 01 0F
	0F 00 00 00 00 00 00 00 00 00 00 00 00 30 30 30 30 30 30
	---------
	82 8D
	5D*/
	int JT809_up_gps_track(char * carname,char color,short date_type,char * data,int datalen,char * packetdata)
	{
		//实时位置上传
		//21字节车牌号
		//memcpy(packetdata,carname,21);
		strncpy(packetdata,carname,21);
		//1字节车牌颜色
		packetdata[21]=color;
		//2字节子业务类型标识
		packetdata[22]=(date_type>>8);
		packetdata[23]=(date_type &0x00FF);
		//4字节数据长度
		packetdata[24]=(datalen>>24);
		packetdata[25]=((datalen &0x00FF0000)>>16);
		packetdata[26]=((0X0000FF00&datalen)>>8);
		packetdata[27]=0X000000FF&datalen;
		//gnssdata
		memcpy(packetdata+28,data,datalen);
		return datalen+21+1+2+4;
	}
	int JT809_login_data(int userid,char * password,char * down_link_ip,short down_link_port,char * packetdata)
	{
		//登录生成数据函数，
		//四字节用户名

		packetdata[0]=(userid>>24);
		packetdata[1]=((userid &0x00FF0000)>>16);
		packetdata[2]=((0X0000FF00&userid)>>8);
		packetdata[3]=0X000000FF&userid;
		//8字节密码
		strncpy(packetdata+4,password,8);
		//32字节从链路IP
		strncpy(packetdata+8+4,down_link_ip,32);
		//2字节端口号
		packetdata[44]=(down_link_port>>8);
		packetdata[45]=(down_link_port &0x00FF);
		return 4+8+32+2;
	}
	int JT809_encode_data(int  funword,int  ocm_code, char * data, int datalen,char * packet)
	{
		jt809seq++;
		char packetdata[2000];
		memset(packetdata,0,2000);
		packetdata[0]=0X5B;
		int len=0;
		len+=1;//一字节头5B
		len+=4;//4字节长度4字节表示整个数据包长度
		len+=4;//报文序列号4字节
		len+=2;//2字节业务数据类型0x1200表示UP_EXG_MSG
		len+=4;//4字节下级平台接入码
		len+=3;//3字节协议版本号1.0.0
		len+=1;//1字节加密标识，0为不加密
		len+=4;//数据加密密匙4字节
		len+=datalen;//数据全体长
		len+=2;//2字节校验
		len+=1;//报尾5D
		//5B
		//00 00 00 1A
		// 00 00 00 01
		//10 05
		//00 00 BB CC
		//01 00 00
		//00
		//00 00 00 00
		//0F 1A B5 5B
		//填写长度4字节
		packetdata[1]=(len>>24);
		packetdata[2]=((len &0x00FF0000)>>16);
		packetdata[3]=((0X0000FF00&len)>>8);
		packetdata[4]=0X000000FF&len;
		//填写序列号
		packetdata[5]=((0XFF000000&jt809seq)>>24);
		packetdata[6]=((0X00FF0000&jt809seq)>>16);
		packetdata[7]=((0X0000FF00&jt809seq)>>8);
		packetdata[8]=((0X000000FF&jt809seq)>>0);
		//2字节业务数据类型0x1200表示UP_EXG_MSG
		packetdata[9]=((0X0000FF00&funword)>>8);
		packetdata[10]=(0X0000FF&funword);
		//4字节下级平台接入码
		packetdata[11]=((0XFF000000&ocm_code)>>24);
		packetdata[12]=((0X00FF0000&ocm_code)>>16);
		packetdata[13]=((0X0000FF00&ocm_code)>>8);
		packetdata[14]=((0X000000FF&ocm_code)>>0);
		//3字节协议版本号1.0.0
		packetdata[15]=1;
		packetdata[16]=0;
		packetdata[17]=0;
		//1字节加密标识，0为不加密
		packetdata[18]=0;
		//数据加密密匙4字节
		packetdata[19]=0X5A;
		packetdata[20]=0X5B;
		packetdata[21]=0X5C;
		packetdata[22]=0X5D;
		//数据全体长
		memcpy(packetdata+23,data,datalen);//拷入数据
		//2字节校验,从数据头到校验码前的crc16-ccitt校验
		unsigned short crc=0;
		crc=do_crc(0,(unsigned char *)packetdata,datalen+23);
		//packetdata[datalen+23]=
		packetdata[datalen+23]=((0X0000FF00&crc)>>8);
		packetdata[datalen+24]=((0X000000FF&crc)>>0);
		packetdata[datalen+25]=0X5D;
		//转议
		int i=1;
		int j=1;
		packet[0]=packetdata[0];

		for(j=1;j<datalen+25;j++)
		{
			switch(packetdata[j])
			{
				case 0x5a://5A=5A+02
					packet[i]=0X5A;
					i++;
					packet[i]=0x2;
					break;
				case 0x5b://5B=5A+01
					packet[i]=0X5A;
					i++;
					packet[i]=0x01;
					break;
				case 0x5d://5D=5E+01
					packet[i]=0X5E;
					i++;
					packet[i]=0x01;
					break;
				case 0x5e://5E=5E+02
					packet[i]=0X5E;
					i++;
					packet[i]=0x02;
					break;
				default:
			        packet[i]=packetdata[j];

			}
			i++;
		}

		packet[i]=0x5D;
	  return i+1;
	}
	//长宝平台数据编码，funword
	int cb_encode_data(char * funword,char * ocm_code,int datalen,char * data,char * packetdata)
	{
		int resultlen=0;
		int headlen=0;
		char tmp[1000];
		memset(tmp,0,1000);
		if (datalen>0)
		{
			sprintf(tmp,"~%s&%s&%04d&",funword,ocm_code,datalen);
			headlen=strlen(tmp);
			memcpy(packetdata,tmp,headlen);
			memcpy(packetdata+headlen,data,datalen);//由于数据中可能有0，所以不能用字符串，要拷过去。
			packetdata[headlen+datalen]='#';//加上尾
			resultlen=headlen+datalen+1;
		}
		else
		{
			sprintf(packetdata,"~%s&%s&0000#",funword,ocm_code);
			resultlen=strlen(packetdata);
		}
		return resultlen;

	}
	//长宝登录，输入随机字符串
	int cb_login(char * serial,int fd)
	{
		int result=0;
		char data[100];

		char *ps;
		int len;
		char psstring[100],packetdata[200];
		memset(psstring,0,100);
		sprintf(psstring,"%s%s",CENTERPASSWORD,serial);

		ps=MDString(psstring);
		sprintf(data,"%s|%s|%s",CENTERUSER,ps,serial);

		len=cb_encode_data("L01",ocm_code,strlen(data),data,packetdata);
		if (DebugMode)
		{
		  syslog(LOG_ERR|LOG_LOCAL0,"%s %d send login string %s to fd:%d!\n",	__FILE__,__LINE__,packetdata,fd);
		}

	   // result=send(fd,packetdata,len,0);
		result= cb_lpt_sendto(fd,packetdata,len);
		if (result<len )
			syslog(LOG_ERR|LOG_LOCAL0,"%s %d send %s to fd:%d error  result:%d ,len:%d!\n",	__FILE__,__LINE__,packetdata,fd,result,len);
		return result;
	}
		//丽普盾平台数据处理
	int lpt_decode_data(int fd,char * Data,int len)
	{
		char a[2000];
		  memset(a,0,2000);
		if (DebugMode)
		{

		  PrintArray(Data,len,a);
		  syslog(LOG_ERR|LOG_LOCAL0,"%s %d recive len:%d from  fd:%d<%s>\n",	__FILE__,__LINE__,len,fd,a);
		}
		if (len<11)
		{
			PrintArray(Data,len,a);

			syslog(LOG_ERR|LOG_LOCAL0,"%s %d recive too short len:%d from  fd:%d<%s>\n",	__FILE__,__LINE__,len,fd,a);
			return 0;
		}
		int contenlen=0;
		contenlen=(Data[4]<<8)+Data[5];
		char content[100];
		//7E 2A 00 04 00 00 A2 01 00 00 23
		memset(content,0,100);
		memcpy(content,Data+10,contenlen);
		char msg[100];
		char packet[1000];
		int packetlen=0;
		char main_key=Data[7];//主关键字
		unsigned char  key=Data[6];
	   if (DebugMode)
		syslog(LOG_ERR|LOG_LOCAL0,"%s %d key:%x main_key:%x contentlen:%d content:%s\n",	__FILE__,__LINE__,key,main_key,contenlen,content);
		switch(key)
		{

		 case 0xa1://登录回应7E 2A 00 01 00 04 A1 01 00 00 30 30 30 30 23
			if (main_key==0x21)
			{
			 break;//心跳回应，无需处理
			}
			lpt_show_login(atoi(content));
		    break;
		 case 0xa2://要求所有车辆资料上传
			 lpt_UpdateCarData(fd,Booker);
			break;
		case 0xa3://信息下发
			lpt_send_msg(fd,content,contenlen);//下发信息
			break;
		case 0xa4://参数

			break;
		case 0xa5://相片
			sprintf(msg,"%s|01",content);
			//int lpt_encode_data(char main_key_word,char main_key_code, char sec_key_code,int datalen,char * data,char * packetdata)
			packetlen=lpt_encode_data(0xa5,1,0,strlen(msg),msg,packet);
			cb_lpt_sendto(fd,packet,packetlen);
			break;
		case 0xa6://统计
			break;
		default:
		syslog(LOG_ERR|LOG_LOCAL0,"%s %d key:%d main_key:%d contentlen:%d content:%s\n",__FILE__,__LINE__,key,main_key,contenlen,content);

		}
return 1;
}
	int lpt_show_login(int no)
	{
		if (DebugMode)
		{


		  syslog(LOG_ERR|LOG_LOCAL0,"%s %d login code:%d\n",	__FILE__,__LINE__,no);
		}

		switch(no)
			{
			case 0:
				syslog(LOG_ERR|LOG_LOCAL0,"%s %d login sucess!\n",	__FILE__,__LINE__);
				break;
			case 1:
				syslog(LOG_ERR|LOG_LOCAL0,"%s %d login fail,invalid username!\n",	__FILE__,__LINE__);
				break;
			case 2:
				syslog(LOG_ERR|LOG_LOCAL0,"%s %d login fail,invalid password!\n",	__FILE__,__LINE__);
				break;
			case 3:
                syslog(LOG_ERR|LOG_LOCAL0,"%s %d login refuse,invalid ip!\n",	__FILE__,__LINE__);
				break;
			case 4:
                syslog(LOG_ERR|LOG_LOCAL0,"%s %d login refuse,refuse username!\n",	__FILE__,__LINE__);
				break;
			case 5:
                syslog(LOG_ERR|LOG_LOCAL0,"%s %d login refuse,user name exsist!\n",	__FILE__,__LINE__);
				break;
			case 6:
                syslog(LOG_ERR|LOG_LOCAL0,"%s %d login refuse,user time out!\n",	__FILE__,__LINE__);
				break;
			case 7:
                syslog(LOG_ERR|LOG_LOCAL0,"%s %d please disconnect,user login in other ip\n",__FILE__,__LINE__);
				break;
			default:
				syslog(LOG_ERR|LOG_LOCAL0,"%s %d login fail,error no:%d \n",__FILE__,__LINE__,no);
			}

	return 1;
	}
	//丽普顿下发信息
	int lpt_send_msg(int fd,char* Data,int contentlen)//下发信息
	{
		char  * pa[50];
		int ind=0;
		char  *p;
		p=strtok(Data,"|");
		pa[0]=p;
		while((p=strtok(NULL,"|")))
		{
			ind ++;
			if ((ind>0 )&& (ind<50))
			{
				pa[ind]=p;
			}
		}

		if (ind<2)
		{
			syslog(LOG_ERR|LOG_LOCAL0,"%s %d fd %d error packet index:%d<3!\n",	__FILE__,__LINE__,fd,ind);
		}

		//0:自定义标识 1：车牌号码&颜色 2：内容
		char CarID[100];
		char cmd[100];
		char sql[100];
		int color;
		//[DB44|0123456789|21|今天天气真好！]
		char CarName[100];
		char content[100];
		sscanf(pa[1],"%s&%d",CarName,&color);
		sprintf(sql,"select CarID from Car Where CarNickName='%s'",CarName);
		if (GetValueFromDB(sql,CarID))
		{
		sprintf(cmd,"[DB44|%s|21|%s]",CarID,pa[2]);
		send(GPS_sock_fd,cmd,strlen(cmd),0);
		sprintf(cmd,"%s|%s&%d",pa[0],CarName,color);
		contentlen=lpt_encode_data(0xa3,0x01, 0,strlen(cmd),cmd,content);

		}
		else
		{
		 sprintf(cmd,"%s|%s&%d",pa[0],CarName,color);
         contentlen=lpt_encode_data(0xa3,0x01, 01,strlen(cmd),cmd,content);

		}
		cb_lpt_sendto(fd,content,contentlen);//返回服务器
		return 0;
	}

		int sdjtj_decode_data(int fd,char * Data,int len)
	{//~L00&&0010&gS6N4gg1Vq#^
		int i=0;
		char msg[1000];
		//char b[100];
	   if (len>1000)
		   len=1000;

          if (DebugMode)
		{

			memset(msg,0,1000);
			for(i=0;i<len;i++)
			{
				sprintf(msg,"%s %02X", msg,Data[i]);
			}

			syslog(LOG_ERR|LOG_LOCAL0,"%s %d recive from sdjtj_center Len:%d fd:%d data:%s \n",__FILE__,__LINE__,len,fd,msg);
		  }

		for(i=0;i<len;i++)//转成字符0
		{
		if (Data[i]==0)
		    Data[i]='0';
		}

		char  * pa[50];
		int ind=0;
		char  *p;
		p=strtok(Data,"~&#");
		pa[0]=p;
		while((p=strtok(NULL,"~&#")))
		{
			ind ++;
			if ((ind>0 )&& (ind<50))
			{
				pa[ind]=p;
			}
		}

		if (ind<2)
		{
			syslog(LOG_ERR|LOG_LOCAL0,"%s %d fd %d error packet index:%d<3!\n",	__FILE__,__LINE__,fd,ind);
		}
		char ocm_code[20];
		int data_len;//数据
		//功能关键字0，OCM代码1，数长度2，数据字段3
		//判断功能关键字1
		char str_erro[20];//返回结果
		strcpy(ocm_code,pa[1]);
		data_len=atoi(pa[2]);//数据长度
		if (strcmp(pa[0],"T02")==0)//服务器心跳
		{

			syslog(LOG_ERR|LOG_LOCAL0,"%s %d center_heart recive\n",	__FILE__,__LINE__);
			return 1;
		}

		if (strcmp(pa[0],"L00")==0)//随机字段，一个数据字段，10字节
		{
			syslog(LOG_ERR|LOG_LOCAL0,"%s %d login_str:%s\n",	__FILE__,__LINE__,pa[3]);
			cb_login(pa[3],fd);//用随机字符串登录

			return 1;
		}

		if (strcmp(pa[0],"L02")==0)//登录状态回应
		{
			cb_loginstatu=0;
			switch(atoi(pa[3]))
			{
			case 0:
				sprintf(str_erro,"%s","login success");
				cb_loginstatu=1;//登录成功
				break;
			case 1:
				sprintf(str_erro,"%s","invalid datapacket");
				break;
			case 2:
				sprintf(str_erro,"%s","invalid data type");
				break;
			case 3:
				sprintf(str_erro,"%s","invalid user name");
				break;
			case 4:
				sprintf(str_erro,"%s","invalid password");
				break;
			case 5:
				sprintf(str_erro,"%s","apply refuse,string erro");
				break;
			case 6:
				sprintf(str_erro,"%s","login refuse,invalid ip or ocm code");
				break;
			default:
				sprintf(str_erro,"%s","login fail,unknown error");

			}
			syslog(LOG_ERR|LOG_LOCAL0,"%s %d return code %s,login_result:%s!\n",	__FILE__,__LINE__,pa[3],str_erro);
			if (!cb_loginstatu)//登录不成功，退出程序。
			{
				syslog(LOG_ERR|LOG_LOCAL0,"%s %d log_in fail,program exit!\n",	__FILE__,__LINE__);
				exit(0);
			}

		}





return 1;


	}
	//长宝JT809平台数据处理
	int cb_decode_data(int fd,char * Data,int len)
	{//~L00&&0010&gS6N4gg1Vq#^

		int i=0;
		char msg[1000];
		//char b[100];
	   if (len>1000)
		   len=1000;

          if (DebugMode)
		{

			memset(msg,0,1000);
			for(i=0;i<len;i++)
			{
				sprintf(msg,"%s %02X", msg,Data[i]);
			}

			syslog(LOG_ERR|LOG_LOCAL0,"%s %d recive from cb_center Len:%d fd:%d data:%s \n",__FILE__,__LINE__,len,fd,msg);
		  }

		for(i=0;i<len;i++)//转成字符0
		{
		if (Data[i]==0)
		    Data[i]='0';
		}

		char  * pa[50];
		int ind=0;
		char  *p;
		p=strtok(Data,"~&#");
		pa[0]=p;
		while((p=strtok(NULL,"~&#")))
		{
			ind ++;
			if ((ind>0 )&& (ind<50))
			{
				pa[ind]=p;
			}
		}

		if (ind<2)
		{
			syslog(LOG_ERR|LOG_LOCAL0,"%s %d fd %d error packet index:%d<3!\n",	__FILE__,__LINE__,fd,ind);
		}
		char ocm_code[20];
		int data_len;//数据
		//功能关键字0，OCM代码1，数长度2，数据字段3
		//判断功能关键字1
		char str_erro[20];//返回结果
		strcpy(ocm_code,pa[1]);
		data_len=atoi(pa[2]);//数据长度
		if (strcmp(pa[0],"T02")==0)//服务器心跳
		{

			syslog(LOG_ERR|LOG_LOCAL0,"%s %d center_heart recive\n",	__FILE__,__LINE__);
			return 1;
		}

		if (strcmp(pa[0],"L00")==0)//随机字段，一个数据字段，10字节
		{
			syslog(LOG_ERR|LOG_LOCAL0,"%s %d login_str:%s\n",	__FILE__,__LINE__,pa[3]);
			cb_login(pa[3],fd);//用随机字符串登录

			return 1;
		}

		if (strcmp(pa[0],"L02")==0)//登录状态回应
		{
			cb_loginstatu=0;
			switch(atoi(pa[3]))
			{
			case 0:
				sprintf(str_erro,"%s","login success");
				cb_loginstatu=1;//登录成功
				break;
			case 1:
				sprintf(str_erro,"%s","invalid datapacket");
				break;
			case 2:
				sprintf(str_erro,"%s","invalid data type");
				break;
			case 3:
				sprintf(str_erro,"%s","invalid user name");
				break;
			case 4:
				sprintf(str_erro,"%s","invalid password");
				break;
			case 5:
				sprintf(str_erro,"%s","apply refuse,string erro");
				break;
			case 6:
				sprintf(str_erro,"%s","login refuse,invalid ip or ocm code");
				break;
			default:
				sprintf(str_erro,"%s","login fail,unknown error");

			}
			syslog(LOG_ERR|LOG_LOCAL0,"%s %d return code %s,login_result:%s!\n",	__FILE__,__LINE__,pa[3],str_erro);
			if (!cb_loginstatu)//登录不成功，退出程序。
			{
				syslog(LOG_ERR|LOG_LOCAL0,"%s %d log_in fail,program exit!\n",	__FILE__,__LINE__);
				exit(0);
			}

		}





return 1;


	}
	//每一分钟发送一次心跳
	int cb_send_test_link(int fd)
	{
		//"T01"int cb_encode_data(char * funword,char * ocm_code,int datalen,char * data,char * packetdata)
		char packet[100];
		char data[10];
		int len,result;
		memset(packet,0,100);
		len=cb_encode_data("T01",ocm_code,0,data,packet);
		if (len>5)
	    result=send(fd,packet,15,0);
		//result=cb_lpt_sendto(fd,packet,len);
		syslog(LOG_ERR|LOG_LOCAL0,"%s %d cb_send_test_link<%s> \n",	__FILE__,__LINE__,packet);
		if (result!=len)
			syslog(LOG_ERR|LOG_LOCAL0,"%s %d cb_send_test_link send fail !\n",	__FILE__,__LINE__);

		return result;

	}
	//每一分钟发送一次心跳
	int lpt_send_test_link(int fd)
	{
		//int lpt_encode_data(char main_key_word,char main_key_code, char sec_key_code,int datalen,char * data,char * packetdata)
		char packet[100];
		//char data[10];
		int len,result;
		len=lpt_encode_data(0xa1,0x21,0x00,0,0,packet);
		cb_lpt_sendto(fd,packet,len);
		return result;

	}



