/* sudo apt-get install ros-indigo-navigation */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "geometry_msgs/Twist.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <semaphore.h>

//#define DISP_SERIAL_DATA

#define TOPIC_SUBSCRIBE_MOVING    "cmd_vel"
#define TOPIC_SUBSCRIBE_ARM       "cmd_arm"
#define TOPIC_PUBLISH             "Odom"

#define DEFAULT_BAUDRATE          115200
#define DEFAULT_SERIALPORT        "/dev/ttyS1"  //"/dev/ttyUSB0"

#define HEX2ASCII(a)              ((a)>9?(a)-10+'A':(a)+'0')

#define PI                        (3.1415926)
#define WHEEL_DIAMETER            (0.067)       // m
#define WHEEL_TICKS               (1980.0)      // ticks/cricle
#define BODY_WIDTH                (0.16)        // m
#define GEARRATIO                 (WHEEL_TICKS/(PI*WHEEL_DIAMETER))
#define UPDATE_TIME               (0.2)

#define PACKET_SHIFT_CHAR         0xFF
#define PACKET_HEAD_CHAR          0xFA
#define PACKET_TAIL_CHAR          0xFB

const char PACKET_HEAD[] = {PACKET_SHIFT_CHAR,PACKET_HEAD_CHAR};
const char PACKET_TAIL[] = {PACKET_SHIFT_CHAR,PACKET_TAIL_CHAR};

typedef union
{
  uint8_t u8[4];
  int32_t i32;
  uint32_t u32;
}U32_I32_U8;

//Global data
int32_t fd_serial = -1;
ros::Publisher  ucPositionMsg;
ros::Subscriber ucMovingCtrlMsg;
ros::Subscriber ucArmCtrlMsg;

struct timespec ts = {.tv_sec=0,.tv_nsec = 1e8};
sem_t sem_operation;

uint8_t u8SerialCmdStop[] = {PACKET_SHIFT_CHAR,PACKET_HEAD_CHAR,0x0A,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0B,PACKET_SHIFT_CHAR,PACKET_TAIL_CHAR};

//Initialize serial port, return file descriptor
static int32_t serialInit(char * port, int baud)
{
  int BAUD = 0;
  int32_t fd;
  struct termios newtio;

  //Open the serial port as a file descriptor for low level configuration
  // read/write, not controlling terminal for process,
  fd = open(port, O_RDWR | O_NOCTTY );//| O_NDELAY );
  if(fd < 0)
  {
    ROS_ERROR("serialInit: Could not open serial device %s",port);
    return fd;
  }
  // set up new settings
  memset(&newtio, 0,sizeof(newtio));

  newtio.c_cflag |= CLOCAL | CREAD; 
  newtio.c_cflag &= ~CSIZE; 
  newtio.c_cflag |= CS8;
  newtio.c_cflag &= ~PARENB;

  newtio.c_cflag &=  ~CSTOPB;

  newtio.c_cflag &= ~CRTSCTS; 

  newtio.c_cc[VTIME]  = 1;
  newtio.c_cc[VMIN] = 0;

  // activate new settings
  tcflush(fd, TCIFLUSH);
  //Look up appropriate baud rate constant
  switch (baud)
  {
    case 115200ul:
    default:
      BAUD = B115200;
      break;
    case 57600ul:
      BAUD = B57600;
      break;
    case 38400:
      BAUD = B38400;
      break;
    case 19200:
      BAUD  = B19200;
      break;
    case 9600:
      BAUD  = B9600;
      break;
    case 4800:
      BAUD  = B4800;
      break;
    case 2400:
      BAUD  = B2400;
      break;
    case 1800:
      BAUD  = B1800;
      break;
    case 1200:
      BAUD  = B1200;
      break;
  }  //end of switch baud_rate
  if (cfsetispeed(&newtio, BAUD) < 0 || cfsetospeed(&newtio, BAUD) < 0)
  {
    ROS_ERROR("serialInit: Failed to set serial baud rate: %d", baud);
    close(fd);
    return -1;
  }
  
  tcsetattr(fd, TCSANOW, &newtio);
  tcflush(fd, TCIOFLUSH);
  return fd;
} //serialInit

static uint8_t u8CheckXor(uint8_t *pu8Data,uint32_t u32DataLen)
{
  uint8_t u8CheckXorTemp = 0;
  uint32_t u32Cnt = 0;
  for(u32Cnt = 0;u32Cnt < u32DataLen;u32Cnt++)
    u8CheckXorTemp ^= *(pu8Data+u32Cnt);
  return u8CheckXorTemp;
}

static uint32_t u32PacketShift(uint8_t *pu8DataBuff,uint32_t u32DataLen,uint32_t u32DataBuffSize)
{
  uint8_t *pu8OpearMem = (uint8_t*)malloc(u32DataBuffSize);
  uint32_t u32Src,u32Dst;
  if(pu8OpearMem == NULL)
    return 0;
  for(u32Src = 0,u32Dst = 0;u32Src < u32DataLen;u32Src++)
  {
    if(*(pu8DataBuff+u32Src)==0xFF)
    {
      if(u32Dst < u32DataBuffSize-1)
      {
        *(pu8OpearMem + u32Dst) = 0xFF;
        *(pu8OpearMem + u32Dst + 1) = 0xFF;
        u32Dst += 2;
      }else
        break;
    }else
    {
      if(u32Dst < u32DataBuffSize)
      {
        *(pu8OpearMem + u32Dst) = *(pu8DataBuff+u32Src);
        u32Dst ++;
      }else
        break;
    }
  }
  if(u32Src == u32DataLen)
  {
    memcpy(pu8DataBuff,pu8OpearMem,u32Dst);
    free(pu8OpearMem);
    return u32Dst;
  }else
  {
    free(pu8OpearMem);
    return 0;
  }
}

void ucMovingCtrlMsgCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  char cCmdBuff[32];
  uint8_t u8CmdBuffCnt;
  U32_I32_U8 s32Temp;
  double linear_velocity_ticks,angular_velocity_ticks;
  double fLTicks,fRTicks;
  int32_t iRet;

#ifdef DISP_SERIAL_DATA
  uint8_t u8SrcCnt,u8DstCnt;
  char cCmdBuffDisp[128];  // 3*cCmdBuff
#endif
  ROS_INFO("ucMovingCtrlMsg: Linear:X:%lf,Y:%lf,Z:%lf Angular:X:%lf,Y:%lf,Z:%lf",
  msg->linear.x,msg->linear.y,msg->linear.z,
  msg->angular.x,msg->angular.y,msg->angular.z);

  linear_velocity_ticks = GEARRATIO*msg->linear.x;
  angular_velocity_ticks = BODY_WIDTH*GEARRATIO/2*msg->angular.z;

  fLTicks = linear_velocity_ticks + angular_velocity_ticks;
  fRTicks = linear_velocity_ticks - angular_velocity_ticks;
  
  cCmdBuff[0] = 0x0A;
  cCmdBuff[1] = 0x01;
  ///////////////////////////////////////////////
  // LSpeed
  if(fLTicks > 2000)
    s32Temp.i32 = 2000;
  else if(fLTicks > 500)
    s32Temp.i32 = fLTicks;
  else if(fLTicks >= 0.5)
    s32Temp.i32 = 500;
  else if(fLTicks >= -0.5)
    s32Temp.i32 = 0;
  else if(fLTicks > -500)
    s32Temp.i32 = -500;
  else if(fLTicks >= -2000)
    s32Temp.i32 = fLTicks;
  else
    s32Temp.i32 = -2000;
  ///////////////////////////////////////////////
  cCmdBuff[2] = s32Temp.u8[3];
  cCmdBuff[3] = s32Temp.u8[2];
  cCmdBuff[4] = s32Temp.u8[1];
  cCmdBuff[5] = s32Temp.u8[0];
  ///////////////////////////////////////////////
  // RSpeed
  if(fRTicks > 2000)
    s32Temp.i32 = 2000;
  else if(fRTicks > 500)
    s32Temp.i32 = fRTicks;
  else if(fRTicks >= 0.5)
    s32Temp.i32 = 500;
  else if(fRTicks >= -0.5)
    s32Temp.i32 = 0;
  else if(fRTicks > -500)
    s32Temp.i32 = -500;
  else if(fRTicks >= -2000)
    s32Temp.i32 = fRTicks;
  else
    s32Temp.i32 = -2000;
  ///////////////////////////////////////////////
  cCmdBuff[6] = s32Temp.u8[3];
  cCmdBuff[7] = s32Temp.u8[2];
  cCmdBuff[8] = s32Temp.u8[1];
  cCmdBuff[9] = s32Temp.u8[0];

  cCmdBuff[10] = u8CheckXor((uint8_t*)cCmdBuff,*(cCmdBuff));
  u8CmdBuffCnt = 11;

  u8CmdBuffCnt = u32PacketShift((uint8_t*)cCmdBuff,u8CmdBuffCnt,sizeof(cCmdBuff));

  if((u8CmdBuffCnt != 0) && (sem_timedwait(&sem_operation,&ts) == 0))
  {
    write(fd_serial,(void*)PACKET_HEAD,sizeof(PACKET_HEAD));
    write(fd_serial,(void*)cCmdBuff,u8CmdBuffCnt);
    write(fd_serial,(void*)PACKET_TAIL,sizeof(PACKET_TAIL));
    sem_post(&sem_operation);
  }
#ifdef DISP_SERIAL_DATA
  for(u8SrcCnt = 0,u8DstCnt=0;u8SrcCnt < u8CmdBuffCnt;u8SrcCnt++)
  {
    cCmdBuffDisp[u8DstCnt] = HEX2ASCII(cCmdBuff[u8SrcCnt]>>4);
    cCmdBuffDisp[u8DstCnt+1] = HEX2ASCII(cCmdBuff[u8SrcCnt]&0xF);
    cCmdBuffDisp[u8DstCnt+2] = ' ';
    u8DstCnt+=3;
  }
  cCmdBuffDisp[u8DstCnt] = '\0';
  u8DstCnt++;
  ROS_INFO("Send Original Serial Data[%s]",cCmdBuffDisp);
#endif
} //ucCommandCallback 

void ucArmCtrlMsgCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
  char cCmdBuff[32];
  uint8_t u8CmdBuffCnt;
  U32_I32_U8 s32Temp;
  int32_t iRet;

#ifdef DISP_SERIAL_DATA
  uint8_t u8SrcCnt,u8DstCnt;
  char cCmdBuffDisp[128];  // 3*cCmdBuff
#endif
  ROS_INFO("ucArmCtrlMsg: X:%lf,Y:%lf,Z:%lf",msg->x,msg->y,msg->z);
  //
  cCmdBuff[0] = 0x0A;
  cCmdBuff[1] = 0x02;
  ///////////////////////////////////////////////
  s32Temp.i32 = msg->z * 180.0 / PI;
  if(s32Temp.i32 > 60)
    s32Temp.i32 = 60;
  else if(s32Temp.i32 < -60)
    s32Temp.i32 = -60;
  cCmdBuff[2] = s32Temp.u8[3];
  cCmdBuff[3] = s32Temp.u8[2];
  cCmdBuff[4] = s32Temp.u8[1];
  cCmdBuff[5] = s32Temp.u8[0];
  ///////////////////////////////////////////////
  s32Temp.i32 = msg->x * 180.0 / PI;
  if(s32Temp.i32 > 60)
    s32Temp.i32 = 60;
  else if(s32Temp.i32 < -60)
    s32Temp.i32 = -60;
  cCmdBuff[6] = s32Temp.u8[3];
  cCmdBuff[7] = s32Temp.u8[2];
  cCmdBuff[8] = s32Temp.u8[1];
  cCmdBuff[9] = s32Temp.u8[0];

  cCmdBuff[10] = u8CheckXor((uint8_t*)cCmdBuff,*(cCmdBuff));
  u8CmdBuffCnt = 11;

  u8CmdBuffCnt = u32PacketShift((uint8_t*)cCmdBuff,u8CmdBuffCnt,sizeof(cCmdBuff));

  if((u8CmdBuffCnt != 0) && (sem_timedwait(&sem_operation,&ts) == 0))
  {
    write(fd_serial,(void*)PACKET_HEAD,sizeof(PACKET_HEAD));
    write(fd_serial,(void*)cCmdBuff,u8CmdBuffCnt);
    write(fd_serial,(void*)PACKET_TAIL,sizeof(PACKET_TAIL));
    sem_post(&sem_operation);
  }
#ifdef DISP_SERIAL_DATA
  for(u8SrcCnt = 0,u8DstCnt=0;u8SrcCnt < u8CmdBuffCnt;u8SrcCnt++)
  {
    cCmdBuffDisp[u8DstCnt] = HEX2ASCII(cCmdBuff[u8SrcCnt]>>4);
    cCmdBuffDisp[u8DstCnt+1] = HEX2ASCII(cCmdBuff[u8SrcCnt]&0xF);
    cCmdBuffDisp[u8DstCnt+2] = ' ';
    u8DstCnt+=3;
  }
  cCmdBuffDisp[u8DstCnt] = '\0';
  u8DstCnt++;
  ROS_INFO("Send Original Serial Data[%s]",cCmdBuffDisp);
#endif
}

//Receive command responses from robot uController
//and publish as a ROS message
void rcvThread(void)
{
  char cSerialRecvBuff[32];
  char cTemp = 0;
  uint8_t u8SerialRecvCnt = 0;
  uint8_t u8RecvStatus = 0;
  int32_t iRet;

  U32_I32_U8 s32Temp;
  uint32_t u32SN;
  int32_t i32LSpeed,i32RSpeed;
  int32_t i32LMileage,i32RMileage;

  static double x_=0.0,y_=0.0,th_=0.0;
  double vx_=0.0,vy_=0.0,vth_=0.0;

#ifdef DISP_SERIAL_DATA
  uint8_t u8SrcCnt,u8DstCnt;
#endif
  char cCmdBuffDisp[128];  // 3*cSerialRecvBuff

  tf::TransformBroadcaster odom_broadcaster_; 

  while (ros::ok())
  {
    iRet = read(fd_serial,&cTemp,sizeof(char));
    if(iRet == 1)
    {
      if(u8RecvStatus == 0)
      {
        if(cTemp == PACKET_SHIFT_CHAR)
          u8RecvStatus = 1;
      }else if(u8RecvStatus == 1)
      {
        if(cTemp == PACKET_HEAD_CHAR)
        {
          u8RecvStatus = 2;
          u8SerialRecvCnt = 0;
        }else
          u8RecvStatus = 0;
      }else if(u8RecvStatus == 2)
      {
        if(cTemp == PACKET_SHIFT_CHAR)
          u8RecvStatus = 3;
        else
        {
          cSerialRecvBuff[u8SerialRecvCnt] = cTemp;
          u8SerialRecvCnt++;
        }
      }else if(u8RecvStatus == 3)
      {
        if(cTemp == PACKET_SHIFT_CHAR)
        {
          u8RecvStatus = 2;
          cSerialRecvBuff[u8SerialRecvCnt] = PACKET_SHIFT_CHAR;
          u8SerialRecvCnt++;
        }else if(cTemp == PACKET_TAIL_CHAR)
        { // Packet End
#ifdef DISP_SERIAL_DATA
          for(u8SrcCnt = 0,u8DstCnt=0;u8SrcCnt < u8SerialRecvCnt;u8SrcCnt++)
          {
            cCmdBuffDisp[u8DstCnt] = HEX2ASCII(cSerialRecvBuff[u8SrcCnt]>>4);
            cCmdBuffDisp[u8DstCnt+1] = HEX2ASCII(cSerialRecvBuff[u8SrcCnt]&0xF);
            cCmdBuffDisp[u8DstCnt+2] = ' ';
            u8DstCnt+=3;
          }
          cCmdBuffDisp[u8DstCnt] = '\0';
          u8DstCnt++;
          ROS_INFO("Recv Original Serial Data:%s",cCmdBuffDisp);
#endif
          if((*cSerialRecvBuff == u8SerialRecvCnt-1)&&
            (u8CheckXor((uint8_t*)cSerialRecvBuff,*cSerialRecvBuff)==
            *(cSerialRecvBuff+u8SerialRecvCnt-1)))
          {
            // Packet Opeartion
            if((cSerialRecvBuff[0] == 0x16)&&(cSerialRecvBuff[1]==0x02))
            {
              // SN
              s32Temp.u8[3]=cSerialRecvBuff[2];
              s32Temp.u8[2]=cSerialRecvBuff[3];
              s32Temp.u8[1]=cSerialRecvBuff[4];
              s32Temp.u8[0]=cSerialRecvBuff[5];
              u32SN=s32Temp.u32;
              // Speed_L
              s32Temp.u8[3]=cSerialRecvBuff[6];
              s32Temp.u8[2]=cSerialRecvBuff[7];
              s32Temp.u8[1]=cSerialRecvBuff[8];
              s32Temp.u8[0]=cSerialRecvBuff[9];
              i32LSpeed=s32Temp.i32;
              // Mileage_L
              s32Temp.u8[3]=cSerialRecvBuff[10];
              s32Temp.u8[2]=cSerialRecvBuff[11];
              s32Temp.u8[1]=cSerialRecvBuff[12];
              s32Temp.u8[0]=cSerialRecvBuff[13];
              i32LMileage=s32Temp.u32;
              if(i32LSpeed<0)
                i32LMileage = 0 - i32LMileage;
              // Speed_R
              s32Temp.u8[3]=cSerialRecvBuff[14];
              s32Temp.u8[2]=cSerialRecvBuff[15];
              s32Temp.u8[1]=cSerialRecvBuff[16];
              s32Temp.u8[0]=cSerialRecvBuff[17];
              i32RSpeed=s32Temp.i32;
              // Mileage_R
              s32Temp.u8[3]=cSerialRecvBuff[18];
              s32Temp.u8[2]=cSerialRecvBuff[19];
              s32Temp.u8[1]=cSerialRecvBuff[20];
              s32Temp.u8[0]=cSerialRecvBuff[21];
              i32RMileage=s32Temp.u32;
              if(i32RSpeed<0)
                i32RMileage = 0 - i32RMileage;

              ROS_INFO("SN:%d,Speed_L:%d,Speed_R:%d,Mileage_L:%d,Mileage_R:%d",
                u32SN,i32LSpeed,i32RSpeed,i32LMileage,i32RMileage);
              //--------------------------------------
              vx_= (i32LMileage+i32RMileage)/GEARRATIO/UPDATE_TIME/2;
              vth_ = (i32RMileage-i32LMileage)/GEARRATIO/BODY_WIDTH/UPDATE_TIME;
              
              double dt=0.2;
              double delta_x=vx_*cos(th_)*dt;
              double delta_y=vx_*sin(th_)*dt;  
              double delta_th = vth_*dt;
              x_+=delta_x;
              y_+=delta_y;
              th_+=delta_th;
              ros::Time curr_time = ros::Time::now();

              geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);
              geometry_msgs::TransformStamped odom_trans;

              odom_trans.header.stamp = curr_time;
              odom_trans.header.frame_id = "odom";
              odom_trans.child_frame_id = "base_link";

              odom_trans.transform.translation.x = x_;
              odom_trans.transform.translation.y = y_;
              odom_trans.transform.translation.z = 0.0;
              odom_trans.transform.rotation = odom_quat;
              //send the transform
              odom_broadcaster_.sendTransform(odom_trans);

              nav_msgs::Odometry odom;
              odom.header.stamp = curr_time;
              odom.header.frame_id = "odom";
              //set the position  
              odom.pose.pose.position.x = x_;
              odom.pose.pose.position.y = y_;
              odom.pose.pose.position.z = 0.0;
              odom.pose.pose.orientation = odom_quat;
              //set the velocity
              odom.child_frame_id = "base_link";
              odom.twist.twist.linear.x = vx_;
              odom.twist.twist.linear.y = vy_;
              odom.twist.twist.angular.z = vth_;
              //publish the message  
              ucPositionMsg.publish(odom);
            }
          }
          u8SerialRecvCnt = 0;
          u8RecvStatus = 0;
        }else // Packet Error
          u8RecvStatus = 0;
      }
    }
    ros::spinOnce();
  }
} //rcvThread

int main(int argc, char **argv)
{
  char port[20];  //port namegeometry_msgs/Twist.msg
  int baud;       //baud rate

  pthread_t rcvThrID;   //receive thread ID
  int err;

  ROS_INFO("cos(PI):%f,sin(PI):%f",cos(PI),sin(PI));
  //Initialize ROS
  ros::init(argc, argv, "moving_ctrl");
  ros::NodeHandle rosNode;
  ROS_INFO("moving_ctrl_node starting");

  strcpy(port, DEFAULT_SERIALPORT);
  if (argc > 1)
    strcpy(port, argv[1]);

  baud = DEFAULT_BAUDRATE;
  if (argc > 2)
  {
    if(sscanf(argv[2],"%d", &baud)!=1)
    {
      ROS_ERROR("ucontroller baud rate parameter invalid");
      return 1;
    }
  }

  sem_init(&sem_operation,0,1);

  ROS_INFO("connection initializing (%s) at %d baud", port, baud);
  fd_serial = serialInit(port, baud);
  if(!fd_serial)
  {
    ROS_ERROR("unable to create a new serial port");
    return -1;
  }
  ROS_INFO("serial connection successful");

  ucMovingCtrlMsg = rosNode.subscribe(TOPIC_SUBSCRIBE_MOVING, 0, ucMovingCtrlMsgCallback);
  ucArmCtrlMsg = rosNode.subscribe(TOPIC_SUBSCRIBE_ARM, 0, ucArmCtrlMsgCallback);
  ucPositionMsg = rosNode.advertise<nav_msgs::Odometry>(TOPIC_PUBLISH, 100);

  rcvThread();

  close(fd_serial);
  ROS_INFO("moving_ctrl_node stopping");
  return 0;
}
