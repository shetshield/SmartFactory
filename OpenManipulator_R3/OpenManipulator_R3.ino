#include <DynamixelSDK.h>
#include <math.h>

// Control table address (MX-series with Protocol 2.0)
#define OPERATING_MODE                  11
#define HOMING_OFFSET                   20
#define ADDR_TORQUE_ENABLE              64                 // Control table address is different in Dynamixel model
#define ADDR_GOAL_VELOCITY              104
#define ADDR_PROFILE_ACCELERATION       108
#define ADDR_PROFILE_VELOCITY           112
#define ADDR_GOAL_POSITION              116
#define ADDR_PRESENT_POSITION           132
#define M_PI                            3.141592

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL0                            0                   // 그리퍼 구동 모터
#define DXL1                            1                   // 그리퍼 -1 모터
#define DXL2                            2                   // 그리퍼 -2 모터
#define DXL3                            3                   // 베이스 +1 모터
#define DXL4                            4                   // 베이스 모터
#define BAUDRATE                        57600
#define DEVICENAME                      "3"                 //DEVICENAME "1" -> Serial1(OpenCM9.04 DXL TTL Ports)
                                                            //DEVICENAME "2" -> Serial2
                                                            //DEVICENAME "3" -> Serial3(OpenCM 485 EXP)
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0                   // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4000                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

// Operating Mode
uint8_t OperMode       = 5; // 전류기반 위치제어 모드 - Current Regulation for girpper
double theta3_0        = -53.50; // in degrees 51.58
double theta2_0        = -39.50; // in degrees 38.42
double theta1_0        = 103.00; // in degress

double init_deg2       = 28.36;
double init_deg3       = -118.36;
double init_deg4       = 103.0;

// Inverse Kinematics Parameter
int32_t Goal_Pos[3]    = {0, 0, 0};
double Link2           = 76.5;
double Link3           = 130.0;
double Link4           = 124.0;
double LinkE           = 146.6;
double Link_12_4       = 0.0;
// Point Location
double P0[3]           = {0.0, 0.0, 0.0};
double P12[3]          = {0.0, 0.0, 76.5};
double P3[3]           = {0.0, 0.0, 0.0};
double P4[3]           = {0.0, 0.0, 0.0};

double rad2deg         = 57.2958;
double deg2rad         = M_PI/180.0;

double q1, q2, q3, q4, q1_, q2_, q3_, q4_, alpha, beta, gamma_,  XE_i, YE_i, ZE_i, XE_f, YE_f, ZE_f, cur_x, cur_y, cur_theta_xy;
int cmd1, cmd2, cmd3, cmd4;
double change_length = 5.0;
double cam_Loc       = 70.0;
char msg;

double r11 = cos(3.0*deg2rad);
double r12 = sin(3.0*deg2rad);
double r21 = -sin(3.0*deg2rad);
double r22 = cos(3.0*deg2rad);

double r11_ = cos(1.5*deg2rad);
double r12_ = sin(1.5*deg2rad);
double r21_ = -sin(1.5*deg2rad);
double r22_ = cos(1.5*deg2rad);

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);;
  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position

  uint8_t dxl_error   = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  int32_t dxl0_present_position = 0; int32_t dxl1_present_position = 0; int32_t dxl2_present_position = 0; int32_t dxl3_present_position = 0; int32_t dxl4_present_position = 0;
  
  int32_t dxl0_PresH_Offset = 0; int32_t dxl0_SetH_Offset  = 0; int32_t dxl0_DefaultPos = 0; int32_t dxl0_GoalPos = 0;
  int32_t dxl1_PresH_Offset = 0; int32_t dxl1_SetH_Offset  = 0; int32_t dxl1_DefaultPos = 0; int32_t dxl1_GoalPos = 0;
  int32_t dxl2_PresH_Offset = 0; int32_t dxl2_SetH_Offset  = 0; int32_t dxl2_DefaultPos = 0; int32_t dxl2_GoalPos = 0;
  int32_t dxl3_PresH_Offset = 0; int32_t dxl3_SetH_Offset  = 0; int32_t dxl3_DefaultPos = 0; int32_t dxl3_GoalPos = 0;
  int32_t dxl4_PresH_Offset = 0; int32_t dxl4_SetH_Offset  = 0; int32_t dxl4_DefaultPos = 0; int32_t dxl4_GoalPos = 0;

  // Dinamixel Array 생성
  int32_t dxl_id[5]           = {DXL0, DXL1, DXL2, DXL3, DXL4};
  int32_t dxl_PresH_Offset[5] = {dxl0_PresH_Offset, dxl1_PresH_Offset, dxl2_PresH_Offset, dxl3_PresH_Offset, dxl4_PresH_Offset};
  int32_t dxl_SetH_Offset[5]  = {dxl0_SetH_Offset, dxl1_SetH_Offset, dxl2_SetH_Offset, dxl3_SetH_Offset, dxl4_SetH_Offset};
  int32_t dxl_Default_Pos[5]  = {dxl0_DefaultPos, dxl1_DefaultPos, dxl2_DefaultPos, dxl3_DefaultPos, dxl4_DefaultPos};
  int32_t dxl_GoalPos[5]      = {dxl0_GoalPos, dxl0_GoalPos, dxl0_GoalPos, dxl0_GoalPos, dxl0_GoalPos};
  int32_t dxl_PresPos[5]      = {dxl0_present_position, dxl1_present_position, dxl2_present_position, dxl3_present_position, dxl4_present_position};
  int32_t dxl_ProfileVel[5]   = {0, 0, 0, 0, 0};
  int32_t dxl_ProfileAcc[5]   = {0, 0, 0, 0, 0};
  
  // Tick to Degree 정의
  double tick = 0.088;
  
  // Open port
  if (portHandler->openPort())
  {
  }
  else
  {
    return;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
  }
  else
  {
    return;
  }

  // 모터 토크 오프
  for (int i = 0; i < 5; i++)
  {
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }
    delay(250);
  }
  
  // 동작모드 변경
  for (int i = 0; i < 5; i++)
  {
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], OPERATING_MODE, OperMode, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }
  }
  
  // 현재 Home Offset 값 수정
  for (int i = 0; i < 5; i++)
  {
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], HOMING_OFFSET, 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->getTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->getRxPacketError(dxl_error);
    }
    else
    {
    }
  }
menu_start :
  switch(wait_until_key_pressed())
  {
    case '0' :
    {
      /*
       *  Home Position Re-Setting
       */
        for (int i = 0; i < 5; i++)
        {
          dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], ADDR_PROFILE_VELOCITY, 10, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }
        }
        for (int i = 0; i < 5; i++)
        {
          dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], ADDR_PROFILE_ACCELERATION, 300, &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }
        }
        /*
         *  Profile Velocity Read
         */
        Serial.println("Profile Velocity");
        for (int i = 0; i < 5; i++)
        {
          dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PROFILE_VELOCITY, (uint32_t*)&dxl_ProfileVel[i], &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }
        }
        Serial.println("Profile Acceleration");
        for (int i = 0; i < 5; i++)
        {
          dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PROFILE_ACCELERATION, (uint32_t*)&dxl_ProfileAcc[i], &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }
        }
        for (int i = 0; i < 5; i++)
        {
          Serial.print("Profile Vel["); Serial.print(String(i)); Serial.print("] "); Serial.print(dxl_ProfileVel[i]); Serial.print(" Profile Acc["); Serial.print(String(i)); Serial.print("] "); Serial.println(dxl_ProfileAcc[i]); 
        }
       for (int i = 0; i < 5; i++)
      {
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[i], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
      }
      // 현재 Home Offset 값 Read
      for (int i = 0; i < 5; i++)
      {
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], HOMING_OFFSET, (uint32_t*)&dxl_PresH_Offset[i], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
        else
        {
        }
      }
      // 홈 오프셋 값 계산
      for (int i = 0; i < 5; i++)
      {
        dxl_SetH_Offset[i] = (-1) * dxl_PresPos[i];
      }      
      // 디버깅
      for (int i = 0; i < 5; i++)
      {
        Serial.print("dxl"); Serial.print(String(i)); Serial.print(" "); Serial.print(dxl_PresPos[i]); Serial.print("; ");  Serial.println(dxl_SetH_Offset[i]);
      }
      // Home Offset 값 수정
      for (int i = 0; i < 5; i++)
      {
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], HOMING_OFFSET, dxl_SetH_Offset[i], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
        else
        {
        }
      }
      // Torque On & Off
      for (int i = 0; i < 5; i++)
      {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
        delay(250);
      }
      for (int i = 0; i < 5; i++)
      {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
        delay(250);
      }
    }
    Serial.println("D");
    goto menu_start;
    case '1' :
    { 
      /*
       *  현재 Configuration 의 Tick 값을 보는 용도
       */
      // Torque On & Off
      for (int i = 0; i < 5; i++)
      {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
        delay(250);
      }
      for (int i = 0; i < 5; i++)
      {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
        delay(250);
      }

      while(1)
      {
        // 현재 위치 Read
        for (int i = 0; i < 5; i++)
        {
          dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[i], &dxl_error);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }
        }
        // 키 입력 변화를 읽음
        if(Serial.available()>0)
        {
          char inkey = Serial.read();
          if (inkey != '1')
          {
            break;
          }
        }
        for (int i = 0; i < 5; i++)
        {
          if (i != 4)
          {
            Serial.print("dxl"); Serial.print(String(i)); Serial.print(" "); Serial.print(dxl_PresPos[i]); Serial.print(" ");  
          }
          else
          {
            Serial.print("dxl"); Serial.print(String(i)); Serial.print(" "); Serial.println(dxl_PresPos[i]);  
          }
        }
      }
    }
    goto menu_start;

    case '2' :
    {
      /*
       *  Torque On
       */
      for (int i = 0; i < 5; i++)
      {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
        delay(250);
      }
      /*
       *  Read Present Position
       */
      for (int i = 0; i < 5; i++)
      {
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[i], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
      }
      dxl0_GoalPos   = 0; dxl1_GoalPos = int(theta1_0/tick); dxl2_GoalPos = int(theta2_0/tick); dxl3_GoalPos = int(theta3_0/tick); dxl4_GoalPos = 0;
      // Forward Kinematics Part
      double theta0_r = 0.0;              // J17
      double theta1_r = deg2rad*(28.36 + 53.50); // J18
      double theta2_r = deg2rad*(-28.36 - 53.50); // J19
      double theta3_r = 0.0; // J20
      XE_i = ForwardKinematicsX(theta0_r, theta1_r, theta2_r, theta3_r);
      YE_i = ForwardKinematicsY(theta0_r, theta1_r, theta2_r, theta3_r);
      ZE_i = ForwardKinematicsZ(theta0_r, theta1_r, theta2_r, theta3_r);

      dxl_GoalPos[0] = dxl0_GoalPos; dxl_GoalPos[1] = dxl1_GoalPos; dxl_GoalPos[2] = dxl2_GoalPos; dxl_GoalPos[3] = dxl3_GoalPos; dxl_GoalPos[4] = dxl4_GoalPos;
      for (int i = 4; i > -1; i--)
      {
        Serial.print("Goal Position "); Serial.print("dxl"); Serial.print(String(i)); Serial.print(" "); Serial.print(dxl_GoalPos[i]); Serial.print(" ; "); Serial.println(dxl_PresPos[i]);
      }
      for (int i = 4; i > -1; i--)
      {
        Serial.println(String(i));
        // Go to Each Goal
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], ADDR_GOAL_POSITION, dxl_GoalPos[i], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
        // Check
        do
        {
          dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[i], &dxl_error);
          // Serial.println(dxl_PresPos[i]);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }
        }while((abs(dxl_GoalPos[i]- dxl_PresPos[i]) > DXL_MOVING_STATUS_THRESHOLD));
        delay(500);
      }
      Serial.println("D");
    }
    goto menu_start;

    case '3' :
    {
      /*
       *  Inverse Kinematics
       */
      for (int i = 0; i < 5; i++)
      {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
        delay(250);
      }
      /*
       *  Read Present Position
       */
      for (int i = 0; i < 5; i++)
      {
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[i], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
      }

      int input = 0;
      Serial.println("3개의 Target Position 입력, in mm");
      for (int i = 0; i < 3; i++)
      {
        input = E_EPos();
        Goal_Pos[i] = double(input);
        Serial.println(Goal_Pos[i]);
      }
      for (int i = 0; i < 3; i++)
      {
        if (i != 2)
        {
          P4[i] = Goal_Pos[i];
        }
        else
        {
          P4[i] = Goal_Pos[i] + LinkE;
        }
      }

      double _x = Goal_Pos[0];
      double _y = Goal_Pos[1];
      q1_       = atan(_y/_x) * rad2deg;
      Link_12_4 = sqrt(pow(P4[0], 2) + pow(P4[1],2) + pow(P4[2] - Link2, 2));
      alpha     = asin((P4[2] - Link2)/Link_12_4) * rad2deg;
      beta      = acos((pow(Link3, 2) + pow(Link_12_4, 2) - pow(Link4, 2))/(2*Link3*Link_12_4)) * rad2deg;
      gamma_    = acos((pow(Link4, 2) + pow(Link_12_4, 2) - pow(Link3, 2))/(2*Link4*Link_12_4)) * rad2deg;
      q2_       = alpha + beta;
      q3_       = beta + gamma_;
      q4_       = 90.0 - q3_ + q2_;

      q1        = q1_;
      q2        = -(q2_ - init_deg2);
      q3        = q3_ + init_deg3;
      q4        = q4_ + init_deg4;

      cmd1      = int(q1/tick);
      cmd2      = int(q2/tick);
      cmd3      = int(q3/tick);
      cmd4      = int(q4/tick);
      /*
      Serial.println(Link_12_4); Serial.println(alpha); Serial.println(beta);
      Serial.println(q1_); Serial.println(q2_); Serial.println(q3_); Serial.println(q4_);
      Serial.println(q1); Serial.println(q2); Serial.println(q3); Serial.println(q4);
      */

      dxl_GoalPos[0] = 1600; dxl_GoalPos[1] = cmd4; dxl_GoalPos[2] = cmd3; dxl_GoalPos[3] = cmd2; dxl_GoalPos[4] = cmd1;
      for (int i = 4; i > -1; i--)
      {
        Serial.print("Goal Position "); Serial.print("dxl"); Serial.print(String(i)); Serial.print(" "); Serial.print(dxl_GoalPos[i]); Serial.print(" ; "); Serial.println(dxl_PresPos[i]);
      }
      for (int i = 4; i > -1; i--)
      {
        Serial.println(String(i));
        // Go to Each Goal
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], ADDR_GOAL_POSITION, dxl_GoalPos[i], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
        // Check
        do
        {
          dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[i], &dxl_error);
          // Serial.println(dxl_PresPos[i]);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }
        }while((abs(dxl_GoalPos[i]- dxl_PresPos[i]) > DXL_MOVING_STATUS_THRESHOLD));
      }
      Serial.println("DONE");
    }
    goto menu_start;
 case '4' :
    {
      /*
       *  Inverse Kinematics Change Location
       */
      for (int i = 0; i < 5; i++)
      {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
        delay(250);
      }
      /*
       *  Read Present Position
       */
      for (int i = 0; i < 5; i++)
      {
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[i], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
      }

      int input = 0;
      Serial.println("현재 End Effector Position");
      Serial.print("x: "); Serial.print(Goal_Pos[0]); Serial.print(" y: "); Serial.print(Goal_Pos[1]); Serial.print(" z: "); Serial.println(Goal_Pos[2]);
      Serial.println("변경할 x 좌표 입력, in mm");
      for (int i = 0; i < 3; i++)
      {
        input = E_EPos();
        Goal_Pos[i] = double(input);
        Serial.println(Goal_Pos[i]);
      }
      for (int i = 0; i < 3; i++)
      {
        if (i != 2)
        {
          P4[i] = Goal_Pos[i];
        }
        else
        {
          P4[i] = Goal_Pos[i] + LinkE;
        }
      }

      double _x = Goal_Pos[0];
      double _y = Goal_Pos[1];
      q1_       = atan(_y/_x) * rad2deg;
      Link_12_4 = sqrt(pow(P4[0], 2) + pow(P4[1],2) + pow(P4[2] - Link2, 2));
      alpha     = asin((P4[2] - Link2)/Link_12_4) * rad2deg;
      beta      = acos((pow(Link3, 2) + pow(Link_12_4, 2) - pow(Link4, 2))/(2*Link3*Link_12_4)) * rad2deg;
      gamma_    = acos((pow(Link4, 2) + pow(Link_12_4, 2) - pow(Link3, 2))/(2*Link4*Link_12_4)) * rad2deg;
      q2_       = alpha + beta;
      q3_       = beta + gamma_;
      q4_       = 90.0 - q3_ + q2_;

      q1        = q1_;
      q2        = -(q2_ - init_deg2);
      q3        = q3_ + init_deg3;
      q4        = q4_ + init_deg4;

      cmd1      = int(q1/tick);
      cmd2      = int(q2/tick);
      cmd3      = int(q3/tick);
      cmd4      = int(q4/tick);
      /*
      Serial.println(Link_12_4); Serial.println(alpha); Serial.println(beta);
      Serial.println(q1_); Serial.println(q2_); Serial.println(q3_); Serial.println(q4_);
      Serial.println(q1); Serial.println(q2); Serial.println(q3); Serial.println(q4);
      */

      dxl_GoalPos[0] = dxl0_GoalPos; dxl_GoalPos[1] = cmd4; dxl_GoalPos[2] = cmd3; dxl_GoalPos[3] = cmd2; dxl_GoalPos[4] = cmd1;
      for (int i = 4; i > -1; i--)
      {
        Serial.print("Goal Position "); Serial.print("dxl"); Serial.print(String(i)); Serial.print(" "); Serial.print(dxl_GoalPos[i]); Serial.print(" ; "); Serial.println(dxl_PresPos[i]);
      }
      for (int i = 4; i > -1; i--)
      {
        Serial.println(String(i));
        // Go to Each Goal
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], ADDR_GOAL_POSITION, dxl_GoalPos[i], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
        // Check
        do
        {
          dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[i], &dxl_error);
          // Serial.println(dxl_PresPos[i]);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }
        }while((abs(dxl_GoalPos[i]- dxl_PresPos[i]) > DXL_MOVING_STATUS_THRESHOLD));
      }
      Serial.println("DONE");
    }
    goto menu_start;
  case '5' :
    {
      /*
       *  Task Space Initialize
       */
      for (int i = 0; i < 5; i++)
      {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
        delay(250);
      }
      /*
       *  Read Present Position
       */
      for (int i = 0; i < 5; i++)
      {
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[i], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
      }
      // Define Task Initial Position
      Goal_Pos[0] = 10; Goal_Pos[1] = 130; Goal_Pos[2] = 90;
      for (int i = 0; i < 3; i++)
      {
        if (i != 2)
        {
          P4[i] = Goal_Pos[i];
        }
        else
        {
          P4[i] = Goal_Pos[i] + LinkE;
        }
      }

      double _x = Goal_Pos[0];
      double _y = Goal_Pos[1];
      q1_       = atan(_y/_x) * rad2deg;
      Link_12_4 = sqrt(pow(P4[0], 2) + pow(P4[1],2) + pow(P4[2] - Link2, 2));
      alpha     = asin((P4[2] - Link2)/Link_12_4) * rad2deg;
      beta      = acos((pow(Link3, 2) + pow(Link_12_4, 2) - pow(Link4, 2))/(2*Link3*Link_12_4)) * rad2deg;
      gamma_    = acos((pow(Link4, 2) + pow(Link_12_4, 2) - pow(Link3, 2))/(2*Link4*Link_12_4)) * rad2deg;
      q2_       = alpha + beta;
      q3_       = beta + gamma_;
      q4_       = 90.0 - q3_ + q2_;

      q1        = q1_;
      q2        = -(q2_ - init_deg2);
      q3        = q3_ + init_deg3;
      q4        = q4_ + init_deg4;

      cmd1      = int(q1/tick);
      cmd2      = int(q2/tick);
      cmd3      = int(q3/tick);
      cmd4      = int(q4/tick);
      /*
      Serial.println(Link_12_4); Serial.println(alpha); Serial.println(beta);
      Serial.println(q1_); Serial.println(q2_); Serial.println(q3_); Serial.println(q4_);
      Serial.println(q1); Serial.println(q2); Serial.println(q3); Serial.println(q4);
      */

      dxl_GoalPos[0] = 0; dxl_GoalPos[1] = cmd4; dxl_GoalPos[2] = cmd3; dxl_GoalPos[3] = cmd2; dxl_GoalPos[4] = cmd1;
      for (int i = 4; i > -1; i--)
      {
        Serial.print("Goal Position "); Serial.print("dxl"); Serial.print(String(i)); Serial.print(" "); Serial.print(dxl_GoalPos[i]); Serial.print(" ; "); Serial.println(dxl_PresPos[i]);
      }
      for (int i = 4; i > -1; i--)
      {
        Serial.println(String(i));
        // Go to Each Goal
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], ADDR_GOAL_POSITION, dxl_GoalPos[i], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
        // Check
        do
        {
          dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[i], &dxl_error);
          // Serial.println(dxl_PresPos[i]);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }
        }while((abs(dxl_GoalPos[i]- dxl_PresPos[i]) > DXL_MOVING_STATUS_THRESHOLD));
      }
      Serial.println("D");
    }
    goto menu_start;
  case '6' :
    {
      /*
       *  Task Space Initialize
       */
       Serial.println("D");
       msg = wait_until_key_pressed();
       Serial.println(msg);
       if (msg == 'x')
       {
          // x 센터는 맞고, y 센터가 안 맞는데, y가 중심점보다 안쪽에 있는 경우
          cur_x = Goal_Pos[0]; cur_y = Goal_Pos[1];
          // Goal_Pos Update
          cur_theta_xy = atan(cur_y/cur_x);
          Goal_Pos[0] = cur_x - change_length * cos(cur_theta_xy); Goal_Pos[1] = cur_y - change_length * sin(cur_theta_xy);
          for (int i = 0; i < 5; i++)
          {
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
              packetHandler->getTxRxResult(dxl_comm_result);
            }
            else if (dxl_error != 0)
            {
              packetHandler->getRxPacketError(dxl_error);
            }
            delay(250);
          }
          /*
           *  Read Present Position
           */
          for (int i = 0; i < 5; i++)
          {
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[i], &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
              packetHandler->getTxRxResult(dxl_comm_result);
            }
            else if (dxl_error != 0)
            {
              packetHandler->getRxPacketError(dxl_error);
            }
          }
    
          for (int i = 0; i < 3; i++)
          {
            if (i != 2)
            {
              P4[i] = Goal_Pos[i];
            }
            else
            {
              P4[i] = Goal_Pos[i] + LinkE;
            }
          }

          double _x = Goal_Pos[0];
          double _y = Goal_Pos[1];
          q1_       = atan(_y/_x) * rad2deg;
          Link_12_4 = sqrt(pow(P4[0], 2) + pow(P4[1],2) + pow(P4[2] - Link2, 2));
          alpha     = asin((P4[2] - Link2)/Link_12_4) * rad2deg;
          beta      = acos((pow(Link3, 2) + pow(Link_12_4, 2) - pow(Link4, 2))/(2*Link3*Link_12_4)) * rad2deg;
          gamma_    = acos((pow(Link4, 2) + pow(Link_12_4, 2) - pow(Link3, 2))/(2*Link4*Link_12_4)) * rad2deg;
          q2_       = alpha + beta;
          q3_       = beta + gamma_;
          q4_       = 90.0 - q3_ + q2_;
    
          q1        = q1_;
          q2        = -(q2_ - init_deg2);
          q3        = q3_ + init_deg3;
          q4        = q4_ + init_deg4;
    
          cmd1      = int(q1/tick);
          cmd2      = int(q2/tick);
          cmd3      = int(q3/tick);
          cmd4      = int(q4/tick);
          /*
          Serial.println(Link_12_4); Serial.println(alpha); Serial.println(beta);
          Serial.println(q1_); Serial.println(q2_); Serial.println(q3_); Serial.println(q4_);
          Serial.println(q1); Serial.println(q2); Serial.println(q3); Serial.println(q4);
          */
    
          dxl_GoalPos[0] = dxl0_GoalPos; dxl_GoalPos[1] = cmd4; dxl_GoalPos[2] = cmd3; dxl_GoalPos[3] = cmd2; dxl_GoalPos[4] = cmd1;
          for (int i = 4; i > -1; i--)
          {
            Serial.print("Goal Position "); Serial.print("dxl"); Serial.print(String(i)); Serial.print(" "); Serial.print(dxl_GoalPos[i]); Serial.print(" ; "); Serial.println(dxl_PresPos[i]);
          }
          for (int i = 4; i > -1; i--)
          {
            Serial.println(String(i));
            // Go to Each Goal
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], ADDR_GOAL_POSITION, dxl_GoalPos[i], &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
              packetHandler->getTxRxResult(dxl_comm_result);
            }
            else if (dxl_error != 0)
            {
              packetHandler->getRxPacketError(dxl_error);
            }
            // Check
            do
            {
              dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[i], &dxl_error);
              // Serial.println(dxl_PresPos[i]);
              if (dxl_comm_result != COMM_SUCCESS)
              {
                packetHandler->getTxRxResult(dxl_comm_result);
              }
              else if (dxl_error != 0)
              {
                packetHandler->getRxPacketError(dxl_error);
              }
              }while((abs(dxl_GoalPos[i]- dxl_PresPos[i]) > DXL_MOVING_STATUS_THRESHOLD));
            }
            Serial.println("D");
       }
       else if (msg == 'z')
       {
          // Terminate
          cur_x = Goal_Pos[0]; cur_y = Goal_Pos[1]; Goal_Pos[2] = 30;
          // Goal_Pos Update
          cur_theta_xy = atan(cur_y/cur_x);
          Goal_Pos[0] = cur_x + cam_Loc * cos(cur_theta_xy); Goal_Pos[1] = cur_y + cam_Loc* sin(cur_theta_xy);
          for (int i = 0; i < 5; i++)
          {
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
              packetHandler->getTxRxResult(dxl_comm_result);
            }
            else if (dxl_error != 0)
            {
              packetHandler->getRxPacketError(dxl_error);
            }
            delay(250);
          }
          /*
           *  Read Present Position
           */
          for (int i = 0; i < 5; i++)
          {
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[i], &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
              packetHandler->getTxRxResult(dxl_comm_result);
            }
            else if (dxl_error != 0)
            {
              packetHandler->getRxPacketError(dxl_error);
            }
          }
    
          for (int i = 0; i < 3; i++)
          {
            if (i != 2)
            {
              P4[i] = Goal_Pos[i];
            }
            else
            {
              P4[i] = Goal_Pos[i] + LinkE;
            }
          }

          double _x = Goal_Pos[0];
          double _y = Goal_Pos[1];
          q1_       = atan(_y/_x) * rad2deg;
          Link_12_4 = sqrt(pow(P4[0], 2) + pow(P4[1],2) + pow(P4[2] - Link2, 2));
          alpha     = asin((P4[2] - Link2)/Link_12_4) * rad2deg;
          beta      = acos((pow(Link3, 2) + pow(Link_12_4, 2) - pow(Link4, 2))/(2*Link3*Link_12_4)) * rad2deg;
          gamma_    = acos((pow(Link4, 2) + pow(Link_12_4, 2) - pow(Link3, 2))/(2*Link4*Link_12_4)) * rad2deg;
          q2_       = alpha + beta;
          q3_       = beta + gamma_;
          q4_       = 90.0 - q3_ + q2_;
    
          q1        = q1_;
          q2        = -(q2_ - init_deg2);
          q3        = q3_ + init_deg3;
          q4        = q4_ + init_deg4;
    
          cmd1      = int(q1/tick);
          cmd2      = int(q2/tick);
          cmd3      = int(q3/tick);
          cmd4      = int(q4/tick);
          /*
          Serial.println(Link_12_4); Serial.println(alpha); Serial.println(beta);
          Serial.println(q1_); Serial.println(q2_); Serial.println(q3_); Serial.println(q4_);
          Serial.println(q1); Serial.println(q2); Serial.println(q3); Serial.println(q4);
          */
    
          dxl_GoalPos[0] = 750; dxl_GoalPos[1] = cmd4; dxl_GoalPos[2] = cmd3; dxl_GoalPos[3] = cmd2; dxl_GoalPos[4] = cmd1;
          for (int i = 4; i > -1; i--)
          {
            Serial.print("Goal Position "); Serial.print("dxl"); Serial.print(String(i)); Serial.print(" "); Serial.print(dxl_GoalPos[i]); Serial.print(" ; "); Serial.println(dxl_PresPos[i]);
          }
          for (int i = 4; i > 2; i--)
          {
            Serial.println(String(i));
            // Go to Each Goal
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], ADDR_GOAL_POSITION, dxl_GoalPos[i], &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
              packetHandler->getTxRxResult(dxl_comm_result);
            }
            else if (dxl_error != 0)
            {
              packetHandler->getRxPacketError(dxl_error);
            }
            // Check
            do
            {
              dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[i], &dxl_error);
              // Serial.println(dxl_PresPos[i]);
              if (dxl_comm_result != COMM_SUCCESS)
              {
                packetHandler->getTxRxResult(dxl_comm_result);
              }
              else if (dxl_error != 0)
              {
                packetHandler->getRxPacketError(dxl_error);
              }
              }while((abs(dxl_GoalPos[i]- dxl_PresPos[i]) > DXL_MOVING_STATUS_THRESHOLD));
            }
            delay(100);
          for (int i = 1; i < 3 ; i++)
          {
            Serial.println(String(i));
            // Go to Each Goal
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], ADDR_GOAL_POSITION, dxl_GoalPos[i], &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
              packetHandler->getTxRxResult(dxl_comm_result);
            }
            else if (dxl_error != 0)
            {
              packetHandler->getRxPacketError(dxl_error);
            }
            // Check
            do
            {
              dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[i], &dxl_error);
              // Serial.println(dxl_PresPos[i]);
              if (dxl_comm_result != COMM_SUCCESS)
              {
                packetHandler->getTxRxResult(dxl_comm_result);
              }
              else if (dxl_error != 0)
              {
                packetHandler->getRxPacketError(dxl_error);
              }
              }while((abs(dxl_GoalPos[i]- dxl_PresPos[i]) > DXL_MOVING_STATUS_THRESHOLD));
            }
            // Move 0 Motor
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[0], ADDR_GOAL_POSITION, dxl_GoalPos[0], &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
              packetHandler->getTxRxResult(dxl_comm_result);
            }
            else if (dxl_error != 0)
            {
              packetHandler->getRxPacketError(dxl_error);
            }
            // Check
            do
            {
              dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[0], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[0], &dxl_error);
              // Serial.println(dxl_PresPos[i]);
              if (dxl_comm_result != COMM_SUCCESS)
              {
                packetHandler->getTxRxResult(dxl_comm_result);
              }
              else if (dxl_error != 0)
              {
                packetHandler->getRxPacketError(dxl_error);
              }
              }while((abs(dxl_GoalPos[0]- dxl_PresPos[0]) > DXL_MOVING_STATUS_THRESHOLD));
            
          // 그대로 들어올림
          Goal_Pos[2] = 90;
          for (int i = 0; i < 3; i++)
          {
            if (i != 2)
            {
              P4[i] = Goal_Pos[i];
            }
            else
            {
              P4[i] = Goal_Pos[i] + LinkE;
            }
          }
          _x = Goal_Pos[0];
          _y = Goal_Pos[1];
          q1_       = atan(_y/_x) * rad2deg;
          Link_12_4 = sqrt(pow(P4[0], 2) + pow(P4[1],2) + pow(P4[2] - Link2, 2));
          alpha     = asin((P4[2] - Link2)/Link_12_4) * rad2deg;
          beta      = acos((pow(Link3, 2) + pow(Link_12_4, 2) - pow(Link4, 2))/(2*Link3*Link_12_4)) * rad2deg;
          gamma_    = acos((pow(Link4, 2) + pow(Link_12_4, 2) - pow(Link3, 2))/(2*Link4*Link_12_4)) * rad2deg;
          q2_       = alpha + beta;
          q3_       = beta + gamma_;
          q4_       = 90.0 - q3_ + q2_;
    
          q1        = q1_;
          q2        = -(q2_ - init_deg2);
          q3        = q3_ + init_deg3;
          q4        = q4_ + init_deg4;
    
          cmd1      = int(q1/tick);
          cmd2      = int(q2/tick);
          cmd3      = int(q3/tick);
          cmd4      = int(q4/tick);
          /*
          Serial.println(Link_12_4); Serial.println(alpha); Serial.println(beta);
          Serial.println(q1_); Serial.println(q2_); Serial.println(q3_); Serial.println(q4_);
          Serial.println(q1); Serial.println(q2); Serial.println(q3); Serial.println(q4);
          */
    
          dxl_GoalPos[0] = 750; dxl_GoalPos[1] = cmd4; dxl_GoalPos[2] = cmd3; dxl_GoalPos[3] = cmd2; dxl_GoalPos[4] = cmd1;
          for (int i = 4; i > -1; i--)
          {
            Serial.print("Goal Position "); Serial.print("dxl"); Serial.print(String(i)); Serial.print(" "); Serial.print(dxl_GoalPos[i]); Serial.print(" ; "); Serial.println(dxl_PresPos[i]);
          }
          for (int i = 4; i > -1; i--)
          {
            Serial.println(String(i));
            // Go to Each Goal
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], ADDR_GOAL_POSITION, dxl_GoalPos[i], &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
              packetHandler->getTxRxResult(dxl_comm_result);
            }
            else if (dxl_error != 0)
            {
              packetHandler->getRxPacketError(dxl_error);
            }
            // Check
            do
            {
              dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[i], &dxl_error);
              // Serial.println(dxl_PresPos[i]);
              if (dxl_comm_result != COMM_SUCCESS)
              {
                packetHandler->getTxRxResult(dxl_comm_result);
              }
              else if (dxl_error != 0)
              {
                packetHandler->getRxPacketError(dxl_error);
              }
              }while((abs(dxl_GoalPos[i]- dxl_PresPos[i]) > DXL_MOVING_STATUS_THRESHOLD));
            }  
            Serial.println("D");
       }
       else if (msg == 'c')
       {
          // x 센터는 맞고, y 센터가 안 맞는데, y 중심점보다 바깥쪽에 있는 경우
          cur_x = Goal_Pos[0]; cur_y = Goal_Pos[1];
          // Goal_Pos Update
          cur_theta_xy = atan(cur_y/cur_x);
          Goal_Pos[0] = cur_x + change_length * cos(cur_theta_xy); Goal_Pos[1] = cur_y + change_length * sin(cur_theta_xy);
          for (int i = 0; i < 5; i++)
          {
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
              packetHandler->getTxRxResult(dxl_comm_result);
            }
            else if (dxl_error != 0)
            {
              packetHandler->getRxPacketError(dxl_error);
            }
            delay(250);
          }
          /*
           *  Read Present Position
           */
          for (int i = 0; i < 5; i++)
          {
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[i], &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
              packetHandler->getTxRxResult(dxl_comm_result);
            }
            else if (dxl_error != 0)
            {
              packetHandler->getRxPacketError(dxl_error);
            }
          }
    
          for (int i = 0; i < 3; i++)
          {
            if (i != 2)
            {
              P4[i] = Goal_Pos[i];
            }
            else
            {
              P4[i] = Goal_Pos[i] + LinkE;
            }
          }

          double _x = Goal_Pos[0];
          double _y = Goal_Pos[1];
          q1_       = atan(_y/_x) * rad2deg;
          Link_12_4 = sqrt(pow(P4[0], 2) + pow(P4[1],2) + pow(P4[2] - Link2, 2));
          alpha     = asin((P4[2] - Link2)/Link_12_4) * rad2deg;
          beta      = acos((pow(Link3, 2) + pow(Link_12_4, 2) - pow(Link4, 2))/(2*Link3*Link_12_4)) * rad2deg;
          gamma_    = acos((pow(Link4, 2) + pow(Link_12_4, 2) - pow(Link3, 2))/(2*Link4*Link_12_4)) * rad2deg;
          q2_       = alpha + beta;
          q3_       = beta + gamma_;
          q4_       = 90.0 - q3_ + q2_;
    
          q1        = q1_;
          q2        = -(q2_ - init_deg2);
          q3        = q3_ + init_deg3;
          q4        = q4_ + init_deg4;
    
          cmd1      = int(q1/tick);
          cmd2      = int(q2/tick);
          cmd3      = int(q3/tick);
          cmd4      = int(q4/tick);
          /*
          Serial.println(Link_12_4); Serial.println(alpha); Serial.println(beta);
          Serial.println(q1_); Serial.println(q2_); Serial.println(q3_); Serial.println(q4_);
          Serial.println(q1); Serial.println(q2); Serial.println(q3); Serial.println(q4);
          */
    
          dxl_GoalPos[0] = dxl0_GoalPos; dxl_GoalPos[1] = cmd4; dxl_GoalPos[2] = cmd3; dxl_GoalPos[3] = cmd2; dxl_GoalPos[4] = cmd1;
          for (int i = 4; i > -1; i--)
          {
            Serial.print("Goal Position "); Serial.print("dxl"); Serial.print(String(i)); Serial.print(" "); Serial.print(dxl_GoalPos[i]); Serial.print(" ; "); Serial.println(dxl_PresPos[i]);
          }
          for (int i = 4; i > -1; i--)
          {
            Serial.println(String(i));
            // Go to Each Goal
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], ADDR_GOAL_POSITION, dxl_GoalPos[i], &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
              packetHandler->getTxRxResult(dxl_comm_result);
            }
            else if (dxl_error != 0)
            {
              packetHandler->getRxPacketError(dxl_error);
            }
            // Check
            do
            {
              dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[i], &dxl_error);
              // Serial.println(dxl_PresPos[i]);
              if (dxl_comm_result != COMM_SUCCESS)
              {
                packetHandler->getTxRxResult(dxl_comm_result);
              }
              else if (dxl_error != 0)
              {
                packetHandler->getRxPacketError(dxl_error);
              }
              }while((abs(dxl_GoalPos[i]- dxl_PresPos[i]) > DXL_MOVING_STATUS_THRESHOLD));
            }
            Serial.println("D");
       }
       else if (msg == 'v')
       {
          // y 센터는 맞는데, x 센터가 안 맞고, 중심보다 왼쪽에 있음
          cur_x = Goal_Pos[0]; cur_y = Goal_Pos[1];
          // Goal_Pos Update
          Goal_Pos[0] = r11_ * cur_x + -r12_ * cur_y; Goal_Pos[1] = -r21_ * cur_x + r22_ * cur_y;
          for (int i = 0; i < 5; i++)
          {
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
              packetHandler->getTxRxResult(dxl_comm_result);
            }
            else if (dxl_error != 0)
            {
              packetHandler->getRxPacketError(dxl_error);
            }
            delay(250);
          }
          /*
           *  Read Present Position
           */
          for (int i = 0; i < 5; i++)
          {
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[i], &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
              packetHandler->getTxRxResult(dxl_comm_result);
            }
            else if (dxl_error != 0)
            {
              packetHandler->getRxPacketError(dxl_error);
            }
          }
    
          for (int i = 0; i < 3; i++)
          {
            if (i != 2)
            {
              P4[i] = Goal_Pos[i];
            }
            else
            {
              P4[i] = Goal_Pos[i] + LinkE;
            }
          }

          double _x = Goal_Pos[0];
          double _y = Goal_Pos[1];
          q1_       = atan(_y/_x) * rad2deg;
          Link_12_4 = sqrt(pow(P4[0], 2) + pow(P4[1],2) + pow(P4[2] - Link2, 2));
          alpha     = asin((P4[2] - Link2)/Link_12_4) * rad2deg;
          beta      = acos((pow(Link3, 2) + pow(Link_12_4, 2) - pow(Link4, 2))/(2*Link3*Link_12_4)) * rad2deg;
          gamma_    = acos((pow(Link4, 2) + pow(Link_12_4, 2) - pow(Link3, 2))/(2*Link4*Link_12_4)) * rad2deg;
          q2_       = alpha + beta;
          q3_       = beta + gamma_;
          q4_       = 90.0 - q3_ + q2_;
    
          q1        = q1_;
          q2        = -(q2_ - init_deg2);
          q3        = q3_ + init_deg3;
          q4        = q4_ + init_deg4;
    
          cmd1      = int(q1/tick);
          cmd2      = int(q2/tick);
          cmd3      = int(q3/tick);
          cmd4      = int(q4/tick);
          /*
          Serial.println(Link_12_4); Serial.println(alpha); Serial.println(beta);
          Serial.println(q1_); Serial.println(q2_); Serial.println(q3_); Serial.println(q4_);
          Serial.println(q1); Serial.println(q2); Serial.println(q3); Serial.println(q4);
          */
    
          dxl_GoalPos[0] = dxl0_GoalPos; dxl_GoalPos[1] = cmd4; dxl_GoalPos[2] = cmd3; dxl_GoalPos[3] = cmd2; dxl_GoalPos[4] = cmd1;
          for (int i = 4; i > -1; i--)
          {
            Serial.print("Goal Position "); Serial.print("dxl"); Serial.print(String(i)); Serial.print(" "); Serial.print(dxl_GoalPos[i]); Serial.print(" ; "); Serial.println(dxl_PresPos[i]);
          }
          for (int i = 4; i > -1; i--)
          {
            Serial.println(String(i));
            // Go to Each Goal
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], ADDR_GOAL_POSITION, dxl_GoalPos[i], &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
              packetHandler->getTxRxResult(dxl_comm_result);
            }
            else if (dxl_error != 0)
            {
              packetHandler->getRxPacketError(dxl_error);
            }
            // Check
            do
            {
              dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[i], &dxl_error);
              // Serial.println(dxl_PresPos[i]);
              if (dxl_comm_result != COMM_SUCCESS)
              {
                packetHandler->getTxRxResult(dxl_comm_result);
              }
              else if (dxl_error != 0)
              {
                packetHandler->getRxPacketError(dxl_error);
              }
              }while((abs(dxl_GoalPos[i]- dxl_PresPos[i]) > DXL_MOVING_STATUS_THRESHOLD));
            }
            Serial.println("D");
       }
       else if(msg == 'b')
       {
          // y 센터 맞고, x 센터가 안 맞으면서 중심보다 오른쪽
          cur_x = Goal_Pos[0]; cur_y = Goal_Pos[1];
          // Goal_Pos Update
          Goal_Pos[0] = r11_ * cur_x + r12_ * cur_y; Goal_Pos[1] = r21_ * cur_x + r22_ * cur_y;
          for (int i = 0; i < 5; i++)
          {
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
              packetHandler->getTxRxResult(dxl_comm_result);
            }
            else if (dxl_error != 0)
            {
              packetHandler->getRxPacketError(dxl_error);
            }
            delay(250);
          }
          /*
           *  Read Present Position
           */
          for (int i = 0; i < 5; i++)
          {
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[i], &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
              packetHandler->getTxRxResult(dxl_comm_result);
            }
            else if (dxl_error != 0)
            {
              packetHandler->getRxPacketError(dxl_error);
            }
          }
    
          for (int i = 0; i < 3; i++)
          {
            if (i != 2)
            {
              P4[i] = Goal_Pos[i];
            }
            else
            {
              P4[i] = Goal_Pos[i] + LinkE;
            }
          }

          double _x = Goal_Pos[0];
          double _y = Goal_Pos[1];
          q1_       = atan(_y/_x) * rad2deg;
          Link_12_4 = sqrt(pow(P4[0], 2) + pow(P4[1],2) + pow(P4[2] - Link2, 2));
          alpha     = asin((P4[2] - Link2)/Link_12_4) * rad2deg;
          beta      = acos((pow(Link3, 2) + pow(Link_12_4, 2) - pow(Link4, 2))/(2*Link3*Link_12_4)) * rad2deg;
          gamma_    = acos((pow(Link4, 2) + pow(Link_12_4, 2) - pow(Link3, 2))/(2*Link4*Link_12_4)) * rad2deg;
          q2_       = alpha + beta;
          q3_       = beta + gamma_;
          q4_       = 90.0 - q3_ + q2_;
    
          q1        = q1_;
          q2        = -(q2_ - init_deg2);
          q3        = q3_ + init_deg3;
          q4        = q4_ + init_deg4;
    
          cmd1      = int(q1/tick);
          cmd2      = int(q2/tick);
          cmd3      = int(q3/tick);
          cmd4      = int(q4/tick);
          /*
          Serial.println(Link_12_4); Serial.println(alpha); Serial.println(beta);
          Serial.println(q1_); Serial.println(q2_); Serial.println(q3_); Serial.println(q4_);
          Serial.println(q1); Serial.println(q2); Serial.println(q3); Serial.println(q4);
          */
    
          dxl_GoalPos[0] = dxl0_GoalPos; dxl_GoalPos[1] = cmd4; dxl_GoalPos[2] = cmd3; dxl_GoalPos[3] = cmd2; dxl_GoalPos[4] = cmd1;
          for (int i = 4; i > -1; i--)
          {
            Serial.print("Goal Position "); Serial.print("dxl"); Serial.print(String(i)); Serial.print(" "); Serial.print(dxl_GoalPos[i]); Serial.print(" ; "); Serial.println(dxl_PresPos[i]);
          }
          for (int i = 4; i > -1; i--)
          {
            Serial.println(String(i));
            // Go to Each Goal
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], ADDR_GOAL_POSITION, dxl_GoalPos[i], &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
              packetHandler->getTxRxResult(dxl_comm_result);
            }
            else if (dxl_error != 0)
            {
              packetHandler->getRxPacketError(dxl_error);
            }
            // Check
            do
            {
              dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[i], &dxl_error);
              // Serial.println(dxl_PresPos[i]);
              if (dxl_comm_result != COMM_SUCCESS)
              {
                packetHandler->getTxRxResult(dxl_comm_result);
              }
              else if (dxl_error != 0)
              {
                packetHandler->getRxPacketError(dxl_error);
              }
              }while((abs(dxl_GoalPos[i]- dxl_PresPos[i]) > DXL_MOVING_STATUS_THRESHOLD));
            }
            Serial.println("D");
       }
       else if(msg == 'n')
       {
          // 못 찾음 회전해야 함
          cur_x = Goal_Pos[0]; cur_y = Goal_Pos[1];
          // Goal_Pos Update
          Goal_Pos[0] = r11_ * cur_x + r12_ * cur_y; Goal_Pos[1] = r21_ * cur_x + r22_ * cur_y;
          for (int i = 0; i < 5; i++)
          {
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
              packetHandler->getTxRxResult(dxl_comm_result);
            }
            else if (dxl_error != 0)
            {
              packetHandler->getRxPacketError(dxl_error);
            }
            delay(250);
          }
          /*
           *  Read Present Position
           */
          for (int i = 0; i < 5; i++)
          {
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[i], &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
              packetHandler->getTxRxResult(dxl_comm_result);
            }
            else if (dxl_error != 0)
            {
              packetHandler->getRxPacketError(dxl_error);
            }
          }
    
          for (int i = 0; i < 3; i++)
          {
            if (i != 2)
            {
              P4[i] = Goal_Pos[i];
            }
            else
            {
              P4[i] = Goal_Pos[i] + LinkE;
            }
          }

          double _x = Goal_Pos[0];
          double _y = Goal_Pos[1];
          q1_       = atan(_y/_x) * rad2deg;
          Link_12_4 = sqrt(pow(P4[0], 2) + pow(P4[1],2) + pow(P4[2] - Link2, 2));
          alpha     = asin((P4[2] - Link2)/Link_12_4) * rad2deg;
          beta      = acos((pow(Link3, 2) + pow(Link_12_4, 2) - pow(Link4, 2))/(2*Link3*Link_12_4)) * rad2deg;
          gamma_    = acos((pow(Link4, 2) + pow(Link_12_4, 2) - pow(Link3, 2))/(2*Link4*Link_12_4)) * rad2deg;
          q2_       = alpha + beta;
          q3_       = beta + gamma_;
          q4_       = 90.0 - q3_ + q2_;
    
          q1        = q1_;
          q2        = -(q2_ - init_deg2);
          q3        = q3_ + init_deg3;
          q4        = q4_ + init_deg4;
    
          cmd1      = int(q1/tick);
          cmd2      = int(q2/tick);
          cmd3      = int(q3/tick);
          cmd4      = int(q4/tick);
          /*
          Serial.println(Link_12_4); Serial.println(alpha); Serial.println(beta);
          Serial.println(q1_); Serial.println(q2_); Serial.println(q3_); Serial.println(q4_);
          Serial.println(q1); Serial.println(q2); Serial.println(q3); Serial.println(q4);
          */
    
          dxl_GoalPos[0] = dxl0_GoalPos; dxl_GoalPos[1] = cmd4; dxl_GoalPos[2] = cmd3; dxl_GoalPos[3] = cmd2; dxl_GoalPos[4] = cmd1;
          for (int i = 4; i > -1; i--)
          {
            Serial.print("Goal Position "); Serial.print("dxl"); Serial.print(String(i)); Serial.print(" "); Serial.print(dxl_GoalPos[i]); Serial.print(" ; "); Serial.println(dxl_PresPos[i]);
          }
          for (int i = 4; i > -1; i--)
          {
            Serial.println(String(i));
            // Go to Each Goal
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], ADDR_GOAL_POSITION, dxl_GoalPos[i], &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
              packetHandler->getTxRxResult(dxl_comm_result);
            }
            else if (dxl_error != 0)
            {
              packetHandler->getRxPacketError(dxl_error);
            }
            // Check
            do
            {
              dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[i], &dxl_error);
              // Serial.println(dxl_PresPos[i]);
              if (dxl_comm_result != COMM_SUCCESS)
              {
                packetHandler->getTxRxResult(dxl_comm_result);
              }
              else if (dxl_error != 0)
              {
                packetHandler->getRxPacketError(dxl_error);
              }
              }while((abs(dxl_GoalPos[i]- dxl_PresPos[i]) > DXL_MOVING_STATUS_THRESHOLD));
            }
            Serial.println("D");
       }
       else if(msg == 'm')
       {
          // 못 찾음 회전해야 함
          cur_x = Goal_Pos[0]; cur_y = Goal_Pos[1];
          // Goal_Pos Update
          Goal_Pos[0] = r11 * cur_x + r12 * cur_y; Goal_Pos[1] = r21 * cur_x + r22 * cur_y;
          for (int i = 0; i < 5; i++)
          {
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
              packetHandler->getTxRxResult(dxl_comm_result);
            }
            else if (dxl_error != 0)
            {
              packetHandler->getRxPacketError(dxl_error);
            }
            delay(250);
          }
          /*
           *  Read Present Position
           */
          for (int i = 0; i < 5; i++)
          {
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[i], &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
              packetHandler->getTxRxResult(dxl_comm_result);
            }
            else if (dxl_error != 0)
            {
              packetHandler->getRxPacketError(dxl_error);
            }
          }
    
          for (int i = 0; i < 3; i++)
          {
            if (i != 2)
            {
              P4[i] = Goal_Pos[i];
            }
            else
            {
              P4[i] = Goal_Pos[i] + LinkE;
            }
          }

          double _x = Goal_Pos[0];
          double _y = Goal_Pos[1];
          q1_       = atan(_y/_x) * rad2deg;
          Link_12_4 = sqrt(pow(P4[0], 2) + pow(P4[1],2) + pow(P4[2] - Link2, 2));
          alpha     = asin((P4[2] - Link2)/Link_12_4) * rad2deg;
          beta      = acos((pow(Link3, 2) + pow(Link_12_4, 2) - pow(Link4, 2))/(2*Link3*Link_12_4)) * rad2deg;
          gamma_    = acos((pow(Link4, 2) + pow(Link_12_4, 2) - pow(Link3, 2))/(2*Link4*Link_12_4)) * rad2deg;
          q2_       = alpha + beta;
          q3_       = beta + gamma_;
          q4_       = 90.0 - q3_ + q2_;
    
          q1        = q1_;
          q2        = -(q2_ - init_deg2);
          q3        = q3_ + init_deg3;
          q4        = q4_ + init_deg4;
    
          cmd1      = int(q1/tick);
          cmd2      = int(q2/tick);
          cmd3      = int(q3/tick);
          cmd4      = int(q4/tick);
          /*
          Serial.println(Link_12_4); Serial.println(alpha); Serial.println(beta);
          Serial.println(q1_); Serial.println(q2_); Serial.println(q3_); Serial.println(q4_);
          Serial.println(q1); Serial.println(q2); Serial.println(q3); Serial.println(q4);
          */
    
          dxl_GoalPos[0] = dxl0_GoalPos; dxl_GoalPos[1] = cmd4; dxl_GoalPos[2] = cmd3; dxl_GoalPos[3] = cmd2; dxl_GoalPos[4] = cmd1;
          for (int i = 4; i > -1; i--)
          {
            Serial.print("Goal Position "); Serial.print("dxl"); Serial.print(String(i)); Serial.print(" "); Serial.print(dxl_GoalPos[i]); Serial.print(" ; "); Serial.println(dxl_PresPos[i]);
          }
          for (int i = 4; i > -1; i--)
          {
            Serial.println(String(i));
            // Go to Each Goal
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], ADDR_GOAL_POSITION, dxl_GoalPos[i], &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
              packetHandler->getTxRxResult(dxl_comm_result);
            }
            else if (dxl_error != 0)
            {
              packetHandler->getRxPacketError(dxl_error);
            }
            // Check
            do
            {
              dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[i], &dxl_error);
              // Serial.println(dxl_PresPos[i]);
              if (dxl_comm_result != COMM_SUCCESS)
              {
                packetHandler->getTxRxResult(dxl_comm_result);
              }
              else if (dxl_error != 0)
              {
                packetHandler->getRxPacketError(dxl_error);
              }
              }while((abs(dxl_GoalPos[i]- dxl_PresPos[i]) > DXL_MOVING_STATUS_THRESHOLD));
            }
            Serial.println("D");
          }
        }
        goto menu_start;
  case 't' :
    {
      /*
       *  Task Space Initialize
       */
      wait_until_key_pressed();
      for (int i = 0; i < 5; i++)
      {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
        delay(250);
      }
      /*
       *  Read Present Position
       */
      for (int i = 0; i < 5; i++)
      {
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[i], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
      }
      // Define Task Initial Position
      Goal_Pos[0] = 10.0; Goal_Pos[1] = 200.0; Goal_Pos[2] = 60.0;

      for (int i = 0; i < 3; i++)
      {
        if (i != 2)
        {
          P4[i] = Goal_Pos[i];
        }
        else
        {
          P4[i] = Goal_Pos[i] + LinkE;
        }
      }

      double _x = Goal_Pos[0];
      double _y = Goal_Pos[1];
      q1_       = atan(_y/_x) * rad2deg;
      Link_12_4 = sqrt(pow(P4[0], 2) + pow(P4[1],2) + pow(P4[2] - Link2, 2));
      alpha     = asin((P4[2] - Link2)/Link_12_4) * rad2deg;
      beta      = acos((pow(Link3, 2) + pow(Link_12_4, 2) - pow(Link4, 2))/(2*Link3*Link_12_4)) * rad2deg;
      gamma_    = acos((pow(Link4, 2) + pow(Link_12_4, 2) - pow(Link3, 2))/(2*Link4*Link_12_4)) * rad2deg;
      q2_       = alpha + beta;
      q3_       = beta + gamma_;
      q4_       = 90.0 - q3_ + q2_;

      q1        = q1_;
      q2        = -(q2_ - init_deg2);
      q3        = q3_ + init_deg3;
      q4        = q4_ + init_deg4;

      cmd1      = int(q1/tick);
      cmd2      = int(q2/tick);
      cmd3      = int(q3/tick);
      cmd4      = int(q4/tick);
      /*
      Serial.println(Link_12_4); Serial.println(alpha); Serial.println(beta);
      Serial.println(q1_); Serial.println(q2_); Serial.println(q3_); Serial.println(q4_);
      Serial.println(q1); Serial.println(q2); Serial.println(q3); Serial.println(q4);
      */

      dxl_GoalPos[0] = dxl0_GoalPos; dxl_GoalPos[1] = cmd4; dxl_GoalPos[2] = cmd3; dxl_GoalPos[3] = cmd2; dxl_GoalPos[4] = cmd1;
      for (int i = 4; i > -1; i--)
      {
        Serial.print("Goal Position "); Serial.print("dxl"); Serial.print(String(i)); Serial.print(" "); Serial.print(dxl_GoalPos[i]); Serial.print(" ; "); Serial.println(dxl_PresPos[i]);
      }
      for (int i = 4; i > -1; i--)
      {
        Serial.println(String(i));
        // Go to Each Goal
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id[i], ADDR_GOAL_POSITION, dxl_GoalPos[i], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
        // Check
        do
        {
          dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, dxl_id[i], ADDR_PRESENT_POSITION, (uint32_t*)&dxl_PresPos[i], &dxl_error);
          // Serial.println(dxl_PresPos[i]);
          if (dxl_comm_result != COMM_SUCCESS)
          {
            packetHandler->getTxRxResult(dxl_comm_result);
          }
          else if (dxl_error != 0)
          {
            packetHandler->getRxPacketError(dxl_error);
          }
        }while((abs(dxl_GoalPos[i]- dxl_PresPos[i]) > DXL_MOVING_STATUS_THRESHOLD));
      }
      Serial.println("D");
    }
    goto menu_start;    
    default :
      goto menu_start;
  }
}


/*
 * 
 *  필요한 함수들 정의 *
 * 
 */

 /*
  *  Forward Kinematics 정의
  *  X Y Z 각각 Point 를 구함
  */

double ForwardKinematicsX(double _theta0_r, double _theta1_r, double _theta2_r, double _theta3_r)
{
  double X;
  X = Link3 * cos(_theta0_r)*cos(_theta1_r) + Link4*(cos(_theta0_r)*cos(_theta1_r)*cos(_theta2_r) - cos(_theta0_r)*sin(_theta1_r)*sin(_theta2_r)) + LinkE*(cos(_theta0_r)*cos(_theta1_r)*cos(_theta2_r)*cos(_theta3_r) - cos(_theta0_r)*sin(_theta1_r)*sin(_theta2_r)*cos(_theta3_r) - cos(_theta0_r)*cos(_theta1_r)*sin(_theta2_r)*sin(_theta3_r) - cos(_theta0_r)*sin(_theta1_r)*cos(_theta2_r)*sin(_theta3_r));
  return X;
}
double ForwardKinematicsY(double _theta0_r, double _theta1_r, double _theta2_r, double _theta3_r)
{
  double Y;
  Y = Link3 * sin(_theta0_r)*cos(_theta1_r) + Link4*(sin(_theta0_r)*cos(_theta1_r)*cos(_theta2_r) - sin(_theta0_r)*sin(_theta1_r)*sin(_theta2_r)) + LinkE*(sin(_theta0_r)*cos(_theta1_r)*cos(_theta2_r)*cos(_theta3_r) - sin(_theta0_r)*sin(_theta1_r)*sin(_theta2_r)*cos(_theta3_r) - sin(_theta0_r)*cos(_theta1_r)*sin(_theta2_r)*sin(_theta3_r) - sin(_theta0_r)*sin(_theta1_r)*cos(_theta2_r)*sin(_theta3_r));
  return Y;
}
double ForwardKinematicsZ(double _theta0_r, double _theta1_r, double _theta2_r, double _theta3_r)
{
  double Z;
  Z = Link2 + Link3 * sin(_theta1_r) + Link4 * (sin(_theta1_r)*cos(_theta2_r) + cos(_theta1_r)*sin(_theta2_r))+ LinkE*(sin(_theta1_r)*cos(_theta2_r)*cos(_theta3_r) + cos(_theta1_r)*sin(_theta2_r)*cos(_theta3_r) - sin(_theta1_r)*sin(_theta2_r)*sin(_theta3_r) + cos(_theta1_r)*cos(_theta2_r)*sin(_theta3_r));
  return Z;
}


int E_EPos(void)
{
  int _input;
  while(!(Serial.available()>0));
  while(Serial.available()>0)
  {
    _input= Serial.parseInt();
  }
  return _input;
}

/*
 * Empty 가 아닌 키가 눌렸는지 확인하는 Method
 */
char check_pressed_key(void)
{ 
  char in_key;
  if(Serial.available()>0)
  {
    while(Serial.available()>0) 
      in_key = (char)Serial.read();
    return in_key;
  }
  else
    return 0;
}

/*
 * Empty 포함 모든 키 입력을 받음
 */

char wait_until_key_pressed(void)
{
  char in_key;
  while(!(Serial.available()>0));
  while(Serial.available()>0)
  {
    in_key = (char)Serial.read();
  }
  return in_key;
}

void loop() 
{
}
