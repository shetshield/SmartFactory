#include <DynamixelSDK.h>

// Control table address (MX-series with Protocol 2.0)
#define OPERATING_MODE                  11
#define HOMING_OFFSET                   20
#define ADDR_MX_TORQUE_ENABLE           64                 // Control table address is different in Dynamixel model
#define GOAL_VELOCITY                   104
#define ADDR_MX_PROFILE_VELOCITY        108
#define ADDR_MX_PROFILE_ACCELERATION    112
#define ADDR_MX_GOAL_POSITION           116
#define ADDR_MX_PRESENT_POSITION        132

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL1_ID                         4                   // Dynamixel ID: 4 (Motor 1)
                                                            // Test Dynamixel ID : 4
#define DXL2_ID                         1                   // Dynamixel ID: 1 (Motor 2)
#define BAUDRATE                        1000000
#define DEVICENAME                      "3"                 //DEVICENAME "1" -> Serial1(OpenCM9.04 DXL TTL Ports)
                                                            //DEVICENAME "2" -> Serial2
                                                            //DEVICENAME "3" -> Serial3(OpenCM 485 EXP)
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      0                   // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4000                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

// ETC Variable
uint8_t OperMode       = 4;

// Linear Guided Cylinder Control Pin
int cyl1_dnPin = 0;
int cyl1_upPin = 1;
int cyl2_dnPin = 2;
int cyl2_upPin = 3;

/*
 * Ejector Control Pin 선언
 * 4, 6(Pin): Grey  - 진공 제어
 * 5, 7(Pin): White - Release 제어
 */
int ejt1_VacOnPin   = 4;
int ejt1_ReleasePin = 5;
int ejt2_VacOnPin   = 6;
int ejt2_ReleasePin = 7;

int UP  = 80;
int DN  = 80;
int RLs = 255;

void setup() {
  /*
   * Cylinder 및 Ejector PinMode 선언
   */
  // Cylinder Part
  pinMode(cyl1_upPin, OUTPUT);
  pinMode(cyl1_dnPin, OUTPUT);
  pinMode(cyl2_upPin, OUTPUT);
  pinMode(cyl2_dnPin, OUTPUT);
  // Ejector Part
  pinMode(ejt1_VacOnPin, OUTPUT);
  pinMode(ejt1_ReleasePin, OUTPUT);
  pinMode(ejt2_VacOnPin, OUTPUT);
  pinMode(ejt2_ReleasePin, OUTPUT);
  // Cylinder Part
  analogWrite(cyl1_upPin, UP);
  analogWrite(cyl1_dnPin, 0);
  analogWrite(cyl2_upPin, UP);
  analogWrite(cyl2_dnPin, 0);
  delay(500);
  // Ejector Part
  analogWrite(ejt1_VacOnPin, 0);
  analogWrite(ejt1_ReleasePin, RLs);
  analogWrite(ejt2_VacOnPin, 0);
  analogWrite(ejt2_ReleasePin, RLs);
  delay(500);

  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;              // Communication result
  int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position

  uint8_t dxl_error = 0;                           // Dynamixel1 error
  int32_t dxl_GoalVel = 50;

  int32_t dxl1_present_position = 0;               // Present position of Dynamixel1
  int32_t dxl2_present_position = 0;               // Present position of Dynamixel2

  int32_t dxl1_PresHome_Offset = 0; int32_t dxl1_SetHome_Offset  = 0; int32_t dxl1_DefaultPos = 0; int32_t dxl1_GoalPos = 0;
  int32_t dxl2_PresHome_Offset = 0; int32_t dxl2_SetHome_Offset  = 0; int32_t dxl2_DefaultPos = 0; int32_t dxl2_GoalPos = 0;

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
  // 기본적으로 모터 토크는 오프 상태로 시작
  // 다이나믹셀 모터 1 토크 오프
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }

  // 다이나믹셀 모터 2 토크 오프
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }

  // 다이나믹셀 모터 1 동작모드 변경
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, OPERATING_MODE, OperMode, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }

  // 다이나믹셀 모터 2 동작모드 변경
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, OPERATING_MODE, OperMode, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    packetHandler->getTxRxResult(dxl_comm_result);
  }
  else if (dxl_error != 0)
  {
    packetHandler->getRxPacketError(dxl_error);
  }
  // 다이나믹셀 1 홈 오프셋 수정
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID, HOMING_OFFSET,  0, &dxl_error);
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

  // 다이나믹셀 2 홈 오프셋 수정
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL2_ID, HOMING_OFFSET,  0, &dxl_error);
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
  /*
  // 다이나믹셀 1 GOAL VELOCITY 수정
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID, GOAL_VELOCITY,  dxl_GoalVel, &dxl_error);
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

  // 다이나믹셀 2 GOAL VELOCITY 수정
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL2_ID, GOAL_VELOCITY,  dxl_GoalVel, &dxl_error);
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
  */

menu_start :
  switch(wait_until_key_pressed())
  {
    case '1' :
    {
      // 1) Read Current Position
      // Read present position of Dynamixel1
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION, (uint32_t*)&dxl1_present_position, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->getTxRxResult(dxl_comm_result);
      }
      else if (dxl_error != 0)
      {
        packetHandler->getRxPacketError(dxl_error);
      }
      // Read present position of Dynamixel2
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION, (uint32_t*)&dxl2_present_position, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->getTxRxResult(dxl_comm_result);
      }
      else if (dxl_error != 0)
      {
        packetHandler->getRxPacketError(dxl_error);
      }

      // 현재 HOMING_OFFSET 값 각각 읽어옴
      // 다이나믹셀 1 홈 오프셋
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL1_ID, HOMING_OFFSET,  (uint32_t*)&dxl1_PresHome_Offset, &dxl_error);
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

      // 다이나믹셀 2 홈 오프셋
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL2_ID, HOMING_OFFSET,  (uint32_t*)&dxl2_PresHome_Offset, &dxl_error);
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

      // 홈 오프셋 값 설정
      dxl1_SetHome_Offset = (-1) * dxl1_present_position;
      dxl2_SetHome_Offset = (-1) * dxl2_present_position;

      // 디버깅
      // Serial.print(dxl1_present_position); Serial.print("\t"); Serial.println(dxl1_SetHome_Offset);
      // Serial.print(dxl2_present_position); Serial.print("\t"); Serial.println(dxl2_SetHome_Offset);

      // 다이나믹셀 1 홈 오프셋 수정
      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID, HOMING_OFFSET,  dxl1_SetHome_Offset, &dxl_error);
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
      delay(250);
      // 다이나믹셀 2 홈 오프셋 수정
      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL2_ID, HOMING_OFFSET,  dxl2_SetHome_Offset, &dxl_error);
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

      delay(250);
      // 다이나믹셀 1 토크 온
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
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
      delay(250);

      // 다이나믹셀 2 토크 온
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
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
      delay(250);

      // 다이나믹셀 1 토크 오프
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->getTxRxResult(dxl_comm_result);
      }
      else if (dxl_error != 0)
      {
        packetHandler->getRxPacketError(dxl_error);
      }
      delay(250);

      // 다이나믹셀 2 토크 오프
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->getTxRxResult(dxl_comm_result);
      }
      else if (dxl_error != 0)
      {
        packetHandler->getRxPacketError(dxl_error);
      }
      delay(250);

      // 다이나믹셀 1 홈 오프셋
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL1_ID, HOMING_OFFSET,  (uint32_t*)&dxl1_PresHome_Offset, &dxl_error);
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

      // 다이나믹셀 2 홈 오프셋
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL2_ID, HOMING_OFFSET,  (uint32_t*)&dxl2_PresHome_Offset, &dxl_error);
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

      // 디버깅
      // Serial.print(dxl1_PresHome_Offset); Serial.print("\t"); Serial.println(dxl2_PresHome_Offset);

      // PC 로 다이나믹셀 현재 포지션 전송
      while(1)
      {
        // 다이나믹셀 1 포지션 읽음
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION, (uint32_t*)&dxl1_present_position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
        // 다이나믹셀 2 포지션 읽음
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION, (uint32_t*)&dxl2_present_position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
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
        Serial.print(dxl1_present_position); Serial.print(","); Serial.print(dxl2_present_position); Serial.println(" ");
      }
    }
    goto menu_start;
    case '2' :
    {
      // 다이나믹셀 1 토크 온
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
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
      // 다이나믹셀 2 토크 온
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
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

      // zero position 으로 이동
      // 다이나믹셀 1 골 포지션 입력 & 이동
      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_MX_GOAL_POSITION, dxl1_DefaultPos, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->getTxRxResult(dxl_comm_result);
      }
      else if (dxl_error != 0)
      {
        packetHandler->getRxPacketError(dxl_error);
      }
      // 실제로 해당 위치 근처까지 갔는지 확인하는 코드
      do
      {
        // Read present position
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION, (uint32_t*)&dxl1_present_position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
      }while((abs(dxl1_DefaultPos - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD));
      // 다이나믹셀 2 골 포지션 입력 & 이동
      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL2_ID, ADDR_MX_GOAL_POSITION, dxl2_DefaultPos, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->getTxRxResult(dxl_comm_result);
      }
      else if (dxl_error != 0)
      {
        packetHandler->getRxPacketError(dxl_error);
      }
      // 실제로 해당 위치 근처까지 갔는지 확인하는 코드
      do
      {
        // Read present position
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION, (uint32_t*)&dxl2_present_position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
      }while((abs(dxl2_DefaultPos - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD));
      Serial.println("DONE");
    }
    goto menu_start;
    case '5' :
    {
      // 목표 위치로 이동
      // 다이나믹셀 1 골 포지션 입력 & 이동
      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_MX_GOAL_POSITION, dxl1_GoalPos, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->getTxRxResult(dxl_comm_result);
      }
      else if (dxl_error != 0)
      {
        packetHandler->getRxPacketError(dxl_error);
      }
      // 실제로 해당 위치 근처까지 갔는지 확인하는 코드
      do
      {
        // Read present position
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION, (uint32_t*)&dxl1_present_position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
      }while((abs(dxl1_GoalPos - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD));
      delay(500);
      // 다이나믹셀 2 골 포지션 입력 & 이동
      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL2_ID, ADDR_MX_GOAL_POSITION, dxl2_GoalPos, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->getTxRxResult(dxl_comm_result);
      }
      else if (dxl_error != 0)
      {
        packetHandler->getRxPacketError(dxl_error);
      }
      // 실제로 해당 위치 근처까지 갔는지 확인하는 코드
      do
      {
        // Read present position
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION, (uint32_t*)&dxl2_present_position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
          packetHandler->getTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
          packetHandler->getRxPacketError(dxl_error);
        }
      }while((abs(dxl2_GoalPos - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD));
    }
    goto menu_start;
    case '7' :
    {
      while(!(Serial.available()>0));
      while(Serial.available()>0)
      {
        dxl1_GoalPos = Serial.parseInt(); // Goal Position from Host for D.Xel 1
        dxl2_GoalPos = Serial.parseInt(); // Goal Position from Host for D.Xel 2
      }
    }
    goto menu_start;
    default :
      goto menu_start;
  }
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
