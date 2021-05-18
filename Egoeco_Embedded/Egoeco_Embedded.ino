#include <mcp_can.h>
#include <SPI.h>
#include <MsTimer2.h>
#include <SoftwareSerial.h>

// the cs pin of the version v1.4 is default to D10
const int SPI_CS_PIN = 10; // CAN Shield(AS54887CAN)
MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

const int BT_RX =  2;
const int BT_TX =  3;
// SoftwareSerial(RX, TX) 형식으로 블루투스 모듈과 교차하여 연결
SoftwareSerial BTSerial(BT_RX, BT_TX);


// CAN Msg Define
const int IDX_OBD_DATA_LEN =  0;
const int IDX_OBD_MODE =      1;
const int IDX_OBD_PID =       2;
const int IDX_OBD_DATA0 =       3;
const int IDX_OBD_DATA1 =       4;
const int IDX_OBD_DATA2 =       5;
const int IDX_OBD_DATA3 =       6;
const int IDX_OBD_DATA4 =       7;

const int CAN_DATA_LEN =  8;

const char DATA_LEN_1 = 0x01;
const char DATA_LEN_2 = 0x02;
const char DATA_LEN_3 = 0x03;
const char DATA_LEN_4 = 0x04;
const char DATA_LEN_5 = 0x05;
const char DATA_LEN_6 = 0x06;
const char DATA_LEN_7 = 0x07;

const char MODE_NOW_DATA  = 0x01;

const char PID_EngRPM                 = 0x0C;
const char PID_VehicleSpd             = 0x0D;
const char PID_RelativeThrottlePosD   = 0x49;


// BlueTooth Msg Define
const int IDX_BT_START_BYTE   = 0;
const int IDX_BT_SEND_NUMBER  = 1;
const int IDX_BT_ENGRPM_A     = 2;
const int IDX_BT_ENGRPM_B     = 3;
const int IDX_BT_VS_SPD       = 4;
const int IDX_BT_ECODRV_LV    = 5;
const int IDX_BT_RESERVED     = 6;
const int IDX_BT_CHKSUM       = 7;

const int BT_DATA_LEN =  8;

const char START_BYTE   = 0x55;

const char SEND_NUM_1   = 0x01;


// Request To OBD(Vehicle) CAN Message
unsigned char req2OBD_EngRpm[8] = {DATA_LEN_2, MODE_NOW_DATA, PID_EngRPM, 0x55, 0x55, 0x55, 0x55, 0x55};
unsigned char req2OBD_VsSpd[8] = {DATA_LEN_2, MODE_NOW_DATA, PID_VehicleSpd, 0x55, 0x55, 0x55, 0x55, 0x55};
unsigned char req2OBD_ThrottlePos[8] = {DATA_LEN_2, MODE_NOW_DATA, PID_RelativeThrottlePosD, 0x55, 0x55, 0x55, 0x55, 0x55};


// Vehicle Value, for BlueTooth Send
unsigned char EngRpm_data[2] = {0, 0};
unsigned char VehicleSpeed_data = 0;
unsigned char AccelPedalPos_data = 0;

// Vehicle Value, only test variable
unsigned int EngRpm_val = 0;
unsigned int Vehicle_Speed_val = 0;
unsigned int AccelPedal_Pos_val = 0;


// System Variable
unsigned char flag_CanRecv = 0;
unsigned char can_len = 0;
unsigned char can_revbuf[8];
unsigned int canID_rev = 0;

unsigned int cnt_CanSend = 0;

unsigned char bt_sendbuf[8];
unsigned int bt_len;

void setup() {

  Serial.begin(115200);

  if(CAN_OK == CAN.begin(CAN_500KBPS))                   // init can bus : baudrate = 500k
  {
      Serial.println("CAN BUS Shield init ok!");
  }
  else
  {
      Serial.println("CAN BUS Shield init fail");
  }

  BTSerial.begin(9600);

  attachInterrupt(0, MCP2515_ISR, FALLING); // start interrupt

  MsTimer2::set(100, ISR1);
  MsTimer2::start();
  
}



void loop() {

  unsigned char chk = 10;
 
  if(flag_CanRecv) 
  {
    flag_CanRecv = 0;

    // iterate over all pending messages
    // If either the bus is saturated or the MCU is busy,
    // both RX buffers may be in use and reading a single
    // message does not clear the IRQ conditon.
    while (CAN_MSGAVAIL == CAN.checkReceive()) 
    {
      chk = CAN.readMsgBuf(&can_len, can_revbuf);    
      if(chk == 0)
      {
        chk = 10;
        
        canID_rev = CAN.getCanId();
        if(0x7E0 <= canID_rev && canID_rev <= 0x7EF)
        {
          Log_CanMsg();
          
          Read_CanData();
        }
        else
        {
          break;
        }
      
        
      }
    
    }
    
  }

  /*
  // 블루투스 모듈 ➜ 아두이노 ➜ 시리얼 모니터
  if (BlueToothSerial.available()) 
  {           // 데이터 수신 대기
    char receivechar = BlueToothSerial.read(); // 수신 데이터 읽기
    Serial.write(receivechar);                 // 수신 데이터 시리얼모니터로 출력
  }
  */
  
}

void ISR1()
{
  cnt_CanSend++;
  if(5 <= cnt_CanSend)
  {
    cnt_CanSend = 0;
  }

  switch(cnt_CanSend)
  {
    case 0:
      break;
    
    case 1:
      // send data:  id, standrad frame, data len, data buf
      CAN.sendMsgBuf(0x7DF, 0, 8, req2OBD_EngRpm);
      break;

    case 2:
      //CAN.sendMsgBuf(0x7DF, 0, 8, req2OBD_ThrottlePos);
      break;

    case 3:
      CAN.sendMsgBuf(0x7DF, 0, 8, req2OBD_VsSpd);
      break;

    case 4:
      Send_BTdata();
      break;

    default:
      break;
  }
}

void Read_CanData()
{
  unsigned int PID = can_revbuf[IDX_OBD_PID];
  
  switch(PID)
  {
    case PID_EngRPM:
      EngRpm_data[0] = can_revbuf[IDX_OBD_DATA0];
      EngRpm_data[1] = can_revbuf[IDX_OBD_DATA1];
      break;

    case PID_VehicleSpd:
      VehicleSpeed_data = can_revbuf[IDX_OBD_DATA0];
      break;
    
    case PID_RelativeThrottlePosD:
      //AccelPedalPos_data = can_revbuf[IDX_OBD_DATA0];
      break;
  
    default:
      break;
  }
}

void Log_CanMsg()
{
  Serial.print("CanID: ");
  Serial.println(canID_rev, HEX);
  
  for(int i = 0; i<can_len; i++)    // print the data
  {
    Serial.print(can_revbuf[i], HEX);
    Serial.print("\t");
  }
  Serial.println();

  if(can_revbuf[IDX_OBD_PID] == PID_EngRPM)
  {
    // endgine speed response
    // len,   Mode, PID,  DataA,  DataB
    // 0x04,  0x41, 0x0C, 0x??,   0x??
    // (256*A + B) / 100 = RPM
    EngRpm_val = (unsigned int)((256*can_revbuf[IDX_OBD_DATA0] + can_revbuf[IDX_OBD_DATA1]) / 4);

    Serial.print("EngRPM: ");
    Serial.print(EngRpm_val);
    Serial.println(" RPM");
  }
  else if(can_revbuf[IDX_OBD_PID] == PID_VehicleSpd)
  {
    Vehicle_Speed_val = (unsigned int)can_revbuf[IDX_OBD_DATA0];

    Serial.print("Vehicle Speed: ");
    Serial.print(Vehicle_Speed_val);
    Serial.println(" Km/h");
  }
  else if(can_revbuf[IDX_OBD_PID] == PID_RelativeThrottlePosD)
  {
    AccelPedal_Pos_val = (unsigned int)can_revbuf[IDX_OBD_DATA0];

    Serial.print("AccelPedal: ");
    Serial.print(AccelPedal_Pos_val);
    Serial.println(" %");
  }
  else
  {
    // Nothing
  }
  
}

void Send_BTdata()
{
  int i = 0;

  bt_sendbuf[IDX_BT_START_BYTE] = START_BYTE;
  bt_sendbuf[IDX_BT_SEND_NUMBER] = SEND_NUM_1;
  bt_sendbuf[IDX_BT_ENGRPM_A] = EngRpm_data[0];
  bt_sendbuf[IDX_BT_ENGRPM_B] = EngRpm_data[1];
  bt_sendbuf[IDX_BT_VS_SPD] = VehicleSpeed_data;
  bt_sendbuf[IDX_BT_ECODRV_LV] = 3;
  bt_sendbuf[IDX_BT_RESERVED] = 0x00;
  bt_sendbuf[IDX_BT_CHKSUM] = CheckSum(bt_sendbuf, 6);


  for(i=0; i<BT_DATA_LEN; i++)
  {
    BTSerial.write(bt_sendbuf[i]); 
  }
}


int CheckSum(unsigned char *data, int leng) {
  
  unsigned char csum;

  csum = 0xFF;
  for (;leng > 0;leng--)
  {
      csum += *data++;
      Serial.println(*data, HEX);
  }
  return ~csum;
}

void MCP2515_ISR()
{
    flag_CanRecv = 1;
}
