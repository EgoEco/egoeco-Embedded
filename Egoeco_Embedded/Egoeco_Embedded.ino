#include <mcp_can.h>
#include <SPI.h>
#include <MsTimer2.h>
#include <SoftwareSerial.h>

// Eco Drive Table
const int VS_RANGE = 15;

const int ECO_RANGE_M2  = 0;
const int ECO_RANGE_M1  = 1;
const int ECO_RANGE_P1  = 2;
const int ECO_RANGE_P2  = 3;
const int ECO_RANGE_NUM = 4;

const int ECO_RATE_VERY_BAD   = 1;
const int ECO_RATE_BAD        = 2;
const int ECO_RATE_NORMAL     = 3;
const int ECO_RATE_GOOD       = 4;
const int ECO_RATE_VERY_GOOD  = 5;


unsigned int ecoDrive_rpm_y[VS_RANGE] = // x1
{0, 1200, 1250, 1250, 1500, 1500, 1800, 1900, 1800, 2000,
2100, 2200, 2300, 2400, 2500};

unsigned int ecoDrive_vs_x[VS_RANGE] = // x1
{0, 10,   20,   30,   40,   50,   60,   70,   80,   90, 
100,  110,  120,  130,  140};

double ecoDrive_rateRange[VS_RANGE][4] = 
{ 
  {0.8, 0.9, 1.1 , 1.2},  // 0
  {0.8, 0.9, 1.1 , 1.2},  // 10
  {0.8, 0.9, 1.1 , 1.2},  // 20
  {0.8, 0.9, 1.1 , 1.2},  // 30
  {0.8, 0.9, 1.1 , 1.2},  // 40
  {0.8, 0.9, 1.1 , 1.2},  // 50
  {0.8, 0.9, 1.1 , 1.2},  // 60
  {0.8, 0.9, 1.1 , 1.2},  // 70
  {0.8, 0.9, 1.1 , 1.2},  // 80
  {0.8, 0.9, 1.1 , 1.2},  // 90
  {0.8, 0.9, 1.1 , 1.2},  // 100
  {0.8, 0.9, 1.1 , 1.2},  // 110
  {0.8, 0.9, 1.1 , 1.2},  // 120
  {0.8, 0.9, 1.1 , 1.2},  // 130
  {0.8, 0.9, 1.1 , 1.2},  // 140
};
// Eco Drive Table END

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
// CAN Msg Define END


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
// BlueTooth Msg Define END


// Request To OBD(Vehicle) CAN Message
unsigned char req2OBD_EngRpm[8] = {DATA_LEN_2, MODE_NOW_DATA, PID_EngRPM, 0x55, 0x55, 0x55, 0x55, 0x55};
unsigned char req2OBD_VsSpd[8] = {DATA_LEN_2, MODE_NOW_DATA, PID_VehicleSpd, 0x55, 0x55, 0x55, 0x55, 0x55};
unsigned char req2OBD_ThrottlePos[8] = {DATA_LEN_2, MODE_NOW_DATA, PID_RelativeThrottlePosD, 0x55, 0x55, 0x55, 0x55, 0x55};
// Request To OBD(Vehicle) CAN Message END


// Vehicle Value, for BlueTooth Send
unsigned char EngRpm_data[2] = {0, 0};
unsigned char VehicleSpeed_data = 0;
unsigned char AccelPedalPos_data = 0;
// Vehicle Value, for BlueTooth Send END

// Vehicle Value, only test variable
unsigned int EngRpm_val = 0;
unsigned int Vehicle_Speed_val = 0;
unsigned int AccelPedal_Pos_val = 0;
// Vehicle Value, only test variable END

unsigned char EcoRate = 0;

// System Variable
unsigned char flag_CanRecv = 0;
unsigned char can_len = 0;
unsigned char can_revbuf[8];
unsigned int canID_rev = 0;

unsigned int cnt_CanSend = 0;

unsigned char bt_sendbuf[8];
unsigned int bt_len;

int flag_CanSend_reqRPM = 0;
int flag_CanSend_reqVS = 0;
int flag_BTSend_reqInfo = 0;
int flag_CalEcoRate = 0;

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
 
  if(flag_CanRecv) 
  {
    flag_CanRecv = 0;
    //canID_rev = CAN.getCanId();
       
    // iterate over all pending messages
    // If either the bus is saturated or the MCU is busy,
    // both RX buffers may be in use and reading a single
    // message does not clear the IRQ conditon.
    while (CAN_MSGAVAIL == CAN.checkReceive()) 
    {
      CAN.readMsgBuf(&can_len, can_revbuf);    

      canID_rev = CAN.getCanId();
  
      if(0x7E0 <= canID_rev && canID_rev <= 0x7EF)
      {
        Cal_CanrawData();
        
        //Log_CanMsg();
        
        Read_CanData();
      }

    }
    
  }



  if(flag_CanSend_reqRPM == 1)
  {
    flag_CanSend_reqRPM = 0;

    // send data:  id, standrad frame, data len, data buf
    CAN.sendMsgBuf(0x7DF, 0, 8, req2OBD_EngRpm);
  }

  if(flag_CanSend_reqVS == 1)
  {
    flag_CanSend_reqVS = 0;

    CAN.sendMsgBuf(0x7DF, 0, 8, req2OBD_VsSpd);
  }

  if(flag_BTSend_reqInfo == 1)
  {
    flag_BTSend_reqInfo = 0;

    Send_BTdata();
  }

  if(flag_CalEcoRate == 1)
  {
    flag_CalEcoRate = 0;

    if(Vehicle_Speed_val <= 10)
    {
      EcoRate = ECO_RATE_NORMAL;
    }
    else
    {
      EcoRate = Judge_EcoRate(Vehicle_Speed_val, EngRpm_val, ecoDrive_rateRange, 
                               ecoDrive_vs_x, ecoDrive_rpm_y, VS_RANGE);
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
      //CAN.sendMsgBuf(0x7DF, 0, 8, req2OBD_ThrottlePos);
      break;
    
    case 1:
      flag_CanSend_reqRPM = 1;
      break;

    case 2:
      flag_CanSend_reqVS = 1;
      break;

    case 3:
      flag_CalEcoRate = 1;
      break;

    case 4:
      flag_BTSend_reqInfo = 1;
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

void Cal_CanrawData()
{
  if(can_revbuf[IDX_OBD_PID] == PID_EngRPM)
  {
    // endgine speed response
    // len,   Mode, PID,  DataA,  DataB
    // 0x04,  0x41, 0x0C, 0x??,   0x??
    // (256*A + B) / 100 = RPM
    EngRpm_val = (unsigned int)((256*can_revbuf[IDX_OBD_DATA0] + can_revbuf[IDX_OBD_DATA1]) / 4);
  }
  else if(can_revbuf[IDX_OBD_PID] == PID_VehicleSpd)
  {
    Vehicle_Speed_val = (unsigned int)can_revbuf[IDX_OBD_DATA0];

  }
  else if(can_revbuf[IDX_OBD_PID] == PID_RelativeThrottlePosD)
  {
    AccelPedal_Pos_val = (unsigned int)can_revbuf[IDX_OBD_DATA0];
  }
  else
  {
    // Nothing
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
    // EngRpm_val = (unsigned int)((256*can_revbuf[IDX_OBD_DATA0] + can_revbuf[IDX_OBD_DATA1]) / 4);

    Serial.print("EngRPM: ");
    Serial.print(EngRpm_val);
    Serial.println(" RPM");
  }
  else if(can_revbuf[IDX_OBD_PID] == PID_VehicleSpd)
  {
    //Vehicle_Speed_val = (unsigned int)can_revbuf[IDX_OBD_DATA0];

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
  bt_sendbuf[IDX_BT_ECODRV_LV] = EcoRate;
  bt_sendbuf[IDX_BT_RESERVED] = 0x00;
  bt_sendbuf[IDX_BT_CHKSUM] = CheckSum(bt_sendbuf, 6);

  for(i = 0; i<BT_DATA_LEN; i++)    // print the data
  {
    Serial.print("\t");
    Serial.print(bt_sendbuf[i], DEC);
  }
  Serial.println();

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
  }
  return ~csum;
}

/*
int Interpolation_Arr_returnY(unsigned int x, unsigned int arr_x[], unsigned int arr_y[], unsigned int arr_len)
{
    unsigned int rlt_y = 0; // error value
    double x_ratio = 0;

    unsigned int rel_y = 0;

    if (x <= arr_x[0])
    {
        rlt_y = arr_y[0];
    }
    else if (arr_x[arr_len - 1] <= x)
    {
        rlt_y = arr_y[arr_len - 1];
    }
    else
    {

        for (int i = 0; i < arr_len - 1; i++)
        {
            if (arr_x[i] <= x && x < arr_x[i + 1])
            {
                x_ratio = ((double)x - (double)arr_x[i]) / ((double)arr_x[i + 1] - (double)arr_x[i]);
                rel_y = (unsigned int)(x_ratio * abs((signed int)arr_y[i + 1] - (signed int)arr_y[i]));

                if (arr_y[i] <= arr_y[i + 1])
                {
                    rlt_y = arr_y[i] + rel_y;
                }
                else
                {
                    rlt_y = arr_y[i] - rel_y;
                }

                break;
            }
        }

    }

    return rlt_y;
}
*/

unsigned int Interpolation_returnY(unsigned int x, unsigned int x1, unsigned int x2, unsigned int y1, unsigned int y2)
{
    unsigned int rlt_y = 0;
    double x_ratio = 0;
    unsigned int rel_y = 0;

    if (x <= x1)
    {
        rlt_y = y1;
    }
    else if (x2 <= x)
    {
        rlt_y = y2;
    }
    else
    {
        x_ratio = ((double)x - (double)x1) / ((double)x2 - (double)x1);
        rel_y = (unsigned int)(x_ratio * abs((signed int)y2 - (signed int)y1));

        if (y1 <= y2)
        {
            rlt_y = y1 + rel_y;
        }
        else
        {
            rlt_y = y1 - rel_y;
        }
    }

    return rlt_y;
}

// x : now VS, y : now RPM
unsigned int Judge_EcoRate(unsigned int x, unsigned int y, double ecoRange[][ECO_RANGE_NUM], 
                              unsigned int arr_x[], unsigned int arr_y[], unsigned int arr_len)
{
  unsigned int rlt_rate = 0;
  static int ecoRange_list[ECO_RANGE_NUM] = {0,0,0,0};

  static int ecoRange_list_temp1[ECO_RANGE_NUM] = {0,0,0,0};
  static int ecoRange_list_temp2[ECO_RANGE_NUM] = {0,0,0,0};
  static int eco_temp_arr_y[2] = {0,0};

  unsigned int idx_range1 = 0;
  unsigned int idx_range2 = 0;

  if (x <= arr_x[0])
  {
    ecoRange_list[ECO_RANGE_M2] = (int)(ecoRange[0][ECO_RANGE_M2]*arr_y[0]);
    ecoRange_list[ECO_RANGE_M1] = (int)(ecoRange[0][ECO_RANGE_M1]*arr_y[0]);
    ecoRange_list[ECO_RANGE_P1] = (int)(ecoRange[0][ECO_RANGE_P1]*arr_y[0]);
    ecoRange_list[ECO_RANGE_P2] = (int)(ecoRange[0][ECO_RANGE_P2]*arr_y[0]);
  }
  else if (arr_x[arr_len - 1] <= x)
  {
    ecoRange_list[ECO_RANGE_M2] = (int)(ecoRange[arr_len-1][ECO_RANGE_M2]*arr_y[arr_len-1]);
    ecoRange_list[ECO_RANGE_M1] = (int)(ecoRange[arr_len-1][ECO_RANGE_M1]*arr_y[arr_len-1]);
    ecoRange_list[ECO_RANGE_P1] = (int)(ecoRange[arr_len-1][ECO_RANGE_P1]*arr_y[arr_len-1]);
    ecoRange_list[ECO_RANGE_P2] = (int)(ecoRange[arr_len-1][ECO_RANGE_P2]*arr_y[arr_len-1]);
  }
  else
  {
    for (int i = 0; i < arr_len - 1; i++)
    {
      if (arr_x[i] <= x && x < arr_x[i + 1])
      {
        ecoRange_list_temp1[ECO_RANGE_M2] = (int)(ecoRange[i][ECO_RANGE_M2]*arr_y[i]);
        ecoRange_list_temp1[ECO_RANGE_M1] = (int)(ecoRange[i][ECO_RANGE_M1]*arr_y[i]);
        ecoRange_list_temp1[ECO_RANGE_P1] = (int)(ecoRange[i][ECO_RANGE_P1]*arr_y[i]);
        ecoRange_list_temp1[ECO_RANGE_P2] = (int)(ecoRange[i][ECO_RANGE_P2]*arr_y[i]);

        ecoRange_list_temp2[ECO_RANGE_M2] = (int)(ecoRange[i+1][ECO_RANGE_M2]*arr_y[i+1]);
        ecoRange_list_temp2[ECO_RANGE_M1] = (int)(ecoRange[i+1][ECO_RANGE_M1]*arr_y[i+1]);
        ecoRange_list_temp2[ECO_RANGE_P1] = (int)(ecoRange[i+1][ECO_RANGE_P1]*arr_y[i+1]);
        ecoRange_list_temp2[ECO_RANGE_P2] = (int)(ecoRange[i+1][ECO_RANGE_P2]*arr_y[i+1]);

        ///////
        eco_temp_arr_y[0] = ecoRange_list_temp1[ECO_RANGE_M2];
        eco_temp_arr_y[1] = ecoRange_list_temp2[ECO_RANGE_M2];
        ecoRange_list[ECO_RANGE_M2] = Interpolation_returnY(x, arr_x[i], arr_x[i+1], eco_temp_arr_y[0], eco_temp_arr_y[1]);

        eco_temp_arr_y[0] = ecoRange_list_temp1[ECO_RANGE_M1];
        eco_temp_arr_y[1] = ecoRange_list_temp2[ECO_RANGE_M1];
        ecoRange_list[ECO_RANGE_M1] = Interpolation_returnY(x, arr_x[i], arr_x[i+1], eco_temp_arr_y[0], eco_temp_arr_y[1]);

        eco_temp_arr_y[0] = ecoRange_list_temp1[ECO_RANGE_P1];
        eco_temp_arr_y[1] = ecoRange_list_temp2[ECO_RANGE_P1];
        ecoRange_list[ECO_RANGE_P1] = Interpolation_returnY(x, arr_x[i], arr_x[i+1], eco_temp_arr_y[0], eco_temp_arr_y[1]);

        eco_temp_arr_y[0] = ecoRange_list_temp1[ECO_RANGE_P2];
        eco_temp_arr_y[1] = ecoRange_list_temp2[ECO_RANGE_P2];
        ecoRange_list[ECO_RANGE_P2] = Interpolation_returnY(x, arr_x[i], arr_x[i+1], eco_temp_arr_y[0], eco_temp_arr_y[1]);
        
        break;
      }
    }
  }

  if(y < ecoRange_list[ECO_RANGE_M2])
  {
    rlt_rate = ECO_RATE_VERY_GOOD;
  }
  else if(ecoRange_list[ECO_RANGE_M2] <= y && y < ecoRange_list[ECO_RANGE_M1])
  {
    rlt_rate = ECO_RATE_GOOD;
  }
  else if(ecoRange_list[ECO_RANGE_M1] <= y && y < ecoRange_list[ECO_RANGE_P1])
  {
    rlt_rate = ECO_RATE_NORMAL;
  }
  else if(ecoRange_list[ECO_RANGE_P1] <= y && y < ecoRange_list[ECO_RANGE_P2])
  {
    rlt_rate = ECO_RATE_BAD;
  }
  else
  {
    rlt_rate = ECO_RATE_VERY_BAD;
  }

  return rlt_rate;
}

void MCP2515_ISR()
{
    flag_CanRecv = 1;
}
