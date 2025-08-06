/*
## setup of ros.h
remember to modify the last line of <ros.h> in arduino lib. to the following to ensure a enough buff for msg.
typedef NodeHandle_<ArduinoHardware, 4, 4, 4096, 4096> NodeHandle; // default is 25, 25, 512, 512

Caution one: check the version of ros searial library of arduino (not be 0.7.9, otherwise compile fails?)
Caution two: try overclock of RP2040 in Tools option -> CPU Speed

rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=921600
rostopic pub /pt_configuration std_msgs/UInt16MultiArray "{ data: [10, 1000, 0]}" -1 # adcres sensedelay indicator
rosrun phototactile phototactile.py
*/

#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <vector>

/// lib for jointencoders
#include <SPI.h>

/// Pin Definition
#define STCP 6
#define SHCP 7
#define DS 8

#define SENSOR_MAXCON 4
#define SENSEDS_MAXCON 720
#define SENSE_MAXCON 576 // 8x8x(3x3)

/// Pin Definition for jointencoders

#define JOINT_MAXCON 12

#define CONFIG_SPI_MASTER_DUMMY   0x00
// Register Read Commands
#define ADS_RREG    0x20		//Read n nnnn registers starting at address r rrrr
#define ADS_WREG    0x40		//Write n nnnn registers starting at address r rrrr
#define ADS_WAKEUP  0x02
#define ADS_RESET		0x06
#define ADS_START		0x08		//Start/restart (synchronize) conversions
#define ADS_STOP		0x0A		//Stop conversion
#define ADS_RDATA		0x12		//Read data by command; supports multiple read back.
#define	ADS_INPMUX		0x02
#define	ADS_REF		    0x05
#define	ADS_STATUS		0x01
#define	ADS_DATARATE  0x04

#define ADS_CS_PIN    1
#define ADS_DRDY_PIN  4
#define ADS_START_PIN 5

#define SPI_RX  0
#define SPI_CS  1
#define SPI_CLK 2
#define SPI_TX  3


class ADS124S08
{
public:
  ADS124S08();
  void ADS_Init(void);
  void ADS_Reg_Write(unsigned char READ_WRITE_ADDRESS, unsigned char DATA);
  void ADS_SPI_Command(unsigned char data_in);
  uint32_t ADS_Direct_Read_Data(void);
  uint32_t ADS_Read();
  uint8_t ADS_readRegister(uint8_t address);
  void ADS_setChannel(uint8_t channel);
  void ADS_Check_Cfg_Regs(void);
};

ADS124S08::ADS124S08(void){}

void ADS124S08::ADS_Init(void)
{
  pinMode(ADS_DRDY_PIN, INPUT);
  pinMode(ADS_CS_PIN, OUTPUT);
  pinMode(ADS_START_PIN, OUTPUT);

  digitalWrite(ADS_CS_PIN, LOW);
  digitalWrite(ADS_START_PIN, LOW);
  delay(10);
  digitalWrite(ADS_START_PIN, HIGH);
  delay(100);
  ADS_SPI_Command(ADS_RESET);
  delay(100);
  ADS_Reg_Write(ADS_INPMUX, 0x0C);
  ADS_Reg_Write(ADS_STATUS, 0x00);
  ADS_Reg_Write(ADS_DATARATE, 0x19); // Data rate Hz(bit): 50(5) 60(6) 100(7) 200(8) 400(9) 800(A)
  delay(10);
  ADS_SPI_Command(ADS_START);
}

uint32_t ADS124S08::ADS_Direct_Read_Data()
{
  // if(digitalRead(ADS_DRDY_PIN) == HIGH)  // monitor Data ready(DRDY pin)
  // {
    // while(digitalRead(ADS_DRDY_PIN) == HIGH); 
    uint32_t iData = 0;
    uint8_t rxBuffer[3] = {0};
    digitalWrite(ADS_CS_PIN, LOW);
    // SPI.transfer(ADS_RDATA);
    rxBuffer[0] = SPI.transfer(CONFIG_SPI_MASTER_DUMMY);
    rxBuffer[1] = SPI.transfer(CONFIG_SPI_MASTER_DUMMY);
    rxBuffer[2] = SPI.transfer(CONFIG_SPI_MASTER_DUMMY);
    digitalWrite(ADS_CS_PIN, HIGH);
    iData = (rxBuffer[0] << 16) + (rxBuffer[1] << 8) + rxBuffer[2];
  	return iData;
  // }
  // else return 0;
}

uint32_t ADS124S08::ADS_Read()
{
  uint32_t iData = 0;
  uint8_t rxBuffer[3] = {0};

  digitalWrite(ADS_CS_PIN, LOW);
  SPI.transfer(ADS_RDATA);
  // SPI.transfer(CONFIG_SPI_MASTER_DUMMY);
  rxBuffer[0] = SPI.transfer(CONFIG_SPI_MASTER_DUMMY);
	rxBuffer[1] = SPI.transfer(CONFIG_SPI_MASTER_DUMMY);
  rxBuffer[2] = SPI.transfer(CONFIG_SPI_MASTER_DUMMY);
  // SPI.transfer(CONFIG_SPI_MASTER_DUMMY);
  digitalWrite(ADS_CS_PIN,HIGH);

  iData = (rxBuffer[0] << 16) + (rxBuffer[1] << 8) + rxBuffer[2];
	return iData;
}

uint8_t ADS124S08::ADS_readRegister(uint8_t address)
{
  uint8_t data;
  uint8_t data1;
  digitalWrite(ADS_CS_PIN,LOW);
  SPI.transfer(ADS_RREG|(address & 0x1f));
  SPI.transfer(CONFIG_SPI_MASTER_DUMMY);
  data = SPI.transfer(CONFIG_SPI_MASTER_DUMMY);
  digitalWrite(ADS_CS_PIN,HIGH);
  return data;
}

void ADS124S08::ADS_setChannel(uint8_t channel)
{
  channel = (channel << 4) + 0x0C;
  ADS_Reg_Write(ADS_INPMUX, channel);
}

void ADS124S08::ADS_SPI_Command(unsigned char data_in)
{
  digitalWrite(ADS_CS_PIN, LOW);
  SPI.transfer(data_in);
  digitalWrite(ADS_CS_PIN, HIGH);
}

void ADS124S08::ADS_Check_Cfg_Regs(void)
{
  /// Config Regs Readout
  for(uint8_t add = 0; add < 0x12; add ++)
  {
    Serial.print(" Reg Address ");
    Serial.print(add);
    Serial.print(" : ");
    Serial.println(ADS_readRegister(add));
  }
}

void ADS124S08::ADS_Reg_Write(unsigned char READ_WRITE_ADDRESS, unsigned char DATA)
{
  byte dataToSend = READ_WRITE_ADDRESS + ADS_WREG;
  digitalWrite(ADS_CS_PIN, LOW);
  SPI.transfer(dataToSend);
  SPI.transfer(0x00);
  SPI.transfer(DATA);
  digitalWrite(ADS_CS_PIN, HIGH);
}

unsigned long jointseq = 0;
bool photoupd = false;
uint16_t buff_joints[JOINT_MAXCON] = {};

///
/// The following is for tactile
///
const uint8_t TACTPINS[] = {26, 27, 28, 29};

/// Sensing Unit
// std::vector<uint8_t> USEDTACT = {0, 1, 2, 3}; // 0 to 3
std::vector<uint8_t> USEDTACT = {0}; // 0 to 3

/// Grid configuration
std::vector<uint8_t> SENSEGRID = {1, 2, 3, 4, 5, 6}; // 0 to 7
// std::vector<uint8_t> SENSEGRID = {0, 1, 2, 3, 4, 5, 6, 7}; // 0 to 7

// std::vector<int8_t> SENSEPADX = {0};
// std::vector<int8_t> SENSEPADY = {0};
// std::vector<int8_t> SENSEPADX = {-1, 0, 0, 0, 1};
// std::vector<int8_t> SENSEPADY = { 0,-1, 0, 1, 0};
std::vector<int8_t> SENSEPADX = {-1, -1, -1, 0, 0, 0, 1, 1, 1};
std::vector<int8_t> SENSEPADY = {-1, 0, 1, -1, 0, 1, -1, 0, 1};

/// Configuration buffers
uint8_t TACTSIZE, GRIDLENGTH, PADSIZE = 0;
uint16_t SENSEDS_FIX = 0;
uint16_t buff_sizes[SENSOR_MAXCON] = {}; // {} assign all zeros in C++11
uint16_t buff_digits[SENSOR_MAXCON][SENSEDS_MAXCON] = {}; // {} assign all zeros in C++11
uint16_t buff_digits_last[SENSOR_MAXCON][SENSE_MAXCON] = {}; // {} assign all zeros in C++11

/// Configuration setting
uint8_t HEADERSIZE = 10; // Self; TACTSIZE; TACTID; GRIDSIZE; PADSIZE; ADCRES; SENSEDELAY; SPACING; UDRANGE; ENCODE
uint8_t ADCRES = 12;
uint16_t SENSEDELAY = 200; // us
uint16_t SPACING = 30; // x 0.1 mm, interval between grid lines
uint16_t UDRANGE = 6; // range for updating new values
uint16_t SEQUENCE = 0; // encoding mode for photodigits
uint8_t hardware_version = 2;
uint8_t indicator = 0;
bool req_resetConfig = false;
bool indicator_stay = false;

unsigned long DEGUG, startTime = 0; // Record the current time 

ros::NodeHandle nh;
std_msgs::UInt16MultiArray buff_msg;
ros::Publisher pub_photoarray("hardware_photos", &buff_msg);

std_msgs::UInt16MultiArray buff_msg_joints;
ros::Publisher pub_jointarray("hardware_joints", &buff_msg_joints);

void pt_configuration_callback(const std_msgs::UInt16MultiArray& msg)
{
  ADCRES = msg.data[0];
  SENSEDELAY = msg.data[1];
  UDRANGE = msg.data[2];
  SEQUENCE = msg.data[3];
  hardware_version = msg.data[4];
  indicator = msg.data[5];

  req_resetConfig = true;
}

void pt_gridset_callback(const std_msgs::UInt16MultiArray& msg)
{
  USEDTACT.clear();
  SENSEGRID.clear();
  SENSEPADX.clear();
  SENSEPADY.clear();
  uint8_t idx_num = 0;
  for (uint8_t indx = 0; indx < 4; indx ++){
    if (msg.data[idx_num] != 8) {USEDTACT.push_back(msg.data[idx_num]);}
    idx_num++;
  }
  for (uint8_t indx = 0; indx < 8; indx ++){
    if (msg.data[idx_num] != 8) {SENSEGRID.push_back(msg.data[idx_num]);}
    idx_num++;
  }
  for (uint8_t indx = 0; indx < 9; indx ++){
    if (msg.data[idx_num] != 8) {SENSEPADX.push_back(msg.data[idx_num] - 4);}
    idx_num++;
  }
  for (uint8_t indx = 0; indx < 9; indx ++){
    if (msg.data[idx_num] != 8) {SENSEPADY.push_back(msg.data[idx_num] - 4);}
    idx_num++;
  }
  req_resetConfig = true;
}

void HC_shift_sense_led(uint8_t row_sen, uint8_t col_sen, uint8_t row_led, uint8_t col_led, bool indicator = false, bool fsleep = false)
{
  bool sense_register[8] = {0};
  bool led_register[8] = {0};
  uint8_t ledmap[] = {4, 5, 6, 7, 3, 2, 1, 0,   3, 2, 7, 6, 5, 4, 1, 0}; // row and col shifts
  uint8_t senmap[] = {0, 1, 2, 3, 7, 6, 5, 4,   3, 2, 1, 0, 4, 5, 6, 7}; // row and col shifts
  row_sen = senmap[row_sen];
  col_sen = senmap[8 + col_sen];
  row_led = ledmap[row_led];
  col_led = ledmap[8 + col_led];

  if (hardware_version == 1) {
    /// sensing position shifting
    sense_register[7] = col_sen & 0x01;
    sense_register[6] = col_sen & 0x02;
    sense_register[5] = col_sen & 0x04;
    sense_register[4] = row_sen & 0x01;
    sense_register[3] = row_sen & 0x02;
    sense_register[2] = row_sen & 0x04;
    sense_register[1] = sense_register[2]; // default to last pins
    sense_register[0] = sense_register[2]; // default to last pins

    /// led position shifting
    led_register[7] = row_led & 0x01;
    led_register[6] = row_led & 0x02;
    led_register[5] = row_led & 0x04;
    led_register[4] = col_led & 0x01;
    led_register[3] = col_led & 0x02;
    led_register[2] = col_led & 0x04;
    led_register[1] = led_register[2]; // default to last pins
    led_register[0] = indicator; // default to last pins
  }
  else if (hardware_version == 2) {
    /// sensing position shifting
    sense_register[7] = col_sen & 0x01;
    sense_register[6] = col_sen & 0x02;
    sense_register[5] = col_sen & 0x04;
    sense_register[4] = ! fsleep; // ENRAG1
    sense_register[3] = row_sen & 0x01;
    sense_register[2] = row_sen & 0x02;
    sense_register[1] = row_sen & 0x04;
    sense_register[0] = indicator;

    /// led position shifting
    led_register[7] = row_led & 0x01;
    led_register[6] = row_led & 0x02;
    led_register[5] = row_led & 0x04;
    led_register[4] = ! fsleep; // ENRAG2
    led_register[3] = ! fsleep; // ENCAG2
    led_register[2] = col_led & 0x01;
    led_register[1] = col_led & 0x02;
    led_register[0] = col_led & 0x04;
  }

  /// led position transfer
  for (uint8_t cur_bit = 0; cur_bit < 8; cur_bit ++)
  {
    digitalWrite(SHCP, LOW);
    digitalWrite(DS, led_register[cur_bit]);
    digitalWrite(SHCP, HIGH);
  }
  /// sense position transfer
  for (uint8_t cur_bit = 0; cur_bit < 8; cur_bit ++)
  {
    digitalWrite(SHCP, LOW);
    digitalWrite(DS, sense_register[cur_bit]);
    digitalWrite(SHCP, HIGH);
  }
}

void HC_STCP_activation()
{
  /// storage pin rise to parallel output all pins
  digitalWrite(STCP, LOW);
  digitalWrite(STCP, HIGH);
  digitalWrite(STCP, LOW);
}

ros::Subscriber<std_msgs::UInt16MultiArray> sub_configuration("machine_configuration", pt_configuration_callback);
ros::Subscriber<std_msgs::UInt16MultiArray> sub_grid("machine_gridset", pt_gridset_callback);

void resetConfig()
{
  SEQUENCE = 0;
  /// initialize data buff
  TACTSIZE = USEDTACT.size();
  GRIDLENGTH = SENSEGRID.size();
  PADSIZE = SENSEPADX.size();
  SENSEDS_FIX = HEADERSIZE + GRIDLENGTH + (PADSIZE * 2);
  // SENSEDS_MAX = SENSEDS_FIX + (GRIDLENGTH * GRIDLENGTH) + (GRIDLENGTH * GRIDLENGTH * (PADSIZE));
  // reconf for new sensor pads
  for (uint8_t cnt_tact = 0; cnt_tact < TACTSIZE; cnt_tact ++)
  {
    pinMode(TACTPINS[USEDTACT[cnt_tact]], INPUT);
  }
  analogReadResolution(ADCRES);

  for (uint8_t cnt_tact = 0; cnt_tact < TACTSIZE; cnt_tact ++)
  {
    for (uint16_t cnt_digit = 0; cnt_digit < SENSEDS_MAXCON; cnt_digit ++)
    {
      buff_digits[cnt_tact][cnt_digit] = 0;
    }
    for (uint16_t cnt_digit = 0; cnt_digit < SENSE_MAXCON; cnt_digit ++)
    {
      buff_digits_last[cnt_tact][cnt_digit] = 0;
    }
  }

  uint16_t cnt_photo = 0;
  for (uint8_t cnt_tact = 0; cnt_tact < TACTSIZE; cnt_tact ++)
  {
    buff_digits[cnt_tact][0] = HEADERSIZE;
    buff_digits[cnt_tact][1] = TACTSIZE;
    buff_digits[cnt_tact][2] = USEDTACT[cnt_tact];
    buff_digits[cnt_tact][3] = GRIDLENGTH;
    buff_digits[cnt_tact][4] = PADSIZE;
    buff_digits[cnt_tact][5] = ADCRES;
    buff_digits[cnt_tact][6] = SENSEDELAY;
    buff_digits[cnt_tact][7] = SPACING;
    buff_digits[cnt_tact][8] = UDRANGE;
    buff_digits[cnt_tact][9] = SEQUENCE;
  }
  cnt_photo = HEADERSIZE;
  for (uint8_t grid_r = 0; grid_r < GRIDLENGTH; grid_r ++) {
      for (uint8_t cnt_tact = 0; cnt_tact < TACTSIZE; cnt_tact ++){
        buff_digits[cnt_tact][cnt_photo] = SENSEGRID[grid_r];
      }
      cnt_photo ++;
  }
  for (uint8_t pad_r = 0; pad_r < PADSIZE; pad_r ++) {
      for (uint8_t cnt_tact = 0; cnt_tact < TACTSIZE; cnt_tact ++){
        buff_digits[cnt_tact][cnt_photo] = SENSEPADX[pad_r] + 4; // map (-3 to 3) to (1 to 7) as uint16_t
      }
      cnt_photo ++;
  }
  for (uint8_t pad_r = 0; pad_r < PADSIZE; pad_r ++) {
      for (uint8_t cnt_tact = 0; cnt_tact < TACTSIZE; cnt_tact ++){
        buff_digits[cnt_tact][cnt_photo] = SENSEPADY[pad_r] + 4; // map (-3 to 3) to (1 to 7) as uint16_t
      }
      cnt_photo ++;
  }
  req_resetConfig = false;
}

bool standbymode(bool activate = false)
{
  if ((TACTSIZE == 0) || activate)
  {
    for (uint8_t grid_r = 0; grid_r < 8; grid_r ++) {
      for (uint8_t grid_c = 0; grid_c < 8; grid_c ++) {
        HC_shift_sense_led(grid_r, grid_c, grid_r, grid_c, indicator_stay);
        HC_STCP_activation();
        if ((TACTSIZE == 0) || activate)
        {
          delay(SENSEDELAY / (activate ? 20 : 2));
          nh.spinOnce();
          if (req_resetConfig) resetConfig();
        }else return false;
      }
    }
    indicator_stay = ! indicator_stay;
    return true;
  }else return false;
}

void setup()
{
  nh.initNode();
  nh.advertise(pub_photoarray);
  nh.advertise(pub_jointarray);
  nh.subscribe(sub_configuration);
  nh.subscribe(sub_grid);
  pinMode(STCP, OUTPUT);
  pinMode(SHCP, OUTPUT);
  pinMode(DS, OUTPUT);
  resetConfig();
}

void loop()
{
  if (standbymode()) return;
  
  uint16_t buff_alter[TACTSIZE][GRIDLENGTH*GRIDLENGTH]={}; // {} assign all zeros in C++11
  uint16_t ind_last = 0;
  int upclip, dnclip, angread = 0;
  int8_t shift_r, shift_c = 0;
  unsigned long microsecs = 0;
  int16_t remian_time = 0;
  bool start_HC = true;
  bool end_HC = true;
  int8_t last_pad, last_gridr, last_gridc = 0;

  for (uint8_t cnt_tact = 0; cnt_tact < TACTSIZE; cnt_tact ++){
    buff_sizes[cnt_tact] = SENSEDS_FIX + (GRIDLENGTH * GRIDLENGTH);
  }
  for (uint8_t grid_r = 0; grid_r < GRIDLENGTH; grid_r ++) {
    for (uint8_t grid_c = 0; grid_c < GRIDLENGTH; grid_c ++) {
      for (uint8_t pad = 0; pad < PADSIZE; pad ++) {
        shift_r = SENSEGRID[grid_r] + SENSEPADX[pad];
        shift_c = SENSEGRID[grid_c] + SENSEPADY[pad];
        shift_r = (shift_r >= 0) ? shift_r : - shift_r;
        shift_r = (shift_r <= 7) ? shift_r : 14 - shift_r;
        shift_c = (shift_c >= 0) ? shift_c : - shift_c;
        shift_c = (shift_c <= 7) ? shift_c : 14 - shift_c;

        HC_shift_sense_led(shift_r, shift_c, SENSEGRID[grid_r], SENSEGRID[grid_c], indicator == 0);
        
        if (start_HC)
        {
          HC_STCP_activation(); microsecs = micros(); start_HC = false;
          last_pad = pad; last_gridr = grid_r; last_gridc = grid_c;
          continue;
        }

        /// Warming up the analogread() to remove noise
        // buff_digits[0][buff_sizes[0]] = analogRead(TACTPINS[USEDTACT[0]]);
        READLABLE: // for goto statement to read the final analog photo
        remian_time = (SENSEDELAY - (micros() - microsecs));
        if (remian_time > 0) { delayMicroseconds(remian_time); }
        for (uint8_t cnt_tact = 0; cnt_tact < TACTSIZE; cnt_tact ++)
        {
          angread = analogRead(TACTPINS[USEDTACT[cnt_tact]]);
          upclip = angread - buff_digits_last[cnt_tact][ind_last];
          dnclip = buff_digits_last[cnt_tact][ind_last] - angread;
          if ((upclip >= UDRANGE) || (dnclip >= UDRANGE))
          {
            /// encode pad index to bit 
            buff_alter[cnt_tact][last_gridr*GRIDLENGTH+last_gridc] |= (1 << last_pad);
            buff_digits[cnt_tact][buff_sizes[cnt_tact]] = angread; //angread
            buff_sizes[cnt_tact] ++;
            buff_digits_last[cnt_tact][ind_last] = angread; //angread
          }
        }
        HC_STCP_activation(); microsecs = micros();
        last_pad = pad; last_gridr = grid_r; last_gridc = grid_c;

        ind_last ++;
        // if final iter, go to read again and exit loop
        if (end_HC && (grid_c == (GRIDLENGTH-1)) && (grid_r == (GRIDLENGTH-1)) && (pad == (PADSIZE-1))){
          end_HC = false;
          goto READLABLE;
        }
      }
    }
  }
 
  /// update the resultant buff_alter
  for (uint8_t cnt_tact = 0; cnt_tact < TACTSIZE; cnt_tact ++)
    for (uint16_t grid_cnt = 0; grid_cnt < (GRIDLENGTH * GRIDLENGTH); grid_cnt ++)
      buff_digits[cnt_tact][SENSEDS_FIX + grid_cnt] = buff_alter[cnt_tact][grid_cnt];
  
startTime = micros(); // record the current time
  for (uint8_t cnt_tact = 0; cnt_tact < TACTSIZE; cnt_tact ++)
  {
    while (photoupd) delayMicroseconds(1);
    buff_digits[cnt_tact][1] = DEGUG;
    buff_digits[cnt_tact][9] = SEQUENCE;
    buff_msg.data = buff_digits[cnt_tact];
    buff_msg.data_length = buff_sizes[cnt_tact];
    // pub_photoarray.publish(&buff_msg);
    // nh.spinOnce();
    photoupd = true;
  }
DEGUG = micros() - startTime;
// DEGUG = (uint16_t) (jointangles[1] * 100);
  SEQUENCE ++;
  if (req_resetConfig) resetConfig();
}

ADS124S08 ADS_Device; 
SPISettings spisettings(1000000, MSBFIRST, SPI_MODE1);

void setup1()
{
  SPI.setRX(SPI_RX); 
  SPI.setCS(SPI_CS);
  SPI.setSCK(SPI_CLK);
  SPI.setTX(SPI_TX);
  SPI.begin(true);
  SPI.beginTransaction(spisettings);
  delay(100);

  ADS_Device.ADS_Init();
 }

void loop1()
{
  // float jointangles[12] = {0};
  for(int channel = 0 ;channel < 12; channel++)
  {
    ADS_Device.ADS_setChannel(channel);
    while(digitalRead(ADS_DRDY_PIN) == HIGH) { // monitor Data ready(DRDY pin)
      if (photoupd == true){
        pub_photoarray.publish(&buff_msg);
        nh.spinOnce();
        photoupd = false;
      }
    }
    uint32_t data = ADS_Device.ADS_Direct_Read_Data(); // < 3 ms for 400Hz
    // value = (float) data * 360 / pow(2,23);
    // jointangles[channel] = (uint16_t) (value * 100);
    buff_joints[channel] = (uint16_t) (data >> 8);
  }
  jointseq ++;

  buff_msg_joints.data = buff_joints;
  buff_msg_joints.data_length = JOINT_MAXCON;
  pub_jointarray.publish(&buff_msg_joints);
  nh.spinOnce();
}



