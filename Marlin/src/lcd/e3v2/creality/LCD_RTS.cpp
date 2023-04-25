#include "LCD_RTS.h"
#include <WString.h>
#include <stdio.h>
#include <string.h>
#include <Arduino.h>
#include "../../../MarlinCore.h"
#include "../../../inc/MarlinConfig.h"
#include "../../../module/settings.h"
#include "../../../core/serial.h"
#include "../../../core/macros.h"

#include "../../fontutils.h"
#include "../../../sd/cardreader.h"
#include "../../../feature/powerloss.h"
#include "../../../feature/babystep.h"
#include "../../../module/temperature.h"
#include "../../../module/printcounter.h"
#include "../../../module/motion.h"
#include "../../../module/planner.h"
#include "../../../gcode/queue.h"
#include "../../../gcode/gcode.h"
#include "../../../module/probe.h"

#include "../../../feature/bedlevel/abl/bbl.h"
#include "../../../feature/bedlevel/bedlevel.h"

#if ENABLED(HOST_ACTION_COMMANDS)
  #include "../../../feature/host_actions.h"
#endif

#if ENABLED(ADVANCED_PAUSE_FEATURE)
  #include "../../../feature/pause.h"
  #include "../../../gcode/queue.h"
#endif

#if HAS_FILAMENT_SENSOR
  #include "../../../feature/runout.h"
#endif

#if ENABLED(RTS_AVAILABLE)

//#define CHECKFILEMENT true

float zprobe_zoffset;
float last_zoffset = 0.0;

//const float manual_feedrate_mm_m[] = {50 * 60, 50 * 60, 4 * 60, 60};
const float manual_feedrate_mm_m[] = {100 * 60, 50 * 60, 10 * 60, 120};  // added by John Carlson to speed up travels

int startprogress = 0;
float pause_z = 0;
float pause_e = 0;
bool sdcard_pause_check = true;
bool print_preheat_check = false;
float ChangeFilament0Temp = 200;
float ChangeFilament1Temp = 200;

float current_position_x0_axis = X_MIN_POS;
float current_position_x1_axis = X2_MAX_POS;
int StartFlag = 0;   
int PrintFlag = 0;

int heatway = 0;
millis_t next_rts_update_ms = 0;
int last_target_temperature[4] = {0};
int last_target_temperature_bed;
char waitway = 0;
int recnum = 0;
unsigned char Percentrecord = 0;

bool pause_action_flag = false;
int power_off_type_yes = 0;
// represents to update file list
bool CardUpdate = false;
bool hasSelected = false;
short previousSelectionIndex;

extern CardReader card;
// represents SD-card status, true means SD is available, false means opposite.
bool lcd_sd_status;

char Checkfilenum = 0;
char cmdbuf[20] = {0};
float Filament0LOAD = 10;
float Filament1LOAD = 10;
float XoffsetValue ;

// 1 for 0.1mm, 2 for 1mm, 3 for 10mm, 4 for 100mm
unsigned char AxisUnitMode;
float axis_unit = 10;
unsigned char AutoHomeIconNum;
RTSSHOW rtscheck;
int Update_Time_Value = 0;

// added by John Carlson
int AutoHomeFirstPoint = 0;
bool SoundFlag = false;

bool PoweroffContinue = false;
char commandbuf[30];
bool active_extruder_flag = false;

static int change_page_number = 0; 

char save_dual_x_carriage_mode = 0;

uint16_t remain_time = 0;

static bool last_card_insert_st;
bool card_insert_st;
bool sd_printing;
bool sd_printing_autopause;
inline void RTS_line_to_current(AxisEnum axis)
{
  if (!planner.is_full())
  {
    planner.buffer_line(current_position, MMM_TO_MMS(manual_feedrate_mm_m[(int8_t)axis]), active_extruder);
  }
}

RTSSHOW::RTSSHOW()
{
  recdat.head[0] = snddat.head[0] = FHONE;
  recdat.head[1] = snddat.head[1] = FHTWO;
  memset(databuf, 0, sizeof(databuf));
}

void RTSSHOW::ShowFilesOnCardPage(int page) {
  if (page < 1) {
    page = 1;
  } else if (page > fileInfo.pages) {
    page = fileInfo.pages;
  }

  fileInfo.currentPage = page;

  char pageCount[3];
  char maxPages[3];
  itoa(page, pageCount, 10);
  itoa(fileInfo.pages, maxPages, 10);
  char statStr[7];
  strcpy(statStr, pageCount);
  strcat(statStr, "/");
  strcat(statStr, maxPages);

  for (int h = 0; h < 7; h++) {
    RTS_SndData(0, PAGE_STATUS_TEXT_VP + h);
  }

  RTS_SndData(statStr, PAGE_STATUS_TEXT_VP);

  int max = fileInfo.currentPage*5;
  int min = max-4;
  if (max > fileInfo.fileCount) {
    max = fileInfo.fileCount;
  }

  for (int i = 0; i < 100; i += 20) {
    for (int j = 0; j < 20; j++) {
      RTS_SndData(0, FILE1_TEXT_VP + i + j);
    }
  }

  uint16_t buttonIndex = FILE1_TEXT_VP;
  uint16_t textIndex = 0;

  //enumerate files
  for (int k = min-1; k < max; k++) {
    card.selectFileByIndex(k);
    char shortFileName[20];
    strncpy(shortFileName, card.longest_filename(), 20);

    RTS_SndData(shortFileName, buttonIndex);

    if (!EndsWith(card.longest_filename(), "gcode") && !EndsWith(card.longest_filename(), "GCO") 
        && !EndsWith(card.longest_filename(), "GCODE")) 
    {
      //change color if dir
      RTS_SndData((unsigned long)0x0400, FilenameNature + (textIndex + 5) * 16);
    } else {
      RTS_SndData((unsigned long)0xA514, FilenameNature + (textIndex + 5) * 16);
    }

    textIndex++;
    buttonIndex += 20;
  }
}

void RTSSHOW::InitCardList() {
  for(int j = 0; j < MAXPATHNAMELENGTH; j++)
  {
    fileInfo.currentDir[j] = 0;
  }

  card.getAbsFilenameInCWD(fileInfo.currentDir);
  char shortDirName[20];
  for(int j = 0; j < 20; j++)
  {
    RTS_SndData(0, SELECT_FILE_TEXT_VP + j);
  }

  hasSelected = false;

  strncpy(shortDirName, fileInfo.currentDir, 20);
  RTS_SndData(shortDirName, SELECT_FILE_TEXT_VP);

  fileInfo.fileCount = card.get_num_Files();
  fileInfo.pages = ((fileInfo.fileCount-1) / 5)+1;

  for (uint16_t i = 0; i < 4; i++)
  {
    delay(3);
    RTS_SndData((unsigned long)0xA514, FilenameNature + (i + 5) * 16);
    RTS_SndData(0, FILE1_SELECT_ICON_VP + i);
  }

  ShowFilesOnCardPage(1);
}

void RTSSHOW::RTS_SDCardInit(void)
{
  if(RTS_SD_Detected())
  {
    card.mount();
  }
  if(CardReader::flag.mounted)
  {
    card.cdroot();
    InitCardList();
    lcd_sd_status = IS_SD_INSERTED();
  }
  else
  {
    if(sd_printing_autopause == true)
    {
      RTS_SndData(fileInfo.currentDisplayFilename, PRINT_FILE_TEXT_VP);
      card.mount();
    }
    else
    {
      for(int j = 0;j < 20;j ++)
      {
        // clean screen.
        RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
      }
    }
  }
}

bool RTSSHOW::RTS_SD_Detected(void)
{
  static bool last;
  static bool state;
  static bool flag_stable;
  static uint32_t stable_point_time;

  bool tmp = IS_SD_INSERTED();

  if(tmp != last)
  {
    flag_stable = false;
  }
  else
  {
    if(!flag_stable)
    {
      flag_stable = true;
      stable_point_time = millis();
    }
  }

  if(flag_stable)
  {
    if(millis() - stable_point_time > 30)
    {
      state = tmp;
    }
  }

  last = tmp;

  return state;
}

void RTSSHOW::RTS_SDCardUpate(void)
{
  const bool sd_status = RTS_SD_Detected();
  if (sd_status != lcd_sd_status)
  {
    if (sd_status)
    {
      // SD card power on
      card.mount();
      RTS_SDCardInit();
    }
    else
    {
      card.release();
      if(sd_printing_autopause == true)
      {
        RTS_SndData(fileInfo.currentDisplayFilename, PRINT_FILE_TEXT_VP);
      }
      else
      {
        for(int j = 0;j < 20;j ++)
        {
          // clean screen.
          RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
          RTS_SndData(0, SELECT_FILE_TEXT_VP + j);
        }
      }
    }
    lcd_sd_status = sd_status;
  }

  // represents to update file list
  if(CardUpdate && lcd_sd_status && RTS_SD_Detected())
  {
    card.cdroot();
    InitCardList();
    CardUpdate = false;
  }
}

void RTSSHOW::RTS_Init()
{
  SERIAL_ECHOLNPGM("RTS Init");
  AxisUnitMode = 3;
  active_extruder = active_extruder_font;
  #if ENABLED(DUAL_X_CARRIAGE)
    save_dual_x_carriage_mode = dualXPrintingModeStatus;
    
    if(save_dual_x_carriage_mode == 1)
    {
      RTS_SndData(1, PRINT_MODE_ICON_VP);
      RTS_SndData(1, SELECT_MODE_ICON_VP);
    }
    else if(save_dual_x_carriage_mode == 2)
    {
      RTS_SndData(2, PRINT_MODE_ICON_VP);
      RTS_SndData(2, SELECT_MODE_ICON_VP);
    }
    else if(save_dual_x_carriage_mode == 3)
    {
      RTS_SndData(3, PRINT_MODE_ICON_VP);
      RTS_SndData(3, SELECT_MODE_ICON_VP);
    }
    else if(save_dual_x_carriage_mode == 4)
    {
      RTS_SndData(5, PRINT_MODE_ICON_VP);
      RTS_SndData(5, SELECT_MODE_ICON_VP);
    }
    else 
    {
      RTS_SndData(4, PRINT_MODE_ICON_VP);
      RTS_SndData(4, SELECT_MODE_ICON_VP);
    }
  #endif
  #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
    bool zig = false;
    int8_t inStart, inStop, inInc, showcount;
    showcount = 0;
    //settings.load();
    for (int y = 0; y < GRID_MAX_POINTS_Y; y++)
    {
      // away from origin
      if (zig)
      {
        inStart = 0;
        inStop = GRID_MAX_POINTS_X;
        inInc = 1;
      }
      else
      {
        // towards origin
        inStart = GRID_MAX_POINTS_X - 1;
        inStop = -1;
        inInc = -1;
      }
      zig ^= true;
      for (int x = inStart; x != inStop; x += inInc)
      {
        //SERIAL_ECHOLNPGM("value: ", bedlevel.z_values[x][y] * 1000);
        RTS_SndData(bedlevel.z_values[x][y] * 1000, AUTO_BED_LEVEL_1POINT_VP + showcount * 2);
        showcount++;
      }
    }
    queue.enqueue_now_P(PSTR("M420 S1"));
  #endif

  last_zoffset = zprobe_zoffset = probe.offset.z;
  RTS_SndData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
  RTS_SndData((hotend_offset[1].x - X2_MAX_POS) * 100, TWO_EXTRUDER_HOTEND_XOFFSET_VP);
  RTS_SndData(hotend_offset[1].y * 100, TWO_EXTRUDER_HOTEND_YOFFSET_VP);
  RTS_SndData(hotend_offset[1].z * 100, TWO_EXTRUDER_HOTEND_ZOFFSET_VP);

  last_target_temperature_bed = thermalManager.temp_bed.target;
  last_target_temperature[0] = thermalManager.temp_hotend[0].target;
  last_target_temperature[1] = thermalManager.temp_hotend[1].target;
  feedrate_percentage = 100;
  RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);

  /***************turn off motor*****************/
  RTS_SndData(1, MOTOR_FREE_ICON_VP);

  /***************transmit temperature to screen*****************/
  RTS_SndData(0, HEAD0_SET_TEMP_VP);
  RTS_SndData(0, HEAD1_SET_TEMP_VP);
  RTS_SndData(0, BED_SET_TEMP_VP);
  RTS_SndData(thermalManager.temp_hotend[0].celsius, HEAD0_CURRENT_TEMP_VP);
  RTS_SndData(thermalManager.temp_hotend[1].celsius, HEAD1_CURRENT_TEMP_VP);
  RTS_SndData(thermalManager.temp_bed.celsius, BED_CURRENT_TEMP_VP);

  // added by John Carlson
  /***************transmit E-Steps to screen*******************/
  RTS_SndData(planner.settings.axis_steps_per_mm[3], E0_SET_STEP_VP);
  RTS_SndData(planner.settings.axis_steps_per_mm[4], E1_SET_STEP_VP);
  RTS_SndData(planner.settings.axis_steps_per_mm[0], X_SET_STEP_VP);
  RTS_SndData(planner.settings.axis_steps_per_mm[1], Y_SET_STEP_VP);
  RTS_SndData(planner.settings.axis_steps_per_mm[2], Z_SET_STEP_VP);
  RTS_SndData(planner.flow_percentage[0], E0_SET_FLOW_VP);
  RTS_SndData(planner.flow_percentage[1], E1_SET_FLOW_VP);
  RTS_SndData(thermalManager.fan_speed[0], E0_SET_FAN_VP);
  RTS_SndData(thermalManager.fan_speed[1], E1_SET_FAN_VP);
  #if ENABLED(SOUND_ON_DEFAULT)
    SoundFlag = true;
  #endif

  /***************transmit Fan speed to screen*****************/
  // turn off fans
  thermalManager.set_fan_speed(0, 0);
  thermalManager.set_fan_speed(1, 0);
  RTS_SndData(1, HEAD0_FAN_ICON_VP);
  RTS_SndData(1, HEAD1_FAN_ICON_VP);
  delay(5);

  /*********transmit SD card filename to screen***************/
  RTS_SDCardInit();

  /***************transmit Printer information to screen*****************/
  char sizebuf[20] = {0};
  sprintf(sizebuf, "%d X %d X %d", X_MAX_POS - 2, Y_MAX_POS - 2, Z_MAX_POS);
  RTS_SndData(MACVERSION, PRINTER_MACHINE_TEXT_VP);
  RTS_SndData(SOFTVERSION, PRINTER_VERSION_TEXT_VP);
  RTS_SndData(sizebuf, PRINTER_PRINTSIZE_TEXT_VP);
  RTS_SndData(CORP_WEBSITE, PRINTER_WEBSITE_TEXT_VP);
  //RTS_SndData(Screen_version, Screen_Version_VP);
  /**************************some info init*******************************/
  RTS_SndData(0, PRINT_PROCESS_ICON_VP);
  if(CardReader::flag.mounted)
  {
    change_page_number = 1;
  }
  else
  {
    change_page_number = 0;
  }
}

int RTSSHOW::RTS_RecData()
{
	int frame_index = 0;
	int timeout = 0;
	int framelen = 0;
	bool frame_flag = false;
	if(LCD_SERIAL.available() <= 0){
		return -1;
	}
	do{
		if(LCD_SERIAL.available() > 0){
			databuf[frame_index] = LCD_SERIAL.read();
			timeout = 0;
			/* 0x5A */
			if((frame_index == 0) && (databuf[frame_index] == FHONE)){
				frame_index++;
				continue;
			}
			/* 0xA5 */
			else if(frame_index == 1){
				if(databuf[frame_index] == FHTWO){
					frame_index++;
				}
				else{
					frame_index = 0;
				}
				continue;
			}
			/* 长度 */
			else if(frame_index == 2){
				framelen = databuf[frame_index];
				frame_index++;
				continue;
			}
			else if(frame_index != 0){
				frame_index++;
				/* 一帧数据提取完毕，剩余的串口数据下次进入这个函数会在处理 */
				if(frame_index == (framelen + 3)){
					frame_flag = true;
					break;
				}
			}
		}
		else{
			timeout++;
			delay(1);
		}
	}while(timeout < 50); /* 超时函数 */
//	MYSERIAL0.write(0xBB);
	
	if(frame_flag == true){
		recdat.head[0] = databuf[0];
		recdat.head[1] = databuf[1];
		recdat.len = databuf[2];
		recdat.command = databuf[3];
		for(int idx = 0; idx < frame_index; idx++){
		}

	}
	else{
		return -1;
	}
    // response for writing byte
    if ((recdat.len == 0x03) && 
		((recdat.command == 0x82) || (recdat.command == 0x80)) && 
		(databuf[4] == 0x4F) && 
		(databuf[5] == 0x4B)){
		memset(databuf, 0, sizeof(databuf));
		recnum = 0;
		return -1;
    }
    else if (recdat.command == 0x83){
		// response for reading the data from the variate
		recdat.addr = databuf[4];
		recdat.addr = (recdat.addr << 8) | databuf[5];
		recdat.bytelen = databuf[6];
		for (unsigned int i = 0; i < recdat.bytelen; i += 2){
			recdat.data[i / 2] = databuf[7 + i];
			recdat.data[i / 2] = (recdat.data[i / 2] << 8) | databuf[8 + i];
		}
    }
    else if (recdat.command == 0x81){
		// response for reading the page from the register
		recdat.addr = databuf[4];
		recdat.bytelen = databuf[5];
		for (unsigned int i = 0; i < recdat.bytelen; i ++){
			recdat.data[i] = databuf[6 + i];
			// recdat.data[i] = (recdat.data[i] << 8 )| databuf[7 + i];
		}
    }
	else{
		memset(databuf, 0, sizeof(databuf));
		recnum = 0;
		// receive the wrong data
		return -1;
	}
	memset(databuf, 0, sizeof(databuf));
	recnum = 0;

	return 2;
}

void RTSSHOW::RTS_SndData(void)
{
  if((snddat.head[0] == FHONE) && (snddat.head[1] == FHTWO) && (snddat.len >= 3))
  {
    databuf[0] = snddat.head[0];
    databuf[1] = snddat.head[1];
    databuf[2] = snddat.len;
    databuf[3] = snddat.command;
    // to write data to the register
    if(snddat.command == 0x80)
    {
      databuf[4] = snddat.addr;
      for(int i = 0;i <(snddat.len - 2);i ++)
      {
        databuf[5 + i] = snddat.data[i];
      }
    }
    else if((snddat.len == 3) && (snddat.command == 0x81))
    {
      // to read data from the register
      databuf[4] = snddat.addr;
      databuf[5] = snddat.bytelen;
    }
    else if(snddat.command == 0x82)
    {
      // to write data to the variate
      databuf[4] = snddat.addr >> 8;
      databuf[5] = snddat.addr & 0xFF;
      for(int i =0;i <(snddat.len - 3);i += 2)
      {
        databuf[6 + i] = snddat.data[i/2] >> 8;
        databuf[7 + i] = snddat.data[i/2] & 0xFF;
      }
    }
    else if((snddat.len == 4) && (snddat.command == 0x83))
    {
      // to read data from the variate
      databuf[4] = snddat.addr >> 8;
      databuf[5] = snddat.addr & 0xFF;
      databuf[6] = snddat.bytelen;
    }
     for(int i = 0;i < (snddat.len + 3); i ++)
     {
       LCD_SERIAL.write(databuf[i]);
     }

    memset(&snddat, 0, sizeof(snddat));
    memset(databuf, 0, sizeof(databuf));
    snddat.head[0] = FHONE;
    snddat.head[1] = FHTWO;
  }
}

void RTSSHOW::RTS_SndData(const String &s, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
{
  if(s.length() < 1)
  {
    return;
  }
  RTS_SndData(s.c_str(), addr, cmd);
}

void RTSSHOW::RTS_SndData(const char *str, unsigned long addr, unsigned char cmd/*= VarAddr_W*/)
{
  int len = strlen(str);
  if(len > 0)
  {
    databuf[0] = FHONE;
    databuf[1] = FHTWO;
    databuf[2] = 3 + len;
    databuf[3] = cmd;
    databuf[4] = addr >> 8;
    databuf[5] = addr & 0x00FF;
    for(int i = 0;i < len;i ++)
    {
      databuf[6 + i] = str[i];
    }

    for(int i = 0;i < (len + 6);i ++)
    {
      LCD_SERIAL.write(databuf[i]);
      //delayMicroseconds(1);
    }
    memset(databuf, 0, sizeof(databuf));
  }
}

void RTSSHOW::RTS_SndData(char c, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
{
  snddat.command = cmd;
  snddat.addr = addr;
  snddat.data[0] = (unsigned long)c;
  snddat.data[0] = snddat.data[0] << 8;
  snddat.len = 5;
  RTS_SndData();
}

void RTSSHOW::RTS_SndData(unsigned char *str, unsigned long addr, unsigned char cmd) { RTS_SndData((char *)str, addr, cmd); }

void RTSSHOW::RTS_SndData(int n, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
{
  if (cmd == VarAddr_W)
  {
    if (n > 0xFFFF)
    {
      snddat.data[0] = n >> 16;
      snddat.data[1] = n & 0xFFFF;
      snddat.len = 7;
    }
    else
    {
      snddat.data[0] = n;
      snddat.len = 5;
    }
  }
  else if (cmd == RegAddr_W)
  {
    snddat.data[0] = n;
    snddat.len = 3;
  }
  else if (cmd == VarAddr_R)
  {
    snddat.bytelen = n;
    snddat.len = 4;
  }
  snddat.command = cmd;
  snddat.addr = addr;
  RTS_SndData();
}

void RTSSHOW::RTS_SndData(unsigned int n, unsigned long addr, unsigned char cmd) { RTS_SndData((int)n, addr, cmd); }

void RTSSHOW::RTS_SndData(float n, unsigned long addr, unsigned char cmd) { RTS_SndData((int)n, addr, cmd); }

void RTSSHOW::RTS_SndData(long n, unsigned long addr, unsigned char cmd) { RTS_SndData((unsigned long)n, addr, cmd); }

void RTSSHOW::RTS_SndData(unsigned long n, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
{
  if (cmd == VarAddr_W)
  {
    if (n > 0xFFFF)
    {
      snddat.data[0] = n >> 16;
      snddat.data[1] = n & 0xFFFF;
      snddat.len = 7;
    }
    else
    {
      snddat.data[0] = n;
      snddat.len = 5;
    }
  }
  else if (cmd == VarAddr_R)
  {
    snddat.bytelen = n;
    snddat.len = 4;
  }
  snddat.command = cmd;
  snddat.addr = addr;
  RTS_SndData();
}

void RTSSHOW::RTS_SDcard_Stop()
{
  waitway = 7;
  change_page_number = 1;
  #if ENABLED(DUAL_X_CARRIAGE)
    extruder_duplication_enabled = false;
    dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;
    active_extruder = 0;
  #endif
  card.flag.abort_sd_printing = true;

  #if DISABLED(SD_ABORT_NO_COOLDOWN)
    thermalManager.disable_all_heaters();
  #endif
  print_job_timer.reset();

  thermalManager.setTargetHotend(0, 0);
  RTS_SndData(0, HEAD0_SET_TEMP_VP);
  thermalManager.setTargetHotend(0, 1);
  RTS_SndData(0, HEAD1_SET_TEMP_VP);
  thermalManager.setTargetBed(0);
  RTS_SndData(0, BED_SET_TEMP_VP);
  thermalManager.zero_fan_speeds();
  wait_for_heatup = wait_for_user = false;
  PoweroffContinue = false;
  sd_printing_autopause = false;
  if(CardReader::flag.mounted)
  {
    #if ENABLED(SDSUPPORT) && ENABLED(POWER_LOSS_RECOVERY)
      card.removeJobRecoveryFile();
    #endif
  }
  #ifdef EVENT_GCODE_SD_ABORT
    queue.inject(PSTR(EVENT_GCODE_SD_ABORT));
    // need to fix this..
    SERIAL_ECHOLNPGM("event gcode sd abort finished");
    //queue.inject(PSTR("M84"));
  #endif
  delay(2);
  planner.finish_and_disable();

  // shut down the stepper motor.
  queue.inject(PSTR("M84"));
  RTS_SndData(0, MOTOR_FREE_ICON_VP);

  RTS_SndData(0, PRINT_PROCESS_ICON_VP);
  RTS_SndData(0, PRINT_PROCESS_VP);
  delay(2);
  for(int j = 0;j < 20;j ++)
  {
    // clean screen.
    RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
    // clean filename
    RTS_SndData(0, SELECT_FILE_TEXT_VP + j);
  }
}

void RTSSHOW::RTS_HandleData()
{
  int Checkkey = -1;
  // for waiting
  if(waitway > 0)
  {
    memset(&recdat, 0, sizeof(recdat));
    recdat.head[0] = FHONE;
    recdat.head[1] = FHTWO;
    return;
  }
  for(int i = 0;Addrbuf[i] != 0;i ++)
  {
    if(recdat.addr == Addrbuf[i])
    {
      if(Addrbuf[i] >= ChangePageKey)
      {
        Checkkey = i;
      }
      break;
    }
  }

  if(Checkkey < 0)
  {
    memset(&recdat, 0, sizeof(recdat));
    recdat.head[0] = FHONE;
    recdat.head[1] = FHTWO;
    return;
  }

  char cmd[MAX_CMD_SIZE+16];

  switch(Checkkey)
  {
    case MainPageKey:
      if(recdat.data[0] == 1)
      {
        CardUpdate = true;
        if(CardReader::flag.mounted)
        {
          for (int j = 0; j < 20; j ++)
            {
              RTS_SndData(0, SELECT_FILE_TEXT_VP + j);
            }
          RTS_SndData(ExchangePageBase + 3, ExchangepageAddr);
        }
        else
        {
          RTS_SndData(ExchangePageBase + 47, ExchangepageAddr);
        }
      }
      else if(recdat.data[0] == 2)
      {
        card.flag.abort_sd_printing = true;
        print_job_timer.reset();
        queue.clear();
        quickstop_stepper();
        RTS_SndData(0, MOTOR_FREE_ICON_VP);
        RTS_SndData(0, PRINT_PROCESS_ICON_VP);
        RTS_SndData(0, PRINT_PROCESS_VP);
        delay(2);
        RTS_SndData(0, PRINT_TIME_HOUR_VP);
        RTS_SndData(0, PRINT_TIME_MIN_VP);
        RTS_SndData(0, PRINT_SURPLUS_TIME_HOUR_VP);
        RTS_SndData(0, PRINT_SURPLUS_TIME_MIN_VP);
        RTS_SndData(0, E0_SET_FAN_VP);
        RTS_SndData(0, E1_SET_FAN_VP);

        sd_printing_autopause = false;
        change_page_number = 1;
        RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
      }
      else if(recdat.data[0] == 3)
      {
        thermalManager.fan_speed[0] ? RTS_SndData(0, HEAD0_FAN_ICON_VP) : RTS_SndData(1, HEAD0_FAN_ICON_VP);
        thermalManager.fan_speed[1] ? RTS_SndData(0, HEAD1_FAN_ICON_VP) : RTS_SndData(1, HEAD1_FAN_ICON_VP);
        RTS_SndData(ExchangePageBase + 15, ExchangepageAddr);
      }
      else if(recdat.data[0] == 4)
      {
        RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
      }
      else if(recdat.data[0] == 5)
      {
        #if ENABLED(DUAL_X_CARRIAGE)
          save_dual_x_carriage_mode = dualXPrintingModeStatus;
          
          SetExtruderMode(save_dual_x_carriage_mode, true);

          RTS_SndData(ExchangePageBase + 34, ExchangepageAddr);
        #endif
      }
      break;

    case AdjustmentKey:
      if(recdat.data[0] == 1)
      {
        thermalManager.fan_speed[0] ? RTS_SndData(0, HEAD0_FAN_ICON_VP) : RTS_SndData(1, HEAD0_FAN_ICON_VP);
        thermalManager.fan_speed[1] ? RTS_SndData(0, HEAD1_FAN_ICON_VP) : RTS_SndData(1, HEAD1_FAN_ICON_VP);
        RTS_SndData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
        RTS_SndData(ExchangePageBase + 14, ExchangepageAddr);
      }
      else if(recdat.data[0] == 2)
      {
        if(PrintFlag == 1)
          RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
        else
          RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);
        //settings.save();
      }
      else if(recdat.data[0] == 3)
      {
        if (thermalManager.fan_speed[0])
        {
          RTS_SndData(1, HEAD0_FAN_ICON_VP);
          RTS_SndData(0, E0_SET_FAN_VP);
          thermalManager.set_fan_speed(0, 0);
        }
        else
        {
          RTS_SndData(0, HEAD0_FAN_ICON_VP);
          RTS_SndData(255, E0_SET_FAN_VP);
          thermalManager.set_fan_speed(0, 255);
        }
      }
      else if(recdat.data[0] == 4)
      {
        if (thermalManager.fan_speed[1])
        {
          RTS_SndData(1, HEAD1_FAN_ICON_VP);
          RTS_SndData(0, E1_SET_FAN_VP);
          thermalManager.set_fan_speed(1, 0);
        }
        else
        {
          RTS_SndData(0, HEAD1_FAN_ICON_VP);
          RTS_SndData(255, E1_SET_FAN_VP);
          thermalManager.set_fan_speed(1, 255);
        }
      }
      break;

    case PrintSpeedKey:
      feedrate_percentage = recdat.data[0];
      RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
      break;

    case StopPrintKey:
      if((recdat.data[0] == 1) || (recdat.data[0] == 0xF1))
      {
        RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
        RTS_SndData(0, PRINT_TIME_HOUR_VP);
        RTS_SndData(0, PRINT_TIME_MIN_VP);
        RTS_SndData(0, PRINT_SURPLUS_TIME_HOUR_VP);
        RTS_SndData(0, PRINT_SURPLUS_TIME_MIN_VP);
        RTS_SndData(0, E0_SET_FAN_VP);
        RTS_SndData(0, E1_SET_FAN_VP);
        RTS_SDcard_Stop();
        Update_Time_Value = 0;
        PrintFlag = 0;
        TERN_(HOST_PAUSE_M76, hostui.cancel());
        queue.inject_P(PSTR("M77"));
      }
      else if(recdat.data[0] == 0xF0)
      {
        if(card.isPrinting)
        {
          if (print_job_timer.isRunning) {
            RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);
          } else {
            RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
          }
        }
        else if(sdcard_pause_check == false)
        {
          RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
        }
      }
      break;

    case PausePrintKey:
      if(recdat.data[0] == 0xF0)
      {
        break;
      }
      else if(recdat.data[0] == 0xF1)
      {
        RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
        //pause command
        waitway = 1;
        pause_z = current_position[Z_AXIS];
        card.pauseSDPrint();
        pause_action_flag = true;
        Update_Time_Value = 0;
        sdcard_pause_check = false;
        PrintFlag = 1;
        change_page_number = 12;
        queue.inject_P(PSTR("M76"));
      }
      break;

    case ResumePrintKey:
      #if BOTH(M600_PURGE_MORE_RESUMABLE, ADVANCED_PAUSE_FEATURE)
        pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;  // Simulate menu selection
      #endif
      wait_for_user = false;
      if(recdat.data[0] == 1)
      {
        //resume from paused
        RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
        card.startOrResumeFilePrinting();
        Update_Time_Value = 0;
        sdcard_pause_check = true;
        pause_action_flag = false;
        RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);
        PrintFlag = 2;
        queue.enqueue_now_P(PSTR("M75"));
        TERN_(HOST_PAUSE_M76, hostui.resume());
      }
      else if(recdat.data[0] == 2)
      {
        //change filament and resume
        RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
        Update_Time_Value = 0;
        pause_action_flag = false;
        sdcard_pause_check = true;
        PrintFlag = 2;
        RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);
        TERN_(HOST_PAUSE_M76, hostui.resume());
      }
      else if(recdat.data[0] == 3)
      {
        //heat and change filament, and resume
        if(PoweroffContinue == true)
        {
          RTS_SndData(ExchangePageBase + 8, ExchangepageAddr);
          PoweroffContinue = false; // added by John Carlson to reset the flag
        }
        else if(PoweroffContinue == false)
        {
          char cmd[MAX_CMD_SIZE+16];
          char *c;
          sprintf_P(cmd, PSTR("M23 %s"), fileInfo.currentFilePath);
          SERIAL_ECHOLNPGM("cmd :", cmd);
          for (c = &cmd[4]; *c; c++)
            *c = tolower(*c);

          //queue.enqueue_one_now(cmd);
          //SERIAL_ECHOLNPGM("cmd after:", cmd);
          queue.inject(cmd);
          delay(20);
          //queue.enqueue_now_P(PSTR("M24"));
          //SERIAL_ECHOLNPGM("run m24");
          queue.inject(PSTR("M24"));
          // clean screen.
          for (int j = 0; j < 20; j ++)
          {
            RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
          }
          RTS_SndData(fileInfo.currentDisplayFilename, PRINT_FILE_TEXT_VP);
          delay(2);
          #if ENABLED(BABYSTEPPING)
            RTS_SndData(0, AUTO_BED_LEVEL_ZOFFSET_VP);
          #endif
          feedrate_percentage = 100;
          RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
          zprobe_zoffset = last_zoffset;
          RTS_SndData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
          PoweroffContinue = true;
          RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);
          sdcard_pause_check = true;
        }
      }
      else if(recdat.data[0] == 4)
      {
        //replaced SD card and continue
        if(!CardReader::flag.mounted)
        {
          CardUpdate = true;
          RTS_SDCardUpate();
          RTS_SndData(ExchangePageBase + 46, ExchangepageAddr);
        }
        else
        {
          RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);

          card.startOrResumeFilePrinting();
          Update_Time_Value = 0;
          sdcard_pause_check = true;
          pause_action_flag = false;
          PrintFlag = 2;

          if (card.isPrinting) {
            rtscheck.RTS_SndData(fileInfo.currentDisplayFilename, PRINT_FILE_TEXT_VP);
          }

          sd_printing_autopause = true;
          RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);
          queue.enqueue_now_P(PSTR("M75"));
          TERN_(HOST_PAUSE_M76, hostui.resume());
        }
      }
      break;

    case TempScreenKey:
      if (recdat.data[0] == 1)
      {
        if (thermalManager.fan_speed[0] == 0)
        {
          RTS_SndData(1, HEAD0_FAN_ICON_VP);
        }
        else
        {
          RTS_SndData(0, HEAD0_FAN_ICON_VP);
        }
        RTS_SndData(ExchangePageBase + 16, ExchangepageAddr);
      }
      else if (recdat.data[0] == 2)
      {
        if (thermalManager.fan_speed[1] == 0)
        {
          RTS_SndData(1, HEAD1_FAN_ICON_VP);
        }
        else
        {
          RTS_SndData(0, HEAD1_FAN_ICON_VP);
        }
        RTS_SndData(ExchangePageBase + 17, ExchangepageAddr);
      }
      else if (recdat.data[0] == 3)
      {
        RTS_SndData(ExchangePageBase + 18, ExchangepageAddr);
      }
      else if (recdat.data[0] == 4)
      {
        RTS_SndData(ExchangePageBase + 15, ExchangepageAddr);
      }
      else if (recdat.data[0] == 0xF1)
      {
        #if FAN_COUNT > 0
          for (uint8_t i = 0; i < FAN_COUNT; i++)
          {
            thermalManager.fan_speed[i] = 255;
          }
        #endif

        thermalManager.setTargetHotend(0, 0);
        RTS_SndData(0, HEAD0_SET_TEMP_VP);
        delay(1);
        thermalManager.setTargetHotend(0, 1);
        RTS_SndData(0, HEAD1_SET_TEMP_VP);
        delay(1);
        thermalManager.setTargetBed(0);
        RTS_SndData(0, BED_SET_TEMP_VP);
        delay(1);

        RTS_SndData(ExchangePageBase + 15, ExchangepageAddr);
      }
      else if (recdat.data[0] == 0xF0)
      {
        RTS_SndData(ExchangePageBase + 15, ExchangepageAddr);
      }
      break;

    case CoolScreenKey:
      if (recdat.data[0] == 1)
      {
        thermalManager.setTargetHotend(0, 0);
        RTS_SndData(0, HEAD0_SET_TEMP_VP);
        thermalManager.fan_speed[0] = 255;
        RTS_SndData(0, HEAD0_FAN_ICON_VP);
      }
      else if (recdat.data[0] == 2)
      {
        thermalManager.setTargetBed(0);
        RTS_SndData(0, BED_SET_TEMP_VP);
      }
      else if (recdat.data[0] == 3)
      {
        thermalManager.setTargetHotend(0, 1);
        RTS_SndData(0, HEAD1_SET_TEMP_VP);
        thermalManager.fan_speed[1] = 255;
        RTS_SndData(0, HEAD1_FAN_ICON_VP);
      }
      else if (recdat.data[0] == 4)
      {
        RTS_SndData(ExchangePageBase + 15, ExchangepageAddr);
      }
      else if (recdat.data[0] == 5)
      {
        thermalManager.temp_hotend[0].target = PREHEAT_1_TEMP_HOTEND;
        thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
        RTS_SndData(thermalManager.temp_hotend[0].target, HEAD0_SET_TEMP_VP);
        thermalManager.temp_bed.target = PREHEAT_1_TEMP_BED;
        thermalManager.setTargetBed(thermalManager.temp_bed.target);
        RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
      }
      else if (recdat.data[0] == 6)
      {
        thermalManager.temp_hotend[0].target = PREHEAT_2_TEMP_HOTEND;
        thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
        RTS_SndData(thermalManager.temp_hotend[0].target, HEAD0_SET_TEMP_VP);
        thermalManager.temp_bed.target = PREHEAT_2_TEMP_BED;
        thermalManager.setTargetBed(thermalManager.temp_bed.target);
        RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
      }
      else if (recdat.data[0] == 7)
      {
        thermalManager.temp_hotend[1].target = PREHEAT_1_TEMP_HOTEND;
        thermalManager.setTargetHotend(thermalManager.temp_hotend[1].target, 1);
        RTS_SndData(thermalManager.temp_hotend[1].target, HEAD1_SET_TEMP_VP);
        thermalManager.temp_bed.target = PREHEAT_1_TEMP_BED;
        thermalManager.setTargetBed(thermalManager.temp_bed.target);
        RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
      }
      else if (recdat.data[0] == 8)
      {
        thermalManager.temp_hotend[1].target = PREHEAT_2_TEMP_HOTEND;
        thermalManager.setTargetHotend(thermalManager.temp_hotend[1].target, 1);
        RTS_SndData(thermalManager.temp_hotend[1].target, HEAD1_SET_TEMP_VP);
        thermalManager.temp_bed.target = PREHEAT_2_TEMP_BED;
        thermalManager.setTargetBed(thermalManager.temp_bed.target);
        RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
      }
      else if (recdat.data[0] == 9)
      {
        thermalManager.temp_bed.target = PREHEAT_1_TEMP_BED;
        thermalManager.setTargetBed(thermalManager.temp_bed.target);
        RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
      }
      else if (recdat.data[0] == 10)
      {
        thermalManager.temp_bed.target = PREHEAT_2_TEMP_BED;
        thermalManager.setTargetBed(thermalManager.temp_bed.target);
        RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
      }
      break;

    case Heater0TempEnterKey:
      int h1tgt;
      if (recdat.data[0] >= HEATER_0_MAXTEMP) {
        h1tgt = HEATER_0_MAXTEMP;
      } else {
        h1tgt = recdat.data[0];
      }
      thermalManager.temp_hotend[0].target = h1tgt;
      thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
      RTS_SndData(thermalManager.temp_hotend[0].target, HEAD0_SET_TEMP_VP);
      break;

    case Heater1TempEnterKey:
      int h2tgt;
      if (recdat.data[0] >= HEATER_1_MAXTEMP) {
        h2tgt = HEATER_1_MAXTEMP;
      } else {
        h2tgt = recdat.data[0];
      }
      thermalManager.temp_hotend[1].target = h2tgt;
      thermalManager.setTargetHotend(thermalManager.temp_hotend[1].target, 1);
      RTS_SndData(thermalManager.temp_hotend[1].target, HEAD1_SET_TEMP_VP);
      break;

    case HotBedTempEnterKey:
      int hbtgt;
      if (recdat.data[0] >= BED_MAXTEMP) {
        hbtgt = BED_MAXTEMP;
      } else {
        hbtgt = recdat.data[0];
      }
      thermalManager.temp_bed.target = hbtgt;
      thermalManager.setTargetBed(thermalManager.temp_bed.target);
      RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
      break;

    case Heater0LoadEnterKey:
      Filament0LOAD = ((float)recdat.data[0]) / 10;
      break;

    case Heater1LoadEnterKey:
      Filament1LOAD = ((float)recdat.data[0]) / 10;
      break;

    // added by John Carlson for updating e-steps
    case E0StepsKey:
      sprintf_P(cmd, PSTR("M92 E%d T0"), recdat.data[0]);
      queue.enqueue_now_P(cmd);
      queue.enqueue_now_P(PSTR("M500"));
      RTS_SndData(StartSoundSet, SoundAddr);
      RTS_SndData(recdat.data[0], E0_SET_STEP_VP);
      break;

    case E1StepsKey:
      sprintf_P(cmd, PSTR("M92 E%d T1"), recdat.data[0]);
      queue.enqueue_now_P(cmd);
      queue.enqueue_now_P(PSTR("M500"));
      RTS_SndData(StartSoundSet, SoundAddr);
      RTS_SndData(recdat.data[0], E1_SET_STEP_VP);
      break;

    case XStepsKey:
      sprintf_P(cmd, PSTR("M92 X%d"), recdat.data[0]);
      queue.enqueue_now_P(cmd);
      queue.enqueue_now_P(PSTR("M500"));
      RTS_SndData(StartSoundSet, SoundAddr);
      RTS_SndData(recdat.data[0], X_SET_STEP_VP);
      break;

    case YStepsKey:
      sprintf_P(cmd, PSTR("M92 Y%d"), recdat.data[0]);
      queue.enqueue_now_P(cmd);
      queue.enqueue_now_P(PSTR("M500"));
      RTS_SndData(StartSoundSet, SoundAddr);
      RTS_SndData(recdat.data[0], Y_SET_STEP_VP);
      break;

    case ZStepsKey:
      sprintf_P(cmd, PSTR("M92 Z%d"), recdat.data[0]);
      queue.enqueue_now_P(cmd);
      queue.enqueue_now_P(PSTR("M500"));
      RTS_SndData(StartSoundSet, SoundAddr);
      RTS_SndData(recdat.data[0], Z_SET_STEP_VP);
      break;
    // end updating for e-steps
    // added by John Carlson for updating flow
    case E0FlowKey:
      /*sprintf_P(cmd2, PSTR("M221 S%d T0"), recdat.data[0]);
      queue.enqueue_one_now(cmd2);*/
      planner.set_flow(0, recdat.data[0]);
      RTS_SndData(StartSoundSet, SoundAddr);
      RTS_SndData(recdat.data[0], E0_SET_FLOW_VP);
      break;

    case E1FlowKey:
      /*sprintf_P(cmd2, PSTR("M221 S%d T1"), recdat.data[0]);
      queue.enqueue_one_now(cmd2);*/
      planner.set_flow(1, recdat.data[0]);
      RTS_SndData(StartSoundSet, SoundAddr);
      RTS_SndData(recdat.data[0], E1_SET_FLOW_VP);
      break;
    // end updating for flow
    // added by John Carlson for updating fan speed
    case E0FanKey:
      thermalManager.set_fan_speed(0, recdat.data[0]);
      RTS_SndData(StartSoundSet, SoundAddr);
      RTS_SndData(recdat.data[0], E0_SET_FAN_VP);
      if (recdat.data[0] > 0) {
        RTS_SndData(0, HEAD0_FAN_ICON_VP);
      } else {
        RTS_SndData(1, HEAD0_FAN_ICON_VP);
      }
      break;

    case E1FanKey:
      thermalManager.set_fan_speed(1, recdat.data[0]);
      RTS_SndData(StartSoundSet, SoundAddr);
      RTS_SndData(recdat.data[0], E1_SET_FAN_VP);
      if (recdat.data[0] > 0) {
        RTS_SndData(0, HEAD1_FAN_ICON_VP);
      } else {
        RTS_SndData(1, HEAD1_FAN_ICON_VP);
      }
      break;
    // end updating for fan speed

    case ZOffsetKey: // manual type zoffset
      SERIAL_ECHOLNPGM("ZOffsetKey Value: ", recdat.data[0]);
      float tmp_zprobe_offset;

      if (recdat.data[0] >= 32768)
      {
        tmp_zprobe_offset = (recdat.data[0] - 65536) / 100.0;
      }
      else
      {
        tmp_zprobe_offset = (recdat.data[0]) / 100.0;
      }
      last_zoffset = zprobe_zoffset;
      //SERIAL_ECHOLNPGM("Last ZOffset: ", last_zoffset);
      if (WITHIN((tmp_zprobe_offset), Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX))
      {
        zprobe_zoffset = tmp_zprobe_offset;
        //SERIAL_ECHOLNPGM("Add babbystep: ", tmp_zprobe_offset - last_zoffset);
        babystep.add_mm(Z_AXIS, tmp_zprobe_offset - last_zoffset);
        probe.offset.z = zprobe_zoffset;
      }
      RTS_SndData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);

      break;

    case ChangeFilamentKey: //FILAMENT_RUNOUT_SCRIPT
      char cmdF[MAX_CMD_SIZE+16];
      sprintf_P(cmdF, PSTR(FILAMENT_RUNOUT_SCRIPT), recdat.data[0]);
      queue.inject(cmdF);
      delay(2);
      break;

    case AxisPageSelectKey:
      if(recdat.data[0] == 5)
      {
        AxisUnitMode = 4;
        axis_unit = 100.0;
        RTS_SndData(ExchangePageBase + 58, ExchangepageAddr);
      } 
      else if(recdat.data[0] == 1)
      {
        AxisUnitMode = 3;
        axis_unit = 10.0;
        RTS_SndData(ExchangePageBase + 29, ExchangepageAddr);
      }
      else if(recdat.data[0] == 2)
      {
        AxisUnitMode = 2;
        axis_unit = 1.0;
        RTS_SndData(ExchangePageBase + 30, ExchangepageAddr);
      }
      else if(recdat.data[0] == 3)
      {
        AxisUnitMode = 1;
        axis_unit = 0.01;
        RTS_SndData(ExchangePageBase + 31, ExchangepageAddr);
      }
      else if(recdat.data[0] == 4)
      {
        waitway = 4;
        AutoHomeIconNum = 0;
        queue.enqueue_now_P(PSTR("G28"));
        Update_Time_Value = 0;
        RTS_SndData(ExchangePageBase + 32, ExchangepageAddr);
        RTS_SndData(0, MOTOR_FREE_ICON_VP);
      }
      break;

    case SettingScreenKey:
      SERIAL_ECHOLNPGM("Settings Button ID: ", recdat.data[0]);
      if(recdat.data[0] == 1)
      {
        // Motor Icon
        RTS_SndData(0, MOTOR_FREE_ICON_VP);
        // only for prohibiting to receive massage
        waitway = 6;
        AutoHomeIconNum = 0;
        active_extruder = 0;
        active_extruder_flag = false;
        active_extruder_font = active_extruder;
        Update_Time_Value = 0;
        queue.enqueue_now_P(PSTR("G28"));
        queue.enqueue_now_P(PSTR("G1 F1000 Y150.0")); // added to move nozzle to probe position - John Carlson
        queue.enqueue_now_P(PSTR("G1 F200 Z0.0"));
        RTS_SndData(ExchangePageBase + 32, ExchangepageAddr);

        if (active_extruder == 0)
        {
          RTS_SndData(0, EXCHANGE_NOZZLE_ICON_VP);
        }
        else
        {
          RTS_SndData(1, EXCHANGE_NOZZLE_ICON_VP);
        }
      }
      else if(recdat.data[0] == 2)
      {
        Filament0LOAD = 10;
        Filament1LOAD = 10;
        RTS_SndData(10 * Filament0LOAD, HEAD0_FILAMENT_LOAD_DATA_VP);
        RTS_SndData(10 * Filament1LOAD, HEAD1_FILAMENT_LOAD_DATA_VP);
        RTS_SndData(thermalManager.temp_hotend[0].celsius, HEAD0_CURRENT_TEMP_VP);
        thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
        RTS_SndData(thermalManager.temp_hotend[0].target, HEAD0_SET_TEMP_VP);

        RTS_SndData(thermalManager.temp_hotend[1].celsius, HEAD1_CURRENT_TEMP_VP);
        thermalManager.setTargetHotend(thermalManager.temp_hotend[1].target, 1);
        RTS_SndData(thermalManager.temp_hotend[1].target, HEAD1_SET_TEMP_VP);

        delay(2);
        RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
      }
      else if (recdat.data[0] == 3)
      {
        if(active_extruder == 0)
        {
          RTS_SndData(0, EXCHANGE_NOZZLE_ICON_VP);
          active_extruder_flag = false;
        }
        else if(active_extruder == 1)
        {
          RTS_SndData(1, EXCHANGE_NOZZLE_ICON_VP);
          active_extruder_flag = true;
        }

        AxisUnitMode = 3;
        if(active_extruder == 0)
        {
          if(TEST(axes_trusted, X_AXIS))
          {
            current_position_x0_axis = current_position[X_AXIS];
          }
          else
          {
            current_position[X_AXIS] = current_position_x0_axis;
          }
          RTS_SndData(10 * current_position_x0_axis, AXIS_X_COORD_VP);
          memset(commandbuf, 0, sizeof(commandbuf));
          sprintf_P(commandbuf, PSTR("G92.9 X%6.3f"), current_position_x0_axis);
          queue.enqueue_one_now(commandbuf);
        }
        else if(active_extruder == 1)
        {
          if(TEST(axes_trusted, X_AXIS))
          {
            current_position_x1_axis = current_position[X_AXIS];
          }
          else
          {
            current_position[X_AXIS] = current_position_x1_axis;
          }
          RTS_SndData(10 * current_position_x1_axis, AXIS_X_COORD_VP);
          memset(commandbuf, 0, sizeof(commandbuf));
          sprintf_P(commandbuf, PSTR("G92.9 X%6.3f"), current_position_x1_axis);
          queue.enqueue_one_now(commandbuf);
        }
        RTS_SndData(10 * current_position[Y_AXIS], AXIS_Y_COORD_VP);
        RTS_SndData(10 * current_position[Z_AXIS], AXIS_Z_COORD_VP);

        RTS_SndData(ExchangePageBase + 29, ExchangepageAddr);
      }
      else if (recdat.data[0] == 4)
      {
        RTS_SndData(ExchangePageBase + 35, ExchangepageAddr);
      }
      else if (recdat.data[0] == 5)
      {
        RTS_SndData(CORP_WEBSITE, PRINTER_WEBSITE_TEXT_VP);
        //RTS_SndData(Screen_version, Screen_Version_VP);
        RTS_SndData(ExchangePageBase + 33, ExchangepageAddr);
      }
      else if (recdat.data[0] == 6)
      {
        queue.enqueue_now_P(PSTR("M84"));
        RTS_SndData(1, MOTOR_FREE_ICON_VP);
      }
      else if (recdat.data[0] == 7)
      {
        RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
      }
            else if (recdat.data[0] == 8) // switch to advanced settings page 
      {
        RTS_SndData(ExchangePageBase + 83, ExchangepageAddr);
      }
      break;

    case ASettingsScreenKey:
      SERIAL_ECHOLNPGM("ASettings Button ID: ", recdat.data[0]);
      if (recdat.data[0] == 1) // switch to e-steps page 
      {
        RTS_SndData(ExchangePageBase + 82, ExchangepageAddr);
      }
      else if (recdat.data[0] == 2) // switch to flow settings page 
      {
        RTS_SndData(ExchangePageBase + 84, ExchangepageAddr);
      }
      else if (recdat.data[0] == 3) // switch to pid tuning page)
      {
        RTS_SndData(ExchangePageBase + 85, ExchangepageAddr);
      }
      else if (recdat.data[0] == 9) // switch back to settings page
      {
        RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
      }
      break;

    case PIDScreenKey:
      SERIAL_ECHOLNPGM("PIDScreenkey Button ID: ", recdat.data[0]);
      RTS_SndData(ExchangePageBase + 86, ExchangepageAddr);
      RTS_SndData("                     ", PID_TEXT_OUT_VP);
      if (recdat.data[0] == 1) // nozzle 1 page 
      {
        const heater_id_t hid = (heater_id_t)0;
        thermalManager.PID_autotune(200, hid, 5, 1);
      }
      else if (recdat.data[0] == 2) // nozzle 2 page 
      {
        const heater_id_t hid = (heater_id_t)1;
        thermalManager.PID_autotune(200, hid, 5, 1);
      }
      else if (recdat.data[0] == 3) // bed page 
      {
        const heater_id_t hid = (heater_id_t)-1;
        thermalManager.PID_autotune(60, hid, 5, 1);
      }
      else if (recdat.data[0] == 9)
      {
        RTS_SndData(ExchangePageBase + 85, ExchangepageAddr);
      }
      break;

    case SettingBackKey:
      if (recdat.data[0] == 1)
      {
        Update_Time_Value = RTS_UPDATE_VALUE;
        SetExtruderMode(save_dual_x_carriage_mode, true);
        RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
      }
      else if (recdat.data[0] == 2)
      {
        if(!planner.has_blocks_queued())
        {
          #if ENABLED(HAS_LEVELING)
            RTS_SndData(ExchangePageBase + 22, ExchangepageAddr);
          #else
            RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
          #endif
          queue.enqueue_now_P(PSTR("M420 S1"));
        }
      }
      else if (recdat.data[0] == 3)
      {
        memset(commandbuf, 0, sizeof(commandbuf));
        sprintf_P(commandbuf, PSTR("M218 T1 X%4.1f"), hotend_offset[1].x);
        queue.enqueue_now_P(commandbuf);
        delay(5);
        memset(commandbuf, 0, sizeof(commandbuf));
        sprintf_P(commandbuf, PSTR("M218 T1 Y%4.1f"), hotend_offset[1].y);
        queue.enqueue_now_P(commandbuf);
        delay(5);
        memset(commandbuf, 0, sizeof(commandbuf));
        sprintf_P(commandbuf, PSTR("M218 T1 Z%4.1f"), hotend_offset[1].z);
        queue.enqueue_now_P(commandbuf);
        //settings.save();
      } else if (recdat.data[0] == 4) {
          #if ENABLED(HAS_LEVELING)
            RTS_SndData(ExchangePageBase + 28, ExchangepageAddr);
          #else
            RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
          #endif
      }
      break;

    case BedLevelFunKey:
      if (recdat.data[0] == 1)
      {
        waitway = 6;
        if((active_extruder == 1) || (!TEST(axes_trusted, X_AXIS)) || (!TEST(axes_trusted, Y_AXIS)))
        {
          AutoHomeIconNum = 0;
          active_extruder = 0;
          active_extruder_flag = false;
          active_extruder_font = active_extruder;
          queue.enqueue_now_P(PSTR("G28"));
          RTS_SndData(ExchangePageBase + 32, ExchangepageAddr);
        }
        else
        {
          queue.enqueue_now_P(PSTR("G28 Z0"));
        }
        queue.enqueue_now_P(PSTR("G1 F1000 Y150.0")); // added to move nozzle to probe position - John Carlson
        queue.enqueue_now_P(PSTR("G1 F200 Z0.0"));
      }
      else if (recdat.data[0] == 2)
      {
        last_zoffset = zprobe_zoffset;
        if (WITHIN((zprobe_zoffset + 0.01), Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX))
        {
          #if ENABLED(HAS_LEVELING)
            zprobe_zoffset = (zprobe_zoffset + 0.01);
            zprobe_zoffset = zprobe_zoffset + 0.00001;
          #endif
          babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
          probe.offset.z = zprobe_zoffset;
        }
        RTS_SndData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
      }
      else if (recdat.data[0] == 3)
      {
        last_zoffset = zprobe_zoffset;
        if (WITHIN((zprobe_zoffset - 0.01), Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX))
        {
          #if ENABLED(HAS_LEVELING)
            zprobe_zoffset = (zprobe_zoffset - 0.01);
            zprobe_zoffset = zprobe_zoffset - 0.00001;
          #endif
          babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
          probe.offset.z = zprobe_zoffset;
        }
        RTS_SndData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
      }
      else if (recdat.data[0] == 4)
      {
        RTS_SndData(ExchangePageBase + 28, ExchangepageAddr);
        if(active_extruder == 0)
        {
          RTS_SndData(0, EXCHANGE_NOZZLE_ICON_VP);
        }
        else if(active_extruder == 1)
        {
          RTS_SndData(1, EXCHANGE_NOZZLE_ICON_VP);
        }
        queue.enqueue_now_P(PSTR("M420 S0"));
      }
      else if (recdat.data[0] == 5)
      {
        #if ENABLED(BLTOUCH)
          waitway = 3;
          // new bed level page added by John Carlson
          RTS_SndData(AutoHomeFirstPoint, AUTO_BED_LEVEL_CUR_POINT_VP);
          RTS_SndData(GRID_MAX_POINTS_X * GRID_MAX_POINTS_Y, AUTO_BED_LEVEL_END_POINT);
          RTS_SndData(ExchangePageBase + 80, ExchangepageAddr);
          if (!all_axes_trusted()) {
            queue.enqueue_now_P(PSTR("G28"));
          }
          queue.enqueue_now_P(PSTR("G29"));
        #endif
      }
      if (recdat.data[0] == 6)
      {
        // Assitant Level ,  Centre 1
        if(!planner.has_blocks_queued())
        {
          waitway = 4;
          queue.enqueue_now_P(PSTR("G1 F600 Z3"));
          queue.enqueue_now_P(PSTR("G1 X150 Y150 F3000"));
          queue.enqueue_now_P(PSTR("G1 F600 Z0"));
          waitway = 0;
        }
      }
      else if (recdat.data[0] == 7)
      {
        // Assitant Level , Front Left 2
        if(!planner.has_blocks_queued())
        {
          waitway = 4;
          queue.enqueue_now_P(PSTR("G1 F600 Z3"));
          queue.enqueue_now_P(PSTR("G1 X30 Y30 F3000"));
          queue.enqueue_now_P(PSTR("G1 F600 Z0"));
          waitway = 0;
        }
      }
      else if (recdat.data[0] == 8)
      {
        // Assitant Level , Front Right 3
        if(!planner.has_blocks_queued())
        {
          waitway = 4;
          queue.enqueue_now_P(PSTR("G1 F600 Z3"));
          queue.enqueue_now_P(PSTR("G1 X275 Y30 F3000"));
          queue.enqueue_now_P(PSTR("G1 F600 Z0"));
          waitway = 0;
        }
      }
      else if (recdat.data[0] == 9)
      {
        // Assitant Level , Back Right 4
        if(!planner.has_blocks_queued())
        {
          waitway = 4;
          queue.enqueue_now_P(PSTR("G1 F600 Z3"));
          queue.enqueue_now_P(PSTR("G1 X275 Y275 F3000"));
          queue.enqueue_now_P(PSTR("G1 F600 Z0"));
          waitway = 0;
        }
      }
      else if (recdat.data[0] == 10)
      {
        // Assitant Level , Back Left 5
        if(!planner.has_blocks_queued())
        {   
          waitway = 4;
          queue.enqueue_now_P(PSTR("G1 F600 Z3"));
          queue.enqueue_now_P(PSTR("G1 X30 Y275 F3000"));
          queue.enqueue_now_P(PSTR("G1 F600 Z0"));
          waitway = 0;
        }
      }
       else if (recdat.data[0] == 11)
      { 
        waitway = 3;
        RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
        queue.enqueue_now_P(PSTR("G28 X"));
        queue.enqueue_now_P(PSTR("G34"));
        Update_Time_Value = 0;
        waitway = 0;
      } 
      else if (recdat.data[0] == 12) {
        RTS_SndData(ExchangePageBase + 40, ExchangepageAddr);
        queue.enqueue_now_P(PSTR("G28 X"));
        queue.enqueue_now_P(PSTR("T0"));
        active_extruder_flag = false;
        active_extruder = 0;
        active_extruder_font = active_extruder;
        RTS_SndData(0, EXCHANGE_NOZZLE_ICON_VP);
        RTS_SndData(10 * current_position[X_AXIS], AXIS_X_COORD_VP);
        queue.enqueue_now_P(PSTR("G35"));
      }
      RTS_SndData(0, MOTOR_FREE_ICON_VP);
      break;

    case XaxismoveKey:
    if(!planner.has_blocks_queued())
      {
        waitway = 4;
        if(active_extruder == 0)
        {
          active_extruder_flag = false;
        }
        else if(active_extruder == 1)
        {
          active_extruder_flag = true;
        }

        if(active_extruder == 1)
        {
          if(recdat.data[0] >= 32768)
          {
            current_position_x1_axis = ((float)recdat.data[0] - 65536) / 10;
          }
          else
          {
            current_position_x1_axis = ((float)recdat.data[0]) / 10;
          }

          if(current_position_x1_axis > X2_MAX_POS)
          {
            current_position_x1_axis = X2_MAX_POS;
          }
          else if((TEST(axes_trusted, X_AXIS)) && (current_position_x1_axis < (current_position_x0_axis - X_MIN_POS)))
          {
            current_position_x1_axis = current_position_x0_axis - X_MIN_POS;
          }
          else if(current_position_x1_axis < (X_MIN_POS - X_MIN_POS))
          {
            current_position_x1_axis = X_MIN_POS - X_MIN_POS;
          }
          current_position[X_AXIS] = current_position_x1_axis;
        }
        else if(active_extruder == 0)
        {
          if(recdat.data[0] >= 32768)
          {
            current_position_x0_axis = ((float)recdat.data[0] - 65536) / 10;
          }
          else
          {
            current_position_x0_axis = ((float)recdat.data[0]) / 10;
          }

          if(current_position_x0_axis < X_MIN_POS)
          {
            current_position_x0_axis = X_MIN_POS;
          }
          else if((TEST(axes_trusted, X_AXIS)) && (current_position_x0_axis > (current_position_x1_axis + X_MIN_POS)))
          {
            current_position_x0_axis = current_position_x1_axis + X_MIN_POS;
          }
          else if(current_position_x0_axis > (X2_MAX_POS + X_MIN_POS))
          {
            current_position_x0_axis = X2_MAX_POS + X_MIN_POS;
          }
          current_position[X_AXIS] = current_position_x0_axis;
        }
        RTS_line_to_current(X_AXIS);
        RTS_SndData(10 * current_position[X_AXIS], AXIS_X_COORD_VP);
        RTS_SndData(0, MOTOR_FREE_ICON_VP);
        waitway = 0;
      }
      break;

    case YaxismoveKey:
    if(!planner.has_blocks_queued())
      {
        float y_min, y_max;
        waitway = 4;
        y_min = Y_MIN_POS;
        y_max = Y_MAX_POS;
        current_position[Y_AXIS] = ((float)recdat.data[0]) / 10;
        if (current_position[Y_AXIS] < y_min)
        {
          current_position[Y_AXIS] = y_min;
        }
        else if (current_position[Y_AXIS] > y_max)
        {
          current_position[Y_AXIS] = y_max;
        }
        RTS_line_to_current(Y_AXIS);
        RTS_SndData(10 * current_position[Y_AXIS], AXIS_Y_COORD_VP);
        RTS_SndData(0, MOTOR_FREE_ICON_VP);
        waitway = 0;
      }
      break;
    
    case ZaxismoveKey:
      if(!planner.has_blocks_queued())
      {
        float z_min, z_max;
        waitway = 4;
        z_min = Z_MIN_POS;
        z_max = Z_MAX_POS;
        current_position[Z_AXIS] = ((float)recdat.data[0]) / 10;
        if (current_position[Z_AXIS] < z_min)
        {
          current_position[Z_AXIS] = z_min;
        }
        else if (current_position[Z_AXIS] > z_max)
        {
          current_position[Z_AXIS] = z_max;
        }
        RTS_line_to_current(Z_AXIS);
        RTS_SndData(10 * current_position[Z_AXIS], AXIS_Z_COORD_VP);
        RTS_SndData(0, MOTOR_FREE_ICON_VP);
        waitway = 0;
      }
      break;
    case SelectExtruderKey:
      if(recdat.data[0] == 1)
      {
        if(!planner.has_blocks_queued())
        {
          if(active_extruder == 0)
          {
            queue.enqueue_now_P(PSTR("G28 X"));
            queue.enqueue_now_P(PSTR("T1"));
            active_extruder = 1;
            active_extruder_flag = true;
            active_extruder_font = active_extruder;

            RTS_SndData(10 * X2_MAX_POS, AXIS_X_COORD_VP);
            RTS_SndData(1, EXCHANGE_NOZZLE_ICON_VP);
          }
          else if(active_extruder == 1)
          {
            queue.enqueue_now_P(PSTR("G28 X"));
            queue.enqueue_now_P(PSTR("T0"));
            active_extruder = 0;
            active_extruder_flag = false;
            active_extruder_font = active_extruder;

            RTS_SndData(10 * X_MIN_POS, AXIS_X_COORD_VP);
            RTS_SndData(0, EXCHANGE_NOZZLE_ICON_VP);
          }
        }
      }
      else if (recdat.data[0] == 2)
      {
        if(!planner.has_blocks_queued())
        {
          waitway = 4;
          if(active_extruder == 0)
          {
            queue.enqueue_now_P(PSTR("G28 X"));
            queue.enqueue_now_P(PSTR("T1"));
            active_extruder_flag = true;
            active_extruder = 1;
            active_extruder_font = active_extruder;
            RTS_SndData(1, EXCHANGE_NOZZLE_ICON_VP);
            RTS_SndData(10 * current_position[X_AXIS], AXIS_X_COORD_VP);
          }
          else if(active_extruder == 1)
          {
            queue.enqueue_now_P(PSTR("G28 X"));
            queue.enqueue_now_P(PSTR("T0"));
            active_extruder_flag = false;
            active_extruder = 0;
            active_extruder_font = active_extruder;
            RTS_SndData(0, EXCHANGE_NOZZLE_ICON_VP);
            RTS_SndData(10 * current_position[X_AXIS], AXIS_X_COORD_VP);
          }
          RTS_SndData(0, MOTOR_FREE_ICON_VP);
          waitway = 0;
        }
      }
      break;

    case FilamentLoadKey:

      if(recdat.data[0] == 1)
      {
        if(!planner.has_blocks_queued())
        {
          #if ENABLED(CHECKFILEMENT)
            if(0 == READ(FIL_RUNOUT_PIN))
            {
              RTS_SndData(ExchangePageBase + 20, ExchangepageAddr);
            }
          #endif
          current_position[E_AXIS] -= Filament0LOAD;
          active_extruder = 0;
          queue.enqueue_now_P(PSTR("T0"));

          if(thermalManager.temp_hotend[0].celsius < (ChangeFilament0Temp - 5))
          {
            RTS_SndData((int)ChangeFilament0Temp, CHANGE_FILAMENT0_TEMP_VP);
            RTS_SndData(ExchangePageBase + 24, ExchangepageAddr);
          }
          else
          {
            RTS_line_to_current(E_AXIS);
            RTS_SndData(10 * Filament0LOAD, HEAD0_FILAMENT_LOAD_DATA_VP);
          }
        }
      }
      else if(recdat.data[0] == 2)
      {
        if(!planner.has_blocks_queued())
        {
          #if ENABLED(CHECKFILEMENT)
            if(0 == READ(FIL_RUNOUT_PIN))
            {
              RTS_SndData(ExchangePageBase + 20, ExchangepageAddr);
            }
          #endif
          current_position[E_AXIS] += Filament0LOAD;
          active_extruder = 0;
          queue.enqueue_now_P(PSTR("T0"));

          if(thermalManager.temp_hotend[0].celsius < (ChangeFilament0Temp - 5))
          {
            RTS_SndData((int)ChangeFilament0Temp, CHANGE_FILAMENT0_TEMP_VP);
            RTS_SndData(ExchangePageBase + 24, ExchangepageAddr);
          }
          else
          {
            RTS_line_to_current(E_AXIS);
            RTS_SndData(10 * Filament0LOAD, HEAD0_FILAMENT_LOAD_DATA_VP);
          }
        }
      }
      else if(recdat.data[0] == 3)
      {
        if(!planner.has_blocks_queued())
        {
          #if ENABLED(CHECKFILEMENT)
            if(0 == READ(FIL_RUNOUT2_PIN))
            {
              RTS_SndData(ExchangePageBase + 20, ExchangepageAddr);
            }
          #endif
          current_position[E_AXIS] -= Filament1LOAD;
          active_extruder = 1;
          queue.enqueue_now_P(PSTR("T1"));

          if (thermalManager.temp_hotend[1].celsius < (ChangeFilament1Temp - 5))
          {
            RTS_SndData((int)ChangeFilament1Temp, CHANGE_FILAMENT1_TEMP_VP);

            RTS_SndData(ExchangePageBase + 25, ExchangepageAddr);
          }
          else
          {
            RTS_line_to_current(E_AXIS);
            RTS_SndData(10 * Filament1LOAD, HEAD1_FILAMENT_LOAD_DATA_VP);
          }
        }
      }
      else if(recdat.data[0] == 4)
      {
        if(!planner.has_blocks_queued())
        {
          #if ENABLED(CHECKFILEMENT)
            if(0 == READ(FIL_RUNOUT2_PIN))
            {
              RTS_SndData(ExchangePageBase + 20, ExchangepageAddr);
            }
          #endif
          current_position[E_AXIS] += Filament1LOAD;
          active_extruder = 1;
          queue.enqueue_now_P(PSTR("T1"));

          if(thermalManager.temp_hotend[1].celsius < (ChangeFilament1Temp - 5))
          {
            RTS_SndData((int)ChangeFilament1Temp, CHANGE_FILAMENT1_TEMP_VP);
            RTS_SndData(ExchangePageBase + 25, ExchangepageAddr);
          }
          else
          {
            RTS_line_to_current(E_AXIS);
            RTS_SndData(10 * Filament1LOAD, HEAD1_FILAMENT_LOAD_DATA_VP);
          }
        }
      }
      else if(recdat.data[0] == 5)
      {
        if(!planner.has_blocks_queued())
        {
          RTS_SndData(thermalManager.temp_hotend[0].celsius, HEAD0_CURRENT_TEMP_VP);
          thermalManager.setTargetHotend(ChangeFilament0Temp, 0);
          RTS_SndData(ChangeFilament0Temp, HEAD0_SET_TEMP_VP);
          RTS_SndData(ExchangePageBase + 26, ExchangepageAddr);
          heatway = 1;
        }
      }
      else if(recdat.data[0] == 6)
      {
        if(!planner.has_blocks_queued())
        {
          Filament0LOAD = 10;
          Filament1LOAD = 10;
          RTS_SndData(10 * Filament0LOAD, HEAD0_FILAMENT_LOAD_DATA_VP);
          RTS_SndData(10 * Filament1LOAD, HEAD1_FILAMENT_LOAD_DATA_VP);
          RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
        }
      }
      else if(recdat.data[0] == 7)
      {
        if(!planner.has_blocks_queued())
        {
          RTS_SndData(thermalManager.temp_hotend[1].celsius, HEAD1_CURRENT_TEMP_VP);
          thermalManager.setTargetHotend(ChangeFilament1Temp, 1);
          RTS_SndData(ChangeFilament1Temp, HEAD1_SET_TEMP_VP);
          RTS_SndData(ExchangePageBase + 26, ExchangepageAddr);
          heatway = 2;
        }
      }
      else if (recdat.data[0] == 8)
      {
        if(!planner.has_blocks_queued())
        {
          Filament0LOAD = 10;
          Filament1LOAD = 10;
          RTS_SndData(10 * Filament0LOAD, HEAD0_FILAMENT_LOAD_DATA_VP);
          RTS_SndData(10 * Filament1LOAD, HEAD1_FILAMENT_LOAD_DATA_VP);
          RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
        }
      }
      else if(recdat.data[0] == 0xF1)
      {
        if(!planner.has_blocks_queued())
        {
          thermalManager.temp_hotend[0].target = 0;
          thermalManager.temp_hotend[1].target = 0;
          RTS_SndData(thermalManager.temp_hotend[0].target, HEAD0_SET_TEMP_VP);
          RTS_SndData(thermalManager.temp_hotend[1].target, HEAD1_SET_TEMP_VP);
          RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
          Filament0LOAD = 10;
          Filament1LOAD = 10;
          RTS_SndData(10 * Filament0LOAD, HEAD0_FILAMENT_LOAD_DATA_VP);
          RTS_SndData(10 * Filament1LOAD, HEAD1_FILAMENT_LOAD_DATA_VP);
          break;
        }
      }
      else if(recdat.data[0] == 0xF0)
      {
        break;
      }
      break;

    case FilamentCheckKey:
      if (recdat.data[0] == 1)
      {
        #if ENABLED(CHECKFILEMENT)
          if((0 == READ(FIL_RUNOUT_PIN)) && (active_extruder == 0))
          {
            RTS_SndData(ExchangePageBase + 20, ExchangepageAddr);
          }
          else if((0 == READ(FIL_RUNOUT2_PIN)) && (active_extruder == 1))
          {
            RTS_SndData(ExchangePageBase + 20, ExchangepageAddr);
          }
          else
          {
            RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
          }
        #endif
      }
      else if (recdat.data[0] == 2)
      {
        RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
        Filament0LOAD = 10;
        Filament1LOAD = 10;
      }
      break;

    case PowerContinuePrintKey:
      if (recdat.data[0] == 1) // yes to resume print
      {
        #if ENABLED(DUAL_X_CARRIAGE)
          save_dual_x_carriage_mode = dualXPrintingModeStatus;
          
          switch(save_dual_x_carriage_mode)
          {
            case 1:
              queue.enqueue_now_P(PSTR("M605 S1"));
              break;
            case 2:
              queue.enqueue_now_P(PSTR("M605 S2"));
              break;
            case 3:
              queue.enqueue_now_P(PSTR("M605 S2 X150 R0"));
              queue.enqueue_now_P(PSTR("M605 S3"));
              break;
            case 4:
              queue.enqueue_now_P(PSTR("M605 S4"));
              break;
            default:
              queue.enqueue_now_P(PSTR("M605 S0"));
              break;
          }
        #endif
        if (recovery.info.recovery_flag)
        {
          power_off_type_yes = 1;
          Update_Time_Value = 0;
          RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);
          // recovery.resume();
          queue.enqueue_now_P(PSTR("M1000"));

          PoweroffContinue = true;
          sdcard_pause_check = true;
          zprobe_zoffset = probe.offset.z;
          RTS_SndData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
          RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
          PrintFlag = 2;
        }
      }
      else if (recdat.data[0] == 2) // no to stop print
      {
        Update_Time_Value = RTS_UPDATE_VALUE;
        #if ENABLED(DUAL_X_CARRIAGE)
          extruder_duplication_enabled = false;
          dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;
          active_extruder = 0;
        #endif
        RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);

        PoweroffContinue = false;
        sdcard_pause_check = false;
        queue.clear();
        quickstop_stepper();
        print_job_timer.stop();
        thermalManager.disable_all_heaters();
        print_job_timer.reset();

        if(CardReader::flag.mounted)
        {
          #if ENABLED(SDSUPPORT) && ENABLED(POWER_LOSS_RECOVERY)
            card.removeJobRecoveryFile();
            recovery.info.valid_head = 0;
            recovery.info.valid_foot = 0;
            recovery.close();
          #endif
        }

        wait_for_heatup = wait_for_user = false;
        sd_printing_autopause = false;
        PrintFlag = 0;
      }
      break;

    case SelectFileKey:
      if (RTS_SD_Detected())
      {
        fileInfo.pageFileIndex = recdat.data[0]-5;
        int index = 5*(fileInfo.currentPage-1) + fileInfo.pageFileIndex;

        if (index >= card.countFilesInWorkDir())
        {
          break;
        }

        card.selectFileByIndex(index);
        strncpy(fileInfo.currentDisplayFilename, card.longest_filename(), 20);
        for (int j = 0; j < MAXPATHNAMELENGTH; j ++)
        {
          fileInfo.currentFilePath[j] = 0;
        }
        card.getAbsFilenameInCWD(fileInfo.currentFilePath);
        strcat(fileInfo.currentFilePath, card.filename);

        if (!EndsWith(fileInfo.currentFilePath, "gcode") && !EndsWith(fileInfo.currentFilePath, "GCO") 
        && !EndsWith(fileInfo.currentFilePath, "GCODE")) 
        {
          card.cd(fileInfo.currentFilePath);
          InitCardList();
          break;
        }

        for (int j = 0; j < 10; j ++)
        {
          RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
        }

        RTS_SndData(fileInfo.currentDisplayFilename, PRINT_FILE_TEXT_VP);
        delay(2);

        if (hasSelected) {
          RTS_SndData((unsigned long)0xA514, FilenameNature + previousSelectionIndex * 16);
        }

        previousSelectionIndex = recdat.data[0];
        hasSelected = true;

        RTS_SndData((unsigned long)0x073F, FilenameNature + recdat.data[0] * 16);
        RTS_SndData(1, FILE1_SELECT_ICON_VP + (recdat.data[0] - 1));
      }

      break;

    case PrintFileKey:
    if (recdat.data[0] == 1)
      {
        if((0 != dualXPrintingModeStatus) && (4 != dualXPrintingModeStatus))
        {
          RTS_SndData(dualXPrintingModeStatus, SELECT_MODE_ICON_VP);        
        }
        else if(4 == dualXPrintingModeStatus)
        {
          RTS_SndData(5, SELECT_MODE_ICON_VP);  
        }
        else
        {
          RTS_SndData(4, SELECT_MODE_ICON_VP);
        }
        RTS_SndData(fileInfo.currentDisplayFilename, PRINT_FILE_TEXT_VP);
        RTS_SndData(ExchangePageBase + 56, ExchangepageAddr);
      }
      else if (recdat.data[0] == 2) 
      {
        //dir back
        card.cdup();
        InitCardList();
      }
      else if(recdat.data[0] == 3)
      {
        //page left
        ShowFilesOnCardPage(fileInfo.currentPage-1);
      }
      else if(recdat.data[0] == 4)
      {
        //page right
        ShowFilesOnCardPage(fileInfo.currentPage+1);
      }
      else if ((recdat.data[0] == 11) && RTS_SD_Detected())
      {
        if (!card.fileExists(fileInfo.currentFilePath)) {
          break;
        }

        char cmd[MAX_CMD_SIZE+16];
        char *c;
        sprintf_P(cmd, PSTR("M23 %s"), fileInfo.currentFilePath);
        for (c = &cmd[4]; *c; c++)
          *c = tolower(*c);

        memset(cmdbuf, 0, sizeof(cmdbuf));
        strcpy(cmdbuf, cmd);

        save_dual_x_carriage_mode = dualXPrintingModeStatus;
        SERIAL_ECHOLNPGM("dualXPrintingModeStatus: ", dualXPrintingModeStatus);
        
        switch(save_dual_x_carriage_mode)
        {
          case 1:
            queue.enqueue_now_P(PSTR("M605 S1"));
            break;
          case 2:
            queue.enqueue_now_P(PSTR("M605 S2"));
            break;
          case 3:
            queue.enqueue_now_P(PSTR("M605 S2 X150 R0"));
            queue.enqueue_now_P(PSTR("M605 S3"));
            break;
          case 4:
              queue.enqueue_now_P(PSTR("M605 S4"));
              break;
          default:
            queue.enqueue_now_P(PSTR("M605 S0"));
            break;
        }
        queue.enqueue_one_now(cmd);
        delay(20);
        queue.enqueue_now_P(PSTR("M24"));
        // clean screen.
        for (int j = 0; j < 20; j ++)
        {
          RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
        }

        RTS_SndData(fileInfo.currentDisplayFilename, PRINT_FILE_TEXT_VP);

        delay(2);

        #if ENABLED(BABYSTEPPING)
          RTS_SndData(0, AUTO_BED_LEVEL_ZOFFSET_VP);
        #endif
        feedrate_percentage = 100;
        RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
        zprobe_zoffset = last_zoffset;
        RTS_SndData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
        PoweroffContinue = true;
        RTS_SndData(ExchangePageBase + 11, ExchangepageAddr);
        Update_Time_Value = 0;
        PrintFlag = 2;
        change_page_number = 11;
      }
      else if(recdat.data[0] == 12)
      {
        RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
      }
      break;

    case PrintSelectModeKey:
      SetExtruderMode(recdat.data[0], false);
      break;

    case StoreMemoryKey:
      if(recdat.data[0] == 0xF1)
      {
        settings.init_eeprom();
        #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
          bool zig = false;
          int8_t inStart, inStop, inInc, showcount;
          showcount = 0;
          for (int y = 0; y < GRID_MAX_POINTS_Y; y++)
          {
            // away from origin
            if (zig)
            {
              inStart = 0;
              inStop = GRID_MAX_POINTS_X;
              inInc = 1;
            }
            else
            {
              // towards origin
              inStart = GRID_MAX_POINTS_X - 1;
              inStop = -1;
              inInc = -1;
            }
            zig ^= true;
            for (int x = inStart; x != inStop; x += inInc)
            {
              RTS_SndData(bedlevel.z_values[x][y]*1000, AUTO_BED_LEVEL_1POINT_VP + showcount * 2);
              showcount++;
            }
          }
          queue.enqueue_now_P(PSTR("M420 S1"));
        #endif
        zprobe_zoffset = 0;
        last_zoffset = 0;
        RTS_SndData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
        rtscheck.RTS_SndData(ExchangePageBase + 21, ExchangepageAddr);
        RTS_SndData((hotend_offset[1].x - X2_MAX_POS) * 100, TWO_EXTRUDER_HOTEND_XOFFSET_VP);
        RTS_SndData(hotend_offset[1].y * 100, TWO_EXTRUDER_HOTEND_YOFFSET_VP);
        RTS_SndData(hotend_offset[1].z * 100, TWO_EXTRUDER_HOTEND_ZOFFSET_VP);
      }
      else if (recdat.data[0] == 0xF0)
      {
        memset(commandbuf, 0, sizeof(commandbuf));
        sprintf_P(commandbuf, PSTR("M218 T1 X%4.1f"), hotend_offset[1].x);
        queue.enqueue_now_P(commandbuf);
        sprintf_P(commandbuf, PSTR("M218 T1 Y%4.1f"), hotend_offset[1].y);
        queue.enqueue_now_P(commandbuf);
        sprintf_P(commandbuf, PSTR("M218 T1 Z%4.1f"), hotend_offset[1].z);
        queue.enqueue_now_P(commandbuf);
        //settings.save();
        rtscheck.RTS_SndData(ExchangePageBase + 35, ExchangepageAddr);
      }
      break;

    case XhotendOffsetKey:
      if (recdat.data[0] >= 32768)
      {
        hotend_offset[1].x = (recdat.data[0] - 65536) / 100.0;
        hotend_offset[1].x = hotend_offset[1].x - 0.00001 + X2_MAX_POS;
      }
      else
      {
        hotend_offset[1].x = (recdat.data[0]) / 100.0;
        hotend_offset[1].x = hotend_offset[1].x + 0.00001 + X2_MAX_POS;
      }

      RTS_SndData((hotend_offset[1].x - X2_MAX_POS)* 100, TWO_EXTRUDER_HOTEND_XOFFSET_VP);
      break;

    case YhotendOffsetKey:
      if (recdat.data[0] >= 32768)
      {
        hotend_offset[1].y = (recdat.data[0] - 65536) / 100.0;
        hotend_offset[1].y = hotend_offset[1].y - 0.00001;
      }
      else
      {
        hotend_offset[1].y = (recdat.data[0]) / 100.0;
        hotend_offset[1].y = hotend_offset[1].y + 0.00001;
      }

      RTS_SndData(hotend_offset[1].y * 100, TWO_EXTRUDER_HOTEND_YOFFSET_VP);
      break;

    case ZhotendOffsetKey:
      if (recdat.data[0] >= 32768)
      {
        hotend_offset[1].z = (recdat.data[0] - 65536) / 100.0;
        hotend_offset[1].z = hotend_offset[1].z - 0.00001;
      }
      else
      {
        hotend_offset[1].z = (recdat.data[0]) / 100.0;
        hotend_offset[1].z = hotend_offset[1].z + 0.00001;
      }

      RTS_SndData(hotend_offset[1].z * 100, TWO_EXTRUDER_HOTEND_ZOFFSET_VP);
      break;

    case SaveEEPROM:
      if (recdat.data[0] == 1)
      {
        settings.save();
        //RTS_SndData(StartSoundSet, SoundAddr);
        RTS_AlertBeep();
      }
    break;
    case ChangePageKey:
      SERIAL_ECHOLNPGM("Change Page: ", change_page_number);
      if ((change_page_number == 36) || (change_page_number == 76))
      {
        break;
      }
      else if(change_page_number == 11)
      {
        RTS_SndData(change_page_number + ExchangePageBase, ExchangepageAddr);
        if((0 != dualXPrintingModeStatus) && (4 != dualXPrintingModeStatus))
        {
          RTS_SndData(dualXPrintingModeStatus, PRINT_MODE_ICON_VP);        
        }
        else if(4 == dualXPrintingModeStatus)
        {
          RTS_SndData(5, PRINT_MODE_ICON_VP);
        }
        else
        {
          RTS_SndData(4, PRINT_MODE_ICON_VP);
        }
      }
      else if(change_page_number == 12)
      {
        RTS_SndData(change_page_number + ExchangePageBase, ExchangepageAddr);
        if((0 != dualXPrintingModeStatus) && (4 != dualXPrintingModeStatus))
        {
          RTS_SndData(dualXPrintingModeStatus, PRINT_MODE_ICON_VP);        
        }
        else if(4 == dualXPrintingModeStatus)
        {
          RTS_SndData(5, PRINT_MODE_ICON_VP);
        }
        else
        {
          RTS_SndData(4, PRINT_MODE_ICON_VP);
        }
      }
      else
      {
        RTS_SndData(change_page_number + ExchangePageBase, ExchangepageAddr);
        change_page_number = 1;
      }
      if((0 != dualXPrintingModeStatus) && (4 != dualXPrintingModeStatus))
      {
        RTS_SndData(dualXPrintingModeStatus, SELECT_MODE_ICON_VP);        
      }
      else if(4 == dualXPrintingModeStatus)
      {
        RTS_SndData(5, SELECT_MODE_ICON_VP);
      }
      else
      {
        RTS_SndData(4, SELECT_MODE_ICON_VP);
      }

      for (int i = 0; i < card.get_num_Files(); i ++)
      {
        for (int j = 0; j < 20; j ++)
        {
          RTS_SndData(0, FILE1_TEXT_VP + i * 20 + j);
        }
      }

      for (int i = 1; i < 5; i++)
      {
        RTS_SndData((unsigned long)0xA514, FilenameNature + (i + 4) * 16);
      }

      for (int j = 0; j < 20; j ++)
      {
        // clean screen.
        RTS_SndData(0, PRINT_FILE_TEXT_VP + j);
        // clean filename
        RTS_SndData(0, SELECT_FILE_TEXT_VP + j);
      }
      // clean filename Icon
      for (int j = 0; j < 20; j ++)
      {
        RTS_SndData(10, FILE1_SELECT_ICON_VP + j);
      }

      RTS_SndData(fileInfo.currentDisplayFilename, PRINT_FILE_TEXT_VP);

      // represents to update file list
      if (CardUpdate && lcd_sd_status && IS_SD_INSERTED())
      {
        card.cdroot();
        InitCardList();
      }

      char sizeBuf[20];
      sprintf(sizeBuf, "%d X %d X %d", X_MAX_POS - 2, Y_MAX_POS - 2, Z_MAX_POS);
      RTS_SndData(MACVERSION, PRINTER_MACHINE_TEXT_VP);
      RTS_SndData(SOFTVERSION, PRINTER_VERSION_TEXT_VP);
      RTS_SndData(sizeBuf, PRINTER_PRINTSIZE_TEXT_VP);

      RTS_SndData(CORP_WEBSITE, PRINTER_WEBSITE_TEXT_VP);
      //RTS_SndData(Screen_version, Screen_Version_VP);

      if (thermalManager.fan_speed[0] == 0)
      {
        RTS_SndData(1, HEAD0_FAN_ICON_VP);
        RTS_SndData(thermalManager.fan_speed[0], E0_SET_FAN_VP);
      }
      else
      {
        RTS_SndData(0, HEAD0_FAN_ICON_VP);
        RTS_SndData(thermalManager.fan_speed[0], E0_SET_FAN_VP);
      }
      if (thermalManager.fan_speed[1] == 0)
      {
        RTS_SndData(1, HEAD1_FAN_ICON_VP);
        RTS_SndData(thermalManager.fan_speed[1], E1_SET_FAN_VP);
      }
      else
      {
        RTS_SndData(0, HEAD1_FAN_ICON_VP);
        RTS_SndData(thermalManager.fan_speed[1], E1_SET_FAN_VP);
      }
      Percentrecord = card.percentDone() + 1;
      if (Percentrecord <= 100)
      {
        rtscheck.RTS_SndData((unsigned char)Percentrecord, PRINT_PROCESS_ICON_VP);
      }
      rtscheck.RTS_SndData((unsigned char)card.percentDone(), PRINT_PROCESS_VP);

      RTS_SndData(probe.offset.z * 100, AUTO_BED_LEVEL_ZOFFSET_VP);

      RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);
      RTS_SndData(thermalManager.temp_hotend[0].target, HEAD0_SET_TEMP_VP);
      RTS_SndData(thermalManager.temp_hotend[1].target, HEAD1_SET_TEMP_VP);
      RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
      break;

    default:
      break;
  }
  memset(&recdat, 0, sizeof(recdat));
  recdat.head[0] = FHONE;
  recdat.head[1] = FHTWO;
}

int EndsWith(const char *str, const char *suffix)
{
    if (!str || !suffix)
        return 0;
    size_t lenstr = strlen(str);
    size_t lensuffix = strlen(suffix);
    if (lensuffix >  lenstr)
        return 0;
    return strncmp(str + lenstr - lensuffix, suffix, lensuffix) == 0;
}

void EachMomentUpdate()
{
  millis_t ms = millis();
  if(ms > next_rts_update_ms)
  {
    // print the file before the power is off.
    if((power_off_type_yes == 0) && lcd_sd_status && (recovery.info.recovery_flag == true))
    {
      rtscheck.RTS_SndData(ExchangePageBase, ExchangepageAddr);
      if(startprogress < 100)
      {
        rtscheck.RTS_SndData(startprogress, START1_PROCESS_ICON_VP);
      }
      delay(30);
      if((startprogress += 1) > 100)
      {
        rtscheck.RTS_SndData(StartSoundSet, SoundAddr);
        power_off_type_yes = 1;
        #if ENABLED(POWER_LOSS_RECOVERY)
          if (card.jobRecoverFileExists()) {
            rtscheck.RTS_SndData(recovery.filename, PRINT_FILE_TEXT_VP);
            rtscheck.RTS_SndData(ExchangePageBase + 36, ExchangepageAddr);
          }
        #endif
        StartFlag = 1;
      }
      return;
    }
    else if((power_off_type_yes == 0) && (recovery.info.recovery_flag == false))
    {
      rtscheck.RTS_SndData(ExchangePageBase, ExchangepageAddr);
      if(startprogress < 100)
      {
        rtscheck.RTS_SndData(startprogress, START1_PROCESS_ICON_VP);
      }
      delay(30);
      if((startprogress += 1) > 100)
      {
        rtscheck.RTS_SndData(StartSoundSet, SoundAddr);
        power_off_type_yes = 1;
        Update_Time_Value = RTS_UPDATE_VALUE;
        rtscheck.RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
        change_page_number = 1;
      }
      return;
    }
    else
    {
      // need to optimize
      if(recovery.info.print_job_elapsed != 0)
      {
        duration_t elapsed = print_job_timer.duration();
        static unsigned char last_cardpercentValue = 100;
        rtscheck.RTS_SndData(elapsed.value / 3600, PRINT_TIME_HOUR_VP);
        rtscheck.RTS_SndData((elapsed.value % 3600) / 60, PRINT_TIME_MIN_VP);

        if(card.isPrinting() && (last_cardpercentValue != card.percentDone()))
        {
          if((unsigned char) card.percentDone() > 0)
          {
            Percentrecord = card.percentDone();
            if(Percentrecord <= 100)
            {
              rtscheck.RTS_SndData((unsigned char)Percentrecord, PRINT_PROCESS_ICON_VP);
            }
            // Estimate remaining time every 20 seconds
            static millis_t next_remain_time_update = 0;
            if(ELAPSED(ms, next_remain_time_update))
            {
              if((0 == save_dual_x_carriage_mode) && (thermalManager.temp_hotend[0].celsius >= (thermalManager.temp_hotend[0].target - 5)))
              {
                remain_time = elapsed.value / (Percentrecord * 0.01f) - elapsed.value;
                next_remain_time_update += 20 * 1000UL;
                rtscheck.RTS_SndData(remain_time / 3600, PRINT_SURPLUS_TIME_HOUR_VP);
                rtscheck.RTS_SndData((remain_time % 3600) / 60, PRINT_SURPLUS_TIME_MIN_VP);
              }
              else if((0 != save_dual_x_carriage_mode) && (thermalManager.temp_hotend[0].celsius >= (thermalManager.temp_hotend[0].target - 5)) && (thermalManager.temp_hotend[1].celsius >= (thermalManager.temp_hotend[1].target - 5)))
              {
                remain_time = elapsed.value / (Percentrecord * 0.01f) - elapsed.value;
                next_remain_time_update += 20 * 1000UL;
                rtscheck.RTS_SndData(remain_time / 3600, PRINT_SURPLUS_TIME_HOUR_VP);
                rtscheck.RTS_SndData((remain_time % 3600) / 60, PRINT_SURPLUS_TIME_MIN_VP);
              }
            }
          }
          else
          {
            rtscheck.RTS_SndData(0, PRINT_PROCESS_ICON_VP);
            rtscheck.RTS_SndData(0, PRINT_SURPLUS_TIME_HOUR_VP);
            rtscheck.RTS_SndData(0, PRINT_SURPLUS_TIME_MIN_VP);
          }
          rtscheck.RTS_SndData((unsigned char)card.percentDone(), PRINT_PROCESS_VP);
          last_cardpercentValue = card.percentDone();
        }
      }

      if(pause_action_flag && (false == sdcard_pause_check) && printingIsPaused() && !planner.has_blocks_queued())
      {
        pause_action_flag = false;
        if((1 == active_extruder) && (1 == save_dual_x_carriage_mode))
        {
          queue.enqueue_now_P(PSTR("G0 F3000 X362 Y0"));
        }
        else
        {
          queue.enqueue_now_P(PSTR("G0 F3000 X-62 Y0"));
        }
      }

      rtscheck.RTS_SndData(thermalManager.temp_hotend[0].celsius, HEAD0_CURRENT_TEMP_VP);
      rtscheck.RTS_SndData(thermalManager.temp_hotend[1].celsius, HEAD1_CURRENT_TEMP_VP);
      rtscheck.RTS_SndData(thermalManager.temp_bed.celsius, BED_CURRENT_TEMP_VP);

      if((last_target_temperature[0] != thermalManager.temp_hotend[0].target) || (last_target_temperature[1] != thermalManager.temp_hotend[1].target) || (last_target_temperature_bed != thermalManager.temp_bed.target))
      {
        thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
        thermalManager.setTargetHotend(thermalManager.temp_hotend[1].target, 1);
        thermalManager.setTargetBed(thermalManager.temp_bed.target);
        rtscheck.RTS_SndData(thermalManager.temp_hotend[0].target, HEAD0_SET_TEMP_VP);
        rtscheck.RTS_SndData(thermalManager.temp_hotend[1].target, HEAD1_SET_TEMP_VP);
        rtscheck.RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
        last_target_temperature[0] = thermalManager.temp_hotend[0].target;
        last_target_temperature[1] = thermalManager.temp_hotend[1].target;
        last_target_temperature_bed = thermalManager.temp_bed.target;
      }

      if((thermalManager.temp_hotend[0].celsius >= thermalManager.temp_hotend[0].target) && (heatway == 1))
      {
        rtscheck.RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
        heatway = 0;
        rtscheck.RTS_SndData(10 * Filament0LOAD, HEAD0_FILAMENT_LOAD_DATA_VP);
      }
      else if((thermalManager.temp_hotend[1].celsius >= thermalManager.temp_hotend[1].target) && (heatway == 2))
      {
        rtscheck.RTS_SndData(ExchangePageBase + 23, ExchangepageAddr);
        heatway = 0;
        rtscheck.RTS_SndData(10 * Filament1LOAD, HEAD1_FILAMENT_LOAD_DATA_VP);
      }

      rtscheck.RTS_SndData(AutoHomeIconNum ++, AUTO_HOME_DISPLAY_ICON_VP);
      if (AutoHomeIconNum > 8)
      {
        AutoHomeIconNum = 0;
      }
    }
    // moved z height output to make sure it is always up to date
    rtscheck.RTS_SndData(10 * current_position[Z_AXIS], AXIS_Z_COORD_VP);
    
    next_rts_update_ms = ms + RTS_UPDATE_INTERVAL + Update_Time_Value;
  }
}

void SetExtruderMode(unsigned int mode, bool isDirect) {
  SERIAL_ECHOLNPGM("Selected extruder mode: ", mode);
  if (isDirect && mode == 4) {
    mode = 5;
  } else if (isDirect && mode == 0) {
    mode = 4;
  }
  SERIAL_ECHOLNPGM("Select new extruder mode: ", mode);
  if (mode == 1)
  {
    //dual mode
    rtscheck.RTS_SndData(1, TWO_COLOR_MODE_ICON_VP);
    rtscheck.RTS_SndData(0, COPY_MODE_ICON_VP);
    rtscheck.RTS_SndData(0, MIRROR_MODE_ICON_VP);
    rtscheck.RTS_SndData(0, SINGLE_MODE_ICON_VP);
    dualXPrintingModeStatus = 1;
    rtscheck.RTS_SndData(1, PRINT_MODE_ICON_VP);
    rtscheck.RTS_SndData(1, SELECT_MODE_ICON_VP);
  }
  else if (mode == 2)
  {
    //duplicate mode
    rtscheck.RTS_SndData(1, COPY_MODE_ICON_VP);
    rtscheck.RTS_SndData(0, TWO_COLOR_MODE_ICON_VP);
    rtscheck.RTS_SndData(0, MIRROR_MODE_ICON_VP);
    rtscheck.RTS_SndData(0, SINGLE_MODE_ICON_VP);
    dualXPrintingModeStatus = 2;
    rtscheck.RTS_SndData(2, PRINT_MODE_ICON_VP);
    rtscheck.RTS_SndData(2, SELECT_MODE_ICON_VP);
  }
  else if (mode == 3)
  {
    //mirror mode
    rtscheck.RTS_SndData(1, MIRROR_MODE_ICON_VP);
    rtscheck.RTS_SndData(0, TWO_COLOR_MODE_ICON_VP);
    rtscheck.RTS_SndData(0, COPY_MODE_ICON_VP);
    rtscheck.RTS_SndData(0, SINGLE_MODE_ICON_VP);
    dualXPrintingModeStatus = 3;
    rtscheck.RTS_SndData(3, PRINT_MODE_ICON_VP);
    rtscheck.RTS_SndData(3, SELECT_MODE_ICON_VP);
  }
  else if (mode == 4)
  {
    //single 1 mode
    rtscheck.RTS_SndData(0, MIRROR_MODE_ICON_VP);
    rtscheck.RTS_SndData(0, TWO_COLOR_MODE_ICON_VP);
    rtscheck.RTS_SndData(0, COPY_MODE_ICON_VP);
    rtscheck.RTS_SndData(1, SINGLE_MODE_ICON_VP);
    dualXPrintingModeStatus = 0;
    rtscheck.RTS_SndData(4, PRINT_MODE_ICON_VP);
    rtscheck.RTS_SndData(4, SELECT_MODE_ICON_VP);
  }
  else if (mode == 5)
  {
    //single 2 mode
    rtscheck.RTS_SndData(0, MIRROR_MODE_ICON_VP);
    rtscheck.RTS_SndData(0, TWO_COLOR_MODE_ICON_VP);
    rtscheck.RTS_SndData(0, COPY_MODE_ICON_VP);
    rtscheck.RTS_SndData(2, SINGLE_MODE_ICON_VP);
    dualXPrintingModeStatus = 4;
    rtscheck.RTS_SndData(5, PRINT_MODE_ICON_VP);
    rtscheck.RTS_SndData(5, SELECT_MODE_ICON_VP);
  } else if (mode == 6)
  {
    save_dual_x_carriage_mode = dualXPrintingModeStatus;
    //SERIAL_ECHOLNPGM("save_dual_x_carriage_mode: ", save_dual_x_carriage_mode);
    settings.save();
    rtscheck.RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
  } else {
    //single 1 mode
    rtscheck.RTS_SndData(0, MIRROR_MODE_ICON_VP);
    rtscheck.RTS_SndData(0, TWO_COLOR_MODE_ICON_VP);
    rtscheck.RTS_SndData(0, COPY_MODE_ICON_VP);
    rtscheck.RTS_SndData(1, SINGLE_MODE_ICON_VP);
    dualXPrintingModeStatus = 0;
    rtscheck.RTS_SndData(4, PRINT_MODE_ICON_VP);
    rtscheck.RTS_SndData(4, SELECT_MODE_ICON_VP);
  }
}

// looping at the loop function
void RTSUpdate()
{
  // Check the status of card
  rtscheck.RTS_SDCardUpate();

	sd_printing = IS_SD_PRINTING();
	card_insert_st = IS_SD_INSERTED() ;

	if((card_insert_st == false) && (sd_printing == true)){
		rtscheck.RTS_SndData(ExchangePageBase + 46, ExchangepageAddr);	
		rtscheck.RTS_SndData(0, CHANGE_SDCARD_ICON_VP);
		/* 暂停打印，使得喷头可以回到零点 */
		card.pauseSDPrint();
		print_job_timer.pause();
    pause_action_flag = true;
    sdcard_pause_check = false;

	}

	/* 更新拔卡和插卡提示图标 */
	if(last_card_insert_st != card_insert_st){
		/* 当前页面显示为拔卡提示页面，但卡已经插入了，更新插卡图标 */
		rtscheck.RTS_SndData((int)card_insert_st, CHANGE_SDCARD_ICON_VP);
		last_card_insert_st = card_insert_st;
	}
  
  EachMomentUpdate();
  // wait to receive massage and response
  while(rtscheck.RTS_RecData() > 0)
  {
    rtscheck.RTS_HandleData();
  }
}

void RTS_PauseMoveAxisPage()
{
  if(waitway == 1)
  {
    rtscheck.RTS_SndData(ExchangePageBase + 12, ExchangepageAddr);
    waitway = 0;
  }
  else if(waitway == 5)
  {
    rtscheck.RTS_SndData(ExchangePageBase + 39, ExchangepageAddr);
    waitway = 0;
  }
}

void RTS_AutoBedLevelPage()
{
  if(waitway == 3)
  {
    rtscheck.RTS_SndData(ExchangePageBase + 81, ExchangepageAddr);
    waitway = 0;
  }
}

void RTS_MoveAxisHoming()
{
  if(waitway == 4)
  {
    if(AxisUnitMode == 4)
    {
      rtscheck.RTS_SndData(ExchangePageBase + 58, ExchangepageAddr);
    } 
    else if(AxisUnitMode == 3)
    {
      rtscheck.RTS_SndData(ExchangePageBase + 29, ExchangepageAddr);
    } 
    else if(AxisUnitMode == 2)
    {
      rtscheck.RTS_SndData(ExchangePageBase + 30, ExchangepageAddr);
    } 
    else if(AxisUnitMode == 1)
    {
      rtscheck.RTS_SndData(ExchangePageBase + 31, ExchangepageAddr);
    } 
    waitway = 0;
  }
  else if(waitway == 6)
  {
    #if ENABLED(HAS_LEVELING)
      rtscheck.RTS_SndData(ExchangePageBase + 22, ExchangepageAddr);
    #else
      rtscheck.RTS_SndData(ExchangePageBase + 28, ExchangepageAddr);
    #endif
    waitway = 0;
  }
  else if(waitway == 7)
  {
    // Click Print finish
    rtscheck.RTS_SndData(ExchangePageBase + 1, ExchangepageAddr);
    waitway = 0;
  }
  if(active_extruder == 0)
  {
    rtscheck.RTS_SndData(0, EXCHANGE_NOZZLE_ICON_VP);
  }
  else
  {
    rtscheck.RTS_SndData(1, EXCHANGE_NOZZLE_ICON_VP);
  }

  rtscheck.RTS_SndData(10*current_position[X_AXIS], AXIS_X_COORD_VP);
  rtscheck.RTS_SndData(10*current_position[Y_AXIS], AXIS_Y_COORD_VP);
  rtscheck.RTS_SndData(10*current_position[Z_AXIS], AXIS_Z_COORD_VP);
}

void RTS_AlertBeep() {
  if (SoundFlag == true)
  {
    rtscheck.RTS_SndData(StartSoundSet, SoundAddr);
  }
}

#endif
