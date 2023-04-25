#ifndef LCD_RTS_H
#define LCD_RTS_H

#include "../../../sd/cardreader.h"
#include "string.h"
#include <Arduino.h>

#include "../../../inc/MarlinConfig.h"

extern int power_off_type_yes;

/*********************************/
#define FHONE   (0x5A)
#define FHTWO   (0xA5)
#define FHLENG  (0x06)
#define TEXTBYTELEN     20
#define MaxFileNumber   20

#define FileNum             MaxFileNumber
#define FileNameLen         TEXTBYTELEN
#define RTS_UPDATE_INTERVAL 2000
#define RTS_UPDATE_VALUE    (2 * RTS_UPDATE_INTERVAL)

#define SizeofDatabuf       26

/*************Register and Variable addr*****************/
#define RegAddr_W   0x80
#define RegAddr_R   0x81
#define VarAddr_W   0x82
#define VarAddr_R   0x83
#define ExchangePageBase    ((unsigned long)0x5A010000)
#define StartSoundSet       ((unsigned long)0x060480A0)
#define FONT_EEPROM         0

/*variable addr*/
#define ExchangepageAddr      0x0084
#define SoundAddr             0x00A0

#define START1_PROCESS_ICON_VP             0x1000
#define PRINT_SPEED_RATE_VP                0x1006
#define PRINT_PROCESS_ICON_VP              0x100E
#define PRINT_TIME_HOUR_VP                 0x1010
#define PRINT_TIME_MIN_VP                  0x1012
#define PRINT_PROCESS_VP                   0x1016
#define HEAD0_FAN_ICON_VP                  0x101E
#define HEAD1_FAN_ICON_VP                  0x101F
#define CHANGE_FILAMENT0_TEMP_VP           0x1020
#define CHANGE_FILAMENT1_TEMP_VP           0x1022
#define AUTO_BED_LEVEL_ZOFFSET_VP          0x1026

#define HEAD0_SET_TEMP_VP                  0x1034
#define HEAD0_CURRENT_TEMP_VP              0x1036
#define HEAD1_SET_TEMP_VP                  0x1038
#define HEAD1_CURRENT_TEMP_VP              0x1052
#define BED_SET_TEMP_VP                    0x103A
#define BED_CURRENT_TEMP_VP                0x103C
#define AUTO_HOME_DISPLAY_ICON_VP          0x1042
#define AXIS_X_COORD_VP                    0x1048
#define AXIS_Y_COORD_VP                    0x104A
#define AXIS_Z_COORD_VP                    0x104C
#define HEAD0_FILAMENT_LOAD_DATA_VP        0x1054
#define HEAD1_FILAMENT_LOAD_DATA_VP        0x1058
#define PRINTER_MACHINE_TEXT_VP            0x1060
#define PRINTER_VERSION_TEXT_VP            0x106A
#define PRINTER_PRINTSIZE_TEXT_VP          0x1074
#define PRINTER_WEBSITE_TEXT_VP            0x107E
#define AUTO_BED_LEVEL_ICON_VP             0x108D
#define CHANGE_FILAMENT_ICON_VP            0x108E
#define TWO_EXTRUDER_HOTEND_XOFFSET_VP     0x1092
#define TWO_EXTRUDER_HOTEND_YOFFSET_VP     0x1094
#define TWO_EXTRUDER_HOTEND_ZOFFSET_VP     0x1096

#define AUTO_BED_LEVEL_1POINT_VP           0x4000 // changed to 0x4000 from 0x1100 by John Carlson

// added by John Carlson for new bed point leveling display
#define AUTO_BED_LEVEL_CUR_POINT_VP          0x0110
#define AUTO_BED_LEVEL_END_POINT          0x0112
#define AUTO_BED_LEVEL_MESH_VP            0x0111
#define PID_TUNING_RUNNING_VP             0x1001
#define PID_ICON_MODE_VP                  0x1003
#define PID_TEXT_OUT_VP                   0x205B

// add by John Carlson for new e-steps update
#define E0_SET_STEP_VP                        0x1103
#define E1_SET_STEP_VP                       0X1104
#define E0_SET_FLOW_VP                        0x1105
#define E1_SET_FLOW_VP                       0X1106
#define E0_SET_FAN_VP                        0x1107
#define E1_SET_FAN_VP                       0x1108
#define X_SET_STEP_VP                       0x1122
#define Y_SET_STEP_VP                       0x1124
#define Z_SET_STEP_VP                       0x1126

#define AUTO_TRAM_1TEXT_VP                 0x1120

#define PRINT_SURPLUS_TIME_HOUR_VP         0x1162
#define PRINT_SURPLUS_TIME_MIN_VP          0x1164
#define SELECT_MODE_ICON_VP                0x1166
#define CHANGE_SDCARD_ICON_VP              0x1168

#define MOTOR_FREE_ICON_VP                 0x1200
#define FILE1_SELECT_ICON_VP               0x1225

#define PAGE_STATUS_TEXT_VP                0x201E
#define FILE1_TEXT_VP                      0x205A

#define SELECT_FILE_TEXT_VP                0x219A
#define TWO_COLOR_MODE_ICON_VP             0x21B8
#define COPY_MODE_ICON_VP                  0x21B9
#define MIRROR_MODE_ICON_VP                0x21BA
#define SINGLE_MODE_ICON_VP                0x21BB
#define EXCHANGE_NOZZLE_ICON_VP            0x21BC
#define PRINT_MODE_ICON_VP                 0x21BD
#define PRINT_FILE_TEXT_VP                 0x21C0
#define Screen_Version_VP                  0x2200
#define FilenameNature                     0x6003
#define	Beep       					               ((unsigned long)0x02AF0100)
/************struct**************/
typedef struct DataBuf
{
  unsigned char len;
  unsigned char head[2];
  unsigned char command;
  unsigned long addr;
  unsigned long bytelen;
  unsigned short data[32];
  unsigned char reserv[4];
} DB;

typedef struct 
{
  char currentDir[MAXPATHNAMELENGTH];
  char currentFilePath[MAXPATHNAMELENGTH];
  uint16_t fileCount;
  uint16_t pages;
  int pageFileIndex;
  int currentPage;
  char currentDisplayFilename[20];
} FileInfo;

class RTSUI
{
  public:
  #if ENABLED(LCD_BED_LEVELING) && EITHER(PROBE_MANUALLY, MESH_BED_LEVELING)
    static bool wait_for_bl_move;
  #else
    static constexpr bool wait_for_bl_move = false;
  #endif
};

extern RTSUI rtsui;

class RTSSHOW
{
  public:
    RTSSHOW();
    int RTS_RecData();
    void RTS_SDCardInit(void);
    bool RTS_SD_Detected(void);
    void RTS_SDCardUpate(void);
    void RTS_SndData(void);
    void RTS_SndData(const String &, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(const char[], unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(char, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(unsigned char*, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(int, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(float, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(unsigned int,unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(long,unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(unsigned long,unsigned long, unsigned char = VarAddr_W);
    void RTS_SDcard_Stop();
    void RTS_HandleData();
    void RTS_Init();
    void InitCardList();
    void ShowFilesOnCardPage(int);
    FileInfo fileInfo;
    DB recdat;
    DB snddat;
  private:
    unsigned char databuf[SizeofDatabuf];
};

extern RTSSHOW rtscheck;

enum PROC_COM
{
  MainPageKey = 0,
  AdjustmentKey,
  PrintSpeedKey,
  StopPrintKey,
  PausePrintKey,
  ResumePrintKey,
  ZOffsetKey,
  TempScreenKey,
  CoolScreenKey,
  Heater0TempEnterKey,
  Heater1TempEnterKey,
  HotBedTempEnterKey,
  E0StepsKey,
  E1StepsKey,
  E0FlowKey,
  E1FlowKey,
  E0FanKey,
  E1FanKey,
  XStepsKey,
  YStepsKey,
  ZStepsKey,
  ChangeFilamentKey,
  SettingScreenKey,
  SettingBackKey,
  BedLevelFunKey,
  AxisPageSelectKey,
  XaxismoveKey,
  YaxismoveKey,
  ZaxismoveKey,
  SelectExtruderKey,
  Heater0LoadEnterKey,
  FilamentLoadKey,
  Heater1LoadEnterKey,
  SelectLanguageKey,
  FilamentCheckKey,
  PowerContinuePrintKey,
  PrintSelectModeKey,
  XhotendOffsetKey,
  YhotendOffsetKey,
  ZhotendOffsetKey,
  StoreMemoryKey,
  PrintFileKey,
  SelectFileKey,
  SaveEEPROM,
  ASettingsScreenKey,
  PIDScreenKey,
  ChangePageKey
};

const unsigned long Addrbuf[] = 
{
  0x1002,
  0x1004,
  0x1006,
  0x1008,
  0x100A,
  0x100C,
  0x1026,
  0x1030,
  0x1032,
  0x1034,
  0x1038,
  0x103A,
  0x1103,
  0x1104,
  0x1105,
  0x1106,
  0x1107,
  0x1108,
  0x1122,
  0x1124,
  0x1126,
  0x1109,
  0x103E,
  0x1040,
  0x1044,
  0x1046,
  0x1048,
  0x104A,
  0x104C,
  0x104E,
  0x1054,
  0x1056,
  0x1058,
  0x105C,
  0x105E,
  0x105F,
  0x1090,
  0x1092,
  0x1094,
  0x1096,
  0x1098,
  0x2198,
  0x2199,
  0x2202,
  0x103F,
  0x1080,
  0x110E,
  0
};

extern int EndsWith(const char*, const char*);
extern void SetExtruderMode(unsigned int, bool);
extern void RTSUpdate();
extern void RTSInit();

extern int Update_Time_Value;
extern bool PoweroffContinue;
extern bool sdcard_pause_check;

extern char save_dual_x_carriage_mode;
extern float current_position_x0_axis;
extern float current_position_x1_axis;

void RTS_PauseMoveAxisPage();
void RTS_AutoBedLevelPage();
void RTS_MoveAxisHoming();
void RTS_AlertBeep();

#endif
