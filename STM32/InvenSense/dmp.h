#ifndef DMP_H
#define DMP_H

#include "stdint.h"
#include "extra_math.h"
void quaternionToEuler(Quat *q, float* x, float* y, float* z );

// Known MPU9250 registers concerning DMP
#define BANK_SEL                0x6d
#define MEM_R_W                 0x6f
#define PRGM_START_H            0x70


// DMP memory areas
#define CFG_LP_QUAT             (2712)
#define END_ORIENT_TEMP         (1866)
#define CFG_27                  (2742)
#define CFG_20                  (2224)
#define CFG_23                  (2745)
#define CFG_FIFO_ON_EVENT       (2690)
#define END_PREDICTION_UPDATE   (1761)
#define CGNOTICE_INTR           (2620)
#define X_GRT_Y_TMP             (1358)
#define CFG_DR_INT              (1029)
#define CFG_AUTH                (1035)
#define UPDATE_PROP_ROT         (1835)
#define END_COMPARE_Y_X_TMP2    (1455)
#define SKIP_X_GRT_Y_TMP        (1359)
#define SKIP_END_COMPARE        (1435)
#define FCFG_3                  (1088)
#define FCFG_2                  (1066)
#define FCFG_1                  (1062)
#define END_COMPARE_Y_X_TMP3    (1434)
#define FCFG_7                  (1073)
#define FCFG_6                  (1106)
#define FLAT_STATE_END          (1713)
#define SWING_END_4             (1616)
#define SWING_END_2             (1565)
#define SWING_END_3             (1587)
#define SWING_END_1             (1550)
#define CFG_8                   (2718)
#define CFG_15                  (2727)
#define CFG_16                  (2746)
#define CFG_EXT_GYRO_BIAS       (1189)
#define END_COMPARE_Y_X_TMP     (1407)
#define DO_NOT_UPDATE_PROP_ROT  (1839)
#define CFG_7                   (1205)
#define FLAT_STATE_END_TEMP     (1683)
#define END_COMPARE_Y_X         (1484)
#define SKIP_SWING_END_1        (1551)
#define SKIP_SWING_END_3        (1588)
#define SKIP_SWING_END_2        (1566)
#define TILTG75_START           (1672)
#define CFG_6                   (2753)
#define TILTL75_END             (1669)
#define END_ORIENT              (1884)
#define CFG_FLICK_IN            (2573)
#define TILTL75_START           (1643)
#define CFG_MOTION_BIAS         (1208)
#define X_GRT_Y                 (1408)
#define TEMPLABEL               (2324)
#define CFG_ANDROID_ORIENT_INT  (1853)
#define CFG_GYRO_RAW_DATA       (2722)
#define X_GRT_Y_TMP2            (1379)

#define D_0_22                  (22+512)
#define D_0_24                  (24+512)

#define D_0_36                  (36)
#define D_0_52                  (52)
#define D_0_96                  (96)
#define D_0_104                 (104)
#define D_0_108                 (108)
#define D_0_163                 (163)
#define D_0_188                 (188)
#define D_0_192                 (192)
#define D_0_224                 (224)
#define D_0_228                 (228)
#define D_0_232                 (232)
#define D_0_236                 (236)

#define D_1_2                   (256 + 2)
#define D_1_4                   (256 + 4)
#define D_1_8                   (256 + 8)
#define D_1_10                  (256 + 10)
#define D_1_24                  (256 + 24)
#define D_1_28                  (256 + 28)
#define D_1_36                  (256 + 36)
#define D_1_40                  (256 + 40)
#define D_1_44                  (256 + 44)
#define D_1_72                  (256 + 72)
#define D_1_74                  (256 + 74)
#define D_1_79                  (256 + 79)
#define D_1_88                  (256 + 88)
#define D_1_90                  (256 + 90)
#define D_1_92                  (256 + 92)
#define D_1_96                  (256 + 96)
#define D_1_98                  (256 + 98)
#define D_1_106                 (256 + 106)
#define D_1_108                 (256 + 108)
#define D_1_112                 (256 + 112)
#define D_1_128                 (256 + 144)
#define D_1_152                 (256 + 12)
#define D_1_160                 (256 + 160)
#define D_1_176                 (256 + 176)
#define D_1_178                 (256 + 178)
#define D_1_218                 (256 + 218)
#define D_1_232                 (256 + 232)
#define D_1_236                 (256 + 236)
#define D_1_240                 (256 + 240)
#define D_1_244                 (256 + 244)
#define D_1_250                 (256 + 250)
#define D_1_252                 (256 + 252)
#define D_2_12                  (512 + 12)
#define D_2_96                  (512 + 96)
#define D_2_108                 (512 + 108)
#define D_2_208                 (512 + 208)
#define D_2_224                 (512 + 224)
#define D_2_236                 (512 + 236)
#define D_2_244                 (512 + 244)
#define D_2_248                 (512 + 248)
#define D_2_252                 (512 + 252)




#define TAP_X               (0x01)
#define TAP_Y               (0x02)
#define TAP_Z               (0x04)
#define TAP_XYZ             (0x07)

#define TAP_X_UP            (0x01)
#define TAP_X_DOWN          (0x02)
#define TAP_Y_UP            (0x03)
#define TAP_Y_DOWN          (0x04)
#define TAP_Z_UP            (0x05)
#define TAP_Z_DOWN          (0x06)


#define DMP_FEATURE_TAP             0x001
#define DMP_FEATURE_ANDROID_ORIENT  0x002
#define DMP_FEATURE_LP_QUAT         0x004
#define DMP_FEATURE_PEDOMETER       0x008
#define DMP_FEATURE_6X_LP_QUAT      0x010
#define DMP_FEATURE_GYRO_CAL        0x020
#define DMP_FEATURE_SEND_RAW_ACCEL  0x040
#define DMP_FEATURE_SEND_RAW_GYRO   0x080
#define DMP_FEATURE_SEND_CAL_GYRO   0x100
#define DMP_FEATURE_SEND_ANY_GYRO   (DMP_FEATURE_SEND_RAW_GYRO | \
                                     DMP_FEATURE_SEND_CAL_GYRO)
                                     
#define DMP_SAMPLE_RATE     (200)
#define GYRO_SF             (46850825LL * 200 / DMP_SAMPLE_RATE)


#define DINA0A 0x0a
#define DINA22 0x22
#define DINA42 0x42
#define DINA5A 0x5a

#define DINA06 0x06
#define DINA0E 0x0e
#define DINA16 0x16
#define DINA1E 0x1e
#define DINA26 0x26
#define DINA2E 0x2e
#define DINA36 0x36
#define DINA3E 0x3e
#define DINA46 0x46
#define DINA4E 0x4e
#define DINA56 0x56
#define DINA5E 0x5e
#define DINA66 0x66
#define DINA6E 0x6e
#define DINA76 0x76
#define DINA7E 0x7e

#define DINA00 0x00
#define DINA08 0x08
#define DINA10 0x10
#define DINA18 0x18
#define DINA20 0x20
#define DINA28 0x28
#define DINA30 0x30
#define DINA38 0x38
#define DINA40 0x40
#define DINA48 0x48
#define DINA50 0x50
#define DINA58 0x58
#define DINA60 0x60
#define DINA68 0x68
#define DINA70 0x70
#define DINA78 0x78

#define DINA04 0x04
#define DINA0C 0x0c
#define DINA14 0x14
#define DINA1C 0x1C
#define DINA24 0x24
#define DINA2C 0x2c
#define DINA34 0x34
#define DINA3C 0x3c
#define DINA44 0x44
#define DINA4C 0x4c
#define DINA54 0x54
#define DINA5C 0x5c
#define DINA64 0x64
#define DINA6C 0x6c
#define DINA74 0x74
#define DINA7C 0x7c

#define DINA01 0x01
#define DINA09 0x09
#define DINA11 0x11
#define DINA19 0x19
#define DINA21 0x21
#define DINA29 0x29
#define DINA31 0x31
#define DINA39 0x39
#define DINA41 0x41
#define DINA49 0x49
#define DINA51 0x51
#define DINA59 0x59
#define DINA61 0x61
#define DINA69 0x69
#define DINA71 0x71
#define DINA79 0x79

#define DINA25 0x25
#define DINA2D 0x2d
#define DINA35 0x35
#define DINA3D 0x3d
#define DINA4D 0x4d
#define DINA55 0x55
#define DINA5D 0x5D
#define DINA6D 0x6d
#define DINA75 0x75
#define DINA7D 0x7d

#define DINADC 0xdc
#define DINAF2 0xf2
#define DINAAB 0xab
#define DINAAA 0xaa
#define DINAF1 0xf1
#define DINADF 0xdf
#define DINADA 0xda
#define DINAB1 0xb1
#define DINAB9 0xb9
#define DINAF3 0xf3
#define DINA8B 0x8b
#define DINAA3 0xa3
#define DINA91 0x91
#define DINAB6 0xb6
#define DINAB4 0xb4


#define DINC00 0x00
#define DINC01 0x01
#define DINC02 0x02
#define DINC03 0x03
#define DINC08 0x08
#define DINC09 0x09
#define DINC0A 0x0a
#define DINC0B 0x0b
#define DINC10 0x10
#define DINC11 0x11
#define DINC12 0x12
#define DINC13 0x13
#define DINC18 0x18
#define DINC19 0x19
#define DINC1A 0x1a
#define DINC1B 0x1b

#define DINC20 0x20
#define DINC21 0x21
#define DINC22 0x22
#define DINC23 0x23
#define DINC28 0x28
#define DINC29 0x29
#define DINC2A 0x2a
#define DINC2B 0x2b
#define DINC30 0x30
#define DINC31 0x31
#define DINC32 0x32
#define DINC33 0x33
#define DINC38 0x38
#define DINC39 0x39
#define DINC3A 0x3a
#define DINC3B 0x3b

#define DINC40 0x40
#define DINC41 0x41
#define DINC42 0x42
#define DINC43 0x43
#define DINC48 0x48
#define DINC49 0x49
#define DINC4A 0x4a
#define DINC4B 0x4b
#define DINC50 0x50
#define DINC51 0x51
#define DINC52 0x52
#define DINC53 0x53
#define DINC58 0x58
#define DINC59 0x59
#define DINC5A 0x5a
#define DINC5B 0x5b

#define DINC60 0x60
#define DINC61 0x61
#define DINC62 0x62
#define DINC63 0x63
#define DINC68 0x68
#define DINC69 0x69
#define DINC6A 0x6a
#define DINC6B 0x6b
#define DINC70 0x70
#define DINC71 0x71
#define DINC72 0x72
#define DINC73 0x73
#define DINC78 0x78
#define DINC79 0x79
#define DINC7A 0x7a
#define DINC7B 0x7b

#define DIND40 0x40


#define DINA80 0x80
#define DINA90 0x90
#define DINAA0 0xa0
#define DINAC9 0xc9
#define DINACB 0xcb
#define DINACD 0xcd
#define DINACF 0xcf
#define DINAC8 0xc8
#define DINACA 0xca
#define DINACC 0xcc
#define DINACE 0xce
#define DINAD8 0xd8
#define DINADD 0xdd
#define DINAF8 0xf0
#define DINAFE 0xfe

#define DINBF8 0xf8
#define DINAC0 0xb0
#define DINAC1 0xb1
#define DINAC2 0xb4
#define DINAC3 0xb5
#define DINAC4 0xb8
#define DINAC5 0xb9
#define DINBC0 0xc0
#define DINBC2 0xc2
#define DINBC4 0xc4
#define DINBC6 0xc6

#define DMP_CODE_SIZE 3062

typedef struct {
    void (*tap_cb)(unsigned char count, unsigned char direction);
    void (*android_orient_cb)(unsigned char orientation);
    unsigned short orient;
    unsigned short feature_mask;
    unsigned short fifo_rate;
    unsigned char packet_length;
} DMP;

int DMP_SelectDevice(int device);
void DMP_InitStructures(void);
int DMP_LoadMotionDriverFirmware(void);
int DMP_SetFIFORate(uint16_t rate);
int DMP_EnableGyroCal(uint8_t enable);
int DMP_EnableLPQuat(uint8_t enable); 
int DMP_Enable6XLPQuat(uint8_t enable);
int DMP_EnableFeature(uint16_t mask);
void DMP_ParseFIFOData(uint8_t *fifo_data);

#endif // DMP_H
