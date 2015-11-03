#ifndef COMMANDS_H
#define COMMANDS_H

#define TURN_EVERYTHING_OFF 'a'

#define STABILIZATION 'e'

#define CALIBRATION 'd'

#define LOWPASS 'f'

#define STOP_MOTORS 'g'

#define TELEMETRY 'h'

#define MIN_PWM 'i'
#define MAX_PWM 'j'

#define MAX_ANGLE 'k'
#define ACCEL_DEVIATION 'l'
#define BOUNDARY_ANGLE 'r'
#define MAX_ANGVEL 's'

#define PWM1 'm'
#define PWM2 'n'

#define KP 'o'
#define KI 'p'
#define KD 'q'

#define TURN_USELESS 't'
#define GYRO_RECALIBRATION 'u'
#define TRANQUILITY_TIME 'v'
#define PWM_STEP 'w'
#define EVERY_N 'x'

#define ACCEL_FREQ_HZ25     'A'
#define ACCEL_FREQ_HZ50     'B'
#define ACCEL_FREQ_HZ100    'C'
#define ACCEL_FREQ_HZ800    'D'
#define ACCEL_FREQ_HZ1600   'E'
#define ACCEL_FREQ_HZ3200   'F'

#define GYRO_FREQ_HZ100     'G'
#define GYRO_FREQ_HZ250     'H'
#define GYRO_FREQ_HZ500     'I'
#define GYRO_FREQ_HZ1000    'J'

#define IMPULSE             'K'
#define STEP                'L'
#define SINE                'M'
#define EXP                 'N'
#define NO_RESEARCH_SYMBOL  'O'
#define SIMPLE              'P'
#define PID					'Q'
#define OPERATOR			'R'
#define ADJUST				'S'

#define PROGRAMMING_MODE 'Z'

#define NUMBER_END 'b'

#include <QtGlobal>

typedef qint16 int16_t;
typedef quint8 uint8_t;

typedef struct {
	//    float ars1_x;
	//    float ars1_y;
	//    int16_t ars1_t;
	//    float ars2_x;
	//    float ars2_y;
	//    int16_t ars2_t;
	//    float ars3_z;
	//    
	//    int16_t accel_x;
	//    int16_t accel_y;
	//    int16_t accel_z;
	float roll;
	float pitch;
} Message;

extern uint8_t Message_Size;
#define MESSAGE_HEADER  0x21

void Message_ToByteArray(Message *message, uint8_t *a);

uint8_t Message_FromByteArray(uint8_t *a, uint8_t n, Message *message); 

#endif
