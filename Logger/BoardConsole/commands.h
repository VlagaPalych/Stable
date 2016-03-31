#ifndef COMMANDS_H
#define COMMANDS_H

#include <QtGlobal>

typedef qint16 int16_t;
typedef quint8 uint8_t;
typedef qint8 int8_t;
typedef quint16 uint16_t;

#define TURN_EVERYTHING_OFF 'a'

#define STABILIZATION 'e'

#define CALIBRATION 'd'

#define STOP_MOTORS 'g'

#define TELEMETRY 'h'

#define MIN_PWM 'i'
#define MAX_PWM 'j'

#define PWM1 'm'
#define PWM2 'n'

#define KP 'o'
#define KI 'p'
#define KD 'q'

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

#define PARAMS_BITMASK_SYMBOL 'f'

#define BIT_ACCEL_X			0x00000001
#define BIT_ACCEL_Y			0x00000002
#define BIT_ACCEL_Z			0x00000004
#define BIT_GYRO_X			0x00000008
#define BIT_GYRO_Y			0x00000010
#define BIT_GYRO_Z			0x00000020
#define BIT_COMPASS_X		0x00000040
#define BIT_COMPASS_Y		0x00000080
#define BIT_COMPASS_Z		0x00000100
#define BIT_EULER_X			0x00000200
#define BIT_EULER_Y			0x00000400
#define BIT_EULER_Z			0x00000800
#define BIT_EULERRATE_X		0x00001000
#define BIT_EULERRATE_Y		0x00002000
#define BIT_EULERRATE_Z		0x00004000
#define BIT_PWM1			0x00008000
#define BIT_PWM2			0x00010000
#define BIT_FREQ1			0x00020000
#define BIT_FREQ2			0x00040000
#define BIT_F				0x00080000


typedef struct {
	int16_t accel[3];
	int16_t gyro[3];
	int16_t compass[3];
	float euler[3];
	float eulerRate[3];
	uint16_t pwm1;
	uint16_t pwm2;
	uint16_t freq1;
	uint16_t freq2;
	float f;
} Message;
extern uint8_t Message_Size;
#define MESSAGE_HEADER  0x21

void Message_ToByteArray(Message *message, uint8_t *a);

uint8_t Message_FromByteArray(const QByteArray &bytes, quint32 paramsBitMask, Message *message);

#endif
