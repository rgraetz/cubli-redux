int flip_time = 3000;
int dir_time = 6000;
int pwm_time = 100;
int value = LOW;
int test_GPIO = 12;
long currentT1, previousT1, currentT2, previousT2 = 0;

int M1_BRK = 1;
int M1_BRK_GPIO = 13;

int M1_DIR = 0;
int M1_DIR_GPIO = 32;

int M1_PWM = 0;
int M1_PWM_CH = 0;
int M1_PWM_GPIO = 33;

int M1_CHA = 0;
int M1_CHA_GPIO = 5;

int M1_CHB = 0;
int M1_CHB_GPIO = 17;

int START_GPIO = 19;

int BATT_AIN = 34;

#define TIMER_BIT 8
#define PWM_BASE_FREQ 20000
