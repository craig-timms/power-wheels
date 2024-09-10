// Pin definitions

// ADC inputs
#define THROTTLE_PIN 36
#define T1_PIN      27
#define T2_PIN      14
#define T3_PIN      12
#define T4_PIN      13
#define T5_PIN      15
#define VB_ADC_PIN  38  // Battery (system) Input voltage
#define IB_ADC_PIN  39  // Battery (system) Input current
#define VM_ADC_PIN  35  // Motor voltage ie post contactor
#define iM_ADC_PIN  34  // MCU current

// ADC Outputs
#define VREF_PIN    25  // MCU
#define IDRIVE_PIN  26  // MCU gate drive power

// Digital inputs
#define ON_PIN      4   // System switch
#define STATUS_PIN  19  // DO
#define SNSOUT_PIN  21  // INPUT
#define nFAULT_PIN  22  // INPUT
#define REV_PIN     37

// Digital Outputs
#define CONN_PIN    9
#define SLEEP_PIN   5 // DO
#define DIR_MTR_PIN 23
#define PWM_MTR_PIN 18
#define R_DRIVE_PIN 32
#define L_DRIVE_PIN 33
// #define STATUS      33


#define WINDOW_SIZE 5
