// Pin D3 for TCC1 (STEP)
// Pin D6 for TCC0 (DIR)
#include "Stepper_WiFi.h"

// SAMD21 TCC0/1 configurations
// Timer max. values
#define TimerTOP 0xFFFFFF // Timer MAX count 2**24 (24 bit counter)

#define COUNTING_UP 0
#define COUNTING_DOWN 1

// Timer prescalers
const byte prescaleBitsArray[8] = {0, 1, 2, 3, 4, 5, 6, 7};
const uint16_t prescaleFactorArray[8] = {1, 2, 4, 8, 16, 64, 256, 1024};

// AMPLITUDE CONTROL VARIABLES
#define STEPPING_ANGLE 1.8 // grades
#define MICROSTEPPING 32 // max is 32 for DRV8825, 16 for A4988
#define ANG_STEP STEPPING_ANGLE/MICROSTEPPING // grades
#define R_GEAR 7.45 // mm

// #define MOV_AMPLITUDE 0.1 // mm, total horizontal movement

// FREQUENCY CONTROL VARIABLES
#define INITIAL_DIR_FREQ 1.0 // Hz
float new_DIR_freq = INITIAL_DIR_FREQ;
volatile bool changeDIRFreq = false;

//AMPLITUDE CONTROL VARIABLES
volatile bool changeSTEPAmplitude = false;
volatile bool waitAtExtremes = false;
volatile bool stoppedAtExtremes = false;
volatile int amplitudeIncrementsCount = 0;

volatile bool send_CHANGE_OK = false;

void setup()
{
  //Serial.begin(115200);

  // while(!Serial); // Wait for serial to be ready

  // Print system limitation
  Serial.print("TCC0/1 (DIR/STEP) MIN FREQ = ");
  Serial.print((float)F_CPU/prescaleFactorArray[7]/TimerTOP/2., 4);
  Serial.println(" Hz");
  Serial.println("In PWM mode with Toogle output, output freqs. will be half of these (MODE IN USE).");
  
  setupGCLK();
  DIR_PWM_init();
  STEP_PWM_init();

  // connectToWiFi(); // use this to connect to existing WiFi network
  setupWiFiAP(); // use this to create an access point

  // Wait until client is connected
  if (!isClientConnected()) {
    waitForClient();
  }

  // Wait for amplitude to be received via WiFi
  while (!amplitudeRequestedViaWiFi());

  // Inform that amplitude has been received
  send_CHANGE_OK_WiFi();

  // Configure timers' frequencies
  DIRSetFrequency(INITIAL_DIR_FREQ);
  STEPSetFrequency(get_STEP_freq(INITIAL_DIR_FREQ));

  enableTimers();
}

void loop() {
  // Check if client is not connected or disconnects
  if (!isClientConnected()) {
    waitForClient();
  }

  // Check if new DIR frequency is sent via WiFi and command frequencies update
  // (also check for reset command)
  int res = newRequestViaWiFi();
  if (res == FREQ_CHANGE) {
    new_DIR_freq = NEW_FREQ_FROM_WIFI;
    changeDIRFreq = true;
    // If motor is stopped at a extreme, this resets normal behaviour
    if (stoppedAtExtremes) {
      stoppedAtExtremes = false;
      enableTimers();
    }
  }else if (res == AMP_CHANGE) {
    changeSTEPAmplitude = true;
    // If motor is stopped at a extreme, it must be reenabled
    if (stoppedAtExtremes) {
      changeSTEPAmplitude = false;
      invertDIRoutput();
      enableTimers();
    }
  }else if (res == WAIT_AT_EXTREMES) {
    // If motor is already stopped at a extreme, then move it to the other one
    if (stoppedAtExtremes) {
      enableTimers();
    }
    // Then command wait interrupt
    waitAtExtremes = true;
  }

  // Check if frequency has been changed and inform client
  if (send_CHANGE_OK) {
    send_CHANGE_OK = false;
    send_CHANGE_OK_WiFi();
  }
}

// ------------------------------------------------------------------
// General clock functions

void setupGCLK() {
  // Configure clock source for TCCx at 48MHz and 50% duty cycle
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
}

void enableTimers() {
  // Enable outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  REG_TCC1_CTRLA |= TCC_CTRLA_ENABLE;             // Enable the TCC1 output
  while (TCC1->SYNCBUSY.bit.ENABLE && TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
}

void disableTimers() {
  // Stop Timers
  REG_TCC0_CTRLA &= ~TCC_CTRLA_ENABLE;
  while (TCC0->SYNCBUSY.bit.ENABLE);
  REG_TCC1_CTRLA &= ~TCC_CTRLA_ENABLE;
  while (TCC1->SYNCBUSY.bit.ENABLE);              // Wait for synchronization

  // Reset counters
  REG_TCC0_COUNT = 0;
  while(TCC0->SYNCBUSY.bit.COUNT);
  REG_TCC1_COUNT = 0;
  while(TCC1->SYNCBUSY.bit.COUNT);
}

void invertDIRoutput() {
  // Invert WO[6] output (PA20)
  REG_TCC0_DRVCTRL |= TCC_DRVCTRL_INVEN6;
}

void disableInversionDIRoutput() {
  // Disable inversion on WO[6] output (PA20)
  REG_TCC0_DRVCTRL &= ~TCC_DRVCTRL_INVEN6;
}

// ------------------------------------------------------------------
// DIR PWM functions

void DIR_PWM_init() {
  // Enable the port multiplexer for the TCC0 PWM channel 2 (digital pin D6), SAMD21 pin PA20
  PORT->Group[g_APinDescription[6].ulPort].PINCFG[g_APinDescription[6].ulPin].bit.PMUXEN = 1;
  
  // Connect the TCC0 timer to the port outputs - port pins are paired odd PMUO and even PMUXE
  // F & E specify the timers: TCC0, TCC1 and TCC2
  PORT->Group[g_APinDescription[6].ulPort].PMUX[g_APinDescription[6].ulPin >> 1].reg |= /*PORT_PMUX_PMUXO_F |*/ PORT_PMUX_PMUXE_F;

  // Dual-slope PWM operation: timer countinuouslys counts up to PER register value and then down to 0
  // Overflow interrupt event occurs at 0 (DSBOTTOM)
  REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN_DSBOTTOM;         // Setup dual slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);                // Wait for synchronization

  // Interrupts 
  REG_TCC0_INTENSET = 0;              // disable all interrupts
  REG_TCC0_INTENSET |= TCC_INTENSET_OVF;  // Overflow capture
  REG_TCC0_INTENSET |= TCC_INTENSET_MC2;  // Match count on 2
  NVIC_SetPriority(TCC0_IRQn, 0);
  NVIC_EnableIRQ(TCC0_IRQn);
}

void DIRSetFrequency(float signal_frequency) {
  byte prescaleBits;
  uint16_t prescaleFactor;
  uint32_t top;

  // In Dual-slope mode counter freq. is twice output freq.
  // float frequency = 2UL * signal_frequency;

  // Find the smallest prescale factor that will fit the TOP value within 24 bits.
  // frequency = F_CPU / (2 * prescale *  TOP)
  // TOP = F_CPU / (2UL * prescale * frequency);

  int i = 0;
  do {
    prescaleBits = prescaleBitsArray[i];
    prescaleFactor = prescaleFactorArray[i++];
    top = F_CPU / (2UL * prescaleFactor * signal_frequency);
  }
  while (top > TimerTOP && i < 8);

  // Serial.print("prescaleBits ");
  // Serial.println(prescaleBits);

  // Serial.print("prescaleFactor ");
  // Serial.println(prescaleFactor);

  // Serial.print("TOP ");
  // Serial.println(top);

  // Divide the GCLK4 48MHz signal by corresponding prescaler
  // Original code REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1024 == 7 << TCC_CTRLA_PRESCALER_Pos
  REG_TCC0_CTRLA &= ~(7 << TCC_CTRLA_PRESCALER_Pos); // reset prescaler bits
  REG_TCC0_CTRLA |= prescaleBits << TCC_CTRLA_PRESCALER_Pos; // set new prescaler

  // Serial.print("REG_TCC0_CTRLA ");
  // Serial.println(REG_TCC0_CTRLA, BIN);
  
  // The PER register value determines the frequency
  REG_TCC0_PER = top;      // Set the frequency of the PWM on TCC0
  while(TCC0->SYNCBUSY.bit.PER);

  // The CCx register value determines the duty cycle
  REG_TCC0_CC2 = top/2;       // TCC0 CC2 - 50% duty cycle on D6
  while(TCC0->SYNCBUSY.bit.CC2);
}

// ------------------------------------------------------------------
// STEP PWM functions

float get_fixed_amplitude_A4988() {
  float a = 0;
  float b = 0;
  float c = 0;
  float AMPLITUDE = 0;

  if (AMPLITUDE_FROM_WIFI <= 5.0) {
    a = -0.40234403030202087;
    b = 2.6385563867328083;
    c = -0.20333233398465206;

    AMPLITUDE = (-b+sqrt(b*b-4*a*(c - AMPLITUDE_FROM_WIFI)))/(2*a);
  } else {
    AMPLITUDE = AMPLITUDE_FROM_WIFI;
  }

  return AMPLITUDE;
}

float get_fixed_amplitude_DRV8825() {
  float a = 0;
  float b = 0;
  float AMPLITUDE = 0;

  if (AMPLITUDE_FROM_WIFI <= 0.15) {
    a = 0.9397619045546778;
    b = -0.04348809518794858;
    
    AMPLITUDE = (AMPLITUDE_FROM_WIFI - b)/a;
  } else if (AMPLITUDE_FROM_WIFI < 0.9) {
    a = 1.1540701751667246;
    b = -0.09601461970764529;

    AMPLITUDE = (AMPLITUDE_FROM_WIFI - b)/a;
  } else if (AMPLITUDE_FROM_WIFI <= 5.0) {
    a = 1.009033823412879;
    b = 0.045701470731844385;

    AMPLITUDE = (AMPLITUDE_FROM_WIFI - b)/a;
  } else {
    AMPLITUDE = AMPLITUDE_FROM_WIFI;
  }

  return AMPLITUDE;
}

float get_STEP_freq(float DIR_freq) {
  float theta = get_fixed_amplitude_A4988()/R_GEAR*180./PI; // grados
  float totalSteps = floor(2*theta/ANG_STEP); // int part of totalSteps
  return 1./(((1./DIR_freq)/2.)/totalSteps); // Hz
}

void STEP_PWM_init() {
  // Enable the port multiplexer for the TCC1 PWM channel 1 (digital pin D3), SAMD21 pin PA11
  PORT->Group[g_APinDescription[3].ulPort].PINCFG[g_APinDescription[3].ulPin].bit.PMUXEN = 1;
  
  // Connect the TCC1 timer to the port outputs - port pins are paired odd PMUO and even PMUXE
  // F & E specify the timers: TCC0, TCC1 and TCC2
  PORT->Group[g_APinDescription[3].ulPort].PMUX[g_APinDescription[3].ulPin >> 1].reg |= PORT_PMUX_PMUXO_E;

  // Dual-slope PWM operation: timer countinuouslys counts up to PER register value and then down to 0
  // Overflow interrupt event occurs at 0 (DSBOTTOM)
  REG_TCC1_WAVE |= TCC_WAVE_WAVEGEN_DSBOTTOM;         // Setup dual slope PWM on TCC1
  while (TCC1->SYNCBUSY.bit.WAVE);                // Wait for synchronization
}

void STEPSetFrequency(float signal_frequency) {
  byte prescaleBits;
  uint16_t prescaleFactor;
  uint32_t top;

  // In Dual-slope mode counter freq. is twice output freq.
  // float frequency = 2UL * signal_frequency;

  // Find the smallest prescale factor that will fit the TOP value within 24 bits.
  // frequency = F_CPU / (2 * prescale *  TOP)
  // TOP = F_CPU / (2UL * prescale * frequency);

  int i = 0;
  do {
    prescaleBits = prescaleBitsArray[i];
    prescaleFactor = prescaleFactorArray[i++];
    top = F_CPU / (2UL * prescaleFactor * signal_frequency);
  }
  while (top > TimerTOP && i < 8);

  // Divide the GCLK4 48MHz signal by corresponding prescaler
  REG_TCC1_CTRLA &= ~(7 << TCC_CTRLA_PRESCALER_Pos); // reset prescaler bits
  REG_TCC1_CTRLA |= prescaleBits << TCC_CTRLA_PRESCALER_Pos; // set new prescaler
  
  // The PER register value determines the frequency
  REG_TCC1_PER = top;      // Set the frequency of the PWM on TCC1
  while(TCC1->SYNCBUSY.bit.PER);

  // The CCx register value determines the duty cycle
  REG_TCC1_CC1 = top/2;       // TCC1 CC1 - 50% duty cycle on D3
  while(TCC1->SYNCBUSY.bit.CC1);
}

// -----------------------------------------------------------------
// Interrupt service routine to change frequencies

void TCC0_Handler()
{
  if (REG_TCC0_INTFLAG & TCC_INTFLAG_OVF) {  // An overflow caused the interrupt
    REG_TCC0_INTFLAG |= TCC_INTFLAG_OVF;    // writing a one clears the ovf flag

    if (changeDIRFreq) {
      disableTimers();

      //Change frequencies
      DIRSetFrequency(new_DIR_freq);
      STEPSetFrequency(get_STEP_freq(new_DIR_freq));

      // Restart the timers
      enableTimers();

      // Disable frequency change trigger
      changeDIRFreq = false;

      // Send OK through WiFi
      send_CHANGE_OK = true;
    }

    if (changeSTEPAmplitude) {
      disableTimers();

      //Change frequencies
      DIRSetFrequency(new_DIR_freq);
      STEPSetFrequency(get_STEP_freq(new_DIR_freq));

      // Restart the timers
      enableTimers();

      // Disable amplitude change trigger
      changeSTEPAmplitude = false;

      // If stopped at a extreme, then command interrupt again to stop after amplitude change
      // and dont send OK, because it will be sent in waitAtExtremes interrupt
      if (stoppedAtExtremes) {
        waitAtExtremes = true;
      } else {
        // Send OK through WiFi
        send_CHANGE_OK = true;
      }
    }
  }else if (REG_TCC0_INTFLAG & TCC_INTFLAG_MC2) { // compare match on 2 caused the interrupt
    REG_TCC0_INTFLAG |= TCC_INTFLAG_MC2;    // writing a one clears the mc2 flag

    // 1. Stop when DIR=1 (counting down) at -A. Remember that when motors are enabled they
    //    think they are at the equilibrim point
    // 2. Change to stop at DIR=0 and enable motors two times, thus moving to +A
    // 3. Invert output WO[x] and move motors once with DIR=0, thus ending in equilibrium point
    // 4. Restart normal movement at current amplitude and frequency. Change amplitude and go to 1.

    if (waitAtExtremes) {
      switch (amplitudeIncrementsCount) {
        case 0:
          // Step 1
          if (REG_TCC0_CTRLBSET == COUNTING_DOWN) {
            disableTimers();

            // Disable trigger
            waitAtExtremes = false;

            // Indicate stopped state
            stoppedAtExtremes = true;

            // Send OK through WiFi
            send_CHANGE_OK = true;

            amplitudeIncrementsCount++;
          }
        break;

        case 1:
          // Step 2
          if (REG_TCC0_CTRLBSET == COUNTING_UP) {
            disableTimers();
            enableTimers();
            amplitudeIncrementsCount++;
          }
        break;

        case 2:
          // Step 2, second half
          if (REG_TCC0_CTRLBSET == COUNTING_UP) {
            disableTimers();
            invertDIRoutput();
            send_CHANGE_OK = true;

            amplitudeIncrementsCount++;
          }
        break;

        case 3:
          // Step 3
          if (REG_TCC0_CTRLBSET == COUNTING_UP) {
            disableTimers();
            disableInversionDIRoutput();

            waitAtExtremes = false;
            stoppedAtExtremes = false;
            amplitudeIncrementsCount = 0;
            send_CHANGE_OK = true;

            enableTimers();
          }
        break;
      }
    }
  }
}

// ----------------------------------------------------------------------------
// Useful references
// (Arduino Zero & MKR1000 both use SAMD21)
// SAMD21 datasheet: https://ww1.microchip.com/downloads/en/DeviceDoc/SAM_D21_DA1_Family_DataSheet_DS40001882F.pdf
//                   https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-42454-Advanced-Features-of-SAM-D21-Timer-Counter-for-Control-Applications_Application-Note_AT12031.pdf
// SAMD21 TCC0 & TCC1 example to get pin outputs: https://forum.arduino.cc/t/timer-synchronization-in-arduino-zero/1146234/3
// SAMD21 TCC0 example to define timer logic control: https://forum.arduino.cc/t/changing-arduino-zero-pwm-frequency/334231/223
// SAMD21 Timer interrupts example: https://forum.arduino.cc/t/how-to-setup-timer-interrupts/383295/2
// SAMD21 Timers functioning explained: http://www.technoblogy.com/show?3RC9
// SAMD21 Another Timer example explained: https://blog.thea.codes/phase-shifted-pwm-on-samd/

// TCCx channel and PA outputs: https://tinygo.org/docs/reference/microcontrollers/machine/arduino-mkr1000/
// MKR1000 pinout: https://docs.arduino.cc/hardware/mkr-1000-wifi