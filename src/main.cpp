////////////////////////////////////////////////////////////////
//  Board: STM32F103C8T6 "Blue Pill"
//  Core: ARM Cortex-M3
//  Clock: 72 MHz
//  IDE: PlatformIO
//  Framework: Arduino
//  Version: 1.0
////////////////////////////////////////////////////////////////
//  Description:
////////////////////////////////////////////////////////////////
//  This is a DDS Driver for the W2HAT Costas Array beacon.
//
//  The AD9850 is loaded by an STM32F103C8T6 MCU, which is a 
//  helper core to the onboard GW2AR FPGA. It also provides
//  a serial interface to the GPSDO unit for timestamping.
//  The timestamps are transmitted the next second after the
//  beacon is triggered. Timestamps are DPSK modulated. 
//  Timestamp transmission and format is WIP.
//
//  The DDS frequency output is triggered by an FPGA which is 
//  programmed to provide precise timing based on an external
//  GPSDO.
//  This code loads a word into the AD9850 when the trigger 
//  pin is high. It then waits for the FQ_UD pin to go high 
//  before loading another word (to prevent race conditions).
//  
//  The Costas array and its parameters, as well as pin 
//  definitions, are defined in the user configuration section.
////////////////////////////////////////////////////////////////
// Author: Ruslan Gindullin, W2HAT
// Date: 2025-02-16
////////////////////////////////////////////////////////////////
// For the HamSCI 2025 Workshop
////////////////////////////////////////////////////////////////

#include <Arduino.h>

////////////////////////////////////////////////////////////////
// User configuration
////////////////////////////////////////////////////////////////
#define CARRIER_FREQ 10.0 // "Carrier" (suppressed) frequency in MHz
#define INITIAL_PHASE 0.0 // Initial phase in degrees
#define COSTAS_ARR_SIZE 7 // Size of the Costas array
#define COSTAS_ARR_FREQ_STEP 100 // Frequency resolution of the costas array in Hz
#define ARRAY {3, 1, 4, 0, 6, 5, 2} // Costas array
////////////////////////////////////////////////////////////////
// End of user configuration
////////////////////////////////////////////////////////////////

// Define pin connections
#define TRIG_PIN PB4
#define FQ_UD_PIN PB5
#define DATA_PIN PB6
#define CLOCK_PIN PB7

// put function declarations here:
void loadAD9850(uint64_t frequency, uint8_t phase);
uint32_t calculateFrequencyWord(float frequencyMHz);
uint8_t calculatePhaseWord(float phaseDegrees);
uint64_t calculateWaveform(uint64_t frequency, uint8_t phase);

// Define the reference clock frequency in MHz
#define REF_CLK 125.0

// Function to load bits into AD9850
void loadAD9850(uint64_t word) {
  // Load the 32-bit frequency word
  for (int i = 0; i < 40; i++) {
    digitalWrite(DATA_PIN, (word >> i) & 0x01);
    digitalWrite(CLOCK_PIN, HIGH);
    delayMicroseconds(1);
    digitalWrite(CLOCK_PIN, LOW);
    delayMicroseconds(1);
  }
}

// Function to calculate the frequency word
uint32_t calculateFrequencyWord(float frequencyMHz) {
  return (uint64_t)((frequencyMHz * (1ULL << 32)) / REF_CLK);
}
uint8_t calculatePhaseWord(float phaseDegrees) {
  return (uint8_t)((phaseDegrees * (1ULL << 8)) / 360);
}
uint64_t calculateWaveform(uint64_t frequency, uint8_t phase) {
  return (calculateFrequencyWord(frequency) << 8) | calculatePhaseWord(phase);
}

bool readyToLoad;

void setup() {
  pinMode(TRIG_PIN, INPUT);
  pinMode(FQ_UD_PIN, INPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  readyToLoad = true;
}

void loop() {

  if (readyToLoad && digitalRead(TRIG_PIN) == HIGH) {
    loadAD9850(loadWord);
    readyToLoad = false;
  }

  if (digitalRead(FQ_UD_PIN) == HIGH) {
    readyToLoad = true;
  }
}