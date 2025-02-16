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
// Parameters
#define CARRIER_FREQ 10000000           // "Carrier" (suppressed) frequency in Hz
#define CARRIER_OFFSET 1000             // Offset from the carrier frequency in Hz
#define INITIAL_PHASE 0                 // Initial phase in degrees (0-180)
#define COSTAS_ARR_SIZE 7               // Size of the Costas array
#define COSTAS_ARR_FREQ_STEP 100        // Frequency resolution of the costas array in Hz
#define ARRAY {3, 1, 4, 0, 6, 5, 2}     // Costas array
#define DATA_SIZE 25                     // number of bytes in the data array
#define BEACON_ID_MSG "W2HAT COSTAS ARRAY BEACON" // Beacon ID message
#define REF_CLK 125000000               // Reference clock frequency in Hz
// Pin definitions
#define COSTAS_TRIGGER_PIN PB2          // Costas trigger pin
#define PSK_TRIG_PIN PB3                // Trigger pin
#define COSTAS_TRIG_PIN PB4             // Costas trigger pin
#define FQ_UD_PIN PB5                   // FQ_UD pin
#define DATA_PIN PB6                    // Data pin
#define CLOCK_PIN PB7                   // Clock pin
#define COSTAS_TX_REQ_PIN PB8           // Costas transmit request pin
#define PSK_TX_REQ_PIN PB9              // BPSK transmit request pin
#define PSK_CLK_PIN PB10                // BPSK clock pin
////////////////////////////////////////////////////////////////
// End of user configuration
////////////////////////////////////////////////////////////////

// Function Prototypes
void loadAD9850(uint64_t loadWord);                                   // Load the AD9850 with the word 
void txCostasArray(uint32_t costasArray[], uint8_t costasArraySize);  // Transmit the Costas array
void txPSK(uint8_t data[], uint8_t data_size);                        // Transmit PSK data
void calculateArray(uint64_t CarrierFreq, uint32_t CostasFreqStep, uint8_t CostasArraySize, uint8_t Array[], uint32_t Offset); // Calculate the Costas array
uint32_t calculateFrequencyWord(uint64_t frequency);                  // Calculate the frequency word
uint8_t calculatePhaseWord(uint32_t phaseDegrees);                       // Calculate the phase word
uint64_t calculateWaveform(uint64_t frequency, uint32_t phase);        // Calculate the full waveform word


// Variable Definitions
bool readyToLoad;
uint8_t costasCounter = 0;
uint32_t costasArray[COSTAS_ARR_SIZE];
uint8_t arrayinit[COSTAS_ARR_SIZE] = ARRAY;
uint64_t loadWord;
uint8_t data[DATA_SIZE]; // Data to be transmitted
// Function to load bits into AD9850
void loadAD9850(uint64_t word) {
  for (int i = 0; i < 40; i++) {
    digitalWrite(DATA_PIN, (word >> i) & 0x01);
    digitalWrite(CLOCK_PIN, HIGH);
    delayMicroseconds(1);
    digitalWrite(CLOCK_PIN, LOW);
    delayMicroseconds(1);
  }
}

// Function to calculate the frequency word
uint32_t calculateFrequencyWord(uint64_t frequency) {
  return (uint32_t)((frequency * (1ULL << 32)) / REF_CLK);
}

// Function to calculate the phase word
uint8_t calculatePhaseWord(uint32_t phaseDegrees) {
  return (uint8_t)((phaseDegrees * (1ULL << 5)) / 360);
}

// Function to calculate the full waveform word
uint64_t calculateWaveform(uint64_t frequency, uint32_t phase) {
  return ((uint64_t)calculatePhaseWord(phase) << 32) | calculateFrequencyWord(frequency) ;
}

// Function to calculate the Costas array
void calculateArray(uint64_t CarrierFreq, uint32_t CostasFreqStep, uint8_t CostasArraySize, uint8_t Array[], uint32_t Offset) {
  for (int i = 0; i < CostasArraySize; i++) {
    costasArray[i] = Array[i] * CostasFreqStep + CarrierFreq + Offset;  
  }
}
uint8_t* calculateData(String stuff, uint8_t *data) {
  for (int i = 0; i < (sizeof(stuff) > DATA_SIZE)?DATA_SIZE:sizeof(stuff); i++) {
    data[i] = stuff.charAt(i);
  }
  return data;
}
// Function to transmit PSK data
void txPSK(uint8_t data[], uint8_t data_size) {
  digitalWrite(PSK_TX_REQ_PIN, HIGH);  // Start transmission
  loadWord = calculateWaveform(CARRIER_FREQ, INITIAL_PHASE); 
  loadAD9850(loadWord);              // Load the carrier frequency
  for( int i = 0; i < 8; i++) {      // Transmit the preamble (8 bits long)
    while(!digitalRead(FQ_UD_PIN));     // Wait for the last word to be loaded
    while(!digitalRead(PSK_CLK_PIN));   // Wait for the trigger
    loadAD9850(loadWord);               // Load the carrier frequency
  }
  for (int i = 0; i < data_size; i++) { // Transmit the PSK data (little endian, byte array format) LSB first out, MSB last out.
    for (int j = 0; j < 8; j++) {                 // Transmit each byte
      while(!digitalRead(FQ_UD_PIN));                 // Wait for the last word to be loaded
      while(!digitalRead(PSK_CLK_PIN));               // Wait for the trigger
      loadWord = calculateWaveform(CARRIER_FREQ, (data[i] >> j) * 180+INITIAL_PHASE); // Calculate the word
      loadAD9850(loadWord);                           // Load the word
    }
  }
  while(!digitalRead(FQ_UD_PIN));     // Wait for the last word to be loaded
  while(!digitalRead(PSK_CLK_PIN));   // Wait for the trigger
  loadWord = calculateWaveform(calculateFrequencyWord(0), calculatePhaseWord(0)); 
  loadAD9850(loadWord);               // Load an empty word
  while(!digitalRead(FQ_UD_PIN));     // Wait for the last word to be loaded
  while(!digitalRead(PSK_CLK_PIN));   // Wait for the trigger
  digitalWrite(PSK_TX_REQ_PIN, LOW);  // End transmission
}

// Function to transmit the Costas array
void txCostasArray(uint32_t costasArray[], uint8_t costasArraySize) {
  digitalWrite(COSTAS_TX_REQ_PIN, HIGH); // Start transmission
  for (int i = 0; i < costasArraySize; i++) { // Transmit the Costas array
    while(!digitalRead(COSTAS_TRIGGER_PIN)); // Wait for the trigger
    loadWord = calculateWaveform(costasArray[i], calculatePhaseWord(INITIAL_PHASE));
    loadAD9850(loadWord);
    while(!digitalRead(FQ_UD_PIN));     // Wait for the word to be loaded
  }
  while(!digitalRead(COSTAS_TRIGGER_PIN));  // Wait for the trigger
  loadWord = calculateWaveform(calculateFrequencyWord(0), calculatePhaseWord(0)); 
  loadAD9850(loadWord);                 // Load an empty word        
  while(!digitalRead(FQ_UD_PIN));       // Wait for the word to be loaded
  digitalWrite(COSTAS_TX_REQ_PIN, LOW); // End transmission
}

void setup() {

  // Initialize the AD9850
  pinMode(FQ_UD_PIN, INPUT);
  pinMode(PSK_TX_REQ_PIN, OUTPUT);
  pinMode(PSK_TRIG_PIN, INPUT);
  pinMode(PSK_CLK_PIN, INPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(COSTAS_TX_REQ_PIN, OUTPUT);
  pinMode(COSTAS_TRIGGER_PIN, INPUT);
  pinMode(COSTAS_TRIG_PIN, INPUT);
  Serial.begin(9600);
  loadAD9850((0ULL << 40));     // Load an empty word
  loadWord = 0;                 // Initialize the load word variable
  digitalWrite(COSTAS_TX_REQ_PIN, HIGH); // Enable costas clock
  while(!digitalRead(FQ_UD_PIN));        // Wait for the last word to be loaded
  digitalWrite(COSTAS_TX_REQ_PIN, LOW);  // Disable costas clock
  calculateArray(CARRIER_FREQ, COSTAS_ARR_FREQ_STEP, COSTAS_ARR_SIZE, arrayinit, CARRIER_OFFSET);
  Serial.println("AD9850 Initialized");
}

void loop() {
  // Calculate the Costas array
  if(digitalRead(COSTAS_TRIGGER_PIN)) {
    Serial.println("Costas Triggered");
    txCostasArray(costasArray, COSTAS_ARR_SIZE);
    Serial.println("Costas Transmitted");
    Serial.println("Costas Array first element:");
    Serial.println(costasArray[0]);
  }
  // Calculate the data
  if(digitalRead(PSK_TRIG_PIN)) {
  Serial.println("PSK Triggered");
  calculateData(BEACON_ID_MSG, data);
  // Transmit the data
  txPSK(data, DATA_SIZE);
  Serial.println("PSK Transmitted:");
  Serial.println(BEACON_ID_MSG);
  }
}