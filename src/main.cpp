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
//  a serial interface to the GPSDO unit for timestamping (TODO!).
//  The timestamps are transmitted the next second after the
//  beacon is triggered. Timestamps are BPSK modulated.
//   
//  (Currently implemented is a simple beacon ID once per 10
//  minutes, as per FCC regulations.)
//
//  The DDS frequency output is triggered by an FPGA which is 
//  programmed to provide precise timing based on an external
//  GPSDO.
//
//  This code loads a word into the AD9850 when a trigger 
//  pin is high. It then waits for the FQ_UD pin to go high 
//  before loading another word (to prevent race conditions).
//  After the last word is loaded, the trigger pin is set low and
//  the MCU enters a NOP loop until unlock pin is set high.
//  Then, the process is repeated.
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
// Uncomment the following line to enable debugging
//#define DEBUG
////////////////////////////////////////////////////////////////
// Parameters
#define CARRIER_FREQ 1000000           // "Carrier" (suppressed) frequency in Hz
#define CARRIER_OFFSET 1000             // Offset from the carrier frequency in Hz
#define INITIAL_PHASE 0                 // Initial phase in degrees (0-180)
#define COSTAS_ARR_SIZE 7               // Size of the Costas array
#define COSTAS_ARR_FREQ_STEP 100        // Frequency resolution of the costas array in Hz
#define ARRAY {3, 1, 4, 0, 6, 5, 2}     // Costas array
#define DATA_SIZE 25                    // number of bytes in the data array
#define BEACON_ID_MSG "W2HAT COSTAS ARRAY BEACON" // Beacon ID message
#define REF_CLK 125000000               // Reference clock frequency in Hz
// Pin definitions
#define COSTAS_CLK_PIN PB14             // Costas clock pin
#define COSTAS_TRIG_PIN PB5             // Costas trigger pin
#define PSK_TRIG_PIN PB3                // Trigger pin
#define FQ_UD_PIN PB12                  // FQ_UD pin
#define DATA_PIN PA0                    // Data pin
#define CLOCK_PIN PB0                   // Clock pin
#define COSTAS_TX_REQ_PIN PB9           // Costas transmit request pin
#define PSK_TX_REQ_PIN PB7              // BPSK transmit request pin
#define PSK_CLK_PIN PA8                 // BPSK clock pin
#define COSTAS_UNLOCK_PIN PA12          // Costas unlock pin
#define PSK_UNLOCK_PIN PA10             // BPSK unlock pin
////////////////////////////////////////////////////////////////
// End of user configuration
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
// Function Prototypes:
////////////////////////////////////////////////////////////////
/**
 * @brief Loads a 40-bit control word into AD9850 DDS module using bit-banging
 */
void loadAD9850(uint64_t loadWord);                       
/**
 * @brief Transmits a Costas array sequence
 */
void txCostasArray(uint32_t costasArray[], uint8_t costasArraySize); 
/**
 * @brief Transmits data using Phase Shift Keying (PSK) modulation
 */
void txPSK(uint8_t data[], uint8_t data_size);                     
/**
 * @brief Calculates frequency values for Costas array
 */
void calculateArray(uint64_t CarrierFreq, uint32_t CostasFreqStep, uint8_t CostasArraySize, uint8_t Array[], uint32_t Offset); 
/**
 * @brief Calculates the frequency word for AD9850 DDS module
 */
uint32_t calculateFrequencyWord(uint64_t frequency);                 
/**
 * @brief Calculates the 5-bit phase word for AD9850 DDS from phase in degrees
 */
uint8_t calculatePhaseWord(uint32_t phaseDegrees);                
/**
 * @brief Combines frequency and phase information into a single 64-bit word for AD9850
 */
uint64_t calculateWaveform(uint64_t frequency, uint32_t phase);     
/**
 * @brief Calculate the data to be transmitted.
 */
void calculateData(String stuff, uint8_t *data);    
/**
 * @brief Idle loop, for improved readability of code not involved in active transmission
 */             
void idleLoop();

////////////////////////////////////////////////////////////////
// End of function prototypes
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
// Variable Definitions:
////////////////////////////////////////////////////////////////

// Global data buffers:
uint32_t costasArray[COSTAS_ARR_SIZE];      // Array containing frequencies for the Costas sequence
uint8_t arrayinit[COSTAS_ARR_SIZE] = ARRAY; // Initial Costas array
uint64_t loadWord;                          // Word to be loaded into AD9850
uint8_t data[DATA_SIZE];                    // Data to be transmitted

// Static variables for loadAD9850 function
static uint8_t bitIndexLoad = 0;            // Bit index for loading word
static uint64_t currentWordLoad = 0;
static bool isLoading = false;
static bool wordLoaded = false;

// Static variables for txPSK function
static enum {
  PSK_IDLE,
  PSK_START_TX,
  PSK_DATA_TX,
  PSK_END_TX
} pskState = PSK_IDLE;
static int byteIndexPSK = 0;
static int bitIndexPSK = 0;
static int preambleCount = 0;
static bool lockPSK = false;

// Static variables for txCostasArray function
static enum {
  COSTAS_IDLE,
  COSTAS_TRANSMITTING,
  COSTAS_FINAL_WORD,
  COSTAS_END_TX
} costasState = COSTAS_IDLE;
static int arrayIndexCostas = 0;
static bool lockCostas = false;

// Static variables for main loop
static enum {
  LOOP_IDLE,
  COSTAS_ACTIVE,
  PSK_ACTIVE
} loopState = LOOP_IDLE;

////////////////////////////////////////////////////////////////
// End of variable definitions
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
// Function Implementations:
////////////////////////////////////////////////////////////////

/**
 * @brief Loads a 40-bit control word into AD9850 DDS module using bit-banging
 * 
 * This function implements serial loading of the AD9850 DDS (Direct Digital Synthesis) module.
 * It shifts out 40 bits of data MSB first, with each bit being clocked by a rising edge
 * followed by a falling edge on the CLOCK_PIN.
 * 
 * @param word 64-bit value containing the 40-bit control word to be loaded
 *             (only lower 40 bits are used)
 * 
 * @note The function assumes:
 *       - DATA_PIN is on GPIOA
 *       - CLOCK_PIN is on GPIOB
 *       - Pins are already configured as outputs
 */
// Static variables to maintain state between calls


void loadAD9850(uint64_t word) {
  // Start new transmission if not already loading
  if (!isLoading) {
    currentWordLoad = word;
    bitIndexLoad = 0;
    isLoading = true;
  }

  // Process one bit at a time
  if (isLoading && bitIndexLoad < 40) {
    if ((currentWordLoad >> bitIndexLoad) & 0x01) {
      GPIOA->BSRR = (1 << (DATA_PIN & 0xF));
    } else {
      GPIOA->BRR = (1 << (DATA_PIN & 0xF));
    }
    GPIOB->BSRR = (1 << (CLOCK_PIN & 0xF));
    GPIOB->BRR = (1 << (CLOCK_PIN & 0xF));
    bitIndexLoad++;
  } else {
    // Transmission complete
    isLoading = false;
    wordLoaded = true;
  }
}

// Function to calculate the frequency word
/**
 * @brief Calculates the frequency word for AD9850 DDS module
 * 
 * This function converts the desired output frequency to a 32-bit frequency word
 * required by the AD9850 DDS module. The calculation uses the formula:
 * FREQ_WORD = (OUTPUT_FREQ * 2^32) / REF_CLK
 * 
 * @param frequency Desired output frequency in Hz (64-bit unsigned integer)
 * @return uint32_t 32-bit frequency tuning word for AD9850
 * 
 * @note REF_CLK must be defined as the reference clock frequency of AD9850 (typically 125MHz)
 */
uint32_t calculateFrequencyWord(uint64_t frequency) {
  return (uint32_t)((frequency * (1ULL << 32)) / REF_CLK);
}

// Function to calculate the phase word
/**
 * @brief Calculates the 5-bit phase word for AD9850 DDS from phase in degrees
 * 
 * This function converts a phase angle in degrees to a 5-bit phase word
 * required by the AD9850 DDS module. The phase word represents phase offset
 * in 32 discrete steps (11.25 degrees per step).
 * 
 * @param phaseDegrees Phase angle in degrees (0-359)
 * @return uint8_t 5-bit phase word (0-31)
 * 
 * @note The formula used is: phase_word = (phase_degrees * 32) / 360
 * @note Input values >= 360 will overflow and give incorrect results
 */
uint8_t calculatePhaseWord(uint32_t phaseDegrees) {
  return (uint8_t)((phaseDegrees * (1ULL << 5)) / 360);
}

// Function to calculate the full waveform word
/**
 * @brief Combines frequency and phase information into a single 64-bit word for AD9850
 * 
 * @param frequency The desired output frequency in Hz
 * @param phase The desired phase offset in degrees (0-360)
 * @return uint64_t A 64-bit word containing phase in upper 32 bits and frequency in lower 32 bits
 * 
 * This function combines the calculated phase word (upper 32 bits) and frequency word (lower 32 bits)
 * into a single 64-bit control word for the AD9850 DDS module using bitwise operations.
 */
uint64_t calculateWaveform(uint64_t frequency, uint32_t phase) {
  return ((uint64_t)calculatePhaseWord(phase) << 32) | calculateFrequencyWord(frequency) ;
}

// Function to calculate the Costas array
/**
 * @brief Calculates frequency values for an array based on carrier frequency and Costas parameters
 * 
 * @param CarrierFreq Base carrier frequency in Hz
 * @param CostasFreqStep Frequency step between adjacent elements
 * @param CostasArraySize Size of the array to be calculated
 * @param Array Input array containing multiplier values
 * @param Offset Additional frequency offset value in Hz
 * 
 * @details This function calculates frequencies for each element using the formula:
 * frequency = (Array[i] * CostasFreqStep) + CarrierFreq + Offset
 * Results are stored in the global costasArray
 * 
 * @note Assumes the existence of a global costasArray of sufficient size
 */
void calculateArray(uint64_t CarrierFreq, uint32_t CostasFreqStep, uint8_t CostasArraySize, uint8_t Array[], uint32_t Offset) {
  for (int i = 0; i < CostasArraySize; i++) {
    costasArray[i] = Array[i] * CostasFreqStep + CarrierFreq + Offset;  
  }
  return;
}
/**
 * @brief Calculate the data to be transmitted.
 * 
 * This function takes a string and converts it into a byte array.
 * The byte array is used for PSK transmission.
 * 
 * @param stuff The input string to be converted.
 * @param data The output byte array to store the converted data.
 */
void calculateData(String stuff, uint8_t *data) {
  for (int i = 0; i < (stuff.length() > DATA_SIZE)?DATA_SIZE:stuff.length(); i++) {
    data[i] = stuff.charAt(i);
  }
  return;
}

/**
 * @brief Transmits data using Phase Shift Keying (PSK) modulation
 * 
 * This function implements PSK transmission using the AD9850 DDS module:
 * 1. Sends an 8-bit preamble
 * 2. Transmits data bytes in LSB-first order
 * 3. Each bit modulates the carrier phase by 0° or 180°
 *
 * The function synchronizes with external clock signals (FQ_UD_PIN and PSK_CLK_PIN)
 * before loading each new phase value.
 *
 * @param data Array containing the bytes to transmit
 * @param data_size Number of bytes in the data array
 * 
 * @note The carrier frequency is set by CARRIER_FREQ constant
 * @note Initial phase is set by INITIAL_PHASE constant
 * @note Transmission is framed by PSK_TX_REQ_PIN signal
 */
// Static variables to maintain state between calls

void txPSK(uint8_t data[], uint8_t data_size) {
  static uint64_t waveformCache[2] = {0};

  switch(pskState) {
    case PSK_IDLE:
      if (digitalRead(PSK_TRIG_PIN) && !lockPSK) {
        digitalWrite(PSK_TX_REQ_PIN, HIGH);
        byteIndexPSK = 0;
        bitIndexPSK = 0;
        preambleCount = 0;
        pskState = PSK_START_TX;
        waveformCache[0] = calculateWaveform(CARRIER_FREQ, calculatePhaseWord(0));
        waveformCache[1] = calculateWaveform(CARRIER_FREQ, calculatePhaseWord(180));
      }
      break;
    case PSK_START_TX:
      if (digitalRead(PSK_CLK_PIN)) {
        if (preambleCount < 8 && !wordLoaded) {
          loadWord = waveformCache[0];
          loadAD9850(loadWord);
        } else if(preambleCount < 8 && wordLoaded) {
          if (digitalRead(FQ_UD_PIN)) {
            wordLoaded = false;
            preambleCount++;
          }
        } else {
          preambleCount = 0;
          pskState = PSK_DATA_TX;
        }
      }
      break;
    case PSK_DATA_TX:
      if (digitalRead(PSK_CLK_PIN)) {
        if (byteIndexPSK < data_size) {
          if (bitIndexPSK < 8 && !wordLoaded) {
            loadWord = waveformCache[(data[byteIndexPSK] >> bitIndexPSK) & 0x01];
            loadAD9850(loadWord);
          } else if (bitIndexPSK < 8 && wordLoaded) {
            if (digitalRead(FQ_UD_PIN)) {
              wordLoaded = false;
              bitIndexPSK++;
            }
          } else {
            bitIndexPSK = 0;
            byteIndexPSK++;
          }
        } else {
          byteIndexPSK = 0;
          pskState = PSK_END_TX;
        }
      }
      break;
    case PSK_END_TX:
      if (digitalRead(PSK_CLK_PIN)) {
        loadWord = calculateWaveform(0, 0);
        loadAD9850(loadWord);
        if (digitalRead(FQ_UD_PIN)) {
          pskState = PSK_IDLE;
        }
      }
      break;
    default:
      pskState = PSK_IDLE;
      break;
  }
}

// Function to transmit the Costas array
/**
 * @brief Transmits a Costas array sequence using AD9850 DDS module
 * 
 * This function handles the transmission of frequency hopping sequence defined by Costas array.
 * It synchronizes with external clock signals and manages the transmission protocol including:
 * - Setting transmission request flag
 * - Waiting for clock triggers
 * - Loading frequency words to AD9850
 * - Clearing frequency after transmission
 * 
 * @param costasArray[] Array containing frequency values for the Costas sequence
 * @param costasArraySize Size of the Costas array to be transmitted
 * 
 * @note Function uses following pins:
 *       - COSTAS_TX_REQ_PIN for transmission request
 *       - COSTAS_CLK_PIN for transmission timing
 *       - FQ_UD_PIN for frequency update timing
 * 
 * @return void
 */

void txCostasArray(uint32_t costasArray[], uint8_t costasArraySize) {
  switch(costasState) {
    case COSTAS_IDLE:
      if (digitalRead(COSTAS_TRIG_PIN) && !lockCostas) {
        digitalWrite(COSTAS_TX_REQ_PIN, HIGH);
        arrayIndexCostas = 0;
        costasState = COSTAS_TRANSMITTING;
      }
      break;

    case COSTAS_TRANSMITTING:
      if (arrayIndexCostas < costasArraySize) {
        if (digitalRead(COSTAS_CLK_PIN) && !wordLoaded) {
          loadWord = calculateWaveform(costasArray[arrayIndexCostas], calculatePhaseWord(INITIAL_PHASE));
          loadAD9850(loadWord);

        }
        else if (wordLoaded && digitalRead(FQ_UD_PIN)) {
          wordLoaded = false;
          arrayIndexCostas++;
        }
      }
      else {
        costasState = COSTAS_FINAL_WORD;
      }
      break;

    case COSTAS_FINAL_WORD:
      if (digitalRead(COSTAS_CLK_PIN) && !wordLoaded) {
        loadWord = calculateWaveform(0, 0);
        loadAD9850(loadWord);
      }
      else if (wordLoaded && digitalRead(FQ_UD_PIN)) {
        costasState = COSTAS_END_TX;
      }
      break;

    case COSTAS_END_TX:
      digitalWrite(COSTAS_TX_REQ_PIN, LOW);
      wordLoaded = false;
      costasState = COSTAS_IDLE;
      lockCostas = true;
      break;

    default:
      costasState = COSTAS_IDLE;
      break;
  }
}

/**
 * @brief Initializes the AD9850 DDS module and sets up GPIO pins
 * 
 * This function performs the following initialization steps:
 * - Configures GPIO pins for AD9850 control
 * - Sets up PSK and Costas loop control pins
 * - Initializes serial communication if DEBUG is defined
 * - Loads initial empty word into AD9850
 * - Enables and disables Costas clock to ensure proper initialization
 * - Calculates frequency array for Costas loop
 * 
 * Pin Configuration:
 * - FQ_UD_PIN: Input - Frequency Update pin
 * - PSK_TX_REQ_PIN: Output - PSK transmission request
 * - PSK_TRIG_PIN: Input - PSK trigger
 * - PSK_CLK_PIN: Input - PSK clock
 * - DATA_PIN: Output - Data line for AD9850
 * - CLOCK_PIN: Output - Clock line for AD9850
 * - COSTAS_TX_REQ_PIN: Output - Costas loop transmission request
 * - COSTAS_CLK_PIN: Input - Costas loop clock
 * 
 * @note This function blocks until the last word is loaded into AD9850
 */
void setup() {

  // Initialize the AD9850
  pinMode(FQ_UD_PIN, INPUT);
  pinMode(PSK_TX_REQ_PIN, OUTPUT);
  pinMode(PSK_TRIG_PIN, INPUT);
  pinMode(PSK_CLK_PIN, INPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(COSTAS_TX_REQ_PIN, OUTPUT);
  pinMode(COSTAS_CLK_PIN, INPUT);
  loadAD9850((0ULL << 40));     // Load an empty word
  loadWord = 0;                 // Initialize the load word variable
  digitalWrite(COSTAS_TX_REQ_PIN, HIGH); // Enable costas clock
  while(!digitalRead(FQ_UD_PIN)){};        // Wait for the last word to be loaded
  digitalWrite(COSTAS_TX_REQ_PIN, LOW);  // Disable costas clock
  calculateArray(CARRIER_FREQ, COSTAS_ARR_FREQ_STEP, COSTAS_ARR_SIZE, arrayinit, CARRIER_OFFSET);
  #ifdef DEBUG
    Serial.begin(9600);
    Serial.println("AD9850 Initialized");
  #endif
}

/**
 * @brief Main loop function that handles the transmission of Costas and PSK signals.
 * 
 * This function continuously checks for trigger signals to initiate the transmission
 * of Costas and PSK data. It performs the following tasks:
 * 
 * - Checks if the Costas trigger pin is high. If triggered:
 *   - Logs the trigger event (if DEBUG is defined).
 *   - Transmits the Costas array.
 *   - Logs the transmission status and array details (if DEBUG is defined).
 *   - Waits for the Costas unlock pin to go high.
 *   - Logs the unlock event (if DEBUG is defined).
 * 
 * - Checks if the PSK trigger pin is high. If triggered:
 *   - Logs the trigger event (if DEBUG is defined).
 *   - Calculates the data to be transmitted.
 *   - Transmits the PSK data.
 *   - Logs the transmission status and data details (if DEBUG is defined).
 *   - Waits for the PSK unlock pin to go high.
 *   - Logs the unlock event (if DEBUG is defined).
 * 
 * The function relies on the following external functions and variables:
 * - digitalRead(pin): Reads the state of the specified pin.
 * - txCostasArray(array, size): Transmits the Costas array.
 * - txPSK(data, size): Transmits the PSK data.
 * - calculateData(message, data): Calculates the data to be transmitted.
 * - COSTAS_TRIG_PIN: Pin number for Costas trigger.
 * - COSTAS_UNLOCK_PIN: Pin number for Costas unlock.
 * - PSK_TRIG_PIN: Pin number for PSK trigger.
 * - PSK_UNLOCK_PIN: Pin number for PSK unlock.
 * - costasArray: Array containing Costas data.
 * - COSTAS_ARR_SIZE: Size of the Costas array.
 * - data: Array containing PSK data.
 * - DATA_SIZE: Size of the PSK data array.
 * - BEACON_ID_MSG: Message ID for the beacon.
 * - DEBUG: Macro to enable debug logging.
 */

void loop() {
  idleLoop();
  switch(loopState) {

    case LOOP_IDLE:
      if(digitalRead(COSTAS_TRIG_PIN)) {
        #ifdef DEBUG
          Serial.println("Costas Triggered");
        #endif
        loopState = COSTAS_ACTIVE;
      }
      else if(digitalRead(PSK_TRIG_PIN)) {
        #ifdef DEBUG
          Serial.println("PSK Triggered");
        #endif
        calculateData(BEACON_ID_MSG, data);
        loopState = PSK_ACTIVE;
      }
      break;

    case COSTAS_ACTIVE:
      txCostasArray(costasArray, COSTAS_ARR_SIZE);
      if(digitalRead(COSTAS_UNLOCK_PIN)) {
        #ifdef DEBUG
          Serial.println("Costas Transmitted");
          Serial.println("Transmission unlocked!");
        #endif
        lockCostas = false;
        loopState = LOOP_IDLE;
      }
      break;

    case PSK_ACTIVE:
      txPSK(data, DATA_SIZE);
      if(digitalRead(PSK_UNLOCK_PIN)) {
        #ifdef DEBUG
          Serial.println("PSK Transmitted");
          Serial.println("Transmission unlocked!");
        #endif
        lockPSK = false;
        loopState = LOOP_IDLE;
      }
      break;

    default:
      loopState = LOOP_IDLE;
      break;
  }
}

/**
 * @brief Idle loop, for improved readability of code not involved in active transmission
 */
void idleLoop() {
  // Add any code here that should run continuously. TODO: Add GPSDO timestamping.
}