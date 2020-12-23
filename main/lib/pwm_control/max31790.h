#pragma once

#ifndef MAX31970_h
#define MAX31970_h

#include "I2Cbus.hpp"
#include "esp_err.h"

namespace {
const uint8_t NR_CHANNEL = 6;
const uint16_t FAN_RPM_MIN = 0;
const uint16_t FAN_RPM_MAX = 65504;
const uint16_t FAN_TACH_MIN = 0x1;
const uint16_t FAN_TACH_MAX = 0x7FF;
const uint8_t PWM_MIN = 0;
const uint16_t PWM_MAX = 511;
const uint16_t SEC_PER_MIN = 60;
const uint16_t PULES_PER_REV = 2;
const uint16_t CLOCK_CYCLES = 8192;
const uint16_t TACH_SHIFT_BITS = 5;
const uint8_t PWM_SHIFT_BITS = 7;
}  // namespace

// Based off of datasheet @
// https://datasheets.maximintegrated.com/en/ds/MAX31790.pdf

class MAX31790 {
 public:
  enum class Register : const uint8_t {
    Config = 0x00,
    PWM_Frequency = 0x01,
    Fan1_Config = 0x02,
    Fan2_Config = 0x03,
    Fan3_Config = 0x04,
    Fan4_Config = 0x05,
    Fan5_Config = 0x06,
    Fan6_Config = 0x07,
    Fan1_Dynamics = 0x08,
    Fan2_Dynamics = 0x09,
    Fan3_Dynamics = 0x0A,
    Fan4_Dynamics = 0x0B,
    Fan5_Dynamics = 0x0C,
    Fan6_Dynamics = 0x0D,
    User_Byte1 = 0x0E,  // 2 wide
    Fan_Fault_Status1 = 0x10,
    Fan_Fault_Status2 = 0x11,
    Fan_Fault_Mask1 = 0x12,
    Fan_Fault_Mask2 = 0x13,
    Fan_Options = 0x14,
    User_Byte2 = 0x15,  // 3 wide
    Tach1 = 0x18,       // 2 wide
    Tach2 = 0x1A,       // 2 wide
    Tach3 = 0x1C,       // 2 wide
    Tach4 = 0x1E,       // 2 wide
    Tach5 = 0x20,       // 2 wide
    Tach6 = 0x22,       // 2 wide
    Tach7 = 0x24,       // 2 wide
    Tach8 = 0x26,       // 2 wide
    Tach9 = 0x28,       // 2 wide
    Tach10 = 0x2A,      // 2 wide
    Tach11 = 0x2C,      // 2 wide
    Tach12 = 0x2E,      // 2 wide
    PWM_Duty1 = 0x30,   // 2 wide
    PWM_Duty2 = 0x32,   // 2 wide
    PWM_Duty3 = 0x34,   // 2 wide
    PWM_Duty4 = 0x36,   // 2 wide
    PWM_Duty5 = 0x38,   // 2 wide
    PWM_Duty6 = 0x3A,   // 2 wide
    // 0x3C-0x3F Reserved
    PWM_Target1 = 0x40,   // 2 wide
    PWM_Target2 = 0x42,   // 2 wide
    PWM_Target3 = 0x44,   // 2 wide
    PWM_Target4 = 0x46,   // 2 wide
    PWM_Target5 = 0x48,   // 2 wide
    PWM_Target6 = 0x4A,   // 2 wide
    User_Byte3 = 0x4C,    // 4 wide
    Tach_Target1 = 0x50,  // 2 wide
    Tach_Target2 = 0x52,  // 2 wide
    Tach_Target3 = 0x54,  // 2 wide
    Tach_Target4 = 0x56,  // 2 wide
    Tach_Target5 = 0x58,  // 2 wide
    Tach_Target6 = 0x5A,  // 2 wide
    User_Byte4 = 0x5C,    // 4 wide
    Window1 = 0x60,
    Window2 = 0x61,
    Window3 = 0x62,
    Window4 = 0x63,
    Window5 = 0x64,
    Window6 = 0x65,
    User_Byte5 = 0x66  // 2 wide
  };

  const Register FanConfig[NR_CHANNEL] = {
      Register::Fan1_Config, Register::Fan2_Config, Register::Fan3_Config,
      Register::Fan4_Config, Register::Fan5_Config, Register::Fan6_Config};

  const Register FanDynamics[NR_CHANNEL] = {
      Register::Fan1_Dynamics, Register::Fan2_Dynamics,
      Register::Fan3_Dynamics, Register::Fan4_Dynamics,
      Register::Fan5_Dynamics, Register::Fan6_Dynamics};

  const Register Tach[NR_CHANNEL * 2] = {
      Register::Tach1, Register::Tach2,  Register::Tach3,  Register::Tach4,
      Register::Tach5, Register::Tach6,  Register::Tach7,  Register::Tach8,
      Register::Tach9, Register::Tach10, Register::Tach11, Register::Tach12};

  const Register PWMDuty[NR_CHANNEL] = {
      Register::PWM_Duty1, Register::PWM_Duty2, Register::PWM_Duty3,
      Register::PWM_Duty4, Register::PWM_Duty5, Register::PWM_Duty6};

  const Register PWMTarget[NR_CHANNEL] = {
      Register::PWM_Target1, Register::PWM_Target2, Register::PWM_Target3,
      Register::PWM_Target4, Register::PWM_Target5, Register::PWM_Target6};

  const Register TachTarget[NR_CHANNEL] = {
      Register::Tach_Target1, Register::Tach_Target2, Register::Tach_Target3,
      Register::Tach_Target4, Register::Tach_Target5, Register::Tach_Target6};

  const Register Window[NR_CHANNEL] = {Register::Window1, Register::Window2,
                                       Register::Window3, Register::Window4,
                                       Register::Window5, Register::Window6};

  enum class ConfigMask : const uint8_t {
    Watch_Dog_Status = 0b1 << 0,
    Watch_Dog_Period = 0b11 << 1,
    Oscillator_Select = 0b1 << 3,
    // Bit 4 Reserved
    Bus_Timeout = 0b1 << 5,
    Reset = 0b1 << 6,
    Run_Standby = 0b1 << 7
  };

  enum class Watch_Dog_Status : const uint8_t {
    I2C_Occured = 0b0 << 0,
    I2C_Exceed_Watchdog = 0b1 << 0
  };

  enum class Watch_Dog_Period : const uint8_t {
    Inactive = 0b00 << 1,
    Sec_5 = 0b01 << 1,
    Sec_10 = 0b10 << 1,
    Sec_30 = 0b11 << 1
  };

  enum class Oscillator_Select : const uint8_t {
    Internal = 0b0 << 3,
    External = 0b1 << 3
  };

  enum class Bus_Timeout : const uint8_t {
    Enabled = 0b0 << 5,
    Disabled = 0b1 << 5
  };

  enum class Reset : const uint8_t { Normal = 0b0 << 6, Reset = 0b1 << 6 };

  enum class Run_Standby : uint8_t { Run = 0b0 << 7, Standby = 0b1 << 7 };

  struct CONFIG {
    Watch_Dog_Status watchDogStatus;
    Watch_Dog_Period watchDogPeriod;
    Oscillator_Select oscillatorSelect;
    Bus_Timeout busTimeout;
    Reset reset;
    Run_Standby runStandby;
  };

  enum class PWM_FreqMask : const uint8_t {
    PWM1_3 = 0b1111 << 0,
    PWM4_6 = 0b1111 << 4
  };

  enum class PWMFreq : const uint8_t {
    Hz_25 = 0b0000,
    Hz_30 = 0b0001,
    Hz_35 = 0b0010,
    Hz_100 = 0b0011,
    Hz_125 = 0b0100,
    Hz_149_7 = 0b0101,
    kHz_1_25 = 0b0110,
    kHz_1_47 = 0b0111,
    kHz_3_57 = 0b1000,
    kHz_5 = 0b1001,
    kHz_12_5 = 0b1010,
    kHz_25 = 0b1011
  };

  struct PWMFREQ {
    PWMFreq PWM1_3;
    PWMFreq PWM4_6;
  };

  enum class FanConfigMask : const uint8_t {
    Output_Mode = 0b1 << 0,
    Locked_Rotor_Polarity = 0b1 << 1,
    Tach_Input_Mode = 0b1 << 2,
    Tach_Input_Enable = 0b1 << 3,
    Control_Monitor = 0b1 << 4,
    Spin_Up = 0b11 << 5,
    Mode = 0b1 << 7
  };

  enum class Output_Mode : const uint8_t {
    PWM_Output = 0b0 << 0,
    Tach_Input = 0b1 << 0
  };

  enum class Locked_Rotor_Polarity : const uint8_t {
    Low = 0b0 << 1,
    High = 0b1 << 1
  };

  enum class Tach_Input_Mode : const uint8_t {
    Tach_Count = 0b0 << 2,
    Locked_Rotor = 0b1 << 2
  };

  enum class Tach_Input_Enable : const uint8_t {
    Disabled = 0b0 << 3,
    Enabled = 0b1 << 3
  };

  enum class Control_Monitor : const uint8_t {
    Control = 0b0 << 4,
    Monitor_Only = 0b1 << 4
  };

  enum class Spin_Up : const uint8_t {
    None = 0b00 << 5,
    Sec_0_5 = 0b01 << 5,
    Sec_1 = 0b10 << 5,
    Sec_2 = 0b11 << 5
  };

  enum class Mode : const uint8_t { PWM = 0b0 << 7, RPM = 0b1 << 7 };

  struct FANCONFIG {
    Output_Mode outputMode;
    Locked_Rotor_Polarity lockedRotorPolarity;
    Tach_Input_Mode tachInputMode;
    Tach_Input_Enable tachInputEnable;
    Control_Monitor controlMonitor;
    Spin_Up spinUp;
    Mode mode;
  };

  enum class FanDynamicsMask : const uint8_t {
    // Bit 0 Reserved
    Asym_ROC = 0b1 << 1,
    PWM_ROC = 0b111 << 2,
    Speed_Range = 0b111 << 5,
  };

  enum class Asym_ROC : const uint8_t {
    Enabled = 0b0 << 1,  // Same rate of change whether duty cycle is increasing
                         // or decreasing.
    Disabled = 0b1 << 1  // Rate of change when duty cycle is decreasing is half
                         // the rate when increasing.
  };

  enum class PWM_ROC : const uint8_t {
    ms_0 = 0b000 << 2,  // PWM = 0ms per LSB , RPM = 0.9765ms per LSB
    ms_1_953125 = 0b001 << 2,
    ms_3_90625 = 0b010 << 2,
    ms_7_8125 = 0b011 << 2,
    ms_15_625 = 0b100 << 2,
    ms_31_25 = 0b101 << 2,
    ms_62_5 = 0b110 << 2,
    ms_125 = 0b111 << 2
  };

  enum class Speed_Range : const uint8_t {
    SR_1 = 0b000 << 5,
    SR_2 = 0b001 << 5,
    SR_4 = 0b010 << 5,
    SR_8 = 0b011 << 5,
    SR_16 = 0b100 << 5,
    SR_32_1 = 0b101 << 5,
    SR_32_2 = 0b110 << 5,
    SR_32_3 = 0b111 << 5
  };

  struct FANDYNAMICS {
    Asym_ROC asymROC;
    PWM_ROC pwmROC;
    Speed_Range speedRange;
  };

  enum class FanFaultMask : const uint16_t {
    Fan1 = 0b1 << 0,
    Fan2 = 0b1 << 1,
    Fan3 = 0b1 << 2,
    Fan4 = 0b1 << 3,
    Fan5 = 0b1 << 4,
    Fan6 = 0b1 << 5,
    // Bits 6,7 RESERVED
    Fan7 = 0b1 << 8,
    Fan8 = 0b1 << 9,
    Fan9 = 0b1 << 10,
    Fan10 = 0b1 << 11,
    Fan11 = 0b1 << 12,
    Fan12 = 0b1 << 13
    // Bits 14,15 RESERVED
  };

  enum class FanFaultStatus : const uint8_t { Normal = 0b0, Fault = 0b1 };

  enum class FanFaultMaskFail : const uint8_t { Normal = 0b0, Masked = 0b1 };

  enum class FanOptionsMask : const uint8_t {
    Fan_Fault_Queue = 0b11 << 0,
    Fan_Fail_Options = 0b11 << 2,
    // Bit 4 RESERVED
    Seq_Start_Delay = 0b111 << 5
  };

  enum class Fan_Fault_Queue : const uint8_t {
    FFQ_1 = 0b00 << 0,
    FFQ_2 = 0b01 << 0,
    FFQ_4 = 0b10 << 0,
    FFQ_6 = 0b11 << 0
  };

  enum class Fan_Fail_Options : const uint8_t {
    Duty_0 = 0b00 << 2,
    Continue = 0b01 << 2,
    Duty_100 = 0b10 << 2,
    Duty_100_Unmasked = 0b11 << 2
  };

  enum class Seq_Start_Delay : const uint8_t {
    Sec_0 = 0b000 << 5,
    ms_250 = 0b001 << 5,
    ms_500 = 0b010 << 5,
    Sec_1 = 0b011 << 5,
    Sec_2 = 0b100 << 5,
    Sec_4_1 = 0b101 << 5,
    Sec_4_2 = 0b110 << 5,
    Sec_4_3 = 0b111 << 5
  };

  struct FANOPTIONS {
    Fan_Fault_Queue fanFaultQueue;
    Fan_Fail_Options fanFailOptions;
    Seq_Start_Delay seqStartDelay;
  };

  MAX31790(I2C_t* i2c, const uint8_t& deviceAddress) {
    _i2c = i2c;
    _deviceAddress = deviceAddress;
  }

  esp_err_t init();

  esp_err_t readConfig(CONFIG* config);

  esp_err_t writeConfig(const CONFIG& config);

  esp_err_t readPWMFreq(PWMFREQ* pwmFreq);

  esp_err_t writePWMFreq(const PWMFREQ& pwmFreq);

  esp_err_t readFanConfig(const uint8_t& index, FANCONFIG* fanConfig);

  esp_err_t writeFanConfig(const uint8_t& index, const FANCONFIG& fanConfig);

  esp_err_t readFanDynamics(const uint8_t& index, FANDYNAMICS* fanDynamics);

  esp_err_t writeFanDynamics(const uint8_t& index,
                             const FANDYNAMICS& fanDynamics);

  esp_err_t readUserByte();   // Not Implemented yet
  esp_err_t writeUserByte();  // Not Implemented yet

  esp_err_t readFanFaultStatus(const uint8_t& index);  // Not Implemented yet
  esp_err_t readFanFaultMask(const uint8_t& index);    // Not Implemented yet
  esp_err_t writeFanFaultMask(const uint8_t& index);   // Not Implemented yet

  esp_err_t readFanOptions(FANOPTIONS* fanOptions);
  esp_err_t writeFanOptions(const FANOPTIONS& fanOptions);

  esp_err_t readTachRaw(const uint8_t& index, uint16_t* data);
  esp_err_t readTachRPM(const uint8_t& index, uint16_t* data);
  esp_err_t readPWMDuty(const uint8_t& index, uint16_t* data);

  esp_err_t readPWMTarget(const uint8_t& index, uint16_t* data);
  esp_err_t writePWMTarget(const uint8_t& index, const uint16_t& data);

  esp_err_t readTachTargetRaw(const uint8_t& index, uint16_t* data);
  esp_err_t writeTachTargetRaw(const uint8_t& index, uint16_t data,
                               const Speed_Range& speedRange);

  esp_err_t readTachTargetRPM(const uint8_t& index, uint16_t* data);
  esp_err_t writeTachTargetRPM(const uint8_t& index, uint16_t data);

  esp_err_t readWindow(const uint8_t& index, uint8_t* data);
  esp_err_t writeWindow(const uint8_t& index, const uint8_t& data);

 private:
  I2C_t* _i2c;
  uint8_t _deviceAddress;

  inline uint16_t fanDynamicsSpeedRangeToInt(const Speed_Range& speedRange) {
    switch (speedRange) {
      case Speed_Range::SR_1:
        return 1;
      case Speed_Range::SR_2:
        return 2;
      case Speed_Range::SR_4:
        return 4;
      case Speed_Range::SR_8:
        return 8;
      case Speed_Range::SR_16:
        return 16;
      case Speed_Range::SR_32_1:
      case Speed_Range::SR_32_2:
      case Speed_Range::SR_32_3:
        return 32;
      default:
        return 0;
    }
  }

  inline Speed_Range speedRangeFromRPM(const uint16_t& rpm) {
    if (rpm < 500)
      return Speed_Range::SR_1;
    else if (rpm < 1000)
      return Speed_Range::SR_2;
    else if (rpm < 2000)
      return Speed_Range::SR_4;
    else if (rpm < 4000)
      return Speed_Range::SR_8;
    else if (rpm < 8000)
      return Speed_Range::SR_16;
    else
      return Speed_Range::SR_32_1;
  }
};
#endif
