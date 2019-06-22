#include "MAX31790.h"

esp_err_t MAX31790::init()
{
    CONFIG config;
    uint8_t rv = readConfig(&config);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
        return rv;
    PWMFREQ pwmFreq;
    rv = readPWMFreq(&pwmFreq);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
        return rv;
    for (uint8_t i = 0; i < NR_CHANNEL; i++) {
        FANCONFIG fanConfig;
        rv = readFanConfig(i, &fanConfig);
        if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
            return rv;
        FANDYNAMICS fanDynamics;
        uint8_t rv = readFanDynamics(i, &fanDynamics);
        if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
            return rv;
    }
    return ESP_OK;
}

esp_err_t MAX31790::readConfig(CONFIG* config)
{
    uint8_t data;

    uint8_t rv = _i2c->readByte(_deviceAddress, static_cast<uint8_t>(Register::Config), &data);

    if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
        return rv;

    config->watchDogStatus = static_cast<Watch_Dog_Status>(data & static_cast<uint8_t>(ConfigMask::Watch_Dog_Status));
    config->watchDogPeriod = static_cast<Watch_Dog_Period>(data & static_cast<uint8_t>(ConfigMask::Watch_Dog_Period));
    config->oscillatorSelect = static_cast<Oscillator_Select>(data & static_cast<uint8_t>(ConfigMask::Oscillator_Select));
    config->busTimeout = static_cast<Bus_Timeout>(data & static_cast<uint8_t>(ConfigMask::Bus_Timeout));
    config->reset = static_cast<Reset>(data & static_cast<uint8_t>(ConfigMask::Reset));
    config->runStandby = static_cast<Run_Standby>(data & static_cast<uint8_t>(ConfigMask::Run_Standby));

    return ESP_OK;
}

esp_err_t MAX31790::writeConfig(const CONFIG& config)
{
    uint8_t data = static_cast<uint8_t>(config.watchDogStatus) | static_cast<uint8_t>(config.watchDogPeriod)
        | static_cast<uint8_t>(config.oscillatorSelect) | static_cast<uint8_t>(config.busTimeout)
        | static_cast<uint8_t>(config.reset) | static_cast<uint8_t>(config.runStandby);
    return _i2c->writeByte(_deviceAddress, static_cast<uint8_t>(Register::Config), data);
}

esp_err_t MAX31790::readPWMFreq(PWMFREQ* pwmFreq)
{
    uint8_t data;

    uint8_t rv = _i2c->readByte(_deviceAddress, static_cast<uint8_t>(Register::PWM_Frequency), &data);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
        return rv;

    pwmFreq->PWM1_3 = static_cast<PWMFreq>(data & static_cast<uint8_t>(PWM_FreqMask::PWM1_3));
    pwmFreq->PWM4_6 = static_cast<PWMFreq>(data & static_cast<uint8_t>(PWM_FreqMask::PWM4_6));

    return ESP_OK;
}

esp_err_t MAX31790::writePWMFreq(const PWMFREQ& pwmFreq)
{
    uint8_t data = static_cast<uint8_t>(pwmFreq.PWM4_6) << 4 | static_cast<uint8_t>(pwmFreq.PWM1_3);
    return _i2c->writeByte(_deviceAddress, static_cast<uint8_t>(Register::PWM_Frequency), data);
}

esp_err_t MAX31790::readFanConfig(const uint8_t& index, FANCONFIG* fanConfig)
{
    if (index > NR_CHANNEL)
        return ESP_ERR_INVALID_ARG;

    uint8_t data;

    uint8_t rv = _i2c->readByte(_deviceAddress, static_cast<uint8_t>(FanConfig[index]), &data);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
        return rv;

    fanConfig->outputMode = static_cast<Output_Mode>(data & static_cast<uint8_t>(FanConfigMask::Output_Mode));
    fanConfig->lockedRotorPolarity =
        static_cast<Locked_Rotor_Polarity>(data & static_cast<uint8_t>(FanConfigMask::Locked_Rotor_Polarity));
    fanConfig->tachInputMode = static_cast<Tach_Input_Mode>(data & static_cast<uint8_t>(FanConfigMask::Tach_Input_Mode));
    fanConfig->tachInputEnable = static_cast<Tach_Input_Enable>(data & static_cast<uint8_t>(FanConfigMask::Tach_Input_Enable));
    fanConfig->controlMonitor = static_cast<Control_Monitor>(data & static_cast<uint8_t>(FanConfigMask::Control_Monitor));
    fanConfig->spinUp = static_cast<Spin_Up>(data & static_cast<uint8_t>(FanConfigMask::Spin_Up));
    fanConfig->mode = static_cast<Mode>(data & static_cast<uint8_t>(FanConfigMask::Mode));

    return ESP_OK;
}

esp_err_t MAX31790::writeFanConfig(const uint8_t& index, const FANCONFIG& fanConfig)
{
    if (index > NR_CHANNEL)
        return ESP_ERR_INVALID_ARG;

    uint8_t data = static_cast<uint8_t>(fanConfig.outputMode) | static_cast<uint8_t>(fanConfig.lockedRotorPolarity)
        | static_cast<uint8_t>(fanConfig.tachInputMode) | static_cast<uint8_t>(fanConfig.tachInputEnable)
        | static_cast<uint8_t>(fanConfig.controlMonitor) | static_cast<uint8_t>(fanConfig.spinUp)
        | static_cast<uint8_t>(fanConfig.mode);
    return _i2c->writeByte(_deviceAddress, static_cast<uint8_t>(Register::PWM_Frequency), data);
}

esp_err_t MAX31790::readFanDynamics(const uint8_t& index, FANDYNAMICS* fanDynamics)
{
    if (index > NR_CHANNEL)
        return ESP_ERR_INVALID_ARG;

    uint8_t data;

    uint8_t rv = _i2c->readByte(_deviceAddress, static_cast<uint8_t>(FanDynamics[index]), &data);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
        return rv;

    fanDynamics->asymROC = static_cast<Asym_ROC>(data & static_cast<uint8_t>(FanDynamicsMask::Asym_ROC));
    fanDynamics->pwmROC = static_cast<PWM_ROC>(data & static_cast<uint8_t>(FanDynamicsMask::PWM_ROC));
    fanDynamics->speedRange = static_cast<Speed_Range>(data & static_cast<uint8_t>(FanDynamicsMask::Speed_Range));

    return ESP_OK;
}

esp_err_t MAX31790::writeFanDynamics(const uint8_t& index, const FANDYNAMICS& fanDynamics)
{
    if (index > NR_CHANNEL)
        return ESP_ERR_INVALID_ARG;

    uint8_t data = static_cast<uint8_t>(fanDynamics.asymROC) | static_cast<uint8_t>(fanDynamics.pwmROC)
        | static_cast<uint8_t>(fanDynamics.speedRange);
    return _i2c->writeByte(_deviceAddress, static_cast<uint8_t>(FanDynamics[index]), data);
}

esp_err_t MAX31790::readUserByte()
{
    // Not Implemented yet
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t MAX31790::writeUserByte()
{
    // Not Implemented yet
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t MAX31790::readFanFaultStatus(const uint8_t& index)
{
    // Not Implemented yet
    //*val = !!(_fault_status & (1 << channel));
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t MAX31790::readFanFaultMask(const uint8_t& index)
{
    // Not Implemented yet
    //*val = !!(_fault_status & (1 << channel));
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t MAX31790::writeFanFaultMask(const uint8_t& index)
{
    // Not Implemented yet
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t MAX31790::readFanOptions(FANOPTIONS* fanOptions)
{
    uint8_t data;
    uint8_t rv = _i2c->readByte(_deviceAddress, static_cast<uint8_t>(Register::Fan_Options), &data);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
        return rv;

    fanOptions->fanFaultQueue = static_cast<Fan_Fault_Queue>(data & static_cast<uint8_t>(FanOptionsMask::Fan_Fault_Queue));
    fanOptions->fanFailOptions = static_cast<Fan_Fail_Options>(data & static_cast<uint8_t>(FanOptionsMask::Fan_Fail_Options));
    fanOptions->seqStartDelay = static_cast<Seq_Start_Delay>(data & static_cast<uint8_t>(FanOptionsMask::Seq_Start_Delay));

    return ESP_OK;
}

esp_err_t MAX31790::writeFanOptions(const FANOPTIONS& fanOptions)
{
    uint8_t data = static_cast<uint8_t>(fanOptions.fanFaultQueue) | static_cast<uint8_t>(fanOptions.fanFailOptions)
        | static_cast<uint8_t>(fanOptions.seqStartDelay);
    return _i2c->writeByte(_deviceAddress, static_cast<uint8_t>(Register::Fan_Options), data);
}

esp_err_t MAX31790::readTachRaw(const uint8_t& index, uint16_t* data)
{

    if (index > NR_CHANNEL * 2)
        return ESP_ERR_INVALID_ARG;

    const uint8_t numBytes = 2;
    const bool reverse = false;
    uint8_t dataArray[2];
    uint16_t dataCombined;

    uint8_t rv = _i2c->readBytes(_deviceAddress, static_cast<uint8_t>(Tach[index]), numBytes, dataArray);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
        return rv;

    if (reverse) {
        dataCombined = dataArray[0];
        dataCombined |= dataArray[1] << 8;
    } else {
        dataCombined = dataArray[0] << 8;
        dataCombined |= dataArray[1];
    }

    *data = dataCombined >> TACH_SHIFT_BITS;

    return ESP_OK;
}

esp_err_t MAX31790::readTachRPM(const uint8_t& index, uint16_t* data)
{
    if (index > NR_CHANNEL * 2)
        return ESP_ERR_INVALID_ARG;

    uint8_t rv = readTachRaw(index, data);

    uint8_t adjIndex = index;

    // TODO: check if being used as input before returning value from 7-12
    if (index > NR_CHANNEL)
        adjIndex = index - NR_CHANNEL;

    FANDYNAMICS fanDynamics;
    rv = readFanDynamics(adjIndex, &fanDynamics);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
        return rv;

    uint8_t speedRangeInt = fanDynamicsSpeedRangeToInt(fanDynamics.speedRange);

    *data = (SEC_PER_MIN * speedRangeInt * CLOCK_CYCLES) / (*data * PULES_PER_REV);

    return ESP_OK;
}

esp_err_t MAX31790::readPWMDuty(const uint8_t& index, uint16_t* data)
{
    if (index > NR_CHANNEL)
        return ESP_ERR_INVALID_ARG;

    const uint8_t numBytes = 2;
    const bool reverse = false;
    uint8_t dataArray[2];
    uint16_t dataCombined;

    uint8_t rv = _i2c->readBytes(_deviceAddress, static_cast<uint8_t>(PWMDuty[index]), numBytes, dataArray);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
        return rv;

    if (reverse) {
        dataCombined = dataArray[0];
        dataCombined |= dataArray[1] << 8;
    } else {
        dataCombined = dataArray[0] << 8;
        dataCombined |= dataArray[1];
    }

    *data = dataCombined >> PWM_SHIFT_BITS;

    return ESP_OK;
}

esp_err_t MAX31790::readPWMTarget(const uint8_t& index, uint16_t* data)
{
    if (index > NR_CHANNEL)
        return ESP_ERR_INVALID_ARG;

    const uint8_t numBytes = 2;
    const bool reverse = false;
    uint8_t dataArray[2];
    uint16_t dataCombined;

    uint8_t rv = _i2c->readBytes(_deviceAddress, static_cast<uint8_t>(PWMTarget[index]), numBytes, dataArray);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
        return rv;

    if (reverse) {
        dataCombined = dataArray[0];
        dataCombined |= dataArray[1] << 8;
    } else {
        dataCombined = dataArray[0] << 8;
        dataCombined |= dataArray[1];
    }

    *data = dataCombined >> PWM_SHIFT_BITS;

    return ESP_OK;
}

esp_err_t MAX31790::writePWMTarget(const uint8_t& index, const uint16_t& data)
{
    if (index > NR_CHANNEL || data < PWM_MIN || data > PWM_MAX)
        return ESP_ERR_INVALID_ARG;

    uint16_t shiftData = data << PWM_SHIFT_BITS;
    const uint8_t numBytes = 2;
    const bool reverse = false;

    uint8_t dataArray[2];
    if (reverse) {
        dataArray[0] = shiftData & 0xff;
        dataArray[1] = shiftData >> 8;
    } else {
        dataArray[0] = shiftData >> 8;
        dataArray[1] = shiftData & 0xff;
    }
    return _i2c->writeBytes(_deviceAddress, static_cast<uint8_t>(PWMTarget[index]), numBytes, dataArray);
}

esp_err_t MAX31790::readTachTargetRaw(const uint8_t& index, uint16_t* data)
{
    if (index > NR_CHANNEL)
        return ESP_ERR_INVALID_ARG;

    const uint8_t numBytes = 2;
    const bool reverse = false;
    uint8_t dataArray[2];
    uint16_t dataCombined;

    uint8_t rv = _i2c->readBytes(_deviceAddress, static_cast<uint8_t>(TachTarget[index]), numBytes, dataArray);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
        return rv;

    if (reverse) {
        dataCombined = dataArray[0];
        dataCombined |= dataArray[1] << 8;
    } else {
        dataCombined = dataArray[0] << 8;
        dataCombined |= dataArray[1];
    }

    *data = dataCombined >> TACH_SHIFT_BITS;

    return ESP_OK;
}

esp_err_t MAX31790::readTachTargetRPM(const uint8_t& index, uint16_t* data)
{
    if (index > NR_CHANNEL)
        return ESP_ERR_INVALID_ARG;

    uint8_t rv = readTachTargetRaw(index, data);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
        return rv;

    FANDYNAMICS fanDynamics;
    rv = readFanDynamics(index, &fanDynamics);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
        return rv;

    uint16_t speedRangeInt = fanDynamicsSpeedRangeToInt(fanDynamics.speedRange);

    *data = (SEC_PER_MIN * speedRangeInt * CLOCK_CYCLES) / (*data * PULES_PER_REV);

    return ESP_OK;
}

esp_err_t MAX31790::writeTachTargetRaw(const uint8_t& index, uint16_t data, const Speed_Range& speedRange)
{
    if (index > NR_CHANNEL)
        return ESP_ERR_INVALID_ARG;

    FANDYNAMICS fanDynamics;
    uint8_t rv = readFanDynamics(index, &fanDynamics);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
        return rv;

    fanDynamics.speedRange = speedRange;

    rv = writeFanDynamics(index, fanDynamics);
    if (ESP_ERROR_CHECK_WITHOUT_ABORT(rv))
        return rv;

    data = clamp_val(data, FAN_TACH_MIN, FAN_TACH_MAX) << TACH_SHIFT_BITS;

    const uint8_t numBytes = 2;
    const bool reverse = false;

    uint8_t dataArray[2];
    if (reverse) {
        dataArray[0] = data & 0xff;
        dataArray[1] = data >> 8;
    } else {
        dataArray[0] = data >> 8;
        dataArray[1] = data & 0xff;
    }

    return _i2c->writeBytes(_deviceAddress, static_cast<uint8_t>(TachTarget[index]), numBytes, dataArray);
}

esp_err_t MAX31790::writeTachTargetRPM(const uint8_t& index, uint16_t data)
{
    if (index > NR_CHANNEL)
        return ESP_ERR_INVALID_ARG;

    Speed_Range speedRange = speedRangeFromRPM(data);

    data = clamp_val(data, FAN_RPM_MIN, FAN_RPM_MAX);
    data = ((SEC_PER_MIN / (data * PULES_PER_REV)) * static_cast<uint8_t>(speedRange) * CLOCK_CYCLES);

    return writeTachTargetRaw(index, data << TACH_SHIFT_BITS, speedRange);
}

esp_err_t MAX31790::readWindow(const uint8_t& index, uint8_t* data)
{
    if (index > NR_CHANNEL)
        return ESP_ERR_INVALID_ARG;

    return _i2c->readByte(_deviceAddress, static_cast<uint8_t>(Window[index]), data);
}

esp_err_t MAX31790::writeWindow(const uint8_t& index, const uint8_t& data)
{
    if (index > NR_CHANNEL)
        return ESP_ERR_INVALID_ARG;
    return _i2c->writeByte(_deviceAddress, static_cast<uint8_t>(Window[index]), data);
}
