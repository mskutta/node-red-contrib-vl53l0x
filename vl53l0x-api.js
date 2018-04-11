// Most of the functionality of this library is based on the VL53L0X API
// provided by ST (STSW-IMG005), and some of the explanatory comments are quoted
// or paraphrased from the API source code, API user manual (UM2039), and the
// VL53L0X datasheet.

// https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.cpp
// https://github.com/popunder/VL53L0X
// https://github.com/cassou/VL53L0X_rasp
// C:\Users\mskutta\Downloads\2017_VL53L0X_lidar_17_0_0_%2812F1840%29.zip

'use strict'
var i2c = require('i2c-bus')
var util = require('util')

module.exports = function (busNumber = 1, address = 0x29) {
  var self = this
  var i2cBus = i2c.openSync(busNumber)

  // Constants
  // register addresses from API vl53l0x_device.h (ordered as listed there)

  const SYSRANGE_START = 0x00

  // const SYSTEM_THRESH_HIGH = 0x0C
  // const SYSTEM_THRESH_LOW = 0x0E

  const SYSTEM_SEQUENCE_CONFIG = 0x01
  // const SYSTEM_RANGE_CONFIG = 0x09
  const SYSTEM_INTERMEASUREMENT_PERIOD = 0x04

  const SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A

  const GPIO_HV_MUX_ACTIVE_HIGH = 0x84

  const SYSTEM_INTERRUPT_CLEAR = 0x0B

  const RESULT_INTERRUPT_STATUS = 0x13
  const RESULT_RANGE_STATUS = 0x14

  // const RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = 0xBC
  // const RESULT_CORE_RANGING_TOTAL_EVENTS_RTN = 0xC0
  // const RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF = 0xD0
  // const RESULT_CORE_RANGING_TOTAL_EVENTS_REF = 0xD4
  // const RESULT_PEAK_SIGNAL_RATE_REF = 0xB6

  // const ALGO_PART_TO_PART_RANGE_OFFSET_MM = 0x28

  const I2C_SLAVE_DEVICE_ADDRESS = 0x8A

  const MSRC_CONFIG_CONTROL = 0x60

  // const PRE_RANGE_CONFIG_MIN_SNR = 0x27
  const PRE_RANGE_CONFIG_VALID_PHASE_LOW = 0x56
  const PRE_RANGE_CONFIG_VALID_PHASE_HIGH = 0x57
  // const PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT = 0x64

  // const FINAL_RANGE_CONFIG_MIN_SNR = 0x67
  const FINAL_RANGE_CONFIG_VALID_PHASE_LOW = 0x47
  const FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = 0x48
  const FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44

  // const PRE_RANGE_CONFIG_SIGMA_THRESH_HI = 0x61
  // const PRE_RANGE_CONFIG_SIGMA_THRESH_LO = 0x62

  const PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50
  const PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51
  // const PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x52

  // const SYSTEM_HISTOGRAM_BIN = 0x81
  // const HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT = 0x33
  // const HISTOGRAM_CONFIG_READOUT_CTRL = 0x55

  const FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70
  const FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71
  // const FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72
  // const CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = 0x20

  const MSRC_CONFIG_TIMEOUT_MACROP = 0x46

  // const SOFT_RESET_GO2_SOFT_RESET_N = 0xBF
  // const IDENTIFICATION_MODEL_ID = 0xC0
  // const IDENTIFICATION_REVISION_ID = 0xC2

  const OSC_CALIBRATE_VAL = 0xF8

  const GLOBAL_CONFIG_VCSEL_WIDTH = 0x32
  const GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0
  // const GLOBAL_CONFIG_SPAD_ENABLES_REF_1 = 0xB1
  // const GLOBAL_CONFIG_SPAD_ENABLES_REF_2 = 0xB2
  // const GLOBAL_CONFIG_SPAD_ENABLES_REF_3 = 0xB3
  // const GLOBAL_CONFIG_SPAD_ENABLES_REF_4 = 0xB4
  // const GLOBAL_CONFIG_SPAD_ENABLES_REF_5 = 0xB5

  const GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6
  const DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E
  const DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F
  // const POWER_MANAGEMENT_GO1_POWER_FORCE = 0x80

  const VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89

  const ALGO_PHASECAL_LIM = 0x30
  const ALGO_PHASECAL_CONFIG_TIMEOUT = 0x30

  var vcselPeriodType = {
    VcselPeriodPreRange: 0,
    VcselPeriodFinalRange: 1
  }

  // Defines /////////////////////////////////////////////////////////////////////

  var stopVariable // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
  var measurementTimingBudgetUs
  var ioTimeout = 0
  var didTimeout = false

  this.setTimeout = function (timeout) {
    ioTimeout = timeout
  }

  // Record the current time to check an upcoming timeout against
  var timeoutStartMs
  function startTimeout () {
    timeoutStartMs = new Date()
  }

  // Check if timeout is enabled (set to nonzero value) and has expired
  function checkTimeoutExpired () {
    return ioTimeout > 0 && (new Date() - timeoutStartMs) > ioTimeout
  }

  // Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
  // from register value
  // based on VL53L0X_decode_vcsel_period()
  function decodeVcselPeriod (regVal) {
    return ((regVal) + 1) << 1
  }

  // Encode VCSEL pulse period register value from period in PCLKs
  // based on VL53L0X_encode_vcsel_period()
  function encodeVcselPeriod (periodPclks) {
    return ((periodPclks) >> 1) - 1
  }

  // Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
  // based on VL53L0X_calc_macro_period_ps()
  // PLL_period_ps = 1655; macro_period_vclks = 2304
  function calcMacroPeriod (vcselPeriodPclks) {
    return ((2304 * (vcselPeriodPclks) * 1655) + 500) / 1000
  }

  // Public Methods //////////////////////////////////////////////////////////////

  this.setAddress = function (newAddr) {
    writeReg(I2C_SLAVE_DEVICE_ADDRESS, newAddr & 0x7F)
    address = newAddr
  }

  this.init = function (io2v8 = true) {
    // VL53L0X_DataInit() begin

    // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
    if (io2v8) {
      writeReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
        readReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) || 0x01) // set bit 0
    }

    // "Set I2C standard mode"
    writeReg(0x88, 0x00)

    writeReg(0x80, 0x01)
    writeReg(0xFF, 0x01)
    writeReg(0x00, 0x00)
    stopVariable = readReg(0x91)
    writeReg(0x00, 0x01)
    writeReg(0xFF, 0x00)
    writeReg(0x80, 0x00)

    // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
    writeReg(MSRC_CONFIG_CONTROL, readReg(MSRC_CONFIG_CONTROL) || 0x12)

    // set final range signal rate limit to 0.25 MCPS (million counts per second)
    this.setSignalRateLimit(0.25)

    writeReg(SYSTEM_SEQUENCE_CONFIG, 0xFF)

    // VL53L0X_DataInit() end

    // VL53L0X_StaticInit() begin

    var spad = getSpadInfo()
    if (!spad) { return false }
    var spadCount = spad.count
    var spadTypeIsAperture = spad.type_is_aperture

    // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
    // the API, but the same data seems to be more easily readable from
    // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
    var refSpadMap = new Uint8Array(6)
    readMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, refSpadMap, 6)

    // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

    writeReg(0xFF, 0x01)
    writeReg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00)
    writeReg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C)
    writeReg(0xFF, 0x00)
    writeReg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4)

    var firstSpadToEnable = spadTypeIsAperture ? 12 : 0 // 12 is the first aperture spad
    var spadsEnabled = 0

    for (var i = 0; i < 48; i++) {
      if (i < firstSpadToEnable || spadsEnabled === spadCount) {
        // This bit is lower than the first one that should be enabled, or
        // (reference_spad_count) bits have already been enabled, so zero this bit
        refSpadMap[i / 8] &= ~(1 << (i % 8))
      } else if ((refSpadMap[i / 8] >> (i % 8)) & 0x1) {
        spadsEnabled++
      }
    }

    writeMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, refSpadMap, 6)

    // -- VL53L0X_set_reference_spads() end

    // -- VL53L0X_load_tuning_settings() begin
    // DefaultTuningSettings from vl53l0x_tuning.h

    writeReg(0xFF, 0x01)
    writeReg(0x00, 0x00)

    writeReg(0xFF, 0x00)
    writeReg(0x09, 0x00)
    writeReg(0x10, 0x00)
    writeReg(0x11, 0x00)

    writeReg(0x24, 0x01)
    writeReg(0x25, 0xFF)
    writeReg(0x75, 0x00)

    writeReg(0xFF, 0x01)
    writeReg(0x4E, 0x2C)
    writeReg(0x48, 0x00)
    writeReg(0x30, 0x20)

    writeReg(0xFF, 0x00)
    writeReg(0x30, 0x09)
    writeReg(0x54, 0x00)
    writeReg(0x31, 0x04)
    writeReg(0x32, 0x03)
    writeReg(0x40, 0x83)
    writeReg(0x46, 0x25)
    writeReg(0x60, 0x00)
    writeReg(0x27, 0x00)
    writeReg(0x50, 0x06)
    writeReg(0x51, 0x00)
    writeReg(0x52, 0x96)
    writeReg(0x56, 0x08)
    writeReg(0x57, 0x30)
    writeReg(0x61, 0x00)
    writeReg(0x62, 0x00)
    writeReg(0x64, 0x00)
    writeReg(0x65, 0x00)
    writeReg(0x66, 0xA0)

    writeReg(0xFF, 0x01)
    writeReg(0x22, 0x32)
    writeReg(0x47, 0x14)
    writeReg(0x49, 0xFF)
    writeReg(0x4A, 0x00)

    writeReg(0xFF, 0x00)
    writeReg(0x7A, 0x0A)
    writeReg(0x7B, 0x00)
    writeReg(0x78, 0x21)

    writeReg(0xFF, 0x01)
    writeReg(0x23, 0x34)
    writeReg(0x42, 0x00)
    writeReg(0x44, 0xFF)
    writeReg(0x45, 0x26)
    writeReg(0x46, 0x05)
    writeReg(0x40, 0x40)
    writeReg(0x0E, 0x06)
    writeReg(0x20, 0x1A)
    writeReg(0x43, 0x40)

    writeReg(0xFF, 0x00)
    writeReg(0x34, 0x03)
    writeReg(0x35, 0x44)

    writeReg(0xFF, 0x01)
    writeReg(0x31, 0x04)
    writeReg(0x4B, 0x09)
    writeReg(0x4C, 0x05)
    writeReg(0x4D, 0x04)

    writeReg(0xFF, 0x00)
    writeReg(0x44, 0x00)
    writeReg(0x45, 0x20)
    writeReg(0x47, 0x08)
    writeReg(0x48, 0x28)
    writeReg(0x67, 0x00)
    writeReg(0x70, 0x04)
    writeReg(0x71, 0x01)
    writeReg(0x72, 0xFE)
    writeReg(0x76, 0x00)
    writeReg(0x77, 0x00)

    writeReg(0xFF, 0x01)
    writeReg(0x0D, 0x01)

    writeReg(0xFF, 0x00)
    writeReg(0x80, 0x01)
    writeReg(0x01, 0xF8)

    writeReg(0xFF, 0x01)
    writeReg(0x8E, 0x01)
    writeReg(0x00, 0x01)
    writeReg(0xFF, 0x00)
    writeReg(0x80, 0x00)

    // -- VL53L0X_load_tuning_settings() end

    // "Set interrupt config to new sample ready"
    // -- VL53L0X_SetGpioConfig() begin

    writeReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)
    writeReg(GPIO_HV_MUX_ACTIVE_HIGH, readReg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10) // active low
    clearInterruptMask()

    // -- VL53L0X_SetGpioConfig() end

    measurementTimingBudgetUs = this.getMeasurementTimingBudget()

    // "Disable MSRC and TCC by default"
    // MSRC = Minimum Signal Rate Check
    // TCC = Target CentreCheck
    // -- VL53L0X_SetSequenceStepEnable() begin

    writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8)

    // -- VL53L0X_SetSequenceStepEnable() end

    // "Recalculate timing budget"
    this.setMeasurementTimingBudget(measurementTimingBudgetUs)

    // VL53L0X_StaticInit() end

    // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

    // -- VL53L0X_perform_vhv_calibration() begin

    writeReg(SYSTEM_SEQUENCE_CONFIG, 0x01)
    if (!performSingleRefCalibration(0x40)) { return false }

    // -- VL53L0X_perform_vhv_calibration() end

    // -- VL53L0X_perform_phase_calibration() begin

    writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02)
    if (!performSingleRefCalibration(0x00)) { return false }

    // -- VL53L0X_perform_phase_calibration() end

    // "restore the previous Sequence Config"
    writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8)

    // VL53L0X_PerformRefCalibration() end

    return true
  }

  // Set the return signal rate limit check value in units of MCPS (mega counts
  // per second). "This represents the amplitude of the signal reflected from the
  // target and detected by the device"; setting this limit presumably determines
  // the minimum measurement necessary for the sensor to report a valid reading.
  // Setting a lower limit increases the potential range of the sensor but also
  // seems to increase the likelihood of getting an inaccurate reading because of
  // unwanted reflections from objects other than the intended target.
  // Defaults to 0.25 MCPS as initialized by the ST API and this library.
  this.setSignalRateLimit = function (limitMcps) {
    if (limitMcps < 0 || limitMcps > 511.99) { return false }

    // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
    writeReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limitMcps * (1 << 7))
    return true
  }

  // Get the return signal rate limit check value in MCPS
  this.getSignalRateLimit = function () {
    return readReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7)
  }

  // Set the measurement timing budget in microseconds, which is the time allowed
  // for one measurement; the ST API and this library take care of splitting the
  // timing budget among the sub-steps in the ranging sequence. A longer timing
  // budget allows for more accurate measurements. Increasing the budget by a
  // factor of N decreases the range measurement standard deviation by a factor of
  // sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
  // based on VL53L0X_set_measurement_timing_budget_micro_seconds()
  this.setMeasurementTimingBudget = function (budgetUs) {
    var enables = {}
    var timeouts = {}

    const StartOverhead = 1910
    const EndOverhead = 960
    const MsrcOverhead = 660
    const TccOverhead = 590
    const DssOverhead = 690
    const PreRangeOverhead = 660
    const FinalRangeOverhead = 550

    const MinTimingBudget = 20000

    if (budgetUs < MinTimingBudget) { return false }

    var usedBudgetUs = StartOverhead + EndOverhead

    getSequenceStepEnables(enables)
    getSequenceStepTimeouts(enables, timeouts)

    if (enables.tcc) {
      usedBudgetUs += (timeouts.msrc_dss_tcc_us + TccOverhead)
    }

    if (enables.dss) {
      usedBudgetUs += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead)
    } else if (enables.msrc) {
      usedBudgetUs += (timeouts.msrc_dss_tcc_us + MsrcOverhead)
    }

    if (enables.pre_range) {
      usedBudgetUs += (timeouts.pre_range_us + PreRangeOverhead)
    }

    if (enables.final_range) {
      usedBudgetUs += FinalRangeOverhead

      // "Note that the final range timeout is determined by the timing
      // budget and the sum of all other timeouts within the sequence.
      // If there is no room for the final range timeout, then an error
      // will be set. Otherwise the remaining time will be applied to
      // the final range."

      if (usedBudgetUs > budgetUs) {
        // "Requested timeout too big."
        return false
      }

      var finalRangeTimeoutUs = budgetUs - usedBudgetUs

      // set_sequence_step_timeout() begin
      // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

      // "For the final range timeout, the pre-range timeout
      //  must be added. To do this both final and pre-range
      //  timeouts must be expressed in macro periods MClks
      //  because they have different vcsel periods."

      var finalRangeTimeoutMclks =
        timeoutMicrosecondsToMclks(finalRangeTimeoutUs,
          timeouts.final_range_vcsel_period_pclks)

      if (enables.pre_range) {
        finalRangeTimeoutMclks += timeouts.pre_range_mclks
      }

      writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
        encodeTimeout(finalRangeTimeoutMclks))

      // set_sequence_step_timeout() end

      measurementTimingBudgetUs = budgetUs // store for internal reuse
    }
    return true
  }

  // Get the measurement timing budget in microseconds
  // based on VL53L0X_get_measurement_timing_budget_micro_seconds()
  // in us
  this.getMeasurementTimingBudget = function () {
    var enables = {}
    var timeouts = {}

    const StartOverhead = 1910
    const EndOverhead = 960
    const MsrcOverhead = 660
    const TccOverhead = 590
    const DssOverhead = 690
    const PreRangeOverhead = 660
    const FinalRangeOverhead = 550

    // "Start and end overhead times always present"
    var budgetUs = StartOverhead + EndOverhead

    getSequenceStepEnables(enables)
    getSequenceStepTimeouts(enables, timeouts)

    if (enables.tcc) {
      budgetUs += (timeouts.msrc_dss_tcc_us + TccOverhead)
    }

    if (enables.dss) {
      budgetUs += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead)
    } else if (enables.msrc) {
      budgetUs += (timeouts.msrc_dss_tcc_us + MsrcOverhead)
    }

    if (enables.pre_range) {
      budgetUs += (timeouts.pre_range_us + PreRangeOverhead)
    }

    if (enables.final_range) {
      budgetUs += (timeouts.final_range_us + FinalRangeOverhead)
    }

    measurementTimingBudgetUs = budgetUs // store for internal reuse

    return budgetUs
  }

  // Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
  // given period type (pre-range or final range) to the given value in PCLKs.
  // Longer periods seem to increase the potential range of the sensor.
  // Valid values are (even numbers only):
  //  pre:  12 to 18 (initialized default: 14)
  //  final: 8 to 14 (initialized default: 10)
  // based on VL53L0X_set_vcsel_pulse_period()
  this.setVcselPulsePeriod = function (type, periodPclks) {
    var vcselPeriodReg = encodeVcselPeriod(periodPclks)

    var enables = {}
    var timeouts = {}

    getSequenceStepEnables(enables)
    getSequenceStepTimeouts(enables, timeouts)

    // "Apply specific settings for the requested clock period"
    // "Re-calculate and apply timeouts, in macro periods"

    // "When the VCSEL period for the pre or final range is changed,
    // the corresponding timeout must be read from the device using
    // the current VCSEL period, then the new VCSEL period can be
    // applied. The timeout then must be written back to the device
    // using the new VCSEL period.
    //
    // For the MSRC timeout, the same applies - this timeout being
    // dependant on the pre-range vcsel period."

    if (type === vcselPeriodType.VcselPeriodPreRange) {
      // "Set phase check limits"
      switch (periodPclks) {
        case 12:
          writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18)
          break

        case 14:
          writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30)
          break

        case 16:
          writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40)
          break

        case 18:
          writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50)
          break

        default:
          // invalid period
          return false
      }
      writeReg(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08)

      // apply new VCSEL period
      writeReg(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcselPeriodReg)

      // update timeouts

      // set_sequence_step_timeout() begin
      // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

      var newPreRangeTimeoutMclks =
        timeoutMicrosecondsToMclks(timeouts.pre_range_us, periodPclks)

      writeReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
        encodeTimeout(newPreRangeTimeoutMclks))

      // set_sequence_step_timeout() end

      // set_sequence_step_timeout() begin
      // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

      var newMsrcTimeoutMclks =
        timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, periodPclks)

      writeReg(MSRC_CONFIG_TIMEOUT_MACROP,
        (newMsrcTimeoutMclks > 256) ? 255 : (newMsrcTimeoutMclks - 1))

      // set_sequence_step_timeout() end
    } else if (type === vcselPeriodType.VcselPeriodFinalRange) {
      switch (periodPclks) {
        case 8:
          writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10)
          writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08)
          writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02)
          writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C)
          writeReg(0xFF, 0x01)
          writeReg(ALGO_PHASECAL_LIM, 0x30)
          writeReg(0xFF, 0x00)
          break

        case 10:
          writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28)
          writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08)
          writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03)
          writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09)
          writeReg(0xFF, 0x01)
          writeReg(ALGO_PHASECAL_LIM, 0x20)
          writeReg(0xFF, 0x00)
          break

        case 12:
          writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38)
          writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08)
          writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03)
          writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08)
          writeReg(0xFF, 0x01)
          writeReg(ALGO_PHASECAL_LIM, 0x20)
          writeReg(0xFF, 0x00)
          break

        case 14:
          writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48)
          writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08)
          writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03)
          writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07)
          writeReg(0xFF, 0x01)
          writeReg(ALGO_PHASECAL_LIM, 0x20)
          writeReg(0xFF, 0x00)
          break

        default:
          // invalid period
          return false
      }

      // apply new VCSEL period
      writeReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcselPeriodReg)

      // update timeouts

      // set_sequence_step_timeout() begin
      // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

      // "For the final range timeout, the pre-range timeout
      //  must be added. To do this both final and pre-range
      //  timeouts must be expressed in macro periods MClks
      //  because they have different vcsel periods."

      var newFinalRangeTimeoutMclks =
                timeoutMicrosecondsToMclks(timeouts.final_range_us, periodPclks)

      if (enables.pre_range) {
        newFinalRangeTimeoutMclks += timeouts.pre_range_mclks
      }

      writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
        encodeTimeout(newFinalRangeTimeoutMclks))

      // set_sequence_step_timeout end
    } else {
      // invalid type
      return false
    }

    // "Finally, the timing budget must be re-applied"
    this.setMeasurementTimingBudget(measurementTimingBudgetUs)

    // "Perform the phase calibration. This is needed after changing on vcsel period."
    // VL53L0X_perform_phase_calibration() begin

    var sequenceConfig = readReg(SYSTEM_SEQUENCE_CONFIG)
    writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02)
    performSingleRefCalibration(0x0)
    writeReg(SYSTEM_SEQUENCE_CONFIG, sequenceConfig)

    // VL53L0X_perform_phase_calibration() end

    return true
  }

  // Get the VCSEL pulse period in PCLKs for the given period type.
  // based on VL53L0X_get_vcsel_pulse_period()
  this.getVcselPulsePeriod = function (type) {
    if (type === vcselPeriodType.VcselPeriodPreRange) {
      return decodeVcselPeriod(readReg(PRE_RANGE_CONFIG_VCSEL_PERIOD))
    } else if (type === vcselPeriodType.VcselPeriodFinalRange) {
      return decodeVcselPeriod(readReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD))
    } else { return 255 }
  }

  // Start continuous ranging measurements. If period_ms (optional) is 0 or not
  // given, continuous back-to-back mode is used (the sensor takes measurements as
  // often as possible); otherwise, continuous timed mode is used, with the given
  // inter-measurement period in milliseconds determining how often the sensor
  // takes a measurement.
  // based on VL53L0X_StartMeasurement()
  this.startContinuous = function (periodMs = 0) {
    writeReg(0x80, 0x01)
    writeReg(0xFF, 0x01)
    writeReg(0x00, 0x00)
    writeReg(0x91, stopVariable)
    writeReg(0x00, 0x01)
    writeReg(0xFF, 0x00)
    writeReg(0x80, 0x00)

    if (periodMs !== 0) {
      // continuous timed mode

      // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

      var oscCalibrateVal = readReg16Bit(OSC_CALIBRATE_VAL)

      if (oscCalibrateVal !== 0) {
        periodMs *= oscCalibrateVal
      }

      writeReg32Bit(SYSTEM_INTERMEASUREMENT_PERIOD, periodMs)

      // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

      writeReg(SYSRANGE_START, 0x04) // VL53L0X_REG_SYSRANGE_MODE_TIMED
    } else {
      // continuous back-to-back mode
      writeReg(SYSRANGE_START, 0x02) // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
    }
  }

  // Stop continuous measurements
  // based on VL53L0X_StopMeasurement()
  this.stopContinuous = function () {
    writeReg(SYSRANGE_START, 0x01) // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

    writeReg(0xFF, 0x01)
    writeReg(0x00, 0x00)
    writeReg(0x91, 0x00)
    writeReg(0x00, 0x01)
    writeReg(0xFF, 0x00)
  }

  // Returns a range reading in millimeters when continuous mode is active
  // (readRangeSingleMillimeters() also calls this function after starting a
  // single-shot range measurement)
  this.readRangeContinuousMillimeters = function () {
    startTimeout()
    while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) === 0) {
      if (checkTimeoutExpired()) {
        didTimeout = true
        return 65535
      }
    }
    // assumptions: Linearity Corrective Gain is 1000 (default);
    // fractional ranging is not enabled
    var range = readReg16Bit(RESULT_RANGE_STATUS + 10)

    clearInterruptMask()

    return range
  }

  // Performs a single-shot range measurement and returns the reading in
  // millimeters
  // based on VL53L0X_PerformSingleRangingMeasurement()
  this.readRangeSingleMillimeters = function () {
    writeReg(0x80, 0x01)
    writeReg(0xFF, 0x01)
    writeReg(0x00, 0x00)
    writeReg(0x91, stopVariable)
    writeReg(0x00, 0x01)
    writeReg(0xFF, 0x00)
    writeReg(0x80, 0x00)

    writeReg(SYSRANGE_START, 0x01)

    // "Wait until start bit has been cleared"
    startTimeout()
    while (readReg(SYSRANGE_START) & 0x01) {
      if (checkTimeoutExpired()) {
        didTimeout = true
        return 65535
      }
    }

    return this.readRangeContinuousMillimeters()
  }

  // Did a timeout occur in one of the read functions since the last call to
  // timeoutOccurred()?
  this.timeoutOccurred = function () {
    var tmp = didTimeout
    didTimeout = false
    return tmp
  }

  this.close = function () {
    i2cBus.closeSync()
  }

  // Private Methods /////////////////////////////////////////////////////////////

  // Write an 8-bit register
  function writeReg (reg, value) {
    i2cBus.writeByteSync(address, reg, value)
  }

  // Write a 16-bit register
  function writeReg16Bit (reg, value) {
    i2cBus.writeWordSync(address, reg, value)
  }

  // Write a 32-bit register
  function writeReg32Bit (reg, value) {
    // TODO: confirm logic
    var src = []
    src.push((value >> 24) & 0xFF) // value highest byte
    src.push((value >> 16) & 0xFF)
    src.push((value >> 8) & 0xFF)
    src.push(value & 0xFF) // value lowest byte
    writeMulti(reg, src, 4)
  }

  // Read an 8-bit register
  function readReg (reg) {
    return i2cBus.readByteSync(address, reg)
  }

  // Read a 16-bit register
  function readReg16Bit (reg) {
    return i2cBus.readWordSync(address, reg)
  }

  // Write an arbitrary number of bytes from the given array to the sensor,
  // starting at the given register
  function writeMulti (reg, src, count) {
    i2cBus.writeI2cBlockSync(address, reg, count, src)
  }

  // Read an arbitrary number of bytes from the sensor, starting at the given
  // register, into the given array
  function readMulti (reg, dst, count) {
    i2cBus.readI2cBlockSync(address, reg, count, dst)
  }

  // Get reference SPAD (single photon avalanche diode) count and type
  // based on VL53L0X_get_info_from_device(),
  // but only gets reference SPAD count and type
  function getSpadInfo () {
    var spad = {}

    writeReg(0x80, 0x01)
    writeReg(0xFF, 0x01)
    writeReg(0x00, 0x00)

    writeReg(0xFF, 0x06)
    writeReg(0x83, readReg(0x83) || 0x04)
    writeReg(0xFF, 0x07)
    writeReg(0x81, 0x01)

    writeReg(0x80, 0x01)

    writeReg(0x94, 0x6b)
    writeReg(0x83, 0x00)
    startTimeout()
    while (readReg(0x83) === 0x00) {
      if (checkTimeoutExpired()) { return false }
    }
    writeReg(0x83, 0x01)
    var tmp = readReg(0x92)

    spad.count = tmp & 0x7f
    spad.type_is_aperture = (tmp >> 7) & 0x01

    writeReg(0x81, 0x00)
    writeReg(0xFF, 0x06)
    writeReg(0x83, readReg(0x83) & ~0x04)
    writeReg(0xFF, 0x01)
    writeReg(0x00, 0x01)

    writeReg(0xFF, 0x00)
    writeReg(0x80, 0x00)

    return spad
  }

  // Get sequence step enables
  // based on VL53L0X_GetSequenceStepEnables()
  function getSequenceStepEnables (enables) {
    var sequenceConfig = readReg(SYSTEM_SEQUENCE_CONFIG)

    enables.tcc = (sequenceConfig >> 4) & 0x1
    enables.dss = (sequenceConfig >> 3) & 0x1
    enables.msrc = (sequenceConfig >> 2) & 0x1
    enables.pre_range = (sequenceConfig >> 6) & 0x1
    enables.final_range = (sequenceConfig >> 7) & 0x1
  }

  // Get sequence step timeouts
  // based on get_sequence_step_timeout(),
  // but gets all timeouts instead of just the requested one, and also stores
  // intermediate values
  function getSequenceStepTimeouts (enables, timeouts) {
    timeouts.pre_range_vcsel_period_pclks = self.getVcselPulsePeriod(vcselPeriodType.VcselPeriodPreRange)

    timeouts.msrc_dss_tcc_mclks = readReg(MSRC_CONFIG_TIMEOUT_MACROP) + 1
    timeouts.msrc_dss_tcc_us =
      timeoutMclksToMicroseconds(timeouts.msrc_dss_tcc_mclks,
        timeouts.pre_range_vcsel_period_pclks)

    timeouts.pre_range_mclks =
      decodeTimeout(readReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI))
    timeouts.pre_range_us =
      timeoutMclksToMicroseconds(timeouts.pre_range_mclks,
        timeouts.pre_range_vcsel_period_pclks)

    timeouts.final_range_vcsel_period_pclks = self.getVcselPulsePeriod(vcselPeriodType.VcselPeriodFinalRange)

    timeouts.final_range_mclks =
      decodeTimeout(readReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI))

    if (enables.pre_range) {
      timeouts.final_range_mclks -= timeouts.pre_range_mclks
    }

    timeouts.final_range_us =
      timeoutMclksToMicroseconds(timeouts.final_range_mclks,
        timeouts.final_range_vcsel_period_pclks)
  }

  // Decode sequence step timeout in MCLKs from register value
  // based on VL53L0X_decode_timeout()
  // Note: the original function returned a uint32_t, but the return value is
  // always stored in a uint16_t.
  function decodeTimeout (regVal) {
    // format: "(LSByte * 2^MSByte) + 1"
    return ((regVal & 0x00FF) <<
                ((regVal & 0xFF00) >> 8)) + 1
  }

  // Encode sequence step timeout register value from timeout in MCLKs
  // based on VL53L0X_encode_timeout()
  // Note: the original function took a uint16_t, but the argument passed to it
  // is always a uint16_t.
  function encodeTimeout (timeoutMclks) {
    // format: "(LSByte * 2^MSByte) + 1"

    var lsByte = 0
    var msByte = 0

    if (timeoutMclks > 0) {
      lsByte = timeoutMclks - 1

      while ((lsByte & 0xFFFFFF00) > 0) {
        lsByte >>= 1
        msByte++
      }

      return (msByte << 8) | (lsByte & 0xFF)
    } else { return 0 }
  }

  // Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
  // based on VL53L0X_calc_timeout_us()
  function timeoutMclksToMicroseconds (timeoutPeriodMclks, vcselPeriodPclks) {
    var macroPeriodNs = calcMacroPeriod(vcselPeriodPclks)

    return ((timeoutPeriodMclks * macroPeriodNs) + (macroPeriodNs / 2)) / 1000
  }

  // Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
  // based on VL53L0X_calc_timeout_mclks()
  function timeoutMicrosecondsToMclks (timeoutPeriodUs, vcselPeriodPclks) {
    var macroPeriodNs = calcMacroPeriod(vcselPeriodPclks)

    return (((timeoutPeriodUs * 1000) + (macroPeriodNs / 2)) / macroPeriodNs)
  }

  // based on VL53L0X_perform_single_ref_calibration()
  function performSingleRefCalibration (vhvInitByte) {
    writeReg(SYSRANGE_START, 0x01 | vhvInitByte) // VL53L0X_REG_SYSRANGE_MODE_START_STOP

    startTimeout()
    while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) === 0) {
      if (checkTimeoutExpired()) { return false }
    }

    clearInterruptMask()

    writeReg(SYSRANGE_START, 0x00)

    return true
  }

  function clearInterruptMask () {
    /* clear bit 0 range interrupt, bit 1 error interrupt */
    var loopCount = 0
    var byte
    do {
      writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01)
      writeReg(SYSTEM_INTERRUPT_CLEAR, 0x00)
      byte = readReg(RESULT_INTERRUPT_STATUS)
      loopCount++
    } while (((byte & 0x07) !== 0x00) && (loopCount < 3))

    if (loopCount >= 3) {
      console.log('INTERRUPT_NOT_CLEARED')
    }
  }
}
