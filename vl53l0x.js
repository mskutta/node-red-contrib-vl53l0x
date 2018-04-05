// Most of the functionality of this library is based on the VL53L0X API
// provided by ST (STSW-IMG005), and some of the explanatory comments are quoted
// or paraphrased from the API source code, API user manual (UM2039), and the
// VL53L0X datasheet.

// https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.cpp

"use strict";
var i2c = require("i2c-bus");

module.exports = function(RED) {

    // register addresses from API vl53l0x_device.h (ordered as listed there)

    const SYSRANGE_START                              = 0x00

    const SYSTEM_THRESH_HIGH                          = 0x0C
    const SYSTEM_THRESH_LOW                           = 0x0E

    const SYSTEM_SEQUENCE_CONFIG                      = 0x01
    const SYSTEM_RANGE_CONFIG                         = 0x09
    const SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04

    const SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A

    const GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84

    const SYSTEM_INTERRUPT_CLEAR                      = 0x0B

    const RESULT_INTERRUPT_STATUS                     = 0x13
    const RESULT_RANGE_STATUS                         = 0x14

    const RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC
    const RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0
    const RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0
    const RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4
    const RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6

    const ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28

    const I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A

    const MSRC_CONFIG_CONTROL                         = 0x60

    const PRE_RANGE_CONFIG_MIN_SNR                    = 0x27
    const PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56
    const PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57
    const PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64

    const FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67
    const FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47
    const FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48
    const FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44

    const PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61
    const PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62

    const PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50
    const PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51
    const PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52

    const SYSTEM_HISTOGRAM_BIN                        = 0x81
    const HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33
    const HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55

    const FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70
    const FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71
    const FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72
    const CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20

    const MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46

    const SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF
    const IDENTIFICATION_MODEL_ID                     = 0xC0
    const IDENTIFICATION_REVISION_ID                  = 0xC2

    const OSC_CALIBRATE_VAL                           = 0xF8

    const GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32
    const GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0
    const GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1
    const GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2
    const GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3
    const GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4
    const GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5

    const GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6
    const DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E
    const DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F
    const POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80

    const VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89

    const ALGO_PHASECAL_LIM                           = 0x30
    const ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30

    var i2cBus = i2c.openSync(1);
    const address = 0x29; // Default address

    function Read(config) {
        RED.nodes.createNode(this, config);
        this.bus = parseInt(config.bus);

        var node = this;
        node.port = i2c.openSync(node.bus)

        node.on('input', function(msg) {
            var address = node.address || msg.address;
            address = parseInt(address);
            if (isNaN(address)) {
                this.status({fill:"red",shape:"ring",text:"Address ("+address+") value is missing or incorrect"});	
                return;
            } else {
                this.status({});
            }


            msg.payload = msg.payload.toLowerCase();
            node.send(msg);
        });
    }
    RED.nodes.registerType("vl53l0x",Read);

    function Init() {
        // VL53L0X_DataInit() begin

        // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
        if (io_2v8)
        {
            i2cBus.writeByteSync(address, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
                i2cBus.readByteSync(address, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) || 0x01) // set bit 0
        }

        // "Set I2C standard mode"
        i2cBus.writeByteSync(address, 0x88, 0x00)

        i2cBus.writeByteSync(address, 0x80, 0x01)
        i2cBus.writeByteSync(address, 0xFF, 0x01)
        i2cBus.writeByteSync(address, 0x00, 0x00)
        var stop_variable = i2cBus.readByteSync(address, 0x91)
        i2cBus.writeByteSync(address, 0x00, 0x01)
        i2cBus.writeByteSync(address, 0xFF, 0x00)
        i2cBus.writeByteSync(address, 0x80, 0x00)

        // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
        i2cBus.writeByteSync(address, MSRC_CONFIG_CONTROL, i2cBus.readByteSync(address, MSRC_CONFIG_CONTROL) || 0x12)

        // set final range signal rate limit to 0.25 MCPS (million counts per second)
        setSignalRateLimit(0.25)

        i2cBus.writeByteSync(address, SYSTEM_SEQUENCE_CONFIG, 0xFF)

        // VL53L0X_DataInit() end

        // VL53L0X_StaticInit() begin

        var spad = getSpadInfo()
        if (!spad) { return false }
        var spad_count = spad.count
        var spad_type_is_aperture = spad.type_is_aperture;

        // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
        // the API, but the same data seems to be more easily readable from
        // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
        var ref_spad_map = new Uint8Array(6)
        i2cBus.readI2cBlockSync(address, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6, ref_spad_map)

        // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

        i2cBus.writeByteSync(address, 0xFF, 0x01)
        i2cBus.writeByteSync(address, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00)
        i2cBus.writeByteSync(address, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C)
        i2cBus.writeByteSync(address, 0xFF, 0x00)
        i2cBus.writeByteSync(address, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4)

        var first_spad_to_enable = spad_type_is_aperture ? 12 : 0 // 12 is the first aperture spad
        var spads_enabled = 0

        for (i = 0; i < 48; i++)
        {
            if (i < first_spad_to_enable || spads_enabled == spad_count)
            {
                // This bit is lower than the first one that should be enabled, or
                // (reference_spad_count) bits have already been enabled, so zero this bit
                ref_spad_map[i / 8] &= ~(1 << (i % 8))
            }
            else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
            {
                spads_enabled++
            }
        }

        i2cBus.writeI2cBlockSync(address, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 6, ref_spad_map)

        // -- VL53L0X_set_reference_spads() end


        // -- VL53L0X_load_tuning_settings() begin
        // DefaultTuningSettings from vl53l0x_tuning.h
        
        i2cBus.writeByteSync(address, 0xFF, 0x01)
        i2cBus.writeByteSync(address, 0x00, 0x00)

        i2cBus.writeByteSync(address, 0xFF, 0x00)
        i2cBus.writeByteSync(address, 0x09, 0x00)
        i2cBus.writeByteSync(address, 0x10, 0x00)
        i2cBus.writeByteSync(address, 0x11, 0x00)

        i2cBus.writeByteSync(address, 0x24, 0x01)
        i2cBus.writeByteSync(address, 0x25, 0xFF)
        i2cBus.writeByteSync(address, 0x75, 0x00)

        i2cBus.writeByteSync(address, 0xFF, 0x01)
        i2cBus.writeByteSync(address, 0x4E, 0x2C)
        i2cBus.writeByteSync(address, 0x48, 0x00)
        i2cBus.writeByteSync(address, 0x30, 0x20)

        i2cBus.writeByteSync(address, 0xFF, 0x00)
        i2cBus.writeByteSync(address, 0x30, 0x09)
        i2cBus.writeByteSync(address, 0x54, 0x00)
        i2cBus.writeByteSync(address, 0x31, 0x04)
        i2cBus.writeByteSync(address, 0x32, 0x03)
        i2cBus.writeByteSync(address, 0x40, 0x83)
        i2cBus.writeByteSync(address, 0x46, 0x25)
        i2cBus.writeByteSync(address, 0x60, 0x00)
        i2cBus.writeByteSync(address, 0x27, 0x00)
        i2cBus.writeByteSync(address, 0x50, 0x06)
        i2cBus.writeByteSync(address, 0x51, 0x00)
        i2cBus.writeByteSync(address, 0x52, 0x96)
        i2cBus.writeByteSync(address, 0x56, 0x08)
        i2cBus.writeByteSync(address, 0x57, 0x30)
        i2cBus.writeByteSync(address, 0x61, 0x00)
        i2cBus.writeByteSync(address, 0x62, 0x00)
        i2cBus.writeByteSync(address, 0x64, 0x00)
        i2cBus.writeByteSync(address, 0x65, 0x00)
        i2cBus.writeByteSync(address, 0x66, 0xA0)

        i2cBus.writeByteSync(address, 0xFF, 0x01)
        i2cBus.writeByteSync(address, 0x22, 0x32)
        i2cBus.writeByteSync(address, 0x47, 0x14)
        i2cBus.writeByteSync(address, 0x49, 0xFF)
        i2cBus.writeByteSync(address, 0x4A, 0x00)

        i2cBus.writeByteSync(address, 0xFF, 0x00)
        i2cBus.writeByteSync(address, 0x7A, 0x0A)
        i2cBus.writeByteSync(address, 0x7B, 0x00)
        i2cBus.writeByteSync(address, 0x78, 0x21)

        i2cBus.writeByteSync(address, 0xFF, 0x01)
        i2cBus.writeByteSync(address, 0x23, 0x34)
        i2cBus.writeByteSync(address, 0x42, 0x00)
        i2cBus.writeByteSync(address, 0x44, 0xFF)
        i2cBus.writeByteSync(address, 0x45, 0x26)
        i2cBus.writeByteSync(address, 0x46, 0x05)
        i2cBus.writeByteSync(address, 0x40, 0x40)
        i2cBus.writeByteSync(address, 0x0E, 0x06)
        i2cBus.writeByteSync(address, 0x20, 0x1A)
        i2cBus.writeByteSync(address, 0x43, 0x40)

        i2cBus.writeByteSync(address, 0xFF, 0x00)
        i2cBus.writeByteSync(address, 0x34, 0x03)
        i2cBus.writeByteSync(address, 0x35, 0x44)

        i2cBus.writeByteSync(address, 0xFF, 0x01)
        i2cBus.writeByteSync(address, 0x31, 0x04)
        i2cBus.writeByteSync(address, 0x4B, 0x09)
        i2cBus.writeByteSync(address, 0x4C, 0x05)
        i2cBus.writeByteSync(address, 0x4D, 0x04)

        i2cBus.writeByteSync(address, 0xFF, 0x00)
        i2cBus.writeByteSync(address, 0x44, 0x00)
        i2cBus.writeByteSync(address, 0x45, 0x20)
        i2cBus.writeByteSync(address, 0x47, 0x08)
        i2cBus.writeByteSync(address, 0x48, 0x28)
        i2cBus.writeByteSync(address, 0x67, 0x00)
        i2cBus.writeByteSync(address, 0x70, 0x04)
        i2cBus.writeByteSync(address, 0x71, 0x01)
        i2cBus.writeByteSync(address, 0x72, 0xFE)
        i2cBus.writeByteSync(address, 0x76, 0x00)
        i2cBus.writeByteSync(address, 0x77, 0x00)

        i2cBus.writeByteSync(address, 0xFF, 0x01)
        i2cBus.writeByteSync(address, 0x0D, 0x01)

        i2cBus.writeByteSync(address, 0xFF, 0x00)
        i2cBus.writeByteSync(address, 0x80, 0x01)
        i2cBus.writeByteSync(address, 0x01, 0xF8)

        i2cBus.writeByteSync(address, 0xFF, 0x01)
        i2cBus.writeByteSync(address, 0x8E, 0x01)
        i2cBus.writeByteSync(address, 0x00, 0x01)
        i2cBus.writeByteSync(address, 0xFF, 0x00)
        i2cBus.writeByteSync(address, 0x80, 0x00)

        // -- VL53L0X_load_tuning_settings() end

        // "Set interrupt config to new sample ready"
        // -- VL53L0X_SetGpioConfig() begin

        i2cBus.writeByteSync(address, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)
        i2cBus.writeByteSync(address, GPIO_HV_MUX_ACTIVE_HIGH, readReg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10) // active low
        i2cBus.writeByteSync(address, SYSTEM_INTERRUPT_CLEAR, 0x01)

        // -- VL53L0X_SetGpioConfig() end

        measurement_timing_budget_us = getMeasurementTimingBudget();

        // "Disable MSRC and TCC by default"
        // MSRC = Minimum Signal Rate Check
        // TCC = Target CentreCheck
        // -- VL53L0X_SetSequenceStepEnable() begin

        writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

        // -- VL53L0X_SetSequenceStepEnable() end

        // "Recalculate timing budget"
        setMeasurementTimingBudget(measurement_timing_budget_us);

        // VL53L0X_StaticInit() end

        // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

        // -- VL53L0X_perform_vhv_calibration() begin

        writeReg(SYSTEM_SEQUENCE_CONFIG, 0x01);
        if (!performSingleRefCalibration(0x40)) { return false; }

        // -- VL53L0X_perform_vhv_calibration() end

        // -- VL53L0X_perform_phase_calibration() begin

        writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
        if (!performSingleRefCalibration(0x00)) { return false; }

        // -- VL53L0X_perform_phase_calibration() end

        // "restore the previous Sequence Config"
        writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

        // VL53L0X_PerformRefCalibration() end

        return true;


        
    }

    // Set the return signal rate limit check value in units of MCPS (mega counts
    // per second). "This represents the amplitude of the signal reflected from the
    // target and detected by the device"; setting this limit presumably determines
    // the minimum measurement necessary for the sensor to report a valid reading.
    // Setting a lower limit increases the potential range of the sensor but also
    // seems to increase the likelihood of getting an inaccurate reading because of
    // unwanted reflections from objects other than the intended target.
    // Defaults to 0.25 MCPS as initialized by the ST API and this library.
    function setSignalRateLimit(limit_Mcps)
    {
      if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }
    
      // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
      i2cBus.writeWordSync(address, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
      return true;
    }

    // Get reference SPAD (single photon avalanche diode) count and type
    // based on VL53L0X_get_info_from_device(),
    // but only gets reference SPAD count and type
    function getSpadInfo()
    {
        var spad = {};

        i2cBus.writeByteSync(address, 0x80, 0x01);
        i2cBus.writeByteSync(address, 0xFF, 0x01);
        i2cBus.writeByteSync(address, 0x00, 0x00);

        i2cBus.writeByteSync(address, 0xFF, 0x06);
        i2cBus.writeByteSync(address, 0x83, i2cBus.readByteSync(address, 0x83) || 0x04);
        i2cBus.writeByteSync(address, 0xFF, 0x07);
        i2cBus.writeByteSync(address, 0x81, 0x01);

        i2cBus.writeByteSync(address, 0x80, 0x01);

        i2cBus.writeByteSync(address, 0x94, 0x6b);
        i2cBus.writeByteSync(address, 0x83, 0x00);
        startTimeout();
        while (i2cBus.readByteSync(address, 0x83) == 0x00)
        {
            if (checkTimeoutExpired()) { return false; }
        }
        i2cBus.writeByteSync(address, 0x83, 0x01);
        var tmp = i2cBus.readByteSync(address, 0x92);

        spad.count = tmp & 0x7f;
        spad.type_is_aperture = (tmp >> 7) & 0x01;

        i2cBus.writeByteSync(address, 0x81, 0x00);
        i2cBus.writeByteSync(address, 0xFF, 0x06);
        i2cBus.writeByteSync(address, 0x83, i2cBus.readByteSync(address, 0x83) & ~0x04);
        i2cBus.writeByteSync(address, 0xFF, 0x01);
        i2cBus.writeByteSync(address, 0x00, 0x01);

        i2cBus.writeByteSync(address, 0xFF, 0x00);
        i2cBus.writeByteSync(address, 0x80, 0x00);

        return spad;
    }
}