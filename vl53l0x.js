// ==============================================================================
/**
    @file       vl53l0x.js
    @brief      Node-RED support for the VL53L0X.
    @author     Mike Skutta
    @version    0.0.1

    @copyright 2018 Mike Skutta
    @attention
    <b><center>&copy; Copyright (c) 2018 Mike Skutta</center></b>
    Licensed under Mozilla Public License, v. 2.0 (MPL-2.0), a flexible license
    that just requires that source changes are returned to the community.
    If a copy of the MPL-2.0 was not distributed with this file,
    you can obtain one at http://mozilla.org/MPL/2.0/.
    @see  LICENSE
    @see  NOTICE
*/
// ==============================================================================

// =======================================================================================
//
//  Node-RED support for the VL53L0X. Code and notes ported from VL53L0X_lidar.c by
//  Vaux Electronics, Inc., dba SurlEE http://www.surlee.com/
//  Original source: https://www.chiefdelphi.com/forums/showthread.php?p=1708192#post1708192
//
//  VL53L0X lidar chip is running at 2.8V and draws about 19 mA when ranging (40mA peak).
//  The interval of readings is configurable via the node in Node-RED.
//  Only doing minimal init and running in "default mode" which is good to about 1200 mm.
//
//  We return the range in mm. If the lidar gets a good reading, the range
//  should be considered valid   Some sort of moving average should probably be
//  applied to smooth out the data stream a bit.  The lidar returns 20mm or 8190mm for
//  bogus readings, so we trap them out, and return 1200 (max supported range).
//
//  Not using any lidar code from ST (which would be copyright 2016 STMicroelectronics, btw),
//  but register addresses and sequence of steps for init and operation (details that
//  should really have been in the datasheet) extracted from ST's incredibly-convoluted
//  and bloated api code.
//
//  - We presume default mode, factory NVM trims are good, and no window over lidar.
//  - We are not doing any calibration of the lidar module directly, just a simple
//      calibration tweak routine after reading range data based on empirical testing.
//  - Just minimal init to use chip in out-of-box default state.
//  - Much of the init stuff in the bloated api is not needed for default mode.
//  - Just using continuous conversion mode.
//  - Bad ranging returns 20mm (quite often).  Range supposedly may be qualified by reading
//      the status byte (s/b 0), but we just trap out and qualify the bogus range numbers.
//  - Not doing any config for long ranging (2000mm) -- default mode spec'd to 1200mm.
//
// ===================================================================================

module.exports = function (RED) {
  'use strict'

  var i2c = require('i2c-bus')
  var util = require('util')

  function Vl53l0xNode (config) {
    RED.nodes.createNode(this, config)
    this.bus = parseInt(config.bus) // I2C Bus Number
    this.address = parseInt(config.address) // VL53L0X Address
    this.interval = parseInt(config.interval) // Polling Interval

    var node = this
    console.log(util.inspect(node))

    var i2cBus = i2c.openSync(node.bus)

    // Returns a range reading in millimeters.
    function readRangeMillimeters () {
      var timeout = 10
      // wait until reading available, or timeout:
      while (i2cBus.readByteSync(node.address, 0x13) & 0x07 === 0) {
        if (--timeout === 0) {
          console.log(new Date().toISOString + 'TIMEOUT')
          return 0
        }
      }

      // get range in mm (just deal with erroneous readings in calling routine).
      var range = i2cBus.readWordSync(node.address, (0x14 + 10))
      // byte swap
      range = ((range & 0xFF) << 8) | ((range >> 8) & 0xFF)

      // clear interrupt (is this needed for a simple polling scheme?):
      i2cBus.writeByteSync(node.address, 0x0B, 0x01)

      return range
    }

    // lidar config VL53L0X_DataInit() stuff:
    //   (just the basic stuff)

    // set 2.8V mode (register 0x89):
    var i = i2cBus.readByteSync(node.address, 0x89) | 0x01 // read and set lsb
    i2cBus.writeByteSync(node.address, 0x89, i) // then write it back

    // set I2C standard mode:
    i2cBus.writeByteSync(node.address, 0x88, 0x00)

    // lidar start continuous back-to-back ranging measurements:
    i2cBus.writeByteSync(node.address, 0x00, 0x02)

    node.status({fill: 'red', shape: 'ring', text: 'pollstop'})

    node.on('close', function (removed, done) {
      i2cBus.closeSync()
      done()
    })

    node.on('input', function (msg) {
      if (msg.payload === 'start') {
        node.intervalId = setInterval(function () {
          // read lidar:
          var range = readRangeMillimeters()

          // so, the max range of this gizmo is about 2000mm, but we will use it in
          // "default mode," which is specified to 1200mm (30 ms range timing budget).
          // note: 20mm or 8190mm seems to be returned for bogus readings, so trap/limit:
          if ((range <= 20) || (range > 1200)) {
            range = 1200
          }

          // --- super-simple "calibration" by surlee:
          if (range > 400) {
            range -= 50
          } else if (range > 160) {
            range -= 40
          } else if (range > 100) {
            range -= 35
          } else {
            range -= 30
          }

          msg.payload = range
          node.send(msg)
        }, node.interval)

        node.status({fill: 'green', shape: 'dot', text: 'pollstart'})
      } else if (msg.payload === 'stop') {
        if (node.intervalId) {
          clearInterval(node.intervalId)
        }

        node.status({fill: 'red', shape: 'ring', text: 'pollstop'})
      }
    })
  }
  RED.nodes.registerType('vl53l0x', Vl53l0xNode)
}
