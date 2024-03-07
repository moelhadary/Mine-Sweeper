// Auto-generated. Do not edit!

// (in-package motor_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class PwmArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pwm_values = null;
    }
    else {
      if (initObj.hasOwnProperty('pwm_values')) {
        this.pwm_values = initObj.pwm_values
      }
      else {
        this.pwm_values = new Array(6).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PwmArray
    // Check that the constant length array field [pwm_values] has the right length
    if (obj.pwm_values.length !== 6) {
      throw new Error('Unable to serialize array field pwm_values - length must be 6')
    }
    // Serialize message field [pwm_values]
    bufferOffset = _arraySerializer.int32(obj.pwm_values, buffer, bufferOffset, 6);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PwmArray
    let len;
    let data = new PwmArray(null);
    // Deserialize message field [pwm_values]
    data.pwm_values = _arrayDeserializer.int32(buffer, bufferOffset, 6)
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'motor_control/PwmArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bd3f5c6582467ef6ecc8c0b440e1fb77';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32[6] pwm_values
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PwmArray(null);
    if (msg.pwm_values !== undefined) {
      resolved.pwm_values = msg.pwm_values;
    }
    else {
      resolved.pwm_values = new Array(6).fill(0)
    }

    return resolved;
    }
};

module.exports = PwmArray;
