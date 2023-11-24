// Auto-generated. Do not edit!

// (in-package tomm_hardware_real.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let DriverState = require('./DriverState.js');

//-----------------------------------------------------------

class OmniBaseState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.state = null;
    }
    else {
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OmniBaseState
    // Serialize message field [state]
    // Serialize the length for message field [state]
    bufferOffset = _serializer.uint32(obj.state.length, buffer, bufferOffset);
    obj.state.forEach((val) => {
      bufferOffset = DriverState.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OmniBaseState
    let len;
    let data = new OmniBaseState(null);
    // Deserialize message field [state]
    // Deserialize array length for message field [state]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.state = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.state[i] = DriverState.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 14 * object.state.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tomm_hardware_real/OmniBaseState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fc9f3e21e19731c56a4c57b6a5b80bb7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    DriverState[] state
    ================================================================================
    MSG: tomm_hardware_real/DriverState
    int32 position
    uint32 digital_inputs
    int32 velocity
    uint16 status
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OmniBaseState(null);
    if (msg.state !== undefined) {
      resolved.state = new Array(msg.state.length);
      for (let i = 0; i < resolved.state.length; ++i) {
        resolved.state[i] = DriverState.Resolve(msg.state[i]);
      }
    }
    else {
      resolved.state = []
    }

    return resolved;
    }
};

module.exports = OmniBaseState;
