// Auto-generated. Do not edit!

// (in-package tomm_hardware_real.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class DriverState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.position = null;
      this.digital_inputs = null;
      this.velocity = null;
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = 0;
      }
      if (initObj.hasOwnProperty('digital_inputs')) {
        this.digital_inputs = initObj.digital_inputs
      }
      else {
        this.digital_inputs = 0;
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = 0;
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DriverState
    // Serialize message field [position]
    bufferOffset = _serializer.int32(obj.position, buffer, bufferOffset);
    // Serialize message field [digital_inputs]
    bufferOffset = _serializer.uint32(obj.digital_inputs, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = _serializer.int32(obj.velocity, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.uint16(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DriverState
    let len;
    let data = new DriverState(null);
    // Deserialize message field [position]
    data.position = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [digital_inputs]
    data.digital_inputs = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 14;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tomm_hardware_real/DriverState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a4fed67ceab1a7997c41bd586c46cc74';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new DriverState(null);
    if (msg.position !== undefined) {
      resolved.position = msg.position;
    }
    else {
      resolved.position = 0
    }

    if (msg.digital_inputs !== undefined) {
      resolved.digital_inputs = msg.digital_inputs;
    }
    else {
      resolved.digital_inputs = 0
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = msg.velocity;
    }
    else {
      resolved.velocity = 0
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = 0
    }

    return resolved;
    }
};

module.exports = DriverState;
