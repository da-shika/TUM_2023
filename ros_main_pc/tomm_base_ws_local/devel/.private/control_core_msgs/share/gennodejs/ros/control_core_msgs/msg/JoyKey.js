// Auto-generated. Do not edit!

// (in-package control_core_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class JoyKey {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JoyKey
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JoyKey
    let len;
    let data = new JoyKey(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'control_core_msgs/JoyKey';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7d7dd2dc15a1d1ee57b8e83eb173156c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Keys for the Logitech F710 controller
    # Note: The controller mode is set to 'X'
    
    uint8 KEY_A=0
    uint8 KEY_B=1
    uint8 KEY_X=2
    uint8 KEY_Y=3
    uint8 KEY_LB=4
    uint8 KEY_RB=5
    uint8 JOY_LX=0
    uint8 JOY_LY=1
    uint8 JOY_LT=2
    uint8 JOY_RX=3
    uint8 JOY_RY=4
    uint8 JOY_RT=5
    uint8 ARROW_LEFT_RIGHT=6
    uint8 ARROW_UP_DOWN=7
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JoyKey(null);
    return resolved;
    }
};

// Constants for message
JoyKey.Constants = {
  KEY_A: 0,
  KEY_B: 1,
  KEY_X: 2,
  KEY_Y: 3,
  KEY_LB: 4,
  KEY_RB: 5,
  JOY_LX: 0,
  JOY_LY: 1,
  JOY_LT: 2,
  JOY_RX: 3,
  JOY_RY: 4,
  JOY_RT: 5,
  ARROW_LEFT_RIGHT: 6,
  ARROW_UP_DOWN: 7,
}

module.exports = JoyKey;
