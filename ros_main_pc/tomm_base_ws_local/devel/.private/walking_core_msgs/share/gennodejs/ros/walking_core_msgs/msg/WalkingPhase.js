// Auto-generated. Do not edit!

// (in-package walking_core_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class WalkingPhase {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.phase = null;
    }
    else {
      if (initObj.hasOwnProperty('phase')) {
        this.phase = initObj.phase
      }
      else {
        this.phase = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WalkingPhase
    // Serialize message field [phase]
    bufferOffset = _serializer.uint8(obj.phase, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WalkingPhase
    let len;
    let data = new WalkingPhase(null);
    // Deserialize message field [phase]
    data.phase = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a message object
    return 'walking_core_msgs/WalkingPhase';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1712835dbfb69f4b5728c67900acc6bf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 PHASE_STANCE=0
    uint8 PHASE_DOUBLESUPPORT=1
    uint8 PHASE_SINGLESUPPORT=2
    uint8 phase
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WalkingPhase(null);
    if (msg.phase !== undefined) {
      resolved.phase = msg.phase;
    }
    else {
      resolved.phase = 0
    }

    return resolved;
    }
};

// Constants for message
WalkingPhase.Constants = {
  PHASE_STANCE: 0,
  PHASE_DOUBLESUPPORT: 1,
  PHASE_SINGLESUPPORT: 2,
}

module.exports = WalkingPhase;
