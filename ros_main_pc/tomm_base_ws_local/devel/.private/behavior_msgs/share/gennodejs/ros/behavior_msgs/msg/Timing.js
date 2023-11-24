// Auto-generated. Do not edit!

// (in-package behavior_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Timing {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.control_loop_dur = null;
      this.solver_dur = null;
      this.geometry_dur = null;
    }
    else {
      if (initObj.hasOwnProperty('control_loop_dur')) {
        this.control_loop_dur = initObj.control_loop_dur
      }
      else {
        this.control_loop_dur = new std_msgs.msg.Int64();
      }
      if (initObj.hasOwnProperty('solver_dur')) {
        this.solver_dur = initObj.solver_dur
      }
      else {
        this.solver_dur = new std_msgs.msg.Int64();
      }
      if (initObj.hasOwnProperty('geometry_dur')) {
        this.geometry_dur = initObj.geometry_dur
      }
      else {
        this.geometry_dur = new std_msgs.msg.Int64();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Timing
    // Serialize message field [control_loop_dur]
    bufferOffset = std_msgs.msg.Int64.serialize(obj.control_loop_dur, buffer, bufferOffset);
    // Serialize message field [solver_dur]
    bufferOffset = std_msgs.msg.Int64.serialize(obj.solver_dur, buffer, bufferOffset);
    // Serialize message field [geometry_dur]
    bufferOffset = std_msgs.msg.Int64.serialize(obj.geometry_dur, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Timing
    let len;
    let data = new Timing(null);
    // Deserialize message field [control_loop_dur]
    data.control_loop_dur = std_msgs.msg.Int64.deserialize(buffer, bufferOffset);
    // Deserialize message field [solver_dur]
    data.solver_dur = std_msgs.msg.Int64.deserialize(buffer, bufferOffset);
    // Deserialize message field [geometry_dur]
    data.geometry_dur = std_msgs.msg.Int64.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'behavior_msgs/Timing';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '860a4c96fe2b2a5b2000e2a96ca33c2c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Int64 control_loop_dur
    std_msgs/Int64 solver_dur
    std_msgs/Int64 geometry_dur
    ================================================================================
    MSG: std_msgs/Int64
    int64 data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Timing(null);
    if (msg.control_loop_dur !== undefined) {
      resolved.control_loop_dur = std_msgs.msg.Int64.Resolve(msg.control_loop_dur)
    }
    else {
      resolved.control_loop_dur = new std_msgs.msg.Int64()
    }

    if (msg.solver_dur !== undefined) {
      resolved.solver_dur = std_msgs.msg.Int64.Resolve(msg.solver_dur)
    }
    else {
      resolved.solver_dur = new std_msgs.msg.Int64()
    }

    if (msg.geometry_dur !== undefined) {
      resolved.geometry_dur = std_msgs.msg.Int64.Resolve(msg.geometry_dur)
    }
    else {
      resolved.geometry_dur = new std_msgs.msg.Int64()
    }

    return resolved;
    }
};

module.exports = Timing;
