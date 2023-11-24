// Auto-generated. Do not edit!

// (in-package ics_tsid_task_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let control_core_msgs = _finder('control_core_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class SelfCollisionConstraint {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.dur = null;
      this.num_active_ieq = null;
      this.num_violated_ieq = null;
      this.min_distance = null;
      this.distances = null;
    }
    else {
      if (initObj.hasOwnProperty('dur')) {
        this.dur = initObj.dur
      }
      else {
        this.dur = new std_msgs.msg.Int64();
      }
      if (initObj.hasOwnProperty('num_active_ieq')) {
        this.num_active_ieq = initObj.num_active_ieq
      }
      else {
        this.num_active_ieq = new std_msgs.msg.Int32();
      }
      if (initObj.hasOwnProperty('num_violated_ieq')) {
        this.num_violated_ieq = initObj.num_violated_ieq
      }
      else {
        this.num_violated_ieq = new std_msgs.msg.Int32();
      }
      if (initObj.hasOwnProperty('min_distance')) {
        this.min_distance = initObj.min_distance
      }
      else {
        this.min_distance = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('distances')) {
        this.distances = initObj.distances
      }
      else {
        this.distances = new control_core_msgs.msg.Vector();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SelfCollisionConstraint
    // Serialize message field [dur]
    bufferOffset = std_msgs.msg.Int64.serialize(obj.dur, buffer, bufferOffset);
    // Serialize message field [num_active_ieq]
    bufferOffset = std_msgs.msg.Int32.serialize(obj.num_active_ieq, buffer, bufferOffset);
    // Serialize message field [num_violated_ieq]
    bufferOffset = std_msgs.msg.Int32.serialize(obj.num_violated_ieq, buffer, bufferOffset);
    // Serialize message field [min_distance]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.min_distance, buffer, bufferOffset);
    // Serialize message field [distances]
    bufferOffset = control_core_msgs.msg.Vector.serialize(obj.distances, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SelfCollisionConstraint
    let len;
    let data = new SelfCollisionConstraint(null);
    // Deserialize message field [dur]
    data.dur = std_msgs.msg.Int64.deserialize(buffer, bufferOffset);
    // Deserialize message field [num_active_ieq]
    data.num_active_ieq = std_msgs.msg.Int32.deserialize(buffer, bufferOffset);
    // Deserialize message field [num_violated_ieq]
    data.num_violated_ieq = std_msgs.msg.Int32.deserialize(buffer, bufferOffset);
    // Deserialize message field [min_distance]
    data.min_distance = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [distances]
    data.distances = control_core_msgs.msg.Vector.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += control_core_msgs.msg.Vector.getMessageSize(object.distances);
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ics_tsid_task_msgs/SelfCollisionConstraint';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f09e82fd510858f1c3f5993df0a68ab2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Int64 dur
    std_msgs/Int32 num_active_ieq
    std_msgs/Int32 num_violated_ieq
    
    std_msgs/Float64 min_distance
    control_core_msgs/Vector distances
    ================================================================================
    MSG: std_msgs/Int64
    int64 data
    ================================================================================
    MSG: std_msgs/Int32
    int32 data
    ================================================================================
    MSG: std_msgs/Float64
    float64 data
    ================================================================================
    MSG: control_core_msgs/Vector
    float64[] data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SelfCollisionConstraint(null);
    if (msg.dur !== undefined) {
      resolved.dur = std_msgs.msg.Int64.Resolve(msg.dur)
    }
    else {
      resolved.dur = new std_msgs.msg.Int64()
    }

    if (msg.num_active_ieq !== undefined) {
      resolved.num_active_ieq = std_msgs.msg.Int32.Resolve(msg.num_active_ieq)
    }
    else {
      resolved.num_active_ieq = new std_msgs.msg.Int32()
    }

    if (msg.num_violated_ieq !== undefined) {
      resolved.num_violated_ieq = std_msgs.msg.Int32.Resolve(msg.num_violated_ieq)
    }
    else {
      resolved.num_violated_ieq = new std_msgs.msg.Int32()
    }

    if (msg.min_distance !== undefined) {
      resolved.min_distance = std_msgs.msg.Float64.Resolve(msg.min_distance)
    }
    else {
      resolved.min_distance = new std_msgs.msg.Float64()
    }

    if (msg.distances !== undefined) {
      resolved.distances = control_core_msgs.msg.Vector.Resolve(msg.distances)
    }
    else {
      resolved.distances = new control_core_msgs.msg.Vector()
    }

    return resolved;
    }
};

module.exports = SelfCollisionConstraint;
