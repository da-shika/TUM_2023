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

class SkinDistanceConstraint {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.dur = null;
      this.num_active_ieq = null;
      this.num_violated_ieq = null;
      this.conflict = null;
      this.max_force = null;
      this.min_distance = null;
      this.max_proximity = null;
      this.distances = null;
      this.cell_nums = null;
      this.vel_limits = null;
      this.vel_cmds = null;
      this.acc_limits = null;
      this.weight = null;
      this.a_relaxed = null;
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
      if (initObj.hasOwnProperty('conflict')) {
        this.conflict = initObj.conflict
      }
      else {
        this.conflict = new std_msgs.msg.Int32();
      }
      if (initObj.hasOwnProperty('max_force')) {
        this.max_force = initObj.max_force
      }
      else {
        this.max_force = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('min_distance')) {
        this.min_distance = initObj.min_distance
      }
      else {
        this.min_distance = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('max_proximity')) {
        this.max_proximity = initObj.max_proximity
      }
      else {
        this.max_proximity = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('distances')) {
        this.distances = initObj.distances
      }
      else {
        this.distances = new control_core_msgs.msg.Vector();
      }
      if (initObj.hasOwnProperty('cell_nums')) {
        this.cell_nums = initObj.cell_nums
      }
      else {
        this.cell_nums = new control_core_msgs.msg.Vector();
      }
      if (initObj.hasOwnProperty('vel_limits')) {
        this.vel_limits = initObj.vel_limits
      }
      else {
        this.vel_limits = new control_core_msgs.msg.Vector();
      }
      if (initObj.hasOwnProperty('vel_cmds')) {
        this.vel_cmds = initObj.vel_cmds
      }
      else {
        this.vel_cmds = new control_core_msgs.msg.Vector();
      }
      if (initObj.hasOwnProperty('acc_limits')) {
        this.acc_limits = initObj.acc_limits
      }
      else {
        this.acc_limits = new control_core_msgs.msg.Vector();
      }
      if (initObj.hasOwnProperty('weight')) {
        this.weight = initObj.weight
      }
      else {
        this.weight = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('a_relaxed')) {
        this.a_relaxed = initObj.a_relaxed
      }
      else {
        this.a_relaxed = new control_core_msgs.msg.Vector();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SkinDistanceConstraint
    // Serialize message field [dur]
    bufferOffset = std_msgs.msg.Int64.serialize(obj.dur, buffer, bufferOffset);
    // Serialize message field [num_active_ieq]
    bufferOffset = std_msgs.msg.Int32.serialize(obj.num_active_ieq, buffer, bufferOffset);
    // Serialize message field [num_violated_ieq]
    bufferOffset = std_msgs.msg.Int32.serialize(obj.num_violated_ieq, buffer, bufferOffset);
    // Serialize message field [conflict]
    bufferOffset = std_msgs.msg.Int32.serialize(obj.conflict, buffer, bufferOffset);
    // Serialize message field [max_force]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.max_force, buffer, bufferOffset);
    // Serialize message field [min_distance]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.min_distance, buffer, bufferOffset);
    // Serialize message field [max_proximity]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.max_proximity, buffer, bufferOffset);
    // Serialize message field [distances]
    bufferOffset = control_core_msgs.msg.Vector.serialize(obj.distances, buffer, bufferOffset);
    // Serialize message field [cell_nums]
    bufferOffset = control_core_msgs.msg.Vector.serialize(obj.cell_nums, buffer, bufferOffset);
    // Serialize message field [vel_limits]
    bufferOffset = control_core_msgs.msg.Vector.serialize(obj.vel_limits, buffer, bufferOffset);
    // Serialize message field [vel_cmds]
    bufferOffset = control_core_msgs.msg.Vector.serialize(obj.vel_cmds, buffer, bufferOffset);
    // Serialize message field [acc_limits]
    bufferOffset = control_core_msgs.msg.Vector.serialize(obj.acc_limits, buffer, bufferOffset);
    // Serialize message field [weight]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.weight, buffer, bufferOffset);
    // Serialize message field [a_relaxed]
    bufferOffset = control_core_msgs.msg.Vector.serialize(obj.a_relaxed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SkinDistanceConstraint
    let len;
    let data = new SkinDistanceConstraint(null);
    // Deserialize message field [dur]
    data.dur = std_msgs.msg.Int64.deserialize(buffer, bufferOffset);
    // Deserialize message field [num_active_ieq]
    data.num_active_ieq = std_msgs.msg.Int32.deserialize(buffer, bufferOffset);
    // Deserialize message field [num_violated_ieq]
    data.num_violated_ieq = std_msgs.msg.Int32.deserialize(buffer, bufferOffset);
    // Deserialize message field [conflict]
    data.conflict = std_msgs.msg.Int32.deserialize(buffer, bufferOffset);
    // Deserialize message field [max_force]
    data.max_force = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [min_distance]
    data.min_distance = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [max_proximity]
    data.max_proximity = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [distances]
    data.distances = control_core_msgs.msg.Vector.deserialize(buffer, bufferOffset);
    // Deserialize message field [cell_nums]
    data.cell_nums = control_core_msgs.msg.Vector.deserialize(buffer, bufferOffset);
    // Deserialize message field [vel_limits]
    data.vel_limits = control_core_msgs.msg.Vector.deserialize(buffer, bufferOffset);
    // Deserialize message field [vel_cmds]
    data.vel_cmds = control_core_msgs.msg.Vector.deserialize(buffer, bufferOffset);
    // Deserialize message field [acc_limits]
    data.acc_limits = control_core_msgs.msg.Vector.deserialize(buffer, bufferOffset);
    // Deserialize message field [weight]
    data.weight = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [a_relaxed]
    data.a_relaxed = control_core_msgs.msg.Vector.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += control_core_msgs.msg.Vector.getMessageSize(object.distances);
    length += control_core_msgs.msg.Vector.getMessageSize(object.cell_nums);
    length += control_core_msgs.msg.Vector.getMessageSize(object.vel_limits);
    length += control_core_msgs.msg.Vector.getMessageSize(object.vel_cmds);
    length += control_core_msgs.msg.Vector.getMessageSize(object.acc_limits);
    length += control_core_msgs.msg.Vector.getMessageSize(object.a_relaxed);
    return length + 52;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ics_tsid_task_msgs/SkinDistanceConstraint';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f1adb03c236ae24d7d056bb4d4bea499';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Int64 dur
    std_msgs/Int32 num_active_ieq
    std_msgs/Int32 num_violated_ieq
    std_msgs/Int32 conflict
    
    std_msgs/Float64 max_force
    std_msgs/Float64 min_distance
    std_msgs/Float64 max_proximity
    
    control_core_msgs/Vector distances
    control_core_msgs/Vector cell_nums
    control_core_msgs/Vector vel_limits
    control_core_msgs/Vector vel_cmds
    control_core_msgs/Vector acc_limits
    
    std_msgs/Float64 weight
    control_core_msgs/Vector a_relaxed
    
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
    const resolved = new SkinDistanceConstraint(null);
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

    if (msg.conflict !== undefined) {
      resolved.conflict = std_msgs.msg.Int32.Resolve(msg.conflict)
    }
    else {
      resolved.conflict = new std_msgs.msg.Int32()
    }

    if (msg.max_force !== undefined) {
      resolved.max_force = std_msgs.msg.Float64.Resolve(msg.max_force)
    }
    else {
      resolved.max_force = new std_msgs.msg.Float64()
    }

    if (msg.min_distance !== undefined) {
      resolved.min_distance = std_msgs.msg.Float64.Resolve(msg.min_distance)
    }
    else {
      resolved.min_distance = new std_msgs.msg.Float64()
    }

    if (msg.max_proximity !== undefined) {
      resolved.max_proximity = std_msgs.msg.Float64.Resolve(msg.max_proximity)
    }
    else {
      resolved.max_proximity = new std_msgs.msg.Float64()
    }

    if (msg.distances !== undefined) {
      resolved.distances = control_core_msgs.msg.Vector.Resolve(msg.distances)
    }
    else {
      resolved.distances = new control_core_msgs.msg.Vector()
    }

    if (msg.cell_nums !== undefined) {
      resolved.cell_nums = control_core_msgs.msg.Vector.Resolve(msg.cell_nums)
    }
    else {
      resolved.cell_nums = new control_core_msgs.msg.Vector()
    }

    if (msg.vel_limits !== undefined) {
      resolved.vel_limits = control_core_msgs.msg.Vector.Resolve(msg.vel_limits)
    }
    else {
      resolved.vel_limits = new control_core_msgs.msg.Vector()
    }

    if (msg.vel_cmds !== undefined) {
      resolved.vel_cmds = control_core_msgs.msg.Vector.Resolve(msg.vel_cmds)
    }
    else {
      resolved.vel_cmds = new control_core_msgs.msg.Vector()
    }

    if (msg.acc_limits !== undefined) {
      resolved.acc_limits = control_core_msgs.msg.Vector.Resolve(msg.acc_limits)
    }
    else {
      resolved.acc_limits = new control_core_msgs.msg.Vector()
    }

    if (msg.weight !== undefined) {
      resolved.weight = std_msgs.msg.Float64.Resolve(msg.weight)
    }
    else {
      resolved.weight = new std_msgs.msg.Float64()
    }

    if (msg.a_relaxed !== undefined) {
      resolved.a_relaxed = control_core_msgs.msg.Vector.Resolve(msg.a_relaxed)
    }
    else {
      resolved.a_relaxed = new control_core_msgs.msg.Vector()
    }

    return resolved;
    }
};

module.exports = SkinDistanceConstraint;
