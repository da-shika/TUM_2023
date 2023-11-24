// Auto-generated. Do not edit!

// (in-package control_core_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Vector = require('./Vector.js');

//-----------------------------------------------------------

class JointState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.position = null;
      this.velocity = null;
      this.acceleration = null;
    }
    else {
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new Vector();
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = new Vector();
      }
      if (initObj.hasOwnProperty('acceleration')) {
        this.acceleration = initObj.acceleration
      }
      else {
        this.acceleration = new Vector();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JointState
    // Serialize message field [position]
    bufferOffset = Vector.serialize(obj.position, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = Vector.serialize(obj.velocity, buffer, bufferOffset);
    // Serialize message field [acceleration]
    bufferOffset = Vector.serialize(obj.acceleration, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JointState
    let len;
    let data = new JointState(null);
    // Deserialize message field [position]
    data.position = Vector.deserialize(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = Vector.deserialize(buffer, bufferOffset);
    // Deserialize message field [acceleration]
    data.acceleration = Vector.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += Vector.getMessageSize(object.position);
    length += Vector.getMessageSize(object.velocity);
    length += Vector.getMessageSize(object.acceleration);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'control_core_msgs/JointState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6682f84f321f6df0d77b7afd20d85702';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Vector position
    Vector velocity
    Vector acceleration
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
    const resolved = new JointState(null);
    if (msg.position !== undefined) {
      resolved.position = Vector.Resolve(msg.position)
    }
    else {
      resolved.position = new Vector()
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = Vector.Resolve(msg.velocity)
    }
    else {
      resolved.velocity = new Vector()
    }

    if (msg.acceleration !== undefined) {
      resolved.acceleration = Vector.Resolve(msg.acceleration)
    }
    else {
      resolved.acceleration = new Vector()
    }

    return resolved;
    }
};

module.exports = JointState;
