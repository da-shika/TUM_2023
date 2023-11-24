// Auto-generated. Do not edit!

// (in-package walking_core_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class PlanFootstepsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.n_steps = null;
      this.length = null;
      this.lateral = null;
      this.angle = null;
    }
    else {
      if (initObj.hasOwnProperty('n_steps')) {
        this.n_steps = initObj.n_steps
      }
      else {
        this.n_steps = new std_msgs.msg.Int64();
      }
      if (initObj.hasOwnProperty('length')) {
        this.length = initObj.length
      }
      else {
        this.length = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('lateral')) {
        this.lateral = initObj.lateral
      }
      else {
        this.lateral = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('angle')) {
        this.angle = initObj.angle
      }
      else {
        this.angle = new std_msgs.msg.Float64();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PlanFootstepsRequest
    // Serialize message field [n_steps]
    bufferOffset = std_msgs.msg.Int64.serialize(obj.n_steps, buffer, bufferOffset);
    // Serialize message field [length]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.length, buffer, bufferOffset);
    // Serialize message field [lateral]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.lateral, buffer, bufferOffset);
    // Serialize message field [angle]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.angle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlanFootstepsRequest
    let len;
    let data = new PlanFootstepsRequest(null);
    // Deserialize message field [n_steps]
    data.n_steps = std_msgs.msg.Int64.deserialize(buffer, bufferOffset);
    // Deserialize message field [length]
    data.length = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [lateral]
    data.lateral = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [angle]
    data.angle = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a service object
    return 'walking_core_msgs/PlanFootstepsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4d8147ed6e65a975ff8e2b53102cc02a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Int64      n_steps
    std_msgs/Float64    length
    std_msgs/Float64    lateral
    std_msgs/Float64    angle
    
    ================================================================================
    MSG: std_msgs/Int64
    int64 data
    ================================================================================
    MSG: std_msgs/Float64
    float64 data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PlanFootstepsRequest(null);
    if (msg.n_steps !== undefined) {
      resolved.n_steps = std_msgs.msg.Int64.Resolve(msg.n_steps)
    }
    else {
      resolved.n_steps = new std_msgs.msg.Int64()
    }

    if (msg.length !== undefined) {
      resolved.length = std_msgs.msg.Float64.Resolve(msg.length)
    }
    else {
      resolved.length = new std_msgs.msg.Float64()
    }

    if (msg.lateral !== undefined) {
      resolved.lateral = std_msgs.msg.Float64.Resolve(msg.lateral)
    }
    else {
      resolved.lateral = new std_msgs.msg.Float64()
    }

    if (msg.angle !== undefined) {
      resolved.angle = std_msgs.msg.Float64.Resolve(msg.angle)
    }
    else {
      resolved.angle = new std_msgs.msg.Float64()
    }

    return resolved;
    }
};

class PlanFootstepsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = new std_msgs.msg.Bool();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PlanFootstepsResponse
    // Serialize message field [success]
    bufferOffset = std_msgs.msg.Bool.serialize(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlanFootstepsResponse
    let len;
    let data = new PlanFootstepsResponse(null);
    // Deserialize message field [success]
    data.success = std_msgs.msg.Bool.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'walking_core_msgs/PlanFootstepsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5f31cb2e813cfb0e488c574c3b5d9dbe';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Bool success
    
    
    ================================================================================
    MSG: std_msgs/Bool
    bool data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PlanFootstepsResponse(null);
    if (msg.success !== undefined) {
      resolved.success = std_msgs.msg.Bool.Resolve(msg.success)
    }
    else {
      resolved.success = new std_msgs.msg.Bool()
    }

    return resolved;
    }
};

module.exports = {
  Request: PlanFootstepsRequest,
  Response: PlanFootstepsResponse,
  md5sum() { return '6b7b78d5ced6128d564af0ca90b3d6e2'; },
  datatype() { return 'walking_core_msgs/PlanFootsteps'; }
};
