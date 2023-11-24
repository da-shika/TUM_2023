// Auto-generated. Do not edit!

// (in-package behavior_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ChangeBehaviorRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.start_behaviors = null;
      this.stop_behaviors = null;
    }
    else {
      if (initObj.hasOwnProperty('start_behaviors')) {
        this.start_behaviors = initObj.start_behaviors
      }
      else {
        this.start_behaviors = [];
      }
      if (initObj.hasOwnProperty('stop_behaviors')) {
        this.stop_behaviors = initObj.stop_behaviors
      }
      else {
        this.stop_behaviors = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ChangeBehaviorRequest
    // Serialize message field [start_behaviors]
    bufferOffset = _arraySerializer.string(obj.start_behaviors, buffer, bufferOffset, null);
    // Serialize message field [stop_behaviors]
    bufferOffset = _arraySerializer.string(obj.stop_behaviors, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ChangeBehaviorRequest
    let len;
    let data = new ChangeBehaviorRequest(null);
    // Deserialize message field [start_behaviors]
    data.start_behaviors = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [stop_behaviors]
    data.stop_behaviors = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.start_behaviors.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    object.stop_behaviors.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'behavior_msgs/ChangeBehaviorRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '43b5cecd3c7bffade9aa666791c6249f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] start_behaviors
    string[] stop_behaviors
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ChangeBehaviorRequest(null);
    if (msg.start_behaviors !== undefined) {
      resolved.start_behaviors = msg.start_behaviors;
    }
    else {
      resolved.start_behaviors = []
    }

    if (msg.stop_behaviors !== undefined) {
      resolved.stop_behaviors = msg.stop_behaviors;
    }
    else {
      resolved.stop_behaviors = []
    }

    return resolved;
    }
};

class ChangeBehaviorResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ok = null;
    }
    else {
      if (initObj.hasOwnProperty('ok')) {
        this.ok = initObj.ok
      }
      else {
        this.ok = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ChangeBehaviorResponse
    // Serialize message field [ok]
    bufferOffset = _serializer.bool(obj.ok, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ChangeBehaviorResponse
    let len;
    let data = new ChangeBehaviorResponse(null);
    // Deserialize message field [ok]
    data.ok = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'behavior_msgs/ChangeBehaviorResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6f6da3883749771fac40d6deb24a8c02';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool ok
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ChangeBehaviorResponse(null);
    if (msg.ok !== undefined) {
      resolved.ok = msg.ok;
    }
    else {
      resolved.ok = false
    }

    return resolved;
    }
};

module.exports = {
  Request: ChangeBehaviorRequest,
  Response: ChangeBehaviorResponse,
  md5sum() { return 'aa4bf15e9a641af0b18ba2be792bf867'; },
  datatype() { return 'behavior_msgs/ChangeBehavior'; }
};
