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

class ListBehaviorRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ListBehaviorRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ListBehaviorRequest
    let len;
    let data = new ListBehaviorRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'behavior_msgs/ListBehaviorRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ListBehaviorRequest(null);
    return resolved;
    }
};

class ListBehaviorResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.loaded_behaviors = null;
      this.running_behaviors = null;
    }
    else {
      if (initObj.hasOwnProperty('loaded_behaviors')) {
        this.loaded_behaviors = initObj.loaded_behaviors
      }
      else {
        this.loaded_behaviors = [];
      }
      if (initObj.hasOwnProperty('running_behaviors')) {
        this.running_behaviors = initObj.running_behaviors
      }
      else {
        this.running_behaviors = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ListBehaviorResponse
    // Serialize message field [loaded_behaviors]
    bufferOffset = _arraySerializer.string(obj.loaded_behaviors, buffer, bufferOffset, null);
    // Serialize message field [running_behaviors]
    bufferOffset = _arraySerializer.string(obj.running_behaviors, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ListBehaviorResponse
    let len;
    let data = new ListBehaviorResponse(null);
    // Deserialize message field [loaded_behaviors]
    data.loaded_behaviors = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [running_behaviors]
    data.running_behaviors = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.loaded_behaviors.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    object.running_behaviors.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'behavior_msgs/ListBehaviorResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'db7d6a6bd85a4f8fde354aa4e08d9629';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] loaded_behaviors
    string[] running_behaviors
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ListBehaviorResponse(null);
    if (msg.loaded_behaviors !== undefined) {
      resolved.loaded_behaviors = msg.loaded_behaviors;
    }
    else {
      resolved.loaded_behaviors = []
    }

    if (msg.running_behaviors !== undefined) {
      resolved.running_behaviors = msg.running_behaviors;
    }
    else {
      resolved.running_behaviors = []
    }

    return resolved;
    }
};

module.exports = {
  Request: ListBehaviorRequest,
  Response: ListBehaviorResponse,
  md5sum() { return 'db7d6a6bd85a4f8fde354aa4e08d9629'; },
  datatype() { return 'behavior_msgs/ListBehavior'; }
};
