// Auto-generated. Do not edit!

// (in-package ur_script_manager.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class getScriptManagerStatesRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type getScriptManagerStatesRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type getScriptManagerStatesRequest
    let len;
    let data = new getScriptManagerStatesRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ur_script_manager/getScriptManagerStatesRequest';
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
    const resolved = new getScriptManagerStatesRequest(null);
    return resolved;
    }
};

class getScriptManagerStatesResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.names = null;
      this.enabled = null;
    }
    else {
      if (initObj.hasOwnProperty('names')) {
        this.names = initObj.names
      }
      else {
        this.names = [];
      }
      if (initObj.hasOwnProperty('enabled')) {
        this.enabled = initObj.enabled
      }
      else {
        this.enabled = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type getScriptManagerStatesResponse
    // Serialize message field [names]
    bufferOffset = _arraySerializer.string(obj.names, buffer, bufferOffset, null);
    // Serialize message field [enabled]
    bufferOffset = _arraySerializer.bool(obj.enabled, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type getScriptManagerStatesResponse
    let len;
    let data = new getScriptManagerStatesResponse(null);
    // Deserialize message field [names]
    data.names = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [enabled]
    data.enabled = _arrayDeserializer.bool(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.names.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    length += object.enabled.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ur_script_manager/getScriptManagerStatesResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '44bb719088af7b4a0bc3023901924bc8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] names		# the names of the script sub programs
    bool[] enabled		# the states of the script sub programs
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new getScriptManagerStatesResponse(null);
    if (msg.names !== undefined) {
      resolved.names = msg.names;
    }
    else {
      resolved.names = []
    }

    if (msg.enabled !== undefined) {
      resolved.enabled = msg.enabled;
    }
    else {
      resolved.enabled = []
    }

    return resolved;
    }
};

module.exports = {
  Request: getScriptManagerStatesRequest,
  Response: getScriptManagerStatesResponse,
  md5sum() { return '44bb719088af7b4a0bc3023901924bc8'; },
  datatype() { return 'ur_script_manager/getScriptManagerStates'; }
};
