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

class ChangeContactRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.activate = null;
      this.deactivate = null;
    }
    else {
      if (initObj.hasOwnProperty('activate')) {
        this.activate = initObj.activate
      }
      else {
        this.activate = [];
      }
      if (initObj.hasOwnProperty('deactivate')) {
        this.deactivate = initObj.deactivate
      }
      else {
        this.deactivate = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ChangeContactRequest
    // Serialize message field [activate]
    bufferOffset = _arraySerializer.string(obj.activate, buffer, bufferOffset, null);
    // Serialize message field [deactivate]
    bufferOffset = _arraySerializer.string(obj.deactivate, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ChangeContactRequest
    let len;
    let data = new ChangeContactRequest(null);
    // Deserialize message field [activate]
    data.activate = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [deactivate]
    data.deactivate = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.activate.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    object.deactivate.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'behavior_msgs/ChangeContactRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b51902abff0e958587393c965f5ce9d9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] activate
    string[] deactivate
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ChangeContactRequest(null);
    if (msg.activate !== undefined) {
      resolved.activate = msg.activate;
    }
    else {
      resolved.activate = []
    }

    if (msg.deactivate !== undefined) {
      resolved.deactivate = msg.deactivate;
    }
    else {
      resolved.deactivate = []
    }

    return resolved;
    }
};

class ChangeContactResponse {
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
    // Serializes a message object of type ChangeContactResponse
    // Serialize message field [ok]
    bufferOffset = _serializer.bool(obj.ok, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ChangeContactResponse
    let len;
    let data = new ChangeContactResponse(null);
    // Deserialize message field [ok]
    data.ok = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'behavior_msgs/ChangeContactResponse';
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
    const resolved = new ChangeContactResponse(null);
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
  Request: ChangeContactRequest,
  Response: ChangeContactResponse,
  md5sum() { return 'fb67b79c35378211caa96d9ff91d121b'; },
  datatype() { return 'behavior_msgs/ChangeContact'; }
};
