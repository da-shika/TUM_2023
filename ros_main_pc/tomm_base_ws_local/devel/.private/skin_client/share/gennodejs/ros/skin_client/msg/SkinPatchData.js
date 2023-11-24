// Auto-generated. Do not edit!

// (in-package skin_client.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SkinPatchData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.prox = null;
      this.force = null;
      this.dist = null;
    }
    else {
      if (initObj.hasOwnProperty('prox')) {
        this.prox = initObj.prox
      }
      else {
        this.prox = [];
      }
      if (initObj.hasOwnProperty('force')) {
        this.force = initObj.force
      }
      else {
        this.force = [];
      }
      if (initObj.hasOwnProperty('dist')) {
        this.dist = initObj.dist
      }
      else {
        this.dist = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SkinPatchData
    // Serialize message field [prox]
    bufferOffset = _arraySerializer.float64(obj.prox, buffer, bufferOffset, null);
    // Serialize message field [force]
    bufferOffset = _arraySerializer.float64(obj.force, buffer, bufferOffset, null);
    // Serialize message field [dist]
    bufferOffset = _arraySerializer.float64(obj.dist, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SkinPatchData
    let len;
    let data = new SkinPatchData(null);
    // Deserialize message field [prox]
    data.prox = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [force]
    data.force = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [dist]
    data.dist = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.prox.length;
    length += 8 * object.force.length;
    length += 8 * object.dist.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'skin_client/SkinPatchData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '179ab063081e8357cb8a2a1174181125';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] prox
    float64[] force
    float64[] dist
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SkinPatchData(null);
    if (msg.prox !== undefined) {
      resolved.prox = msg.prox;
    }
    else {
      resolved.prox = []
    }

    if (msg.force !== undefined) {
      resolved.force = msg.force;
    }
    else {
      resolved.force = []
    }

    if (msg.dist !== undefined) {
      resolved.dist = msg.dist;
    }
    else {
      resolved.dist = []
    }

    return resolved;
    }
};

module.exports = SkinPatchData;
