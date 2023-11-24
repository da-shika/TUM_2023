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
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class BoxManipulation {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.state = null;
      this.W_r_ref = null;
      this.W_l_ref = null;
      this.W_r_off = null;
      this.W_l_off = null;
      this.W_r_des = null;
      this.W_l_des = null;
      this.W_r_real_skin = null;
      this.W_l_real_skin = null;
      this.W_r_real_ft = null;
      this.W_l_real_ft = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = 0;
      }
      if (initObj.hasOwnProperty('W_r_ref')) {
        this.W_r_ref = initObj.W_r_ref
      }
      else {
        this.W_r_ref = new geometry_msgs.msg.Wrench();
      }
      if (initObj.hasOwnProperty('W_l_ref')) {
        this.W_l_ref = initObj.W_l_ref
      }
      else {
        this.W_l_ref = new geometry_msgs.msg.Wrench();
      }
      if (initObj.hasOwnProperty('W_r_off')) {
        this.W_r_off = initObj.W_r_off
      }
      else {
        this.W_r_off = new geometry_msgs.msg.Wrench();
      }
      if (initObj.hasOwnProperty('W_l_off')) {
        this.W_l_off = initObj.W_l_off
      }
      else {
        this.W_l_off = new geometry_msgs.msg.Wrench();
      }
      if (initObj.hasOwnProperty('W_r_des')) {
        this.W_r_des = initObj.W_r_des
      }
      else {
        this.W_r_des = new geometry_msgs.msg.Wrench();
      }
      if (initObj.hasOwnProperty('W_l_des')) {
        this.W_l_des = initObj.W_l_des
      }
      else {
        this.W_l_des = new geometry_msgs.msg.Wrench();
      }
      if (initObj.hasOwnProperty('W_r_real_skin')) {
        this.W_r_real_skin = initObj.W_r_real_skin
      }
      else {
        this.W_r_real_skin = new geometry_msgs.msg.Wrench();
      }
      if (initObj.hasOwnProperty('W_l_real_skin')) {
        this.W_l_real_skin = initObj.W_l_real_skin
      }
      else {
        this.W_l_real_skin = new geometry_msgs.msg.Wrench();
      }
      if (initObj.hasOwnProperty('W_r_real_ft')) {
        this.W_r_real_ft = initObj.W_r_real_ft
      }
      else {
        this.W_r_real_ft = new geometry_msgs.msg.Wrench();
      }
      if (initObj.hasOwnProperty('W_l_real_ft')) {
        this.W_l_real_ft = initObj.W_l_real_ft
      }
      else {
        this.W_l_real_ft = new geometry_msgs.msg.Wrench();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BoxManipulation
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _serializer.uint8(obj.state, buffer, bufferOffset);
    // Serialize message field [W_r_ref]
    bufferOffset = geometry_msgs.msg.Wrench.serialize(obj.W_r_ref, buffer, bufferOffset);
    // Serialize message field [W_l_ref]
    bufferOffset = geometry_msgs.msg.Wrench.serialize(obj.W_l_ref, buffer, bufferOffset);
    // Serialize message field [W_r_off]
    bufferOffset = geometry_msgs.msg.Wrench.serialize(obj.W_r_off, buffer, bufferOffset);
    // Serialize message field [W_l_off]
    bufferOffset = geometry_msgs.msg.Wrench.serialize(obj.W_l_off, buffer, bufferOffset);
    // Serialize message field [W_r_des]
    bufferOffset = geometry_msgs.msg.Wrench.serialize(obj.W_r_des, buffer, bufferOffset);
    // Serialize message field [W_l_des]
    bufferOffset = geometry_msgs.msg.Wrench.serialize(obj.W_l_des, buffer, bufferOffset);
    // Serialize message field [W_r_real_skin]
    bufferOffset = geometry_msgs.msg.Wrench.serialize(obj.W_r_real_skin, buffer, bufferOffset);
    // Serialize message field [W_l_real_skin]
    bufferOffset = geometry_msgs.msg.Wrench.serialize(obj.W_l_real_skin, buffer, bufferOffset);
    // Serialize message field [W_r_real_ft]
    bufferOffset = geometry_msgs.msg.Wrench.serialize(obj.W_r_real_ft, buffer, bufferOffset);
    // Serialize message field [W_l_real_ft]
    bufferOffset = geometry_msgs.msg.Wrench.serialize(obj.W_l_real_ft, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BoxManipulation
    let len;
    let data = new BoxManipulation(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [W_r_ref]
    data.W_r_ref = geometry_msgs.msg.Wrench.deserialize(buffer, bufferOffset);
    // Deserialize message field [W_l_ref]
    data.W_l_ref = geometry_msgs.msg.Wrench.deserialize(buffer, bufferOffset);
    // Deserialize message field [W_r_off]
    data.W_r_off = geometry_msgs.msg.Wrench.deserialize(buffer, bufferOffset);
    // Deserialize message field [W_l_off]
    data.W_l_off = geometry_msgs.msg.Wrench.deserialize(buffer, bufferOffset);
    // Deserialize message field [W_r_des]
    data.W_r_des = geometry_msgs.msg.Wrench.deserialize(buffer, bufferOffset);
    // Deserialize message field [W_l_des]
    data.W_l_des = geometry_msgs.msg.Wrench.deserialize(buffer, bufferOffset);
    // Deserialize message field [W_r_real_skin]
    data.W_r_real_skin = geometry_msgs.msg.Wrench.deserialize(buffer, bufferOffset);
    // Deserialize message field [W_l_real_skin]
    data.W_l_real_skin = geometry_msgs.msg.Wrench.deserialize(buffer, bufferOffset);
    // Deserialize message field [W_r_real_ft]
    data.W_r_real_ft = geometry_msgs.msg.Wrench.deserialize(buffer, bufferOffset);
    // Deserialize message field [W_l_real_ft]
    data.W_l_real_ft = geometry_msgs.msg.Wrench.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 481;
  }

  static datatype() {
    // Returns string type for a message object
    return 'behavior_msgs/BoxManipulation';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1764b3b821c4db67f223b0ac4257c2d9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint8 state
    
    geometry_msgs/Wrench W_r_ref
    geometry_msgs/Wrench W_l_ref
    
    geometry_msgs/Wrench W_r_off
    geometry_msgs/Wrench W_l_off
    
    geometry_msgs/Wrench W_r_des
    geometry_msgs/Wrench W_l_des
    
    geometry_msgs/Wrench W_r_real_skin
    geometry_msgs/Wrench W_l_real_skin
    
    geometry_msgs/Wrench W_r_real_ft
    geometry_msgs/Wrench W_l_real_ft
    
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Wrench
    # This represents force in free space, separated into
    # its linear and angular parts.
    Vector3  force
    Vector3  torque
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BoxManipulation(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = 0
    }

    if (msg.W_r_ref !== undefined) {
      resolved.W_r_ref = geometry_msgs.msg.Wrench.Resolve(msg.W_r_ref)
    }
    else {
      resolved.W_r_ref = new geometry_msgs.msg.Wrench()
    }

    if (msg.W_l_ref !== undefined) {
      resolved.W_l_ref = geometry_msgs.msg.Wrench.Resolve(msg.W_l_ref)
    }
    else {
      resolved.W_l_ref = new geometry_msgs.msg.Wrench()
    }

    if (msg.W_r_off !== undefined) {
      resolved.W_r_off = geometry_msgs.msg.Wrench.Resolve(msg.W_r_off)
    }
    else {
      resolved.W_r_off = new geometry_msgs.msg.Wrench()
    }

    if (msg.W_l_off !== undefined) {
      resolved.W_l_off = geometry_msgs.msg.Wrench.Resolve(msg.W_l_off)
    }
    else {
      resolved.W_l_off = new geometry_msgs.msg.Wrench()
    }

    if (msg.W_r_des !== undefined) {
      resolved.W_r_des = geometry_msgs.msg.Wrench.Resolve(msg.W_r_des)
    }
    else {
      resolved.W_r_des = new geometry_msgs.msg.Wrench()
    }

    if (msg.W_l_des !== undefined) {
      resolved.W_l_des = geometry_msgs.msg.Wrench.Resolve(msg.W_l_des)
    }
    else {
      resolved.W_l_des = new geometry_msgs.msg.Wrench()
    }

    if (msg.W_r_real_skin !== undefined) {
      resolved.W_r_real_skin = geometry_msgs.msg.Wrench.Resolve(msg.W_r_real_skin)
    }
    else {
      resolved.W_r_real_skin = new geometry_msgs.msg.Wrench()
    }

    if (msg.W_l_real_skin !== undefined) {
      resolved.W_l_real_skin = geometry_msgs.msg.Wrench.Resolve(msg.W_l_real_skin)
    }
    else {
      resolved.W_l_real_skin = new geometry_msgs.msg.Wrench()
    }

    if (msg.W_r_real_ft !== undefined) {
      resolved.W_r_real_ft = geometry_msgs.msg.Wrench.Resolve(msg.W_r_real_ft)
    }
    else {
      resolved.W_r_real_ft = new geometry_msgs.msg.Wrench()
    }

    if (msg.W_l_real_ft !== undefined) {
      resolved.W_l_real_ft = geometry_msgs.msg.Wrench.Resolve(msg.W_l_real_ft)
    }
    else {
      resolved.W_l_real_ft = new geometry_msgs.msg.Wrench()
    }

    return resolved;
    }
};

module.exports = BoxManipulation;
