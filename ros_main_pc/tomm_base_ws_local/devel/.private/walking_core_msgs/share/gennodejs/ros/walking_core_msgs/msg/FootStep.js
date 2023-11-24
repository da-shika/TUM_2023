// Auto-generated. Do not edit!

// (in-package walking_core_msgs.msg)


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

class FootStep {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.contact = null;
      this.body_id = null;
      this.final_step = null;
      this.n_step = null;
    }
    else {
      if (initObj.hasOwnProperty('contact')) {
        this.contact = initObj.contact
      }
      else {
        this.contact = new control_core_msgs.msg.Contact();
      }
      if (initObj.hasOwnProperty('body_id')) {
        this.body_id = initObj.body_id
      }
      else {
        this.body_id = new std_msgs.msg.Int64();
      }
      if (initObj.hasOwnProperty('final_step')) {
        this.final_step = initObj.final_step
      }
      else {
        this.final_step = new std_msgs.msg.Bool();
      }
      if (initObj.hasOwnProperty('n_step')) {
        this.n_step = initObj.n_step
      }
      else {
        this.n_step = new std_msgs.msg.Int64();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FootStep
    // Serialize message field [contact]
    bufferOffset = control_core_msgs.msg.Contact.serialize(obj.contact, buffer, bufferOffset);
    // Serialize message field [body_id]
    bufferOffset = std_msgs.msg.Int64.serialize(obj.body_id, buffer, bufferOffset);
    // Serialize message field [final_step]
    bufferOffset = std_msgs.msg.Bool.serialize(obj.final_step, buffer, bufferOffset);
    // Serialize message field [n_step]
    bufferOffset = std_msgs.msg.Int64.serialize(obj.n_step, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FootStep
    let len;
    let data = new FootStep(null);
    // Deserialize message field [contact]
    data.contact = control_core_msgs.msg.Contact.deserialize(buffer, bufferOffset);
    // Deserialize message field [body_id]
    data.body_id = std_msgs.msg.Int64.deserialize(buffer, bufferOffset);
    // Deserialize message field [final_step]
    data.final_step = std_msgs.msg.Bool.deserialize(buffer, bufferOffset);
    // Deserialize message field [n_step]
    data.n_step = std_msgs.msg.Int64.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += control_core_msgs.msg.Contact.getMessageSize(object.contact);
    return length + 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'walking_core_msgs/FootStep';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9d0f09bb8a0492982eab7b79dd33028a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    control_core_msgs/Contact contact
    std_msgs/Int64 body_id
    std_msgs/Bool final_step
    std_msgs/Int64 n_step
    ================================================================================
    MSG: control_core_msgs/Contact
    geometry_msgs/Pose pose
    geometry_msgs/Polygon hull
    geometry_msgs/Point offset
    std_msgs/Float64 friction
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: geometry_msgs/Polygon
    #A specification of a polygon where the first and last points are assumed to be connected
    Point32[] points
    
    ================================================================================
    MSG: geometry_msgs/Point32
    # This contains the position of a point in free space(with 32 bits of precision).
    # It is recommeded to use Point wherever possible instead of Point32.  
    # 
    # This recommendation is to promote interoperability.  
    #
    # This message is designed to take up less space when sending
    # lots of points at once, as in the case of a PointCloud.  
    
    float32 x
    float32 y
    float32 z
    ================================================================================
    MSG: std_msgs/Float64
    float64 data
    ================================================================================
    MSG: std_msgs/Int64
    int64 data
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
    const resolved = new FootStep(null);
    if (msg.contact !== undefined) {
      resolved.contact = control_core_msgs.msg.Contact.Resolve(msg.contact)
    }
    else {
      resolved.contact = new control_core_msgs.msg.Contact()
    }

    if (msg.body_id !== undefined) {
      resolved.body_id = std_msgs.msg.Int64.Resolve(msg.body_id)
    }
    else {
      resolved.body_id = new std_msgs.msg.Int64()
    }

    if (msg.final_step !== undefined) {
      resolved.final_step = std_msgs.msg.Bool.Resolve(msg.final_step)
    }
    else {
      resolved.final_step = new std_msgs.msg.Bool()
    }

    if (msg.n_step !== undefined) {
      resolved.n_step = std_msgs.msg.Int64.Resolve(msg.n_step)
    }
    else {
      resolved.n_step = new std_msgs.msg.Int64()
    }

    return resolved;
    }
};

module.exports = FootStep;
