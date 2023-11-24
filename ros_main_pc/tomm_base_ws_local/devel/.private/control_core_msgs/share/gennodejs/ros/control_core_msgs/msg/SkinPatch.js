// Auto-generated. Do not edit!

// (in-package control_core_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SkinModality = require('./SkinModality.js');
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class SkinPatch {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.pose = null;
      this.force = null;
      this.proximity = null;
      this.min_dist = null;
      this.max_dist = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('force')) {
        this.force = initObj.force
      }
      else {
        this.force = new SkinModality();
      }
      if (initObj.hasOwnProperty('proximity')) {
        this.proximity = initObj.proximity
      }
      else {
        this.proximity = new SkinModality();
      }
      if (initObj.hasOwnProperty('min_dist')) {
        this.min_dist = initObj.min_dist
      }
      else {
        this.min_dist = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('max_dist')) {
        this.max_dist = initObj.max_dist
      }
      else {
        this.max_dist = new std_msgs.msg.Float64();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SkinPatch
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [force]
    bufferOffset = SkinModality.serialize(obj.force, buffer, bufferOffset);
    // Serialize message field [proximity]
    bufferOffset = SkinModality.serialize(obj.proximity, buffer, bufferOffset);
    // Serialize message field [min_dist]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.min_dist, buffer, bufferOffset);
    // Serialize message field [max_dist]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.max_dist, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SkinPatch
    let len;
    let data = new SkinPatch(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [force]
    data.force = SkinModality.deserialize(buffer, bufferOffset);
    // Deserialize message field [proximity]
    data.proximity = SkinModality.deserialize(buffer, bufferOffset);
    // Deserialize message field [min_dist]
    data.min_dist = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [max_dist]
    data.max_dist = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += SkinModality.getMessageSize(object.force);
    length += SkinModality.getMessageSize(object.proximity);
    return length + 72;
  }

  static datatype() {
    // Returns string type for a message object
    return 'control_core_msgs/SkinPatch';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a5c4ba5fbedd1c63abbdd1a1450dda87';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    geometry_msgs/Pose pose
    SkinModality force
    SkinModality proximity
    std_msgs/Float64 min_dist
    std_msgs/Float64 max_dist
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
    MSG: control_core_msgs/SkinModality
    std_msgs/Float64 min
    std_msgs/Float64 max
    std_msgs/Float64 area
    geometry_msgs/Point cop
    geometry_msgs/Wrench wrench
    geometry_msgs/Polygon hull
    ================================================================================
    MSG: std_msgs/Float64
    float64 data
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SkinPatch(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.Pose.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.Pose()
    }

    if (msg.force !== undefined) {
      resolved.force = SkinModality.Resolve(msg.force)
    }
    else {
      resolved.force = new SkinModality()
    }

    if (msg.proximity !== undefined) {
      resolved.proximity = SkinModality.Resolve(msg.proximity)
    }
    else {
      resolved.proximity = new SkinModality()
    }

    if (msg.min_dist !== undefined) {
      resolved.min_dist = std_msgs.msg.Float64.Resolve(msg.min_dist)
    }
    else {
      resolved.min_dist = new std_msgs.msg.Float64()
    }

    if (msg.max_dist !== undefined) {
      resolved.max_dist = std_msgs.msg.Float64.Resolve(msg.max_dist)
    }
    else {
      resolved.max_dist = new std_msgs.msg.Float64()
    }

    return resolved;
    }
};

module.exports = SkinPatch;
