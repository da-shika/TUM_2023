// Auto-generated. Do not edit!

// (in-package control_core_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SkinPatch = require('./SkinPatch.js');

//-----------------------------------------------------------

class SkinPatches {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.patches = null;
    }
    else {
      if (initObj.hasOwnProperty('patches')) {
        this.patches = initObj.patches
      }
      else {
        this.patches = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SkinPatches
    // Serialize message field [patches]
    // Serialize the length for message field [patches]
    bufferOffset = _serializer.uint32(obj.patches.length, buffer, bufferOffset);
    obj.patches.forEach((val) => {
      bufferOffset = SkinPatch.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SkinPatches
    let len;
    let data = new SkinPatches(null);
    // Deserialize message field [patches]
    // Deserialize array length for message field [patches]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.patches = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.patches[i] = SkinPatch.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.patches.forEach((val) => {
      length += SkinPatch.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'control_core_msgs/SkinPatches';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '54325af118430bc2db92b5f684606b52';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    SkinPatch[] patches
    ================================================================================
    MSG: control_core_msgs/SkinPatch
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
    const resolved = new SkinPatches(null);
    if (msg.patches !== undefined) {
      resolved.patches = new Array(msg.patches.length);
      for (let i = 0; i < resolved.patches.length; ++i) {
        resolved.patches[i] = SkinPatch.Resolve(msg.patches[i]);
      }
    }
    else {
      resolved.patches = []
    }

    return resolved;
    }
};

module.exports = SkinPatches;
