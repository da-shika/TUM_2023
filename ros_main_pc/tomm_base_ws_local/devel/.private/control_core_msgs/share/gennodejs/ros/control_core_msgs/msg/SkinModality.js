// Auto-generated. Do not edit!

// (in-package control_core_msgs.msg)


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

class SkinModality {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.min = null;
      this.max = null;
      this.area = null;
      this.cop = null;
      this.wrench = null;
      this.hull = null;
    }
    else {
      if (initObj.hasOwnProperty('min')) {
        this.min = initObj.min
      }
      else {
        this.min = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('max')) {
        this.max = initObj.max
      }
      else {
        this.max = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('area')) {
        this.area = initObj.area
      }
      else {
        this.area = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('cop')) {
        this.cop = initObj.cop
      }
      else {
        this.cop = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('wrench')) {
        this.wrench = initObj.wrench
      }
      else {
        this.wrench = new geometry_msgs.msg.Wrench();
      }
      if (initObj.hasOwnProperty('hull')) {
        this.hull = initObj.hull
      }
      else {
        this.hull = new geometry_msgs.msg.Polygon();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SkinModality
    // Serialize message field [min]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.min, buffer, bufferOffset);
    // Serialize message field [max]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.max, buffer, bufferOffset);
    // Serialize message field [area]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.area, buffer, bufferOffset);
    // Serialize message field [cop]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.cop, buffer, bufferOffset);
    // Serialize message field [wrench]
    bufferOffset = geometry_msgs.msg.Wrench.serialize(obj.wrench, buffer, bufferOffset);
    // Serialize message field [hull]
    bufferOffset = geometry_msgs.msg.Polygon.serialize(obj.hull, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SkinModality
    let len;
    let data = new SkinModality(null);
    // Deserialize message field [min]
    data.min = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [max]
    data.max = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [area]
    data.area = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [cop]
    data.cop = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [wrench]
    data.wrench = geometry_msgs.msg.Wrench.deserialize(buffer, bufferOffset);
    // Deserialize message field [hull]
    data.hull = geometry_msgs.msg.Polygon.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += geometry_msgs.msg.Polygon.getMessageSize(object.hull);
    return length + 96;
  }

  static datatype() {
    // Returns string type for a message object
    return 'control_core_msgs/SkinModality';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'db74e377abd390ca1fca36f1294853e5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
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
    const resolved = new SkinModality(null);
    if (msg.min !== undefined) {
      resolved.min = std_msgs.msg.Float64.Resolve(msg.min)
    }
    else {
      resolved.min = new std_msgs.msg.Float64()
    }

    if (msg.max !== undefined) {
      resolved.max = std_msgs.msg.Float64.Resolve(msg.max)
    }
    else {
      resolved.max = new std_msgs.msg.Float64()
    }

    if (msg.area !== undefined) {
      resolved.area = std_msgs.msg.Float64.Resolve(msg.area)
    }
    else {
      resolved.area = new std_msgs.msg.Float64()
    }

    if (msg.cop !== undefined) {
      resolved.cop = geometry_msgs.msg.Point.Resolve(msg.cop)
    }
    else {
      resolved.cop = new geometry_msgs.msg.Point()
    }

    if (msg.wrench !== undefined) {
      resolved.wrench = geometry_msgs.msg.Wrench.Resolve(msg.wrench)
    }
    else {
      resolved.wrench = new geometry_msgs.msg.Wrench()
    }

    if (msg.hull !== undefined) {
      resolved.hull = geometry_msgs.msg.Polygon.Resolve(msg.hull)
    }
    else {
      resolved.hull = new geometry_msgs.msg.Polygon()
    }

    return resolved;
    }
};

module.exports = SkinModality;
