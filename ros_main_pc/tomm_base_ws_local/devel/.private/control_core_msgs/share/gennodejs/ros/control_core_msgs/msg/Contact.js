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

class Contact {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pose = null;
      this.hull = null;
      this.offset = null;
      this.friction = null;
    }
    else {
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('hull')) {
        this.hull = initObj.hull
      }
      else {
        this.hull = new geometry_msgs.msg.Polygon();
      }
      if (initObj.hasOwnProperty('offset')) {
        this.offset = initObj.offset
      }
      else {
        this.offset = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('friction')) {
        this.friction = initObj.friction
      }
      else {
        this.friction = new std_msgs.msg.Float64();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Contact
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [hull]
    bufferOffset = geometry_msgs.msg.Polygon.serialize(obj.hull, buffer, bufferOffset);
    // Serialize message field [offset]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.offset, buffer, bufferOffset);
    // Serialize message field [friction]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.friction, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Contact
    let len;
    let data = new Contact(null);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [hull]
    data.hull = geometry_msgs.msg.Polygon.deserialize(buffer, bufferOffset);
    // Deserialize message field [offset]
    data.offset = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [friction]
    data.friction = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += geometry_msgs.msg.Polygon.getMessageSize(object.hull);
    return length + 88;
  }

  static datatype() {
    // Returns string type for a message object
    return 'control_core_msgs/Contact';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '092ef89bc484cc9b681ba742d6241a00';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Contact(null);
    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.Pose.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.Pose()
    }

    if (msg.hull !== undefined) {
      resolved.hull = geometry_msgs.msg.Polygon.Resolve(msg.hull)
    }
    else {
      resolved.hull = new geometry_msgs.msg.Polygon()
    }

    if (msg.offset !== undefined) {
      resolved.offset = geometry_msgs.msg.Point.Resolve(msg.offset)
    }
    else {
      resolved.offset = new geometry_msgs.msg.Point()
    }

    if (msg.friction !== undefined) {
      resolved.friction = std_msgs.msg.Float64.Resolve(msg.friction)
    }
    else {
      resolved.friction = new std_msgs.msg.Float64()
    }

    return resolved;
    }
};

module.exports = Contact;
