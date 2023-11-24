// Auto-generated. Do not edit!

// (in-package control_core_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetCartesianPositionGoalRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.goal = null;
    }
    else {
      if (initObj.hasOwnProperty('goal')) {
        this.goal = initObj.goal
      }
      else {
        this.goal = new geometry_msgs.msg.Pose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetCartesianPositionGoalRequest
    // Serialize message field [goal]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.goal, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetCartesianPositionGoalRequest
    let len;
    let data = new SetCartesianPositionGoalRequest(null);
    // Deserialize message field [goal]
    data.goal = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 56;
  }

  static datatype() {
    // Returns string type for a service object
    return 'control_core_msgs/SetCartesianPositionGoalRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '313b76aa4f010582b3257488c62ac366';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose goal
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetCartesianPositionGoalRequest(null);
    if (msg.goal !== undefined) {
      resolved.goal = geometry_msgs.msg.Pose.Resolve(msg.goal)
    }
    else {
      resolved.goal = new geometry_msgs.msg.Pose()
    }

    return resolved;
    }
};

class SetCartesianPositionGoalResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetCartesianPositionGoalResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetCartesianPositionGoalResponse
    let len;
    let data = new SetCartesianPositionGoalResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'control_core_msgs/SetCartesianPositionGoalResponse';
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
    const resolved = new SetCartesianPositionGoalResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: SetCartesianPositionGoalRequest,
  Response: SetCartesianPositionGoalResponse,
  md5sum() { return '313b76aa4f010582b3257488c62ac366'; },
  datatype() { return 'control_core_msgs/SetCartesianPositionGoal'; }
};
