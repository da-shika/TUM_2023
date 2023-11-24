// Auto-generated. Do not edit!

// (in-package control_core_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let CartesianState = require('../msg/CartesianState.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetCartesianStateGoalRequest {
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
        this.goal = new CartesianState();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetCartesianStateGoalRequest
    // Serialize message field [goal]
    bufferOffset = CartesianState.serialize(obj.goal, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetCartesianStateGoalRequest
    let len;
    let data = new SetCartesianStateGoalRequest(null);
    // Deserialize message field [goal]
    data.goal = CartesianState.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 152;
  }

  static datatype() {
    // Returns string type for a service object
    return 'control_core_msgs/SetCartesianStateGoalRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2f3406a57b13431c34e22c35d33bdee4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    CartesianState goal
    
    ================================================================================
    MSG: control_core_msgs/CartesianState
    geometry_msgs/Pose position
    geometry_msgs/Twist velocity
    geometry_msgs/Accel acceleration
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
    MSG: geometry_msgs/Twist
    # This expresses velocity in free space broken into its linear and angular parts.
    Vector3  linear
    Vector3  angular
    
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
    MSG: geometry_msgs/Accel
    # This expresses acceleration in free space broken into its linear and angular parts.
    Vector3  linear
    Vector3  angular
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetCartesianStateGoalRequest(null);
    if (msg.goal !== undefined) {
      resolved.goal = CartesianState.Resolve(msg.goal)
    }
    else {
      resolved.goal = new CartesianState()
    }

    return resolved;
    }
};

class SetCartesianStateGoalResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetCartesianStateGoalResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetCartesianStateGoalResponse
    let len;
    let data = new SetCartesianStateGoalResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'control_core_msgs/SetCartesianStateGoalResponse';
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
    const resolved = new SetCartesianStateGoalResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: SetCartesianStateGoalRequest,
  Response: SetCartesianStateGoalResponse,
  md5sum() { return '2f3406a57b13431c34e22c35d33bdee4'; },
  datatype() { return 'control_core_msgs/SetCartesianStateGoal'; }
};
