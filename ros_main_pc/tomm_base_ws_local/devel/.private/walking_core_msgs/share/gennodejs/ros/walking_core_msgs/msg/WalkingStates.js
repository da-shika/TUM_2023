// Auto-generated. Do not edit!

// (in-package walking_core_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let WalkingPhase = require('./WalkingPhase.js');
let FootStep = require('./FootStep.js');
let control_core_msgs = _finder('control_core_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class WalkingStates {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.phase = null;
      this.stance_foot = null;
      this.swing_foot = null;
      this.elapsed = null;
      this.left_foot_ratio = null;
      this.cur_step = null;
      this.target_step = null;
      this.next_step = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('phase')) {
        this.phase = initObj.phase
      }
      else {
        this.phase = new WalkingPhase();
      }
      if (initObj.hasOwnProperty('stance_foot')) {
        this.stance_foot = initObj.stance_foot
      }
      else {
        this.stance_foot = new control_core_msgs.msg.BodyId();
      }
      if (initObj.hasOwnProperty('swing_foot')) {
        this.swing_foot = initObj.swing_foot
      }
      else {
        this.swing_foot = new control_core_msgs.msg.BodyId();
      }
      if (initObj.hasOwnProperty('elapsed')) {
        this.elapsed = initObj.elapsed
      }
      else {
        this.elapsed = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('left_foot_ratio')) {
        this.left_foot_ratio = initObj.left_foot_ratio
      }
      else {
        this.left_foot_ratio = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('cur_step')) {
        this.cur_step = initObj.cur_step
      }
      else {
        this.cur_step = new FootStep();
      }
      if (initObj.hasOwnProperty('target_step')) {
        this.target_step = initObj.target_step
      }
      else {
        this.target_step = new FootStep();
      }
      if (initObj.hasOwnProperty('next_step')) {
        this.next_step = initObj.next_step
      }
      else {
        this.next_step = new FootStep();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WalkingStates
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [phase]
    bufferOffset = WalkingPhase.serialize(obj.phase, buffer, bufferOffset);
    // Serialize message field [stance_foot]
    bufferOffset = control_core_msgs.msg.BodyId.serialize(obj.stance_foot, buffer, bufferOffset);
    // Serialize message field [swing_foot]
    bufferOffset = control_core_msgs.msg.BodyId.serialize(obj.swing_foot, buffer, bufferOffset);
    // Serialize message field [elapsed]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.elapsed, buffer, bufferOffset);
    // Serialize message field [left_foot_ratio]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.left_foot_ratio, buffer, bufferOffset);
    // Serialize message field [cur_step]
    bufferOffset = FootStep.serialize(obj.cur_step, buffer, bufferOffset);
    // Serialize message field [target_step]
    bufferOffset = FootStep.serialize(obj.target_step, buffer, bufferOffset);
    // Serialize message field [next_step]
    bufferOffset = FootStep.serialize(obj.next_step, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WalkingStates
    let len;
    let data = new WalkingStates(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [phase]
    data.phase = WalkingPhase.deserialize(buffer, bufferOffset);
    // Deserialize message field [stance_foot]
    data.stance_foot = control_core_msgs.msg.BodyId.deserialize(buffer, bufferOffset);
    // Deserialize message field [swing_foot]
    data.swing_foot = control_core_msgs.msg.BodyId.deserialize(buffer, bufferOffset);
    // Deserialize message field [elapsed]
    data.elapsed = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [left_foot_ratio]
    data.left_foot_ratio = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [cur_step]
    data.cur_step = FootStep.deserialize(buffer, bufferOffset);
    // Deserialize message field [target_step]
    data.target_step = FootStep.deserialize(buffer, bufferOffset);
    // Deserialize message field [next_step]
    data.next_step = FootStep.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += FootStep.getMessageSize(object.cur_step);
    length += FootStep.getMessageSize(object.target_step);
    length += FootStep.getMessageSize(object.next_step);
    return length + 19;
  }

  static datatype() {
    // Returns string type for a message object
    return 'walking_core_msgs/WalkingStates';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e12756d011ab43b51ff2b08d69aadcb4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    WalkingPhase phase
    control_core_msgs/BodyId stance_foot
    control_core_msgs/BodyId swing_foot
    std_msgs/Float64 elapsed
    std_msgs/Float64 left_foot_ratio
    FootStep cur_step
    FootStep target_step
    FootStep next_step
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
    MSG: walking_core_msgs/WalkingPhase
    uint8 PHASE_STANCE=0
    uint8 PHASE_DOUBLESUPPORT=1
    uint8 PHASE_SINGLESUPPORT=2
    uint8 phase
    ================================================================================
    MSG: control_core_msgs/BodyId
    uint8 ID_LEFT_FOOT=0
    uint8 ID_RIGHT_FOOT=1
    uint8 ID_LEFT_HAND=2
    uint8 ID_RIGHT_HAND=3
    uint8 id
    ================================================================================
    MSG: std_msgs/Float64
    float64 data
    ================================================================================
    MSG: walking_core_msgs/FootStep
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
    const resolved = new WalkingStates(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.phase !== undefined) {
      resolved.phase = WalkingPhase.Resolve(msg.phase)
    }
    else {
      resolved.phase = new WalkingPhase()
    }

    if (msg.stance_foot !== undefined) {
      resolved.stance_foot = control_core_msgs.msg.BodyId.Resolve(msg.stance_foot)
    }
    else {
      resolved.stance_foot = new control_core_msgs.msg.BodyId()
    }

    if (msg.swing_foot !== undefined) {
      resolved.swing_foot = control_core_msgs.msg.BodyId.Resolve(msg.swing_foot)
    }
    else {
      resolved.swing_foot = new control_core_msgs.msg.BodyId()
    }

    if (msg.elapsed !== undefined) {
      resolved.elapsed = std_msgs.msg.Float64.Resolve(msg.elapsed)
    }
    else {
      resolved.elapsed = new std_msgs.msg.Float64()
    }

    if (msg.left_foot_ratio !== undefined) {
      resolved.left_foot_ratio = std_msgs.msg.Float64.Resolve(msg.left_foot_ratio)
    }
    else {
      resolved.left_foot_ratio = new std_msgs.msg.Float64()
    }

    if (msg.cur_step !== undefined) {
      resolved.cur_step = FootStep.Resolve(msg.cur_step)
    }
    else {
      resolved.cur_step = new FootStep()
    }

    if (msg.target_step !== undefined) {
      resolved.target_step = FootStep.Resolve(msg.target_step)
    }
    else {
      resolved.target_step = new FootStep()
    }

    if (msg.next_step !== undefined) {
      resolved.next_step = FootStep.Resolve(msg.next_step)
    }
    else {
      resolved.next_step = new FootStep()
    }

    return resolved;
    }
};

module.exports = WalkingStates;
