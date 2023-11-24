// Auto-generated. Do not edit!

// (in-package control_core_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let RobotState = require('../msg/RobotState.js');

//-----------------------------------------------------------

let JointState = require('../msg/JointState.js');

//-----------------------------------------------------------

class ComputeControlRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.state = null;
      this.do_stop = null;
    }
    else {
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = new RobotState();
      }
      if (initObj.hasOwnProperty('do_stop')) {
        this.do_stop = initObj.do_stop
      }
      else {
        this.do_stop = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ComputeControlRequest
    // Serialize message field [state]
    bufferOffset = RobotState.serialize(obj.state, buffer, bufferOffset);
    // Serialize message field [do_stop]
    bufferOffset = _serializer.uint8(obj.do_stop, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ComputeControlRequest
    let len;
    let data = new ComputeControlRequest(null);
    // Deserialize message field [state]
    data.state = RobotState.deserialize(buffer, bufferOffset);
    // Deserialize message field [do_stop]
    data.do_stop = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += RobotState.getMessageSize(object.state);
    return length + 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'control_core_msgs/ComputeControlRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cc8060cad1db6edde9e175062a572328';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    RobotState state    # current robot state
    uint8 do_stop       # plugin request stop
    
    ================================================================================
    MSG: control_core_msgs/RobotState
    JointState joints
    CartesianState floating_base
    sensor_msgs/Imu imu
    geometry_msgs/WrenchStamped[] ft_sensors
    SkinPatch[] patches
    ================================================================================
    MSG: control_core_msgs/JointState
    Vector position
    Vector velocity
    Vector acceleration
    ================================================================================
    MSG: control_core_msgs/Vector
    float64[] data
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
    
    ================================================================================
    MSG: sensor_msgs/Imu
    # This is a message to hold data from an IMU (Inertial Measurement Unit)
    #
    # Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
    #
    # If the covariance of the measurement is known, it should be filled in (if all you know is the 
    # variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
    # A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
    # data a covariance will have to be assumed or gotten from some other source
    #
    # If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
    # estimate), please set element 0 of the associated covariance matrix to -1
    # If you are interpreting this message, please check for a value of -1 in the first element of each 
    # covariance matrix, and disregard the associated estimate.
    
    Header header
    
    geometry_msgs/Quaternion orientation
    float64[9] orientation_covariance # Row major about x, y, z axes
    
    geometry_msgs/Vector3 angular_velocity
    float64[9] angular_velocity_covariance # Row major about x, y, z axes
    
    geometry_msgs/Vector3 linear_acceleration
    float64[9] linear_acceleration_covariance # Row major x, y z 
    
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
    MSG: geometry_msgs/WrenchStamped
    # A wrench with reference coordinate frame and timestamp
    Header header
    Wrench wrench
    
    ================================================================================
    MSG: geometry_msgs/Wrench
    # This represents force in free space, separated into
    # its linear and angular parts.
    Vector3  force
    Vector3  torque
    
    ================================================================================
    MSG: control_core_msgs/SkinPatch
    std_msgs/Header header
    geometry_msgs/Pose pose
    SkinModality force
    SkinModality proximity
    std_msgs/Float64 min_dist
    std_msgs/Float64 max_dist
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
    const resolved = new ComputeControlRequest(null);
    if (msg.state !== undefined) {
      resolved.state = RobotState.Resolve(msg.state)
    }
    else {
      resolved.state = new RobotState()
    }

    if (msg.do_stop !== undefined) {
      resolved.do_stop = msg.do_stop;
    }
    else {
      resolved.do_stop = 0
    }

    return resolved;
    }
};

class ComputeControlResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.command = null;
      this.do_stop = null;
    }
    else {
      if (initObj.hasOwnProperty('command')) {
        this.command = initObj.command
      }
      else {
        this.command = new JointState();
      }
      if (initObj.hasOwnProperty('do_stop')) {
        this.do_stop = initObj.do_stop
      }
      else {
        this.do_stop = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ComputeControlResponse
    // Serialize message field [command]
    bufferOffset = JointState.serialize(obj.command, buffer, bufferOffset);
    // Serialize message field [do_stop]
    bufferOffset = _serializer.uint8(obj.do_stop, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ComputeControlResponse
    let len;
    let data = new ComputeControlResponse(null);
    // Deserialize message field [command]
    data.command = JointState.deserialize(buffer, bufferOffset);
    // Deserialize message field [do_stop]
    data.do_stop = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += JointState.getMessageSize(object.command);
    return length + 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'control_core_msgs/ComputeControlResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e1d5e39018c55415d1bd96b8a2c1d90f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    JointState command  # current robot command
    uint8 do_stop       # client request stop
    
    ================================================================================
    MSG: control_core_msgs/JointState
    Vector position
    Vector velocity
    Vector acceleration
    ================================================================================
    MSG: control_core_msgs/Vector
    float64[] data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ComputeControlResponse(null);
    if (msg.command !== undefined) {
      resolved.command = JointState.Resolve(msg.command)
    }
    else {
      resolved.command = new JointState()
    }

    if (msg.do_stop !== undefined) {
      resolved.do_stop = msg.do_stop;
    }
    else {
      resolved.do_stop = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: ComputeControlRequest,
  Response: ComputeControlResponse,
  md5sum() { return '3f913e6ebf5507fa61207688a03bb51a'; },
  datatype() { return 'control_core_msgs/ComputeControl'; }
};
