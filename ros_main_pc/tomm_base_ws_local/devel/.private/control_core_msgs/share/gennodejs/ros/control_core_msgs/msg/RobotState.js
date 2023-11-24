// Auto-generated. Do not edit!

// (in-package control_core_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let JointState = require('./JointState.js');
let CartesianState = require('./CartesianState.js');
let SkinPatch = require('./SkinPatch.js');
let geometry_msgs = _finder('geometry_msgs');
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------

class RobotState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joints = null;
      this.floating_base = null;
      this.imu = null;
      this.ft_sensors = null;
      this.patches = null;
    }
    else {
      if (initObj.hasOwnProperty('joints')) {
        this.joints = initObj.joints
      }
      else {
        this.joints = new JointState();
      }
      if (initObj.hasOwnProperty('floating_base')) {
        this.floating_base = initObj.floating_base
      }
      else {
        this.floating_base = new CartesianState();
      }
      if (initObj.hasOwnProperty('imu')) {
        this.imu = initObj.imu
      }
      else {
        this.imu = new sensor_msgs.msg.Imu();
      }
      if (initObj.hasOwnProperty('ft_sensors')) {
        this.ft_sensors = initObj.ft_sensors
      }
      else {
        this.ft_sensors = [];
      }
      if (initObj.hasOwnProperty('patches')) {
        this.patches = initObj.patches
      }
      else {
        this.patches = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotState
    // Serialize message field [joints]
    bufferOffset = JointState.serialize(obj.joints, buffer, bufferOffset);
    // Serialize message field [floating_base]
    bufferOffset = CartesianState.serialize(obj.floating_base, buffer, bufferOffset);
    // Serialize message field [imu]
    bufferOffset = sensor_msgs.msg.Imu.serialize(obj.imu, buffer, bufferOffset);
    // Serialize message field [ft_sensors]
    // Serialize the length for message field [ft_sensors]
    bufferOffset = _serializer.uint32(obj.ft_sensors.length, buffer, bufferOffset);
    obj.ft_sensors.forEach((val) => {
      bufferOffset = geometry_msgs.msg.WrenchStamped.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [patches]
    // Serialize the length for message field [patches]
    bufferOffset = _serializer.uint32(obj.patches.length, buffer, bufferOffset);
    obj.patches.forEach((val) => {
      bufferOffset = SkinPatch.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotState
    let len;
    let data = new RobotState(null);
    // Deserialize message field [joints]
    data.joints = JointState.deserialize(buffer, bufferOffset);
    // Deserialize message field [floating_base]
    data.floating_base = CartesianState.deserialize(buffer, bufferOffset);
    // Deserialize message field [imu]
    data.imu = sensor_msgs.msg.Imu.deserialize(buffer, bufferOffset);
    // Deserialize message field [ft_sensors]
    // Deserialize array length for message field [ft_sensors]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.ft_sensors = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.ft_sensors[i] = geometry_msgs.msg.WrenchStamped.deserialize(buffer, bufferOffset)
    }
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
    length += JointState.getMessageSize(object.joints);
    length += sensor_msgs.msg.Imu.getMessageSize(object.imu);
    object.ft_sensors.forEach((val) => {
      length += geometry_msgs.msg.WrenchStamped.getMessageSize(val);
    });
    object.patches.forEach((val) => {
      length += SkinPatch.getMessageSize(val);
    });
    return length + 160;
  }

  static datatype() {
    // Returns string type for a message object
    return 'control_core_msgs/RobotState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c93ef624511e3a3594ee3528444b88cf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new RobotState(null);
    if (msg.joints !== undefined) {
      resolved.joints = JointState.Resolve(msg.joints)
    }
    else {
      resolved.joints = new JointState()
    }

    if (msg.floating_base !== undefined) {
      resolved.floating_base = CartesianState.Resolve(msg.floating_base)
    }
    else {
      resolved.floating_base = new CartesianState()
    }

    if (msg.imu !== undefined) {
      resolved.imu = sensor_msgs.msg.Imu.Resolve(msg.imu)
    }
    else {
      resolved.imu = new sensor_msgs.msg.Imu()
    }

    if (msg.ft_sensors !== undefined) {
      resolved.ft_sensors = new Array(msg.ft_sensors.length);
      for (let i = 0; i < resolved.ft_sensors.length; ++i) {
        resolved.ft_sensors[i] = geometry_msgs.msg.WrenchStamped.Resolve(msg.ft_sensors[i]);
      }
    }
    else {
      resolved.ft_sensors = []
    }

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

module.exports = RobotState;
