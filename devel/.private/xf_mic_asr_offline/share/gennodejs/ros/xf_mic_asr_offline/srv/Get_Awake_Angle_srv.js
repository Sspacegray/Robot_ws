// Auto-generated. Do not edit!

// (in-package xf_mic_asr_offline.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class Get_Awake_Angle_srvRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.get_awake_angle = null;
    }
    else {
      if (initObj.hasOwnProperty('get_awake_angle')) {
        this.get_awake_angle = initObj.get_awake_angle
      }
      else {
        this.get_awake_angle = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Get_Awake_Angle_srvRequest
    // Serialize message field [get_awake_angle]
    bufferOffset = _serializer.int32(obj.get_awake_angle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Get_Awake_Angle_srvRequest
    let len;
    let data = new Get_Awake_Angle_srvRequest(null);
    // Deserialize message field [get_awake_angle]
    data.get_awake_angle = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xf_mic_asr_offline/Get_Awake_Angle_srvRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '34fe57eef7b5a4ca8b4241d2a8849207';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 get_awake_angle #1,0
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Get_Awake_Angle_srvRequest(null);
    if (msg.get_awake_angle !== undefined) {
      resolved.get_awake_angle = msg.get_awake_angle;
    }
    else {
      resolved.get_awake_angle = 0
    }

    return resolved;
    }
};

class Get_Awake_Angle_srvResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.result = null;
      this.awake_angle = null;
      this.fail_reason = null;
    }
    else {
      if (initObj.hasOwnProperty('result')) {
        this.result = initObj.result
      }
      else {
        this.result = '';
      }
      if (initObj.hasOwnProperty('awake_angle')) {
        this.awake_angle = initObj.awake_angle
      }
      else {
        this.awake_angle = 0;
      }
      if (initObj.hasOwnProperty('fail_reason')) {
        this.fail_reason = initObj.fail_reason
      }
      else {
        this.fail_reason = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Get_Awake_Angle_srvResponse
    // Serialize message field [result]
    bufferOffset = _serializer.string(obj.result, buffer, bufferOffset);
    // Serialize message field [awake_angle]
    bufferOffset = _serializer.int32(obj.awake_angle, buffer, bufferOffset);
    // Serialize message field [fail_reason]
    bufferOffset = _serializer.string(obj.fail_reason, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Get_Awake_Angle_srvResponse
    let len;
    let data = new Get_Awake_Angle_srvResponse(null);
    // Deserialize message field [result]
    data.result = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [awake_angle]
    data.awake_angle = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [fail_reason]
    data.fail_reason = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.result);
    length += _getByteLength(object.fail_reason);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'xf_mic_asr_offline/Get_Awake_Angle_srvResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '855bb7af07835de2687b3e34b41da9c3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string result
    int32 awake_angle
    string fail_reason 
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Get_Awake_Angle_srvResponse(null);
    if (msg.result !== undefined) {
      resolved.result = msg.result;
    }
    else {
      resolved.result = ''
    }

    if (msg.awake_angle !== undefined) {
      resolved.awake_angle = msg.awake_angle;
    }
    else {
      resolved.awake_angle = 0
    }

    if (msg.fail_reason !== undefined) {
      resolved.fail_reason = msg.fail_reason;
    }
    else {
      resolved.fail_reason = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: Get_Awake_Angle_srvRequest,
  Response: Get_Awake_Angle_srvResponse,
  md5sum() { return '9a96c781963794bda6b872fc7788b6bb'; },
  datatype() { return 'xf_mic_asr_offline/Get_Awake_Angle_srv'; }
};
