// Auto-generated. Do not edit!

// (in-package RobotCar.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class web2robotRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.room_point = null;
    }
    else {
      if (initObj.hasOwnProperty('room_point')) {
        this.room_point = initObj.room_point
      }
      else {
        this.room_point = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type web2robotRequest
    // Serialize message field [room_point]
    bufferOffset = _serializer.int32(obj.room_point, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type web2robotRequest
    let len;
    let data = new web2robotRequest(null);
    // Deserialize message field [room_point]
    data.room_point = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'RobotCar/web2robotRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6bda4dac81e87b974068cf32c26e9399';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 room_point
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new web2robotRequest(null);
    if (msg.room_point !== undefined) {
      resolved.room_point = msg.room_point;
    }
    else {
      resolved.room_point = 0
    }

    return resolved;
    }
};

class web2robotResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.goal_point = null;
    }
    else {
      if (initObj.hasOwnProperty('goal_point')) {
        this.goal_point = initObj.goal_point
      }
      else {
        this.goal_point = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type web2robotResponse
    // Serialize message field [goal_point]
    bufferOffset = _serializer.int32(obj.goal_point, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type web2robotResponse
    let len;
    let data = new web2robotResponse(null);
    // Deserialize message field [goal_point]
    data.goal_point = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'RobotCar/web2robotResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '836a7491234659f67cc4e1bb62045b48';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 goal_point
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new web2robotResponse(null);
    if (msg.goal_point !== undefined) {
      resolved.goal_point = msg.goal_point;
    }
    else {
      resolved.goal_point = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: web2robotRequest,
  Response: web2robotResponse,
  md5sum() { return '1b7da60caa7293bcfbc236579ba5c0d8'; },
  datatype() { return 'RobotCar/web2robot'; }
};
