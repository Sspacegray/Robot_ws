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

class voice2robotRequest {
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
    // Serializes a message object of type voice2robotRequest
    // Serialize message field [room_point]
    bufferOffset = _serializer.int32(obj.room_point, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type voice2robotRequest
    let len;
    let data = new voice2robotRequest(null);
    // Deserialize message field [room_point]
    data.room_point = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'RobotCar/voice2robotRequest';
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
    const resolved = new voice2robotRequest(null);
    if (msg.room_point !== undefined) {
      resolved.room_point = msg.room_point;
    }
    else {
      resolved.room_point = 0
    }

    return resolved;
    }
};

class voice2robotResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.roompoint_check = null;
    }
    else {
      if (initObj.hasOwnProperty('roompoint_check')) {
        this.roompoint_check = initObj.roompoint_check
      }
      else {
        this.roompoint_check = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type voice2robotResponse
    // Serialize message field [roompoint_check]
    bufferOffset = _serializer.bool(obj.roompoint_check, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type voice2robotResponse
    let len;
    let data = new voice2robotResponse(null);
    // Deserialize message field [roompoint_check]
    data.roompoint_check = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'RobotCar/voice2robotResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '87ea2ec7918d83076fd2ac06b133f6ea';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool roompoint_check
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new voice2robotResponse(null);
    if (msg.roompoint_check !== undefined) {
      resolved.roompoint_check = msg.roompoint_check;
    }
    else {
      resolved.roompoint_check = false
    }

    return resolved;
    }
};

module.exports = {
  Request: voice2robotRequest,
  Response: voice2robotResponse,
  md5sum() { return '983f78df28f3a33759c7924630309e9f'; },
  datatype() { return 'RobotCar/voice2robot'; }
};
