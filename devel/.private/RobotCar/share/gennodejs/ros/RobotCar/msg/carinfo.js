// Auto-generated. Do not edit!

// (in-package RobotCar.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class carinfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.speed_x = null;
      this.speed_z = null;
      this.power = null;
    }
    else {
      if (initObj.hasOwnProperty('speed_x')) {
        this.speed_x = initObj.speed_x
      }
      else {
        this.speed_x = 0;
      }
      if (initObj.hasOwnProperty('speed_z')) {
        this.speed_z = initObj.speed_z
      }
      else {
        this.speed_z = 0;
      }
      if (initObj.hasOwnProperty('power')) {
        this.power = initObj.power
      }
      else {
        this.power = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type carinfo
    // Serialize message field [speed_x]
    bufferOffset = _serializer.int32(obj.speed_x, buffer, bufferOffset);
    // Serialize message field [speed_z]
    bufferOffset = _serializer.int32(obj.speed_z, buffer, bufferOffset);
    // Serialize message field [power]
    bufferOffset = _serializer.int32(obj.power, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type carinfo
    let len;
    let data = new carinfo(null);
    // Deserialize message field [speed_x]
    data.speed_x = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [speed_z]
    data.speed_z = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [power]
    data.power = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'RobotCar/carinfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '45c0e504be1d5153074c3d5b10d3b65d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 speed_x
    int32 speed_z
    int32 power
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new carinfo(null);
    if (msg.speed_x !== undefined) {
      resolved.speed_x = msg.speed_x;
    }
    else {
      resolved.speed_x = 0
    }

    if (msg.speed_z !== undefined) {
      resolved.speed_z = msg.speed_z;
    }
    else {
      resolved.speed_z = 0
    }

    if (msg.power !== undefined) {
      resolved.power = msg.power;
    }
    else {
      resolved.power = 0
    }

    return resolved;
    }
};

module.exports = carinfo;
