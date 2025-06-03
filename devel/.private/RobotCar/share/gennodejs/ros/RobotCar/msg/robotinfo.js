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

class robotinfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robotstate = null;
      this.robotvoltage = null;
      this.lastroompoint = null;
    }
    else {
      if (initObj.hasOwnProperty('robotstate')) {
        this.robotstate = initObj.robotstate
      }
      else {
        this.robotstate = 0;
      }
      if (initObj.hasOwnProperty('robotvoltage')) {
        this.robotvoltage = initObj.robotvoltage
      }
      else {
        this.robotvoltage = 0;
      }
      if (initObj.hasOwnProperty('lastroompoint')) {
        this.lastroompoint = initObj.lastroompoint
      }
      else {
        this.lastroompoint = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type robotinfo
    // Serialize message field [robotstate]
    bufferOffset = _serializer.int32(obj.robotstate, buffer, bufferOffset);
    // Serialize message field [robotvoltage]
    bufferOffset = _serializer.int32(obj.robotvoltage, buffer, bufferOffset);
    // Serialize message field [lastroompoint]
    bufferOffset = _serializer.int32(obj.lastroompoint, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type robotinfo
    let len;
    let data = new robotinfo(null);
    // Deserialize message field [robotstate]
    data.robotstate = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [robotvoltage]
    data.robotvoltage = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [lastroompoint]
    data.lastroompoint = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'RobotCar/robotinfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '00ba0d14f8e4b705d2bc45e5c870cf2b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 robotstate
    int32 robotvoltage
    int32 lastroompoint
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new robotinfo(null);
    if (msg.robotstate !== undefined) {
      resolved.robotstate = msg.robotstate;
    }
    else {
      resolved.robotstate = 0
    }

    if (msg.robotvoltage !== undefined) {
      resolved.robotvoltage = msg.robotvoltage;
    }
    else {
      resolved.robotvoltage = 0
    }

    if (msg.lastroompoint !== undefined) {
      resolved.lastroompoint = msg.lastroompoint;
    }
    else {
      resolved.lastroompoint = 0
    }

    return resolved;
    }
};

module.exports = robotinfo;
