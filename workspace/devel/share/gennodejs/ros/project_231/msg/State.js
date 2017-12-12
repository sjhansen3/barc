// Auto-generated. Do not edit!

// (in-package project_231.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class State {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.car_vel = null;
      this.car_accl = null;
      this.us_dist = null;
      this.us_rate = null;
      this.us_accl = null;
      this.obj_psi = null;
    }
    else {
      if (initObj.hasOwnProperty('car_vel')) {
        this.car_vel = initObj.car_vel
      }
      else {
        this.car_vel = 0.0;
      }
      if (initObj.hasOwnProperty('car_accl')) {
        this.car_accl = initObj.car_accl
      }
      else {
        this.car_accl = 0.0;
      }
      if (initObj.hasOwnProperty('us_dist')) {
        this.us_dist = initObj.us_dist
      }
      else {
        this.us_dist = 0.0;
      }
      if (initObj.hasOwnProperty('us_rate')) {
        this.us_rate = initObj.us_rate
      }
      else {
        this.us_rate = 0.0;
      }
      if (initObj.hasOwnProperty('us_accl')) {
        this.us_accl = initObj.us_accl
      }
      else {
        this.us_accl = 0.0;
      }
      if (initObj.hasOwnProperty('obj_psi')) {
        this.obj_psi = initObj.obj_psi
      }
      else {
        this.obj_psi = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type State
    // Serialize message field [car_vel]
    bufferOffset = _serializer.float32(obj.car_vel, buffer, bufferOffset);
    // Serialize message field [car_accl]
    bufferOffset = _serializer.float32(obj.car_accl, buffer, bufferOffset);
    // Serialize message field [us_dist]
    bufferOffset = _serializer.float32(obj.us_dist, buffer, bufferOffset);
    // Serialize message field [us_rate]
    bufferOffset = _serializer.float32(obj.us_rate, buffer, bufferOffset);
    // Serialize message field [us_accl]
    bufferOffset = _serializer.float32(obj.us_accl, buffer, bufferOffset);
    // Serialize message field [obj_psi]
    bufferOffset = _serializer.float32(obj.obj_psi, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type State
    let len;
    let data = new State(null);
    // Deserialize message field [car_vel]
    data.car_vel = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [car_accl]
    data.car_accl = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [us_dist]
    data.us_dist = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [us_rate]
    data.us_rate = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [us_accl]
    data.us_accl = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [obj_psi]
    data.obj_psi = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'project_231/State';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '99ffbc9b2ce9f309bebfb0d24dae7716';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 car_vel
    float32 car_accl
    float32 us_dist
    float32 us_rate
    float32 us_accl
    float32 obj_psi
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new State(null);
    if (msg.car_vel !== undefined) {
      resolved.car_vel = msg.car_vel;
    }
    else {
      resolved.car_vel = 0.0
    }

    if (msg.car_accl !== undefined) {
      resolved.car_accl = msg.car_accl;
    }
    else {
      resolved.car_accl = 0.0
    }

    if (msg.us_dist !== undefined) {
      resolved.us_dist = msg.us_dist;
    }
    else {
      resolved.us_dist = 0.0
    }

    if (msg.us_rate !== undefined) {
      resolved.us_rate = msg.us_rate;
    }
    else {
      resolved.us_rate = 0.0
    }

    if (msg.us_accl !== undefined) {
      resolved.us_accl = msg.us_accl;
    }
    else {
      resolved.us_accl = 0.0
    }

    if (msg.obj_psi !== undefined) {
      resolved.obj_psi = msg.obj_psi;
    }
    else {
      resolved.obj_psi = 0.0
    }

    return resolved;
    }
};

module.exports = State;
