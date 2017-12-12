// Auto-generated. Do not edit!

// (in-package barc.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class obj_offset {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pixel_center_offset = null;
      this.est_dist = null;
      this.est_center_offset = null;
    }
    else {
      if (initObj.hasOwnProperty('pixel_center_offset')) {
        this.pixel_center_offset = initObj.pixel_center_offset
      }
      else {
        this.pixel_center_offset = 0.0;
      }
      if (initObj.hasOwnProperty('est_dist')) {
        this.est_dist = initObj.est_dist
      }
      else {
        this.est_dist = 0.0;
      }
      if (initObj.hasOwnProperty('est_center_offset')) {
        this.est_center_offset = initObj.est_center_offset
      }
      else {
        this.est_center_offset = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type obj_offset
    // Serialize message field [pixel_center_offset]
    bufferOffset = _serializer.float32(obj.pixel_center_offset, buffer, bufferOffset);
    // Serialize message field [est_dist]
    bufferOffset = _serializer.float32(obj.est_dist, buffer, bufferOffset);
    // Serialize message field [est_center_offset]
    bufferOffset = _serializer.float32(obj.est_center_offset, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type obj_offset
    let len;
    let data = new obj_offset(null);
    // Deserialize message field [pixel_center_offset]
    data.pixel_center_offset = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [est_dist]
    data.est_dist = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [est_center_offset]
    data.est_center_offset = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'barc/obj_offset';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '906dd9894d5ba5a23af5c11934a5eac3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 pixel_center_offset
    float32 est_dist
    float32 est_center_offset
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new obj_offset(null);
    if (msg.pixel_center_offset !== undefined) {
      resolved.pixel_center_offset = msg.pixel_center_offset;
    }
    else {
      resolved.pixel_center_offset = 0.0
    }

    if (msg.est_dist !== undefined) {
      resolved.est_dist = msg.est_dist;
    }
    else {
      resolved.est_dist = 0.0
    }

    if (msg.est_center_offset !== undefined) {
      resolved.est_center_offset = msg.est_center_offset;
    }
    else {
      resolved.est_center_offset = 0.0
    }

    return resolved;
    }
};

module.exports = obj_offset;
