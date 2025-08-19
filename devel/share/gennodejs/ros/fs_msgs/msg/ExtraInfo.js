// Auto-generated. Do not edit!

// (in-package fs_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ExtraInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.doo_counter = null;
      this.laps = null;
    }
    else {
      if (initObj.hasOwnProperty('doo_counter')) {
        this.doo_counter = initObj.doo_counter
      }
      else {
        this.doo_counter = 0;
      }
      if (initObj.hasOwnProperty('laps')) {
        this.laps = initObj.laps
      }
      else {
        this.laps = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ExtraInfo
    // Serialize message field [doo_counter]
    bufferOffset = _serializer.uint32(obj.doo_counter, buffer, bufferOffset);
    // Serialize message field [laps]
    bufferOffset = _arraySerializer.float32(obj.laps, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ExtraInfo
    let len;
    let data = new ExtraInfo(null);
    // Deserialize message field [doo_counter]
    data.doo_counter = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [laps]
    data.laps = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.laps.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fs_msgs/ExtraInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '722e1c6d60f33a21de73c32b7583c9b8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # The number of Down Or Out cones during the run
    uint32 doo_counter
    
    # The number of finished laps driven by the vehicle
    float32[] laps 
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ExtraInfo(null);
    if (msg.doo_counter !== undefined) {
      resolved.doo_counter = msg.doo_counter;
    }
    else {
      resolved.doo_counter = 0
    }

    if (msg.laps !== undefined) {
      resolved.laps = msg.laps;
    }
    else {
      resolved.laps = []
    }

    return resolved;
    }
};

module.exports = ExtraInfo;
