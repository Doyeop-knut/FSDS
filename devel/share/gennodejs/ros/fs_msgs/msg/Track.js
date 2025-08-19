// Auto-generated. Do not edit!

// (in-package fs_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Cone = require('./Cone.js');

//-----------------------------------------------------------

class Track {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.track = null;
    }
    else {
      if (initObj.hasOwnProperty('track')) {
        this.track = initObj.track
      }
      else {
        this.track = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Track
    // Serialize message field [track]
    // Serialize the length for message field [track]
    bufferOffset = _serializer.uint32(obj.track.length, buffer, bufferOffset);
    obj.track.forEach((val) => {
      bufferOffset = Cone.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Track
    let len;
    let data = new Track(null);
    // Deserialize message field [track]
    // Deserialize array length for message field [track]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.track = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.track[i] = Cone.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 25 * object.track.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fs_msgs/Track';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '03d754d3091336842daac1a974511950';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    fs_msgs/Cone[] track
    ================================================================================
    MSG: fs_msgs/Cone
    geometry_msgs/Point location #x,y,z [m] wrt to the car start location (origin)
    uint8 color # use the enum below
    # Constants
    uint8 BLUE=0
    uint8 YELLOW=1
    uint8 ORANGE_BIG=2
    uint8 ORANGE_SMALL=3
    uint8 UNKNOWN=4
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Track(null);
    if (msg.track !== undefined) {
      resolved.track = new Array(msg.track.length);
      for (let i = 0; i < resolved.track.length; ++i) {
        resolved.track[i] = Cone.Resolve(msg.track[i]);
      }
    }
    else {
      resolved.track = []
    }

    return resolved;
    }
};

module.exports = Track;
