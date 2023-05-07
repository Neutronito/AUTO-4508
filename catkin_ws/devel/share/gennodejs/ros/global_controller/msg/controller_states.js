// Auto-generated. Do not edit!

// (in-package global_controller.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class controller_states {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.is_driving_automatically = null;
      this.is_allowed_to_drive = null;
      this.tiz_is_boosted = null;
    }
    else {
      if (initObj.hasOwnProperty('is_driving_automatically')) {
        this.is_driving_automatically = initObj.is_driving_automatically
      }
      else {
        this.is_driving_automatically = false;
      }
      if (initObj.hasOwnProperty('is_allowed_to_drive')) {
        this.is_allowed_to_drive = initObj.is_allowed_to_drive
      }
      else {
        this.is_allowed_to_drive = false;
      }
      if (initObj.hasOwnProperty('tiz_is_boosted')) {
        this.tiz_is_boosted = initObj.tiz_is_boosted
      }
      else {
        this.tiz_is_boosted = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type controller_states
    // Serialize message field [is_driving_automatically]
    bufferOffset = _serializer.bool(obj.is_driving_automatically, buffer, bufferOffset);
    // Serialize message field [is_allowed_to_drive]
    bufferOffset = _serializer.bool(obj.is_allowed_to_drive, buffer, bufferOffset);
    // Serialize message field [tiz_is_boosted]
    bufferOffset = _serializer.bool(obj.tiz_is_boosted, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type controller_states
    let len;
    let data = new controller_states(null);
    // Deserialize message field [is_driving_automatically]
    data.is_driving_automatically = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [is_allowed_to_drive]
    data.is_allowed_to_drive = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [tiz_is_boosted]
    data.tiz_is_boosted = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 3;
  }

  static datatype() {
    // Returns string type for a message object
    return 'global_controller/controller_states';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '656034952bcd2a7d6b51db3480a3adeb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool is_driving_automatically
    bool is_allowed_to_drive
    bool tiz_is_boosted
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new controller_states(null);
    if (msg.is_driving_automatically !== undefined) {
      resolved.is_driving_automatically = msg.is_driving_automatically;
    }
    else {
      resolved.is_driving_automatically = false
    }

    if (msg.is_allowed_to_drive !== undefined) {
      resolved.is_allowed_to_drive = msg.is_allowed_to_drive;
    }
    else {
      resolved.is_allowed_to_drive = false
    }

    if (msg.tiz_is_boosted !== undefined) {
      resolved.tiz_is_boosted = msg.tiz_is_boosted;
    }
    else {
      resolved.tiz_is_boosted = false
    }

    return resolved;
    }
};

module.exports = controller_states;
