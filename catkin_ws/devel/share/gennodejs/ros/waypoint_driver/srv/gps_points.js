// Auto-generated. Do not edit!

// (in-package waypoint_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class gps_pointsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.latitude = null;
      this.longitude = null;
    }
    else {
      if (initObj.hasOwnProperty('latitude')) {
        this.latitude = initObj.latitude
      }
      else {
        this.latitude = 0.0;
      }
      if (initObj.hasOwnProperty('longitude')) {
        this.longitude = initObj.longitude
      }
      else {
        this.longitude = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gps_pointsRequest
    // Serialize message field [latitude]
    bufferOffset = _serializer.float64(obj.latitude, buffer, bufferOffset);
    // Serialize message field [longitude]
    bufferOffset = _serializer.float64(obj.longitude, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gps_pointsRequest
    let len;
    let data = new gps_pointsRequest(null);
    // Deserialize message field [latitude]
    data.latitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [longitude]
    data.longitude = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'waypoint_driver/gps_pointsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '680c6dc7da65a2421a822205dcbdb600';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 latitude
    float64 longitude
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new gps_pointsRequest(null);
    if (msg.latitude !== undefined) {
      resolved.latitude = msg.latitude;
    }
    else {
      resolved.latitude = 0.0
    }

    if (msg.longitude !== undefined) {
      resolved.longitude = msg.longitude;
    }
    else {
      resolved.longitude = 0.0
    }

    return resolved;
    }
};

class gps_pointsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.reached_status = null;
    }
    else {
      if (initObj.hasOwnProperty('reached_status')) {
        this.reached_status = initObj.reached_status
      }
      else {
        this.reached_status = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gps_pointsResponse
    // Serialize message field [reached_status]
    bufferOffset = _serializer.bool(obj.reached_status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gps_pointsResponse
    let len;
    let data = new gps_pointsResponse(null);
    // Deserialize message field [reached_status]
    data.reached_status = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'waypoint_driver/gps_pointsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2a7f193d6d3387641f8cb3e2fb24e17d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool reached_status
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new gps_pointsResponse(null);
    if (msg.reached_status !== undefined) {
      resolved.reached_status = msg.reached_status;
    }
    else {
      resolved.reached_status = false
    }

    return resolved;
    }
};

module.exports = {
  Request: gps_pointsRequest,
  Response: gps_pointsResponse,
  md5sum() { return 'b5bec956a12980a00e59b3e48c455bdc'; },
  datatype() { return 'waypoint_driver/gps_points'; }
};
