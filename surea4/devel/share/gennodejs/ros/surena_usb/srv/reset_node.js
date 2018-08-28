// Auto-generated. Do not edit!

// (in-package surena_usb.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class reset_nodeRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.nodeID = null;
    }
    else {
      if (initObj.hasOwnProperty('nodeID')) {
        this.nodeID = initObj.nodeID
      }
      else {
        this.nodeID = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type reset_nodeRequest
    // Serialize message field [nodeID]
    bufferOffset = _serializer.int32(obj.nodeID, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type reset_nodeRequest
    let len;
    let data = new reset_nodeRequest(null);
    // Deserialize message field [nodeID]
    data.nodeID = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'surena_usb/reset_nodeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '83f1d0e77402d46c26211c4c64e0f71c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 nodeID
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new reset_nodeRequest(null);
    if (msg.nodeID !== undefined) {
      resolved.nodeID = msg.nodeID;
    }
    else {
      resolved.nodeID = 0
    }

    return resolved;
    }
};

class reset_nodeResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.result = null;
    }
    else {
      if (initObj.hasOwnProperty('result')) {
        this.result = initObj.result
      }
      else {
        this.result = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type reset_nodeResponse
    // Serialize message field [result]
    bufferOffset = _serializer.int32(obj.result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type reset_nodeResponse
    let len;
    let data = new reset_nodeResponse(null);
    // Deserialize message field [result]
    data.result = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'surena_usb/reset_nodeResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '034a8e20d6a306665e3a5b340fab3f09';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 result
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new reset_nodeResponse(null);
    if (msg.result !== undefined) {
      resolved.result = msg.result;
    }
    else {
      resolved.result = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: reset_nodeRequest,
  Response: reset_nodeResponse,
  md5sum() { return '0c471dfb6b81ef71ddb24300538963e0'; },
  datatype() { return 'surena_usb/reset_node'; }
};
