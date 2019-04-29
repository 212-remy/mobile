// Auto-generated. Do not edit!

// (in-package me212bot.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------

class mobile_step {
  constructor() {
    this.step = 0;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type mobile_step
    // Serialize message field [step]
    bufferInfo = _serializer.int32(obj.step, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type mobile_step
    let tmp;
    let len;
    let data = new mobile_step();
    // Deserialize message field [step]
    tmp = _deserializer.int32(buffer);
    data.step = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'me212bot/mobile_step';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '99174260c0c07917ce2b7a46302ab7a8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 step
    
    `;
  }

};

module.exports = mobile_step;
