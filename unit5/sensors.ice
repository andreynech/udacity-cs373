/* Copyright (c) 2012 Andrey Nechypurenko
   See the file LICENSE for copying permission. 
*/

#ifndef __SENSORS_ICE
#define __SENSORS_ICE

#include <Ice/BuiltinSequences.ice>
#include "admin.ice"


module sensors
{

  // Sensor data container. It is a kind of union, i.e. typically,
  // only one member (plus id) will be used for particular sensor.
  // We decide to avoid class-based approach to "simulate" union
  // behavior and assume that Ice is smart enough to not introduce an
  // overhead if zero-length sequence should be transfered.
  struct SensorData
  {
    // which sensor
    short sensorid;
    Ice::ByteSeq bytedata;
    Ice::ShortSeq shortdata;
    Ice::IntSeq intdata;
    Ice::LongSeq longdata;
    Ice::FloatSeq floatdata;
  };

  sequence<SensorData> SensorFrame;

  enum SensorType
  {
    Unknown,
    Camera,
    StereoCamera,
    Compass,
    Accelerometer,
    Gyro,
    GPS,
    Range,
    Temperature,
    Pressure,
    Joystick,
    Keyboard
  };

  struct SensorDescription
  {
    // sensor unique (for the group) id
    short id;
    SensorType type;
    // data range
    // both values should be 0 if range is undefined
    float minvalue;
    float maxvalue;
    // recommended sensor refresh rate
    float refreshrate;
    // for example compass or gyroscope
    string description;
    // model id
    string vendorid;
  };
  
  sequence<SensorDescription> SensorDescriptionSeq;

  // Callback interface to receive sensor data
  // for push-model communication.
  interface SensorFrameReceiver
  {
    ["ami"] void nextSensorFrame(SensorFrame frame);
  };
  
  // Represents one or multiple sensors
  interface SensorGroup
  {
    idempotent admin::State* getStateInterface();
    idempotent SensorDescriptionSeq getSensorDescription();

    // Support for pull-model
    idempotent SensorFrame getCurrentValues();
	
    // Support for push-model.
    // Returns: true if success, false if push-model is not 
    //          supported or any other error happened.
    idempotent bool setSensorReceiver(SensorFrameReceiver *callback);
    idempotent void cleanSensorReceiver();
  };

};


#endif // __SENSORS_ICE
