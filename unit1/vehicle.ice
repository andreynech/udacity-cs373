/* Copyright (c) 2010 Andrey Nechypurenko
   See the file LICENSE for copying permission. 
*/

#ifndef __VEHICLE_ICE
#define __VEHICLE_ICE

#include "comtypes.ice"
#include "vehicleadmin.ice"


module vehicle
{

  struct ActuatorData
  {
    short id; // which actuator
    short duty; // in percent: 0-100%
  };

  sequence<ActuatorData> ActuatorFrame;

  // Here we assume that Ice is smart enough to not introduce an
  // overhead if zero-length sequence should be transfered.
  // Typically only one member will be used for particular sensor.
  struct SensorData
  {
    short sensorid; // which sensor
    comtypes::ByteSeq bytedata;
    comtypes::ShortSeq shortdata;
    comtypes::IntSeq intdata;
    comtypes::LongSeq longdata;
    comtypes::FloatSeq floatdata;
  };

  sequence<SensorData> SensorFrame;

  // Callback interface to receive sensor data
  interface SensorFrameReceiver
  {
    ["ami"] void nextSensorFrame(SensorFrame frame);
  };

  interface RemoteVehicle
  {
    idempotent string getVehicleDescription();
    idempotent vehicleadmin::SensorList getSensorList();
    idempotent vehicleadmin::ActuatorList getActuatorList();
    
    idempotent void setActuators(ActuatorFrame duties);
    idempotent void setSensorReceiver(SensorFrameReceiver *callback);
    idempotent void cleanSensorReceiver();
  };

};


#endif // __VEHICLE_ICE
