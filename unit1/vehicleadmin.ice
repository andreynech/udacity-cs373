/* Copyright (c) 2010 Andrey Nechypurenko
   See the file LICENSE for copying permission. 
*/

#ifndef __VEHICLEADMIN_ICE
#define __VEHICLEADMIN_ICE

#include "comtypes.ice"


module vehicleadmin
{

  interface Admin
  {
    idempotent void start();
    idempotent void stop();
  };

  enum SensorType
  {
    Unknown,
    Camera,
    StereoCamera,
    Compass,
    AccelerometerX,
    AccelerometerY,
    AccelerometerZ,
    GyroX,
    GyroY,
    GyroZ,
    GPS,
    Range,
    Temperature,
    Pressure
  };

  struct SensorDescription
  {
    short id; // sensor unique (for the vehicle) id
    string description; // for example compass or gyroscope
    string vendorid;        // model id
    SensorType type;
    float minvalue; // data range
    float maxvalue; // both values should be 0 if range is undefined
  };

  struct ActuatorDescription
  {
    short id; // actuator unique (for the vehicle) id
    string description; // for example compass or gyroscope
    string type;        // model id
  };

  // Typedefs for calibration data.
  // We assume that calibration data is the list of matricies. 
  // Single values could be presented as a sequence with one element.
  // String is the name of the calibration matrix
  dictionary<string, comtypes::FloatSeq> CalibrationData;

  interface Sensor
  {
    idempotent Admin* getAdminInterface();
    idempotent SensorDescription getDescription();
    idempotent CalibrationData getCalibrationData();
    idempotent void setCalibrationData(CalibrationData d);
  };

  sequence<Sensor*> SensorList;

  interface Actuator
  {
    idempotent Admin* getAdminInterface();
    idempotent ActuatorDescription getDescription();
  };

  sequence<Actuator*> ActuatorList;

}; 

#endif // __VEHICLEADMIN_ICE
