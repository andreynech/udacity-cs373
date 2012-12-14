/* Copyright (c) 2012 Andrey Nechypurenko
   See the file LICENSE for copying permission. 
*/

#ifndef __ACTUATORS_ICE
#define __ACTUATORS_ICE

#include "admin.ice"


module actuators
{
  struct ActuatorDescription
  {
    // Actuator unique (for the group) id
    short id;
    // For example camera servo
    string description;
    // Model id
    string vendorid;
  };
  
  sequence<ActuatorDescription> ActuatorDescriptionSeq;
  
  // Used to tell actuator to make the movement with
  // specified speed and distance.
  struct ActuatorData
  {
    // Which actuator
    short id;
    // In rotations per second. Rotate backwards if negative
    float speed;
    // In rotations
    float distance;
  };

  sequence<ActuatorData> ActuatorFrame;

  interface ActuatorGroup
  {
    idempotent admin::State* getStateInterface();
    idempotent ActuatorDescriptionSeq getActuatorDescription();
    
    void setActuatorsAndWait(ActuatorFrame duties);
    void setActuatorsNoWait(ActuatorFrame duties);
  };

};


#endif // __ACTUATORS_ICE
