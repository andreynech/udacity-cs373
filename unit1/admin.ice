/* Copyright (c) 2012 Andrey Nechypurenko
   See the file LICENSE for copying permission. 
*/

#ifndef __ADMIN_ICE
#define __ADMIN_ICE


module admin
{
  interface State
  {
    idempotent void start();
    idempotent void stop();
  };

}; 

#endif // __ADMIN_ICE
