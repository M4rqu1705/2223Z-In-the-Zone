#ifndef MAIN_H_
#define MAIN_H_

#include <API.h>
#include "utils.h"

namespace drive{
  // extern short motorOutputs[2];
  // extern bool toggleDirectionButton, normalDirection;

  // extern void normalEqualsNotDriveNormal();
  void normalEqualsNotDriveNormal();
}

namespace mobileGoal{
  // extern signed char motorOutputs[2];
  // extern float PID[7];
  // extern bool toggleRetractButton, toggleExtendButton, retract;

  // extern void retractEqualsTrue();
  // extern void retractEqualsFalse();
  void retractEqualsTrue();
  void retractEqualsFalse();
}

#ifdef __cplusplus
extern "C" {
  void __libc_init_array();
#endif


void autonomous();

void initializeIO();

void initialize();

void operatorControl();


#ifdef __cplusplus
}
#endif

#endif
