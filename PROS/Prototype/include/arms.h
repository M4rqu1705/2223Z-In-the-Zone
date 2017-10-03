#ifndef ARMS_H_
#define ARMS_H_

enum armsPositions {d, u, lr, ll};

armsPositions armsPosition;

bool armsButtonPressed, armsLoaderButtonPressed, armsDone;

void armsOperatorControl();

void armsControl(armsPositions state);

#endif
