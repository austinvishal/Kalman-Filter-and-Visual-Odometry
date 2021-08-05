% Constants defining the robot and sensor setup
% All lengths are in mm.

global rwheel trackGauge encoderRes ;
global sensorRes nbReedSensors noMagnetDetected magnetDetected ;
global sensorOrient sensorAboveYm sensorPosAlongXm ;
global xSpacing ySpacing ;
global dots2rad jointToCartesian cartesianToJoint ;

% Robot characteristics

rwheel     = 21.5 ;          % Wheel radius
trackGauge = 112  ;          % Distance between the fixed wheels
encoderRes = 360  ;          % In dots per wheel rotation

% The sensor is supposed to be orthogonal to axis Xm of the robot.

sensorRes        = 10   ;         % Distance between to Reed sensors
nbReedSensors    =  8   ;         % Nb of Reed sensors
sensorOrient     = +1   ;         % +1 if sensor + axis is Ym, -1 otherwise
sensorAboveYm    =  4.5 ;         % Number of the Reed sensor above Ym.
sensorPosAlongXm = 80   ;         % Position of linear sensor along Xm
noMagnetDetected =  1   ;         % Bit value when no magnet is detected
magnetDetected   =  0   ;         % Bit value when a magnet is detected

% Array of magnets in the ground   

xSpacing = 55 ;
ySpacing = 55 ;


% ---------------------------------------------------------------
% The following are calculated from previous data. Do not modify.
% ---------------------------------------------------------------

dots2rad = (2*pi)/encoderRes ;

jointToCartesian = [ rwheel/2           rwheel/2          ;
                     rwheel/trackGauge -rwheel/trackGauge ] ;
                
cartesianToJoint = inv(jointToCartesian) ;                