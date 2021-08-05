% Due to the manipulations required by the recording software, there is
% usually at the beginning and at the end of the data, time intervals where
% the robot is motionless. This function determines the start and stop
% indices of the meaningful part of the data, allowing to ignore these two
% phases.

function [nbLoops,t,qL,qR,sensorReadings] = SkipInitialAndFinalMotionlessParts(data)

global dots2rad

% Skip motionless initial part if any
nbSamples = size(data,1) ;
i = 1 ;
while (i<nbSamples) && (data(i,1)-data(i+1,1)==0) && (data(i,2)-data(i+1,2)==0) ,
    i = i+1 ;
end ;
startIndex = i ;

% Skip motionless final part if any
i = nbSamples ;
while (i>1) && (data(i,1)-data(i-1,1)==0) && (data(i,2)-data(i-1,2)==0) ,
    i = i-1 ;
end
stopIndex = i ;

% Extract relevant data by removing motionless parts).
t0 = data(startIndex,4) ;
qL = data(startIndex:stopIndex,1).'*dots2rad ;      % left wheel position in radians
qR = data(startIndex:stopIndex,2).'*dots2rad ;      % right wheel position in radians 
t  = data(startIndex:stopIndex,4).' - t0     ;      % time instants of each period.
sensorReadings = data(startIndex:stopIndex,3).' ;   % Raw sensor data

nbLoops = numel(t) ;

return