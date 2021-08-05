% Set the parameters in this file which have the "Determined by student" 
% comment, to tune the Kalman filter. 
% Do not modify anything else in this file.

global Pinit Qgamma Qbeta mahaThreshold ;

% Uncertainty on initial position of the robot.

sigmaX     = 5        ;  % Determined by student
sigmaY     = 5        ;  % Determined by student
sigmaTheta = 5*pi/180        ;  % Determined by student
Pinit = diag( [sigmaX^2 sigmaY^2 sigmaTheta^2] ) ;


% Measurement noise.

sigmaXmeasurement = 5 ;  % Determined by student
sigmaYmeasurement = 5 ;  % Determined by student
Qgamma = diag( [sigmaXmeasurement^2 sigmaYmeasurement^2] ) ;


% Input noise

sigmaWheels = 0.08 ;  % Determined by student result sigmaWheels = 0.08 
Qwheels = sigmaWheels^2 * eye(2) ;
Qbeta   = jointToCartesian * Qwheels * jointToCartesian.' ; 

% State noise
 
Qalpha = zeros(3) ;

% Mahalanobis distance threshold

mahaThreshold = chi2inv(0.9,2) ;  % Determined by student