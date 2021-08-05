% This function stores information for later analysis and display. It also
% calculates data related to the magnets located near the one used to
% update the posture. It has no role in the actual localization process.

function CalculateAndStoreResultsForAnalysis( phase ) 

global indRes indMagnetDetection mainLoopIndex ;
global t tRes tMagnetDetection  X P U Y Qbeta Xodom oTm mTo ;
global Xodo vOdo wOdo travDistance Xres sigx sigy sigtheta ;
global update dMahaAll dMaha nbMagnetsDetected ;
global measures estMagnetPos oPest exactMagnetPos oPmagnet ;
global xSpacing ySpacing ;

if strcmp( phase , 'prediction' ) ,

    tRes(indRes)     = t(mainLoopIndex)          ;
    Xres(:,indRes)   = X                         ; 
    sigx(indRes)     = sqrt( P(1,1) )            ;
    sigy(indRes)     = sqrt( P(2,2) )            ;
    sigtheta(indRes) = sqrt( P(3,3) ) * 180/pi   ;
    indRes           = indRes+1                  ;
    
    nbMagnetsDetected(mainLoopIndex) = numel(measures) ;  
    
    % Maintain an odometry-only estimation for comparison
    % mainLoopIndex is the "for" loop index of main program: don't update.
    Xodom = EvolutionModel( Xodom , U ) ;
    Xodo(:,mainLoopIndex) = Xodom ;
    vOdo(mainLoopIndex)   = U(1)/(t(mainLoopIndex)-t(mainLoopIndex-1)) ;
    wOdo(mainLoopIndex)   = U(2)/(t(mainLoopIndex)-t(mainLoopIndex-1)) ;
    travDistance = travDistance + U(1) ;
    
elseif strcmp( phase , 'measurement' ) ,

    % Those are stored for each detected magnet. 
    % Unknown number, so they are allocated dynamically from the start.

    % tMagnetDetection (1*n): time of each magnet detection.
    % dMahaAll (5*n): Mahalanobis distance of closest magnet and neighbors
    % estMagnetPos (2*n): Est. magnet position using measurement and robot posture.
    % exactMagnetPos (2*n): Exact magnet position closest to estimated magnet.

    dMahaAll(1,indMagnetDetection)       = dMaha            ;
    estMagnetPos(:,indMagnetDetection)   = oPest            ;
    exactMagnetPos(:,indMagnetDetection) = oPmagnet         ;
    tMagnetDetection(indMagnetDetection) = t(mainLoopIndex) ;

    % Offset vectors to generate the neighbors, in homogeneous coordinates.

    deltas = [ xSpacing  -xSpacing      0           0       ;
                  0          0       ySpacing   -ySpacing   ;
                  0          0          0           0       ] ;

    for neighborIndex = 1 : 4 ,
        oPneighbor = oPmagnet + deltas(:,neighborIndex) ;

        % The position of the magnet in robot frame is the expected measurement
        % YhatNeighbor
        YhatNeighbor = mTo * oPneighbor ;

        Cneighbor = [ -cos(X(3)) -sin(X(3)) -sin(X(3))*(oPneighbor(1)-X(1))+cos(X(3))*(oPneighbor(2)-X(2)) ;
                       sin(X(3)) -cos(X(3)) -sin(X(3))*(oPneighbor(2)-X(2))-cos(X(3))*(oPneighbor(1)-X(1)) ] ;

        innovNeighbor = Y(1:2) - YhatNeighbor(1:2) ;    % Not in homogeneous coordinates.
        dMahaNeighbor = sqrt( innovNeighbor.' * inv( Cneighbor*P*Cneighbor.' + Qbeta) * innovNeighbor ) ;
        dMahaAll(neighborIndex+1,indMagnetDetection) = dMahaNeighbor ;
    end

    tRes(indRes) = t(mainLoopIndex) ;
    Xres(:,indRes) = X ; 
    sigx(indRes) = sqrt( P(1,1) ) ;
    sigy(indRes) = sqrt( P(2,2) ) ;
    sigtheta(indRes) = sqrt( P(3,3) ) * 180/pi ;
    indRes = indRes+1 ;

    indMagnetDetection = indMagnetDetection + 1 ;
    
elseif strcmp( phase , 'update' ) ,
        
    tRes(indRes)     = t(mainLoopIndex)        ;
    Xres(:,indRes)   = X                       ; 
    update(indRes)   = 1                       ;
    sigx(indRes)     = sqrt( P(1,1) )          ;
    sigy(indRes)     = sqrt( P(2,2) )          ;
    sigtheta(indRes) = sqrt( P(3,3) ) * 180/pi ;
    indRes           = indRes+1                ;
    
else
    disp( 'Data recording: incorrect phase specification.');
    return
end