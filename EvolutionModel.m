% Implements the evolution model of the system. Here, this model is simply
% the equations of odometry.

function Xnew = EvolutionModel( Xold , U )

% Recall X=(x,y,theta)' and U=(deltaD,deltaTheta)'

Xnew = Xold + [ U(1)*cos(Xold(3)) ;
                U(1)*sin(Xold(3)) ;
                U(2)              ;
              ] ;
          
return          