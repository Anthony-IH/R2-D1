% desired coordinates in frame 0
P0x = %desired x coordinate;
P0y = % desired y coordinate;

% TODO initialise geometry of 2-arm robotic system
r1 = 120;
r2 = 92;


%inverse kinematics to calculate angles t2 and t1 from P0x and P0y

t2=acosd((P0x^2+P0y^2-r1^2-r2^2)/(2*r1*r2))

t1=atand(P0y/P0x)-atand((r2*sind(t2))/(r1+r2*cosd(t2)))