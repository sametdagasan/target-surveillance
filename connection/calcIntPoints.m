function intersectPoints = calcIntPoints(DronePos1,DronePos2,CommRange)
a = DronePos1(1);
b = DronePos1(2);
c = DronePos2(1);
d = DronePos2(2);
R = CommRange;
D = sqrt((c-a)^2 + (d-b)^2);

if 2*R < D
    intersectPoints = [];
    return
else
    %Point 1 X coordinate
    intersectPoints(1,1) = (a+c)/2 + (b-d)*sqrt(4*R^2 - D^2)/(2*D);
    %Point 1 Y coordinate
    intersectPoints(1,2) = (b+d)/2 - (a-c)*sqrt(4*R^2 - D^2)/(2*D); 
    %Point 2 X coordinate
    intersectPoints(2,1) = (a+c)/2 - (b-d)*sqrt(4*R^2 - D^2)/(2*D); 
    %Point 2 Y coordinate
    intersectPoints(2,2) = (b+d)/2 + (a-c)*sqrt(4*R^2 - D^2)/(2*D); 
end
