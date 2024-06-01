function result = probCalc(leadDronePos,leadDroneCommRange,DronePos,DroneProbRange)
C1 = leadDronePos;
C2 = DronePos;
r = DroneProbRange;
R = leadDroneCommRange;

d = sqrt(sum((C1-C2').^2));

if (d < R + r)
    if (d <= abs(R - r))
        result = 1;
        return;
    end
    intersectionArea = R^2*acos((R^2-r^2+d^2)/(2*d*R)) + r^2*acos((r^2-R^2+d^2)/(2*d*r))...
         -(1/2)*sqrt((R+r+d)*(R+d-r)*(r-R+d)*(R+r-d));
     overAllDroneArea = pi*r^2;
     result = intersectionArea/overAllDroneArea;
else
    result = 0;
end
end