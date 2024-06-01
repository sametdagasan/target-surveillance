clear;
close all;
%% d + r < R1 + R2 should be satisfied
R0 = 100; % leader drone communication radius
C0 = [0,0]; % leader drone center
R1 = 60; % drone communication radius
C1 = [72,96]; % drone center
d = sqrt(sum((C1-C0).^2));
r = 30; %(small drone flight range)
Drones(1).R = R0;
Drones(1).C = C0;
Drones(2).R = R1;
Drones(2).C = C1;

intersectionArea = calcArea(Drones);
aveIntersectionArea = calcAveIntArea(Drones,r);
centers = [C0;C1];
figure
viscircles(centers,[R0,R1],'Color','b')
hold on
viscircles(C1,r,'LineStyle','--')
plot(centers(:,1),centers(:,2),'o')
title('Drones with communication range')
 xlim([-200,200])
 ylim([-200,200])
figure
step = 0.01;
x = d-r:step:d+r;
PDF = 2*x.*acos((x.^2-r^2+d^2)./(2*d.*x))/(pi*r^2);
plot(x,PDF)
title('PDF of Intersection')
checkPDF = sum(PDF)*step;
