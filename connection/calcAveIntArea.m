function area = calcAveIntArea(Drones,r)
C1 = Drones(1).C;
C2 = Drones(2).C;
R1 = Drones(1).R;
R2 = Drones(2).R;

d = sqrt(sum((C1-C2).^2));
syms x
g = (2*R1^2.*acos((R1^2-R2^2+x.^2)./(2*x*R1))+ 2*R2^2.*acos((R2^2-R1^2+x.^2)./(2*x*R2))...
    -sqrt((R2 + R1 - x).*(R2 + x - R1).*(R1 + R2 + x).*(R1 + x - R2))/2);
f = 2*x.*acos((x.^2-r^2+d^2)./(2*d.*x))/(pi*r^2);
if(d-r > R1 - R2)
    F = int(g*f,x,d-r,min(d+r,R1+R2));
    area = double(vpa(F));
else
    F = int(g*f,x,R1-R2,min(d+r,R1+R2)) + int(pi*R2^2*f,x,d-r,R1-R2);
    area = double(vpa(F));
end
end

% fun1 = @(x) (2*R1^2.*acos((R1^2-R2^2+x.^2)./(2*x*R1))+ 2*R2^2.*acos((R2^2-R1^2+x.^2)./(2*x*R2))...
%     -sqrt((R2 + R1 - x).*(R2 + x - R1).*(R1 + R2 + x).*(R1 + x - R2))/2)*2.*acos((x.^2-r^2+d^2)./(2*d.*x)).*x;
% area = integral(fun1,d-r,min(d+r,R1+R2))/(pi*r^2);