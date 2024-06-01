function area = calcArea(Drones)
C1 = Drones(1).C;
C2 = Drones(2).C;
R1 = Drones(1).R;
R2 = Drones(2).R;

d = sqrt(sum((C1-C2).^2));

if (d < R1 + R2)
    if (d <= abs(R1 - R2))
        area = pi*min(R1,R2)^2;
        return;
    end
    x = (R1^2 - R2^2 + d^2) / (2*d);
    y = sqrt(R1^2 - x^2);
    area = R1^2*asin(y/R1) + R2^2*asin(y/R2) - d*y;
else 
    area = 0;
end
end