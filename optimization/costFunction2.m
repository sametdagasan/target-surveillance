function minResult = costFunction2(leadDronePos,leadDroneCommRange,DronesPos,DronesProbRange,D)
minResult = Inf;
epsilon = 1e-12;
dists = sqrt(sum((leadDronePos-DronesPos').^2));
for i = 1:D
   result =  probCalc(leadDronePos,leadDroneCommRange,DronesPos(i,:),DronesProbRange(i)) - epsilon*max(dists);
   if result < minResult
       minResult = result;
   end
end 
end
% function aveDist = costFunction2(leadDronePos,DronesPos)
%  aveDist = mean(sqrt(sum((leadDronePos-DronesPos').^2)));
% end