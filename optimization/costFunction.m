function minResult = costFunction(leadDronePos,leadDroneCommRange,DronesPos,DronesProbRange,D)
minResult = 1;
% epsilon = 1e-6;
for i = 1:D
   result =  probCalc(leadDronePos,leadDroneCommRange,DronesPos(i,:),DronesProbRange(i));
   if result < minResult
       minResult = result;
   end
end 
end