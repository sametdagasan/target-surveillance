function totalCost = costFunction3(leadDronePos,leadDroneCommRange,DronesPos,DronesProbRange,D,links,results)
% minResult = Inf;
epsilon = 1e-12;
dists = sqrt(sum((leadDronePos-DronesPos').^2));
% dists(4) = sqrt(sum((DronesPos(1,:)-DronesPos(2,:)).^2));
% dists(5) = sqrt(sum((DronesPos(1,:)-DronesPos(3,:)).^2));
% dists(6) = sqrt(sum((DronesPos(2,:)-DronesPos(3,:)).^2));
probs = zeros(6,1);
for i = 1:D
   probs(i) =  probCalc(leadDronePos,leadDroneCommRange,DronesPos(i,:),DronesProbRange(i));
end
probs(4) = probCalc(DronesPos(1,:)',leadDroneCommRange,DronesPos(2,:),DronesProbRange(2));
probs(5) = probCalc(DronesPos(1,:)',leadDroneCommRange,DronesPos(3,:),DronesProbRange(3));
probs(6) = probCalc(DronesPos(2,:)',leadDroneCommRange,DronesPos(3,:),DronesProbRange(3));

res = zeros(64,1);
connProb = 0;
for i = 1:size(links,1)
    if results(i) == 1
        res(i) = prod(probs(links(i,:) ~= 0))*prod(1-probs(links(i,:) ~= 1));
        connProb = connProb + res(i);
    end
end
totalCost = connProb - epsilon*max(dists);

end
% function aveDist = costFunction2(leadDronePos,DronesPos)
%  aveDist = mean(sqrt(sum((leadDronePos-DronesPos').^2)));
% end