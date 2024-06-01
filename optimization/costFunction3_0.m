function totalCost = costFunction3_0(leadDronePos,leadDroneCommRange,DronesPos,DronesProbRange,D,links,results)
% minResult = Inf;
epsilon = 1e-12;

distsToLead = sqrt(sum((leadDronePos-DronesPos').^2));
dists(1) = sqrt(sum((DronesPos(2,:)-DronesPos(3,:)).^2));
dists(2) = sqrt(sum((DronesPos(1,:)-DronesPos(3,:)).^2));
dists(3) = sqrt(sum((DronesPos(1,:)-DronesPos(2,:)).^2));
[~,minIndex] = min(dists);

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
% totalCost = connProb - epsilon*max(distsToLead);
% totalCost = connProb - epsilon*distsToLead(minIndex);
% totalCost = connProb - epsilon*(distsToLead(minIndex) + max(distsToLead));
% if connProb > 0
%     totalCost = connProb - epsilon*distsToLead(minIndex);
% else  
%     totalCost = connProb - epsilon*max(distsToLead);
% end
% Closest Drones minimum distance, Lonely one  
loneDrone = distsToLead(minIndex);
distsToLead(minIndex) = [];
totalCost = connProb - epsilon*max([loneDrone,min(distsToLead)]);

% Closest Centers vs Lonely one
% loneDrone = distsToLead(minIndex);
% loneDronePos = DronesPos(minIndex,:);
% 
% DronesPos(minIndex,:) = [];
% distsToLead(minIndex) = [];
% centerClosestDrones = mean(DronesPos);
% distToCenter = sqrt(sum((leadDronePos-centerClosestDrones').^2));
% 
%  totalCost = connProb - epsilon*max([loneDrone,distToCenter]);



end
% function aveDist = costFunction2(leadDronePos,DronesPos)
%  aveDist = mean(sqrt(sum((leadDronePos-DronesPos').^2)));
% end