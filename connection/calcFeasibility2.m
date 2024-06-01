% clear;
N = 5121;
feasibility = zeros(20,4,N);
leadDroneCommRange = [50e3 100e3 150e3 200e3];
for posIter = 1:20
    clear x_true
    filename = ['dronePos',num2str(posIter),'.mat'];
    load(['DroneDataset\' filename])
    DronesPos = x_true(:,1:2,:);
    for rangeIter = 1:4
        DroneCommRange = leadDroneCommRange(rangeIter); % lead Drone communication range 50 km        
        for i = 1:N
            DronePos = DronesPos(:,:,i);
            dists(1) = sqrt(sum((DronePos(1,:)-DronePos(2,:)).^2));
            dists(2) = sqrt(sum((DronePos(1,:)-DronePos(3,:)).^2));
            dists(3) = sqrt(sum((DronePos(2,:)-DronePos(3,:)).^2));
            dists = sort(dists);
            if dists(1) <= DroneCommRange && dists(2) <= DroneCommRange
                feasibility(posIter,rangeIter,i) = feasibility(posIter,rangeIter,i) + 1;
            end
            if dists(1) <= DroneCommRange && DroneCommRange < dists(2) && dists(2) <= 2*DroneCommRange
                feasibility(posIter,rangeIter,i) = feasibility(posIter,rangeIter,i) + 1;
            end
            Points1 = calcIntPoints(DronePos(1,:),DronePos(2,:),DroneCommRange);
            if ~isempty(Points1)
                feasibility(posIter,rangeIter,i) = feasibility(posIter,rangeIter,i) + ((DronePos(3,1) - Points1(1,1))^2 + (DronePos(3,2) - Points1(1,2))^2 < DroneCommRange^2);
                feasibility(posIter,rangeIter,i) = feasibility(posIter,rangeIter,i) + ((DronePos(3,1) - Points1(2,1))^2 + (DronePos(3,2) - Points1(2,2))^2 < DroneCommRange^2);
            end
            
            Points2 = calcIntPoints(DronePos(1,:),DronePos(3,:),DroneCommRange);
            if ~isempty(Points2)
                feasibility(posIter,rangeIter,i) = feasibility(posIter,rangeIter,i) + ((DronePos(2,1) - Points2(1,1))^2 + (DronePos(2,2) - Points2(1,2))^2 < DroneCommRange^2);
                feasibility(posIter,rangeIter,i) = feasibility(posIter,rangeIter,i) + ((DronePos(2,1) - Points2(2,1))^2 + (DronePos(2,2) - Points2(2,2))^2 < DroneCommRange^2);
            end
            
            Points3 = calcIntPoints(DronePos(2,:),DronePos(3,:),DroneCommRange);
            if ~isempty(Points3)
                feasibility(posIter,rangeIter,i) = feasibility(posIter,rangeIter,i) + ((DronePos(1,1) - Points3(1,1))^2 + (DronePos(1,2) - Points3(1,2))^2 < DroneCommRange^2);
                feasibility(posIter,rangeIter,i) = feasibility(posIter,rangeIter,i) + ((DronePos(1,1) - Points3(2,1))^2 + (DronePos(1,2) - Points3(2,2))^2 < DroneCommRange^2);
            end
              feasibility(posIter,rangeIter,i) = feasibility(posIter,rangeIter,i) > 0;
        end   
    end
end
feasibilityTable2 = sum(feasibility,3);