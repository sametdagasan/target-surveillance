clear;
close all;
%% settings
N = 5120; % number of time steps
dt = 1; % time between time steps
D = 3; % number of drones
M = 100; % Number of Monte Carlo Simulations

sig_acc_true = [0.3; 0.3; 0.3]; % true value of standard deviation of accelerometer noise
sig_gps_true = [3; 3; 3; 0.03; 0.03; 0.03]; % true value of standard deviation of GPS noise

sig_err = 3;
sig_pos = 30;
sig_vel = 0.3;
sig_acc = sig_err* ones(3,1);  % user input of standard deviation of accelerometer noise
sig_gps = [sig_pos*ones(3,1);sig_vel*ones(3,1)]; % user input of standard deviation of GPS noise

Q = [diag(0.25*dt^4*sig_acc.^2), zeros(3); zeros(3), diag(dt^2*sig_acc.^2)]; % process noise covariance matrix
R = [diag(sig_gps(1:3).^2), zeros(3); zeros(3), diag(sig_gps(4:6).^2)]; % measurement noise covariance matrix

B = [0.5*eye(3)*dt^2; eye(3)*dt]; % control-input matrix
F = [eye(3), eye(3)*dt; zeros(3), eye(3)]; % state transition matrix
H = eye(6); % measurement matrix
% for d = 1:D
%% classification
% class_velocity = [80];
%% dataset loading
load('connections.mat');
%% parameters
leadDroneCommRange = [100e3]; % lead Drone communication range 50 km
algorithm = [0,1,2,3]; % 0 if single-hop, 1 nearest, 2 midpoint, 3 hybrid
optimization_step_size = [1,4]; % should be smaller than kalman_step_size
kalmanCalc = 0;
%% true trajectory
load('DroneDataset\dronePos15.mat');
leadDroneinitPos = mean(x_true(:,1:3,1))'; % lead Drone position
leadDroneinitVelocity = [50;50;0];
% leadDroneinitVel = sqrt(sum(leadDroneinitVelocity.^2));
leadDroneinitVel = [40;60;80];

x_lead = zeros(6,N+1);
x_lead(:,1) = [leadDroneinitPos;leadDroneinitVelocity];
%% Kalman filter simulation
if ~kalmanCalc
    load('DroneDataset\droneKalmanPos15.mat');
else
    disp('Kalman Filter Simulation Time')
    tic
    res_x_est = zeros(D,6,N+1,M); % Monte-Carlo estimates
    res_x_err = zeros(D,6,N+1,M); % Monte-Carlo estimate errors
    % filtering
    for m = 1:M
        % initial guess
        for d = 1:D
            x_est(d,:,1) = (x_true(d,:,1))' + [normrnd(0,ones(3,1));normrnd(0,0.1*ones(3,1))];
            P_d(d,:,:) = [eye(3)*4^2, zeros(3); zeros(3), eye(3)*0.4^2];
        end
        
        for k = 2:N+1
            %%% Prediction
            for d = 1:D
                % obtain acceleration output
                %         u = a_true(:,k-1) + normrnd(0, sig_acc_true);
                u = a_true(d,:,k-1)' + normrnd(0, sig_acc_true);
                % predicted state estimate
                x_est(d,:,k) = F*(x_est(d,:,k-1))' + B*u;
                
                % predicted error covariance
                P = reshape(P_d(d,:,:),[6,6]);
                P = F*P*F' + Q;
                
                %%% Update
                % obtain measurement
                z = (x_true(d,:,k))' + normrnd(0, sig_gps_true);
                
                % measurement residual
                y = z - H*(x_est(d,:,k))';
                
                % Kalman gain
                K = P*H'/(R+H*P*H');
                
                % updated state estimate
                x_est(d,:,k) = (x_est(d,:,k))' + K*y;
                
                % updated error covariance
                P_d(d,:,:) = (eye(6) - K*H)*P;
            end
        end
        res_x_est(:,:,:,m) = x_est;
        res_x_err(:,:,:,m) = x_est - x_true;
    end
    toc
    %% get result statistics
    x_est_avg = mean(res_x_est,4);
    x_err_avg = x_est - x_true;
    
    x_RMSE = zeros(D,6,N+1); % root mean square error
    x_RMSE_cumulative = zeros(D,6,N+1); % root mean square error
    
    for d = 1:D
        for k = 1:1:N+1
            x_RMSE_cumulative(d,:,k) = sqrt(mean(x_err_avg(d,:,1:k).^2,3));
            x_RMSE(d,:,k) = sqrt(mean(res_x_err(d,:,k,:).^2,4));
        end
    end    
end
x_lead_all = zeros([length(leadDroneCommRange),length(algorithm),length(optimization_step_size),size(x_lead)]);
for sim1 = 1:length(leadDroneCommRange)
    for sim2 = 1:length(algorithm)
        for sim3 = 1:length(optimization_step_size)
            %% optimization
            disp('Optimization time');
            tic
            kalman_step_size = 16;
            % optimization_step_size = 4; % should be smaller than kalman_step_size
            turn_angle = pi/6;
            num_of_optimization = N/kalman_step_size;
            old_velocity = leadDroneinitVelocity;
            temp_lead = x_lead;
            temp_lead_m = temp_lead;
            old_theta = pi/4;
            new_theta = old_theta;
            best_nodes = [];
            best_costs = 0;
            for i = 1:num_of_optimization
                dronesX = x_est_avg(:,:,((i-1)*kalman_step_size+1):(i*kalman_step_size)); %from kalman
                arr = (1:3^optimization_step_size(sim3))-1;
                arr3 = dec2base(arr,3);
                for l = 1:(kalman_step_size/optimization_step_size(sim3))
                    currX = dronesX(:,:,((l-1)*optimization_step_size(sim3)+1):(l*optimization_step_size(sim3)));
                    bestNode = [];
                    bestCost = -Inf;
                    bestDistCost = Inf;
                    % check each node to get the best
                    for j = 1:3^optimization_step_size(sim3)
                        temp_theta = new_theta;
                        currentNode = arr3(j,:);
                        cost = 0;
                        %             distCost = 0;
                        % lead_velocity = zeros(3,1,optimization_step_size);
                        for k = 1:optimization_step_size(sim3)
                            bestVcost = -Inf;
                            t = (i-1)*kalman_step_size+(l-1)*optimization_step_size(sim3) + k;
                            temp_theta = temp_theta + (str2double(currentNode(k))-1)*turn_angle;
                            for m = 1:length(leadDroneinitVel)
                                curr_vel_m = leadDroneinitVel(m)*[cos(temp_theta);sin(temp_theta);0];
                                temp_lead_m(:,t+1) = F*([temp_lead(1:3,t);curr_vel_m]);
                                if algorithm(sim2) == 0
                                    Vcost = costFunction2(temp_lead_m(1:3,t+1),leadDroneCommRange(sim1),currX(:,1:3,k),drone_probRange,D);
                                elseif algorithm(sim2) == 1
                                    Vcost = costFunction3_0(temp_lead_m(1:3,t+1),leadDroneCommRange(sim1),currX(:,1:3,k),drone_probRange,D,links,results);
                                elseif algorithm(sim2) == 2
                                    Vcost = costFunction3_5(temp_lead_m(1:3,t+1),leadDroneCommRange(sim1),currX(:,1:3,k),drone_probRange,D,links,results);                                    
                                elseif algorithm(sim2) == 3
                                     Vcost = costFunction2(temp_lead_m(1:3,t+1),leadDroneCommRange(sim1),currX(:,1:3,k),drone_probRange,D);
                                     if (Vcost < 0)
                                        Vcost = costFunction3_5(temp_lead_m(1:3,t+1),leadDroneCommRange(sim1),currX(:,1:3,k),drone_probRange,D,links,results);
                                     end
                                end
                                %                     distCost = distCost + mean(sqrt(sum((temp_lead(1:3,t+1)-currX(:,1:3,k)').^2)));
                                if Vcost > bestVcost
                                    bestVcost = Vcost;
                                    temp_lead(:,t+1) = F*([temp_lead(1:3,t);curr_vel_m]);
                                end
                            end
                            cost = bestVcost + cost;
                        end
                        if cost > bestCost
                            bestCost = cost;
                            t2 = ((i-1)*kalman_step_size+(l-1)*optimization_step_size(sim3) + 2):((i-1)*kalman_step_size+l*optimization_step_size(sim3) +1);
                            bestNode = currentNode;
                            x_lead(:,t2) = temp_lead(:,t2);
                            best_theta = temp_theta;
                            %             elseif cost == bestCost
                            %                 if distCost < bestDistCost
                            %                     bestDistCost = distCost;
                            %                     t2 = ((i-1)*kalman_step_size+(l-1)*optimization_step_size + 2):((i-1)*kalman_step_size+l*optimization_step_size +1);
                            %                     bestNode = currentNode;
                            %                     x_lead(:,t2) = temp_lead(:,t2);
                            %                     best_theta = temp_theta;
                            %                 end
                        end
                    end
                    new_theta = best_theta;
                    best_nodes = [best_nodes;bestNode];
                    best_costs = [best_costs;bestCost];
                    temp_lead = x_lead;
                end
            end
            toc
            
            %% connection results
            isConnected = zeros(1,N+1);
            for k = 1:N+1
                G = graph;
                G = addnode(G,{'Leader' 'Drone1' 'Drone2' 'Drone3'});
                x_drones = [x_lead(1:2,k)';x_est_avg(:,1:2,k)];
                for i= 1:(size(x_drones,1)-1)
                    for j = i+1:size(x_drones,1)
                        if(sum((x_drones(i,:)-x_drones(j,:)).^2) <= leadDroneCommRange(sim1)^2)
                            G = addedge(G,i,j);
                        end
                    end
                end
                bins = conncomp(G);
                deg = degree(G);
                if algorithm(sim2) == 0
                    isConnected(k) = deg(1) == D;
                else
                    isConnected(k) = all(bins == 1);
                end
                
                
                %     if (k > 2 && isConnected(k-1) == 1 && isConnected(k) == 0 )
                %         disp(['connection lost at time ' num2str(k)]);
                %         figure
                %         plot(G);
                %     end
            end
            timeConnected(sim1,sim2,sim3) = sum(isConnected);
            disp(num2str([leadDroneCommRange(sim1), algorithm(sim2), optimization_step_size(sim3)]))
            disp(['Total time connected for our method is ' num2str(timeConnected(sim1,sim2,sim3))]);
            
            
            %% connection results for center of gravity
            isConnected2 = zeros(1,N+1);
            x_center = mean(x_est_avg(:,1:2,:));
            for k = 1:N+1
                G2 = graph;
                G2 = addnode(G2,{'Leader' 'Drone1' 'Drone2' 'Drone3'});
                x_drones = [x_center(1,1:2,k);x_est_avg(:,1:2,k)];
                for i= 1:(size(x_drones,1)-1)
                    for j = i+1:size(x_drones,1)
                        if(sum((x_drones(i,:)-x_drones(j,:)).^2) <= leadDroneCommRange(sim1)^2)
                            G2 = addedge(G2,i,j);
                        end
                    end
                end
                if algorithm(sim2) == 0
                    deg = degree(G2);
                    isConnected2(k) = deg(1) == D;
                else
                    bins = conncomp(G2);
                    isConnected2(k) = all(bins == 1);
                end
                %     if (k > 2 && isConnected2(k-1) == 1 && isConnected2(k) == 0 )
                %         disp(['center of gravity connection lost at time ' num2str(k)]);
                %         figure
                %         plot(G);
                %     end
            end
            timeConnected2(sim1,sim2,sim3) = sum(isConnected2);
            disp(['Total time connected for center of gravity is ' num2str(timeConnected2(sim1,sim2,sim3))]);
            
            %% plot drone positions
            simTime = 5120;
            timeSpan = 1:simTime;
            figure
            for d = 1:D
                plot(reshape(x_est_avg(d,1,timeSpan),size(timeSpan)),reshape(x_est_avg(d,2,timeSpan),size(timeSpan)),'-o',...
                    'DisplayName',['drone', num2str(d)],'LineWidth',2,...
                    'MarkerIndices',[1024 2048 3072 4096],'MarkerFaceColor','red','MarkerSize',5);
                hold on;
            end
            plot(x_lead(1,timeSpan),x_lead(2,timeSpan),'-o','DisplayName','Leader Drone','LineWidth',4,...
                                    'MarkerIndices',[1024 2048 3072 4096],'MarkerFaceColor','red','MarkerSize',5);
             plot(reshape(x_center(1,1,timeSpan),size(timeSpan)),reshape(x_center(1,2,timeSpan),size(timeSpan)),'DisplayName','Center of Mass','Linewidth',4,...
                 'MarkerIndices',[1024 2048 3072 4096],'MarkerFaceColor','red','MarkerSize',5);
            ylabel('Y position');xlabel('X position'); grid on;
            xlim([-2.5e5 2.5e5])
            ylim([-2.5e5 2.5e5])
            title('Drone Positions');
            legend('show')
            % %% optimization step vs time connected figure
            % figure
            % plot([1,2,4,8],[776,778,876,877])
            % hold on
            % plot([1,2,4,8],[747,747,747,747])
            % title('Total time connected')
            % xlabel('optimization step size')
            % ylabel('time')
            % legend('optimization results','center of gravity')
            % % createVideo(x_est_avg(:,1:2,timeSpan),x_lead(1:2,timeSpan),D,simTime,classVelocities,optimization_step_size);
            x_lead_all(sim1,sim2,sim3,:,:) = x_lead;
        end
    end
end

%% plot results
plotFigureFlag = 0;
if plotFigureFlag
    time = (0:1:N)*dt;
    for d = 1:D
        figure
        subplot(2,1,1); hold on;
        plot(2*time, reshape(x_true(d,1,:),[1,N+1]), 'linewidth', 2);
        plot(2*time, reshape(x_est_avg(d,1,:),[1,N+1]), '--', 'linewidth', 2);
        legend({'True Value', 'Estimated Value'}, 'fontsize', 12);
        ylabel('X position', 'fontsize', 12); grid on;
        title(['\sigma_{acc} = ',num2str(sig_err),',\sigma_{pos} = ',num2str(sig_pos), ',\sigma_{vel} = ',num2str(sig_vel)])
        subplot(2,1,2); hold on;
        plot(2*time, reshape(x_true(d,4,:),[1,N+1]), 'linewidth', 2);
        plot(2*time, reshape(x_est_avg(d,4,:),[1,N+1]), '--', 'linewidth', 2);
        ylabel('X velocity', 'fontsize', 12); xlabel('Time', 'fontsize', 12); grid on;
        legend({'True Value', 'Estimated Value'}, 'fontsize', 12);

        figure
        subplot(2,1,1); hold on;
        plot(2*time, reshape(x_true(d,2,:),[1,N+1]), 'linewidth', 2);
        plot(2*time, reshape(x_est_avg(d,2,:),[1,N+1]), '--', 'linewidth', 2);
        legend({'True Value', 'Estimated Value'}, 'fontsize', 12);
        ylabel('Y position', 'fontsize', 12); grid on;
        title(['\sigma_{acc} = ',num2str(sig_err),',\sigma_{pos} = ',num2str(sig_pos), ',\sigma_{vel} = ',num2str(sig_vel)])
        subplot(2,1,2); hold on;
        plot(2*time, reshape(x_true(d,5,:),[1,N+1]), 'linewidth', 2);
        plot(2*time, reshape(x_est_avg(d,5,:),[1,N+1]), '--', 'linewidth', 2);
        ylabel('Y velocity', 'fontsize', 12); xlabel('Time', 'fontsize', 12); grid on;
        legend({'True Value', 'Estimated Value'}, 'fontsize', 12);
        
        figure
        subplot(2,1,1); hold on;
        plot(2*time, reshape(x_RMSE(d,1,:),[1,N+1]), 'linewidth', 2);
        legend('RMSE', 'fontsize', 12);
        ylabel('X position error std', 'fontsize', 12); grid on;
        subplot(2,1,2); hold on;
        plot(2*time, reshape(x_RMSE(d,4,:)/2,[1,N+1]), 'linewidth', 2);
        ylabel('X velocity error std', 'fontsize', 12); xlabel('Time', 'fontsize', 12); grid on;
        legend('Error', 'fontsize', 12);
        
        figure
        subplot(2,1,1); hold on;
        plot(2*time, reshape(x_RMSE_cumulative(d,1,:),[1,N+1]), 'linewidth', 2);
        legend({'RMSE_{cumulative}'}, 'fontsize', 12);
        ylabel('X position error std', 'fontsize', 12); grid on;
        subplot(2,1,2); hold on;
        plot(2*time, reshape(x_RMSE_cumulative(d,4,:)/2,[1,N+1]), 'linewidth', 2);
        ylabel('X velocity error std', 'fontsize', 12); xlabel('Time', 'fontsize', 12); grid on;
        legend('Error_{cumulative}', 'fontsize', 12);
        
        figure
        subplot(2,1,1); hold on;
        plot(2*time, reshape(x_RMSE(d,2,:),[1,N+1]), 'linewidth', 2);
        legend('RMSE', 'fontsize', 12);
        ylabel('Y position error std', 'fontsize', 12); grid on;
        subplot(2,1,2); hold on;
        plot(2*time, reshape(x_RMSE(d,5,:)/2,[1,N+1]), 'linewidth', 2);
        ylabel('Y velocity error std', 'fontsize', 12); xlabel('Time', 'fontsize', 12); grid on;
        legend('Error', 'fontsize', 12);
        
        figure
        subplot(2,1,1); hold on;
        plot(2*time, reshape(x_RMSE_cumulative(d,2,:),[1,N+1]), 'linewidth', 2);
        legend({'RMSE_{cumulative}'}, 'fontsize', 12);
        ylabel('Y position error std', 'fontsize', 12); grid on;
        subplot(2,1,2); hold on;
        plot(2*time, reshape(x_RMSE_cumulative(d,5,:)/2,[1,N+1]), 'linewidth', 2);
        ylabel('Y velocity error std', 'fontsize', 12); xlabel('Time', 'fontsize', 12); grid on;
        legend('RMSE_{cumulative}', 'fontsize', 12);

        %     figure
        %     subplot(2,1,1); hold on;
        %     plot(time, reshape(x_true(d,3,:),[1,N+1]), 'linewidth', 2);
        %     plot(time, reshape(x_est_avg(d,3,:),[1,N+1]), '--', 'linewidth', 2);
        %     legend({'True', 'Estimated'}, 'fontsize', 12);
        %     ylabel('Z position', 'fontsize', 12); grid on;
        %     subplot(2,1,2); hold on;
        %     plot(time, reshape(x_true(d,6,:),[1,N+1]), 'linewidth', 2);
        %     plot(time, reshape(x_est_avg(d,6,:),[1,N+1]), '--', 'linewidth', 2);
        %     ylabel('Z velocity', 'fontsize', 12); xlabel('Time', 'fontsize', 12); grid on;
    end
end
