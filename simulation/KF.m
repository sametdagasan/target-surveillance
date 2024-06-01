 close all
clc
clear
%% settings
N = 4000; % number of time steps
dt = 1; % time between time steps
D = 1; % number of drones
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
class_velocity = [80];

%% true trajectory

x_true = zeros(D,6,N+1); % true state
a_true = 0.1*zeros(D,3,N);   % true acceleration

for d = 1:D
    vel_select = randi([1,length(class_velocity)]);
    drone_vel = [class_velocity(vel_select);class_velocity(vel_select);0];
    drone_pos = [randi(50,2,1);0];
    x_true(d,:,1) = [drone_pos;drone_vel]; % initial true state
    for k = 2:N+1
        x_true_f = x_true(d,:,k-1);
        if (k == 2000)
            drone_vel_new = [40,40,0];
            x_true_f = [x_true(d,1:3,k-1),drone_vel_new];
        end
        x_true(d,:,k) = F*(x_true_f)' + B*a_true(d,:,k-1)';
    end
end
%% Kalman filter simulation

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
%% plot results
for d = 1:D
    time = (0:1:N)*dt;
    figure
    subplot(2,1,1); hold on;
    plot(time, reshape(x_true(d,1,:),[1,N+1]), 'linewidth', 2);
    plot(time, reshape(x_est_avg(d,1,:),[1,N+1]), '--', 'linewidth', 2);
    legend({'True', 'Estimated'}, 'fontsize', 12);
    ylabel('X position', 'fontsize', 12); grid on;
    title(['\sigma_{acc} = ',num2str(sig_err),',\sigma_{pos} = ',num2str(sig_pos), ',\sigma_{vel} = ',num2str(sig_vel)])
    subplot(2,1,2); hold on;
    plot(time, reshape(x_true(d,4,:),[1,N+1]), 'linewidth', 2);
    plot(time, reshape(x_est_avg(d,4,:),[1,N+1]), '--', 'linewidth', 2);
    ylabel('X velocity', 'fontsize', 12); xlabel('Time', 'fontsize', 12); grid on;
    
    figure
    subplot(2,1,1); hold on;
    plot(time, reshape(x_true(d,2,:),[1,N+1]), 'linewidth', 2);
    plot(time, reshape(x_est_avg(d,2,:),[1,N+1]), '--', 'linewidth', 2);
    legend({'True', 'Estimated'}, 'fontsize', 12);
    ylabel('Y position', 'fontsize', 12); grid on;
    title(['\sigma_{acc} = ',num2str(sig_err),',\sigma_{pos} = ',num2str(sig_pos), ',\sigma_{vel} = ',num2str(sig_vel)])
    subplot(2,1,2); hold on;
    plot(time, reshape(x_true(d,5,:),[1,N+1]), 'linewidth', 2);
    plot(time, reshape(x_est_avg(d,5,:),[1,N+1]), '--', 'linewidth', 2);
    ylabel('Y velocity', 'fontsize', 12); xlabel('Time', 'fontsize', 12); grid on;
    
    figure
    subplot(2,1,1); hold on;
    plot(time, reshape(x_RMSE(d,1,:),[1,N+1]), 'linewidth', 2);
    legend({'RMSE', 'Estimated'}, 'fontsize', 12);
    ylabel('X position error std', 'fontsize', 12); grid on;
    subplot(2,1,2); hold on;
    plot(time, reshape(x_RMSE(d,4,:),[1,N+1]), 'linewidth', 2);
    ylabel('X velocity error std', 'fontsize', 12); xlabel('Time', 'fontsize', 12); grid on;
    
    figure
    subplot(2,1,1); hold on;
    plot(time, reshape(x_RMSE_cumulative(d,1,:),[1,N+1]), 'linewidth', 2);
    legend({'RMSE_{cumulative}'}, 'fontsize', 12);
    ylabel('X position error std', 'fontsize', 12); grid on;
    subplot(2,1,2); hold on;
    plot(time, reshape(x_RMSE_cumulative(d,4,:),[1,N+1]), 'linewidth', 2);
    ylabel('X velocity error std', 'fontsize', 12); xlabel('Time', 'fontsize', 12); grid on;
    
    figure
    subplot(2,1,1); hold on;
    plot(time, reshape(x_RMSE(d,2,:),[1,N+1]), 'linewidth', 2);
    legend({'RMSE', 'Estimated'}, 'fontsize', 12);
    ylabel('Y position error std', 'fontsize', 12); grid on;
    subplot(2,1,2); hold on;
    plot(time, reshape(x_RMSE(d,5,:),[1,N+1]), 'linewidth', 2);
    ylabel('Y velocity error std', 'fontsize', 12); xlabel('Time', 'fontsize', 12); grid on;
    
    figure
    subplot(2,1,1); hold on;
    plot(time, reshape(x_RMSE_cumulative(d,2,:),[1,N+1]), 'linewidth', 2);
    legend({'RMSE_{cumulative}'}, 'fontsize', 12);
    ylabel('Y position error std', 'fontsize', 12); grid on;
    subplot(2,1,2); hold on;
    plot(time, reshape(x_RMSE_cumulative(d,5,:),[1,N+1]), 'linewidth', 2);
    ylabel('Y velocity error std', 'fontsize', 12); xlabel('Time', 'fontsize', 12); grid on;
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
