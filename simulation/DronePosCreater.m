close all
for i = 1:20
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
    
    %% parameters
    classVelocities = [70;60;50]; % different tracking classes with velocities
    initialAngles = [pi/4,0,-pi/2];
    turnAngle = pi/12;
    droneProbRange = 5*classVelocities; % if the drone is slow we are more certain about its whereabout
    droneStepSize = 64;
    drones_pos = [0,0,0;500,500,0;0,1000,0];
    
    drone_probRange = zeros(1,D);
    x_true = zeros(D,6,N+1); % true state
    a_true = 0.1*zeros(D,3,N);   % true acceleration
    for d = 1:D
        drone_vel = classVelocities(d)*[cos(initialAngles(d)),sin(initialAngles(d)),0];
        drone_probRange(d) = droneProbRange(d);
        drone_pos = drones_pos(d,:);
        x_true(d,:,1) = [drone_pos,drone_vel];
        for k = 2:N+1
            x_true_f = x_true(d,:,k-1);
            if(rem(k,droneStepSize)==0)
                if (d == 1 && i > 10)
                    initialAngles(d) = initialAngles(d) + 0.25*randi([0,2])*turnAngle;
                elseif (d == 1 && i > 5)
                    initialAngles(d) = initialAngles(d) - 0.25*randi([0,2])*turnAngle;
                else
                    initialAngles(d) = initialAngles(d) + randi([-1,1])*turnAngle;
                end
                drone_vel = classVelocities(d)*[cos(initialAngles(d)),sin(initialAngles(d)),0];
                x_true_f = [x_true(d,1:3,k-1),drone_vel];
            end
            x_true(d,:,k) = F*(x_true_f)' + B*a_true(d,:,k-1)';
        end
    end
    
    %% plot drone positions
    simTime = 5120;
    timeSpan = 1:simTime;
    figure
    for d = 1:D
        plot(reshape(x_true(d,1,timeSpan),size(timeSpan)),reshape(x_true(d,2,timeSpan),size(timeSpan)),'DisplayName',['drone', num2str(d)],'LineWidth',2);
        hold on;
    end
    % plot(x_lead(1,timeSpan),x_lead(2,timeSpan),'DisplayName','Leader Drone','LineWidth',4);
    % plot(reshape(x_center(1,1,timeSpan),size(timeSpan)),reshape(x_center(1,2,timeSpan),size(timeSpan)),'DisplayName','Center','Linewidth',4);
    ylabel('Y position');xlabel('X position'); grid on;
    xlim([-2.5e5 2.5e5])
    ylim([-2.5e5 2.5e5])
    title(['Drone Positions',num2str(i)]);
    legend('show');
    filename = ['dronePos',num2str(i),'.mat'];
    save(['DroneDataset\' filename],'a_true','x_true','drone_probRange'); 
end