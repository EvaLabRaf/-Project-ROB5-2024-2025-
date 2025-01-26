function Kalman_all()
    %{
    -------------------------------------------------------------------------
      1) Connects MATLAB to TurtleBot via ROS
      2) Collects odometry & IMU data for the specified duration
      3) Runs an Extended Kalman Filter on the collected data
      4) Plots comparisons
      5) Animates the robot’s trajectory
    -------------------------------------------------------------------------
    %}
rosshutdown();

%% ------------ Data Collection Function -----------------------------------
function data_collection_turtle(durationSecs)
    %{
    -------------------------------------------------------------------------
     data_collection_turtle(durationSecs)
       - Connects to /odom and /imu topics
       - Collects data for 'durationSecs' at ~10 Hz
       - Places data in base workspace:
            pos_x, pos_y, lin_vel_x, lin_vel_y, orientation, ang_vel_z, Ts
    -------------------------------------------------------------------------
    %}
    
    % Subscribe to odometry and IMU
    odomSub = rossubscriber('/odom','nav_msgs/Odometry');
    imuSub  = rossubscriber('/imu','sensor_msgs/Imu');
    
    % Set a nominal sample rate
    sampleRateHz = 10;   % 10 Hz
    Ts = 1/sampleRateHz; 
    rateObj = robotics.Rate(sampleRateHz);
    
    % Preallocate
    N = ceil(durationSecs * sampleRateHz);
    pos_x       = zeros(N,1);
    pos_y       = zeros(N,1);
    lin_vel_x   = zeros(N,1);
    lin_vel_y   = zeros(N,1);
    orientation = zeros(N,1);  % store yaw in degrees
    ang_vel_z   = zeros(N,1);
    
    disp('Collecting odom + IMU data...');
    tic;
    k = 0;
    while toc < durationSecs && k < N
        k = k + 1;
        
        % Read Odometry
        odomMsg = receive(odomSub, 5);
        if isempty(odomMsg)
            error('No message in odom');
        end
        %odomSub.LatestMessage;
        disp(odomMsg.Pose.Pose.Position.X);
        

        pos_x(k)     = odomMsg.Pose.Pose.Position.X;
        pos_y(k)     = odomMsg.Pose.Pose.Position.Y;
        lin_vel_x(k) = odomMsg.Twist.Twist.Linear.X;
        lin_vel_y(k) = odomMsg.Twist.Twist.Linear.Y;
        
        % Read IMU
        imuMsg = receive(imuSub, 5);%imuSub.LatestMessage;
        if isempty(imuMsg)
            error('No imu msg')
        end
        %disp(imuMsg);
        
        q = [imuMsg.Orientation.W, ...
             imuMsg.Orientation.X, ...
             imuMsg.Orientation.Y, ...
             imuMsg.Orientation.Z];
        
        eul    = quat2eul(q);          % [roll, pitch, yaw] in rad
        yaw = rad2deg(eul(3));      % convert to deg
        orientation(k) = yaw;
        
        ang_vel_z(k) = imuMsg.AngularVelocity.Z;  % rad/s
        
        waitfor(rateObj);
    end
    
    % Trim
    pos_x       = pos_x(1:k);
    pos_y       = pos_y(1:k);
    lin_vel_x   = lin_vel_x(1:k);
    lin_vel_y   = lin_vel_y(1:k);
    orientation = orientation(1:k);
    ang_vel_z   = ang_vel_z(1:k);
    
    % Assign to base workspace
    assignin('base','pos_x',       pos_x);
    assignin('base','pos_y',       pos_y);
    assignin('base','lin_vel_x',   lin_vel_x);
    assignin('base','lin_vel_y',   lin_vel_y);
    assignin('base','orientation', orientation);
    assignin('base','ang_vel_z',   ang_vel_z);
    assignin('base','Ts',          Ts);
    
    disp('Data collection complete!');
end


    %% ------------ PARAMETERS ----------------------------------------------
    robotIP = '192.168.43.116';  %IP of the rosmaster (ordi)
    rosMasterPort = 11311;  % Default ROS master port
    
    % Duration of data collection (seconds)
    durationSecs = 20;
    
    % EKF parameters
    dt = 0.1;  
    L  = 0.16; % Wheelbase 
    
    %% ---------------- Initialize ROS --------------------------------------
    % Make sure your TurtleBot and PC are on the same network.
    try
        rosinit(robotIP, rosMasterPort);
    catch ME
        warning('Could not connect to ROS master. Check IP and network.');
        rethrow(ME);
    end
    
    disp('ROS Initialized. Topics available:');
    rostopic list  % For confirmation
    
    %% ----------------- Collect Data ----------------------------------------

    % Lancer la data collection
    data_collection_turtle(durationSecs);
    
    % Data :
    pos_x = evalin('base','pos_x');
    pos_y = evalin('base','pos_y');
    lin_vel_x = evalin('base','lin_vel_x');
    lin_vel_y = evalin('base','lin_vel_y');
    orientation = evalin('base','orientation');  % orientation in deg (yaw)
    Ts = evalin('base','Ts');  % nominal sampling time
    
    yaw = orientation; %yaw in degree
    % yaw = deg2rad(orientation); %yaw in radian
    
    
    %% ------------------Extended Kalman Filter -------------------------------
    % We assume we have pos_x, pos_y, yaw, Ts in the workspace after data collection.
    
    % If any data ended up as timeseries, convert :
    if isa(pos_x, 'timeseries'); pos_x = pos_x.Data; end
    if isa(pos_y, 'timeseries'); pos_y = pos_y.Data; end
    if isa(yaw,   'timeseries'); yaw   = yaw.Data;   end
    
    % The number of data points collected
    num_steps = length(pos_x);
    if (length(pos_y) ~= num_steps || length(yaw) ~= num_steps)
        error('pos_x, pos_y, and yaw must have the same length.');
    end
    
    dt = Ts;  % Overwrite the dt with the actual sampling time from data collection
    
    % Define Noise / Covariances 
    gamma = 1000;
    process_noise_cov      = diag([1e-3, 1e-3, 1e-3, 1e-3, gamma, gamma, gamma]); 
    measurement_noise_cov  = diag([0.01, 0.01, 0.0025]);
    
    % Define State Matrices
    % State vector x_est = [ X, Y, Vx, Vy, Ax, Ay, Yaw ]^T (7x1)
    phi_d = eye(7);
    
    % We'll show some sample usage (update X w.r.t. Vx, etc.)
    phi_d(1,3) = dt;  % X depends on Vx
    phi_d(2,4) = dt;  % Y depends on Vy
    % example placeholders for acceleration coupling
    phi_d(3,5) = 0.3679;  
    phi_d(4,6) = 0.3679;  
    
    % Measurement matrix: 
    H = zeros(3,7);
    H(1,1) = 1;  % measure X
    H(2,2) = 1;  % measure Y
    H(3,7) = 1;  % measure Yaw
    
    % Initialize State and Covariance 
    x_est = zeros(7,1);     % [X, Y, Vx, Vy, Ax, Ay, Yaw]
    P     = eye(7)*1e6;     % Large initial uncertainty
    
    % Preallocate for data logging
    pos_dev_error_x  = zeros(num_steps,1);
    pos_dev_error_y  = zeros(num_steps,1);
    vel_dev_error_x  = zeros(num_steps,1);
    vel_dev_error_y  = zeros(num_steps,1);
    acc_dev_error_x  = zeros(num_steps,1);
    acc_dev_error_y  = zeros(num_steps,1);
    estimated_positions = zeros(num_steps,2);
    estimated_yaws      = zeros(num_steps,1);
    
    % Approximate Velocities & Accelerations from data 
    velocities    = zeros(num_steps,2);  % [Vx, Vy]
    accelerations = zeros(num_steps,2);  % [Ax, Ay]
    
    for k = 2:num_steps
        velocities(k-1,1) = (pos_x(k) - pos_x(k-1)) / dt; 
        velocities(k-1,2) = (pos_y(k) - pos_y(k-1)) / dt;
    end
    for k = 3:num_steps
        accelerations(k-2,1) = (velocities(k-1,1) - velocities(k-2,1)) / dt;
        accelerations(k-2,2) = (velocities(k-1,2) - velocities(k-2,2)) / dt;
    end
    
    % ========== EKF Loop ==========
    for k = 1:num_steps
        % Simulate measurement with additive noise
        noise = [ sqrt(measurement_noise_cov(1,1))*randn(1); 
                  sqrt(measurement_noise_cov(2,2))*randn(1); 
                  sqrt(measurement_noise_cov(3,3))*randn(1) ];
        z = [pos_x(k); pos_y(k); yaw(k)] + noise;
        
        % --- Prediction Step ---
        % Simple motion model:
        % X_k+1 = X_k + dt*Vx_k
        % Y_k+1 = Y_k + dt*Vy_k
        x_est(1) = x_est(1) + dt*velocities(k,1);  % position X
        x_est(2) = x_est(2) + dt*velocities(k,2);  % position Y
        
        % Example acceleration placeholders (adapt to your model):
        phi_d(3,5) = dt;  % velocity x depends on acceleration x
        phi_d(4,6) = dt;  % velocity y depends on acceleration y
        
        % Update covariance
        P = phi_d * P * phi_d' + process_noise_cov;
        
        % --- Update Step ---
        y = z - H*x_est;                 % innovation
        S = H * P * H' + measurement_noise_cov;
        K = P * H' / S;                  % Kalman gain
        x_est = x_est + K*y;            % updated state
        P = (eye(7) - K*H)*P;           % updated covariance
        
        % Store standard deviations for plotting
        pos_dev_error_x(k) = sqrt(P(1,1));
        pos_dev_error_y(k) = sqrt(P(2,2));
        vel_dev_error_x(k) = sqrt(P(3,3));
        vel_dev_error_y(k) = sqrt(P(4,4));
        acc_dev_error_x(k) = sqrt(P(5,5));
        acc_dev_error_y(k) = sqrt(P(6,6));
        
        % Store the estimated position and yaw
        estimated_positions(k,:) = x_est(1:2)';
        estimated_yaws(k)        = x_est(7);
    end
    
    % --- Reconstruct velocities and accelerations from final estimates (if desired) ---
    est_velocities    = zeros(num_steps,2);
    est_accelerations = zeros(num_steps,2);
    for k = 2:num_steps
        est_velocities(k-1,1) = (estimated_positions(k,1) - estimated_positions(k-1,1)) / dt;
        est_velocities(k-1,2) = (estimated_positions(k,2) - estimated_positions(k-1,2)) / dt;
    end
    for k = 3:num_steps
        est_accelerations(k-2,1) = (est_velocities(k-1,1) - est_velocities(k-2,1)) / dt;
        est_accelerations(k-2,2) = (est_velocities(k-1,2) - est_velocities(k-2,2)) / dt;
    end
    
    % Create time vector
    time_vector = (0:num_steps-1)*dt;
    
    %% -----------------------Save Results -------------------------------------
    save('collected_data.mat', 'pos_x', 'pos_y', 'lin_vel_x', 'lin_vel_y', 'orientation', 'yaw', 'Ts');
    
 
end