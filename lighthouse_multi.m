%% Define Sensor Positions Relative to Tracker Center

sd_x = 3 / 100; % x-distance between sensors in Meters
sd_y = 1.5 / 100; % y-distance between sensors in Meters

S = [];
S = [S, [- sd_x / 2; sd_y / 2; 0]]; % Sensor 0
S = [S, [- sd_x / 2; - sd_y / 2; 0]]; % Sensor 1
S = [S, [sd_x / 2; sd_y / 2; 0]]; % Sensor 2
S = [S, [sd_x / 2; - sd_y / 2; 0]]; % Sensor 3

%% Define Tracker Position and Orientation


% R_actual = Helper.deg2dcm(45, 45, 45); % degrees
R_actual = Helper.deg2dcm(0, 0, 0); % degrees
R_sampled = R_actual; % no error

R_sampled = Helper.deg2dcm(45, 0, 0); % sampled and real is different

tracker_center = [0; 0; 1]; % center of tracker

%% Define Base Stations & Detections


% detection number 1 will serve as reference point in calculations 
clear detection

INTRODUCE_ERRORS = false;
INTRODUCE_ERRORS = true;

detection(1).color = 'k';
detection(1).B = [-1.789562; 5.251678; 2.641019];
detection(1).sens = 0;
detection(1).r_error = Helper.deg2dcm(0,0,0); % zeros if exactly on sensor
if INTRODUCE_ERRORS
    detection(1).r_error = Helper.deg2dcm(-0.1,0,0); % zeros if exactly on sensor
end

detection(2).color = 'r';
detection(2).B = [1.734847; -4.475452; 2.665298];
detection(2).sens = 2;
detection(2).r_error = Helper.deg2dcm(0,0,0); % zeros if exactly on sensor
if INTRODUCE_ERRORS
    detection(2).r_error = Helper.deg2dcm(-0.1,0,0); % zeros if exactly on sensor
end

detection(3).color = 'g';
detection(3).B = [-1.759562; -2.005452; 2.635298];
detection(3).B = [-1.759562; -4.505452; 2.635298];
detection(3).sens = 1;
detection(3).r_error = Helper.deg2dcm(0,0,0); % zeros if exactly on sensor
if INTRODUCE_ERRORS
    detection(3).r_error = Helper.deg2dcm(-0.1,0,0); % zeros if exactly on sensor
end

detection(4).color = 'b';
detection(4).B = [1.729562; 5.251678; 2.641019];
detection(4).sens = 3;
detection(4).r_error = Helper.deg2dcm(0,0,0); % zeros if exactly on 
if INTRODUCE_ERRORS
    detection(4).r_error = Helper.deg2dcm(-0.1,0,0); % zeros if exactly on sensor
end




SINGLE_BASESTATION = false;
% SINGLE_BASESTATION = true;

if SINGLE_BASESTATION
    detection(2).B = detection(1).B; % single base station
    detection(3).B = detection(1).B; % single base station
end


%% Plot Axes

O = zeros(3, 3);

ID = eye(3, 3);
ID_x = ID(1, :);
ID_y = ID(2, :);
ID_z = ID(3, :);

quiver3(O(1, :), O(2, :), O(3, :), ID_x, ID_y, ID_z, 'k'); % plot origin
grid on
daspect([1 1 1])
hold on

%%  Plot Sensors

% for i = 1 : size(S(), 2)
%     Si = S(:, i);
%     plot3(Si(1), Si(2), Si(3), 'k.-'); % plot sensors
%     text(Si(1), Si(2), Si(3), num2str(i - 1), 'Color', 'k')
% end

%% Plot Basestations

for i = 1:length(detection)
    B = detection(i).B;
    
    plot3(B(1), B(2), B(3), 'b^-'); % plot Base Station
    text(B(1), B(2), B(3), num2str(i), 'Color', 'r')
end

for i = 2:length(detection)
    % distance between Base Stations
    B = detection(i).B;
    fprintf('Distance Between BS at detection %d & 1: %f\n', i, norm(B - detection(1).B));
end


%% Plot & Configure Detections

plot3(tracker_center(1), tracker_center(2), tracker_center(3), '^-', 'Color', 'k'); % plot Tracker center
% text(tracker_center(1), tracker_center(2), tracker_center(3), '.', 'Color', 'k');


for i = 1:length(detection)
    s_1 = (S(:, detection(i).sens + 1) - S(:, detection(1).sens + 1)); % relative to detection(1).sens
    D_1_actual = R_actual*s_1;
    D_1 = R_sampled*s_1;
    
%     fprintf('Sensor Vector: \n');
%     disp(D_1);

    detection(i).D_1_actual = D_1_actual;
    detection(i).D_1 = D_1;
end

P_1 = tracker_center + R_actual * S(:, detection(1).sens + 1); % detection(1).sens as reference point

for i = 2:length(detection)
    D_1_actual = detection(i).D_1_actual;
   
    fprintf('Distance Between Sensors at detection %d & 1: %f\n', i, norm(D_1_actual));
    
    quiver3(P_1(1), P_1(2), P_1(3), D_1_actual(1), D_1_actual(2), D_1_actual(3), 'k'); % plot D_1_actual on rays
end




%% Plot & Configure Rays

P_1 = tracker_center + R_actual * S(:, detection(1).sens + 1); % detection(1).sens as reference point

for i = 1:length(detection)
    % Artibary point on a Ray in World Frame (in Meters), to generate the Ray Vector
    
    Rp = P_1 + detection(i).D_1_actual;

    plot3(Rp(1), Rp(2), Rp(3), 'k.-'); % plot Artibary point on Ray

    % Unit Vector of Ray from Base Stations
    B = detection(i).B;
    
    detection(i).Rp = Rp;
    detection(i).Rp_dist = norm(Rp - B);
    
    r = (Rp - B) / norm(Rp - B);

    r = detection(i).r_error * r;
    detection(i).r = r;
    
    % Simulated End of Ray
    plot_ray_length = 10;
    Ray = plot_ray_length * r;

    quiver3(B(1), B(2),B(3), Ray(1), Ray(2), Ray(3), 'k'); % plot Rays
end


%% 2 Base Stations, 1 Sensor (Find Shortest Segment between Rays)

detection_pairs = nchoosek( 1:length(detection), 2);

for i = 1:size(detection_pairs(), 1)
    detection_pair = detection_pairs(i,:);
    
    first = detection_pair(1);
    second = detection_pair(2);
    
    [Sc_1, Sc_2, dist, dir, vec] = shortestSegment(detection(first).B, detection(first).r, detection(second).B, detection(second).r);
%     fprintf('Shortest Distance Between Rays from BS 2 & 3: %f\n', dist);
    plot3(Sc_1(1), Sc_1(2), Sc_1(3), 'b.-', Sc_2(1), Sc_2(2), Sc_2(3), 'y.-'); % plot Shortest Segment points
    quiver3(Sc_1(1), Sc_1(2), Sc_1(3), vec(1), vec(2), vec(3), 'y'); % plot Shortest Segment vector

    % since Rays from Base Stations should surely touch the Sensor at the same
    % point, the distance here should be zero.
    % the extra distance may signify incorrect Base Station origins or lack of
    % calibration
end


%%  2 Base Stations on 2 different Sensors (Best Fit of Segment between Rays)

prev_Sf_1 = false;

d_1_suggestions = [];
Sf_1_suggestions = [];
Pc_suggestions = [];

for i = 2:length(detection)
   
    D_1 = detection(i).D_1;

    [Sf_2, Sf_1, d_2, d_1, segment_error] = Helper.bestFitBetweenRays(detection(i).B, detection(1).B, detection(i).r, detection(1).r, D_1);
    
    fprintf('Magnitude of Segment Error for detection %d: %f mm\n', i, segment_error * 1000);
    

    Pf = Sf_1 + (Sf_2 - Sf_1) / 2; % midpoint of Best Fit Segment

    Pf_1 = Pf - D_1 / 2;
    Pf_2 = Pf + D_1 / 2;

    Pc = Pf_1 - S(:, detection(1).sens + 1);
            
%     diff_center = norm(Pc - tracker_center) * 1000 ;
            
    Helper.plotSensors(detection(i).color, S, R_sampled, Sf_2, Sf_1, detection(i).sens, detection(1).sens, D_1);
    
    
    d_1_suggestions = [d_1_suggestions , d_1];
    Sf_1_suggestions = [Sf_1_suggestions , Sf_1];
    Pc_suggestions = [Pc_suggestions , Pc];
    prev_Sf_1 = Sf_1;
    
    fprintf('Suggested Distance on Ray 1 by detection %d: %f\n', i, d_1);
    fprintf('Resulting Distance on Ray %d by detection %d: %f\n', i, i, d_2);

    if prev_Sf_1
        diff = norm(Sf_1 - prev_Sf_1);
    end
end

d_1_mean = mean(d_1_suggestions, 2);
d_1_diff = norm(d_1_mean - detection(1).Rp_dist) * 1000;

%  Rp_mean = detection(1).B + d_1_mean * detection(1).r;
%  plot3(Rp_mean(1), Rp_mean(2), Rp_mean(3), 'o', 'Color', detection(1).color); % plot sensor_on_ray_1
    
fprintf('Mean Distance on Ray 1: %f with error %f mm\n', d_1_mean, d_1_diff);
    
for i = 2:length(detection)
   
    D_1 = detection(i).D_1;

    Rpx_dist = Helper.rayDistFromRayDist(detection(i).B, detection(1).B, detection(i).r, detection(1).r, D_1, d_1_mean);
 
    Rpx_dist_diff = norm(Rpx_dist - detection(i).Rp_dist) * 1000;

%     Rpx = detection(i).B + Rpx_dist * detection(i).r;
%     plot3(Rpx(1), Rpx(2), Rpx(3), 'o', 'Color', detection(i).color); % plot sensor_on_ray_1

    fprintf('Mean Distance on Ray %d: %f with error %f mm\n', i, Rpx_dist, Rpx_dist_diff);
end



Sf_1_mean = mean(Sf_1_suggestions, 2);
Sf_1_diff = norm(Sf_1_mean - detection(1).Rp) * 1000; % same  as d_1_diff




Pc_mean = mean(Pc_suggestions, 2);
Pc_diff = norm(Pc_mean - tracker_center) * 1000;


plot3(Pc_mean(1), Pc_mean(2), Pc_mean(3), '^-', 'Color', 'c');
fprintf('Mean Tracker Center:\n');
disp(Pc_mean);
fprintf('Mean Tracker Center error %f mm\n',  Pc_diff);

%% Plot End

% legend({'detection(1).B', 'detection(2).B', 'Rp_1', 'Rp_2', 'Sa_1', 'Sa_2'});

% view(0,0) % X & Z
view(0,90) % X & Y
zoom(50)
% view(90,0) % Y & Z

% rotate3d on


hold off