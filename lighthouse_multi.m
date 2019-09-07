%% Define Sensor Positions Relative to Tracker Center

sd_x = 3 / 100; % x-distance between sensors in Meters
sd_y = 1.5 / 100; % y-distance between sensors in Meters

% sd_x = 8 / 100; % x-distance between sensors in Meters
% sd_y = 8 / 100; % y-distance between sensors in Meters
% 8x8 in single BS with 0.01 degree error gives at most 50mm error, however in 3x1.5, it's 330mm, 6 times the error

S = [];
S = [S, [- sd_x / 2; sd_y / 2; 0]]; % Sensor 0
S = [S, [- sd_x / 2; - sd_y / 2; 0]]; % Sensor 1
S = [S, [sd_x / 2; sd_y / 2; 0]]; % Sensor 2
S = [S, [sd_x / 2; - sd_y / 2; 0]]; % Sensor 3

%% Yaw sensors by 180 deg
% S = Helper.swapCols(S, 1, 2);
% S = Helper.swapCols(S, 3, 4);
% 
% S = Helper.swapCols(S, 1, 3);
% S = Helper.swapCols(S, 2, 4);
% %%  

%% Define Tracker Position and Orientation

INTRODUCE_ORIENTATION_ERROR = false;
% INTRODUCE_ORIENTATION_ERROR = true;

R_actual = Helper.deg2dcm(0, 0, 0); % degrees
% R_actual = Helper.deg2dcm(45, 45, 45); % degrees
% R_actual = Helper.deg2dcm(0, 0, 45); % degrees roll left
% R_actual = Helper.deg2dcm(0, -45, 0); % degrees pitch forward
% R_actual = Helper.deg2dcm(-45, 0, 0); % degrees yaw left

R_sampled = R_actual; % no error

if INTRODUCE_ORIENTATION_ERROR
    R_sampled = Helper.deg2dcm(45, 0, 0); % sampled and real is different
end

tracker_center = [0; 0; 1]; % center of tracker

%% Define Base Stations & Detections

% plot_ray_length = 10;
plot_ray_length = 5;

% detection number 1 will serve as reference point in calculations 
clear detection

INTRODUCE_RAY_ERROR = false;
% INTRODUCE_RAY_ERROR = true;

% RANDOM_ERROR_DEGREES = 0.001; % 1 BS = 30mm
RANDOM_ERROR_DEGREES = 0.01; % 1 BS = 330mm, 2 BS = 290mm or 88mm ,4 BS = 1.7mm
% RANDOM_ERROR_DEGREES = 0.1; % 4 BS = 21mm
% RANDOM_ERROR_DEGREES = 1; % 4 BS = mm

% BS_1 = [0; 0; 10.000000]; % from the top % 1 BS, 8x8 from the top only with 0.01 degree error gives 100mm error
% BS_1 = [0; 0; 2.641019]; % from the top % 1 BS, 8x8 from the top only with 0.01 degree error only gives 3mm error!
% BS_1 = [-1.789562; 5.251678; 2.641019];
% BS_2 = [1.734847; -4.475452; 2.665298];
% BS_3 = [-1.759562; -4.505452; 2.635298];
% BS_4 = [1.729562; 5.251678; 2.641019];

BSR_1 = Helper.deg2dcm(0, 0, 0);
BSR_2 = Helper.deg2dcm(180, 0, 0);
BSR_3 = Helper.deg2dcm(0, 0, 0);
BSR_4 = Helper.deg2dcm(0, 0, 0);




BS_1 = Helper.cfToReal([-2.61173797; 2.6828599; -1.73622894])
BS_2 = Helper.cfToReal([ 2.37578106; 2.73936605; 1.37233901])

BSR_1 = Helper.cfToRealRot([[-0.516858,  0.607955, -0.602701]; [0.025856, 0.714796, 0.698855]; [ 0.855681,  0.345626, -0.385167]]);
BSR_2 = Helper.cfToRealRot([[ 0.534727, -0.598345,  0.596699]; [0.082423, 0.739697, 0.667874]; [-0.840995, -0.307949,  0.444853]]);




bs(1).origin = BS_1;
bs(1).mat = BSR_1;

bs(2).origin = BS_2;
bs(2).mat = BSR_2;

% bs(3).origin = BS_3;
% bs(3).mat = BSR_3;
% 
% bs(4).origin = BS_4;
% bs(4).mat = BSR_4;


detection(1).color = 'k';
detection(1).bs = bs(1);
detection(1).sens = 0;
detection(1).r_error = Helper.deg2dcm(0,0,0); % zeros if exactly on sensor
if INTRODUCE_RAY_ERROR
%     detection(1).r_error = Helper.deg2dcm(-0.1,0,0); % zeros if exactly on sensor
    detection(1).r_error = Helper.randDeg2Dcm(RANDOM_ERROR_DEGREES); % zeros if exactly on sensor
end
detection(2).color = 'r';
detection(2).bs = bs(2);
detection(2).sens = 2;
detection(2).r_error = Helper.deg2dcm(0,0,0); % zeros if exactly on sensor
if INTRODUCE_RAY_ERROR
%     detection(2).r_error = Helper.deg2dcm(-0.1,0,0); % zeros if exactly on sensor
    detection(2).r_error = Helper.randDeg2Dcm(RANDOM_ERROR_DEGREES); % zeros if exactly on sensor
end

detection(3).color = 'g';
detection(3).bs = bs(1);
% detection(3).bs = bs(2);
% detection(3).bs = bs(3);
detection(3).sens = 1;
detection(3).r_error = Helper.deg2dcm(0,0,0); % zeros if exactly on sensor
if INTRODUCE_RAY_ERROR
%     detection(3).r_error = Helper.deg2dcm(-0.1,0,0); % zeros if exactly on sensor
    detection(3).r_error = Helper.randDeg2Dcm(RANDOM_ERROR_DEGREES); % zeros if exactly on sensor
end

detection(4).color = 'b';
% detection(4).bs = bs(1);
detection(4).bs = bs(2);
% detection(4).bs = bs(4);
detection(4).sens = 3;
detection(4).r_error = Helper.deg2dcm(0,0,0); % zeros if exactly on 
if INTRODUCE_RAY_ERROR
%     detection(4).r_error = Helper.deg2dcm(-0.1,0,0); % zeros if exactly on sensor
    detection(4).r_error = Helper.randDeg2Dcm(RANDOM_ERROR_DEGREES); % zeros if exactly on sensor
end


%% Real Data

USE_REAL_RAY = false;
% USE_REAL_RAY = true;

if USE_REAL_RAY
    clear detection

    %somehow CF firmware orients basestations differently
    % BS_1 = [-1.73622894; -2.61173797; 2.6828599];
    % BS_2 = [1.37233901; 2.37578106; 2.73936605];
    BS_1 = [-2.61173797; 2.6828599; -1.73622894];
    BS_2 = [ 2.37578106; 2.73936605; 1.37233901;];
    
    BSR_1 = [[-0.516858,  0.607955, -0.602701]; [0.025856, 0.714796, 0.698855]; [ 0.855681,  0.345626, -0.385167]];
    BSR_2 = [[ 0.534727, -0.598345,  0.596699]; [0.082423, 0.739697, 0.667874]; [-0.840995, -0.307949,  0.444853]];


    bs(1).origin = BS_1;
    bs(1).mat = BSR_1;

    bs(2).origin = BS_2;
    bs(2).mat = BSR_2;


    %so then the crazyflie should also orient differently
    S = Helper.swapRows(S, 2, 3);
    S = Helper.swapRows(S, 1, 3);
    
    S = Helper.swapCols(S, 1, 2);
    S = Helper.swapCols(S, 3, 4);
    
    S = Helper.swapCols(S, 1, 3);
    S = Helper.swapCols(S, 2, 4);
   
    
    %attempt to correct the rotation

%     R_actual = [0.999969661 -0.0066452031 -0.0040709218; 0.006622802 0.999962986 -0.00549162691; 0.00410726387 0.00546449935 0.999976695];
%     R_actual = Helper.deg2dcm(90, 0, 90); % degrees
%     R_sampled = R_actual; % no error


%     tracker_center = ([-2.21150947; 2.39448023; -1.6517576] + [-2.19172335; 2.38026118; -1.65189958])/2; % center of tracker

    detection(1).color = 'k';
    detection(1).bs = bs(1);
    detection(1).sens = 0;
    
    detection(1).sens = 0;
    detection(1).real_r = [0.798410177; -0.577826262; 0.169288218];
    
%     detection(1).bs = bs(1);
%     detection(1).sens = 3;
%     detection(1).real_r = [0.79735142; -0.581848264; 0.160261855];

%     detection(1).bs = bs(1);
%     detection(1).sens = 0;
%     detection(1).real_r = [0.798819661; -0.577314794; 0.169101238];

%     detection(1).bs = bs(1);
%     detection(1).sens = 0;
%     detection(1).real_r = [0.798809946; -0.577327847; 0.169102848];

%     detection(1).bs = bs(1);
%     detection(1).sens = 0;
%     detection(1).real_r = [0.776623249; -0.609121561; 0.160710543];





    detection(2).color = 'r';
    detection(2).bs = bs(2);
    detection(2).sens = 2;
    
    detection(2).sens = 1;
    detection(2).real_r = [-0.478063911; -0.568822145; -0.669249952];

%     detection(2).bs = bs(2);
%     detection(2).sens = 3;
%     detection(2).real_r = [-0.47880125; -0.564612567; -0.672280788];

%     detection(2).bs = bs(1);
%     detection(2).sens = 2;
%     detection(2).real_r = [0.799880087; -0.578143716; 0.16106534];

%     detection(2).bs = bs(1);
%     detection(2).sens = 3;
%     detection(2).real_r = [0.801220775; -0.576445818; 0.160486817];

%     detection(2).bs = bs(1);
%     detection(2).sens = 3;
%     detection(2).real_r = [0.778907955; -0.608527899; 0.151647255];
end


%% Pre-calculate sensor vectors

% for j = 1:length(S)
%     for i = 1:length(S)
% %         if i ~= j
%             fprintf('%d & %d \n',  j-1, i-1);
%             s_1 = (S(:, j) - S(:, i))
% %         end
%     end
% end
% return

%%

SINGLE_BASESTATION = false;
% SINGLE_BASESTATION = true;

if SINGLE_BASESTATION
    detection(2).bs = detection(1).bs; % single base station
    detection(3).bs = detection(1).bs; % single base station
    detection(4).bs = detection(1).bs; % single base station
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

for i = 1:length(bs)
    B = bs(i).origin;
    
    plot3(B(1), B(2), B(3), 'b^-'); % plot Base Station
    text(B(1), B(2), B(3), num2str(i), 'Color', 'r')
    
    if isfield(bs(i), 'mat') && ~isempty(bs(i).mat)
        Helper.plotRotMat(B, bs(i).mat, 0.5, 'g', 'b' ,'r'); %ref: https://wiki.bitcraze.io/doc:lighthouse:setup
    end
end

for i = 2:length(detection)
    % distance between Base Stations
    B = detection(i).bs.origin;
    fprintf('Distance Between BS at detection %d & 1: %f\n', i, norm(B - detection(1).bs.origin));
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
    

    % Unit Vector of Ray from Base Stations
    B = detection(i).bs.origin;
        
    if ~USE_REAL_RAY
        % Artibary point on a Ray in World Frame (in Meters), to generate the Ray Vector
        Rp = P_1 + detection(i).D_1_actual;

        plot3(Rp(1), Rp(2), Rp(3), 'k.-'); % plot Artibary point on Ray

        detection(i).Rp = Rp;
        detection(i).Rp_dist = norm(Rp - B);
        
        r = (Rp - B) / norm(Rp - B);
    else
%     if ~isfield(detection(i), 'r') || isempty(detection(i).r)
        r = detection(i).real_r; %use pre-defined
    end
    

    if isfield(detection(i), 'r_error') && ~isempty(detection(i).r_error)
        r = detection(i).r_error * r; %apply error
    end
    
    detection(i).r = r;
    
    % Simulated End of Ray
    Ray = plot_ray_length * r;

    quiver3(B(1), B(2),B(3), Ray(1), Ray(2), Ray(3), 'k'); % plot Rays
end


%% 2 Base Stations, 1 Sensor (Find Shortest Segment between Rays)

detection_pairs = nchoosek( 1:length(detection), 2);

for i = 1:size(detection_pairs(), 1)
    detection_pair = detection_pairs(i,:);
    
    first = detection_pair(1);
    second = detection_pair(2);
    
    [Sc_1, Sc_2, dist, dir, vec] = shortestSegment(detection(first).bs.origin, detection(first).r, detection(second).bs.origin, detection(second).r);
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

    [Sf_2, Sf_1, d_2, d_1, segment_error] = Helper.bestFitBetweenRays(detection(i).bs.origin, detection(1).bs.origin, detection(i).r, detection(1).r, D_1);
    
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


%  Rp_mean = detection(1).bs.origin + d_1_mean * detection(1).r;
%  plot3(Rp_mean(1), Rp_mean(2), Rp_mean(3), 'o', 'Color', detection(1).color); % plot sensor_on_ray_1
    
d_1_mean = mean(d_1_suggestions, 2);
if ~USE_REAL_RAY
    d_1_diff = norm(d_1_mean - detection(1).Rp_dist) * 1000;
    fprintf('Mean Distance on Ray 1: %f with error %f mm\n', d_1_mean, d_1_diff);
else
    fprintf('Mean Distance on Ray 1: %f\n', d_1_mean);
end
    
for i = 2:length(detection)
   
    D_1 = detection(i).D_1;

    Rpx_dist = Helper.rayDistFromRayDist(detection(i).bs.origin, detection(1).bs.origin, detection(i).r, detection(1).r, D_1, d_1_mean);
 

%     Rpx = detection(i).bs.origin + Rpx_dist * detection(i).r;
%     plot3(Rpx(1), Rpx(2), Rpx(3), 'o', 'Color', detection(i).color); % plot sensor_on_ray_1

    if ~USE_REAL_RAY
        Rpx_dist_diff = norm(Rpx_dist - detection(i).Rp_dist) * 1000;
        fprintf('Mean Distance on Ray %d: %f with error %f mm\n', i, Rpx_dist, Rpx_dist_diff);
    else
        fprintf('Mean Distance on Ray %d: %f\n', i, Rpx_dist);
    end
end



Sf_1_mean = mean(Sf_1_suggestions, 2);

if ~USE_REAL_RAY
    Sf_1_diff = norm(Sf_1_mean - detection(1).Rp) * 1000; % same  as d_1_diff
else
    Sf_1_diff = 'n/a';
end


Pc_mean = mean(Pc_suggestions, 2);
plot3(Pc_mean(1), Pc_mean(2), Pc_mean(3), '^-', 'Color', 'c');
fprintf('Mean Tracker Center:\n');
disp(Pc_mean);


if ~USE_REAL_RAY
    Pc_diff = norm(Pc_mean - tracker_center) * 1000;
    fprintf('Mean Tracker Center error %f mm\n',  Pc_diff);
end


%% Plot End

% legend({'detection(1).bs.origin', 'detection(2).bs.origin', 'Rp_1', 'Rp_2', 'Sa_1', 'Sa_2'});

% view(0,0) % X & Z
% view(0,90) % X & Y
% zoom(50)
% view(90,0) % Y & Z

% rotate3d on


hold off