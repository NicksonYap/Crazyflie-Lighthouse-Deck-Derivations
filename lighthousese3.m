
%% Basestation Configuration

O = zeros(3, 3);

ID = eye(3, 3);
ID_x = ID(1, :);
ID_y = ID(2, :);
ID_z = ID(3, :);

quiver3(O(1, :), O(2, :), O(3, :), ID_x, ID_y, ID_z, 'k'); % plot origin
grid on
daspect([1 1 1])
hold on

SINGLE_BASESTATION = false;
% SINGLE_BASESTATION = true;


% Base Station positions in World Frame: (in Meters)
B_1 = [-1.789562; 5.251678; 2.641019];
B_2 = [1.734847; -4.475452; 2.665298];
B_3 = [-1.759562; -4.505452; 2.635298];
% B_3 = [-1.759562; -2.005452; 2.635298];

if SINGLE_BASESTATION
    B_2 = B_1; % single base station
    B_3 = B_1; % single base station
end

plot3(B_1(1), B_1(2), B_1(3), 'b^-'); % plot Base Station
text(B_1(1), B_1(2), B_1(3), '1', 'Color', 'r')

plot3(B_2(1), B_2(2), B_2(3), 'b^-'); % plot Base Station
text(B_2(1), B_2(2), B_2(3), '2', 'Color', 'r')

plot3(B_3(1), B_3(2), B_3(3), 'b^-'); % plot Base Station
text(B_3(1), B_3(2), B_3(3), '3', 'Color', 'r')


%%  Tracker Sensor Configuration

% Sensor Positions Relative to Tracker

sd_x = 3 / 100; % x-distance between sensors in Meters
sd_y = 1.5 / 100; % y-distance between sensors in Meters

S = [];
S = [S, [- sd_x / 2; sd_y / 2; 0]]; % Sensor 0
S = [S, [- sd_x / 2; - sd_y / 2; 0]]; % Sensor 1
S = [S, [sd_x / 2; sd_y / 2; 0]]; % Sensor 2
S = [S, [sd_x / 2; - sd_y / 2; 0]]; % Sensor 3

for i = 1 : size(S(), 2)
    Si = S(:, i);
    plot3(Si(1), Si(2), Si(3), 'k.-'); % plot sensors
    text(Si(1), Si(2), Si(3), num2str(i - 1), 'Color', 'k')
end

% simulate D vector as if Rays fall directly on both of Sensors (ignores sensor dimensions)

% format long

sensor_on_ray_1 = 0;
sensor_on_ray_2 = 2;
sensor_on_ray_3 = 1;

yaw = 45; % degrees
pitch = 45;
roll = 45;
rot_mat = angle2dcm(deg2rad(yaw), deg2rad(pitch), deg2rad(roll));

tracker_center = [0; 0; 1]; % center of tracker
% tracker_center = P_sf - rot_mat * S(:, sensor_on_ray_1 + 1);

% P_sf = [0; 0; 1]; % center of sensor_on_ray_1
P_sf = tracker_center + rot_mat * S(:, sensor_on_ray_1 + 1); % sensor_on_ray_1 as reference point

sensor_vector_21 = (S(:, sensor_on_ray_2 + 1) - S(:, sensor_on_ray_1 + 1)); % relative to sensor_on_ray_1
sensor_vector_31 = (S(:, sensor_on_ray_3 + 1) - S(:, sensor_on_ray_1 + 1)); % relative to sensor_on_ray_1

D_21 = rot_mat * sensor_vector_21;
D_31 = rot_mat * sensor_vector_31;

%% Test Rotation Matrix




%%

% D_21 = D_21*1.1; % introduce errors by scaling
% D_31 = D_31*0.9; % introduce errors by scaling

plot3(tracker_center(1), tracker_center(2), tracker_center(3), 'r.-'); % plot center of drone at rays
text(tracker_center(1), tracker_center(2), tracker_center(3), 'c', 'Color', 'r')

fprintf('Assumed Distance Between Sensors at BS2 Ray & BS1 Ray: %f\n', norm(D_21));
fprintf('Sensor Vector: \n');
disp(D_21);

quiver3(P_sf(1), P_sf(2), P_sf(3), D_21(1), D_21(2), D_21(3), 'r'); % plot D_21 on rays

fprintf('Assumed Distance Between Sensors at BS3 Ray & BS1 Ray: %f\n', norm(D_31));
fprintf('Sensor Vector: \n');
disp(D_31);

quiver3(P_sf(1), P_sf(2), P_sf(3), D_31(1), D_31(2), D_31(3), 'r'); % plot D_31 on rays
    

%% Rays Setup

RAYS_ON_SENSOR = true
% RAYS_ON_SENSOR = false

if RAYS_ON_SENSOR
    
    % Artibary point on a Ray in World Frame (in Meters), to generate the Ray Vector

    Rp_1 = P_sf;
    Rp_2 = P_sf + D_21;
    Rp_3 = P_sf + D_31;
    
else

    % Artibary point on a Ray in World Frame (in Meters), to generate the Ray Vector
    % Rp_1 = [- 0.1; - 0.1; 1 - 0.1];
    % Rp_2 = [+ 0.1; + 0.1; 1 + 0.1];

    Rp_1 = [- 0.1; - 0.1; 1 - 0.1];

    RAYS_FAR_APART = false;
    RAYS_FAR_APART = true;

    if RAYS_FAR_APART
        Rp_2 = Rp_1 + [0.2; 0 ; 0];
        Rp_3 = Rp_1 + [0.1; 0.1 ; 0.1];
    else
        Rp_2 = Rp_1 + [0.03; 0 ; 0];
        Rp_3 = Rp_1 + [0.0075; 0.0075 ; 0.0075];
    end

end

plot3(Rp_1(1), Rp_1(2), Rp_1(3), 'k.-'); % plot Artibary point on Ray
plot3(Rp_2(1), Rp_2(2), Rp_2(3), 'k.-'); % plot Artibary point on Ray
plot3(Rp_3(1), Rp_3(2), Rp_3(3), 'k.-'); % plot Artibary point on Ray

% Unit Vector of Ray from Base Stations
u = (Rp_1 - B_1) / norm(Rp_1 - B_1);
v = (Rp_2 - B_2) / norm(Rp_2 - B_2);
g = (Rp_3 - B_3) / norm(Rp_3 - B_3);

% Simulated End of Ray
ray_length = 10;
R_1 = ray_length * u;
R_2 = ray_length * v;
R_3 = ray_length * g;

quiver3(B_1(1), B_1(2), B_1(3), R_1(1), R_1(2), R_1(3), 'b'); % plot Rays
quiver3(B_2(1), B_2(2), B_2(3), R_2(1), R_2(2), R_2(3), 'b');
quiver3(B_3(1), B_3(2), B_3(3), R_3(1), R_3(2), R_3(3), 'b');

% distance between Base Stations
fprintf('Distance Between BS 2 & 1: %f\n', norm(B_2 - B_1));
fprintf('Distance Between BS 3 & 1: %f\n', norm(B_3 - B_1));
fprintf('Distance Between BS 3 & 2: %f\n', norm(B_2 - B_3));

%% 2 Base Stations, 1 Sensor (Find Shortest Segment between Rays)


[Sc_1, Sc_2, d_c, k, d_c_k] = shortestSegment(B_1, u, B_2, v);
fprintf('Shortest Distance Between Rays from BS 1 & 2: %f\n', d_c);
plot3(Sc_1(1), Sc_1(2), Sc_1(3), 'b.-', Sc_2(1), Sc_2(2), Sc_2(3), 'b.-'); % plot Shortest Segment points
quiver3(Sc_1(1), Sc_1(2), Sc_1(3), d_c_k(1), d_c_k(2), d_c_k(3), 'b'); % plot Shortest Segment vector

[Sc_1, Sc_2, d_c, k, d_c_k] = shortestSegment(B_1, u, B_3, g);
fprintf('Shortest Distance Between Rays from BS 1 & 3: %f\n', d_c);
plot3(Sc_1(1), Sc_1(2), Sc_1(3), 'b.-', Sc_2(1), Sc_2(2), Sc_2(3), 'b.-'); % plot Shortest Segment points
quiver3(Sc_1(1), Sc_1(2), Sc_1(3), d_c_k(1), d_c_k(2), d_c_k(3), 'b'); % plot Shortest Segment vector

[Sc_1, Sc_2, d_c, k, d_c_k] = shortestSegment(B_2, v, B_3, g);
fprintf('Shortest Distance Between Rays from BS 2 & 3: %f\n', d_c);
plot3(Sc_1(1), Sc_1(2), Sc_1(3), 'b.-', Sc_2(1), Sc_2(2), Sc_2(3), 'b.-'); % plot Shortest Segment points
quiver3(Sc_1(1), Sc_1(2), Sc_1(3), d_c_k(1), d_c_k(2), d_c_k(3), 'b'); % plot Shortest Segment vector

% since Rays from Base Stations should surely touch the Sensor at the same
% point, the distance here should be zero.
% the extra distance may signify incorrect Base Station origins or lack of
% calibration

%%  2 Base Stations on 2 different Sensors (Best Fit of Segment between Rays)


%% Plot Calc

step_size = 0.05;
offset = 0;
offset = -step_size/2;
increments = 80;
for i=1:increments
%     disp(i)
    increment = (i-increments/2)*step_size + offset;

    s_f = norm(Rp_1 - B_1) + increment;
    
    [t_f, Rf21_1, Rf21_2, v_error_21, error_21] = getResultAndErrors(B_1, u, B_2, v, s_f, D_21);
    d21 = Rf21_2 - Rf21_1;
    
    [r_f, Rf31_1, Rf31_2, v_error_31, error_31] = getResultAndErrors(B_1, u, B_3, g, s_f, D_31);
    d31 = Rf31_2 - Rf31_1;
    
    
    fprintf('%f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \n', increment, s_f, t_f, error_21, v_error_21(1), v_error_21(2), v_error_21(3), r_f, error_31, v_error_31(1), v_error_31(2), v_error_31(3));
    
    
    plot3(Rf21_1(1), Rf21_1(2), Rf21_1(3), 'g.-');
    plot3(Rf21_2(1), Rf21_2(2), Rf21_2(3), 'g.-');
    quiver3(Rf21_1(1), Rf21_1(2), Rf21_1(3), d21(1), d21(2), d21(3), 'g');

    plot3(s_f, t_f, error_21, 'g.-');
    
    
    
    plot3(Rf31_1(1), Rf31_1(2), Rf31_1(3), 'c.-');
    plot3(Rf31_1(1), Rf31_1(2), Rf31_1(3), 'c.-');
    quiver3(Rf31_1(1), Rf31_1(2), Rf31_1(3), d31(1), d31(2), d31(3), 'c');

    plot3(s_f, r_f, error_31, 'c.-');
    
end



%% Plot End

% legend({'B_1', 'B_2', 'Rp_1', 'Rp_2', 'Sa_1', 'Sa_2'});

hold off