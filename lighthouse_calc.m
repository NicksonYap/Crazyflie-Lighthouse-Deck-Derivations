
%% Setup

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

RAYS_FAR_APART = false;
% RAYS_FAR_APART = true;

% Base Station positions in World Frame: (in Meters)
B_1 = [-1.789562; 5.251678; 2.641019];
B_2 = [1.734847; -4.475452; 2.665298];

if SINGLE_BASESTATION
    B_2 = B_1; % single base station
end

plot3(B_1(1), B_1(2), B_1(3), 'b^-'); % plot Base Station
text(B_1(1), B_1(2), B_1(3), '1', 'Color', 'g')

plot3(B_2(1), B_2(2), B_2(3), 'b^-'); % plot Base Station
text(B_2(1), B_2(2), B_2(3), '2', 'Color', 'r')

% Artibary point on a Ray in World Frame (in Meters), to generate the Ray Vector
% Rp_1 = [- 0.1; - 0.1; 1 - 0.1];
% Rp_2 = [+ 0.1; + 0.1; 1 + 0.1];

Rp_1 = [- 0.1; - 0.1; 1 - 0.1];

if RAYS_FAR_APART
    Rp_2 = Rp_1 + [0.2; 0 ; 0];
else
    Rp_2 = Rp_1 + [0.03; 0 ; 0];
end

plot3(Rp_1(1), Rp_1(2), Rp_1(3), 'k.-'); % plot Artibary point on Ray
plot3(Rp_2(1), Rp_2(2), Rp_2(3), 'k.-'); % plot Artibary point on Ray

% Unit Vector of Ray from Base Stations
u = (Rp_1 - B_1) / norm(Rp_1 - B_1);
v = (Rp_2 - B_2) / norm(Rp_2 - B_2);

% Simulated End of Ray
ray_length = 10;
R_1 = ray_length * u;
R_2 = ray_length * v;

quiver3(B_1(1), B_1(2), B_1(3), R_1(1), R_1(2), R_1(3), 'b'); % plot Rays
quiver3(B_2(1), B_2(2), B_2(3), R_2(1), R_2(2), R_2(3), 'b');

% distance between Base Stations
fprintf('Distance Between BS 2 & 1: %f\n', norm(B_2 - B_1));

%% 2 Base Stations, 1 Sensor (Find Shortest Segment between Rays)


[Sc_1, Sc_2, d_c, k, d_c_k] = shortestSegment(B_1, u, B_2, v);
fprintf('Shortest Distance Between Rays: %f\n', d_c);
% since Rays from Base Stations should surely touch the Sensor at the same
% point, the distance here should be zero.
% the extra distance may signify incorrect Base Station origins or lack of
% calibration

plot3(Sc_1(1), Sc_1(2), Sc_1(3), 'b.-', Sc_2(1), Sc_2(2), Sc_2(3), 'b.-'); % plot Shortest Segment points
quiver3(Sc_1(1), Sc_1(2), Sc_1(3), d_c_k(1), d_c_k(2), d_c_k(3), 'b'); % plot Shortest Segment vector

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

SIMULATE_SENSORS_ON_RAY = true;
SIMULATE_SENSORS_ON_RAY = false;

% simulate D vector as if Rays fall directly on both of Sensors (ignores sensor dimensions)

if SIMULATE_SENSORS_ON_RAY
    d_r_1 = 5.206868; % distance from ray 1
    d_r_2 = 5.750902; % distance from ray 2
 
    Sp_1 = B_1 + d_r_1 * u;
    Sp_2 = B_2 + d_r_2 * v;
 
    plot3(Sp_1(1), Sp_1(2), Sp_1(3), 'r.-', Sp_2(1), Sp_2(2), Sp_2(3), 'r.-'); % plot
 
    D = Sp_2 - Sp_1;
else
 
    %    Rp_1 = [0; 0; 0]; % from origin
 
    %    Rp_2 = [1; 0; 0];  % pointing in X
    %    Rp_2 = [0; 1; 0];  % pointing in Y
    %    Rp_2 = [0; 0; 1];  % pointing in Z
    %    Rp_2 = [-1; 0; 0];  % pointing in -X
    %    Rp_2 = [0; -1; 0];  % pointing in -Y
    %    Rp_2 = [0; 0; -1];  % pointing in -Z
 
    %    r = (Rp_2 - Rp_1) / norm(Rp_2 - Rp_1) % rotation unit vector
 
    Rp = [1; 0; 0]; % default direction, pointing in X
 
    yaw = 0; % degrees
    pitch = 0;
    roll = 0;
    rot_mat = angle2dcm(deg2rad(yaw), deg2rad(pitch), deg2rad(roll));
 
    sensor_on_ray_1 = 0;
    sensor_on_ray_2 = 2;
%     sensor_on_ray_2 = sensor_on_ray_1; % if only 1 sensor
 
    distance_between_sensors = norm(S(:, sensor_on_ray_2 + 1) - S(:, sensor_on_ray_1 + 1));
 
    %    D = distance_between_sensors*r;
 
    D = distance_between_sensors * rot_mat * Rp ;
 
end

% D = D*1.1; % introduce errors by scaling

fprintf('Assumed Distance Between Sensors: %f\n', norm(D));
fprintf('Sensor Vector: \n');
disp(D);

%%  2 Base Stations on 2 different Sensors (Best Fit of Segment between Rays)

w = B_2 - B_1;
Dw = D - w;
m = (v\u);
c = v\(Dw);

% 0 = s_f*( (v\u)*v - u ) + ( v\(Dw)*v - Dw );
% s_f = - ((v\u)*v - u) \ ( v\(Dw)*v - Dw );
s_f = - (m*v - u) \ (c*v - Dw);


t_f = m*s_f + c; % linear equation

Sf_1 = B_1 + s_f * u;
Sf_2 = B_2 + t_f * v;

d_f = norm(Sf_2 - Sf_1);

fprintf('Best Fit Segment Distance: %f\n', d_f);

q = (Sf_2 - Sf_1) / norm(Sf_2 - Sf_1);
d_f_q = d_f * q;

plot3(Sf_1(1), Sf_1(2), Sf_1(3), 'g.-', Sf_2(1), Sf_2(2), Sf_2(3), 'g.-');
quiver3(Sf_1(1), Sf_1(2), Sf_1(3), d_f_q(1), d_f_q(2), d_f_q(3), 'g');

Pf = Sf_1 + d_f_q / 2; % midpoint of Best Fit Segment
plot3(Pf(1), Pf(2), Pf(3), 'r.-'); % plot mid point

if ~ SIMULATE_SENSORS_ON_RAY
    Pf_1 = Pf - D / 2;
    Pf_2 = Pf + D / 2;
    plot3(Pf_1(1), Pf_1(2), Pf_1(3), 'g.-'); % plot sensor_on_ray_1
    plot3(Pf_2(1), Pf_2(2), Pf_2(3), 'm.-'); % plot sensor_on_ray_2
 
    text(Pf_1(1), Pf_1(2), Pf_1(3), num2str(sensor_on_ray_1), 'Color', 'g')
    text(Pf_2(1), Pf_2(2), Pf_2(3), num2str(sensor_on_ray_2), 'Color', 'm')
 
    norm(Pf_2 - Pf_1);
 
    S_ex = S; % excluded sensors
    S_ex(:, sensor_on_ray_1 + 1) = []; % exclude sensor_on_ray_1
    S_ex(:, sensor_on_ray_2 + 1 - 1) = []; % exclude sensor_on_ray_2
 
    for i = 1 : size(S_ex(), 2)
        Sx = S_ex(:, i);
        Dx = rot_mat * (Sx - S(:, sensor_on_ray_1 + 1));
        Px = Pf_1 + Dx;
     
        plot3(Px(1), Px(2), Px(3), 'k.-'); % plot sensor_on_ray_1
    end
 
end

segment_error = norm(d_f_q - D);

% fprintf('Magnitude of Segment Error (in Meters): %f\n', segment_error);
fprintf('Magnitude of Segment Error (in mm): %f\n', segment_error * 1000);

%% Plot End

% legend({'B_1', 'B_2', 'Rp_1', 'Rp_2', 'Sa_1', 'Sa_2'});

hold off