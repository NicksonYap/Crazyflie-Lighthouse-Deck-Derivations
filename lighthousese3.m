
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
RAYS_FAR_APART = true;

% Base Station positions in World Frame: (in Meters)
B_1 = [-1.789562; 5.251678; 2.641019];
B_2 = [1.734847; -4.475452; 2.665298];
B_3 = [-1.759562; -4.505452; 2.635298];

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

% Artibary point on a Ray in World Frame (in Meters), to generate the Ray Vector
% Rp_1 = [- 0.1; - 0.1; 1 - 0.1];
% Rp_2 = [+ 0.1; + 0.1; 1 + 0.1];

Rp_1 = [- 0.1; - 0.1; 1 - 0.1];

if RAYS_FAR_APART
    Rp_2 = Rp_1 + [0.2; 0 ; 0];
    Rp_3 = Rp_1 + [0.1; 0.1 ; 0.1];
else
    Rp_2 = Rp_1 + [0.03; 0 ; 0];
    Rp_3 = Rp_1 + [0.0075; 0.0075 ; 0.0075];
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


[Sc_1, Sc_2, d_c, k, d_c_k] = shortest_segment(B_1, u, B_2, v);
fprintf('Shortest Distance Between Rays from BS 1 & 2: %f\n', d_c);
plot3(Sc_1(1), Sc_1(2), Sc_1(3), 'b.-', Sc_2(1), Sc_2(2), Sc_2(3), 'b.-'); % plot Shortest Segment points
quiver3(Sc_1(1), Sc_1(2), Sc_1(3), d_c_k(1), d_c_k(2), d_c_k(3), 'b'); % plot Shortest Segment vector

[Sc_1, Sc_2, d_c, k, d_c_k] = shortest_segment(B_1, u, B_3, g);
fprintf('Shortest Distance Between Rays from BS 1 & 3: %f\n', d_c);
plot3(Sc_1(1), Sc_1(2), Sc_1(3), 'b.-', Sc_2(1), Sc_2(2), Sc_2(3), 'b.-'); % plot Shortest Segment points
quiver3(Sc_1(1), Sc_1(2), Sc_1(3), d_c_k(1), d_c_k(2), d_c_k(3), 'b'); % plot Shortest Segment vector

[Sc_1, Sc_2, d_c, k, d_c_k] = shortest_segment(B_2, v, B_3, g);
fprintf('Shortest Distance Between Rays from BS 2 & 3: %f\n', d_c);
plot3(Sc_1(1), Sc_1(2), Sc_1(3), 'b.-', Sc_2(1), Sc_2(2), Sc_2(3), 'b.-'); % plot Shortest Segment points
quiver3(Sc_1(1), Sc_1(2), Sc_1(3), d_c_k(1), d_c_k(2), d_c_k(3), 'b'); % plot Shortest Segment vector

% since Rays from Base Stations should surely touch the Sensor at the same
% point, the distance here should be zero.
% the extra distance may signify incorrect Base Station origins or lack of
% calibration

%%  2 Base Stations on 2 different Sensors (Best Fit of Segment between Rays)
%% Calc

% re-use the points for array as Tracker sensor detections
% so the error should be zero

D_21 = Rp_2 - Rp_1;
fprintf('Assumed Distance Between Sensors at BS2 Ray & BS1 Ray: %f\n', norm(D_21));
fprintf('Sensor Vector: \n');
disp(D_21);

D_31 = Rp_3 - Rp_1;
fprintf('Assumed Distance Between Sensors at BS3 Ray & BS1 Ray: %f\n', norm(D_31));
fprintf('Sensor Vector: \n');
disp(D_31);


%% Plot Calc

step_size = 0.05;
offset = 0;
offset = -step_size/2;
increments = 40;
for i=1:increments
%     disp(i)
    increment = (i-increments/2)*step_size + offset;

    s_f = norm(Rp_1 - B_1) + increment;
    
    w_21 = B_2 - B_1;
    x = w_21 - D_21 - s_f*u;
    
%     t_n = v\(d_S*q + s_f*u - w_21);
%     t_n = v\(D_21 + s_f*u - w_21);
%     t_n = v\(-x);
%     t_n = v\(s_f*u + D_21 - w_21);
%     t_n = v\(s_f*u) + v\(D_21 - w_21);
%     t_n = (v\u)*s_f + v\(D_21 - w_21);
%     t_n = (v\u)*s_f + v\(D_21 - w_21); % linear equation

    D_w = D_21 - w_21;
    m = (v\u);
    c = v\(D_w);
    t_n = m*s_f + c; % linear equation
    
%     t_n = mldivide(v, (d_S*q + s_f*u - w_21));
%     t_n = (d_S*q + s_f*u - w_21).' * v;
%     t_n = (D_21 + s_f*u - w_21).' * v; 
    
    Sn_1 = B_1 + s_f*u;
%     Sn_2 = B_2 + t_n*v;
%     Sn_2 = B_2 + ((d_S*q + s_f*u - w_21).' * v)*v;
%     Sn_2 = B_2 + (d_S*q + s_f*u - w_21).' * v*v;
%     Sn_2 = B_2 + (D_21 + s_f*u - w_21).' * v*v;
    Sn_2 = B_2 + (D_21 + s_f*u - w_21).' * v*v;

    k = (Sn_2 - Sn_1) / norm(Sn_2 - Sn_1);
    
%     d_n = norm(Sn_2 - Sn_1)
%     d_n = norm((B_2 + t_n*v) - (B_1 + s_f*u));
%     d_n = (Sn_2 - Sn_1).' * k;
    d_n = (Sn_2 - Sn_1).' * k;
    
    d_n_k = d_n * k;
    
    d_S = norm(D_21);
    q = D_21/norm(D_21);
    % d_S_q = d_S*q;
    
    
%     error = norm(d_n_k - d_S_q);
%     error = power(norm(d_n_k - d_S_q), 2);
%     error = power(norm(d_n_k - D_21), 2);
%     error = power( norm( w_21 +  t_n*v - s_f*u - D_21 ), 2);
%     error = power( norm( w_21 +  ( (D_21 + s_f*u - w_21).' * v )*v - s_f*u - D_21 ), 2);
%     error = power( norm( w_21 +  (D_21 + s_f*u - w_21).' * v*v - s_f*u - D_21 ), 2);
%     error = power( norm( w_21 +  v\(D_21 + s_f*u - w_21) *v - s_f*u - D_21 ), 2);

%     error = power( norm( x + v\(-x) *v  ), 2);
%     error = power( norm( x + t_n *v  ), 2);
%     error = power( norm( x + ((v\u)*s_f + v\(D_21 - w_21)) *v  ), 2);
%     error = power( norm( x + (v\u)*v*s_f + v\(D_21 - w_21)*v ), 2);
%     error = power( norm( x + m*v*s_f + v\(D_21 - w_21)*v ), 2);
    
%     v_error = x + m*v*s_f + c*v;
%     v_error = w_21 - D_21 - s_f*u + m*v*s_f + c*v;
%     v_error = m*v*s_f - s_f*u + c*v +  w_21 - D_21;
%     v_error = s_f*m*v - s_f*u + (c*v +  w_21 - D_21);
%     v_error = s_f*(m*v - u) + (c*v +  w_21 - D_21);
    
%     m_v = (m*v - u);
%     c_v = (c*v +  w_21 - D_21);
%     m_v = ((v\u)*v - u);
%     c_v = (v\(D_21 - w_21)*v +  w_21 - D_21);
%     v_error = s_f*m_v + c_v;
    
    v_error = s_f*(m*v - u) + (c*v - D_w);
    
    error = power( norm( v_error ), 2);
    
    
    fprintf('%f \t %f \t %f \t %f \t %f \t %f \t %f \n', increment, s_f, t_n, error, v_error(1), v_error(2), v_error(3));
    
    plot3(Sn_1(1), Sn_1(2), Sn_1(3), 'g.-', Sn_2(1), Sn_2(2), Sn_2(3), 'g.-');
    quiver3(Sn_1(1), Sn_1(2), Sn_1(3), d_n_k(1), d_n_k(2), d_n_k(3), 'g');

    plot3(s_f, t_n, error, 'g.-');
    
end



%% Plot End

% legend({'B_1', 'B_2', 'Rp_1', 'Rp_2', 'Sa_1', 'Sa_2'});

hold off