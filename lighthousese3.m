
%% Setup

O = zeros(3,3);

ID = eye(3,3);
ID_x = ID(1,:);
ID_y = ID(2,:);
ID_z = ID(3,:);

quiver3(O(1,:), O(2,:), O(3,:), ID_x, ID_y, ID_z, 'k'); % plot origin
grid on
daspect([1 1 1])
hold on

B_1 = [-1.789562; 5.251678; 2.641019];
B_2 = [1.734847; -4.475452; 2.665298];
B_3 = [-1.789562; -4.475452; 2.665298];


plot3(B_1(1), B_1(2), B_1(3), 'b^-'); % plot Base Station
text(B_1(1), B_1(2), B_1(3), '1', 'Color', 'g')

plot3(B_2(1), B_2(2), B_2(3), 'b^-'); % plot Base Station
text(B_2(1), B_2(2), B_2(3), '2', 'Color', 'r')

plot3(B_3(1), B_3(2), B_3(3), 'b^-'); % plot Base Station
text(B_3(1), B_3(2), B_3(3), '3', 'Color', 'r')


% Artibary point on a Ray in World Frame (in Meters), to generate the Ray Vector

S_1 = [-0.1; -0.1; 1-0.1];
S_2 = [+0.1; +0.1; 1+0.1];
S_3 = [+0.1; -0.1; 1-0.1];

u = (S_1 - B_1) / norm(S_1 - B_1);
v = (S_2 - B_2) / norm(S_2 - B_2);
g = (S_3 - B_3) / norm(S_3 - B_3);

R_1 = 10 * u;
R_2 = 10 * v;
R_3 = 10 * g;

quiver3(B_1(1), B_1(2), B_1(3), R_1(1), R_1(2), R_1(3), 'b'); % plot Rays
quiver3(B_2(1), B_2(2), B_2(3), R_2(1), R_2(2), R_2(3), 'b');
quiver3(B_3(1), B_3(2), B_3(3), R_3(1), R_3(2), R_3(3), 'b');

fprintf('Distance Between BS 2 & 1: %f\n', norm(B_2 - B_1));
fprintf('Distance Between BS 3 & 1: %f\n', norm(B_3 - B_1));
fprintf('Distance Between BS 3 & 2: %f\n', norm(B_2 - B_3));

%% Shortest Segment between two Rays

[Sc_1, Sc_2, d_c, k, d_c_k] = shortest_segment(B_1, u, B_2, v);
fprintf('Shortest Distance Between Rays from BS 2 & 1: %f\n', d_c);
    
%% Calc


d_S = norm(S_2 - S_1);
q = (S_2 - S_1) / norm(S_2 - S_1);
d_S_q = d_S*q;
% D = d_S*q;
D = S_2 - S_1;

fprintf('Assumed Distance Between Sensors: %f\n', d_S);
% fprintf('Sensor Vector: [%f, %f, %f]\n', q(1), q(2), q (3));
fprintf('Sensor Vector: \n');
disp(D);

d_BS1 = norm(S_1 - B_1);
d_BS2 = norm(S_2 - B_2);
fprintf('Sensor_1 Distance from BS_1: %f\n', d_BS1);
fprintf('Sensor_2 Distance from BS_2: %f\n', d_BS2);

% Sp_1 = B_1 + d_BS1*u
% Sp_2 = B_2 + d_BS2*v

w_0 = B_2 - B_1;

s_n = d_BS1;
% t_n = v\(d_S*q + s_n*u - w_0);
% t_n = mldivide(v, (d_S*q + s_n*u - w_0));
% t_n = (d_S*q + s_n*u - w_0).' * v ;
t_n = (D + s_n*u - w_0).' * v; 
    
Sn_1 = B_1 + s_n*u;
Sn_2 = B_2 + t_n*v;



%% override

% d_BS1 = norm(Sc_1 - B_1)
% d_S = d_c
% q = k

%% Plot Shortest Segment between Rays

plot3(Sc_1(1), Sc_1(2), Sc_1(3), 'c.-', Sc_2(1), Sc_2(2), Sc_2(3), 'c.-');
quiver3(Sc_1(1), Sc_1(2), Sc_1(3), d_c_k(1), d_c_k(2), d_c_k(3), 'c');

%% Plot Calc

quiver3(S_1(1), S_1(2), S_1(3), d_S_q(1), d_S_q(2), d_S_q(3), 'r');

Sn_1_ = Sn_1;
Sn_2_ = Sn_2;

step_size = 0.05;
offset = 0;
offset = -step_size/2;
increments = 40;
for i=1:increments
%     disp(i)
    increment = (i-increments/2)*step_size + offset;

    s_n = d_BS1 + increment;
    
    x = w_0 - D - s_n*u;
    
%     t_n = v\(d_S*q + s_n*u - w_0);
%     t_n = v\(D + s_n*u - w_0);
%     t_n = v\(-x);
%     t_n = v\(s_n*u + D - w_0);
%     t_n = v\(s_n*u) + v\(D - w_0);
%     t_n = (v\u)*s_n + v\(D - w_0);
%     t_n = (v\u)*s_n + v\(D - w_0); % linear equation

    D_w = D - w_0;
    m = (v\u);
    c = v\(D_w);
    t_n = m*s_n + c; % linear equation
    
%     t_n = mldivide(v, (d_S*q + s_n*u - w_0));
%     t_n = (d_S*q + s_n*u - w_0).' * v;
%     t_n = (D + s_n*u - w_0).' * v; 
    
    Sn_1 = B_1 + s_n*u;
%     Sn_2 = B_2 + t_n*v;
%     Sn_2 = B_2 + ((d_S*q + s_n*u - w_0).' * v)*v;
%     Sn_2 = B_2 + (d_S*q + s_n*u - w_0).' * v*v;
%     Sn_2 = B_2 + (D + s_n*u - w_0).' * v*v;
    Sn_2 = B_2 + (D + s_n*u - w_0).' * v*v;

    k = (Sn_2 - Sn_1) / norm(Sn_2 - Sn_1);
    
%     d_n = norm(Sn_2 - Sn_1)
%     d_n = norm((B_2 + t_n*v) - (B_1 + s_n*u));
%     d_n = (Sn_2 - Sn_1).' * k;
    d_n = (Sn_2 - Sn_1).' * k;
    
    d_n_k = d_n * k;
    
%     d_error = d_n - d_S;
    d_error = abs(d_n - d_S);
%     d_error = sqrt(power(d_n - d_S, 2));
    
%     r_error = k - q;
    r_error = norm(k - q);
    t_error = d_error*r_error;
    
%     error = norm(d_n_k - d_S_q);
%     error = power(norm(d_n_k - d_S_q), 2);
%     error = power(norm(d_n_k - D), 2);
%     error = power( norm( w_0 +  t_n*v - s_n*u - D ), 2);
%     error = power( norm( w_0 +  ( (D + s_n*u - w_0).' * v )*v - s_n*u - D ), 2);
%     error = power( norm( w_0 +  (D + s_n*u - w_0).' * v*v - s_n*u - D ), 2);
%     error = power( norm( w_0 +  v\(D + s_n*u - w_0) *v - s_n*u - D ), 2);

%     error = power( norm( x + v\(-x) *v  ), 2);
%     error = power( norm( x + t_n *v  ), 2);
%     error = power( norm( x + ((v\u)*s_n + v\(D - w_0)) *v  ), 2);
%     error = power( norm( x + (v\u)*v*s_n + v\(D - w_0)*v ), 2);
%     error = power( norm( x + m*v*s_n + v\(D - w_0)*v ), 2);
    
%     v_error = x + m*v*s_n + c*v;
%     v_error = w_0 - D - s_n*u + m*v*s_n + c*v;
%     v_error = m*v*s_n - s_n*u + c*v +  w_0 - D;
%     v_error = s_n*m*v - s_n*u + (c*v +  w_0 - D);
%     v_error = s_n*(m*v - u) + (c*v +  w_0 - D);
    
%     m_v = (m*v - u);
%     c_v = (c*v +  w_0 - D);
%     m_v = ((v\u)*v - u);
%     c_v = (v\(D - w_0)*v +  w_0 - D);
%     v_error = s_n*m_v + c_v;
    
    v_error = s_n*(m*v - u) + (c*v - D_w);
    
    error = power( norm( v_error ), 2);
    
%     fprintf('%f, %f, %f, %f, %f, %f, %f \n', increment, s_n, t_n, d_n, d_error, r_error, t_error);
%     fprintf('%f \t %f \t %f \t %f \n', increment, s_n, t_n, error);
    fprintf('%f \t %f \t %f \t %f \t %f \t %f \t %f \n', increment, s_n, t_n, error, v_error(1), v_error(2), v_error(3));
    
    plot3(Sn_1(1), Sn_1(2), Sn_1(3), 'g.-', Sn_2(1), Sn_2(2), Sn_2(3), 'g.-');
    quiver3(Sn_1(1), Sn_1(2), Sn_1(3), d_n_k(1), d_n_k(2), d_n_k(3), 'g');

    plot3(s_n, t_n, error, 'g.-');
    
    dist1 = norm(Sn_1 - Sn_1_);
    dist2 = norm(Sn_2 - Sn_2_);
    
    Sn_1_ = Sn_1;
    Sn_2_ = Sn_2;
end


D_w = D - w_0;
m = (v\u);
c = v\(D_w);

% t_f = m*s_f + c; % linear equation

a = (m*v - u);
b = (c*v - D_w);

% v_error = s_f*(m*v - u) + (c*v - D_w); % linear equation
% v_error = s_f*a + b; % linear equation

% s_no_error_linear = - (m*v - u) \ (c*v - D_w); % assuming v_error = 0, find s_f

% v_error_sequared = power( norm(s_f*a + b), 2); % quadratic equation for reference only, to show derivation
% v_error_sequared_diffed = 2*a*(a*s_f + b); %squared and differentiated by s_f
% s_no_error_quadratic = - b / a; % assuming v_error_sequared_diffed = 0, find s_f
% s_no_error_quadratic = - a \ b
% s_no_error_quadratic = - (m*v - u) \ (c*v - D_w); % same as s_no_error_linear

    
% syms W Dd S U V X
% e = power( norm( W +  V\(Dd + S*U - W) *V - S*U - Dd ), 2);
% diff(e, S)

% quiver3(S_2(1), S_2(2), S_2(3),-res(1),-res(2),-res(3), 'r');

%% Plot End

% legend({'B_1', 'B_2', 'S_1', 'S_2', 'Sa_1', 'Sa_2'});

hold off