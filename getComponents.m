
function [Rf_1, Rf_2, t_f, m, c, Dw] = getComponents(B_1, u, B_2, v, s_f, D)
    w = B_2 - B_1;
%     x = w - D - s_f*u;
    
%     t_f = v\(d_S*q + s_f*u - w);
%     t_f = v\(D + s_f*u - w);
%     t_f = v\(-x);
%     t_f = v\(s_f*u + D - w);
%     t_f = v\(s_f*u) + v\(D - w);
%     t_f = (v\u)*s_f + v\(D - w);
%     t_f = (v\u)*s_f + v\(D - w); % linear equation

    Dw = D - w;
    m = (v\u);
    c = v\(Dw);
    t_f = m*s_f + c; % linear equation
    
%     t_f = mldivide(v, (d_S*q + s_f*u - w));
%     t_f = (d_S*q + s_f*u - w).' * v;
%     t_f = (D + s_f*u - w).' * v; 
    
    Rf_1 = B_1 + s_f*u;
%     Rf_2 = B_2 + t_f*v;
%     Rf_2 = B_2 + ((d_S*q + s_f*u - w).' * v)*v;
%     Rf_2 = B_2 + (d_S*q + s_f*u - w).' * v*v;
%     Rf_2 = B_2 + (D + s_f*u - w).' * v*v;
    Rf_2 = B_2 + (D + s_f*u - w).' * v*v;

%     k = (Rf_2 - Rf_1) / norm(Rf_2 - Rf_1);
    
%     d_n = norm(Rf_2 - Rf_1)
%     d_n = norm((B_2 + t_f*v) - (B_1 + s_f*u));
%     d_n = (Rf_2 - Rf_1).' * k;
%     d_n = (Rf_2 - Rf_1).' * k;
%     d_n = k\(Rf_2 - Rf_1);
    
%     d_n_k = d_n * k;
    
%     d_S = norm(D);
%     q = D/norm(D);
    % d_S_q = d_S*q;
end
