
function [Rf_1, Rf_2, t_f, d_n_k, m_21, c_21, Dw_21] = getComponents(B_1, u, B_2, v, s_f, D_21)
    w_21 = B_2 - B_1;
%     x = w_21 - D_21 - s_f*u;
    
%     t_f = v\(d_S*q + s_f*u - w_21);
%     t_f = v\(D_21 + s_f*u - w_21);
%     t_f = v\(-x);
%     t_f = v\(s_f*u + D_21 - w_21);
%     t_f = v\(s_f*u) + v\(D_21 - w_21);
%     t_f = (v\u)*s_f + v\(D_21 - w_21);
%     t_f = (v\u)*s_f + v\(D_21 - w_21); % linear equation

    Dw_21 = D_21 - w_21;
    m_21 = (v\u);
    c_21 = v\(Dw_21);
    t_f = m_21*s_f + c_21; % linear equation
    
%     t_f = mldivide(v, (d_S*q + s_f*u - w_21));
%     t_f = (d_S*q + s_f*u - w_21).' * v;
%     t_f = (D_21 + s_f*u - w_21).' * v; 
    
    Rf_1 = B_1 + s_f*u;
%     Rf_2 = B_2 + t_f*v;
%     Rf_2 = B_2 + ((d_S*q + s_f*u - w_21).' * v)*v;
%     Rf_2 = B_2 + (d_S*q + s_f*u - w_21).' * v*v;
%     Rf_2 = B_2 + (D_21 + s_f*u - w_21).' * v*v;
    Rf_2 = B_2 + (D_21 + s_f*u - w_21).' * v*v;

%     k = (Rf_2 - Rf_1) / norm(Rf_2 - Rf_1);
    
%     d_n = norm(Rf_2 - Rf_1)
%     d_n = norm((B_2 + t_f*v) - (B_1 + s_f*u));
%     d_n = (Rf_2 - Rf_1).' * k;
%     d_n = (Rf_2 - Rf_1).' * k;
%     d_n = k\(Rf_2 - Rf_1);
    
%     d_n_k = d_n * k;
    d_n_k = Rf_2 - Rf_1;
    
%     d_S = norm(D_21);
%     q = D_21/norm(D_21);
    % d_S_q = d_S*q;
end
