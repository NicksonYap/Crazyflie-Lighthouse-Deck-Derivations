
function [t_f, Rf_1, Rf_2, v_error, error] = getResultAndErrors(B_1, u, B_2, v, s_f, D_21)
    [Rf_1, Rf_2, t_f, m_21, c_21, Dw_21] = getComponents(B_1, u, B_2, v, s_f, D_21);
    
    
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
    

    v_error = s_f*(m_21*v - u) + (c_21*v - Dw_21);
    error = power( norm( v_error ), 2);
end
