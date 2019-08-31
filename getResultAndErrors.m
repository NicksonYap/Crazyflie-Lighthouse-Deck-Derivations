
function [t_f, Rf_1, Rf_2, v_error, error] = getResultAndErrors(B_1, u, B_2, v, s_f, D)
    [Rf_1, Rf_2, t_f, m, c, Dw] = getComponents(B_1, u, B_2, v, s_f, D);
    
    
%     error = norm(d_n_k - d_S_q);
%     error = power(norm(d_n_k - d_S_q), 2);
%     error = power(norm(d_n_k - D), 2);
%     error = power( norm( w +  t_n*v - s_f*u - D ), 2);
%     error = power( norm( w +  ( (D + s_f*u - w).' * v )*v - s_f*u - D ), 2);
%     error = power( norm( w +  (D + s_f*u - w).' * v*v - s_f*u - D ), 2);
%     error = power( norm( w +  v\(D + s_f*u - w) *v - s_f*u - D ), 2);

%     error = power( norm( x + v\(-x) *v  ), 2);
%     error = power( norm( x + t_n *v  ), 2);
%     error = power( norm( x + ((v\u)*s_f + v\(D - w)) *v  ), 2);
%     error = power( norm( x + (v\u)*v*s_f + v\(D - w)*v ), 2);
%     error = power( norm( x + m*v*s_f + v\(D - w)*v ), 2);
    
%     x = w - D - s_f*u;

%     v_error = x + m*v*s_f + c*v;
%     v_error = w - D - s_f*u + m*v*s_f + c*v;
%     v_error = m*v*s_f - s_f*u + c*v +  w - D;
%     v_error = s_f*m*v - s_f*u + (c*v +  w - D);
%     v_error = s_f*(m*v - u) + (c*v +  w - D);
    
%     m_v = (m*v - u);
%     c_v = (c*v +  w - D);
%     m_v = ((v\u)*v - u);
%     c_v = (v\(D - w)*v +  w - D);
%     v_error = s_f*m_v + c_v;
    

    v_error = s_f*(m*v - u) + (c*v - Dw);
%     0 = s_f*(m*v - u) + (c*v - Dw);
%     s_f = (m*v - u) \ (c*v - Dw);
    
    error = power( norm( v_error ), 2);
end
