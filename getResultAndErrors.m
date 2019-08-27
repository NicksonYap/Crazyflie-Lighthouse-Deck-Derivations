
function [t_f, Rf_1, Rf_2, v_error, error] = getResultAndErrors(B_1, u, B_2, v, s_f, D_21)
    [Rf_1, Rf_2, t_f, m_21, c_21, Dw_21] = getComponents(B_1, u, B_2, v, s_f, D_21);
    v_error = s_f*(m_21*v - u) + (c_21*v - Dw_21);
    error = power( norm( v_error ), 2);
end
