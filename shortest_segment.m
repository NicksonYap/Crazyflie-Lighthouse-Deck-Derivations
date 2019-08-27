
function [Sc_1, Sc_2, d_c, k, d_c_k] = shortest_segment(B_1, u, B_2, v)
    w_0 = B_2 - B_1;

    a = dot(v, v);
    b = dot(v, u);
    c = dot(u, u);
    d = dot(v, w_0);
    e = dot(u, w_0);

    det = (a*c-b*b);
    t_c = (b*e-c*d) / det;
    s_c = (a*e-b*d) / det;

    Sc_1 = B_1 + s_c*u;
    Sc_2 = B_2 + t_c*v;

    d_c = norm(Sc_2 - Sc_1);
    k = (Sc_2 - Sc_1) / norm(Sc_2 - Sc_1);
    d_c_k = d_c * k;
end
