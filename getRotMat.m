
function R = getRotMat(a, b)
    % ref: https://math.stackexchange.com/a/2161631
    % assuming a & b are unit vectors
    
    u = a/norm(a);                      % a and b must be column vectors
    v = b/norm(b);                      % of equal length
    N = length(u);
    S = reflection( eye(N), v+u );      % S*u = -v, S*v = -u 
    R = reflection( S, v );             % v = R*u
    
%     R = R/norm(R); % not needed
    % test: 
%     a_ = inv(R) * b
%     b_ = R * a
    
end

function v = reflection(u, n)
    % Reflection of u on hyperplane n.
    %
    % u can be a matrix. u and v must have the same number of rows.
    v = u - 2 * n * (n'*u) / (n'*n);
end