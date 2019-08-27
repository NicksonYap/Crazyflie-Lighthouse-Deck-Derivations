% two random 3D vectors
p0 = randi(10,3,1)
p1 = randi(10,3,1)

% p0 = [1; 0; 0]
% p1 = [0; 1; 0]

% calculate cross and dot products
C = cross(p0, p1) ; 
D = dot(p0, p1) ;
NP0 = norm(p0) ; % used for scaling
if ~all(C==0) % check for colinearity    
    Z = [0 -C(3) C(2); C(3) 0 -C(1); -C(2) C(1) 0] ; 
    T = (eye(3) + Z + Z^2 * (1-D)/(norm(C)^2)) / NP0^2 % rotation matrix
else
    T = sign(D) * (norm(p1) / NP0) % orientation and scaling
end

% R is the rotation matrix from p0 to p1, so that (except for round-off errors) ...
p0_ = inv(T) * p1 % ... equals p0
p1_ = T * p0      % ... equals p1 

R = T/norm(T)