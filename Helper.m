classdef Helper
    % Static functions ref: https://stackoverflow.com/a/25055533/3553367
    
    methods(Static)
        function B = pInv(A) % equivalent to build-in pinv()
            % ref: https://www.youtube.com/watch?v=pTUfUjIQjoE
            A_t = transpose(A);
            B = inv(A_t*A)*A_t;
        end
        
        function R = planeRot(a_x, a_y, b_x, b_y)
            % ref: https://math.stackexchange.com/a/1876662/699426
            
            
            % checks:
            
            % floating point equality ref: https://www.mathworks.com/help/matlab/ref/eq.html
            tol = eps(0.5);
            
            if abs(norm(a_x) - norm(b_x)) >= tol || abs(norm(a_y) - norm(b_y)) >= tol
                error('Magnitudes mismatch')
            end
            
            if abs( dot(a_x, a_y) - dot(b_x, b_y)) >= tol 
                error('Dot products mismatch')
            end
            
            if abs( norm(cross(a_x, a_y)) - norm(cross(b_x, b_y)) ) >= tol 
                error('Magnitude of cross products mismatch')
            end
            
            a_z = cross(a_y, a_y); % cannot be cross(a_x, a_x)
            b_z = cross(b_y, b_y); % cannot be cross(b_x, b_x)
            
            A = [a_x, a_y, a_z];
            B = [b_x, b_y, b_z];
            
%             R = B*pinv(A);
            R = B*Helper.pInv(A);
            
        end
        
        function R = vectorRot(a, b)
            % ref: https://math.stackexchange.com/a/2161631
            % assuming a & b are unit vectors
            
%             R = Helper.planeRot(a, zeros(3, 1), b, zeros(3, 1) ); %equivalent, only if norm(a) == norm(b)
%             return

            u = a/norm(a);                      % a and b must be column vectors
            v = b/norm(b);                      % of equal length
            N = length(u);


            function v = reflection(u, n)
                % Reflection of u on hyperplane n.
                %
                % u can be a matrix. u and v must have the same number of rows.
                v = u - 2 * n * (n'*u) / (n'*n);
            end


            S = reflection( eye(N), v+u );      % S*u = -v, S*v = -u 
            R = reflection( S, v );             % v = R*u

        %     R = R/norm(R); % not needed
            % test: 
        %     a_ = inv(R) * b
        %     b_ = R * a

        end
        
        function testRot(sensor_vector_21, sensor_vector_31, rot_mat, D_21, D_31)
            
            format long

            rot_mat_21 = D_21/sensor_vector_21;
            rot_mat_31 = D_31/sensor_vector_31;

            diff = norm(rot_mat_31 - rot_mat_21);

            rot_mat_21 = Helper.vectorRot(sensor_vector_21, D_21);
            rot_mat_31 = Helper.vectorRot(sensor_vector_31, D_31);

            diff = norm(rot_mat_31 - rot_mat_21);

            % vrrotvec(sensor_vector_21, D_21)

            rel_rot_mat = Helper.vectorRot(sensor_vector_31, sensor_vector_21)
            rel_rotted_mat = Helper.vectorRot(D_31, D_21)


            % ref for rotation/transformation matrix equivalent 3x3, not 4x4 = https://www.mathworks.com/matlabcentral/answers/254132-how-to-calculate-transformation-matrix-x-a-y#
            % rel_trans_mat = sensor_vector_31*pinv(sensor_vector_21)
            % rel_transed_mat = D_31*pinv(D_21)

            % rot_mat\(D_31*pinv(D_21)) == (sensor_vector_31*pinv(sensor_vector_21)) / rot_mat
            % (rot_mat\(D_31*pinv(D_21)))*rot_mat == ((sensor_vector_31*pinv(sensor_vector_21)) / rot_mat) * rot_mat
            % (rot_mat\(D_31*pinv(D_21)))*rot_mat == sensor_vector_31*pinv(sensor_vector_21)
            % (rot_mat\(D_31*pinv(D_21)))*rot_mat == rel_trans_mat
            % (rot_mat\rel_transed_mat)*rot_mat == rel_trans_mat
            % rot_mat\rel_transed_mat == rel_trans_mat / rot_mat
            % rot_mat\rel_transed_mat == rel_trans_mat / rot_mat
            % rot_mat.' * rel_transed_mat == rel_trans_mat / rot_mat
            % transpose(rot_mat) * rel_transed_mat == rel_trans_mat * inv(rot_mat)
            % transpose(rot_mat) * rel_transed_mat * rot_mat == rel_trans_mat




            xx = rot_mat\(D_31*pinv(D_21)) 
            yy = (sensor_vector_31*Helper.pInv(sensor_vector_21)) / rot_mat

            diff = norm(xx - yy)
            

        end
        
        function testRot2(sensor_vector_21, sensor_vector_31, rot_mat, D_21, D_31)
            
            format long
            
            rot_mat
            rot_mat_rev = Helper.planeRot(sensor_vector_31, sensor_vector_21, D_31, D_21)
            % rot_mat & rot_mat_rev may differ, however the result will be the same, and rot_mat_rev is cleaner

            D_21
            D_21_ = rot_mat_rev * sensor_vector_21
            
            D_31
            D_31_ = rot_mat_rev * sensor_vector_31

            

        end
    end
end

