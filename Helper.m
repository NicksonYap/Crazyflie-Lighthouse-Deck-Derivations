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
            
%             Ainv = inv(A)
%             Apinv = pinv(A)
%             Ahpinv = Helper.pInv(A)
           
%             R = B*inv(A);
            R = B*pinv(A);
%             R = B*Helper.pInv(A); %why does this not work??
            
%             B_ = R*A; %test
%             B_ == B;
            
        end
        
        function u = unit(a)
            u = a/norm(a);
            return
        end
        
        function R = vectorRot2(a, b)
            % ref: https://math.stackexchange.com/a/2161631
            % assuming a & b are unit vectors
            
            R = Helper.planeRot(Helper.unit(a), zeros(3, 1), Helper.unit(b), zeros(3, 1) ); %equivalent, only if norm(a) == norm(b)
            return
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

            rel_rot_mat_a = Helper.vectorRot(sensor_vector_31, sensor_vector_21)
            rel_rot_mat_a = Helper.vectorRot2(sensor_vector_31, sensor_vector_21)
            
            rel_rot_mat_b = Helper.vectorRot(D_31,D_21)
            rel_rot_mat_b = Helper.vectorRot2(D_31,D_21)
            

        end
        
        function [Sf_2, Sf_1, segment_error] = bestFitBetweenRays(B_2, B_1, v, u, D)
            
%             D = R*s;
            w = B_2 - B_1;
            Dw = D - w;
%             m = v\u;
            m = transpose(u)*v;
%             c = v\(Dw);
            c = transpose(Dw)*v;

            % 0 = s_f*( (v\u)*v - u ) + ( v\(Dw)*v - Dw );
            % s_f = - ((v\u)*v - u) \ ( v\(Dw)*v - Dw );
            s_f = - (m*v - u) \ (c*v - Dw);

            % D_21 = R*s_21;
            % w_21 = B_2 - B_1;
            % Dw_21 = D_21 - w_21;
            % s_f = - ((v\u)*v - u) \ ( v\(Dw_21)*v - Dw_21 );
            % s_f = - ((v\u)*v - u) \ ( v\( D_21 - w_21 )*v - ( D_21 - w_21 ) );
            % s_f = - ((v\u)*v - u) \ ( v\( R*s_21 - w_21 )*v - ( R*s_21 - w_21 ) );

            % D_31 = R*s_31;
            % w_31 = B_3 - B_1;
            % Dw_31 = D_31 - w_31;
            % s_f = - ((g\u)*g - u) \ ( g\(Dw_31)*g - Dw_31 );
            % s_f = - ((g\u)*g - u) \ ( g\( D_31 - w_31 )*g - ( D_31 - w_31 ) );
            % s_f = - ((g\u)*g - u) \ ( g\( R*s_31 - w_31 )*g - ( R*s_31 - w_31 ) );

            % s_f = s_f;
            % - ((v\u)*v - u) \ ( v\( R*s_21 - w_21 )*v - ( R*s_21 - w_21 ) ) = - ((g\u)*g - u) \ ( g\( R*s_31 - w_31 )*g - ( R*s_31 - w_31 ) )
            % ((v\u)*v - u) \ ( v\( R*s_21 - w_21 )*v - ( R*s_21 - w_21 ) ) = ((g\u)*g - u) \ ( g\( R*s_31 - w_31 )*g - ( R*s_31 - w_31 ) )

            t_f = m*s_f + c; % linear equation

            Sf_1 = B_1 + s_f * u;
            Sf_2 = B_2 + t_f * v;
            
            segment_error = norm( (Sf_2 - Sf_1) - D);

            % fprintf('Magnitude of Segment Error (in Meters): %f\n', segment_error);
%             fprintf('Magnitude of Segment Error (in mm): %f\n', segment_error * 1000);

        end
        
        function plotSensors(S, R, Sf_2, Sf_1, sensor_on_ray_2, sensor_on_ray_1)
            
            D = Sf_2 - Sf_1;

%             d_f = norm(D);
%             fprintf('Best Fit Segment Distance: %f\n', d_f);
            
            plot3(Sf_1(1), Sf_1(2), Sf_1(3), 'g.-', Sf_2(1), Sf_2(2), Sf_2(3), 'g.-');
            quiver3(Sf_1(1), Sf_1(2), Sf_1(3), D(1), D(2), D(3), 'g');

            Pf = Sf_1 + D / 2; % midpoint of Best Fit Segment
            plot3(Pf(1), Pf(2), Pf(3), 'r.-'); % plot mid point

            
            Pf_1 = Pf - D / 2;
            Pf_2 = Pf + D / 2;
            plot3(Pf_1(1), Pf_1(2), Pf_1(3), 'g.-'); % plot sensor_on_ray_1
            plot3(Pf_2(1), Pf_2(2), Pf_2(3), 'm.-'); % plot sensor_on_ray_2

            text(Pf_1(1), Pf_1(2), Pf_1(3), num2str(sensor_on_ray_1), 'Color', 'g')
            text(Pf_2(1), Pf_2(2), Pf_2(3), num2str(sensor_on_ray_2), 'Color', 'm')

            norm(Pf_2 - Pf_1);

            S_ex = S; % excluded sensors
            S_ex(:, sensor_on_ray_1 + 1) = []; % exclude sensor_on_ray_1
            S_ex(:, sensor_on_ray_2 + 1 - 1) = []; % exclude sensor_on_ray_2

            for i = 1 : size(S_ex(), 2)
                Sx = S_ex(:, i);
                Dx = R * (Sx - S(:, sensor_on_ray_1 + 1));
                Px = Pf_1 + Dx;

                plot3(Px(1), Px(2), Px(3), 'k.-'); % plot sensor_on_ray_1
            end


        end
        
        function R = deg2dcm(yaw, pitch, roll)
           R = angle2dcm(deg2rad(yaw), deg2rad(pitch), deg2rad(roll)) ;
        end
        
    end
end

