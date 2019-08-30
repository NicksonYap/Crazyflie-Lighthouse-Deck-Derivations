classdef Helper
    % Static functions ref: https://stackoverflow.com/a/25055533/3553367
    
    methods(Static)
        function B = pInv(A) % equivalent to build-in pinv()
            % ref: https://www.youtube.com/watch?v=pTUfUjIQjoE
            A_t = transpose(A);
            B = inv(A_t*A)*A_t;
        end
        
    end
end

