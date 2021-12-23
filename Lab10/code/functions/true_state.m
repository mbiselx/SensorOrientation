function [x] = true_state(t, r, w, alpha)
%TRUE_STATE Summary of this function goes here
%   Detailed explanation goes here

    p_t    = r * [ cos(w*t);  sin(w*t)];
    v_t    = r*w*[-sin(w*t);  cos(w*t)];
    azth_t = alpha + w*t;
    x      = [azth_t; v_t; p_t];

end

