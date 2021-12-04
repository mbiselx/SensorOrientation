% This is just a script to set the initialization parameters

p0            = [     r;   0];  % [m]
sigma_p0      = [    10;  10];  % [m]
v0            = [     0; r*w];  % [m/s]
sigma_v0      = [   0.1; 0.1];  % [m/s]

f_GPS         = 1;              % [Hz]      GPS update rate
dt_GPS        = 1/f_GPS;        % [s]       GPS sampling period 
sigma_GPS     = [    .5;  .5];  % [m]       GPS measurement noise

R = diag(sigma_GPS.^2); 

switch LabV 
    case 7  % constant velocity motion model
        disp("Executing Lab 7")
        sigma_mov_v   = [0.05; 0.05];   % [m/s^2/sqrt(Hz)]
        
        % system equations 
        F  = diag(ones(1,2), 2);
        G = [zeros(2); eye(2)];
        Q = diag([sigma_mov_v.^2]);

        % linear measurement equations z = h(x)
        h  = @(x) x(1:2,:);
        H  = [eye(2), zeros(2)];

        % transformation to observation space [p; v] = m(x)
        m  = @(x) x;
        M  = @(x) eye(4);

        % initial guess
        x0 = [p0; v0];
        P0 = diag([sigma_p0; sigma_v0].^2);
        
    case 8  % constant acceleration motion model
        disp("Executing Lab 8")
        a0            = [-r*w^2;   0];  % [m/s^2]
        sigma_a0      = [0.1; 0.1];     % [m/s^2]
        sigma_mov_a   = [0.01; 0.01];   % [m/s^3/sqrt(Hz)]
        
        % system equations 
        F  = diag(ones(1,4), 2);
        G = [zeros(4,2); eye(2)];
        Q = diag([sigma_mov_a.^2]);

        % linear measurement equations z = h(x)
        h  = @(x) x(1:2,:);
        H  = eye(2, 6);

        % transformation to observation space [p; v] = m(x)
        m  = @(x) x(1:4,:);
        M  = @(x) eye(4, size(x,1));
        
        % initial guess
        x0 = [p0; v0; a0];
        P0 = diag([sigma_p0; sigma_v0; sigma_a0].^2);

    case 9  % uniform circular model
        disp("Executing Lab 9")
        r0            = sqrt(sum(p0.^2));
        psi0          = atan2(p0(2), p0(1));
        w0            = sqrt(sum(v0.^2))/r0;

        sigma_mov_r   = 0.02;           % [m/s/sqrt(Hz)]
        sigma_mov_w   = w0/20;          % [rad/s/sqrt(Hz)]

        % system equations 
        F = zeros(3); F(2,3) = 1;
        G = [1 0; 0 0; 0 1];
        Q = diag(([sigma_mov_r; sigma_mov_w]).^2);

        % non-linear measurement equations z = h(x)
        h  = @(x) (x(1) * [cos(x(2)); ...
                           sin(x(2))]);
        H  = @(x) [cos(x(2)) -x(1)*sin(x(2)) 0; ...
                   sin(x(2))  x(1)*cos(x(2)) 0];
        
        % transformation to observation space [p; v] = m(x)
        m  = @(x) [   x(1) * cos(x(2));         ...
                      x(1) * sin(x(2));         ...
                    - x(1) * x(3) * sin(x(2));  ...
                      x(1) * x(3) * cos(x(2))]; 
        M  = @(x) [       cos(x(2))       -x(1)*sin(x(2))                0; ...
                          sin(x(2))        x(1)*cos(x(2))                0; ...
                    -x(3)*sin(x(2))  -x(1)*x(3)*cos(x(2))  -x(1)*sin(x(2)); ...
                     x(3)*cos(x(2))  -x(1)*x(3)*sin(x(2))   x(1)*cos(x(2))];

        % initial guess
        x0 = [r0; psi0; w0];
        P0 = diag([10 (pi/100) (pi/1000)].^2);

    otherwise
        error("unknown Lab version")
end




