function varargout = KalmanFilter(step, varargin)
%KALMANFILTER implementation of a Kalman filter of the course Sensor
%   Orientation, Lab 7. Before this function can be used as part of a
%   simulation loop, it must be initialized. 
%   NOTE: this ufnction uses persistant variables, so only one instance of
%   KALMANFILTER may run at a time. 
%   Basic function signature: 
% [x, P] = KalmanFilter(step, varargin)
%   
% KalmanFilter('init', x0, P0)
%   to initialize the function
%   param   x0      : initial state guess
%   param   P0      : initial uncertainty 
%
% [xtilde, Ptilde] = KalmanFilter('predict', delta_t, F, Q);
%   to run a 'prediction' step
%   param   delta_t : time elapsed since last update
%   param   F       : motion model
%   param   Q       : uncertainty of the motion model
%   returns xtilde  : predicted state
%   returns Ptilde  : predicted uncertainty 
%
% [xhat, Phat, innov] = KalmanFilter('update', z, R, H)
%   to run an 'update'step
%   param   z       : measurement
%   param   R       : uncertainty of the measurement model
%   param   H       : measurement model
%   returns xhat    : estimated state
%   returns Phat    : estimated uncertainty 
%   returns innov   : innovation 

% $Author: Michael Biselx $     $Date: 23-11-2021$      $Revision: 0.0 $ 
% Copyright: no? 

persistent xtilde xhat  % state 
persistent Ptilde Phat  % variance

    switch step
        case 'init'
            xhat    = varargin{1}; % x0
            xtilde  = xhat;
            Phat    = varargin{2}; % P0
            Ptilde  = Phat;


        case 'predict'
            if isempty(xhat) || isempty(Phat)
                error("The function must be initialized before it can be used!")
            end

            delta_t = varargin{1}; % time since last update
            F       = varargin{2}; % continuous dynamic model
            Q       = varargin{3}; % motion model noise
            

            Phi     = expm(F * delta_t);

            % !!placeholder 
            qvn = Q(1,1);
            qve = Q(2,2);
            Qk      = [(delta_t^3*qvn)/3,                 0, (delta_t^2*qvn)/2,                 0;
                                       0, (delta_t^3*qve)/3,                 0, (delta_t^2*qve)/2;
                       (delta_t^2*qvn)/2,                 0,       delta_t*qvn,                 0;
                                       0, (delta_t^2*qve)/2,                 0,       delta_t*qve];
            % \placeholder 
            
            xtilde  = Phi * xhat;
            Ptilde  = Phi * Phat * Phi.' + Qk;

            varargout = {xtilde, Ptilde};

        case 'update'
            if isempty(xtilde) || isempty(Ptilde)
                error("The function must be initialized before it can be used!")
            end

            z = varargin{1};
            R = varargin{2};
            H = varargin{3};
            
            K = (Ptilde * H.') / (H * Ptilde * H.' + R);
            innov = (z - H * xtilde);
            xhat = xtilde + K*innov;
            Phat = (eye(size(K,1), size(H,2)) - K*H) * Ptilde;

            varargout = {xhat, Phat, innov};
        otherwise
            error("KalmanFilter: no such step: '%s'", step)

    end

end