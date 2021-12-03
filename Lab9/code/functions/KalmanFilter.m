function varargout = KalmanFilter(step, varargin)
%KALMANFILTER implementation of a Kalman filter of the course Sensor
%   Orientation, Lab 7. Before this function can be used as part of a
%   simulation loop, it must be initialized. 
%   NOTE: this function uses persistent variables, so only one instance of
%   KALMANFILTER may run at a time. 
%   Basic function signature: 
% [x, P] = KalmanFilter(step, varargin)
%   
% [] = KalmanFilter('init', x0, P0)
%   to initialize the function
%   param   x0      : initial state guess
%   param   P0      : initial uncertainty 
%
% [xtilde, Ptilde] = KalmanFilter('predict', F, G, Q, delta_t)
%   to run a 'prediction' step
%   param   F       : motion model
%   param   G       : noise model
%   param   Q       : uncertainty of the motion model
%   param   delta_t : time elapsed since last update
%   returns xtilde  : predicted state
%   returns Ptilde  : predicted uncertainty 
%
% [xhat, Phat]        = KalmanFilter('update', z, H, R)
% [xhat, Phat, innov] = KalmanFilter('update', z, H, R)
%   to run an 'update'step
%   param   z       : measurement
%   param   H       : measurement model
%   param   R       : uncertainty of the measurement model
%   returns xhat    : estimated state
%   returns Phat    : estimated uncertainty 
%   returns innov   : innovation 

% $Author: Michael Biselx $     $Date: 27-11-2021$      $Revision: 0.1 $ 
% Copyright: no? 

persistent xtilde xhat  % state 
persistent Ptilde Phat  % variance

    switch step
        case 'init'
            % KalmanFilter('init', x0, P0)
            xhat    = varargin{1}; % x0
            Phat    = varargin{2}; % P0

            xtilde  = xhat;
            Ptilde  = Phat;


        case 'predict'
            if isempty(xhat) || isempty(Phat)
                error("The function must be initialized before it can be used!")
            end

            % [xtilde, Ptilde] = KalmanFilter('predict', F, G, Q, delta_t)
            F       = varargin{1}; % continuous-time motion model
            G       = varargin{2}; % noise model
            Q       = varargin{3}; % uncertainty of the motion model
            delta_t = varargin{4}; % time since last update
            
            % discretize the motion model, because this is a computer program
            [Phi, Qk] = discretize_model(F, G, Q, delta_t);
            
            % predict
            xtilde  = Phi * xhat;
            Ptilde  = Phi * Phat * Phi.' + Qk;
            
            if nargout == 2
                varargout = {xtilde, Ptilde};
            else 
                varargout = {xtilde, Ptilde, Qk};
            end

        case 'update'
            if isempty(xtilde) || isempty(Ptilde)
                error("The function must be initialized before it can be used!")
            end

            % [xhat, Phat, innov] = KalmanFilter('update', z, H, R)
            z = varargin{1};    % measurement
            H = varargin{2};    % measurement model
            R = varargin{3};    % uncertainty of the measurement model

            % update step 
            K = (Ptilde * H.') / (H * Ptilde * H.' + R);
            innov = (z - H * xtilde);
            xhat = xtilde + K*innov;
            Phat = (eye(size(K,1), size(H,2)) - K*H) * Ptilde;

            if nargout == 2
                varargout = {xhat, Phat};
            else 
                varargout = {xhat, Phat, innov};
            end
        otherwise
            error("KalmanFilter: no such step: '%s'", step)

    end

end