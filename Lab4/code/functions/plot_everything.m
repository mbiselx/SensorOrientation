function fig = plot_everything(varargin)
%PLOT_EVERYTHING plots everything in the simulation for the Sensor
%   Orientation Lab 4. It can be used with different combinatins of input 
%   arguments, however they must be arranged in cells as shown below.
%   PLOT_EVERYTHING returns a handle to the figure used for plots, for making GIFs
%
% [FigureHandle] = plot_everything({trajectory})
% [FigureHandle] = plot_everything({trajectory, true_pose})
% [FigureHandle] = plot_everything({trajectory, true_pose, estimated_pose})
% [FigureHandle] = plot_everything({trajectory, true_pose}, {t, fb1, fb2}, figN)
% [FigureHandle] = plot_everything({trajectory, true_pose, estimated_pose}, {t, fb1, fb2}, figN)
% [FigureHandle] = plot_everything({trajectory, true_pose}, {t, fb1, fb2, wb}, figN)
% [FigureHandle] = plot_everything({trajectory, true_pose, estimated_pose}, {t, fb1, fb2, wb}, figN)

% $Author: Michael Biselx $     $Date: 30-10-2021$      $Revision: 0.0 $ 
% Copyright: no? 


    if nargin > 2
        fig = figure(varargin{3});
    else
        fig = figure(100);
    end

    if nargin == 1 % only plot trajectory

        h = plot(varargin{1}{1}(2,:), varargin{1}{1}(1,:), '--b'); hold on
        labels = {'trajectory'};
        if length(varargin{1}) > 1
            h = [h, plot(varargin{1}{2}(2), varargin{1}{2}(1), 'bo', 'MarkerSize', 5.5)];
            labels{2} = 'true position';
        end
        if length(varargin{1}) > 2
            h = [h, plot(varargin{1}{3}(2), varargin{1}{3}(1), 'ro', 'MarkerSize', 6.0)];
            labels{3} = 'est. position';
        end
        hold off;
        axis equal 
        xlabel('E [m]'), ylabel('N [m]')
        legend(h, labels)
        title('Trajectory in Mapping Frame')

    else
        nb_sensors = length(varargin{2})-1;
        idx = reshape(1:(2*nb_sensors), 2,[]);

        subplot(nb_sensors, 2, idx(1,:)) % plot trajectory
            h = plot(varargin{1}{1}(2,:), varargin{1}{1}(1,:), '--b'); hold on
            labels = {'trajectory'};
            if length(varargin{1}) > 1
                h = [h, plot(varargin{1}{2}(2), varargin{1}{2}(1), 'bo', 'MarkerSize', 5.5)];
                labels{2} = 'true position';
            end
            if length(varargin{1}) > 2
                h = [h, plot(varargin{1}{3}(2), varargin{1}{3}(1), 'ro', 'MarkerSize', 6.0)];
                labels{3} = 'est. position';
            end
            hold off;
            axis equal 
            xlabel('E [m]'), ylabel('N [m]')
            legend(h, labels)
            title('Trajectory in Mapping Frame')
        
        subplot(nb_sensors,2, 2)
            plot(varargin{2}{1}, varargin{2}{2}, 'b')
            grid on
            axis tight
            xlabel('t [s]'), ylabel('f_1^b [m/s^2]')
            title("Sensor Values")
        if nb_sensors > 1
        subplot(nb_sensors,2, 4)
            plot(varargin{2}{1}, varargin{2}{3}, 'b')
            grid on
            axis tight
            xlabel('t [s]'), ylabel('f_2^b [m/s^2]')
        end
        if nb_sensors > 2
        subplot(nb_sensors,2,6)
            plot(varargin{2}{1}, varargin{2}{4}, 'b')
            grid on
            axis tight
            xlabel('t [s]'), ylabel('\omega^b [°/s]')
        end

    end

    drawnow

end

