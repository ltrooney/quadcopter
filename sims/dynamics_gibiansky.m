% parameters from erkka http://sal.aalto.fi/publications/pdf-files/eluu11_public.pdf

m = 0.468;
g = 9.81;

Ix = 4.856e-3;
Iy = Ix;
Iz = 8.801e-3;
I = diag([Ix,Iy,Iz]);

L = .225;
b = 1.140e-7;   % drag constant
k = 2.980e-6;   % lift constant
kd = 0.25;      % drag force coefficients

errorIntegral = zeros(3,1);
    
% time

t0 = 0;
tf = 10;
dt = 0.005;
times = t0:dt:tf;
N = length(times);

% state data

linPosE = [0; 0; 5];    % [x, y, z]
linVelsE = zeros(3, 1);  % [u, v, w]
angles = zeros(3, 1);   % [roll, pitch, yaw]

dev = 100;
angVelsE = deg2rad(2 * dev * rand(3, 1) - dev);  % [p, q, r]

% simulation

global params
params = struct('m',m,'g',g,'I',I,'L',L,'b',b,'k',k,'dt',dt,'errorIntegral',errorIntegral);

state = zeros(16, N);

acc = zeros(3,N);

i = 1;

for t = times
       
    input = controller(angles,angVelsE,'pid');
   
    % convert angular velocities from E frame to B frame
    angVelsB = angVelsE2B(angVelsE, angles); % [p q r]
   
    % linear acceleration
    gravity = [0; 0; -g];
    R = rotate(angles);
    Tb = [0; 0; k*sum(input)];
    Fd = -kd * linVelsE;
   
    linAccE = gravity + ((1/m) * (R * Tb)) + Fd;
   
    acc(:,i) = R(:,3);
   
    % angular acceleration
    torques = [
        L * k * (input(1) - input(3));
        L * k * (input(2) - input(4));
        b * (input(1) - input(2) + input(3) - input(4))
    ];
    angAccB = I\(torques - cross(angVelsB, I * angVelsB));
   
    % update state
    state(:,i) = [linPosE; linVelsE; angles; angVelsB; input];
   
    angVelsB = angVelsB + (dt .* angAccB);
    angVelsE = angVelsB2E(angVelsB, angles);
    angles = angles + (dt .* angVelsE);
    linVelsE = linVelsE + (dt .* linAccE);
    linPosE = linPosE + (dt .* linVelsE);   
    
    i = i + 1;    
end

data = struct('x', state(1:3,:), 'theta', state(7:9,:), 'vel', state(4:6,:), ...
                    'angvel', state(10:12,:), 't', times, 'dt', dt, 'input', state(13:16,:));

x = state(1,:);
y = state(2,:);
z = state(3,:);
u = state(4,:);
v = state(5,:);
w = state(6,:);
roll = state(7,:);
pitch = state(8,:);
yaw = state(9,:);
p = state(10,:);
q = state(11,:);
r = state(12,:);

visualize_test(data);

plot2d = false;

if plot2d
    
    close all

    plotVelocityVectors = false;
    
    if plotVelocityVectors
        for i = 1:N
            if mod(i,50) == 0
                quiver(x(i),z(i),u(i),w(i));
                hold on
                quiver(y(i),z(i),v(i),w(i));
            end
        end
        hold off
    else
        subplot(2,2,1);
        plot(times,x,'DisplayName','x');
        hold on
        plot(times,y,'DisplayName','y');
        plot(times,z,'DisplayName','z');
        title('position');
        legend
        hold off
        
        subplot(2,2,2);
        plot(times,u,'DisplayName','u');
        hold on
        plot(times,v,'DisplayName','v');
        plot(times,w,'DisplayName','w');
        title('linear velocity');
        legend
        hold off
        
        subplot(2,2,3);
        plot(times,roll,'DisplayName','roll');
        hold on
        plot(times,pitch,'DisplayName','pitch');
        plot(times,yaw,'DisplayName','yaw');
        title('angles');
        legend
        hold off
        
        subplot(2,2,4);
        plot(times,p,'DisplayName','p');
        hold on
        plot(times,q,'DisplayName','q');
        plot(times,r,'DisplayName','r');
        title('angular velocity');
        legend
        hold off
        
    end
end


plot3d = false;

if plot3d
   
    for i = 1:N
        if mod(i,50) == 0
           plot3(x(i), y(i), z(i), '*r');
           %quiver3(x(i),y(i),z(i),u(i),v(i),w(i));
           axis([-5 5 -5 5 -3 20]);
           title(i * dt);
           grid on
           hold on
        end
        pause(0.005);
    end
    
end


function i = controller(angles, angVelsE, type)
    global params
   
    if nargin < 3
        type = 'pid';
    end
    
    Kp = 15;
    Kd = 5;
    Ki = 3;
    
    if strcmp(type,'pd')
        error = (Kd * angVelsE) + (Kp * angles);
    elseif strcmp(type,'pid')
        if max(abs(params.errorIntegral)) > 0.01
            params.errorIntegral(:) = 0; 
        end
    
        error = (Kd * angVelsE) + (Kp * angles) - (Ki * params.errorIntegral);
        params.errorIntegral = params.errorIntegral + (params.dt .* angles);
    else
        error = 0;
    end
    
    i = errorToInput(error,angles);
end

function i = errorToInput(error,angles)
    global params
    
    Ix = params.I(1,1);
    Iy = params.I(2,2);
    Iz = params.I(3,3);
    m = params.m;
    g = params.g;
    b = params.b;
    k = params.k;
    L = params.L;
        
    % solve for inputs
    input = zeros(4,1);
    thrustTerm = (m*g) / (4*k*cos(angles(2))*cos(angles(1)));
    input(1) = thrustTerm - ((2*b*error(1)*Ix) + (error(3)*Iz*k*L))/(4*b*k*L);
    input(2) = thrustTerm + ((error(3)*Iz)/(4*b)) - ((error(2)*Iy)/(2*k*L));
    input(3) = thrustTerm - ((-2*b*error(1)*Ix) + (error(3)*Iz*k*L))/(4*b*k*L);
    input(4) = thrustTerm + ((error(3)*Iz)/(4*b)) + ((error(2)*Iy)/(2*k*L));
    
    i = input;
end


function omega = angVelsE2B(angVelsE, angles)
    omega = T(angles) * angVelsE;
end

function thetadot = angVelsB2E(angVelsB, angles)
    thetadot = T(angles)\angVelsB;
end

function r = rotate(angles)
    c = cos(angles);
    s = sin(angles);
    r = [(c(1)*c(3))-(c(2)*s(1)*s(3)) (-c(3)*s(1))-(c(1)*c(2)*s(3)) s(2)*s(3);
        (c(1)*s(3))+(c(2)*c(3)*s(1)) (c(1)*c(2)*c(3))-(s(1)*s(3)) -c(3)*s(2);
        s(1)*s(2) c(1)*s(2) c(2)];
end

function t = T(angles)
    roll = angles(1);
    pitch = angles(2);
    t = [
        1 0 -sin(pitch); 
        0 cos(roll) cos(pitch)*sin(roll);
        0 -sin(roll) cos(pitch)*cos(roll)
    ];
end



% simulation code %



% Visualize the quadcopter simulation as an animation of a 3D quadcopter.
function h = visualize_test(data)
    % Create a figure with three parts. One part is for a 3D visualization,
    % and the other two are for running graphs of angular velocity and displacement.
    figure; plots = [subplot(3, 2, 1:4), subplot(3, 2, 5), subplot(3, 2, 6)];
    subplot(plots(1));
    pause;

    % Create the quadcopter object. Returns a handle to
    % the quadcopter itself as well as the thrust-display cylinders.
    [t,thrusts] = quadcopter;

    % Set axis scale and labels.
    axis([-10 30 -20 20 5 15]);
    zlabel('Height');
    title('Quadcopter Flight Simulation');

    % Animate the quadcopter with data from the simulation.
    animate(data, t, thrusts, plots);
end

% Animate a quadcopter in flight, using data from the simulation.
function animate(data, model, thrusts, plots)
    % Show frames from the animation. However, in the interest of speed,
    % skip some frames to make the animation more visually appealing.
    for t = 1:10:length(data.t)
        % The first, main part, is for the 3D visualization.
        subplot(plots(1));

        % Compute translation to correct linear position coordinates.
        dx = data.x(:, t);
        move = makehgtform('translate', dx);

        % Compute rotation to correct angles. Then, turn this rotation
        % into a 4x4 matrix represting this affine transformation.
        angles = data.theta(:, t);
        R = rotate(angles);
        R = [R zeros(3, 1); zeros(1, 3) 1];

        % Move the quadcopter to the right place, after putting it in the correct orientation.
        set(model,'Matrix', move * R);

        % Compute scaling for the thrust cylinders. The lengths should represent relative
        % strength of the thrust at each propeller, and this is just a heuristic that seems
        % to give a good visual indication of thrusts.
        scales = exp(data.input(:, t) / min(abs(data.input(:, t))) + 5) - exp(6) +  1.5;
        for i = 1:4
            % Scale each cylinder. For negative scales, we need to flip the cylinder
            % using a rotation, because makehgtform does not understand negative scaling.
            s = scales(i);
            if s < 0
                scalez = makehgtform('yrotate', pi)  * makehgtform('scale', [1, 1, abs(s)]);
            elseif s > 0
                scalez = makehgtform('scale', [1, 1, s]);
            end

            % Scale the cylinder as appropriate, then move it to
            % be at the same place as the quadcopter propeller.
            set(thrusts(i), 'Matrix', move * R * scalez);
        end

        % Update the drawing.      
        xmin = data.x(1,t)-20;
        xmax = data.x(1,t)+20;
        ymin = data.x(2,t)-20;
        ymax = data.x(2,t)+20;
        zmin = data.x(3,t)-5;
        zmax = data.x(3,t)+5;
        axis([xmin xmax ymin ymax zmin zmax]);
        drawnow;

        % Use the bottom two parts for angular velocity and displacement.
        subplot(plots(2));
        multiplot(data, data.angvel, t);
        xlabel('Time (s)');
        ylabel('Angular Velocity (rad/s)');
        title('Angular Velocity');

        subplot(plots(3));
        multiplot(data, data.theta, t);
        xlabel('Time (s)');
        ylabel('Angular Displacement (rad)');
        title('Angular Displacement');
    end
end

% Plot three components of a vector in RGB.
function multiplot(data, values, ind)
    % Select the parts of the data to plot.
    times = data.t(:, 1:ind);
    values = values(:, 1:ind);

    % Plot in RGB, with different markers for different components.
    plot(times, values(1, :), 'r-', times, values(2, :), 'g.', times, values(3, :), 'b-.');
    
    % Set axes to remain constant throughout plotting.
    xmin = min(data.t);
    xmax = max(data.t);
    ymin = 1.1 * min(min(values));
    ymax = 1.1 * max(max(values));
    %axis([xmin xmax ymin ymax]);
end

% Draw a quadcopter. Return a handle to the quadcopter object
% and an array of handles to the thrust display cylinders. 
% These will be transformed during the animation to display
% relative thrust forces.
function [h, thrusts] = quadcopter()
    % Draw arms.
    h(1) = prism(-5, -0.25, -0.25, 10, 0.5, 0.5);
    h(2) = prism(-0.25, -5, -0.25, 0.5, 10, 0.5);

    % Draw bulbs representing propellers at the end of each arm.
    [x,y,z] = sphere;
    x = 0.5 * x;
    y = 0.5 * y;
    z = 0.5 * z;
    h(3) = surf(x - 5, y, z, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(4) = surf(x + 5, y, z, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(5) = surf(x, y - 5, z, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(6) = surf(x, y + 5, z, 'EdgeColor', 'none', 'FaceColor', 'b');

    % Draw thrust cylinders.
    [x,y,z] = cylinder(0.1, 7);
    thrusts(1) = surf(x, y + 5, z, 'EdgeColor', 'none', 'FaceColor', 'm');
    thrusts(2) = surf(x + 5, y, z, 'EdgeColor', 'none', 'FaceColor', 'c');
    thrusts(3) = surf(x, y - 5, z, 'EdgeColor', 'none', 'FaceColor', 'm');
    thrusts(4) = surf(x - 5, y, z, 'EdgeColor', 'none', 'FaceColor', 'c');

    % Create handles for each of the thrust cylinders.
    for i = 1:4
        x = hgtransform;
        set(thrusts(i), 'Parent', x);
        thrusts(i) = x;
    end

    % Conjoin all quadcopter parts into one object.
    t = hgtransform;
    set(h, 'Parent', t);
    h = t;
end

% Draw a 3D prism at (x, y, z) with width w,
% length l, and height h. Return a handle to
% the prism object.
function h = prism(x, y, z, w, l, h)
    [X,Y,Z] = prism_faces(x, y, z, w, l, h);

    faces(1, :) = [4 2 1 3];
    faces(2, :) = [4 2 1 3] + 4;
    faces(3, :) = [4 2 6 8];
    faces(4, :) = [4 2 6 8] - 1;
    faces(5, :) = [1 2 6 5];
    faces(6, :) = [1 2 6 5] + 2;

    for i = 1:size(faces, 1)
        h(i) = fill3(X(faces(i, :)), Y(faces(i, :)), Z(faces(i, :)), 'r'); hold on;
    end

    % Conjoin all prism faces into one object.
    t = hgtransform;
    set(h, 'Parent', t);
    h = t;
end

% Compute the points on the edge of a prism at
% location (x, y, z) with width w, length l, and height h.
function [X,Y,Z] = prism_faces(x, y, z, w, l, h)
    X = [x x x x x+w x+w x+w x+w];
    Y = [y y y+l y+l y y y+l y+l];
    Z = [z z+h z z+h z z+h z z+h];
end
