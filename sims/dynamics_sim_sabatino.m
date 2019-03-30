% parameters

I_x = 8.1e-3;
I_y = I_x;
I_z = 14.2e-3;
b = 53.1e-6;    % thrust factor; b = T/(prop speed)^2
d = 1.1e-6;     % drag factor
l = .235;       % distance from propeller to center (m)
m = 1;          % mass (kg)
g = 9.81;       % gravity

controller_inputs = [1050, 1000, 1000, 1000]; % throttle, roll, pitch, yaw

min_pwm = 1000;
max_pwm = 2000;
battery_voltage = 11.1;
max_prop_speed = (battery_voltage * 980) / 60;  % rad/s
%omega = max_prop_speed * (controller_inputs - min_pwm) / (max_pwm - min_pwm);


omega = [100; 100; 100; 100];

A = [zeros(1,3) 1 zeros(1,8);
     zeros(1,4) 1 zeros(1,7);
     zeros(1,5) 1 zeros(1,6);
     zeros(3,12);
     0 -g zeros(1,10);
     g zeros(1,11);
     zeros(1,12);
     zeros(1,6) 1 zeros(1,5);
     zeros(1,7) 1 zeros(1,4);
     zeros(1,8) 1 zeros(1,3)];

B = [zeros(3,4);
     0 1/I_x 0 0;
     0 0 1/I_y 0;
     0 0 0 1/I_z;
     zeros(2,4);
     1/m 0 0 0;
     zeros(3,4)];
 
D = [zeros(3,6);
      zeros(1,3) 1/I_x 0 0;
      zeros(1,4) 1/I_y 0;
      zeros(1,5) 1/I_z;
      1/m zeros(1,5);
      0 1/m zeros(1,4);
      0 0 1/m zeros(1,3);
      zeros(3,6)];
  
% y = [X Y Z]
C = [zeros(1,9) 1 0 0;
     zeros(1,10) 1 0;
     zeros(1,11) 1];
 
% construct state space
sys = ss(A,B,eye(12),[]);
sys.InputName = {'thrust', 'torque_x', 'torque_y', 'torque_z'};
sys.InputGroup.thrust = 1;
sys.InputGroup.torques = [2 3 4];
sys.InputUnit = {'N', 'Nm', 'Nm', 'Nm'};
sys.OutputGroup.angVelE = [1 2 3];  % E frame angular velocities
sys.OutputGroup.linAccB = [4 5 6];  % B frame linear accelerations
sys.OutputGroup.angAccB = [7 8 9];  % B frame angular accelerations
sys.OutputGroup.linVelE = [10 11 12];   % E frame linear velocities
sys.OutputName = { 
    'roll','pitch','yaw', ...
    'p','q','r', ...
    'u','v','w', ...
    'x','y','z'};
sys.OutputUnit = {
    'rad','rad','rad', ...
    'm/s','m/s','m/s', ...
    'm/s','m/s','m/s', ...
    'm','m','m'};

% time
dt = 1;
final = 5;
t = 0:dt:final;
num_time_steps = length(t);

% position
lin_pos_E = [0 0 2];    % linear position: [X Y Z]
ang_pos_E = [0 0 0];    % angular position: [phi theta psi]

% velocity
V_B = [0 0 0];    % linear velocity: [u v w]
w_B = [0 0 0];    % angular velocity: [p q r]

% initial state
x0 = transpose([ang_pos_E w_B V_B lin_pos_E]);

% construct u
% input vector
% u = [b*(omega(1)^2 + omega(2)^2 + omega(3)^2 + omega(4)^2);
%       b*l*(omega(3)^2 - omega(1)^2);
%       b*l*(omega(4)^2 - omega(2)^2);
%       d*(omega(2)^2 + omega(4)^2 - omega(1)^2 - omega(3)^2)];
u = zeros(num_time_steps,4);
for i = 1:num_time_steps
    u(i,:) = [m*g 0 0 0];
end


[y,t,x] = lsim(sys,u,t,x0);

roll    = x(:,1);
pitch   = x(:,2);
yaw     = x(:,3);
p       = x(:,4);
q       = x(:,5);
r       = x(:,6);
u       = x(:,7);
v       = x(:,8);
w       = x(:,9);
x_pos   = x(:,10);
y_pos   = x(:,11);
z_pos   = x(:,12);


plot(t, u, t, v, t, w);

% for k = 1 : num_time_steps
%    plot(t(k), x(k,10), t, x(k,11), t, x(k,12)); 
%    drawnow;
%    legend('x','y','z');
%    
%    pause(1);
% end
% hold off;






