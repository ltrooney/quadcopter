% parameters
I_xx = 0;   % inertia xx (N m s^2)
I_yy = 0;   % inertia yy (N m s^2)
I_zz = 0;   % inertia zz (N m s^2)
m = .5;     % mass (kg)
g = 9.81;   % gravity (m/s^2)
b = 0;
l = 0;
d = 0;
J_TP = 104*10e-6;

% calculated variables
F_E = [0; 0; -m*g];   % gravitational force in inertial frame
E_B = [ zeros(2, 4);    % movement matrix in B
        b   b   b   b;
        0  -b*l 0   b*l;
       -b*l 0   b*l 0;
       -d   d   -d  d];
E_H = [rotate(angles) zeros(3); zeros(3) eye(3)] * E_B; % movement matrix in H

omega = [0;0;0;0];  % propeller speed vector

% position
lin_pos_E = [0 0 0];    % [X Y Z]
ang_pos_E = [0 0 0];    % [phi theta psi]
pos_E = transpose([lin_pos_E ang_pos_E]);   % [X Y Z phi theta psi]

% velocity
V_B = [0 0 0];    % linear velocity: [u v w]
w_B = [0 0 0];    % angular velocity: [p q r]
vel_B = transpose([V_B w_B]);       % body velocity vector: [u v w p q r]
vel_E = J_theta(ang_pos_E) * vel_B; % inertial frame velocity vector

R = rotate(ang_pos_E);

% body frame acceleration
I = [I_xx 0 0; 0 I_yy 0; 0 0 I_zz];  % inertia tensor
M_B = [m*eye(3) zeros(3); zeros(3) I]; % inertia matrix
G_B = [transpose(R)*F_E; zeros(3,1)];  % gravitational vector
O_B = O(vel_B);
U_B = E_B*(omega^2);  % movement vector in B
acc_B = M_B\(-C_B(vel_B)*vel_B + G_B + O_B*omega + U_B);   % [u' v' w' p' q' r']

% hybrid frame acceleration
vel_H = vel_B;
M_H = M_B;
C_H = [zeros(3, 6); zeros(3) -S(I*w_B)];
G_H = [F_E; zeros(3,1)];
O_H = O_B;
U_H = E_H*(omega^2);  % movement vector in H
acc_H = M_H\(-C_H(vel_H)*vel_H + G_H + O_H*omega + U_H);

input_vector = U_B(2:6);


% generate rotation matrix
function r = rotate(angles)
    r = eul2rotm(angles);
end

% generate generalized transfer matrix
function t = transfer(angles)
    phi = angles(1);
    theta = angles(2);
    t = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
         0 cos(phi) -sin(phi);
         0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
end

% generate generalized matrix J
function calculatedJ = J_theta(angles)
    R = rotate(angles);
    T = transfer(angles);
    calculatedJ = [R zeros(3); zeros(3) T];
end

% skew-symmetric operator
function s = S(k)
    s = [0 -k(3) k(1);  k(3) 0 -k(1);   -k(2) k(1) 0];
end

% coriolis-centripetal matrix
function c = C_B(v)
    V_B = v(1:3);
    w_B = v(4:6);
    c = [zeros(3) -m*S(V_B); 
        zeros(3) -S(I_B*w_B)]; % coriolis-centripetal matrix
end

% gyroscopic propeller matrix
function o = O(v)
    q = v(2);
    p = v(1);
    o = J_TP * [zeros(4); q -q q -q; -p p -p p; zeros(1,4)];
end


