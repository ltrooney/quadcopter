% parameters
m = .5;
g = 9.81;
theta = 0;
psi = 0;
phi = 0;
b = 1;
T = b * (o1^2 + o2^2 + o3^2 + o4^2); % thrust constant
G = [0; 0; -m * g]; % gravity in inertial frame


R = rotationMatrix(theta, psi, phi);
T_angle = transferMatrix(theta, phi);

% linear velocities
lin_velocities_body = [0; 0; 0];
lin_velocities_inertial = R * lin_velocities_body;

% angular velocities
ang_velocities_body = [0; 0; 0];
ang_velocities_inertial = T_angle * ang_velocities_body;

% forces
thrust_body = [0; 0; T];
thrust_inertial = R * thrust_body;

ang_accel_inertial = (G + thrust_inertial) / m;


u1 = T;

% ZYX rotation matrix
function R = rotationMatrix(theta, psi, phi)
    Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    Ry = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
    Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];

    R = Rz*Ry*Rx;
end

% transform angular velocities from inertial frame to body frame
function T_angle = transferMatrix(theta, phi)
    T_angle = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
         0 cos(phi) -sin(phi);
         0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
end