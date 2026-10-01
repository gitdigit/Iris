%% INIT.M – Initialize parameters for HyEQ Drone Simulation (From Workspace version)
clear; clc; close all;

disp('=== Initializing Drone Simulation Environment ===');

%% === STEP 1: Generate trajectory if missing ===
if ~isfile('Trajectory_of_Drone.mat')
    disp('Trajectory_of_Drone.mat not found → Generating new circular trajectory...');
    generate_trajectory_local('helix');
else
    disp('Trajectory_of_Drone.mat found → Loading existing trajectory...');
end

%% === STEP 2: Load trajectory ===
load('Trajectory_of_Drone.mat', 'traj');

% Ensure time-synchronized format for Simulink From Workspace blocks
traj.p_ref   = [traj.t, traj.p_ref];     % [time, x, y, z]
traj.v_ref   = [traj.t, traj.v_ref];     % [time, vx, vy, vz]
traj.yaw_ref = [traj.t, traj.yaw_ref];   % [time, yaw]

%% === STEP 3: Load parameters ===
P = quad_params_local();


%% === STEP 4: Precompute LQR gains (for Controller_LQR block) ===
A_pos = [zeros(3), eye(3);
         zeros(3), zeros(3)];
B_pos = [zeros(3,3);
         eye(3)/P.m];
Q_pos = diag([100 100 100 10 10 10]);
R_pos = diag([1 1 1]);
P.K_pos = lqr(A_pos, B_pos, Q_pos, R_pos);

A_att = [zeros(3), eye(3);
         zeros(3,6)];
B_att = [zeros(3,3);
         inv(P.I)];
Q_att = diag([1000 1000 1000 10 10 10]);
R_att = diag([1 1 1]);
P.K_att = lqr(A_att, B_att, Q_att, R_att);

%% === STEP 5: Initial condition ===
x0 = zeros(14,1);
x0(1:3) = traj.p_ref(1,2:4)';   % start position (x,y,z)
x0(9)   = traj.yaw_ref(1,2);    % start yaw (psi)
% x0(13) et x0(14) sont à 0 par défaut (Energie initiale consommée = 0)

%% === STEP 6: Simulation parameters ===
Tf = traj.t(end);   % total time (s)
sim_time = [0 Tf];
T = 100;
J = 100;
rule = 1;

%% === STEP 7: Push everything to base workspace ===
assignin('base','P',P);
assignin('base','x0',x0);
assignin('base','traj',traj);
assignin('base','sim_time',sim_time);
assignin('base','T',T);
assignin('base','J',J);
assignin('base','rule',rule);

disp('✅ Initialization complete: parameters, trajectory, and LQR gains loaded.');

%% ================================================================
%% === Local function: generate_trajectory_local ==================
function generate_trajectory_local(shape)
if nargin<1, shape = 'circle'; end

Tf = 100; dt = 0.02;
t = (0:dt:Tf)';
w = 2*pi/Tf*2;   % angular speed

switch lower(shape)
    case 'circle'
        R = 5;
        x = R*cos(w*t);
        y = R*sin(w*t);
        z = 2 + 0*t;
    case 'lemniscate'
        a = 5;
        x = a*sin(w*t);
        y = a*sin(w*t).*cos(w*t);
        z = 2 + 0.5*sin(0.5*w*t);
    case 'helix'
        R = 4;
        x = R*cos(w*t);
        y = R*sin(w*t);
        z = linspace(0.5, 5.5, numel(t))';
    case 'flower'
    R = 5; n = 5; z0 = 2;
    r = R .* (1 + 0.3*sin(n*w*t));
    x = r .* cos(w*t);
    y = r .* sin(w*t);
    z = z0 + 0.5*sin(0.5*w*t);
    case 'sinewave'
    A = 5; L = 10; z0 = 2;
    x = linspace(0, L, numel(t))';
    y = A * sin(0.5*w*t);
    z = z0 + 0*t;
    otherwise
        error('Unknown trajectory shape "%s".', shape);
end

% Compute yaw and reference velocities
yaw = atan2(diff([y(1); y]), diff([x(1); x]));
xd = [0; diff(x)/dt];
yd = [0; diff(y)/dt];
zd = [0; diff(z)/dt];

traj.t = t;
traj.p_ref = [x y z];
traj.v_ref = [xd yd zd];
traj.yaw_ref = yaw;

save('Trajectory_of_Drone.mat','traj');
fprintf('✅ Generated and saved Trajectory_of_Drone.mat with %d samples.\n', numel(t));
end

%% === Local function: quad_params_local ===========================
function P = quad_params_local()

%% === Masse & Gravité ===
P.m = 1.0;
% AJOUT D'UNE CHARGE UTILE ET CHANGEMENT DE CG
m_charge = 3; % Charge de 0.2 kg
P.m = P.m + m_charge; % Nouvelle masse = 1.2 kg
P.g = 9.81;

%% === Paramètres aérodynamiques ===
P.rho = 1.225;   % densité kg/m³
P.Cd  = 1.0;     % coefficient de traînée

% Surfaces exposées (m²)
P.Sx_front  = 0.020;   % face avant (X+)
P.Sx_back   = 0.020;   % face arrière (X-)

P.Sy_right  = 0.015;   % face droite (Y+)
P.Sy_left   = 0.015;   % face gauche (Y-)

P.Sz_top    = 0.010;   % face dessus (Z+)
P.Sz_bottom = 0.010;   % face dessous (Z-)

% Centres de pression (m) — OFFSET par rapport au centre d'inertie
P.r_cp_x  = [ 0.15;  0;    0];   % X+
P.r_cp_mx = [-0.15;  0;    0];   % X-

P.r_cp_y  = [ 0;   0.15;   0];   % Y+
P.r_cp_my = [ 0;  -0.15;   0];   % Y-

P.r_cp_z  = [ 0;    0;   0.10];  % Z+
P.r_cp_mz = [ 0;    0;  -0.10];  % Z-

%% === Inerties ===
Ix = 0.02; Iy = 0.02; Iz = 0.04;
P.I = diag([Ix Iy Iz]);

%% === Rotor / Bras ===
P.L  = 0.25;
P.kf = 9.32e-5;
P.km = 9.37e-6;

%% === Énergie ===
P.E_sub = 11.0;     % W
P.eta   = 0.80;     % efficacité

%% === Limites ===
P.tau_max = 1.5;

%% === Gains Position ===
P.Kp_pos = diag([1.6 1.6 2.8]);
P.Kd_pos = diag([1.2 1.2 1.6]);
P.Ki_pos = diag([0 0 0.1]);

P.int_pos_max = [0.6; 0.6; 0.6];

%% === Gains Attitude ===
P.Kp_att = diag([6.0 6.0 3.5]);
P.Kd_att = diag([1.5 1.5 0.8]);
P.Ki_att = diag([0.05 0.05 0.01]);

P.int_att_max = [0.3; 0.3; 0.3];

%% === Contrôleur ===
P.dt_ctrl = 0.01;

end
