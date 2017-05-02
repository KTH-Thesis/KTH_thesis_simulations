function main

  %% Initialization
  clc;
  clear all;
  close all;
  tic;

  %% Global parameters

  num_states = 3;
  num_inputs = 2;

  % Agent 1
  global t_1;
  global x_1; % THIS IS THE ERROR
  global u_1;
  global des_1;
  global u_open_loop_1;
  global x_open_loop_1;

  % Agent 2
  global t_2;
  global x_2; % THIS IS THE ERROR
  global u_2;
  global des_2;
  global u_open_loop_2;
  global x_open_loop_2;

  % Penalty matrices
  global Q;
  global R;
  global P;

  % Input bounds
  global u_max;
  global u_min;

  % Obstacles
  global obs;
  global r;

  % Proximities
  global d_min;
  global d_max;
  global ptol;
  global otol;
  global omega_v;
  global epsilon_omega;

  % Global clock
  global global_clock;

  % The sampling time
  global T;

  % The upper threshold within the time-horizon for which the open loop
  % solution of the counterpart agent is taken into consideration
  global point_in_horizon_ceiling;

  % Lipschitz constants
  global L_g;
  global L_v;

  % The magnitude of the disturbance
  global disturbance;
  
  % Flags for agents arriving inside their terminal regions
  global agent_1_arrived_at_terminal_region;
  global agent_2_arrived_at_terminal_region;

  %% NMPC Parameters

  total_iterations = 200;
  mpciterations  = 1;
  N              = 5;       % length of Horizon
  T              = 0.1;     % sampling time
  tol_opt        = 1e-8;
  opt_option     = 0;
  iprint         = 5;
  type           = 'differential equation';
  atol_ode_real  = 1e-12;
  rtol_ode_real  = 1e-12;
  atol_ode_sim   = 1e-4;
  rtol_ode_sim   = 1e-4;


  % init Agent 1
  tmeasure_1     = 0.0;         % t_0
  x_init_1       = [-6, 4.25, 0];   % x_0
  des_1          = [6, 4.25, 0];   % x_des
  xmeasure_1     = x_init_1 - des_1;
  u0_1           = 10*ones(num_inputs, N); % initial guess

  tT_1           = [];
  xX_1           = [];
  uU_1           = [];


  % init Agent 2
  tmeasure_2     = 0.0;         % t_0
  x_init_2       = [-6, 2.75, 0];   % x_0
  des_2          = [6, 2.75, 0];   % x_des
  xmeasure_2     = x_init_2 - des_2;
  u0_2           = 10*ones(num_inputs, N); % initial guess

  tT_2           = [];
  xX_2           = [];
  uU_2           = [];

  % Penalty matrices
%   rn             = 0.5 * rand(3);
%   Q              = 0.5 * (eye(3) + rn);
%   R              = 0.005 * eye(2);
%   P              = 0.5 * (eye(3) + rn);

  Q              = [0.7349    0.0506    0.0034;
                    0.1260    0.6840    0.0710;
                    0.2113    0.1336    0.5599];
                  
  R              = 0.001 * eye(2);
  
  P              = [0.7349    0.0506    0.0034;
                    0.1260    0.6840    0.0710;
                    0.2113    0.1336    0.5599];

  % Input bounds
  u_abs = 10;
  u_max = u_abs;
  u_min = -u_abs;

  % obstacles: x_c, y_c, r
  obs            = [0, 1.85, 1;
                    0, 5.15, 1];

  % radii of agents
  r              = [0.5; 0.5];

  % Proximity tolerance
  ptol           = 0.01;
  otol           = 0.01;

  % Distance bounds
  d_min          = r(1) + r(2) + ptol;
  d_max          = 2 * (r(1) + r(2)) + ptol;
  
%   % The amplitude of the disturbance
%   disturbance    = 0.10
% 
%   % Terminal cost tolerance
%   omega_v        = 0.05
% 
%   % Lipschitz constants
%   L_g            = u_max * sqrt(sum(sum(Q(1:2,1:2))))
%   L_v            = 2 * svds(P,1) * omega_v
% 
%   % Terminal set bounds
%   epsilon_omega  = omega_v^2 * 3 * svds(P,1)
%   epsilon_psi    = (L_v / L_g * exp(L_g * (N * T - T)) * (exp(L_g * T) - 1)) * disturbance + epsilon_omega
%   omega_psi      = sqrt(epsilon_psi / (3 * svds(P,1)))

  % The amplitude of the disturbance
  disturbance    = 0.10

  % Terminal tolerance
  omega_v        = 0.05
  epsilon_omega  = 3 * svds(P,1) * omega_v^2


  % Lipschitz constant L_g
  L_g            = u_max * sqrt(sum(sum(Q(1:2,1:2))))
  
  D              = 4 * epsilon_omega / (3 * svds(P,1)) + (2 * disturbance * exp(L_g * (N * T - T)) * (exp(L_g * T) - 1) / 3 / L_g)^2;  
  omega_psi      = disturbance / (3 * L_g) * exp(L_g * (N * T - T)) * (exp(L_g * T) - 1) + sqrt(D)/2
  
  L_v            = 2 * svds(P,1) * omega_psi

  % Psi set bounds
  epsilon_psi    = 3 * svds(P,1) * omega_psi^2 

  % Flags for agents arriving inside their terminal regions
  agent_1_arrived_at_terminal_region = false;
  agent_2_arrived_at_terminal_region = false;

  % Initialize global clock
  global_clock = 0.0;

  point_in_horizon_ceiling = 2;


  for k = 1:total_iterations

    fprintf('iteration %d\n', k);

    % Increment clock
    global_clock = global_clock + T;


    % Solve for agent 1 --------------------------------------------------------
    tT_1 = [tT_1; tmeasure_1];
    xX_1 = [xX_1; xmeasure_1];

    nmpc_1(@runningcosts_1, @terminalcosts_1, @constraints_1, ...
      @terminalconstraints_1, @linearconstraints_1, @system_ct_1, ...
      @system_ct_1_real, mpciterations, N, T, tmeasure_1, xmeasure_1, u0_1, ...
      tol_opt, opt_option, ...
      type, atol_ode_real, rtol_ode_real, atol_ode_sim, rtol_ode_sim, ...
      iprint, @printHeader_1, @printClosedloopData_1);

    tmeasure_1      = t_1(end);     % new t_0
    x_init_1        = x_1(end,:);   % new x_0
    xmeasure_1      = x_init_1;
    u0_1            = [u_open_loop_1(:,2:size(u_open_loop_1,2)) u_open_loop_1(:,size(u_open_loop_1,2))];

    % Store the applied input
    uU_1 = [uU_1, u_1];

    save('xX_1.mat');
    save('uU_1.mat');
    save('tT_1.mat');
    
    % Indicate whether agent 1 has arrived within its terminal region Omega
    if x_init_1 * P * x_init_1' <= epsilon_psi
      agent_1_arrived_at_terminal_region = true;
    end


    % Solve for agent 2 --------------------------------------------------------
    tT_2 = [tT_2; tmeasure_2];
    xX_2 = [xX_2; xmeasure_2];

    nmpc_2(@runningcosts_2, @terminalcosts_2, @constraints_2, ...
      @terminalconstraints_2, @linearconstraints_2, @system_ct_2, ...
      @system_ct_2_real, mpciterations, N, T, tmeasure_2, xmeasure_2, u0_2, ...
      tol_opt, opt_option, ...
      type, atol_ode_real, rtol_ode_real, atol_ode_sim, rtol_ode_sim, ...
      iprint, @printHeader_2, @printClosedloopData_2);

    tmeasure_2      = t_2(end);     % new t_0
    x_init_2        = x_2(end,:);   % new x_0
    xmeasure_2      = x_init_2;
    u0_2            = [u_open_loop_2(:,2:size(u_open_loop_2,2)) u_open_loop_2(:,size(u_open_loop_2,2))];

    % Store the applied input
    uU_2 = [uU_2, u_2];

    save('xX_2.mat');
    save('uU_2.mat');
    save('tT_2.mat');
    
    % Indicate whether agent 2 has arrived within its terminal region Omega
    if x_init_2 * P * x_init_2' <= epsilon_psi
      agent_2_arrived_at_terminal_region = true;
    end

  end

  toc;
  save('variables.mat');
end



%% Running Cost %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function cost_1 = runningcosts_1(t_1, e_1, u_1)

  global Q;
  global R;

  e_1=e_1';

  Q_1 = Q;
  R_1 = R;

  cost_1 = e_1'*Q_1*e_1 + u_1'*R_1*u_1;
end

function cost_2 = runningcosts_2(t_2, e_2, u_2)

  global Q;
  global R;

  e_2=e_2';

  Q_2 = Q;
  R_2 = R;

  cost_2 = e_2'*Q_2*e_2 + u_2'*R_2*u_2;
end


%% Terminal Cost %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function cost_1 = terminalcosts_1(t_1, e_1)

  global P;

   e_1 = e_1';

   P_1 = P;

   cost_1 = e_1'*P_1*e_1;
end

function cost_2 = terminalcosts_2(t_2, e_2)

  global P;

   e_2 = e_2';

   P_2 = P;

   cost_2 = e_2'*P_2*e_2;
end

%% Constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [c,ceq] = constraints_1(t_1, e_1, u_1)

  global des_1;
  global des_2;
  global obs;
  global r;
  global otol;
  global d_min;
  global d_max;
  global x_open_loop_2;
  global global_clock;
  global T;
  global point_in_horizon_ceiling;
  global disturbance;
  global L_g;
  global Q;
  global agent_1_arrived_at_terminal_region;

  n_2 = size(x_open_loop_2, 1);

  c = [];
  ceq = [];

  
  if agent_1_arrived_at_terminal_region == false
    ball_t_1 = disturbance / L_g * (exp(L_g * (t_1 - global_clock)) - 1);

    th = ball_t_1 / sqrt(3 * svds(Q,1));

    x = e_1(1) + th;
    y = e_1(2) + th;
    z = e_1(3) + th;


    % Avoid collision with obstacle
    for i = 1:2
      c(i) = (obs(i,3) + r(1) + otol) - sqrt( (x+des_1(1) - obs(i,1))^2 + (y+des_1(2) - obs(i,2))^2);
    end

    c(3) = z + des_1(3) - pi;
    c(4) = -z - des_1(3) - pi;


    x = e_1(1) - th;
    y = e_1(2) - th;
    z = e_1(3) - th;


    % Avoid collision with obstacle
    for i = 1:2
      c(4+i) = (obs(i,3) + r(1) + otol) - sqrt( (x+des_1(1) - obs(i,1))^2 + (y+des_1(2) - obs(i,2))^2);
    end

    c(7) = z + des_1(3) - pi;
    c(8) = -z - des_1(3) - pi;

    if n_2 > 0

      x_open_loop_2_cut = x_open_loop_2(1:end,:);

      point_in_horizon = 1 + int8( (t_1 - global_clock) / T );

      if point_in_horizon <= point_in_horizon_ceiling;

        % The distance bewteen the two agents -- max
        dist_x = e_1(1) + des_1(1) - x_open_loop_2_cut(point_in_horizon,1) - des_2(1) + 2*th;
        dist_y = e_1(2) + des_1(2) - x_open_loop_2_cut(point_in_horizon,2) - des_2(2) + 2*th;
        dist = sqrt(dist_x^2 + dist_y^2);

        % Avoid collision with agent 2
        c(9) = d_min - dist;

        % Maintain connectivity with agent 2
        c(10) = dist - d_max;


        % The distance bewteen the two agents -- min
        dist_x = e_1(1) + des_1(1) - x_open_loop_2_cut(point_in_horizon,1) - des_2(1) - 2*th;
        dist_y = e_1(2) + des_1(2) - x_open_loop_2_cut(point_in_horizon,2) - des_2(2) - 2*th;
        dist = sqrt(dist_x^2 + dist_y^2);

        % Avoid collision with agent 2
        c(11) = d_min - dist;

        % Maintain connectivity with agent 2
        c(12) = dist - d_max;

      end
    end
  end
end




function [c,ceq] = constraints_2(t_2, e_2, u_2)

  global des_1;
  global des_2;
  global obs;
  global r;
  global otol;
  global d_min;
  global d_max;
  global x_open_loop_1;
  global global_clock;
  global T;
  global point_in_horizon_ceiling;
  global disturbance;
  global L_g;
  global Q;
  global agent_2_arrived_at_terminal_region;


  %disp('in constraints_2')
  n_1 = size(x_open_loop_1, 1);

  c = [];
  ceq = [];


  if agent_2_arrived_at_terminal_region == false
    ball_t_2 = disturbance / L_g * (exp(L_g * (t_2 - global_clock)) - 1);

    th = ball_t_2 / sqrt(3 * svds(Q,1));

    x = e_2(1) + th;
    y = e_2(2) + th;
    z = e_2(3) + th;

    for i = 1:2
      c(i) = (obs(i,3) + r(2) + otol) - sqrt( (x+des_2(1) - obs(i,1))^2 + (y+des_2(2) - obs(i,2))^2);
    end

    c(3) = z + des_2(3) - pi;
    c(4) = -z - des_2(3) - pi;

    x = e_2(1) - th;
    y = e_2(2) - th;
    z = e_2(3) - th;

    for i = 1:2
      c(4+i) = (obs(i,3) + r(2) + otol) - sqrt( (x+des_2(1) - obs(i,1))^2 + (y+des_2(2) - obs(i,2))^2);
    end

    c(7) = z + des_2(3) - pi;
    c(8) = -z - des_2(3) - pi;

    if n_1 > 0

      x_open_loop_1_cut = x_open_loop_1(1:end,:);

      point_in_horizon = 1 + int8( (t_2 - global_clock) / T );

      if point_in_horizon <= point_in_horizon_ceiling

        % The distance between the two agents -- max
        dist_x = e_2(1) + des_2(1) - x_open_loop_1_cut(point_in_horizon,1) - des_1(1) + 2*th;
        dist_y = e_2(2) + des_2(2) - x_open_loop_1_cut(point_in_horizon,2) - des_1(2) + 2*th;
        dist = sqrt(dist_x^2 + dist_y^2);

        % Avoid collision with agent 1
        c(9) = d_min - dist;

        % Maintain connectivity with agent 1
        c(10) = dist - d_max;

        % The distance between the two agents -- min
        dist_x = e_2(1) + des_2(1) - x_open_loop_1_cut(point_in_horizon,1) - des_1(1) - 2*th;
        dist_y = e_2(2) + des_2(2) - x_open_loop_1_cut(point_in_horizon,2) - des_1(2) - 2*th;
        dist = sqrt(dist_x^2 + dist_y^2);

        % Avoid collision with agent 1
        c(11) = d_min - dist;

        % Maintain connectivity with agent 1
        c(12) = dist - d_max;

      end
    end
  end
end





%% Terminal Constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [c,ceq] = terminalconstraints_1(t_1, e_1)

  global omega_v;

  c = [];
  ceq = [];


  c(1) = e_1(1) - omega_v;
  c(2) = -e_1(1) - omega_v;
  c(3) = e_1(2) - omega_v;
  c(4) = -e_1(2) - omega_v;
  c(5) = e_1(3) - omega_v;
  c(6) = -e_1(3) - omega_v;

end

function [c,ceq] = terminalconstraints_2(t_2, e_2)

  global omega_v;

  c = [];
  ceq = [];


  c(1) = e_2(1) - omega_v;
  c(2) = -e_2(1) - omega_v;
  c(3) = e_2(2) - omega_v;
  c(4) = -e_2(2) - omega_v;
  c(5) = e_2(3) - omega_v;
  c(6) = -e_2(3) - omega_v;

end

%% Control Constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [A, b, Aeq, beq, lb, ub] = linearconstraints_1(t_1, x_1, u_1)
  global u_max;
  global u_min;

  A   = [];
  b   = [];
  Aeq = [];
  beq = [];

  % u constraints
  lb  = u_min;
  ub  = u_max;
end

function [A, b, Aeq, beq, lb, ub] = linearconstraints_2(t_2, x_2, u_2)
  global u_max;
  global u_min;

  A   = [];
  b   = [];
  Aeq = [];
  beq = [];

  % u constraints
  lb  = u_min;
  ub  = u_max;
end


%% Output Results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function printHeader_1()

  fprintf('------|---------------------------------------------------------------------\n');
  fprintf('   k  |    u1_1(t)      u2_1(t)      e1_1(t)     e1_2(t)     e1_3(t)    Time\n');
  fprintf('------|---------------------------------------------------------------------\n');

end

function printHeader_2()

  fprintf('------|---------------------------------------------------------------------\n');
  fprintf('   k  |    u1_2(t)      u2_2(t)      e2_1(t)     e2_2(t)     e2_3(t)    Time\n');
  fprintf('------|---------------------------------------------------------------------\n');

end

function printClosedloopData_1(mpciter, u_1, e_1, t_Elapsed_1)

  fprintf(' %3d  | %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f', ...
    mpciter, u_1(1), u_1(2), e_1(1), e_1(2), e_1(3), t_Elapsed_1);
end

function printClosedloopData_2(mpciter, u_2, e_2, t_Elapsed_2)

  fprintf(' %3d  | %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f', ...
    mpciter, u_2(1), u_2(2), e_2(1), e_2(2), e_2(3), t_Elapsed_2);
end
