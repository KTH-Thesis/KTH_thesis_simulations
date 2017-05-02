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

  % Agent 3
  global t_3;
  global x_3; % THIS IS THE ERROR
  global u_3;
  global des_3;
  global u_open_loop_3;
  global x_open_loop_3;

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


  %% NMPC Parameters

  total_iterations = 100;
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
  x_init_1       = [-6, 3.5, 0];   % x_0
  des_1          = [6, 3.5, 0];   % x_des
  xmeasure_1     = x_init_1 - des_1;
  u0_1           = 10*ones(num_inputs, N); % initial guess

  tT_1           = [];
  xX_1           = [];
  uU_1           = [];


  % init Agent 2
  tmeasure_2     = 0.0;         % t_0
  x_init_2       = [-6, 2.3, 0];   % x_0
  des_2          = [6, 2.3, 0];   % x_des
  xmeasure_2     = x_init_2 - des_2;
  u0_2           = 10*ones(num_inputs, N); % initial guess

  tT_2           = [];
  xX_2           = [];
  uU_2           = [];

  % init Agent 3
  tmeasure_3     = 0.0;         % t_0
  x_init_3       = [-6, 4.7, 0];   % x_0
  des_3          = [6, 4.7, 0];   % x_des
  xmeasure_3     = x_init_3 - des_3;
  u0_3           = 10*ones(num_inputs, N); % initial guess

  tT_3           = [];
  xX_3           = [];
  uU_3           = [];

  % Penalty matrices
  Q              = 0.5 * (eye(3) + rand(3));
  R              = 0.005 * eye(2);
  P              = 0.5 * (eye(3) + rand(3));

  % Input bounds
  u_abs = 10;
  u_max = u_abs;
  u_min = -u_abs;


  % obstacles: x_c, y_c, r
  obs            = [0, 3.5, 1];

  % radii of agents
  r              = [0.5; 0.5; 0.5];

  % Proximity tolerance between agents
  ptol           = 0.01;

  % Proximity tolerance between an agent and obstacles
  otol           = 0.01;

  % Distance bounds
  d_min          = r(1) + r(2) + ptol;
  d_max          = 2 * (r(1) + r(2)) + ptol;

  % The amplitude of the disturbance
  disturbance    = 0.10

  % Terminal cost tolerance
  omega_v        = 0.05

  % Lipschitz constants
  L_g            = u_max * sqrt(sum(sum(Q(1:2,1:2))))
  L_v            = 2 * svds(P,1) * omega_v

  % Terminal set bounds
  epsilon_omega  = omega_v^2 * 3 * svds(P,1)
  epsilon_psi    = (L_v / L_g * exp(L_g * (N * T - T)) * (exp(L_g * T) - 1)) * disturbance + epsilon_omega
  omega_psi      = sqrt(epsilon_psi / (3 * svds(P,1)))

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

   % Solve for agent 3 --------------------------------------------------------
    tT_3 = [tT_3; tmeasure_3];
    xX_3 = [xX_3; xmeasure_3];

    nmpc_3(@runningcosts_3, @terminalcosts_3, @constraints_3, ...
      @terminalconstraints_3, @linearconstraints_3, @system_ct_3, ...
      @system_ct_3_real, mpciterations, N, T, tmeasure_3, xmeasure_3, u0_3, ...
      tol_opt, opt_option, ...
      type, atol_ode_real, rtol_ode_real, atol_ode_sim, rtol_ode_sim, ...
      iprint, @printHeader_3, @printClosedloopData_3);

    tmeasure_3      = t_3(end);     % new t_0
    x_init_3        = x_3(end,:);   % new x_0
    xmeasure_3      = x_init_3;
    u0_3            = [u_open_loop_3(:,2:size(u_open_loop_3,2)) u_open_loop_3(:,size(u_open_loop_3,2))];

    % Store the applied input
    uU_3 = [uU_3, u_3];

    save('xX_3.mat');
    save('uU_3.mat');
    save('tT_3.mat');

  end

  toc;
  save('variables.mat')
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

function cost_3 = runningcosts_3(t_3, e_3, u_3)
  global Q;
  global R;

  e_3=e_3';

  Q_3 = Q;
  R_3 = R;

  cost_3 = e_3'*Q_3*e_3 + u_3'*R_3*u_3;
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

function cost_3 = terminalcosts_3(t_3, e_3)
  global P;

  e_3 = e_3';

  P_3 = P;

  cost_3 = e_3'*P_3*e_3;
end

%% Constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [c,ceq] = constraints_1(t_1, e_1, u_1)

  global des_1;
  global des_2;
  global des_3;
  global obs;
  global r;
  global otol;
  global d_min;
  global d_max;
  global x_open_loop_2;
  global x_open_loop_3;
  global global_clock;
  global T;
  global point_in_horizon_ceiling;
  global disturbance;
  global L_g;
  global Q;
  global P;
  global epsilon_omega;

  %disp('in constraints_1')
  n_2 = size(x_open_loop_2, 1);
  n_3 = size(x_open_loop_3, 1);

  c = [];
  ceq = [];

  arrived_at_terminal_region = false;
  if t_1 == global_clock
    if e_1 * P * e_1' <= epsilon_omega
      arrived_at_terminal_region = true;
    end
  end

  if arrived_at_terminal_region == false

    ball_t_1 = disturbance / L_g * (exp(L_g * (t_1 - global_clock)) - 1);

    th = ball_t_1 / sqrt(3 * svds(Q,1));

    x = e_1(1) + th;
    y = e_1(2) + th;
    z = e_1(3) + th;

    % Avoid collision with obstacle
    c(1) = (obs(1,3) + r(1) + otol)^2 - ((x+des_1(1) - obs(1,1))^2 + (y+des_1(2) - obs(1,2))^2);

    c(2) = z + des_1(3) - pi;
    c(3) = -z - des_1(3) - pi;

    x = e_1(1) - th;
    y = e_1(2) - th;
    z = e_1(3) - th;

    % Avoid collision with obstacle
    c(4) = (obs(1,3) + r(1) + otol)^2 - ((x+des_1(1) - obs(1,1))^2 + (y+des_1(2) - obs(1,2))^2);

    c(5) = z + des_1(3) - pi;
    c(6) = -z - des_1(3) - pi;

    % The index pointing to the current configuration of agent 2 from the
    % perspective of agent 1.
    % index 2 is the current configuration of agents 2,3
    point_in_horizon = 1 + int8( (t_1 - global_clock) / T );

    if n_2 > 0

      x_open_loop_2_cut = x_open_loop_2(1:end,:);

      if point_in_horizon <= point_in_horizon_ceiling;

        % The distance bewteen the two agents -- max
        dist_x = e_1(1) + des_1(1) - x_open_loop_2_cut(point_in_horizon,1) - des_2(1) + 2*th;
        dist_y = e_1(2) + des_1(2) - x_open_loop_2_cut(point_in_horizon,2) - des_2(2) + 2*th;
        dist = sqrt(dist_x^2 + dist_y^2);

        % Avoid collision with agent 2 along the entire horizon
        c(7) = d_min - dist;

        % Maintain connectivity with agent 2 along the entire horizon
        c(8) = dist - d_max;

        % The distance bewteen the two agents -- min
        dist_x = e_1(1) + des_1(1) - x_open_loop_2_cut(point_in_horizon,1) - des_2(1) - 2*th;
        dist_y = e_1(2) + des_1(2) - x_open_loop_2_cut(point_in_horizon,2) - des_2(2) - 2*th;
        dist = sqrt(dist_x^2 + dist_y^2);

        % Avoid collision with agent 2 along the entire horizon
        c(9) = d_min - dist;

        % Maintain connectivity with agent 2 along the entire horizon
        c(10) = dist - d_max;

      end
    end


    if n_3 > 0

      x_open_loop_3_cut = x_open_loop_3(1:end,:);

      if point_in_horizon <= point_in_horizon_ceiling;

        % The distance bewteen the two agents -- max
        dist_x = e_1(1) + des_1(1) - x_open_loop_3_cut(point_in_horizon,1) - des_3(1) + 2*th;
        dist_y = e_1(2) + des_1(2) - x_open_loop_3_cut(point_in_horizon,2) - des_3(2) + 2*th;
        dist = sqrt(dist_x^2 + dist_y^2);

        % Avoid collision with agent 3 along the entire horizon
        c(11) = d_min - dist;

        % Maintain connectivity with agent 3 along the entire horizon
        c(12) = dist - d_max;

        % The distance bewteen the two agents -- min
        dist_x = e_1(1) + des_1(1) - x_open_loop_3_cut(point_in_horizon,1) - des_3(1) - 2*th;
        dist_y = e_1(2) + des_1(2) - x_open_loop_3_cut(point_in_horizon,2) - des_3(2) - 2*th;
        dist = sqrt(dist_x^2 + dist_y^2);

        % Avoid collision with agent 3 along the entire horizon
        c(13) = d_min - dist;

        % Maintain connectivity with agent 3 along the entire horizon
        c(14) = dist - d_max;

      end
    end
  end
end




function [c,ceq] = constraints_2(t_2, e_2, u_2)

  global des_1;
  global des_2;
  global des_3;
  global obs;
  global r;
  global otol;
  global d_min;
  global d_max;
  global x_open_loop_1;
  global x_open_loop_3;
  global global_clock;
  global T;
  global point_in_horizon_ceiling;
  global disturbance;
  global L_g;
  global Q;
  global P;
  global epsilon_omega;

  %disp('in constraints_2')
  n_1 = size(x_open_loop_1, 1);
  n_3 = size(x_open_loop_3, 1);

  c = [];
  ceq = [];

  arrived_at_terminal_region = false;
  if t_2 == global_clock
    if e_2 * P * e_2' <= epsilon_omega
      arrived_at_terminal_region = true;
    end
  end

  if arrived_at_terminal_region == false

    ball_t_2 = disturbance / L_g * (exp(L_g * (t_2 - global_clock)) - 1);

    th = ball_t_2 / sqrt(3 * svds(Q,1));

    x = e_2(1) + th;
    y = e_2(2) + th;
    z = e_2(3) + th;

    c(1) = (obs(1,3) + r(2) + otol) - sqrt((x+des_2(1) - obs(1,1))^2 + (y+des_2(2) - obs(1,2))^2);

    c(2) = z + des_2(3) - pi;
    c(3) = -z - des_2(3) - pi;

    x = e_2(1) - th;
    y = e_2(2) - th;
    z = e_2(3) - th;

    c(4) = (obs(1,3) + r(2) + otol) - sqrt((x+des_2(1) - obs(1,1))^2 + (y+des_2(2) - obs(1,2))^2);

    c(5) = z + des_2(3) - pi;
    c(6) = -z - des_2(3) - pi;

    % The index pointing to the current configuration of agent 1 from the
    % perspective of agent 2.
    % index 2 is the current configuration of agents 1,3
    point_in_horizon = 1 + int8( (t_2 - global_clock) / T );


    if n_1 > 0

      x_open_loop_1_cut = x_open_loop_1(1:end,:);

      if point_in_horizon <= point_in_horizon_ceiling;

        % The distance bewteen the two agents -- max
        dist_x = e_2(1) + des_2(1) - x_open_loop_1_cut(point_in_horizon,1) - des_1(1) + 2*th;
        dist_y = e_2(2) + des_2(2) - x_open_loop_1_cut(point_in_horizon,2) - des_1(2) + 2*th;
        dist = sqrt(dist_x^2 + dist_y^2);

        % Avoid collision with agent 1 along the entire horizon
        c(7) = d_min - dist;

        % Maintain connectivity with agent 1 along the entire horizon
        c(8) = dist - d_max;

        % The distance bewteen the two agents -- min
        dist_x = e_2(1) + des_2(1) - x_open_loop_1_cut(point_in_horizon,1) - des_1(1) - 2*th;
        dist_y = e_2(2) + des_2(2) - x_open_loop_1_cut(point_in_horizon,2) - des_1(2) - 2*th;
        dist = sqrt(dist_x^2 + dist_y^2);

        % Avoid collision with agent 1 along the entire horizon
        c(9) = d_min - dist;

        % Maintain connectivity with agent 1 along the entire horizon
        c(10) = dist - d_max;

      end
    end

    if n_3 > 0

      x_open_loop_3_cut = x_open_loop_3(1:end,:);

      if point_in_horizon <= point_in_horizon_ceiling;

        % The distance bewteen the two agents -- max
        dist_x = e_2(1) + des_2(1) - x_open_loop_3_cut(point_in_horizon,1) - des_3(1) + 2*th;
        dist_y = e_2(2) + des_2(2) - x_open_loop_3_cut(point_in_horizon,2) - des_3(2) + 2*th;
        dist = sqrt(dist_x^2 + dist_y^2);

        % Avoid collision with agent 3 along the entire horizon
        c(11) = d_min - dist;

        % The distance bewteen the two agents -- min
        dist_x = e_2(1) + des_2(1) - x_open_loop_3_cut(point_in_horizon,1) - des_3(1) - 2*th;
        dist_y = e_2(2) + des_2(2) - x_open_loop_3_cut(point_in_horizon,2) - des_3(2) - 2*th;
        dist = sqrt(dist_x^2 + dist_y^2);

        % Avoid collision with agent 3 along the entire horizon
        c(12) = d_min - dist;

      end
    end
  end
end



function [c,ceq] = constraints_3(t_3, e_3, u_3)

  global des_1;
  global des_2;
  global des_3;
  global obs;
  global r;
  global otol;
  global d_min;
  global d_max;
  global x_open_loop_1;
  global x_open_loop_2;
  global global_clock;
  global T;
  global point_in_horizon_ceiling;
  global disturbance;
  global L_g;
  global Q;
  global P;
  global epsilon_omega;

  %disp('in constraints_3')
  n_1 = size(x_open_loop_1, 1);
  n_2 = size(x_open_loop_2, 1);

  c = [];
  ceq = [];

  arrived_at_terminal_region = false;
  if t_3 == global_clock
    if e_3 * P * e_3' <= epsilon_omega
      arrived_at_terminal_region = true;
    end
  end

  if arrived_at_terminal_region == false

    ball_t_3 = disturbance / L_g * (exp(L_g * (t_3 - global_clock)) - 1);

    th = ball_t_3 / sqrt(3 * svds(Q,1));

    x = e_3(1) + th;
    y = e_3(2) + th;
    z = e_3(3) + th;

    c(1) = (obs(1,3) + r(2) + otol) - sqrt((x+des_3(1) - obs(1,1))^2 + (y+des_3(2) - obs(1,2))^2);

    c(2) = z + des_3(3) - pi;
    c(3) = -z - des_3(3) - pi;


    x = e_3(1) - th;
    y = e_3(2) - th;
    z = e_3(3) - th;

    c(4) = (obs(1,3) + r(2) + otol) - sqrt((x+des_3(1) - obs(1,1))^2 + (y+des_3(2) - obs(1,2))^2);

    c(5) = z + des_3(3) - pi;
    c(6) = -z - des_3(3) - pi;


    % The index pointing to the current configuration of agent 3 from the
    % perspective of agents 2,3.
    % index 2 is the current configuration of agent 1
    point_in_horizon = 1 + int8( (t_3 - global_clock) / T );

    if n_1 > 0

      x_open_loop_1_cut = x_open_loop_1(1:end,:);

      if point_in_horizon <= point_in_horizon_ceiling;

        % The distance bewteen the two agents -- max
        dist_x = e_3(1) + des_3(1) - x_open_loop_1_cut(point_in_horizon,1) - des_1(1) + 2*th;
        dist_y = e_3(2) + des_3(2) - x_open_loop_1_cut(point_in_horizon,2) - des_1(2) + 2*th;
        dist = sqrt(dist_x^2 + dist_y^2);

        % Avoid collision with agent 1 along the entire horizon
        c(7) = d_min - dist;
  %
        % Maintain connectivity with agent 1 along the entire horizon
        c(8) = dist - d_max;

        % The distance bewteen the two agents -- min
        dist_x = e_3(1) + des_3(1) - x_open_loop_1_cut(point_in_horizon,1) - des_1(1) - 2*th;
        dist_y = e_3(2) + des_3(2) - x_open_loop_1_cut(point_in_horizon,2) - des_1(2) - 2*th;
        dist = sqrt(dist_x^2 + dist_y^2);

        % Avoid collision with agent 1 along the entire horizon
        c(9) = d_min - dist;
  %
        % Maintain connectivity with agent 1 along the entire horizon
        c(10) = dist - d_max;

      end
    end

    if n_2 > 0

      x_open_loop_2_cut = x_open_loop_2(1:end,:);

      if point_in_horizon <= point_in_horizon_ceiling;

        % The distance bewteen the two agents -- max
        dist_x = e_3(1) + des_3(1) - x_open_loop_2_cut(point_in_horizon,1) - des_2(1) + 2*th;
        dist_y = e_3(2) + des_3(2) - x_open_loop_2_cut(point_in_horizon,2) - des_2(2) + 2*th;
        dist = sqrt(dist_x^2 + dist_y^2);

        % Avoid collision with agent 2 along the entire horizon
        c(11) = d_min - dist;


        % The distance bewteen the two agents -- min
        dist_x = e_3(1) + des_3(1) - x_open_loop_2_cut(point_in_horizon,1) - des_2(1) - 2*th;
        dist_y = e_3(2) + des_3(2) - x_open_loop_2_cut(point_in_horizon,2) - des_2(2) - 2*th;
        dist = sqrt(dist_x^2 + dist_y^2);

        % Avoid collision with agent 2 along the entire horizon
        c(12) = d_min - dist;

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

function [c,ceq] = terminalconstraints_3(t_3, e_3)

  global omega_v;

  c = [];
  ceq = [];


  c(1) = e_3(1) - omega_v;
  c(2) = -e_3(1) - omega_v;
  c(3) = e_3(2) - omega_v;
  c(4) = -e_3(2) - omega_v;
  c(5) = e_3(3) - omega_v;
  c(6) = -e_3(3) - omega_v;

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

function [A, b, Aeq, beq, lb, ub] = linearconstraints_3(t_3, x_3, u_3)
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

function printHeader_3()

  fprintf('------|---------------------------------------------------------------------\n');
  fprintf('   k  |    u1_3(t)      u2_3(t)      e3_1(t)     e3_2(t)     e3_3(t)    Time\n');
  fprintf('------|---------------------------------------------------------------------\n');

end

function printClosedloopData_1(mpciter, u_1, e_1, t_Elapsed_1)

  fprintf(' %3d  | %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f  %+11.6f', ...
    mpciter, u_1(1), u_1(2), e_1(1), e_1(2), e_1(2), t_Elapsed_1);
end

function printClosedloopData_2(mpciter, u_2, e_2, t_Elapsed_2)

  fprintf(' %3d  | %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f', ...
    mpciter, u_2(1), u_2(2), e_2(1), e_2(2), e_2(3), t_Elapsed_2);
end

function printClosedloopData_3(mpciter, u_3, e_3, t_Elapsed_3)

  fprintf(' %3d  | %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f', ...
    mpciter, u_3(1), u_3(2), e_3(1), e_3(2), e_3(3), t_Elapsed_3);
end
