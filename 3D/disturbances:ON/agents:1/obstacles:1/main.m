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

  % Penalty matrices
  global Q;
  global R;
  global P;

  % Obstacles
  global obs;
  global r;

  % Proximities
  global otol;
  global omega_v;

  % Global clock
  global global_clock;

  % The sampling time
  global T;
  
  % The supremum of the disturbance
  global disturbance_ceiling;


  %% NMPC Parameters

  total_iterations = 40;
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


  % Penalty matrices
  Q              = 0.5 * eye(3);
  R              = 0.05 * eye(2);
  P              = 0.5 * eye(3);


  % obstacles: x_c, y_c, r
  obs            = [0, 3.5, 1];

  % radii of agents
  r              = [0.5];

  % Proximity tolerance between an agent and obstacles
  otol           = 0.1;

  % Terminal cost tolerance
  omega_v        = 0.001;
  
  % The supremum of the disturbance
  disturbance_ceiling = pi * 10^(-5);

  % Initialize global clock
  global_clock = 0.0;


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
    uU_1 = [uU_1; u_1];

    save('xX_1.mat');
    save('uU_1.mat');
    save('tT_1.mat');
    
  end

  toc;
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


%% Terminal Cost %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function cost_1 = terminalcosts_1(t_1, e_1)
  global P;

  e_1 = e_1';

  P_1 = P;

  cost_1 = e_1'*P_1*e_1;
end


%% Constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [c,ceq] = constraints_1(t_1, e_1, u_1)

  global des_1;
  global obs;
  global r;
  global otol;

  c = [];
  ceq = [];

  % Avoid collision with obstacle
  c(1) = (obs(1,3) + r(1) + otol) - sqrt((e_1(1)+des_1(1) - obs(1,1))^2 + (e_1(2)+des_1(2) - obs(1,2))^2);

  c(2) = e_1(3) + des_1(3) - pi;
  c(3) = -e_1(3) - des_1(3) - pi;
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


%% Control Constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [A, b, Aeq, beq, lb, ub] = linearconstraints_1(t_1, x_1, u_1)
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];

    % u constraints
    lb  = -10;
    ub  = 10;
end


%% Output Results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function printHeader_1()

  fprintf('------|---------------------------------------------------------------------\n');
  fprintf('   k  |    u1_1(t)      u2_1(t)      e1_1(t)     e1_2(t)     e1_3(t)    Time\n');
  fprintf('------|---------------------------------------------------------------------\n');

end


function printClosedloopData_1(mpciter, u_1, e_1, t_Elapsed_1)

  fprintf(' %3d  | %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f', ...
    mpciter, u_1(1), u_1(2), e_1(1), e_1(2), e_1(3), t_Elapsed_1);
end
