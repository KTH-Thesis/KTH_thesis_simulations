function main

  %% Initialization
  clc;
  clear all;
  close all;
  tic;

  %% Global parameters

  num_states = 2;
  num_inputs = 2;

  global t_1;
  global x_1; % THIS IS THE ERROR:- the formalization is due to the api of nmpc.m
  global u_1;
  global des_1;
  global u_open_loop_1;
  global x_open_loop_1;


  global obs;
  global r;
  global ptol;
  global omega_v;


  %% NMPC Parameters

  total_iterations = 100;
  mpciterations  = 1;
  N              = 3;       % length of Horizon
  T              = 0.1;     % sampling time
  tol_opt        = 1e-8;
  opt_option     = 0;
  iprint         = 5;
  type           = 'differential equation';
  atol_ode_real  = 1e-12;
  rtol_ode_real  = 1e-12;
  atol_ode_sim   = 1e-4;
  rtol_ode_sim   = 1e-4;

  % Proximity tolerance
  ptol           = 0.01;
  omega_v        = 0.001;

  % init Agent 1
  tmeasure_1     = 0.0;         % t_0
  x_init_1       = [0, 0];   % x_0
  des_1          = [2, 2];   % x_des
  xmeasure_1     = x_init_1 - des_1;
  u0_1           = 10*ones(num_inputs, N); % initial guess

  tT_1 = [];
  xX_1 = [];
  uU_1 = [];


  % obstacles: x_c, y_c, r
  obs = [1, 1, 0.2];

  % radii of agents
  r = [0.1];


  for i=1:total_iterations

    fprintf('iteration %d\n', i);


    % Solve for agent 1
    tT_1 = [tT_1; tmeasure_1];
    xX_1 = [xX_1; xmeasure_1];

    nmpc_1(@runningcosts_1, @terminalcosts_1, @constraints_1, ...
      @terminalconstraints_1, @linearconstraints_1, @system_ct_1, ...
      mpciterations, N, T, tmeasure_1, xmeasure_1, u0_1, ...
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


  %% Plots

  % Plot trajectory of agent 1
  plot_results(total_iterations, T, tT_1, des_1, xX_1, uU_1, obs, 1);

  toc;
end



%% Running Cost %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function cost_1 = runningcosts_1(t_1, e_1, u_1)

  e_1=e_1';
  Q_1 = 10*eye(2);
  R_1 = 2*eye(2);
  cost_1 = e_1'*Q_1*e_1 + u_1'*R_1*u_1;
end


%% Terminal Cost %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function cost_1 = terminalcosts_1(t_1, e_1)

   e_1 = e_1';
   P_1 = 100*eye(2);
   cost_1 = e_1'*P_1*e_1;
end


%% Constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [c,ceq] = constraints_1(t_1, e_1, u_1)

  global des_1;
  global obs;
  global r;
  global ptol;
  global d_min;
  global d_max;
  global x_open_loop_1;

  %disp('in constraints_1')
  n_1 = size(x_open_loop_1, 1);

  c = [];
  ceq = [];

  % Avoid collision with obstacle
  c(1) = (obs(1,3) + r(1) + ptol)^2 ...
    - (e_1(1)+des_1(1) - obs(1,1))^2 ...
    - (e_1(2)+des_1(2) - obs(1,2))^2;

end




%% Terminal Constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [c,ceq] = terminalconstraints_1(t_1, e_1)

  global omega_v;

  c = [];
  ceq = [];


  c(1) = sqrt(e_1(1)^2 + e_1(2)^2) - omega_v;

end



%% Control Constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [A, b, Aeq, beq, lb, ub] = linearconstraints_1(t_1, x_1, u_1)
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];

    % u constraints
    lb  = -2;
    ub  = 2;
end


%% Output Results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function printHeader_1()

  fprintf('------|---------------------------------------------------------\n');
  fprintf('   k  |    u1_1(t)      u2_1(t)      e1_1(t)     e2_1(t)    Time\n');
  fprintf('------|---------------------------------------------------------\n');

end

function printClosedloopData_1(mpciter, u_1, e_1, t_Elapsed_1)
  fprintf(' %3d  | %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f', ...
    mpciter, u_1(1), u_1(2), e_1(1), e_1(2), t_Elapsed_1);
end
