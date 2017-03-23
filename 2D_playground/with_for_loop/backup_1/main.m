function main

  %% Initialization
  clc;
  clear all;
  close all;
  tic;

  %% Global parameters

  global t;
  global x; % THIS IS THE ERROR:- the formalization is due to the api of nmpc.m
  global u;
  global des_1;
  global u_open_loop;

  % parameters of agent 1


  total_iterations = 20;

  %% NMPC Parameters

  mpciterations = 1;
  N             = 3;       % length of Horizon
  T             = 0.1;     % sampling time
  tmeasure      = 0.0;       % t_0
  x_init        = [0 0];   % x_0
  des_1         = [1 1];   % x_des
  xmeasure      = x_init - des_1;
  u0            = 10*ones(2,N); % initial guess
  tol_opt       = 1e-8;
  opt_option    = 1;
  iprint        = 5;
  type          = 'differential equation';
  atol_ode_real = 1e-12;
  rtol_ode_real = 1e-12;
  atol_ode_sim  = 1e-4;
  rtol_ode_sim  = 1e-4;

  tT = [];
  xX = [];
  uU = [];


  for i=1:total_iterations

    fprintf('iteration %d', i);


    tT = [tT; tmeasure];
    xX = [xX; xmeasure];


    nmpc(@runningcosts, @terminalcosts, @constraints, ...
      @terminalconstraints, @linearconstraints, @system_ct, ...
      mpciterations, N, T, tmeasure, xmeasure, u0, ...
      tol_opt, opt_option, ...
      type, atol_ode_real, rtol_ode_real, atol_ode_sim, rtol_ode_sim, ...
      iprint, @printHeader, @printClosedloopData);

    tmeasure      = t(end);     % new t_0
    x_init        = x(end,:);   % new x_0
    xmeasure      = x_init;
    u0            = [u_open_loop(:,2:size(u_open_loop,2)) u_open_loop(:,size(u_open_loop,2))];

    uU = [uU; u];


  end


  % Save variables
  % save('variables.mat');
  save('xX.mat');
  save('uU.mat');
  save('tT.mat');

  %% Plots
  plot_results(total_iterations, T, tT, des_1, xX, uU);
  toc;
end






%% Running Cost

function cost = runningcosts(t, e, u)

  e=e';
  Q = 10*eye(2);
  R = 2*eye(2);
  cost = e'*Q*e + u'*R*u;
end
%% Terminal Cost

function cost = terminalcosts(t, e)

   e = e';
   P = 10*eye(2);
   cost = e'*P*e;
end
%% Constraints

function [c,ceq] = constraints(t, e, u)

    global des_1;

    c = [];
    ceq = [];

    c(1) = 0.09 - (e(1)+des_1(1) - 0.5)^2 - (e(2)+des_1(2) - 0.5)^2;

end
%% Terminal Constraints

function [c,ceq] = terminalconstraints(t, e)

  c = [];
  ceq = [];


  c(1) = e(1)^2 + e(2)^2 - 0.01;

end
%% Control Constraints

function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x, u)
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];

    % u constraints
    lb  = -1;
    ub  = 1;
end


%% Output Results

function printHeader()

  fprintf('------|-----------------------------------------------------------\n');
  fprintf('   k  |      u1(t)        u2(t)        e1(t)       e2(t)      Time\n');
  fprintf('------|-----------------------------------------------------------\n');

end

function printClosedloopData(mpciter, u, e, t_Elapsed)
  fprintf(' %3d  | %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f', mpciter, u(1), u(2), e(1), e(2), t_Elapsed);
end
