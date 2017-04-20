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

  global m_1;
  global I_x_1;
  global I_y_1;
  global I_z_1;

  m_1 = 0.1;
  I_x_1 = 0.25;
  I_y_1 = 0.25;
  I_z_1 = 0.25;



  total_iterations = 100;

  %% NMPC Parameters

  mpciterations = 1;
  N             = 3;       % length of Horizon
  T             = 0.1;     % sampling time
  tmeasure      = 0.0;       % t_0
  x_init        = [0 0 0, 0 0 0, 0 0 0, 0 0 0];   % x_0
  des_1         = [1 1 1, 0 0 0, 0 0 0, 0 0 0];   % x_des
  xmeasure      = x_init - des_1;
  u0            = 10*ones(6,N); % initial guess
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
  Q = 10*eye(12);
  R = 2*eye(6);
  cost = e'*Q*e + u'*R*u;
end
%% Terminal Cost

function cost = terminalcosts(t, e)

   e = e';
   P = 10*eye(12);
   cost = e'*P*e;
end
%% Constraints

function [c,ceq] = constraints(t, e, u)

    global des_1;

    c = [];
    ceq = [];

    c(1) = 0.09 - (e(1)+des_1(1) - 0.5)^2 - (e(2)+des_1(2) - 0.5)^2 - (e(3)+des_1(3) - 0.5)^2;
    c(2) = e(4) + des_1(4) - pi/2;
    c(3) = -pi/2 - e(4) + des_1(4);

%    eps = 0.1;
%
%     c(1) = -e(11)-x_des(11)+eps;
%     c(2) = e(11)+x_des(11)-pi/2;
%     c(3) = e(12)+x_des(12)-pi/2;
%     c(4) = -e(12)-x_des(12)-pi/2;
%     c(5) = e(15)+x_des(15)-eps;
%     c(6) = -e(15)-x_des(15)-pi/2;
%     c(7) = e(16)+x_des(16)-pi/2;
%     c(8) = -e(16)-x_des(16)-pi/2;
%     c(9) = 4-(e(1)+x_des(1)-5)^2-(e(2)+x_des(2)-5)^2-(e(3)+x_des(3)-1)^2;
end
%% Terminal Constraints

function [c,ceq] = terminalconstraints(t, e)

    global des_1;

    c(2) = e(4) + des_1(4) - pi/2;
    c(3) = -pi/2 - e(4) + des_1(4);

%    eps = 10;
%    eps2 = 0.1;
%
%    c(1) = e(1)-eps;
%    c(2) = -e(1)-eps;
%    c(3) = e(2)-eps;
%    c(4) = -e(2)-eps;
%    c(5) = e(3)-eps2;
%    c(6) = -e(3)-eps2;
%    c(7) = e(4)-eps;
%    c(8) = -e(4)-eps;
%    c(9) = e(5)-eps;
%    c(10) = -e(5)-eps;
%    c(11) = e(6)-eps;
%    c(12) = -e(6)-eps;
%    c(13) = e(7)-eps;
%    c(14) = -e(7)-eps;
%    c(15) = e(8)-eps;
%    c(16) = -e(8)-eps;
%    c(17) = e(9)-eps;
%    c(18) = -e(9)-eps;
%    c(19) = e(10)-eps;
%    c(20) = -e(10)-eps;
%    c(21) = e(11)-eps2;
%    c(22) = -e(11)-eps2;
%    c(23) = e(12)-eps2;
%    c(24) = -e(12)-eps2;
%    c(25) = e(13)-eps;
%    c(26) = -e(13)-eps;
%    c(27) = e(14)-eps;
%    c(28) = -e(14)-eps;
%    c(29) = e(15)-eps2;
%    c(30) = -e(15)-eps2;
%    c(31) = e(16)-eps2;
%    c(32) = -e(16)-eps2;

  c = [];
  ceq = [];
end
%% Control Constraints

function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x, u)
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];

    % u constraints
    lb  = -10;
    ub  = 10;
end


%% Output Results

function printHeader()

  fprintf('------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n');
  fprintf('   k  |      u1(t)        u2(t)      u3(t)      u4(t)        u5(t)      u6(t)        e1(t)       e2(t)       e3(t)       e4(t)       e5(t)       e6(t)       e7(t)       e8(t)      e9(t)       e10(t)      e11(t)       e12(t)      Time\n');
  fprintf('------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n');

end

function printClosedloopData(mpciter, u, e, t_Elapsed)
  fprintf(' %3d  | %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f', mpciter, ...
    u(1,1), u(2,1), u(3,1), u(4,1), u(5,1), u(6,1), e(1), e(2), e(3), e(4), e(5), e(6), e(7), e(8),...
    e(9), e(10), e(11), e(12), t_Elapsed);
end
