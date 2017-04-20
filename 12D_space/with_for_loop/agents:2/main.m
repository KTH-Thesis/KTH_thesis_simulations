function main

  %% Initialization
  clc;
  clear all;
  close all;
  tic;

  %% Global parameters

  global t_1;
  global x_1; % THIS IS THE ERROR:- the formalization is due to the api of nmpc.m
  global u_1;
  global des_1;
  global u_open_loop_1;

  global t_2;
  global x_2; % THIS IS THE ERROR:- the formalization is due to the api of nmpc.m
  global u_2;
  global des_2;
  global u_open_loop_2;

  % Obstacles
  global obs;

  % Radii of agents
  global r;

  % parameters of agent 1

  global m_1;
  global I_x_1;
  global I_y_1;
  global I_z_1;

  m_1 = 0.1;
  I_x_1 = 0.25;
  I_y_1 = 0.25;
  I_z_1 = 0.25;

  % parameters of agent 2

  global m_2;
  global I_x_2;
  global I_y_2;
  global I_z_2;

  m_2 = 0.1;
  I_x_2 = 0.25;
  I_y_2 = 0.25;
  I_z_2 = 0.25;




  %% NMPC Parameters

  total_iterations = 100;
  mpciterations = 1;
  N             = 3;       % length of Horizon
  T             = 0.1;     % sampling time
  tol_opt       = 1e-8;
  opt_option    = 1;
  iprint        = 5;
  type          = 'differential equation';
  atol_ode_real = 1e-12;
  rtol_ode_real = 1e-12;
  atol_ode_sim  = 1e-4;
  rtol_ode_sim  = 1e-4;


  % init agent 1
  tmeasure_1      = 0.0;                                      % t_0
  x_init_1        = [-0.2 0 0, 0 0 0, 0 0 0, 0 0 0];       % x_0
  des_1           = [0.8 1 1, 0 0 0, 0 0 0, 0 0 0];             % x_des
  xmeasure_1      = x_init_1 - des_1;
  u0_1            = 10*ones(6,N);                             % initial guess

  tT_1 = [];
  xX_1 = [];
  uU_1 = [];


  % init agent 2
  tmeasure_2      = 0.0;                            % t_0
  x_init_2        = [0.2 0 0, 0 0 0, 0 0 0, 0 0 0];   % x_0
  des_2           = [1.2 1 1, 0 0 0, 0 0 0, 0 0 0];   % x_des
  xmeasure_2      = x_init_2 - des_2;
  u0_2            = 10*ones(6,N);                   % initial guess

  tT_2 = [];
  xX_2 = [];
  uU_2 = [];

  % obstacles: x_c, y_c, z_c, r
  obs = [0.5, 0.5, 0.5, 0.2];

  % radii of agents
  r = [0.1; 0.1];


%% ITERATIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  for i=1:total_iterations

    fprintf('iteration %d', i);

    % Agent 1 territory
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
    u0_1            = [u_open_loop_1(:,2:size(u_open_loop_1,2)) ...
      u_open_loop_1(:,size(u_open_loop_1,2))];

    uU_1 = [uU_1; u_1];



    % Agent 2 territory
    tT_2 = [tT_2; tmeasure_2];
    xX_2 = [xX_2; xmeasure_2];


    nmpc_2(@runningcosts_2, @terminalcosts_2, @constraints_2, ...
      @terminalconstraints_2, @linearconstraints_2, @system_ct_2, ...
      mpciterations, N, T, tmeasure_2, xmeasure_2, u0_2, ...
      tol_opt, opt_option, ...
      type, atol_ode_real, rtol_ode_real, atol_ode_sim, rtol_ode_sim, ...
      iprint, @printHeader_2, @printClosedloopData_2);

    tmeasure_2      = t_2(end);     % new t_0
    x_init_2        = x_2(end,:);   % new x_0
    xmeasure_2      = x_init_2;
    u0_2            = [u_open_loop_2(:,2:size(u_open_loop_2,2)) u_open_loop_2(:,size(u_open_loop_2,2))];

    uU_2 = [uU_2; u_2];


  end


  % Save variables
  save('xX_1.mat');
  save('uU_1.mat');
  save('tT_1.mat');
  save('xX_2.mat');
  save('uU_2.mat');
  save('tT_2.mat');

  % Plots for agent 1
  plot_results(total_iterations, T, tT_1, des_1, xX_1, uU_1, obs, 1);

  % Plots for agent 2
  plot_results(total_iterations, T, tT_2, des_2, xX_2, uU_2, obs, 2);

  toc;
end






%% Running Cost %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function cost_1 = runningcosts_1(t, e, u)

  e = e';
  Q = 10*eye(12);
  R = 2*eye(6);
  cost_1 = e'*Q*e + u'*R*u;

end

function cost_2 = runningcosts_2(t, e, u)

  e = e';
  Q = 10*eye(12);
  R = 2*eye(6);
  cost_2 = e'*Q*e + u'*R*u;

end



%% Terminal Cost %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function cost_1 = terminalcosts_1(t, e)

   e = e';
   P = 10*eye(12);
   cost_1 = e'*P*e;

end

function cost_2 = terminalcosts_2(t, e)

   e = e';
   P = 10*eye(12);
   cost_2 = e'*P*e;

end



%% Constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [c,ceq] = constraints_1(t, e, u)

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

function [c,ceq] = constraints_2(t, e, u)

    global des_2;

    c = [];
    ceq = [];

    c(1) = 0.09 - (e(1)+des_2(1) - 0.5)^2 - (e(2)+des_2(2) - 0.5)^2 - (e(3)+des_2(3) - 0.5)^2;
    c(2) = e(4) + des_2(4) - pi/2;
    c(3) = -pi/2 - e(4) + des_2(4);

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




%% Terminal Constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [c,ceq] = terminalconstraints_1(t, e)

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

function [c,ceq] = terminalconstraints_2(t, e)

    global des_2;

    c(2) = e(4) + des_2(4) - pi/2;
    c(3) = -pi/2 - e(4) + des_2(4);

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



%% Control Constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [A, b, Aeq, beq, lb, ub] = linearconstraints_1(t, x, u)
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];

    % u constraints
    lb  = -1;
    ub  = 1;
end

function [A, b, Aeq, beq, lb, ub] = linearconstraints_2(t, x, u)
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];

    % u constraints
    lb  = -1;
    ub  = 1;
end


%% Output Results %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function printHeader_1()

  fprintf('------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n');
  fprintf('   k  |      u1_1(t)      u2_1(t)    u3_1(t)    u4_1(t)      u5_1(t)    u6_1(t)      e1_1(t)     e2_1(t)     e3_1(t)     e4_1(t)     e5_1(t)     e6_1(t)     e7_1(t)     e8_1(t)    e9_1(t)     e10_1(t)    e11_1(t)     e12_1(t)    Time\n');
  fprintf('------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n');
end

function printHeader_2()

  fprintf('------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n');
  fprintf('   k  |      u1_2(t)      u2_2(t)    u3_2(t)    u4_2(t)      u5_2(t)    u6_2(t)      e1_2(t)     e2_2(t)     e3_2(t)     e4_2(t)     e5_2(t)     e6_2(t)     e7_2(t)     e8_2(t)    e9_2(t)     e10_2(t)    e11_2(t)     e12_2(t)    Time\n');
  fprintf('------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n');

end



function printClosedloopData_1(mpciter, u, e, t_Elapsed)
  fprintf(' %3d  | %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f', mpciter, ...
    u(1,1), u(2,1), u(3,1), u(4,1), u(5,1), u(6,1), e(1), e(2), e(3), e(4), e(5), e(6), e(7), e(8),...
    e(9), e(10), e(11), e(12), t_Elapsed);
end

function printClosedloopData_2(mpciter, u, e, t_Elapsed)
  fprintf(' %3d  | %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f', mpciter, ...
    u(1,1), u(2,1), u(3,1), u(4,1), u(5,1), u(6,1), e(1), e(2), e(3), e(4), e(5), e(6), e(7), e(8),...
    e(9), e(10), e(11), e(12), t_Elapsed);
end
