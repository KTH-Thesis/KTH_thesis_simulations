function main

  %% Initialization
  clc;
  clear all;
  close all;
  tic;

  %% Global parameters

  num_states = 2;
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


  % Obstacles
  global obs;
  global r;

  % Proximities
  global d_min;
  global d_max;
  global ptol;
  global omega_v;


  %% NMPC Parameters

  total_iterations = 100;
  mpciterations  = 1;
  N              = 5;       % length of Horizon
  T              = 0.1;     % sampling time
  tol_opt        = 1e-8;
  opt_option     = 1;
  iprint         = 5;
  type           = 'differential equation';
  atol_ode_real  = 1e-12;
  rtol_ode_real  = 1e-12;
  atol_ode_sim   = 1e-4;
  rtol_ode_sim   = 1e-4;


  % init Agent 1
  tmeasure_1     = 0.0;         % t_0
  x_init_1       = [0, 3.5];   % x_0
  des_1          = [12.5, 3.5];   % x_des
  xmeasure_1     = x_init_1 - des_1;
  u0_1           = 10*ones(num_inputs, N); % initial guess

  tT_1           = [];
  xX_1           = [];
  uU_1           = [];


  % init Agent 2
  tmeasure_2     = 0.0;         % t_0
  x_init_2       = [0, 2.3];   % x_0
  des_2          = [12.5, 2.3];   % x_des
  xmeasure_2     = x_init_2 - des_2;
  u0_2           = 10*ones(num_inputs, N); % initial guess

  tT_2           = [];
  xX_2           = [];
  uU_2           = [];


  % obstacles: x_c, y_c, r
  obs            = [5, 2.3, 1];

  % radii of agents
  r              = [0.5; 0.5];

  % Proximity tolerance
  ptol           = 0.01;

  % Terminal cost tolerance
  omega_v        = 0.001;

  % Distance bounds
  d_min          = r(1) + r(2) + ptol;
  d_max          = 2 * (r(1) + r(2)) + ptol;



  for k = 1:total_iterations

    fprintf('iteration %d\n', k);


    % Solve for agent 1 --------------------------------------------------------
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


    % Solve for agent 2 --------------------------------------------------------
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

    % Store the applied input
    uU_2 = [uU_2; u_2];

    save('xX_2.mat');
    save('uU_2.mat');
    save('tT_2.mat');

  end


  %% Plots

  % Plot trajectory of agent 1
  plot_results(total_iterations, T, tT_1, des_1, xX_1, uU_1, obs, 1);

  % Plot trajectory of agent 2
  plot_results(total_iterations, T, tT_2, des_2, xX_2, uU_2, obs, 2);

  % Plot both
  L = findobj(1,'type','line');
  copyobj(L,findobj(2,'type','axes'));
  legend({'$d$'},'interpreter','latex','fontsize',16);
  print('e12','-depsc','-r500');

  %% Plot distance of agents
  figure
  dx = (xX_1(:,1) + des_1(1)*ones(size(tT_1))- xX_2(:,1) - des_2(1)*ones(size(tT_1)));
  dy = (xX_1(:,2) + des_1(2)*ones(size(tT_1))- xX_2(:,2) - des_2(2)*ones(size(tT_1)));
  plot(sqrt(dx.^2 + dy.^2) - (r(1)+r(2))*ones(size(tT_1)));
  legend({'$distance$'},'interpreter','latex','fontsize',16);
  print('distance','-depsc','-r500');

  toc;
end



%% Running Cost %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function cost_1 = runningcosts_1(t_1, e_1, u_1)

  e_1=e_1';

  Q_1 = 10 * eye(2);
  R_1 = 2 * eye(2);

  cost_1 = e_1'*Q_1*e_1 + u_1'*R_1*u_1;
end

function cost_2 = runningcosts_2(t_2, e_2, u_2)

  e_2=e_2';

  Q_2 = 10 * eye(2);
  R_2 = 2*eye(2);

  cost_2 = e_2'*Q_2*e_2 + u_2'*R_2*u_2;
end


%% Terminal Cost %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function cost_1 = terminalcosts_1(t_1, e_1)

   e_1 = e_1';

   P_1 = 10*eye(2);

   cost_1 = e_1'*P_1*e_1;
end

function cost_2 = terminalcosts_2(t_2, e_2)

   e_2 = e_2';

   P_2 = 10*eye(2);

   cost_2 = e_2'*P_2*e_2;
end

%% Constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [c,ceq] = constraints_1(t_1, e_1, u_1)

  global des_1;
  global des_2;
  global obs;
  global r;
  global ptol;
  global d_min;
  global d_max;
  global x_open_loop_1;
  global x_open_loop_2;

  %disp('in constraints_1')
  n_1 = size(x_open_loop_1, 1);
  n_2 = size(x_open_loop_2, 1);

  c = [];
  ceq = [];

  % method: 1= avoid agent 2's current position
  %         2= avoid agent 2's predicted trajectory
  method = 2;


  % Avoid collision with obstacle
   c(1) = (obs(1,3) + r(1) + ptol)^2 - (e_1(1)+des_1(1) - obs(1,1))^2 - (e_1(2)+des_1(2) - obs(1,2))^2;

  if n_2 > 0

    x_open_loop_2_cut = x_open_loop_2(2:3,:);

    if method == 1

      dist_x = e_1(1) + des_1(1) - x_open_loop_2_cut(1,1) - des_2(1);
      dist_y = e_1(2) + des_1(2) - x_open_loop_2_cut(1,2) - des_2(2);

      % Avoid collision with agent 2
      c(2) = d_min^2 - dist_x^2 - dist_y^2;

      % Maintain connectivity with agent 2
      c(3) = dist_x^2 + dist_y^2 - d_max^2;

    elseif method == 2

      % Avoid collision with agent 2 along the entire horizon
      for i=1:2

        dist_x = e_1(1) + des_1(1) - x_open_loop_2_cut(i,1) - des_2(1);
        dist_y = e_1(2) + des_1(2) - x_open_loop_2_cut(i,2) - des_2(2);

        c(1 + i) = d_min^2 - dist_x^2 - dist_y^2;
      end


      % Maintain connectivity with agent 2 along the entire horizon
       for i=1:2

         dist_x = e_1(1) + des_1(1) - x_open_loop_2_cut(i,1) - des_2(1);
         dist_y = e_1(2) + des_1(2) - x_open_loop_2_cut(i,2) - des_2(2);

         c(1 + 2 + i) = sqrt(dist_x^2 + dist_y^2) - d_max;
       end

    end
  end
end




function [c,ceq] = constraints_2(t_2, e_2, u_2)

  global des_1;
  global des_2;
  global obs;
  global r;
  global ptol;
  global d_min;
  global d_max;
  global x_open_loop_1;
  global x_open_loop_2;

  %disp('in constraints_2')
  n_1 = size(x_open_loop_1, 1);
  n_2 = size(x_open_loop_2, 1);

  % method: 1= avoid agent 1's current position
  %         2= avoid agent 1's predicted trajectory
  method = 2;

  c = [];
  ceq = [];

   c(1) = (obs(1,3) + r(2) + ptol)^2 - (e_2(1)+des_2(1) - obs(1,1))^2 - (e_2(2)+des_2(2) - obs(1,2))^2;

  if n_1 > 0

    x_open_loop_1_cut = x_open_loop_1(2:3,:);

    if method == 1

      dist_x = e_2(1) + des_2(1) - x_open_loop_1_cut(1,1) - des_1(1);
      dist_y = e_2(2) + des_2(2) - x_open_loop_1_cut(1,2) - des_1(2);

      % Avoid collision with agent 2
      c(2) = d_min^2 - dist_x^2 - dist_y^2;

      % Maintain connectivity with agent 2
      c(3) = dist_x^2 + dist_y^2 - d_max^2;

    elseif method == 2

      % Avoid collision with agent 2 along the entire horizon
      for i=1:2

        dist_x = e_2(1) + des_2(1) - x_open_loop_1_cut(i,1) - des_1(1);
        dist_y = e_2(2) + des_2(2) - x_open_loop_1_cut(i,2) - des_1(2);

        c(1 + i) = d_min^2 - dist_x^2 - dist_y^2;
      end


      % Maintain connectivity with agent 2 along the entire horizon
       for i=1:2

         dist_x = e_2(1) + des_2(1) - x_open_loop_1_cut(i,1) - des_1(1);
         dist_y = e_2(2) + des_2(2) - x_open_loop_1_cut(i,2) - des_1(2);

         c(1 + 2 + i) = sqrt(dist_x^2 + dist_y^2) - d_max;
       end

     end
  end
end





%% Terminal Constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [c,ceq] = terminalconstraints_1(t_1, e_1)

  global omega_v;

  c = [];
  ceq = [];


  c(1) = sqrt(e_1(1)^2 + e_1(2)^2) - omega_v;

end

function [c,ceq] = terminalconstraints_2(t_2, e_2)

  global omega_v;

  c = [];
  ceq = [];


  c(1) = sqrt(e_2(1)^2 + e_2(2)^2) - omega_v;

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

function [A, b, Aeq, beq, lb, ub] = linearconstraints_2(t_2, x_2, u_2)
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

  fprintf('------|---------------------------------------------------------\n');
  fprintf('   k  |    u1_1(t)      u2_1(t)      e1_1(t)     e2_1(t)    Time\n');
  fprintf('------|---------------------------------------------------------\n');

end

function printHeader_2()

  fprintf('------|---------------------------------------------------------\n');
  fprintf('   k  |    u1_2(t)      u2_2(t)      e1_2(t)     e2_2(t)    Time\n');
  fprintf('------|---------------------------------------------------------\n');

end

function printClosedloopData_1(mpciter, u_1, e_1, t_Elapsed_1)

  fprintf(' %3d  | %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f', ...
    mpciter, u_1(1), u_1(2), e_1(1), e_1(2), t_Elapsed_1);
end

function printClosedloopData_2(mpciter, u_2, e_2, t_Elapsed_2)

  fprintf(' %3d  | %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f', ...
    mpciter, u_2(1), u_2(2), e_2(1), e_2(2), t_Elapsed_2);
end
