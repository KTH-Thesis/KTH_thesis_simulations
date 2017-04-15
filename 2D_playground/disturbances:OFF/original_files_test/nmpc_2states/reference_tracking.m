function reference_tracking
%% A NMPC scheme for a second order system

%% Initialization

tic;
addpath('./nmpcroutine');
clear all;
close all;
global t;
global x_des;
global x;
global u;
%% NMPC parameters

mpciterations = 100;
N             = 3;
T             = 0.1;
tmeasure      = 0.0;
x_init        = [0 0.5];
x_des         = [2 0.5];
xmeasure      = x_init-x_des;
u0            = 1*ones(2,N);
tol_opt       = 1e-8;
opt_option    = 0;
iprint        = 5;
type          = 'differential equation';
atol_ode_real = 1e-12;
rtol_ode_real = 1e-12;
atol_ode_sim  = 1e-4;
rtol_ode_sim  = 1e-4;

nmpc(@runningcosts, @terminalcosts, @constraints, ...
     @terminalconstraints, @linearconstraints, @system_ct, ...
     mpciterations, N, T, tmeasure, xmeasure, u0, ...
     tol_opt, opt_option, ...
     type, atol_ode_real, rtol_ode_real, atol_ode_sim, rtol_ode_sim, ...
     iprint, @printHeader, @printClosedloopData, @plotTrajectories);

plot_results(mpciterations, T, t, x_des, x, u);
rmpath('./nmpcroutine');
toc;
end
%% Running Cost

function cost = runningcosts(t, e, u)
   Q = 1*eye(2);
   R = 1;
   e = e';
   cost = e'*Q*e+u'*R*u;
end
%% Terminal Costs

function cost = terminalcosts(t, e)
   P = 1*eye(2);
   e = e';
   cost = e'*P*e;
end
%% State Constraints

function [c,ceq] = constraints(t, e, u)
  c = [];
%   global x_des
   %c(1) = e(1)+x_des(1)-0.6;
   %c(2) = -e(1)-x_des(1)-0.6;
   %c(3) = e(2)+x_des(2)-0.6;
   %c(4) = -e(2)-x_des(2)-0.6;

   ceq = [];
end
%% Terminal Constraints

function [c,ceq] = terminalconstraints(t, e)
    eps = 0.001;

    c(1) = e(1)-eps;
    c(2) = -e(1)-eps;
    c(3) = e(2)-eps;
    c(4) = -e(2)-eps;
    ceq = [];
end
%% Control Input Constraints

function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x, u)
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
    lb  = -2;
    ub  =  2;
end
%% System

function dx = system_ct(t, x, u, T)
    global x_des
    state = x + x_des';
    u = u(:,1);

    dx = [-state(1) + u(1); -state(2) + u(2)];
end
%%  Output

function printHeader()
    fprintf('   k  |      u(1)        u(2)        x(1)        x(2)     Time\n');
    fprintf('--------------------------------------------------\n');
end

function printClosedloopData(mpciter, u, x, t_Elapsed)

   fprintf(' %3d  | %+11.6f %+11.6f %+11.6f %+11.6f  %+6.3f', mpciter, ...
             u(1,1), u(2,1), x(1), x(2), t_Elapsed);

 end

function plotTrajectories(dynamic, system, T, t0, x0, u, atol_ode, rtol_ode, type)

% a function that nmpc calls. not useful at all
%[x, t_intermediate, e_intermediate] = dynamic(system, T, t0, x0, u, atol_ode, rtol_ode, type);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
