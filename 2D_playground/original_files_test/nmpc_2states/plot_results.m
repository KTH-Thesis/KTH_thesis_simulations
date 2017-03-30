function plot_results(mpciterations, T, t, x_des, x, u)

time = mpciterations*T;

%% Plot States

fig1 = figure(1);
title('$States \ of \ the \ Object$','interpreter','latex','fontsize',16);
xlabel('$t \ [sec]$','interpreter','latex','fontsize',16);
ylabel({'$x_1(t), x_2(t)$'},'interpreter','latex','fontsize',16);
grid on;
axis([0 time-0.1 -1 1]);
hold on;
for i=1:2
    plot(t,x(:,i)+x_des(i)*ones(size(t)), 'LineWidth',2);
end
legend({'$x_1(t)$','$x_2(t)$'},'interpreter','latex','fontsize',16);
print('states','-depsc','-r500');
%% Plot Errors

fig5 = figure(5);
title('$Errors \ of \ the \ Object$','interpreter','latex','fontsize',16);
xlabel('$t \ [sec]$','interpreter','latex','fontsize',16);
ylabel({'$e_1(t), e_2(t)$'},'interpreter','latex','fontsize',16);
grid on;
axis([0 time-0.1 -1 1]);
hold on;
for i=1:2
    plot(t,x(:,i), 'LineWidth',2);
end
legend({'$e_1(t)$','$e_2(t)$'},'interpreter','latex','fontsize',16);
print('errors','-depsc','-r500');

%% Plot Control Inputs

u

% 
j = 1;
u1 = zeros(mpciterations,1);
u2 = zeros(mpciterations,1);
for i=1:2:(2*mpciterations-1)
    u1(j) = u(i);
    u2(j) = u(i+1);
    j = j+1;
end

fig7 = figure(7);
title('$Control \ Inputs$','interpreter','latex','fontsize',16);
xlabel('$t \ [sec]$','interpreter','latex','fontsize',16);
ylabel('$u_1(t), i \in \{1,\dots,2\}$','interpreter','latex','fontsize',16);
grid on;
axis([0 time-0.1 -1.2 1.2]);
hold on;

plot(t,u1,'LineWidth',2);
plot(t,u2,'LineWidth',2);
legend({'$u_1(t)$','$u_2(t)$'},'interpreter','latex','fontsize',16, 'Location','southeast');
print('control_inputs','-depsc','-r500');

save('variables.mat');
end