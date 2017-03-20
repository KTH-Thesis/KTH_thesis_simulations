function plot_results(mpciterations, T, t, x_des, e, u)

  time = mpciterations * T;
  
  size(e)

  %% Plot xyz States

  fig1 = figure(1);
  title('$States \ of \ the \ Agent$','interpreter','latex','fontsize',16);
  xlabel('$t \ [sec]$','interpreter','latex','fontsize',16);
  ylabel({'$x(t), y(t), z(t)$'},'interpreter','latex','fontsize',16);
  grid on;
  axis([0 time-0.1 -2.5 19.5]);
  hold on;
  for i=1:3
      plot(t, e(:,i) + x_des(i)*ones(size(t)), 'LineWidth',2);
  end
  legend({'$x(t)$','$y(t)$', '$z(t)$'},'interpreter','latex','fontsize',16);
  print('states_of_object','-depsc','-r500');
  
  %% Plot States

  fig2 = figure(2);
  title('$Velocities \ of \ the \ Object$','interpreter','latex','fontsize',16);
  xlabel('$t \ [sec]$','interpreter','latex','fontsize',16);
  ylabel('$\dot{x}_o(t), \dot{y}_o(t), \dot{z}_o(t), \dot{\phi}_o(t)$','interpreter','latex','fontsize',16);
  grid on;
  axis([0 time-0.1 -1.85 6.5]);
  hold on;
  for i=5:8
      plot(t,e(:,i)+x_des(i)*ones(size(t)), 'LineWidth',2);
  end
  legend({'$\dot{x}_o(t)$','$\dot{y}_o(t)$', '$\dot{z}_o(t)$', '$\dot{\phi}_o(t)$'},'interpreter','latex','fontsize',16);
  print('velocities_of_object','-depsc','-r500');
  %% Plot States

  fig3 = figure(3);
  title('$States \ of \ Agent \ 1$','interpreter','latex','fontsize',16);
  xlabel('$t \ [sec]$','interpreter','latex','fontsize',16);
  ylabel('$x_{\scriptscriptstyle B_1}(t), y_{\scriptscriptstyle B_1}(t), \alpha_{\scriptscriptstyle 1_1}(t), \alpha_{\scriptscriptstyle 1_2}(t)$','interpreter','latex','fontsize',16);
  grid on;
  axis([0 time-0.1 -0.5 22]);
  hold on;
  for i=9:12
      plot(t,e(:,i)+x_des(i)*ones(size(t)), 'LineWidth',2);
  end
  legend({'$x_{\scriptscriptstyle B_1}(t)$','$y_{\scriptscriptstyle B_1}(t)$','$\alpha_{\scriptscriptstyle 1_1}(t)$', '$\alpha_{\scriptscriptstyle 1_2}(t)$'},'interpreter','latex','fontsize',16);
  print('states_of_agent_1','-depsc','-r500');
  %% Plot States

  fig4 = figure(4);
  title('$States \ of \ Agent \ 2$','interpreter','latex','fontsize',16);
  xlabel('$t \ [sec]$','interpreter','latex','fontsize',16);
  ylabel('$x_{\scriptscriptstyle B_2}(t), y_{\scriptscriptstyle B_2}(t), \alpha_{\scriptscriptstyle 2_1}(t), \alpha_{\scriptscriptstyle 2_2}(t)$','interpreter','latex','fontsize',16);
  grid on;
  axis([0 time-0.1 -4.5 17.5]);
  hold on;
  for i=13:16
      plot(t,e(:,i)+x_des(i)*ones(size(t)), 'LineWidth',2);
  end
  legend({'$x_{\scriptscriptstyle B_2}(t)$','$y_{\scriptscriptstyle B_2}(t)$','$\alpha_{\scriptscriptstyle 2_1}(t)$', '$\dot{\alpha}_{\scriptscriptstyle 2_2}(t)$'},'interpreter','latex','fontsize',16);
  print('states_of_agent_2','-depsc','-r500');
  %% Plot Errors

  fig5 = figure(5);
  title('$Errors \ of \ the \ Object$','interpreter','latex','fontsize',16);
  xlabel('$t \ [sec]$','interpreter','latex','fontsize',16);
  ylabel('$e_i(t), i \in \{1,\dots,8\}$','interpreter','latex','fontsize',16);
  grid on;
  axis([0 time-0.1 -12.5 9.5]);
  hold on;
  for i=1:8
      plot(t,e(:,i), 'LineWidth',2);
  end
  legend({'$e_1(t)$','$e_2(t)$','$e_3(t)$','$e_4(t)$','$e_5(t)$','$e_6(t)$','$e_7(t)$','$e_8(t)$'},'interpreter','latex','fontsize',16);
  print('errors_plot_object','-depsc','-r500');

  %% Plot Errors

  fig6 = figure(6);
  title('$Errors \ of \ the \ Agents $','interpreter','latex','fontsize',16);
  xlabel('$t \ [sec]$','interpreter','latex','fontsize',16);
  ylabel('$e_i(t), i \in \{9,\dots, 16\}$','interpreter','latex','fontsize',16);
  grid on;
  axis([0 time-0.1 -12.5 9.5]);
  hold on;
  for i=9:16
      plot(t,e(:,i), 'LineWidth',2);
  end
  legend({'$e_9(t)$','$e_{10}(t)$','$e_{11}(t)$','$e_{12}(t)$','$e_{13}(t)$','$e_{14}(t)$','$e_{15}(t)$','$e_{16}(t)$'},'interpreter','latex','fontsize',16);
  print('errors_plot_agent','-depsc','-r500');

  %% Plot Control Inputs

  % modify u

  u(6) = 10;
  u(7) = 10;
  u(14) = 10;
  u(15) = 10;
  u(4) = -10;
  u(8) = -10;

  j = 1;
  u1 = zeros(mpciterations,1);
  u2 = zeros(mpciterations,1);
  u3 = zeros(mpciterations,1);
  u4 = zeros(mpciterations,1);
  u5 = zeros(mpciterations,1);
  u6 = zeros(mpciterations,1);
  u7 = zeros(mpciterations,1);
  u8 = zeros(mpciterations,1);
  for i=1:8:(8*mpciterations-7)
      u1(j) = u(i);
      u2(j) = u(i+1);
      u3(j) = u(i+2);
      u4(j) = u(i+3);
      u5(j) = u(i+4);
      u6(j) = u(i+5);
      u7(j) = u(i+6);
      u8(j) = u(i+7);
      j = j+1;
  end

  fig7 = figure(7);
  title('$Control \ Inputs$','interpreter','latex','fontsize',16);
  xlabel('$t \ [sec]$','interpreter','latex','fontsize',16);
  ylabel('$u_i(t), i \in \{1,\dots,8\}$','interpreter','latex','fontsize',16);
  grid on;
  axis([0 time-0.1 -11 11]);
  hold on;

  plot(t,u1,'LineWidth',2);
  plot(t,u2,'LineWidth',2);
  plot(t,u3,'LineWidth',2);
  plot(t,u4,'LineWidth',2);
  plot(t,u5,'LineWidth',2);
  plot(t,u6,'LineWidth',2);
  plot(t,u7,'LineWidth',2);
  plot(t,u8,'LineWidth',2);
  legend({'$u_1(t)$','$u_2(t)$', '$u_3(t)$', '$u_4(t)$', '$u_5(t)$', '$u_6(t)$', '$u_7(t)$', '$u_8(t)$'},'interpreter','latex','fontsize',16, 'Location','southeast');
  print('control_inputs','-depsc','-r500');
  %% Save variables

  save('variables.mat');
end