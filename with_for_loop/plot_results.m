function plot_results(mpciterations, T, t, x_des, e, u)

  time = mpciterations * T;
  
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
  print('states_xyz_of_agent','-depsc','-r500');
  
  
  
  %% Plot phi,theta,psi States

  fig2 = figure(2);
  title('$Euler \ angles \ of \ the \ Agent$','interpreter','latex','fontsize',16);
  xlabel('$t \ [sec]$','interpreter','latex','fontsize',16);
  ylabel('$\phi(t), \theta(t), \psi(t)$','interpreter','latex','fontsize',16);
  grid on;
  axis([0 time-0.1 -1.85 6.5]);
  hold on;
  for i=4:6
      plot(t, e(:,i) + x_des(i)*ones(size(t)), 'LineWidth',2);
  end
  legend({'$\phi(t)$','$\theta(t)$', '$\psi(t)$'},'interpreter','latex','fontsize',16);
  print('states_phi_theta_psi_agent','-depsc','-r500');
  
  
  
  %% Plot x_dot, y_dot, z_dot States

  fig3 = figure(3);
  title('$Velocities \ of \ the \ Agent$','interpreter','latex','fontsize',16);
  xlabel('$t \ [sec]$','interpreter','latex','fontsize',16);
  ylabel('$\dot{x}(t), \dot{y}(t), \dot{z}(t)$','interpreter','latex','fontsize',16);
  grid on;
  axis([0 time-0.1 -1.85 6.5]);
  hold on;
  for i=7:9
      plot(t,e(:,i) + x_des(i)*ones(size(t)), 'LineWidth',2);
  end
  legend({'$\dot{x}(t)$','$\dot{y}(t)$', '$\dot{z}(t)$'},'interpreter','latex','fontsize',16);
  print('states_dot_xyz_agent','-depsc','-r500');
  
  
  
  %% Plot omega_x, omega_y, omega_z States

  fig4 = figure(4);
  title('$Angular \ velocities \ of \ Agent$','interpreter','latex','fontsize',16);
  xlabel('$t \ [sec]$','interpreter','latex','fontsize',16);
  ylabel('$\omega_{\scriptscriptstyle x}(t), \omega_{\scriptscriptstyle y}(t), \omega_{\scriptscriptstyle z}(t)$','interpreter','latex','fontsize',16);
  grid on;
  axis([0 time-0.1 -0.5 22]);
  hold on;
  for i=10:12
      plot(t, e(:,i) + x_des(i)*ones(size(t)), 'LineWidth',2);
  end
  legend({'$\omega_{\scriptscriptstyle z}(t)$','$\omega_{\scriptscriptstyle y}(t)$','$\omega_{\scriptscriptstyle z}(t)$'},'interpreter','latex','fontsize',16);
  print('states_angular_velocities_agent','-depsc','-r500');
  
  

  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  %% Plot Control Inputs

  
  u1 = zeros(mpciterations,1);
  u2 = zeros(mpciterations,1);
  u3 = zeros(mpciterations,1);
  u4 = zeros(mpciterations,1);
  u5 = zeros(mpciterations,1);
  u6 = zeros(mpciterations,1);
  u7 = zeros(mpciterations,1);
  u8 = zeros(mpciterations,1);
  j = 1;
  for i=1:6:(6*mpciterations-7)
      u1(j) = u(i);
      u2(j) = u(i+1);
      u3(j) = u(i+2);
      u4(j) = u(i+3);
      u5(j) = u(i+4);
      u6(j) = u(i+5);
      j = j+1;
  end

  fig9 = figure(9);
  title('$Control \ Inputs$','interpreter','latex','fontsize',16);
  xlabel('$t \ [sec]$','interpreter','latex','fontsize',16);
  ylabel('$u_i(t), i \in \{1,\dots,6\}$','interpreter','latex','fontsize',16);
  grid on;
  axis([0 time-0.1 -11 11]);
  hold on;
  
  plot(t,u1,'LineWidth',2);
  plot(t,u2,'LineWidth',2);
  plot(t,u3,'LineWidth',2);
  plot(t,u4,'LineWidth',2);
  plot(t,u5,'LineWidth',2);
  plot(t,u6,'LineWidth',2);
  legend({'$u_1(t)$','$u_2(t)$', '$u_3(t)$', '$u_4(t)$', '$u_5(t)$', '$u_6(t)$'},'interpreter','latex','fontsize',16, 'Location','southeast');
  print('control_inputs','-depsc','-r500');
  
  
  
  %% Save variables

%   save('variables.mat');
end