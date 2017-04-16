function fig = plot_results(mpciterations, T, t, x_des, e, u, r, id)

  fig = figure(id);
  title('$a$','interpreter','latex','fontsize',16);
  xlabel('$b$','interpreter','latex','fontsize',16);
  ylabel({'$c$'},'interpreter','latex','fontsize',16);
  grid on;
  hold on;

  plot(e(:,1)+x_des(1)*ones(size(t)), e(:,2)+x_des(2)*ones(size(t)))
  viscircles([e(:,1)+x_des(1)*ones(size(t)), e(:,2)+x_des(2)*ones(size(t))], r(1)*ones(size(t)))

  filename = strcat('e', num2str(id));
  legend({'$d$'},'interpreter','latex','fontsize',16);
  print(filename,'-depsc','-r500');

end
