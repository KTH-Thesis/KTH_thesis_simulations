function fig = plot_results(mpciterations, T, t, x_des, e, u, obs, r, id)

  fig = figure(id);
  title('$a$','interpreter','latex','fontsize',16);
  xlabel('$b$','interpreter','latex','fontsize',16);
  ylabel({'$c$'},'interpreter','latex','fontsize',16);
  grid on;
  hold on;

  plot(e(:,1)+x_des(1)*ones(size(t)), e(:,2)+x_des(2)*ones(size(t)))
  viscircles([e(:,1)+x_des(1)*ones(size(t)), e(:,2)+x_des(2)*ones(size(t))], r(id)*ones(size(t)))
  viscircles([obs(:,1), obs(:,2)], obs(:,3))

  filename = strcat('e', num2str(id));
  legend({'$d$'},'interpreter','latex','fontsize',16);
  print(filename,'-depsc','-r500');


  % plot distances from obstacle
  for i=1:size(obs, 1)
    fig = figure(10*id+i)
    title('$a$','interpreter','latex','fontsize',16);
    xlabel('$b$','interpreter','latex','fontsize',16);
    ylabel({'$c$'},'interpreter','latex','fontsize',16);
    grid on;
    hold on;

    plot(sqrt((e(:,1)+(x_des(1)-obs(i,1))*ones(size(t))).^2 + (e(:,2)+(x_des(2)-obs(i,2))*ones(size(t))).^2));


    filename = strcat('obs_to_agent_', num2str(id));
    legend({'$d$'},'interpreter','latex','fontsize',16);
    print(filename,'-depsc','-r500');
  end


end
