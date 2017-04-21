load('tT_1.mat')
load('tT_2.mat')
load('tT_3.mat')
load('uU_1.mat')
load('uU_2.mat')
load('uU_3.mat')
load('xX_1.mat')
load('xX_2.mat')
load('xX_3.mat')

figure(1)
hold on
grid
axis([-10 10 0 5])
axis equal
filename = 'trajectories.gif';
for i=1:size(tT_1,1)
    
  plot(des_1(1), des_1(2), 'X', 'Color', 'b')
  plot(des_2(1), des_2(2), 'X', 'Color', 'r')
  plot(des_3(1), des_3(2), 'X', 'Color', 'g')

  
  viscircles([xX_1(i,1) + des_1(1),  xX_1(i,2) + des_1(2)], r(1), 'EdgeColor', 'b')
  viscircles([xX_2(i,1) + des_2(1),  xX_2(i,2) + des_2(2)], r(2), 'EdgeColor', 'r')
  viscircles([xX_3(i,1) + des_3(1),  xX_3(i,2) + des_3(2)], r(3), 'EdgeColor', 'g')

  viscircles([obs(1,1), obs(1,2)], obs(1,3), 'EdgeColor', 'k')
  viscircles([obs(2,1), obs(2,2)], obs(2,3), 'EdgeColor', 'k')
  
  pause()

   drawnow
   frame = getframe(1);
   im = frame2im(frame);
   [imind,cm] = rgb2ind(im,256);
   if i == 1;
     imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
   else
     imwrite(imind,cm,filename,'gif','WriteMode','append');
   end
  cla
end


% Distances of agents 1,2,3 to the obstacle
figure
hold on
plot(sqrt((xX_1(:,1)+(des_1(1)-obs(1,1))*ones(size(tT_1))).^2 + (xX_1(:,2)+(des_1(2)-obs(1,2))*ones(size(tT_1))).^2));
plot(sqrt((xX_2(:,1)+(des_2(1)-obs(1,1))*ones(size(tT_1))).^2 + (xX_2(:,2)+(des_2(2)-obs(1,2))*ones(size(tT_2))).^2));
plot(sqrt((xX_3(:,1)+(des_3(1)-obs(1,1))*ones(size(tT_1))).^2 + (xX_3(:,2)+(des_3(2)-obs(1,2))*ones(size(tT_2))).^2));
grid

% Distance of agent 1 to agent 2
figure
hold on
grid
plot(...
  sqrt(...
    (xX_1(:,1) - xX_2(:,1)+ (des_1(1) - des_2(1))*ones(size(tT_1))).^2 + ...
    (xX_1(:,2) - xX_2(:,2)+ (des_1(2) - des_2(2))*ones(size(tT_1))).^2 ...
  ) ...
);

% Distance of agent 1 to agent 3
figure
hold on
grid
plot(...
  sqrt(...
    (xX_1(:,1) - xX_3(:,1)+ (des_1(1) - des_3(1))*ones(size(tT_1))).^2 + ...
    (xX_1(:,2) - xX_3(:,2)+ (des_1(2) - des_3(2))*ones(size(tT_1))).^2 ...
  ) ...
);

% Distance of agent 2 to agent 3
figure
hold on
grid
plot(...
  sqrt(...
    (xX_2(:,1) - xX_3(:,1)+ (des_2(1) - des_3(1))*ones(size(tT_1))).^2 + ...
    (xX_2(:,2) - xX_3(:,2)+ (des_2(2) - des_3(2))*ones(size(tT_1))).^2 ...
  ) ...
);

% errors
figure
plot(sqrt(xX_1(:,1).^2 + xX_1(:,2).^2))
figure
plot(sqrt(xX_2(:,1).^2 + xX_2(:,2).^2))