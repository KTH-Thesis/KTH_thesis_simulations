close all
load('tT_1.mat')
load('tT_2.mat')
load('uU_1.mat')
load('uU_2.mat')
load('xX_1.mat')
load('xX_2.mat')

figure(1)
hold on
grid
axis([-10 10 0 5])
axis equal
filename = 'trajectories.gif';
for i=1:size(tT_1,1)-1

  plot(des_1(1), des_1(2), 'X')
  plot(des_2(1), des_2(2), 'X')

  viscircles([xX_1(i,1) + des_1(1),  xX_1(i,2) + des_1(2)], r(1), 'EdgeColor', 'b')
  viscircles([xX_2(i,1) + des_2(1),  xX_2(i,2) + des_2(2)], r(2), 'EdgeColor', 'r')
  viscircles([obs(1,1), obs(1,2)], obs(1,3), 'EdgeColor', 'k')

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


% Distances of agent 1 and agent 2 to the obstacle
figure
plot(sqrt((xX_1(:,1)+(des_1(1)-obs(1,1))*ones(size(tT_1))).^2 + (xX_1(:,2)+(des_1(2)-obs(1,2))*ones(size(tT_1))).^2));
hold on
plot(sqrt((xX_2(:,1)+(des_2(1)-obs(1,1))*ones(size(tT_2))).^2 + (xX_2(:,2)+(des_2(2)-obs(1,2))*ones(size(tT_2))).^2));
grid

% Distance of agent 1 to agent 2
figure
hold on
grid
plot(...
  sqrt(...
    (xX_1(:,1) - xX_2(:,1)+ (des_1(1) - des_2(1))*ones(size(tT_1))).^2 + ...
    (xX_1(:,2) - xX_2(:,2)+ (des_1(2) - des_2(2))*ones(size(tT_2))).^2 ...
  ) ...
);

% errors
figure
hold on
plot(xX_1(:,1))
plot(xX_1(:,2))
plot(xX_1(:,3))
grid

figure
hold on
plot(xX_2(:,1))
plot(xX_2(:,2))
plot(xX_2(:,3))
grid


% Inputs
figure
plot(uU_1(1,:))
hold on
plot(uU_1(2,:))
grid

figure
plot(uU_2(1,:))
hold on
plot(uU_2(2,:))
grid

% V
V_1 = zeros(size(xX_1,1), 1);
V_2 = zeros(size(xX_2,1), 1);
for i = 1:size(xX_1,1)
  V_1(i) = xX_1(i,:) * P * xX_1(i,:)'
  V_2(i) = xX_2(i,:) * P * xX_2(i,:)'
end

figure
plot(V_1)
hold on
plot(V_2)
grid
axis([15 size(V_1,1) 0 0.01])