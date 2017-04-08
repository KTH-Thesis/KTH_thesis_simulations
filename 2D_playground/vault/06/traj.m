load('tT_1.mat')
load('tT_2.mat')
load('uU_1.mat')
load('uU_2.mat')
load('xX_1.mat')
load('xX_2.mat')

figure(1)
grid
axis([-7 7 0 5])
axis equal
hold on
filename = 'trajectories.gif';

for i=1:size(tT_1,1)
  
  plot(des_1(1), des_1(2), 'X')
  plot(des_2(1), des_2(2), 'X')
  
  viscircles([xX_1(i,1) + des_1(1),  xX_1(i,2) + des_1(2)], r(1), 'EdgeColor', 'b')
  viscircles([xX_2(i,1) + des_2(1),  xX_2(i,2) + des_2(2)], r(2), 'EdgeColor', 'r')
  viscircles([obs(:,1), obs(:,2)], obs(:,3), 'EdgeColor', 'k')

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


% Distances of agent 1 from obstacles
figure
hold on
grid
plot(sqrt((xX_1(:,1)+(des_1(1)-obs(1,1))*ones(size(tT_1))).^2 + (xX_1(:,2)+(des_1(2)-obs(1,2))*ones(size(tT_1))).^2));
plot(sqrt((xX_1(:,1)+(des_1(1)-obs(2,1))*ones(size(tT_1))).^2 + (xX_1(:,2)+(des_1(2)-obs(2,2))*ones(size(tT_1))).^2));

% Distances of agent 2 from obstacles
figure
hold on
grid
plot(sqrt((xX_2(:,1)+(des_2(1)-obs(1,1))*ones(size(tT_1))).^2 + (xX_2(:,2)+(des_2(2)-obs(1,2))*ones(size(tT_2))).^2));
plot(sqrt((xX_2(:,1)+(des_2(1)-obs(2,1))*ones(size(tT_1))).^2 + (xX_2(:,2)+(des_2(2)-obs(2,2))*ones(size(tT_2))).^2));

% Distance of agent 1 from agent 2
figure
hold on
grid
plot(...
  sqrt(...
    (xX_1(:,1) - xX_2(:,1)+ (des_1(1) - des_2(1))*ones(size(tT_1))).^2 + ...
    (xX_1(:,2) - xX_2(:,2)+ (des_1(2) - des_2(2))*ones(size(tT_1))).^2 ...
  ) ...
);

% errors
figure
plot(sqrt(xX_1(:,1).^2 + xX_1(:,2).^2))
figure
plot(sqrt(xX_2(:,1).^2 + xX_2(:,2).^2))