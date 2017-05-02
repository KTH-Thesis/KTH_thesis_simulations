load('tT_1.mat')
load('uU_1.mat')
load('xX_1.mat')

figure(1)
hold on
grid
axis([-7 7 0 5])
axis equal
% filename = 'trajectories.gif';
% for i=1:size(tT_1,1)
%   cla
%   plot(des_1(1), des_1(2), 'X', 'Color', 'b')
% 
%   viscircles([xX_1(i,1) + des_1(1),  xX_1(i,2) + des_1(2)], r(1), 'EdgeColor', 'b')
%   
%   plot(xX_1(1:i,1) + des_1(1),  xX_1(1:i,2) + des_1(2) + r(1), 'Color', 'b')
%   plot(xX_1(1:i,1) + des_1(1),  xX_1(1:i,2) + des_1(2) - r(1), 'Color', 'b')
% 
% 
%   viscircles([obs(1,1), obs(1,2)], obs(1,3), 'EdgeColor', 'k')
%   viscircles([obs(2,1), obs(2,2)], obs(2,3), 'EdgeColor', 'k')
% 
%   pause()
% 
%   drawnow
%   frame = getframe(1);
%   im = frame2im(frame);
%   [imind,cm] = rgb2ind(im,256);
%   if i == 1;
%    imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
%   else
%    imwrite(imind,cm,filename,'gif','WriteMode','append');
%   end
%   
% end


% Distances of agent 1 to the obstacles
figure
hold on
plot(sqrt((xX_1(:,1)+(des_1(1)-obs(1,1))*ones(size(tT_1))).^2 + (xX_1(:,2)+(des_1(2)-obs(1,2))*ones(size(tT_1))).^2));
plot(sqrt((xX_1(:,1)+(des_1(1)-obs(2,1))*ones(size(tT_1))).^2 + (xX_1(:,2)+(des_1(2)-obs(2,2))*ones(size(tT_1))).^2));
grid

% Errors
figure
hold on
plot(xX_1(:,1))
plot(xX_1(:,2))
plot(xX_1(:,3))
grid

% Inputs
figure
plot(uU_1(1,:))
hold on
plot(uU_1(2,:))
grid

% V
V = zeros(size(xX_1,1), 1);
for i = 1:size(xX_1,1)
  V(i) = xX_1(i,:) * P * xX_1(i,:)'
end

figure
plot(V)
grid