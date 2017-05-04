close all
clear all
load('xX_1.mat')
load('xX_2.mat')
load('xX_3.mat')
load('tT_1.mat')
load('tT_2.mat')
load('tT_3.mat')
load('uU_1.mat')
load('uU_2.mat')
load('uU_3.mat')

% figure(1)
% hold on
% grid
% axis([-8 8 -2 8])
% axis equal

% for i=1:size(tT_1,1)-1
%   
%   i
% 
%   plot(des_1(1), des_1(2), 'X', 'Color', 'b')
%   plot(des_2(1), des_2(2), 'X', 'Color', 'r')
%   plot(des_3(1), des_3(2), 'X', 'Color', 'g')
% 
%   viscircles([xX_1(i,1) + des_1(1),  xX_1(i,2) + des_1(2)], r(1), 'EdgeColor', 'b')
%   viscircles([xX_2(i,1) + des_2(1),  xX_2(i,2) + des_2(2)], r(2), 'EdgeColor', 'r')
%   viscircles([xX_3(i,1) + des_3(1),  xX_3(i,2) + des_3(2)], r(3), 'EdgeColor', 'g')
% 
%   viscircles([obs(1,1), obs(1,2)], obs(1,3), 'EdgeColor', 'k')
% 
%   pause()
%   drawnow
%   cla
% end

frames = [1,3,6,8,10,13,17,29];

counter = 1;

for i = 1:size(frames,2)
  
  subplot(4,2,counter)
  hold on
 
  axis([-9 9 0 6])
  axis equal
  
  if i < size(frames,2)
    plot(des_1(1), des_1(2), 'X', 'Color', [0.00000,0.44700,0.74100])
    plot(des_2(1), des_2(2), 'X', 'Color', [0.85000,0.32500,0.09800])
    plot(des_3(1), des_3(2), 'X', 'Color', [0.92900,0.69400,0.12500])
    
    centers_connector_x = [xX_1(frames(i),1) + des_1(1), xX_2(frames(i),1) + des_2(1)];
    centers_connector_y = [xX_1(frames(i),2) + des_1(2), xX_2(frames(i),2) + des_2(2)];
    plot(centers_connector_x, centers_connector_y, 'Color', [0.4660    0.6740    0.1880])
        
    centers_connector_x = [xX_1(frames(i),1) + des_1(1), xX_3(frames(i),1) + des_3(1)];
    centers_connector_y = [xX_1(frames(i),2) + des_1(2), xX_3(frames(i),2) + des_3(2)];
    plot(centers_connector_x, centers_connector_y, 'Color', [0.4660    0.6740    0.1880])
  end
  
  
  if i==size(frames,2)
    plot(des_1(1), des_1(2), 'O', 'Color', [0.00000,0.44700,0.74100])
    plot(des_2(1), des_2(2), 'O', 'Color', [0.85000,0.32500,0.09800])
    plot(des_3(1), des_3(2), 'O', 'Color', [0.92900,0.69400,0.12500])
  end

  viscircles([xX_1(frames(i),1) + des_1(1),  xX_1(frames(i),2) + des_1(2)], r(1), 'EdgeColor', [0.00000,0.44700,0.74100])
  viscircles([xX_2(frames(i),1) + des_2(1),  xX_2(frames(i),2) + des_2(2)], r(2), 'EdgeColor', [0.85000,0.32500,0.09800])
  viscircles([xX_3(frames(i),1) + des_3(1),  xX_3(frames(i),2) + des_3(2)], r(3), 'EdgeColor', [0.92900,0.69400,0.12500])
  viscircles([obs(1,1), obs(1,2)], obs(1,3), 'EdgeColor', 'k')
  viscircles([obs(2,1), obs(2,2)], obs(2,3), 'EdgeColor', 'k')

  counter = counter + 1;
end


% matlab2tikz('trajectory_d_OFF_3_2.tex')