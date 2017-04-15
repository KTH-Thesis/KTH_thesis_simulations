load('tT_1.mat')
load('tT_2.mat')
load('uU_1.mat')
load('uU_2.mat')
load('xX_1.mat')
load('xX_2.mat')

figure
hold on
grid
axis([-10 10 0 5])
axis equal
for i=1:size(tT_1,1)

  plot(des_1(1), des_1(2), 'X')
  plot(des_2(1), des_2(2), 'X')

  viscircles([xX_1(i,1) + des_1(1),  xX_1(i,2) + des_1(2)], r(1), 'EdgeColor', 'b')
  viscircles([xX_2(i,1) + des_2(1),  xX_2(i,2) + des_2(2)], r(2), 'EdgeColor', 'r')

  pause()
  cla
end
