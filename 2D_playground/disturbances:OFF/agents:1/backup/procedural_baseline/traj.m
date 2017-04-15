load('tT_1.mat')
load('uU_1.mat')
load('xX_1.mat')

figure
for i=1:size(x_1,1)

  viscircles([x_1(i,1) + des_1(1),  x_1(i,2) + des_1(2)], r(1))
  viscircles([obs(1,1), obs(1,2)], obs(1,3))
  grid
  axis([-1 3 0 1])
  axis equal
  pause()
  cla
  grid
end
