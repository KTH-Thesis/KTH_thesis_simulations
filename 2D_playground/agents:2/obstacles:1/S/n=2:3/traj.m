load('tT_1.mat')
load('tT_2.mat')
load('uU_1.mat')
load('uU_2.mat')
load('xX_1.mat')
load('xX_2.mat')

figure
for i=1:size(tT_1,1)

  viscircles([xX_1(i,1) + des_1(1),  xX_1(i,2) + des_1(2)], r(1))
  viscircles([xX_2(i,1) + des_2(1),  xX_2(i,2) + des_2(2)], r(2))
  viscircles([obs(1,1), obs(1,2)], obs(1,3))
  grid
  axis([-1 3 0 1])
  axis equal
  pause()
  cla
  grid
end
