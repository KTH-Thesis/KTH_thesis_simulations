clear all
close all

load('uU_1.mat')

u1 = [];
u2 = [];

for i = 1:size(uU_1,1)
  if mod(i,2) == 1
    u1 = [u1, uU_1(i)];
  else
    u2 = [u2, uU_1(i)];
  end
end

uU_1_proper = [u1; u2];
save('uU_1_proper.mat')


load('uU_2.mat')

u1 = [];
u2 = [];

for i = 1:size(uU_2,1)
  if mod(i,2) == 1
    u1 = [u1, uU_2(i)];
  else
    u2 = [u2, uU_2(i)];
  end
end

uU_2_proper = [u1; u2];
save('uU_2_proper.mat')


load('uU_3.mat')

u1 = [];
u2 = [];

for i = 1:size(uU_3,1)
  if mod(i,2) == 1
    u1 = [u1, uU_3(i)];
  else
    u2 = [u2, uU_3(i)];
  end
end

uU_3_proper = [u1; u2];
save('uU_3_proper.mat')