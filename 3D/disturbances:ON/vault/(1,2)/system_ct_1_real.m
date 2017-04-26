function dx = system_ct_1_real(t, e, u, T)

  % Desired placements (destinations)
  global des_1;

  % The disturbance
  global disturbance;

  % State vector of agent 1:
  % [x,y]
  state = e' + des_1;

  d = disturbance * cos(2*t);
%   d = disturbance;

  f1 = u(1) * cos(state(3)) + d;
  f2 = u(1) * sin(state(3)) + d;
  f3 = u(2) + d;

  dx = [f1; f2; f3];
  
end
