function dx = system_ct_3_real(t, e, u, T)

  % Desired placements (destinations)
  global des_3;

  % The disturbance
  global disturbance;

  % State vector of agent 1:
  % [x,y]
  state = e' + des_3;

  d = disturbance * cos(2*t);

  f1 = u(1) * cos(state(3)) + d;
  f2 = u(1) * sin(state(3)) + d;
  f3 = u(2) + d;

  dx = [f1; f2; f3];

end
