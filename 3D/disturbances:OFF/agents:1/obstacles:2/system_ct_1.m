function dx = system_ct_1(t, e, u, T)

  % Desired placements (destinations)
  global des_1;

  % State vector of agent 1:
  % [x,y]
  state = e' + des_1;

  f1 = u(1) * cos(state(3));
  f2 = u(1) * sin(state(3));
  f3 = u(2);

  dx = [f1; f2; f3];

end
