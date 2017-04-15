function dx = system_ct_1(t, e, u, T)

  % Desired placements (destinations)
  global des_1;

  % State vector of agent 1:
  % [x,y]
  state = e' + des_1;

  x = state(1);
  y = state(2);

  f1 = -x + u(1);
  f2 = -y + u(2);

  dx = [f1; f2];

end
