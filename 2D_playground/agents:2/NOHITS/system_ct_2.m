function dx = system_ct_2(t, e, u, T)

  % Desired placements (destinations)
  global des_2;

  % State vector of agent 2:
  % [x,y]
  state = e' + des_2;

  x = state(1);
  y = state(2);

  f1 = -x + u(1);
  f2 = -y + u(2);

  dx = [f1; f2];

end
