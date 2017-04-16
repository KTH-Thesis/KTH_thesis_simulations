function dx = system_ct_3(t, e, u, T)

  % Desired placements (destinations)
  global des_3;

  % State vector of agent 3:
  % [x,y]
  state = e' + des_3;

  x = state(1);
  y = state(2);

  f1 = -x + u(1);
  f2 = -y + u(2);

  dx = [f1; f2];

end
