function dx = system_ct_2(t, e, u, T)

  % Desired placements (destinations)
  global des_2;

  % State vector of agent 2:
  % [x,y]
  state = e' + des_2;

  f1 = u(1) * cos(state(3));
  f2 = u(1) * sin(state(3));
  f3 = u(2);

  dx = [f1; f2; f3];

end
