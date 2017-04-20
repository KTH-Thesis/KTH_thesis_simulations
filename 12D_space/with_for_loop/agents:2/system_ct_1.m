function dx = system_ct_1(t, e, u, T)

  % Desired placements (destinations)
  global des_1;
  global I_x_1;
  global I_y_1;
  global I_z_1;
  global m_1;

  % State vector of agent 1:
  % [x,y,z,  phi,theta,psi,  x_dot,y_dot,z_dot,  omega_x,omega_y,omega_z]
  %  1,2,3,   4 ,  5,   6,     7,    8,    9,      10,     11,     12
  state_1 = e' + des_1;

  x_1 = state_1(1:6)';
  v_1 = state_1(7:12)';



  %% First state space equation
  % Jacobian J_q in the text
  J_q_1 = [1, 0, sin(state_1(5));
           0, cos(state_1(4)), -cos(state_1(5))*sin(state_1(4));
           0, sin(state_1(4)), cos(state_1(5))*cos(state_1(4))];

  % Jacobian J_1
  J_1 = [eye(3), zeros(3);
         zeros(3), J_q_1];

  % Next x,y,z,phi,theta,psi
  f1 = inv(J_1)*v_1;


  %% Second state space equation
  R_1 = rotation_matrix(state_1(4), 'x');

  % inertia tensor
  I_1 = R_1 * diag([I_x_1, I_y_1, I_z_1]) * R_1';

  % M_1 matrix of the object
  M_1 = [m_1*eye(3), zeros(3);
         zeros(3), I_1];

  % C matrix of the object
  C_1 = [zeros(3), zeros(3);
         zeros(3), skew_symm([state_1(4), 0, 0]) * I_1];

  % g_1 vector of the object
  g_1 = [0, 0, m_1*9.81, 0, 0, 0]';



  f2 = inv(M_1)*(u(:,1) - C_1*v_1 - g_1);

  dx = [f1; f2];

end
