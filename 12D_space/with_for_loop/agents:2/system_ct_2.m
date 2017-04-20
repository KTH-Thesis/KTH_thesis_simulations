function dx_2 = system_ct_2(t, e, u, T)

  % Desired placements (destinations)
  global des_2;
  global I_x_2;
  global I_y_2;
  global I_z_2;
  global m_2;

  % State vector of agent 1:
  % [x,y,z,  phi,theta,psi,  x_dot,y_dot,z_dot,  omega_x,omega_y,omega_z]
  %  1,2,3,   4 ,  5,   6,     7,    8,    9,      10,     11,     12
  state_2 = e' + des_2;

  x_2 = state_2(1:6)';
  v_2 = state_2(7:12)';



  %% First state space equation
  % Jacobian J_q in the text
  J_q_2 = [1, 0, sin(state_2(5));
           0, cos(state_2(4)), -cos(state_2(5))*sin(state_2(4));
           0, sin(state_2(4)), cos(state_2(5))*cos(state_2(4))];

  % Jacobian J_2
  J_2 = [eye(3), zeros(3);
         zeros(3), J_q_2];

  % Next x,y,z,phi,theta,psi
  f1 = inv(J_2)*v_2;


  %% Second state space equation
  R_2 = rotation_matrix(state_2(4), 'x');

  % inertia tensor
  I_2 = R_2 * diag([I_x_2, I_y_2, I_z_2]) * R_2';

  % M_2 matrix of the object
  M_2 = [m_2*eye(3), zeros(3);
         zeros(3), I_2];

  % C matrix of the object
  C_2 = [zeros(3), zeros(3);
         zeros(3), skew_symm([state_2(4), 0, 0]) * I_2];

  % g_2 vector of the object
  g_2 = [0, 0, m_2*9.81, 0, 0, 0]';



  f2 = inv(M_2)*(u(:,1) - C_2*v_2 - g_2);

  dx_2 = [f1; f2];

end
