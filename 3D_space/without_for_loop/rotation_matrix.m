function y = rotation_matrix(q,par)

  if par =='x'
      y = [1 0 0; 0 cos(q) -sin(q);0 sin(q) cos(q)];
  elseif par == 'y'
      y = [cos(q) 0 sin(q); 0 1 0;-sin(q) 0 cos(q)];
  else
      y = [cos(q) -sin(q) 0; sin(q) cos(q) 0; 0 0 1];
  end

end
