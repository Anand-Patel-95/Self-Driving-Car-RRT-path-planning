function A = geodesic(qr, qn, val, eps)
   qnew = [0 0];
   
   % Steer towards qn with max step size of epsilon
   if val >= eps
       qnew(1) = qn(1) + ((qr(1)-qn(1))*eps)/dist(qr,qn);
       qnew(2) = qn(2) + ((qr(2)-qn(2))*eps)/dist(qr,qn);
   else % if qn is less than eps away, set as qnew
       qnew(1) = qr(1);
       qnew(2) = qr(2);
   end   
   A = [qnew(1), qnew(2)];
end