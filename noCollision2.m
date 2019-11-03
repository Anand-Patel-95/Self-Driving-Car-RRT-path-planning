function nc = noCollision2(n2, n1, o)
    A = [n1(1) n1(2)];
    B = [n2(1) n2(2)];


    for j=1:1:length(o)
        ax = A(1);
        ay = A(2);
        bx = B(1);
        by = B(2);
        obs = o(j);           % temporarily store obstacle struct
        C  = obs.coord;    % Center of circle
        cx = C(1);
        cy = C(2);
        r  = obs.rad;      % Radius of circle
        
        ax = ax - cx;
        ay = ay - cy;
        bx = bx-cx;
        by = by-cy;
        c = ax^2 + ay^2 - r^2;
        b = 2*(ax*(bx - ax) + ay*(by - ay));
        a = (bx - ax)^2 + (by - ay)^2;
        disc = b^2 - 4*a*c;
        sqrtdisc = sqrt(disc);
        t1 = (-b + sqrtdisc)/(2*a);
        t2 = (-b - sqrtdisc)/(2*a);
        if(disc <= 0) 
            doIntersect = 0;
        elseif((0 < t1 && t1 < 1) || (0 < t2 && t2 < 1)) 
            doIntersect = 1;
%             collision_obs = j
            break
        else
            doIntersect = 0;
        end
    end
    
    if doIntersect == 1
        nc = 0;
    else
        nc = 1;
    end
end