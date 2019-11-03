function nc = noCollision3(n2, n1, o)
    A = [n1(1) n1(2)];  %start
    B = [n2(1) n2(2)];  %end
    R_robot = 1;
    
    % no clipping
    notclipping = noCollision2(n2, n1, o);

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
        
        % second point must be outside of obstacle
        if (r < dist(C, B))
            outside = 1; 
        else
            outside = 0;
        end
        
        % circles must not intersect
        [xout,yout] = circcirc(bx,by,R_robot,cx,cy,r);
        cond1 = isnan(xout);
%         cond2 = isnan(yout);
        if (cond1)
            dontIntersect = 1;
        else
            dontIntersect = 0;
        end
        
        % needs to be outside obstacle and not intersecting and not
        % clipping
        if (outside && dontIntersect && notclipping)
            doIntersect = 0;
        else
            doIntersect = 1;
%           collision_obs = j
            break
        end

    end
    
    if doIntersect == 1
        nc = 0;
    else
        nc = 1;
    end
end