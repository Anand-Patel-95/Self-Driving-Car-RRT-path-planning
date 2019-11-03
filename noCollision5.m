function nc = noCollision5(n2, n1, o, del_t, a, gamma, delta)
    % takes in start and end nodes, obstacles, time step, inputs, delta
    start_coord = [n1.coord(1) n1.coord(2)];  %start
%     end_coord = [n2.coord(1) n2.coord(2)];  %end
    
    % There is a trajectory from start to end that uses inputs (a, gamma) to
    % get there in time t. While traveling along this trajectory, we want
    % to collision check at every delta distance travelled from the
    % previously checked point.
    
    point1.coord = start_coord;   % beginning at the start node...
    point1.theta = n1.theta;
    point1.v = n1.v;
    point1.w = n1.w;
    
    % initialize the second point
    point2.coord = start_coord;
    point2.theta = 0;
    point2.v = 0;
    point2.w = 0;
    
    time_total = 0;
    syms t
    doIntersect = 0;
    
    while (time_total < del_t)             % while total time is less than 10 seconds
    if (a == 0)
        t_step = delta/(point1.v);
    else
        eqn = abs((point1.v + a*t)*t) == delta;
        sol_t = solve(eqn, t);  % time step to move delta
        maxVal  = max(sol_t);
        t_step = double(maxVal);      % pick positive value
%         t_step = (1/(2*a))*(-(point1.v) + sqrt(4*a*delta + (point1.v)^2))
    end
    
    % calculate new state delta away
    point2.coord(1) = point1.coord(1) + (point1.v + a*t_step)*cos(point1.theta + (point1.w)*t_step + gamma*(t_step^2))*t_step;  % calculate x2
    point2.coord(2) = point1.coord(2) + (point1.v + a*t_step)*sin(point1.theta + (point1.w)*t_step + gamma*(t_step^2))*t_step;  % calculate y2
    point2.w = point1.w + gamma*t_step;
    point2.theta = point1.theta + (point2.w)*t_step;
    point2.v = point1.v + a*t_step;
    
%     time_temp = time_total + t_step
    % collision check these 2 points
    collision_free = noCollision3(point2.coord, point1.coord, o);
%     time_temp = time_total + t_step;        % check in case this move puts robot past end
    
        if (collision_free) 
            % if collision free and not past end point, assign point2 to point1, add in time step
            point1 = point2;
            % increment total time by time step
            time_total = time_total + t_step;
        else
            doIntersect = 1;
            break
        end
    end   
    
    if (doIntersect == 1)
        nc = 0;
    else
        nc = 1;
    end
end