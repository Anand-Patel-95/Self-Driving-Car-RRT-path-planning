% RRT algorithm in 2D with disc obstacle avoidance.
% Anand Patel
% 
% nodes:    contains its coordinates, cost to reach, and its parent.
%           
% 
% How it works: 
% 1. Pick a random node q_rand.
% 2. Find the closest node q_near from nodes list to branch out from
% towards q_rand.
% 3. Move from q_near towards q_rand: interpolate if node is too far away,
% reach q_new. Check for collisions.
% 4. Update cost of reaching q_new from q_near, Cmin. q_near
% acts as the parent node of q_new.
% 5. Add q_new to node list.
% 6. Continue until maximum number of samples is reached or goal region is
% entered.

clearvars
close all

% make S = [0 100] X [0 100]
x_max = 100;
y_max = 100;

% read in obstacles
obstacle_array = csvread('H3_obstacles.txt');
% turn array into struct
for j=1:1:23
obstacle(j).coord = [obstacle_array(j,1) obstacle_array(j,2)];
obstacle(j).rad = obstacle_array(j,3);
end
nodes_id = 1;
EPS = 20;               % epsilon distance ASSIGNED
numNodes = 100000;        % max number of samples taken
del_t = 10;
delta = .5;

q_start.coord = [40 40];      % start node's (x,y) coordinate ASSIGNED
q_start.cost = 0;           % cost to reach start node set to 0
q_start.parent = 0;         % parent of start node set to 0
q_start.id = nodes_id;
q_start.time = 0;           % start node begins at t=0
q_start.theta = pi/4;         % start node theta ASSIGNED
q_start.v = 0;              % start node trans vel = 0
q_start.w = 0;              % start node steering vel = 0
q_start.a = 0;
q_start.gamma = 0;

q_goal.coord = [0 0];   % goal region center coordinate (x,y) ASSIGNED
q_goal.cost = 0;
goal_region_radius = 20;    % goal region radius ASSIGNED

nodes(1) = q_start;         % V <-- Start
figure(1)                   % plot figure
axis([0 x_max 0 y_max])

% plot goal region
pos = [(q_goal.coord(1)-goal_region_radius) (q_goal.coord(2)-goal_region_radius) 2*goal_region_radius 2*goal_region_radius]; 
h = rectangle('Position',pos,'Curvature',[1 1],'FaceColor','m');
hold on

% plot start point
plot(q_start.coord(1), q_start.coord(2), 'go');

% plot obstacles
for j=1:1:length(obstacle)
    obs = obstacle(j);
    circle2(obs.coord(1),obs.coord(2),obs.rad);
end


for i = 1:1:numNodes
    % Break if goal region is already reached
    for j = 1:1:length(nodes)
        % calculate distance to goal region's center
        distance2goal = dist(nodes(j).coord, q_goal.coord);
        if distance2goal < goal_region_radius
            %check if distance2goal falls within region radius
            flag = 1;   % exit flag
            break
        end
    end
    
    % Check if goal region reached
    if (flag==1)
        break
    else
        % sample random configuration and plot coord as 'x'
        q_rand.coord = [floor(rand(1)*x_max) floor(rand(1)*y_max)];
        plot(q_rand.coord(1), q_rand.coord(2), 'x', 'Color',  [0 0.4470 0.730])
        
        q_rand.theta = 0 + rand(1)*(2*pi - 0);
        q_rand.v = -5 + rand(1)*(5 - -5);
        q_rand.w = (-pi/2) + rand(1)*(pi);

        % Pick the closest (euclidian) node from existing list to branch out from
        ndist = [];
        for j = 1:1:length(nodes)   %for all nodes...
            n = nodes(j);
            tmp = dist(n.coord, q_rand.coord);
            ndist = [ndist tmp];        % list of distances to sample point
        end
        [val, idx] = min(ndist);    % pick the smallest distance to sample pt
        q_near = nodes(idx);    % set nearest node to the node corresponding to 
                                % shortest distance

        % place new node epsilon distance along geodesic towards sample point, from q_near                        
        q_new.coord = geodesic(q_rand.coord, q_near.coord, val, EPS);
        % assign new node same theta, v, w as random sample
        q_new.theta = q_rand.theta;
        q_new.v = q_rand.v;
        q_new.w = q_rand.w;
        
        % generate required input to go from q_near to q_new's coordinates
        [a, gamma, vf_new, wf_new, thetaf_new] = generateinput(del_t, q_new, q_near);
        
        % check if required inputs and new configurations are valid
        a_check = abs(a)<= 2;
        gamma_check = abs(gamma) <= (pi/2);
        vf_check = abs(vf_new) <= 5;
        wf_check = abs(wf_new) <= (pi/2);
        
        if(a_check && gamma_check && vf_check && wf_check)  %if valid
            % assign new node the new configuration and new inputs
            q_new.theta = thetaf_new;
            q_new.v = vf_new;
            q_new.w = wf_new;
            if noCollision3(q_new.coord, q_near.coord, obstacle)    % if the end is good, then check along path
                if noCollision5(q_new, q_near, obstacle, del_t, a, gamma, delta)  % if no collision
                    line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], 'Color', 'k', 'LineWidth', 2);
                    drawnow
                    hold on
                    nodes_id = nodes_id + 1;
                    q_new.cost = dist(q_new.coord, q_near.coord) + q_near.cost;         
                    q_new.parent = idx;     % sets q_new parent as q_near
                    q_new.id = nodes_id;    % adds new node
                    q_new.time = q_near.time + del_t;

                    % update inputs required to get here t_i from t_(i-1)
                    q_new.a = a;            % inputs to move from q_near to q_new
                    q_new.gamma = gamma;

                    % Append to nodes
                    nodes = [nodes q_new];
                end
            end   
        end
    end
end

D = [];
for j = 1:1:length(nodes)
    tmpdist = dist(nodes(j).coord, q_goal.coord);
    D = [D tmpdist];    % list of all the distance2goals of every node
end

% Search backwards from goal to start to find the optimal least cost path
[val, idx] = min(D);        % finds closest node to goal
q_final = nodes(idx);       % assigns closest node to goal as final node
q_end = q_final;            % assigns the final node as end node

% nodes = [nodes q_goal]; % add goal node to list of nodes, in case of no
% goal region, only goal node.

fileID = fopen('found_path.txt','w');
% formatSpec = 'X is %4.2f meters or %8.3f mm\n';
fprintf(fileID,'%10s %10s %10s %10s %10s %10s %10s %10s\n','time', 'X coord', 'Y coord', 'Theta', 'V', 'W', 'a', 'gamma');

while q_end.parent ~= 0 % until the start node
    start = q_end.parent;   % start at the end node
    % draw line from end node to end node's parent
    line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], 'Color', 'r', 'LineWidth', 2);
    hold on
    % save end node to output file here
    fprintf(fileID,'%10.2f %10.2f %10.2f %10.2f %10.2f %10.2f %10.2f %10.2f\n', q_end.time, q_end.coord(1), q_end.coord(2), q_end.theta, q_end.v, q_end.w, q_end.a, q_end.gamma);
    % if nodes(start).parent = 0 (i.e. last iteration of loop), also save
    % start node to file after the end node
    if (nodes(start).parent == 0)
        fprintf(fileID,'%10.2f %10.2f %10.2f %10.2f %10.2f %10.2f %10.2f %10.2f\n', nodes(start).time, nodes(start).coord(1), nodes(start).coord(2), nodes(start).theta, nodes(start).v, nodes(start).w, nodes(start).a, nodes(start).gamma);
        fclose(fileID);
    end
    q_end = nodes(start);   % set end node to previous end node's parent
end



