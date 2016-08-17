function [ x_best, y_best, best_Z ] = pso ...
( func, pop, n_radius, inertia_weight, max_vel, max_iter, c1, c2, ...
min_x, max_x, min_y, max_y )
% PSO A PSO designed to solve a 2 dimensional optimization problem
%   Algorithm follows the following steps:
%   1. Initialize population
%   2. While iterations < max_iterations
%     a. For every particle
%       i. Find new velocity in each direction
%       ii. Find new position for each particle (update pBest if need be)
%     b. Update nBest

% argument checking
objFunc = func;
plotPSO = 1;
if pop < 0 || isnan(pop)
    pop = 64; %default
end
if n_radius < 0 || isnan(n_radius)
    n_radius = inf;
end
if max_iter < 0 || isnan(max_iter)
    max_iter = 50
end
if inertia_weight < 0 || isnan(inertia_weight)
    inertia_weight = 0.792;
end
if max_vel < 0 || isnan(max_vel)
    max_vel = 1;
end
if c1 < 0 || isnan(c1)
    c1 = 1.4944;
end
if c2 < 0 || isnan(c2)
    c2 = 1.4944;
end

% Initialize PSO
% Init population
swarm = zeros(pop, 2); %building for 2 dims
range_min = [min_x min_y];
range_max = [max_x max_y];

swarm(:,1) = range_min(1) + (range_max(1) - range_min(1)).*rand(pop, 1);
swarm(:,2) = range_min(2) + (range_max(2) - range_min(2)).*rand(pop, 1);
% END Init population
% Init memory storing pbest and nbest (store indexes)
pbest = swarm;
currResults = objFunc(swarm(:,1), swarm(:,2));
[zbest,ind] = min(currResults);
nbest = swarm(ind,:);
nbest_ind = ind;

v = zeros(pop,2); % initial velocity to zero

sf = 1; % init scaling factor (GCPSO)
successes = 0; %GCPSO
failures = 0; %GCPSO
sc = 15; % used while calculating scaling factor GCPSO
fc = 5; % used while calculating scaling factor GCPSO

iter = 1;
while iter <= max_iter
    %velocity update (first update x_vel, then y_vel
    for i=1:2
        for j=1:pop
            if j == nbest_ind % current agent is at the best value so far
                sf = scalingfactor(successes, failures, sc, fc, sf);
                v(j,i) = inertia_weight * v(j,i) - swarm(j,i) + ...
                pbest(j,i) + sf * (-1 + 2 * rand());
            else
                v(j,i) = inertia_weight*v(j,i) + ...
                c1*rand()*(pbest(j,i) - swarm(j,i)) + ...
                c2*rand()*(nbest(i) - swarm(j,i));
            end

            v(j,i) = max(min(v(j,i), max_vel), -1*max_vel);
        end
    end

    %position update
    for i=1:2
        for j=1:pop
            swarm(j,i) = swarm(j,i) + v(j,i);
            if swarm(j,i) > range_max(i)
                swarm(j,i) = range_max(i);
            end
            if swarm(j,i) < range_min(i)
                swarm(j,i) = range_min(i);
            end

            %update pbest
            if i == 2
                currResults(j) = objFunc(swarm(j,1),swarm(j,2));
                if currResults(j) < objFunc(pbest(j,1),pbest(j,2))
                    pbest(j,1) = swarm(j,1);
                    pbest(j,2) = swarm(j,2);
                    if j == nbest_ind
                        successes = successes + 1;
                        failures = 0;
                    end
                elseif j == nbest_ind
                    failures = failures + 1;
                    successes = 0;      
                end
            end
        end
    end

    %Update nBest if a better solution was found
    [zbest,ind] = min(currResults);
    if zbest < objFunc(nbest(1), nbest(2))
        nbest = swarm(ind,:);
        if ind ~= nbest_ind %if a new agent has taken place of gbest, reset GCPSO
            sf = 1;
            successes = 0;
            failures = 0;
            nbest_ind = ind;
        end
    end
    
    iter = iter + 1;

    if plotPSO == 1
        scatter(swarm(:,1),swarm(:,2));
        xlim([range_min(1) range_max(1)]);
        ylim([range_min(2) range_max(2)]);
        %hold on
        pause(0.05);
    end
end

x_best = nbest(1);
y_best = nbest(2);
best_Z = objFunc(nbest(1), nbest(2));
