function [ out ] = scalingfactor( successes, failures, sc, fc, oldVal )
%scalingfactor Used in the Guaranteed Conv. PSO version of the algorithm
%   In GCPSO, a different velocity update equation is used for the particle
%   at the global best. The equation is the following:
%   v(t+1) = inertia_weight*v(t)-position(t)+pbest+scalingfactor*rand(-1,1)

% calling function should recall, whenever gbest is improved ++successes
% and failures = 0. This also occurs for vice-versa min(successes,failures)
% = 0 ALWAYS.

if successes > sc
    out = 2*oldVal;
elseif failures > fc
    out = 0.5*oldVal;
else
    out = oldVal;
end

end

