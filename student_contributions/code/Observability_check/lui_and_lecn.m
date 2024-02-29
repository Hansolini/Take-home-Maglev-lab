function [lui, lecn] = lui_and_lecn(obs_gram, states_to_include)

%% Remove unwanted states from equilibrium:
obs_gram_red = obs_gram(states_to_include, states_to_include);


%%  Check the observability:
%obs_gram_eig = eig(obs_gram);
obs_gram_red_eig = eig(obs_gram_red);

% Local singular values
lsv = sqrt(obs_gram_red_eig);
% Local unobservability index:
lui = 1/min(lsv);
% Local estimation condition number:
lecn = sqrt(max(obs_gram_red_eig)/min(obs_gram_red_eig));


end