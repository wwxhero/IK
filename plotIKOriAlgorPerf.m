function plotIKOriAlgorPerf(Theta, Err_r, Err_p, lambda_sqr ...
						, Theta_prime, Err_r_prime, Err_p_prime, lambda_sqr_prime)
	[n_iter, ~] = size(Theta);
	[n_iter_prime, ~] = size(Theta_prime);
	abs_delta = zeros(1, n_iter);
	abs_delta_prime = zeros(1, n_iter_prime);
	Idx_iter = 0:1:n_iter-1;
	for i_iter = 2 : n_iter
		delta_theta = Theta(i_iter, :) - Theta(i_iter - 1, :);
		abs_delta(i_iter) = norm(delta_theta);
	end
	abs_delta(1) = nan;

	Idx_iter_prime = 0:1:n_iter_prime-1;
	for i_iter_prime = 2 : n_iter_prime
		delta_theta_prime = Theta_prime(i_iter_prime, :) - Theta_prime(i_iter_prime - 1, :);
		abs_delta_prime(i_iter_prime) = norm(delta_theta_prime);
	end
	abs_delta_prime(1) = nan;

	figure
	n_row_plot = 3;

	subplot(n_row_plot, 1, 1);
	plot(Idx_iter(2:n_iter), abs_delta(2:n_iter), 'r*' ...
		, Idx_iter_prime(2:n_iter_prime), abs_delta_prime(2:n_iter_prime), 'bo');
	title('Step Distance');
	xlabel('# of interation');
	ylabel('distance');
	legend(sprintf('\\lambda^2=%4.3f', lambda_sqr) ...
		 , sprintf('\\lambda^2=%4.3f', lambda_sqr_prime));

	subplot(n_row_plot, 1, 2);
	plot(Idx_iter, Err_p(1:n_iter), 'r' ...
		, Idx_iter_prime, Err_p_prime(1:n_iter_prime), 'b');
	title('Iteration vs Error_p');
	xlabel('# of interation');
	ylabel('Error_p (in meter)');
	legend(sprintf('\\lambda^2=%4.3f', lambda_sqr) ...
		 , sprintf('\\lambda^2=%4.3f', lambda_sqr_prime));

	subplot(n_row_plot, 1, 3);
	plot(Idx_iter, Err_r(1:n_iter), 'r' ...
		, Idx_iter_prime, Err_r_prime(1:n_iter_prime), 'b');
	title('Iteration vs Error_r');
	xlabel('# of interation');
	ylabel('Error_r (in degree)');
	legend(sprintf('\\lambda^2=%4.3f', lambda_sqr) ...
		 , sprintf('\\lambda^2=%4.3f', lambda_sqr_prime));
end