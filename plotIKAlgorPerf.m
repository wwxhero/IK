function plotIKAlgorPerf(Theta_2, Err_1, lambda ...
						, Theta_2_prime, Err_1_prime, lambda_prime)
	[n_iter, ~] = size(Theta_2);
	[n_iter_prime, ~] = size(Theta_2_prime);
	abs_delta = zeros(1, n_iter);
	abs_delta_prime = zeros(1, n_iter_prime);
	Idx_iter = 0:1:n_iter-1;
	for i_iter = 2 : n_iter
		delta_theta = Theta_2(i_iter, :) - Theta_2(i_iter - 1, :);
		abs_delta(i_iter) = norm(delta_theta);
	end
	abs_delta(1) = nan;

	Idx_iter_prime = 0:1:n_iter_prime-1;
	for i_iter_prime = 2 : n_iter_prime
		delta_theta_prime = Theta_2_prime(i_iter_prime, :) - Theta_2_prime(i_iter_prime - 1, :);
		abs_delta_prime(i_iter_prime) = norm(delta_theta_prime);
	end
	abs_delta_prime(1) = nan;

	figure
	n_row_plot = 3;
	subplot(n_row_plot, 1, 1);

	plot(Theta_2(1:n_iter, 1)', Theta_2(1:n_iter, 2)', 'r*' ...
		, Theta_2_prime(1:n_iter_prime, 1)', Theta_2_prime(1:n_iter_prime, 2)', 'bo');
	title('Iterative Solutions');
	xlabel('\theta_1');
	ylabel('\theta_2');
	legend(sprintf('\\lambda=%4.3f', lambda) ...
		 , sprintf('\\lambda=%4.3f', lambda_prime));


	subplot(n_row_plot, 1, 2);
	plot(Idx_iter(2:n_iter), abs_delta(2:n_iter), 'r' ...
		, Idx_iter_prime(2:n_iter_prime), abs_delta_prime(2:n_iter_prime), 'b');
	title('Step Distance');
	xlabel('# of interation');
	ylabel('distance');
	legend(sprintf('\\lambda=%4.3f', lambda) ...
		 , sprintf('\\lambda=%4.3f', lambda_prime));

	subplot(n_row_plot, 1, 3);
	plot(Idx_iter, Err_1(1:n_iter), 'r' ...
		, Idx_iter_prime, Err_1_prime(1:n_iter_prime), 'b');
	title('Iteration vs Error');
	xlabel('# of interation');
	ylabel('Error');
	legend(sprintf('\\lambda=%4.3f', lambda) ...
		 , sprintf('\\lambda=%4.3f', lambda_prime));
end