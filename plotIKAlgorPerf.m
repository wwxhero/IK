function plotIKAlgorPerf(Theta_2, Err_1, strLengend_1 ...
						, Theta_2_prime, Err_1_prime, strLengend_2 ...
						, c_theta_2)
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

	min_theta_1 = min(Theta_2(:, 1));
	max_theta_1 = max(Theta_2(:, 1));
	step_theta_1 = (max_theta_1 - min_theta_1)/min(n_iter, n_iter_prime);
	Theta_1 = min_theta_1:step_theta_1:max_theta_1;
	Theta_2_limit = Theta_1;
	if nargin < 7
		Theta_2_limit(1:end) = nan;
	else
		Theta_2_limit(1:end) = c_theta_2;
	end

	figure
	n_row_plot = 3;
	subplot(n_row_plot, 1, 1);

	plot(Theta_2(1:n_iter, 1)', Theta_2(1:n_iter, 2)', 'r*' ...
		, Theta_2_prime(1:n_iter_prime, 1)', Theta_2_prime(1:n_iter_prime, 2)', 'bo' ...
		, Theta_1, Theta_2_limit, 'g-');
	title('Iterative Solutions');
	xlabel('\theta_1');
	ylabel('\theta_2');
	legend(strLengend_1 ...
		 , strLengend_2);


	subplot(n_row_plot, 1, 2);
	plot(Idx_iter(2:n_iter), abs_delta(2:n_iter), 'r' ...
		, Idx_iter_prime(2:n_iter_prime), abs_delta_prime(2:n_iter_prime), 'b');
	title('Step Distance');
	xlabel('# of interation');
	ylabel('distance');
	legend(strLengend_1 ...
		 , strLengend_2);

	subplot(n_row_plot, 1, 3);
	plot(Idx_iter, Err_1(1:n_iter), 'r' ...
		, Idx_iter_prime, Err_1_prime(1:n_iter_prime), 'b');
	title('Iteration vs Error');
	xlabel('# of interation');
	ylabel('Error');
	legend(strLengend_1 ...
		 , strLengend_2);

end