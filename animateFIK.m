function animateFIK(rigTree, theta, targets, solInfo)
	figure
	[n_targets, ~] = size(targets);
	show(rigTree,theta(1,:)');				%?
	view(2)								%?
	ax = gca;							%?
	ax.Projection = 'orthographic';		%?
	hold on
	plot(targets(:,1),targets(:,2),'k')
	axis([-0.1 0.7 -0.3 0.5])
	framesPerSecond = 5;
	r = rateControl(framesPerSecond);
	n_step = 5;
	i_pic = 1;
	for i = 1:n_targets
		show(rigTree,theta(i,:)','PreservePlot',false);
		drawnow
		s_info = solInfo(i);
		str_solinfo = sprintf('#it = %d, err_p = %6.4f err_r = %6.4f', s_info.Iterations, s_info.PoseErrorNorm, s_info.OriErrorNorm);
		title(str_solinfo);
		if (1 == (mod(i, n_step))) ...
		  && (i_pic < 7)
			filename = sprintf('fik_snap_%d.fig', i_pic);
			savefig(filename);
			i_pic = i_pic + 1;
		end
		% waitfor(r);
		waitforbuttonpress;
	end
end