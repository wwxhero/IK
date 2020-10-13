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
	for i = 1:n_targets
	   show(rigTree,theta(i,:)','PreservePlot',false);
	   drawnow
	   s_info = solInfo(i);
	   str_solinfo = sprintf('#it = %d, err = %6.4f', s_info.Iterations, s_info.PoseErrorNorm);
	   title(str_solinfo);
	   waitfor(r);
	end
end