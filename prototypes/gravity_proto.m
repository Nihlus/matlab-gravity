function gravity_proto()
	posm = [[7, 3]; [8, 3]];
	disp(posm);
	disp(mean(posm, 2));

	body = [1, 1, 2, [0, 0, 1]];
	disp(body);

	clf('reset');
	graphAxes = axes('PlotBoxAspectRatio', [1, 1, 1]);
	axis([0, 100, 0, 100]);
	% Generate a random RGB triplet
	randColour = rand(1, 3);

	% Display a circle using that triplet
	viscircles(graphAxes, [40, 36; 80, 17], [6, 2], 'Color', randColour);
	disp(randColour);
end

function circle(x, y, r, spec, colour)
	angle = 0 : 0.01 : 2*pi;
	xPlot = r * cos(angle);
	yPlot = r * sin(angle);
	
	plot(x + xPlot, y + yPlot, spec, 'Color', colour);
end