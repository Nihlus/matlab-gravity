1function gravity_proto()
	posm = [[7, 3]; [8, 3]];
	disp(posm);
	disp(mean(posm, 2));

	body = [1, 1, 2, [0, 0, 1]];
	disp(body);

	clf('reset');
	graphAxes = axes('PlotBoxAspectRatio', [1, 1, 1]);
	axis([0, 10, 0, 10]);
	% Generate a random RGB triplet
	randColour = rand(1, 3);
	disp(randColour);

	% Display a circle using that triplet
	hold(graphAxes)
	viscircles(graphAxes, [1, 0; 4, 0], [1, 2], 'Color', randColour);

	disp(coordinatesAtAngle(pi/6, 8, 2, 5));
	
	printSecondsFormatted(7832);
end

function printSecondsFormatted(seconds)
	if (seconds < 0)
		return
	end

	ONE_MINUTE = 60;
	
	danglingSeconds = rem(seconds, ONE_MINUTE);
	totalMinutes = (seconds - danglingSeconds) / 60;
	
	danglingMinutes = rem(totalMinutes, 60);
	totalHours = (totalMinutes - danglingMinutes) / 60;
	
	fprintf('Hours: %d, Minutes: %d, Seconds: %d\n', totalHours, danglingMinutes, danglingSeconds);
end

function coordinates = coordinatesAtAngle(angle, radius, X, Y)
	XCoordinate = (radius * cos(angle)) + X;
	YCoordinate = (radius * sin(angle)) + Y;
	
	coordinates = [XCoordinate, YCoordinate];
end

function circle(x, y, r, spec, colour)
	angle = 0 : 0.01 : 2*pi;
	xPlot = r * cos(angle);
	yPlot = r * sin(angle);
	
	plot(x + xPlot, y + yPlot, spec, 'Color', colour);
end