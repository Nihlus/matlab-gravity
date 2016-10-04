function main()
	bodyCount = 0;
	minMaxX = [0 0];
	minMaxY = [0 0];
	minMaxR = [0 0];
	
	timeStep = 0;

	% Setup - simulation parameters
	isDebug = true;
	if (isDebug)
		bodyCount = 10;
		minMaxX = [0, 100];
		minMaxY = [0, 100];
		minMaxR = [1, 2];
		
		timeStep = 0.001;
		
		rng('shuffle', 'simdTwister')
	else
		bodyCount = input('Enter the desired number of simulated bodies: \n$: ');
		minMaxX = input('Enter the minimum and maxiumum X values as a vector in the format [XMin, XMax]: \n$: ');
		minMaxY = input('Enter the minimum and maxiumum Y values as a vector in the format [YMin, YMax]: \n$: ');
		minMaxR = input('Enter the minimum and maximum initial radii values of the bodies as a vector in the format [RMin, RMax]: \n$: ');
		
		timeStep = input('Enter the desired time step of the simulation: \n$: ');
		
		rng(input('Enter a seed for the random number generator: \n'), 'simdTwister')
		
		input('Parameters loaded. Press enter to begin.');
	end
	
	% Setup - time delta
	lastFrameTime = 5;
	
	% Setup - rendering
	clf('reset');
	graphAxes = axes('PlotBoxAspectRatio', [1, 1, 1]);
	axis(graphAxes, [minMaxX, minMaxY]);
	grid(graphAxes, 'on');

	% Setup - body generation
	gravitationalBodies = GravitationalBody.empty(bodyCount, 0);
	for i = 1 : bodyCount
		gravitationalBodies(i) = GravitationalBody.CreateRandomBody(minMaxX, minMaxY, minMaxR);
	end
	
	% Run simulation
	
	while (size(gravitationalBodies,  2) > 1)
		% Run one frame of the simulation and store the time taken to 
		% perform that simulation.
		[lastFrameTime, gravitationalBodies] = RunFrame(gravitationalBodies, lastFrameTime, timeStep, graphAxes);
		lastFrameTime = lastFrameTime * 1000;
	end
end

function [timeTaken, remainingBodies] = RunFrame(gravitationalBodies, deltaTime, seconds, graphAxes)
	tic
	
	remainingBodies = gravitationalBodies;
	
	% Dump the dead bodies from last frame
	deadIndices = [];
	for i = 1 : size(remainingBodies, 2)
		gravitationalBody = remainingBodies(i);
		if (~gravitationalBody.IsAlive)
			deadIndices = [deadIndices, i];
		end
	end
	
	if (size(deadIndices) > 0)
		% Delete the bodies from the array
		remainingBodies(deadIndices) = [];
	end
	
	% for all bodies
	for i = 1 : size(remainingBodies, 2)
		gravitationalBody = remainingBodies(i);
		
		if (~gravitationalBody.IsAlive)
			continue;
		end

		% Zero out the force from last frame
		gravitationalBody.XYChange = [0, 0];
		gravitationalBody.Acceleration = 0;
		
		% Check for collisions
		for j = 1 : size(remainingBodies, 2)
			otherGravitationalBody = remainingBodies(j);

			% Skip itself
			if (j == i)
				continue;
			end

			if (~otherGravitationalBody.IsAlive)
				continue;
			end
			
			isColliding = gravitationalBody.IsCollidingWith(otherGravitationalBody);
			if (isColliding)
				gravitationalBody.AbsorbBody(otherGravitationalBody);
			else
				% Compute the forces
				gravitationalBody.ComputeForces(otherGravitationalBody);
			end
		end
	end

	% Simulate all forces
	for j = 1 : size(remainingBodies, 2)
		gravitationalBody = remainingBodies(j);

		if (~gravitationalBody.IsAlive)
			continue
		end	
		
		gravitationalBody.SimulateForces(deltaTime, seconds);
	end
	
	% Finally, draw all bodies
	hold(graphAxes, 'off') % Clear the previous frame (swap the buffers, essentially)
	for i = 1 : size(remainingBodies, 2)
		gravitationalBody = remainingBodies(i);
		
		if (~gravitationalBody.IsAlive)
			continue
		end

		gravitationalBody.Draw(graphAxes);
		
		if (i == 1)
			hold(graphAxes, 'on');
		end
	end
	
	
	drawnow;
	pause(0.01);
	
	%remainingBodies = gravitationalBodies;
	timeTaken = toc;
end