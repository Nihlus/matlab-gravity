function main()
	% Setup
	%bodyCount = input('Enter the desired number of simulated bodies: \n');
	%minMaxX = input('Enter the minimum and maxiumum X values as a vector in the format [XMin, XMax]: \n');
	%minMaxY = input('Enter the minimum and maxiumum Y values as a vector in the format [YMin, YMax]: \n');
	%minMaxR = input('Enter the minimum and maximum initial radii values of the bodies as a vector in the format [RMin, RMax]: \n');;
	
	bodyCount = 100;
	minMaxX = [0, 1000];
	minMaxY = [0, 1000];
	minMaxR = [2, 4];
	
	lastFrameTime = 5;
	
	%timeStep = input('Enter the desired time step of the simulation: \n');
	timeStep = 0.1;
	
	% Graph setup
	clf('reset');
	graphAxes = axes('PlotBoxAspectRatio', [1, 1, 1]);
	axis(graphAxes, [minMaxX, minMaxY]);
	grid(graphAxes, 'on');

	% Random setup
	rng('shuffle', 'simdTwister')
	% rng(input('Enter a seed for the random number generator: \n'), 'simdTwister')
	
	% PLACEHOLDER: Generate a set of random bodies
	gravitationalBodies = GravitationalBody.empty(bodyCount, 0);
	for i = 1 : bodyCount
		gravitationalBodies(i) = GravitationalBody.CreateRandomBody(minMaxX, minMaxY, minMaxR);
	end
	
	areAllBodiesMerged = false;
	while (~areAllBodiesMerged)
		% Check if all bodies have merged
		numAliveBodies = 0;
		for i = 1 : size(gravitationalBodies, 2)
			gravitationalBody = gravitationalBodies(i);

			if (gravitationalBody.IsAlive)
				numAliveBodies = numAliveBodies + 1;
			end
		end

		if (numAliveBodies <= 1)
			areAllBodiesMerged = true;
		end
		
		% Run one frame of the simulation and store the time taken to 
		% perform that simulation.
		lastFrameTime = RunFrame(gravitationalBodies, lastFrameTime, timeStep, graphAxes) * 1000;
	end
end

function timeTaken = RunFrame(gravitationalBodies, deltaTime, seconds, graphAxes)
	tic
	% for all bodies
	for i = 1 : size(gravitationalBodies, 2)
		gravitationalBody = gravitationalBodies(i);

		% Check for collisions
		for j = 1 : size(gravitationalBodies, 2)
			otherGravitationalBody = gravitationalBodies(j);

			% Skip itself
			if (j == i)
				continue;
			end

			% If this body is not alive, skip it
			if (~otherGravitationalBody.IsAlive)
				continue;
			end

			isColliding = gravitationalBody.IsCollidingWith(otherGravitationalBody);
			if (isColliding)
				gravitationalBody.AbsorbBody(otherGravitationalBody);
			end
		end

		% Force calculations
		% Zero out the force from last frame
		gravitationalBody.XYDirection = [0, 0];
		gravitationalBody.Acceleration = 0;
		
		% Recompute the affecting forces
		for j = 1 : size(gravitationalBodies, 2)
			otherGravitationalBody = gravitationalBodies(j);

			% Skip itself
			if (j == i)
				continue;
			end

			% If this body is not alive, skip it
			if (~otherGravitationalBody.IsAlive)
				continue;
			end

			% Compute the forces
			gravitationalBody.ComputeForces(otherGravitationalBody);
		end
	end

	% Simulate all forces
	for j = 1 : size(gravitationalBodies, 2)
		gravitationalBody = gravitationalBodies(i);
		gravitationalBody.SimulateForces(deltaTime, seconds);
	end
	
	% Finally, draw all bodies
	hold off % Clear the previous frame (swap the buffers, essentially)
	for i = 1 : size(gravitationalBodies, 2)
		gravitationalBody = gravitationalBodies(i);
		gravitationalBody.Draw(graphAxes);
		
		if (i == 1)
			hold on
		end
	end
	
	drawnow limitrate;
	pause(0.01);
	
	timeTaken = toc;
end