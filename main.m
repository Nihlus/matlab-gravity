function main()
	% Setup
	bodyCount = 5;
	minMaxX = [0 100];
	minMaxY = [0 100];
	minMaxR = [5 15];
	
	lastFrameTime = 5;
	
	timeStep = 0.1;

	% PLACEHOLDER: Generate a set of random bodies
	gravitationalBodies = GravitationalBody.empty(bodyCount, 0);
	for i = 1 : bodyCount
		gravitationalBodies(i) = GravitationalBody.CreateRandomBody(minMaxX, minMaxY, minMaxR);
	end
	
	areAllBodiesMerged = false;
	while (~areAllBodiesMerged)
		% Check if all bodies have merged
		numAliveBodies = 0;
		for i = 1 : size(gravitationalBodies, 1)
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
		lastFrameTime = timeit(RunFrame(lastFrameTime, timeStep)) / 1000;
		
	end
end

function RunFrame(deltaTime, seconds)
	% for all bodies
	for i = 1 : size(gravitationalBodies, 1)
		gravitationalBody = gravitationalBodies(i);

		% Check for collisions
		for j = 1 : size(gravitationalBodies, 1)
			otherGravitationalBody = gravitationalBodies(j);

			% Skip itself
			if (j == i)
				continue;
			end

			% If this body is not alive, skip it
			if (~otherGravitationalBody.IsAlive)
				continue;
			end

			isColliding = IsCollidingWith(otherGravitationalBody);
			if (isColliding)
				gravitationalBody.AbsorbBody(otherGravitationalBody);
			end
		end

		% Force calculations
		for j = 1 : size(gravitationalBodies, 1)
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
	for j = 1 : size(gravitationalBodies, 1)
		gravitationalBody = gravitationalBodies(i);
		gravitationalBody.SimulateForces(deltaTime, timeStep);
	end
end