classdef GravitationalBody < handle
	%GravitationalBody A structure representing a gravitational body.
	%   This class contains all data and methods required to
	%	hold and compute relational gravity data.
	
	properties	
		IsAlive = true;
		
		% Positional data as a 2-element vector
		XY = [0, 0]
		
		% Mass
		Radius = 1
		
		% The colour of the mass as an RGB triplet
		RGB = [1, 1, 1]
		
		% Forces
		XYChange = [0, 0]
		Acceleration = 0
	end
	
	methods (Static)
		function body = CreateRandomBody(minMaxX, minMaxY, minMaxR)
			% CREATERANDOMBODY Creates a random gravitational body,
			% constrained to the parameters provided.
			%
			% PARAM minMaxX The starting range of the X position of the body 
			% (as a 2-element vector).
			% PARAM minMaxY The starting range of the Y position of the body
			% (as a 2-element vector).
			% PARAM minMaxR The minimum and maximum radius of the body
			% (as a 2-element vector).
			
			% Check the input variables
			if (numel(minMaxX) ~= 2)
				fprintf('The parameter "minXY" must be a 2-element vector.');
			end
			
			if (numel(minMaxY) ~= 2)
				fprintf('The parameter "maxXY" must be a 2-element vector.');
			end
			
			if (numel(minMaxR) ~= 2)
				fprintf('The parameter "minMaxR" must be a 2-element vector.');
			end
			
			% Initialize a new body
			body = GravitationalBody();
			
			% Create two random X and Y values
			randX = minMaxX(1) + (minMaxX(2) - minMaxX(1)).*rand(1,1);
			randY = minMaxY(1) + (minMaxY(2) - minMaxY(1)).*rand(1,1);
			
			body.XY = [randX, randY];
			
			% Create a random radius
			randR = (minMaxR(2) - minMaxR(1)).*rand(1,1) + minMaxR(1);
			
			body.Radius = randR;
			
			% Create a random colour
			randColour = rand(1, 3);
			
			body.RGB = randColour;
        end
        
        function result = lerp(v0, v1, t)
            result = (1-t)*v0+t*v1;  
        end
	end
	
	methods
		function isColliding = IsCollidingWith(this, gravitationalBody)
			% ISCOLLIDINGWITH Determines whether or not this body is 
			%	colliding with the provided body.
			%	ISCOLLIDINGWITH(gravitationalBody) Returns true if the
			%	bodies are colliding; otherwise, false.
			
			if (~isa(gravitationalBody, 'GravitationalBody'))
				fprintf('Invalid input object to ApplyForces - must be another gravitational body.');
				return
			end
			
			collisionThreshold = this.Radius + gravitationalBody.Radius;
			
			if (this.DistanceTo(gravitationalBody) <= collisionThreshold)
				isColliding = true;
			else
				isColliding = false;
			end
		end
		
		function distance = DistanceTo(this, gravitationalBody)
			% DISTANCETO Computes the distance between this body and
			%	another supplied body.
			%	DISTANCETO(gravitationalBody) Computes the absolute
			%	distance to another body from this body, using the formula 
			%	derived from the pythagorean theorem.
			
			if (~isa(gravitationalBody, 'GravitationalBody'))
				fprintf('Invalid input object to ApplyForces - must be another gravitational body.');
				return
			end
			
			distance = sqrt ( ...
				(gravitationalBody.XY(1) - this.XY(1))^2 ...
				+ ...
				(gravitationalBody.XY(2) - this.XY(2))^2 ...
				);
		end
		
		function mass = CalculateMass(this)
			% COMPUTEMASS Calculates the absolute mass of this body, using
			% the radius. In this 2D simulation, the mass is represented by
			% the area of the body.
			%	mass = COMPUTEMASS() Calculates the mass of the body using
			%	the area function of a circle.
			
			mass = this.Radius^(2) * pi;
		end
		
		% Github Issue #2
		function ComputeForces(this, gravitationalBody)
			% COMPUTEFORCES Computes the forces between the input body and
			% this body.
			%	COMPUTEFORCES(gravitationalBody) Computes the
			%	gravitational forces which the input body applies to this
			%	body, and cumulatively applies them to this body's stored
			%	forces.
			
			if (~isa(gravitationalBody, 'GravitationalBody'))
				fprintf('Invalid input object to ApplyForces - must be another gravitational body.');
				return
			end
			
			% Calculate the force the input body exerts on this body
			
			% Apply this force cumulatively onto the following variables:
			% this.Acceleration
			% this.XYDirection
            G = 6.67384;
            
			%{
            if (this.XY(1) > gravitationalBody.XY(1))
                Fx = -((G * this.CalculateMass() * gravitationalBody.CalculateMass()) / (this.XY(1) - gravitationalBody.XY(1)) ^ 2 );
            elseif (this.XY(1) < gravitationalBody.XY(1))
                Fx = ((G * this.CalculateMass() * gravitationalBody.CalculateMass()) / (gravitationalBody.XY(1) - this.XY(1)) ^ 2 );
            else
                Fx = 0;
            end
            
            if (this.XY(2) > gravitationalBody.XY(2))
                Fy = -((G * this.CalculateMass() * gravitationalBody.CalculateMass()) / (this.XY(2) - gravitationalBody.XY(2)) ^ 2 );
            elseif (this.XY(2) < gravitationalBody.XY(2))
                Fy = ((G * this.CalculateMass() * gravitationalBody.CalculateMass()) / (gravitationalBody.XY(2) - this.XY(2)) ^ 2 );
            else
                Fy = 0;
            end
			%}
            F = (G * this.CalculateMass() * gravitationalBody.CalculateMass()) / this.DistanceTo(gravitationalBody)^2;
			v = (gravitationalBody.XY - this.XY);
			u = v / norm(v);
			
			Pd = this.XY + (F * u);
			
			this.XYChange = Pd - this.XY;
			
            % this.XYDirection(1) = this.XYDirection(1) + Fx;
            % this.XYDirection(2) = this.XYDirection(2) + Fy;
            % this.Acceleration = sqrt( this.XYDirection(1) ^ 2 + this.XYDirection(2) ^ 2 );
				
			% acceleration
			% speed
			% 
		end
		
		% Github Issue #3
		function SimulateForces(this, deltaTime, seconds)
			% SIMULATEFORCES Simulate the calculated forces over a period of time
			%	SIMULATEFORCES(deltaTime, seconds) Applies the forces over a period of
			%	s seconds, adjusted by deltaTime.
            
			if (~isnumeric(deltaTime))
				fprintf('Invalid input. "deltaTime" must be a numeric value strictly more than 0.');
			end
			
			if (~isnumeric(seconds))
				fprintf('Invalid input. "Seconds" must be a numeric value strictly more than 0.');
			end
			
			if (seconds <= 0)
				fprintf('Invalid input. "Seconds" must be strictly more than 0.');
			end
            	
			% Apply the calculated forces on the body to the following
			% variables:
			
			% this.XY
            
			%{
            xt = this.XY(1);
            yt = this.XY(2);
            alpha = (this.Acceleration * seconds) * deltaTime;
			
			newX = GravitationalBody.lerp(xt, this.XYChange(1), alpha);
			newY = GravitationalBody.lerp(yt, this.XYChange(2), alpha);
			
            this.XY(1) = newX;
            this.XY(2) = newY;
			%}
			
			Pp = this.XY + this.XYChange;
			F = sqrt ( ...
				(Pp(1) - this.XY(1))^2 ...
				+ ...
				(Pp(2) - this.XY(2))^2 ...
				);
			
			this.Acceleration = F / this.CalculateMass();
			
			xt = this.XY(1);
            yt = this.XY(2);
            alpha = (this.Acceleration * seconds^2) * deltaTime;
			
			newX = GravitationalBody.lerp(xt, Pp(1), alpha);
			newY = GravitationalBody.lerp(yt, Pp(2), alpha);
			
            this.XY(1) = newX;
            this.XY(2) = newY;
        end
        
		% GitHub Issue #1
		function AbsorbBody(this, gravitationalBody)
			% ABSORB Absorbs the input body, combining its mass with the
			% mass of this body.
			%	ABSORB(seconds) Absorb the input body. The masses of the
			%	input body and this body are added, and their positions
			%	averaged between each other.
			
			% Combine the masses
			this.Radius = sqrt((gravitationalBody.Radius^2*pi + this.Radius^2*pi) / pi);
			
			% Compute and apply the average positions
			this.XY = [ ...
				(this.XY(1) + gravitationalBody.XY(1)) / 2, ... 
				(this.XY(2) + gravitationalBody.XY(2)) / 2 ...
				];
			
			% "Kill" the other body
			gravitationalBody.IsAlive = false;
		end
		
		function Draw(this, graphAxes)
			xpos = this.XY(1) - this.Radius;
			ypos = this.XY(2) - this.Radius;
			side = this.Radius * 2;
			
				rectangle(graphAxes, ...
					'Position', [xpos, ypos, side, side], ...
					'Curvature', [1, 1], ...
					'FaceColor', this.RGB);
		end
	end
end