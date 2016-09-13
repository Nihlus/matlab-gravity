classdef GravitationalBody
	%GravitationalBody A structure representing a gravitational body.
	%   This class contains all data and methods required to
	%	hold and compute relational gravity data.
	
	properties
		IsAlive = true
		
		% Positional data as a 2-element vector
		XY = [0, 0]
		
		% Mass
		Radius = 1
		
		% The colour of the mass as an RGB triplet
		RGB = [1, 1, 1]
		
		% Forces
		XYDirection = [0, 0]
		Acceleration = 0
	end
	
	methods
		function mass = CalculateMass(this)
			% COMPUTEMASS Calculates the absolute mass of this body, using
			% the radius. In this 2D simulation, the mass is represented by
			% the area of the body.
			%	mass = COMPUTEMASS() Calculates the mass of the body using
			%	the area function of a circle.
			
			mass = this.Radius^(2) * pi;
		end
		
		function ComputeForces(this, gravitationalBody)
			% COMPUTEFORCES Computes the forces between the input body and
			% this body.
			%	COMPUTEFORCES(gravitationalBody) Computes the
			%	gravitational forces which the input body applies to this
			%	body, and cumulatively applies them to this body's stored
			%	forces.
			
			if (~STRCMP(class(gravitationalBody), 'GravitationalBody'))
				fprintf('Invalid input object to ApplyForces - must be another gravitational body.');
				return
			end
			
			% Calculate the force the input body exerts on this body
			
			% Apply this force cumulatively onto the following variables:
			% this.Acceleration
			% this.XYDirection
		end
		
		function ApplyForces(this, seconds)
			% APPLYFORCES Apply the calculated forces over a period of time
			%	APPLYFORCES(seconds) Applies the forces over a period of
			%	s seconds.
			
			if (isnumeric(seconds))
				fprintf('Invalid input. "Seconds" must be a numeric value strictly more than 0.');
			end
			
			if (seconds <= 0)
				fprintf('Invalid input. "Seconds" must be strictly more than 0.');
			end
			
			% Apply the calculated forces on the body to the following
			% variables:
			
			% this.XY
		end
		
		
		function AbsorbBody(this, gravitationalBody)
			% ABSORB Absorbs the input body, combining its mass with the
			% mass of this body.
			%	ABSORB(seconds) Absorb the input body. The masses of the
			%	input body and this body are added, and their positions
			%	averaged between each other.
			
			% Combine the masses
			
			% Compute and apply the average positions
			
			% "Kill" the other body
			gravitationalBody.IsAlive = false;
		end
	end
	
end

