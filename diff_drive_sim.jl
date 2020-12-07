# note arrays start at 1 in julia

# array of x, x_dot, x_double_dot in and out ... could eventually replace with my own struct
function trapIntegral(x_in::Array{Number,1},a_out::Number,delta_t::Number)
	x_out = Array{Number,1}(undef,3)
	x_out[3] = a_out
	#eventually could use a library
	x_out[2] = ((x_in[3]+x_out[3])/2)*delta_t + x_in[2]
	x_out[1] = ((x_in[2]+x_out[2])/2)*delta_t + x_in[1]
	x_out
end

mutable struct LinearSystem
	m::Number
	c::Number
	k::Number
	x::Array{Number,1}
	dt::Number
end

function LinearSystem(m::Number,c::Number,k::Number,dt::Number)
	LinearSystem(m,c,k,[0,0,0],dt)
end

function forceFunction(u,s::LinearSystem)
	u/s.m - (s.k/s.m)*s.x[1] - (s.c/s.m)*s.x[2]
end

function update(u::Number,s::LinearSystem)
	s.x = trapIntegral(s.x,forceFunction(u,s),s.dt)
end

#old LinearSystem tester
# function move_wheel(wheel::LinearSystem,t_end::Number)
# 	t=0
# 	u=1
# 	println(wheel.dt)
# 	while(t<t_end)
# 		t+=wheel.dt
# 		println(wheel.x)
# 		update(u,wheel)
# 	end
# end

#should properly define a wheel type, in order to properly encode units
mutable struct Wheel
	d::Number
	angular::LinearSystem
	v::Number
end

function Wheel(m::Number,c::Number,d::Number,dt::Number)
	Wheel(d,LinearSystem(m,c,0,dt),0)
end

function update(u::Number,w::Wheel)
	update(u,w.angular)
	w.v = w.angular.x[2]*pi*w.d
end

# Put in global_frame
mutable struct DiffDriveSystem
	wheel_base_width::Number
	overall_mass::Number
	left_wheel::Wheel
	right_wheel::Wheel
	x::Array{Number,1}
	θ::Array{Number,1} #all characters are ok
	dt::Number
end

function DiffDriveSystem(w_m,c,w_d,b_width,o_mass,dt)
	DiffDriveSystem(b_width,o_mass,Wheel(w_m,c,w_d,dt),Wheel(w_m,c,w_d,dt),[0,0,0],[0,0,0],dt)
end

function update(u_left::Number,u_right::Number,drv::DiffDriveSystem)
	update(u_left,drv.left_wheel)
	update(u_right,drv.right_wheel)

	new_x = Array{Number,1}(undef,3)
	new_x[2] = (drv.left_wheel.v + drv.right_wheel.v)/2
	new_x[3] = (new_x[2] - drv.x[2])/drv.dt
	new_x[1] = drv.x[1] + ((drv.x[2]+new_x[2])/2)*drv.dt

	#positive is counter clockwise, works for visualization in 2d plane
	new_θ = Array{Number,1}(undef,3)
	new_θ[2] =  (drv.right_wheel.v - drv.left_wheel.v)/drv.wheel_base_width
	new_θ[3] = (new_θ[2] - drv.θ[2])/drv.dt
	new_θ[1] = drv.θ[1] + ((drv.θ[2]+new_θ[2])/2)*drv.dt

	drv.x = new_x
	drv.θ = new_θ
end

mutable struct DiffDrivePose
	drv::DiffDriveSystem
	x::Number
	y::Number
	θ::Number
end

function DiffDrivePose(drv::DiffDriveSystem)
	DiffDrivePose(drv,0,0,0)
end

function update(u_left,u_right,pose::DiffDrivePose)
	update(u_left,u_right,pose.drv)
	pose.θ = drv.θ[1]
	pose.x += drv.x[2]*cos(pose.θ)*drv.dt
	pose.y += drv.x[2]*sin(pose.θ)*drv.dt
end

#some control methods

function coerce(val,min_val,max_val)
	if(val>max_val) return max_val end
	if(val<min_val) return min_val end
	return val
end

function arcadeDrive(lin_throttle,ang_throttle,u_max)::Array{Number,1}
	out = Array{Number,1}(undef,2)
	#left, then right, positive ang is clockwise
	out[1] = coerce((lin_throttle+ang_throttle)*u_max,-u_max,u_max)
	out[2] = coerce((lin_throttle-ang_throttle)*u_max,-u_max,u_max)
	#create some caps later
	return out
end


function drive_around(u_max,pose::DiffDrivePose)
	#actual movement content...
	t = 0
	t_max = 10
	data = Array{Number,1}(undef,2)

	while(t<t_max)

		#action engine, other ways to write as well
		if(t>=0 && t<5)
			u = arcadeDrive(1,0,u_max)
			update(u[1],u[2],pose)
		end

		if(t>=5 && t<10)
			u = arcadeDrive(.5,1,u_max)
			update(u[1],u[2],pose)
		end

		if(t>=10 && t<15)
			u = arcadeDrive(.3,-0.5,u_max)
			update(u[1],u[2],pose)
		end

		# println("$(pose.x) $(pose.y) $(pose.θ)")
		row = [pose.x,pose.y]
		data = [data,row']

		t+=pose.drv.dt
	end
	data
end


# let's say SI units for all of this kg, m, s, etc.
#wheel = LinearSystem(0.3,0.5,0,.01)
# println(wheel)
# move_wheel(wheel,.3)
#DiffDriveSystem(w_m,c,w_d,b_width,o_mass,dt)
#.3 kg wheels, c= .5, 5 cm wheels, 60 cm drive base, 10 kg system, 100 ms update   
drv = DiffDriveSystem(0.3,0.5,.05,.6,10,0.1)
pose = DiffDrivePose(drv)

#u_max of 100(N sort of)
data = drive_around(100,pose)

print(data)