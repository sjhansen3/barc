#!/usr/bin/env julia
using RobotOS
using JuMP
using Ipopt
@rosimport barc.msg: ECU, Encoder, Ultrasound, Z_KinBkMdl
@rosimport data_service.msg: TimeData
rostypegen()
using barc.msg
using data_service.msg


L_a     = 0.125         # distance from CoG to front axel
L_b     = 0.125         # distance from CoG to rear axel
dt      = 0.1           # time step of system
a_max   = 1             # maximum acceleration

# create model

function barc_model(z,u,dt)
    lr = 0.125;
    psiDot = u[1] / lr * sin(u[2]);
    car_vel = u[1];
    car_rel_vel = z[3] + (z[1]-u[1]);
    car_rel_psi = z[4] + psiDot*dt;
    car_rel_dist = z[2] + car_rel_vel * dt * cos(car_rel_psi);
    z_next = [car_vel,car_rel_dist,car_rel_vel,car_rel_psi];
    return transpose(z_next)
end

function followCar(N,xinit)
    mdl = Model(solver = IpoptSolver());
    #obj = []
    @variable(mdl, vel[1:(N+1)]     )
    @variable(mdl, rel_d[1:(N+1)]   )
    @variable(mdl, rel_vel[1:(N+1)] )
    @variable(mdl, rel_psi[1:(N+1)] )
    @variable(mdl, a[1:N]           )
    @variable(mdl, d_f[1:N]         )
    
    @NLparameter(mdl, vel0     == xinit[1] ); @NLconstraint(mdl, vel[1]     == vel0    );
    @NLparameter(mdl, rel_d0   == xinit[2] ); @NLconstraint(mdl, rel_d[1]   == rel_d0  );
    @NLparameter(mdl, rel_vel0  == xinit[3] ); @NLconstraint(mdl, rel_vel[1] == rel_vel0);
    @NLparameter(mdl, rel_psi0 == xinit[4] ); @NLconstraint(mdl, rel_psi[1] == rel_psi0);
    @NLexpression(mdl, bta[i = 1:N], atan( L_a / (L_a + L_b) * tan(d_f[i]) ) )
    dt = .1
    a_max = .5
    for i = 1:N
        # add constraints
        @NLconstraint(mdl, vel[i+1]     == vel[i]     + dt * a[i]                   )
        @NLconstraint(mdl, rel_d[i+1]   == rel_d[i]   + rel_vel[i] * dt             )
        @NLconstraint(mdl, rel_vel[i+1] == rel_vel[i] + (vel[i] - a[i] * dt)        )
        @NLconstraint(mdl, rel_psi[i+1] == rel_psi[i] + dt * (vel[i]/L_b * sin(bta[i]) ) )
        @constraint(mdl, -a_max <= a[i] <= a_max)
    end
    
    #obj = Array(Any,N)
    obj = @NLexpression(mdl, [i=1:N-1], (rel_d[i]-1)^2)
    #for i = 1:N
        #obj = obj + (rel_d[i]-1)^2	
		#I think the problem is that this needs to be a "non-linear" expression... "
        #push!( obj, (rel_d[i]-1)^2)  # push should append to the array
        #obj[i] = @NLexpression(mdl, 
    #end
    @NLobjective(mdl, Min, (rel_d[1]-1)^2 + (rel_d[2]-1)^2 + (rel_d[3]-1)^2)#brute force #sum(obj))	#instead of summing the array at the end, just summing in the for loop
    # solve
    sol = solve(mdl)
    asol = getvalue(a)
    d_fsol = getvalue(d_f)
    loginfo("Solution Found")
    loginfo(asol)
    loginfo(d_fsol)
end

# Subscribe and get the current state
# xinit = value

# Find the reference we want to follow
function main() 
    loginfo("############## Initializing node... ##########")
    init_node("MPC_Solver")
	loop_rate = Rate(2) #Hz
    N = 4 # Place holder for horizon
    pub = Publisher{ECU}("ECU", queue_size = 1)
    # sub = Subscriber('idk', ECU, callback,queue_size = 10)::RobotOS.Subscriber{barc.msg.ECU}
	loginfo("############## NODE INITIALIZED ##############")
    while ! is_shutdown()
        followCar(N,transpose([0,1,1,0]))

        #acceleration_Sequence = TrackRef(N,xinit,xref,uref)
		acceleration_sequence = [1,2,3]
        # cmd = acceleration_sequence[1]

		acc = 1
		steer = 0
		vel = 0

		if acc > 0
			motor_pwm = (acc-0.157*vel)/0.00305+1500 #TODO convert these mystery numbers into real numbers 
			#use acceleration model
		else
			motor_pwm = (acc-0.1064*vel)/0.003297+1500
			#use deceleration model
		end
		
		servo_pwm = (steer - 1.3784)/-0.00089358 # relationship in radians			(steer-0.4919)/(-3.1882*10^-4)
		
		cmd = ECU(motor_pwm,servo_pwm)
		publish(pub, cmd)
        rossleep(loop_rate)
    end
end

if ! isinteractive()
    main()
end
# convert acc to PWM and publish
# convert steer to PWM and publish

