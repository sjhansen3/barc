#!/usr/bin/env julia
using RobotOS
using JuMP
using Ipopt
@rosimport barc.msg: ECU, Encoder, Ultrasound, Z_KinBkMdl
@rosimport data_service.msg: TimeData
@rosimport project_231.msg: State
rostypegen()
using barc.msg
using data_service.msg
using project_231.msg

###### DEBUG SAFETY LIMITS #####
dbg = 1

L_a     = 0.125         # distance from CoG to front axel
L_b     = 0.125         # distance from CoG to rear axel
dt      = 0.1           # time step of system
a_max   = 1             # maximum acceleration

car_vel_state = 0.0
rel_dist_state = 0.0
rel_vel_state = 0.0
rel_psi_state = 0.0
#xCurr = [0.0, 0.0, 0.0, 0.0] #Array{Float32}(uninitialized, 4)


function sub_callback(msg::State) #car_cmd::Publisher(ECU))
    loginfo("CALLBACK SUCCESS")
    loginfo(string(msg.us_dist))
    # xinit := car_vel, rel_dist, rel_vel, rel_psi
    #xCurr[:] = [msg.car_vel, msg.us_dist, msg.us_rate, msg.obj_psi] # add angle and y
    setValue(car_vel_state, msg.car_vel)
    setValue(rel_dist_state, msg.us_dist)
    setValue(rel_vel_state, msg.us_rate)
    setValue(rel_psi_state, msg.obj_psi)
end


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
    # create model
    mdl = Model(solver = IpoptSolver())#, quiet=true)
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
        # add constraints (dynamics)
        @NLconstraint(mdl, vel[i+1]     == vel[i]     + dt * a[i]                   )
        @NLconstraint(mdl, rel_d[i+1]   == rel_d[i]   + rel_vel[i] * dt             )
        @NLconstraint(mdl, rel_vel[i+1] == rel_vel[i] + (vel[i] - a[i] * dt)        )
        @NLconstraint(mdl, rel_psi[i+1] == rel_psi[i] + dt * (vel[i]/L_b * sin(bta[i]) ) )
        #end dynamics
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
    
    # making ipopt quiet
    TT = STDOUT
    redirect_stdout()
    sol = solve(mdl)
    redirect_stdout(TT)
    
    asol = getvalue(a)
    d_fsol = getvalue(d_f)
    loginfo("Solution Found")
    #loginfo(asol)
    #loginfo(d_fsol)
    
    return [asol[1], d_fsol[1]]
end

# Subscribe and get the current state
# xinit = value

# Find the reference we want to follow
function main()
    asol = 0.0
    d_fsol = 0.0
    
    loginfo("############## Initializing node... ##########")
    init_node("MPC_Solver")
	loop_rate = Rate(2) #Hz
    N = 4 # Place holder for horizon
    pub = Publisher{ECU}("ECU", queue_size = 1)
    sub = Subscriber("state", State, sub_callback, queue_size = 10) #::RobotOS.Subscriber{project_231.msg.State}
	loginfo("############## NODE INITIALIZED ##############")
    while ! is_shutdown()
        # xinit := car_vel, rel_dist, rel_vel, rel_psi
        loginfo("US DISTANCE: " * string(rel_dist_state))
        sol_array = followCar(N, transpose([car_vel_state, rel_dist_state, rel_vel_state, rel_psi_state]))#xCurr)#transpose([0,1,1,0]))
        
        a_sol = sol_array[1]
        steer_sol = sol_array[2]
        #loginfo("a_sol: " * string(a_sol))
        #loginfo("steer_sol: " * string(steer_sol))
        #acceleration_Sequence = TrackRef(N,xinit,xref,uref)
		#acceleration_sequence = [1,2,3]
        # cmd = acceleration_sequence[1]

		#acc = 1
		#steer = 0
		#vel = 0

		if a_sol > 0
			motor_pwm = (a_sol-0.157*car_vel_state)/0.00305+1500 #TODO convert these mystery numbers into real numbers 
			#use acceleration model
		else
			motor_pwm = (a_sol-0.1064*car_vel_state)/0.003297+1500
			#use deceleration model
		end
		
		servo_pwm = (steer_sol - 1.3784)/-0.00089358 # relationship in radians			(steer-0.4919)/(-3.1882*10^-4)
		

        ######### PRINTOUT FOR DEBUG #######
        loginfo("a_sol: " * string(a_sol) * "\tsteer_sol: " * string(steer_sol))
        loginfo("MOTOR: " * string(motor_pwm) * "\tSERVO: " * string(servo_pwm))
        
        ######### SAFETY LIMITS FOR TESTING #######
        if dbg == 1
            if motor_pwm > 1580
                motor_pwm = 1580
            end
            
            if motor_pwm < 1420
                motor_pwm = 1420
            end
        
            if servo_pwm > 1600
                servo_pwm = 1600
            end

            if servo_pwm < 1400
                servo_pwm = 1400
            end
        end

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

