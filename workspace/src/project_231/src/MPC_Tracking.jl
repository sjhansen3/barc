#!/usr/bin/env julia
#loginfo("Loading RobotOS")
using RobotOS
loginfo("Loading JuMP")
using JuMP
loginfo("Loading Ipopt")
using Ipopt
loginfo("Loading ros stuff")
@rosimport barc.msg: ECU, Encoder, Ultrasound, Z_KinBkMdl
@rosimport data_service.msg: TimeData
@rosimport project_231.msg: State
loginfo("Generating types")
rostypegen()
loginfo("Declaring message types")
using barc.msg
using data_service.msg
using project_231.msg
loginfo("Loading COMPLETE")
###########################################


###### DEBUG SAFETY LIMITS #####
dbg = 1
###########################################

###########################################
####### CAR PARAMETERS
L_a             = 0.125         # distance from CoG to front axel
L_b             = 0.125         # distance from CoG to rear axel
dt              = 0.1           # time step of system
a_max           = 1             # maximum acceleration
N               = 5             # MPC Solution Horizon
desired_dist    = .5           # Desired following distance in meters
###########################################

###########################################
# CREATE MODEL
loginfo("Initializing Model!")
mdl = Model(solver = IpoptSolver())

# Create our optimal state variable
@variable(mdl, vel[1:(N+1)]     )
@variable(mdl, rel_d[1:(N+1)]   )
@variable(mdl, rel_vel[1:(N+1)] )
@variable(mdl, rel_psi[1:(N+1)] )
# Create our optimal output variable
@variable(mdl, a[1:N]           ) # accel
@variable(mdl, d_f[1:N]         ) # steer

# LEFT SIDE, Create the initial condition variable, this is updated using the state 
# callback function

# RIGHT SIDE, Create the initial condition constraint for MPC solution. This is
# updated because it is a constraint on the variable, which is updated with
# the state callback function
@NLparameter(mdl, vel0     == 0 ); @NLconstraint(mdl, vel[1]     == vel0    );
@NLparameter(mdl, rel_d0   == 1 ); @NLconstraint(mdl, rel_d[1]   == rel_d0  );
@NLparameter(mdl, rel_vel0 == 1 ); @NLconstraint(mdl, rel_vel[1] == rel_vel0);
@NLparameter(mdl, rel_psi0 == 0 ); @NLconstraint(mdl, rel_psi[1] == rel_psi0);
@NLexpression(mdl, bta[i = 1:N], atan( L_a / (L_a + L_b) * tan(d_f[i]) ) )

# Add constraints for each timestep
for i = 1:N
    # add constraints (dynamics)
    @NLconstraint(mdl, vel[i+1]     == vel[i]     + dt * a[i]                   )
    @NLconstraint(mdl, rel_d[i+1]   == rel_d[i]   + rel_vel[i] * dt             )
    @NLconstraint(mdl, rel_vel[i+1] == rel_vel[i] + (vel[i] - a[i] * dt)        )
    @NLconstraint(mdl, rel_psi[i+1] == rel_psi[i] + dt * (vel[i]/L_b * sin(bta[i]) ) )
    #end dynamics
    @constraint(mdl, -a_max <= a[i] <= a_max)
end

# Add objective
@NLobjective(mdl, Min, (rel_d[1]-desired_dist)^2 + (rel_d[2]-desired_dist)^2 + (rel_d[3]-desired_dist)^2 + (rel_d[4]-desired_dist)^2 + (rel_d[5]-desired_dist)^2)#brute force #sum(obj))	#instead of summing the array at the end, just summing in the for loop
###########################################

###########################################
# SOLVE INITIAL MPC
# making ipopt quiet
loginfo("Model Creation complete, starting initial solve!")
#TT = STDOUT
#redirect_stdout()
solve(mdl)
#redirect_stdout(TT)

a_sol = getvalue(a[1])
d_fsol = getvalue(d_f[1])
loginfo("INITIAL Solution Found")
###########################################

###########################################
# Update states with callback function
function sub_callback(msg::State) 
    # Updates the MPC initial values here
    #loginfo("CALLBACK SUCCESS")
    #loginfo(@sprintf("CALLBACK:\t%.3f\t%.3f\t%.3f\t%.3f", msg.car_vel, msg.us_dist, msg.us_rate, msg.obj_psi))

    setvalue(vel0, msg.car_vel)
    setvalue(rel_d0, msg.us_dist)
    setvalue(rel_vel0, msg.us_rate)
    setvalue(rel_psi0, msg.obj_psi)
end
###########################################

###########################################
# MAIN LOOP FUNCTION
function main()
    loginfo("############## Initializing node... ##########")
    init_node("MPC_Solver")
	loop_rate = Rate(10) # Hz
    pub = Publisher{ECU}("ecu_pwm", queue_size = 1)
    sub = Subscriber("state", State, sub_callback, queue_size = 10)
	loginfo("############## NODE INITIALIZED ##############")
    while ! is_shutdown()
        # Debug to find if our state is updating...
		us_dist = getvalue(rel_d0)
        rel_vel = getvalue(rel_vel0)
        #loginfo("US DISTANCE: " * string(getvalue(rel_d0)))

        # making ipopt quiet
        TT = STDOUT
        redirect_stdout()
        status = solve(mdl) # SOLVE MPC
        redirect_stdout(TT)

        # Get the optimal inputs
        if status == :Optimal
            loginfo("OPTIMAL SOLUTION FOUND")
            a_sol  = getvalue( a[1]   )
            d_fsol = getvalue( d_f[1] )
        else
            loginfo("SOLUTION IS NONOPTIMAL")
            a_sol  = 0.0
            d_fsol = 0.0
        end
        
        # Adjust output to PWMs using system ID values
		if a_sol > 0
			motor_pwm = (a_sol-0.1570*getvalue(vel0))/0.003050+1500
			#use acceleration model
		else
			motor_pwm = (a_sol-0.1064*getvalue(vel0))/0.003297+1500
			#use deceleration model
		end
		
		servo_pwm = (d_fsol - 1.3784)/-0.00089358 # relationship in radians			(steer-0.4919)/(-3.1882*10^-4)
		

        ######### PRINTOUT FOR DEBUG #######
        loginfo(@sprintf("US: %.3f\t rel_vel; %.3f\t MOTOR: %.3f \tSERVO: %.3f\ta_sol: %.3f\td_fsol: %.3f", us_dist, rel_vel, motor_pwm, servo_pwm, a_sol, d_fsol)) #"a_sol: " * string(a_sol) * "\td_fsol: " * string(d_fsol))
        #loginfo(#"MOTOR: " * string(motor_pwm) * "\tSERVO: " * string(servo_pwm))
        
        ######### SAFETY LIMITS FOR TESTING #######
        if dbg == 1
            if motor_pwm > 1580
                motor_pwm = 1580
            end
            
            if motor_pwm < 1200
                motor_pwm = 1200
            end
        
            if servo_pwm > 1600
                servo_pwm = 1600
            end

            if servo_pwm < 1400
                servo_pwm = 1400
            end
        end

        # Cast the command to ECU message
		cmd = ECU(motor_pwm,servo_pwm)
		publish(pub, cmd) # PUBLISH ECU COMMAND
        rossleep(loop_rate)
    end
end

if ! isinteractive()
    main()
end

