#!/usr/bin/env julia

using JuMP
using Ipopt
using PyPlot

#function solve_ftoc(vel0,rel_d0,rel_vel0,rel_psi0)
    #vel0 - msg.car_vel
    #rel_d0 - msg.us_dist
    #rel_vel0 - msg.us_rate
    #rel_psi0 - msg.obj_psi

####### CAR PARAMETERS
L_a             = 0.125         # distance from CoG to front axel
L_b             = 0.125         # distance from CoG to rear axel
dt              = 0.1           # time step of system
a_max           = 0.24             # maximum acceleration
N               = 9             # MPC Solution Horizon
desired_dist    = 1           # Desired following distance in meters
###########################################
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
    @NLconstraint(mdl, rel_vel[i+1] == rel_vel[i] + (-a[i] * dt)        )
    @NLconstraint(mdl, rel_psi[i+1] == rel_psi[i] + dt * (vel[i]/L_b * sin(bta[i]) ) )
    #end dynamics
    @constraint(mdl, -a_max <= a[i] <= a_max)
end

# Add objective
@NLobjective(mdl, Min, sum(200*(rel_d[i]-desired_dist)^2+ a[i]^2 for i=1:N))
#@NLobjective(mdl, Min, (rel_d[1]-desired_dist)^2 + (rel_d[2]-desired_dist)^2 + (rel_d[3]-desired_dist)^2 + (rel_d[4]-desired_dist)^2 + (rel_d[5]-desired_dist)^2)#brute force #sum(obj))	#instead of summing the array at the end, just summing in the for loop
###########################################

###########################################
# SOLVE INITIAL MPC
# making ipopt pt
#loginfo("Model Creation complete, starting initial solve!")

#plan = Dict("acc"=>a_sol, "steer"=>d_fsol, "follow_distance"=>rel_d_sol,"v_sol"=>v_sol)
#return plan
#end

###########################################
# MAIN LOOP FUNCTION
function main()
    #vel0 - msg.car_vel
    #rel_d0 - msg.us_dist
    #rel_vel0 - msg.us_rate
    #rel_psi0 - msg.obj_psi
    num = 250
    accelerations = zeros(num)
    steering = zeros(num)
    follow_distance = zeros(num)
    velocities = zeros(num)
    predicted_follow_distance = zeros(num)
    predicted_velocities = zeros(num)

    dt = 0.1
    idx = 1

    car_vel = 0        #my velocity
    us_dist = 1      #my distance
    us_rate = 0    #the rate of change between me and the other car
    obj_psi = 0    #rate of change between me and the other car
    while idx <=num
        #print("velocity before plan: ",vel0)
        #plan = solve_ftoc(vel0,rel_d0,rel_vel0,rel_psi0)
        setvalue(vel0, car_vel)
        setvalue(rel_d0, us_dist)
        setvalue(rel_vel0, us_rate)
        setvalue(rel_psi0, obj_psi)

        #####Solve
        TT = STDOUT
        redirect_stdout()
        solve(mdl)
        redirect_stdout(TT)

        #####Get_values
        a_sol = getvalue(a)
        d_fsol = getvalue(d_f)
        v_sol = getvalue(vel)
        rel_d_sol = getvalue(rel_d)
        
        accl_cmd = a_sol[1]
        #steer_cmd = rel_d_sol[1]
        
        accelerations[idx] = accl_cmd
        #steering[idx] = steer_cmd

        predicted_velocities[idx] = v_sol[1]
        predicted_follow_distance[idx] = rel_d_sol[1]
        #println("velocity after plan before propigate",vel0)
        us_rate, us_dist, car_vel = propigate_states(car_vel,us_dist, accl_cmd,dt)

        #println("velocity after propigate",vel0)
        velocities[idx] = car_vel
        follow_distance[idx] = us_dist

        idx +=1
    end

    t = linspace(dt,dt*num,num)
    #println("actual follow_distance: ",follow_distance)
    #println("steer: ",steering)
    #println("acc: ",accelerations)
    #println("actual vel: ",velocities)
    #figure()
    #println(size(t),size(steering))
    #plot(t,steering, "b.")
    #figure()

    ax1=subplot(311)        
    plot(t,follow_distance, "b:")
    plot(t,predicted_follow_distance, "r-")
    legend(["actual follow dist", "predicted follow dist"])
    
    ax1=subplot(312,sharex=ax1)        
    plot(t,velocities, "b:")
    plot(t,predicted_velocities, "r-")
    legend(["actual vel","MPC predicted vel"])
   
    ax1=subplot(313,sharex=ax1)    
    plot(t,accelerations, "r:")
    title("Accleration commands")
    show()
    #legend(["","follow_distance","velocities","accelerations"])
end

function propigate_states(prev_vel,prev_usdist, accl_cmd,dt)
    #println("acceleration",accl_cmd,"prev_vel",prev_vel)    
    car_vel = prev_vel+accl_cmd*dt
    obj_vel = .5 #assume constant 1 m/s velocity of other car
    obj_acc = 0 #no acceleration

    us_rate = obj_vel-car_vel #negative moving towards the other car
    us_dist = 0.5*(obj_acc-accl_cmd)*dt^2+us_rate*dt+prev_usdist
    
    #println("post_vel",car_vel)
    return (us_rate, us_dist, car_vel)
end

if ! isinteractive()
    main()
end

