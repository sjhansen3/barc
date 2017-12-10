#!/usr/bin/env julia

using RobotOS
using JuMP
using Ipopt

# ros publish
@rosimport barc.msg: ECU, Encoder, Ultrasound, Z_KinBkMdl
@rosimport data_service.msg: TimeData
rostypegen()
using barc.msg
using data_service.msg

# import the path (states and refernce)
#xref =
#xinit
m = Model(solver = IpoptSolver());

#function callback(msg::*states*) #car_cmd::Publisher(ECU))
#    xinit[:] = [msg.motor, msg.servo] # add angle and y

#end

function bikeFE1(x,v,a)
    TS = 0.2;
    lf = 1.738;
    lr = 1.738;
    nX = 4;

    return x+TS*v;
end

#function bikeFE2(x,y,v,psi,a,df)
    # get_param("planning_params")
    # TS = planning_params["TS"]
    # lf = planning_params["lf"]
    # lr = planning_params["lr"]
#    TS = 0.2;
#    lf = 1.738;
#    lr = 1.738;
#    return y+TS*v*sin(psi+df);
#end

function bikeFE3(x,v,a)
    TS = 0.2;
    lf = 1.738;
    lr = 1.738;
    return  v+TS*a;
end

#function bikeFE4(x,y,v,psi,a,df)
    # x is [x y v psi]
#    TS = 0.2;
#    lf = 1.738;
#    lr = 1.738;
#    nX = 4;
#    return psi+TS*v*sin(df)/lr;
#end

# Subscribe and read the start position

function TrackRef(N,xinit,xref,uref)
    @variable(m, x[1:2, 1:(N+1)])
    @variable(m, u[1,1:N])
    # Q =
    # R =
    obj=0
    for i = 1:N
        # add constraints
        @constraint(m, x[1:2,1] == xinit)
        @constraint(m, x[1,i+1] == bikeFE1(x[1,i],x[2,i],u[1,i]))
        # @constraint(m, x[2,i+1] == bikeFE2(x[1,i],x[2,i],x[3,i],x[4,i],u[1,i],u[2,i]))
        @constraint(m, x[2,i+1] == bikeFE3(x[1,i],x[2,i],u[1,i]))
        # @constraint(m, x[4,i+1] == bikeFE3(x[1,i],x[2,i],x[3,i],x[4,i],u[1,i],u[2,i]))
        obj = obj + (x[:,i] - xref[:])'*Q*(x[:,i] - xref[:,i]) + (u[:,i])'*R*(u[:,i])
    end

    # solve
    sol = solve(m)
    xsol = getValue(x)
    usol = getValue(u)
    return usol
end

# Subscribe and get the current state
# xinit = value

# Find the reference we want to follow
function main() 
    
    init_node("MPC_Solver")
	loop_rate = Rate(2)
    N = 4 # Place holder for horizon
    pub = Publisher("ecu", ECU, queue_size = 1)
    # sub = Subscriber('idk', ECU, callback,queue_size = 10)::RobotOS.Subscriber{barc.msg.ECU}
    while ! is_shutdown()

        #acceleration_Sequence = TrackRef(N,xinit,xref,uref)
		acceleration_sequence = [1, 2,3]
        # cmd = acceleration_sequence[1]
		cmd = ECU(2,1)
		publish(pub, cmd)
        rossleep(loop_rate)
    end
end

if ! isinteractive()
    main()
end
# convert acc to PWM and publish
# convert steer to PWM and publish

