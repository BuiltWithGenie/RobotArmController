using GenieFramework
using .RobotArmControl
@genietools
Genie.config.webchannels_keepalive_frequency = 0

# Create channels for communication
# The channels can only store one value at a time. We'll write to the input channel and periodically read from the output channel.
arm_input_channel = Channel{Dict{Symbol,Any}}(1)
arm_output_channel = Channel{Dict{Symbol,Float64}}(1)

# Start the signal generator and arm simulation tasks
@async begin
  try 
    run_arm(arm_input_channel, arm_output_channel)
  catch e
    @error "$e"
  end
end

@app begin
  # Arm inputs
  @in angle = 0.0
  @in Kp = 1.0
  @in Ki = 0.1
  @in Kd = 0.1

  # Dashboard
  @in dashboard_enabled = false
  @in reset = false
  @in max_data_points = 200
  @out t = 0.0
  @private client_connected = true
  @private refresh_rate = 0.05

  # Arm outputs
  @out times = Float64[]
  @out angles = Float64[]
  @out references = Float64[]
  @out errors = Float64[]
  @out velocity = Float64[]

  # Push a new angle value to the arm
  @onchange angle begin
    put!(arm_input_channel, Dict(:command => :set_target, :value => angle))
  end

  # Push the PID controller values to the arm
  @onchange Kp, Ki, Kd begin
    put!(arm_input_channel, Dict(:command => :set_pid, :value => (Kp, Ki, Kd)))
  end

  # Disable the dashboard by default
  @onchange isready begin
    dashboard_enabled = true
  end

  # Start updating the dashboard with the arm output
  @onchange dashboard_enabled begin
    # This task will launch a loop that checks the arm output and updates the dashboard.
    #
    # The dashboard_enabled variable is passed to the handler by value. Changing it from the dashboard
    # will change the value of the reactive variable, but it won't change its value inside the handler. 
    # Hence, it will always have the same value inside the while loop.
    #
    # To check on the up-to-date value from the loop, we need to access the __model__.dashboard_enabled variable.
    # __model__ is the struct that contains the reactive variables. 
    # See here for more details: https://learn.genieframework.com/framework/stipple.jl/docs/reactivity#reactive-variable-scoping
    @async begin
      while __model__.dashboard_enabled[] && client_connected
        # wait(arm_output_channel)
        output = take!(arm_output_channel)
        # Limit the stored data points if needed
        if length(times) > max_data_points-1
          map(popfirst!, (times, angles, references, errors, velocity))
        end
        times = vcat(times, output[:time])
        angles = vcat(angles, output[:angle])
        references = vcat(references, output[:reference])
        errors = vcat(errors, output[:error])
        velocity = vcat(velocity, output[:velocity])
        t = output[:time]
        @show t
        sleep(refresh_rate)
        # Each client is connected via a channel with id stored in __model__.channel__
        # The Genie.WebChannels.SUBSCRIPTIONS dictionary contains details about the clients connected to the app on a channel.
        # When the browser window is closed, the channel ID is removed from the dictionary.
        client_connected = __model__.channel__ in keys(Genie.WebChannels.SUBSCRIPTIONS)
      end
    end
  end

  @onbutton reset begin
    @notify "Resetting arm and dashboard"
    dashboard_enabled = false
    put!(arm_input_channel, Dict(:command => :reset))
    (times, angles, references, errors, velocity) = [Float64[] for _ in 1:5]
    sleep(refresh_rate*3)
    dashboard_enabled = true
  end
end


@page("/", "app.jl.html")
