ENV["CHANNEL__"] = "CommonChannel"
using GenieFramework
using .RobotArmControl
@genietools
Genie.config.webchannels_keepalive_frequency = 0

# Create channels for communication
# The channels can only store one value at a time. We'll write to the input channel and periodically read from the output channel.
arm_input_channel = Channel{Dict{Symbol,Any}}(1)
arm_output_channel = Channel{Dict{Symbol,Float64}}(1)

# Start the arm
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
  @in max_data_points = 200
  @out t_init = 0.0
  @private refresh_rate = 0.2
  @out run_task = false

  # # Arm outputs
  @out times = Float64[]
  @out angles = Float64[]
  @out references = Float64[]
  @out errors = Float64[]
  @out velocity = Float64[]

  @onchange isready begin
  end
  
  # Push a new angle value to the arm
  @onchange angle begin
    @show global_lock
    dashboard_enabled && put!(arm_input_channel, Dict(:command => :set_target, :value => angle))
  end

  # Push the PID controller values to the arm
  @onchange Kp, Ki, Kd begin
    dashboard_enabled && put!(arm_input_channel, Dict(:command => :set_pid, :value => (Kp, Ki, Kd)))
  end

  # Start updating the dashboard with the arm output
  @onchange dashboard_enabled begin
    if dashboard_enabled
      (times, angles, references, errors, velocity) = [Float64[] for _ in 1:5]
      output = take!(arm_output_channel)
      t_init = output[:time]
      run_task = true
    end
  end

  # Continuously update the dashboard with the arm output
  @onchange run_task begin
    @async begin
      while dashboard_enabled 
        output = take!(arm_output_channel)
        # Limit the stored data points if needed
        if length(times) > max_data_points-1
          map(popfirst!, (times, angles, references, errors, velocity))
        end
        times = vcat(times, output[:time]-t_init)
        angles = vcat(angles, output[:angle])
        references = vcat(references, output[:reference])
        errors = vcat(errors, output[:error])
        velocity = vcat(velocity, output[:velocity])
        # Each client is connected via a channel with id stored in __model__.channel__
        # The Genie.WebChannels.SUBSCRIPTIONS dictionary contains details about the clients connected to the app on a channel.
        # When the browser window is closed, the channel ID is removed from the dictionary.
        if !(__model__.channel__ in keys(Genie.WebChannels.SUBSCRIPTIONS)); break; end
        sleep(refresh_rate)
      end
      run_task[!] = false
      @info "Stopped updating dashboard"
    end
  end
end


@page("/", "app.jl.html")

# Why do we run the task in a separate handler instead of running it in the dashboard_enabled handler?
#
# The dashboard_enabled variable is passed to the handler by value. Changing it from the dashboard
# will change the value of the reactive variable, but it won't change its value inside the handler. 
# Hence, it would always have the same value inside the while loop.
#
# If we run the loop in another handler, it'll have access to the updated reactive variable value. This
# way the loop can exit when dashboard_enabled becomes false. 
# 
# More info: https://learn.genieframework.com/framework/stipple.jl/docs/reactivity#reactive-variable-scoping