using GenieFramework
using .RobotArmControl
@genietools

# Create channels for communication
arm_channel = Channel{Dict{Symbol,Any}}(100)
output_channel = Channel{Dict{Symbol,Float64}}(1)

# Start the signal generator and arm simulation tasks
@async begin
  try 
    run_arm(arm_channel, output_channel)
  catch e
    @error "$e"
  end
end

@app begin
  # Arm inputs
  @in angle = 0.0
  @in arm_on = true
  @in Kp = 1.0
  @in Ki = 0.1
  @in Kd = 0.1
  # # Dashboard
  @in refresh_rate = 0.2
  @in dashboard_enabled = false
  @in reset = false
  @in max_data_points = 1000

  # Arm outputs
  @out times = Float64[]
  @out angles = Float64[]
  @out references = Float64[]
  @out errors = Float64[]
  @out velocity = Float64[]
end

@onchange angle begin
  put!(arm_channel, Dict(:command => :set_target, :value => angle))
end

@onchange Kp, Ki, Kd begin
  put!(arm_channel, Dict(:command => :set_pid, :value => (Kp, Ki, Kd)))
end


# @onchange isready begin
# dashboard_enabled = true
# end

@onchange dashboard_enabled begin
  @async begin
    while __model__.dashboard_enabled[] == true
        if isready(output_channel)
          output = take!(output_channel)
          times = vcat(times, output[:time])
          angles = vcat(angles, output[:angle])
          references = vcat(references, output[:reference])
          errors = vcat(errors, output[:error])
          velocity = vcat(velocity, output[:velocity])
          # Limit the stored data points if needed
          if length(times) > max_data_points
            popfirst!(times)
            popfirst!(angles)
            popfirst!(references)
            popfirst!(errors)
            popfirst!(velocity)
          end
        end
        sleep(refresh_rate)
      end
  end
end

@onbutton reset begin
  @info "Resetting arm and dashboard"
  dashboard_enabled = false
  put!(arm_channel, Dict(:command => :reset))
  sleep(refresh_rate*3)
  times = Float64[]
  angles = Float64[]
  references = Float64[]
  errors = Float64[]
  velocity = Float64[]
  dashboard_enabled = true
end


@page("/", "app.jl.html")
