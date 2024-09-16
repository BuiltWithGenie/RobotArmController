using GenieFramework
using .RobotArmControl
@genietools

_signal_params = Dict(:A => 1.0, :f => 0.5, :dt => 0.1)
_reference_on = Ref(false)
_reference = Ref(0.0)
_t = Ref(0.0)
@async gen_signal(_signal_params, _reference_on, _reference, _t)


# arm parameter inputs
_arm_params = Dict(:polling_rate => 0.1)
# arm outputs
_angle = Ref([0.0])
_arm_reference = Ref([0.0])
_error = Ref([0.0])
_times = Ref([0.0])
@async run_arm(_arm_params, _reference, _t, _reference_on, _angle, _arm_reference, _error, _times)


@app begin
  # # Reference
  @in reference_on = false
  @in A = 1
  # # Dashboard
  @in refresh_rate = 0.1
  @in dashboard_enabled = false
  @in reset = false
  # # Arm outputs
  @out times = Float64[]
  @out arm_reference = Float64[]
  @out angle = Float64[]
  @out error = Float64[]

  @onchange reference_on begin
    _reference_on[] = reference_on
  end

  @onchange A begin
    _signal_params[:A] = A
  end

  @onchange isready begin
    @show dashboard_enabled
    @async begin
      while true
        if dashboard_enabled 
          times = _times[]
          angle = _angle[]
          arm_reference = _arm_reference[]
          error = _error[]
          sleep(refresh_rate)
        else 
          sleep(0.5)
        end
      end
    end
  end

  @onbutton reset begin
    _times[] = Float64[]
    _angle[] = Float64[]
    _error[] = Float64[]
    _arm_reference[] = Float64[]
    _t[] = 0
  end

end

@page("/", "app.jl.html")
