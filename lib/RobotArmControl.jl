module RobotArmControl
export PIDController, RobotArm, calculate_control, update_plant, gen_signal, run_arm
mutable struct PIDController
  Kp::Float64
  Ki::Float64
  Kd::Float64
  prev_error::Float64
  integral::Float64
end

function PIDController(Kp, Ki, Kd)
  PIDController(Kp, Ki, Kd, 0.0, 0.0)
end

function calculate_control(pid::PIDController, error, dt)
  pid.integral += error * dt
  derivative = (error - pid.prev_error) / dt
  output = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative
  pid.prev_error = error
  return output
end

mutable struct RobotArm
  angle::Float64
  angular_velocity::Float64
  inertia::Float64
  damping::Float64
end

function RobotArm(inertia, damping)
  RobotArm(0.0, 0.0, inertia, damping)
end

function update_plant(arm::RobotArm, torque, dt)
  angular_acceleration = (torque - arm.damping * arm.angular_velocity) / arm.inertia
  arm.angular_velocity += angular_acceleration * dt
  arm.angle += arm.angular_velocity * dt
end

function gen_signal(params::Dict, on::Ref, r::Ref, t::Ref)
    while true
      if on[]
        r[] = params[:A] * sin(2 * pi * params[:f] * t[])
        t[] += params[:dt]
        sleep(params[:dt])
      else
        sleep(params[:dt])
      end
    end
end

function run_arm(params::Dict,r::Ref, t::Ref, on::Ref, angles, arm_reference, errors, times)
  # PID
  Kp = 10.0
  Ki = 2.0
  Kd = 1.0
  pid = PIDController(Kp, Ki, Kd)
  # Arm
  inertia = 1.0
  damping = 0.5
  arm = RobotArm(inertia, damping)
  while true
    if on[]
      @show r[]
      error = r[] - arm.angle
      control = calculate_control(pid, error, params[:polling_rate])
      update_plant(arm, control, params[:polling_rate])
      push!(angles[], arm.angle)
      push!(arm_reference[],r[])
      push!(errors[],error)
      push!(times[],t[])
      sleep(params[:polling_rate])
    else
      sleep(params[:polling_rate])
    end
  end
end
end
