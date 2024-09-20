module RobotArmControl
export PIDController, RobotArm, calculate_control, update_plant, run_arm
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
  arm.angle = ( arm.angle + arm.angular_velocity * dt) % (2π)
end


function run_arm(arm_channel, output_channel)
  # PID
  Kp = 1.0
  Ki = 0.1
  Kd = 0.1
  pid = PIDController(Kp, Ki, Kd)
  # Arm
  inertia = 0.1
  damping = 0.1
  arm = RobotArm(inertia, damping)
  
  target_angle = 0.0
  t = 0.0
  dt = 0.2

  println("Running arm simulation")
  while true
    # Check for new commands
    if isready(arm_channel)
      cmd = take!(arm_channel)
      if cmd[:command] == :set_target
        @info "Setting target angle to $(cmd[:value])"
        target_angle = cmd[:value]
      elseif cmd[:command] == :reset
        @info "Resetting arm"
        arm.angle = 0.0
        arm.angular_velocity = 0.0
        pid.prev_error = 0.0
        t = 0.0
        continue
      elseif cmd[:command] == :set_pid
        @info "Setting PID to $(cmd[:value])"
        Kp, Ki, Kd = cmd[:value]
        pid = PIDController(Kp, Ki, Kd)
      end
    end

    error = (target_angle - arm.angle) % (2π)
    control = calculate_control(pid, error, dt)
    update_plant(arm, control, dt)
    # we only want to have one element in the channel
    if isready(output_channel)
      take!(output_channel)  # Discard old data
    end 
    # Output scalars instead of vectors
    put!(output_channel, Dict(
      :time => t,
      :angle => arm.angle,
      :velocity => arm.angular_velocity,
      :reference => target_angle,
      :error => error
    ))
    
    t += dt
    sleep(dt)
  end
end
end
