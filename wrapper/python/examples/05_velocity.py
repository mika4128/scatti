"""Velocity control example."""

from scatti import (
    ControlInterface,
    InputParameter,
    OutputParameter,
    Result,
    Ruckig,
)


if __name__ == "__main__":
    ruckig = Ruckig(3, 0.01)
    inp = InputParameter(3)
    out = OutputParameter(3)

    inp.control_interface = ControlInterface.Velocity

    inp.current_position = [0.0, 0.0, 0.5]
    inp.current_velocity = [3.0, -2.2, -0.5]
    inp.current_acceleration = [0.0, 2.5, -0.5]

    inp.target_velocity = [0.0, -0.5, -1.5]
    inp.target_acceleration = [0.0, 0.0, 0.5]

    inp.max_acceleration = [3.0, 2.0, 1.0]
    inp.max_jerk = [6.0, 6.0, 4.0]

    print("t | position")
    while ruckig.update(inp, out) == Result.Working:
        pos = ", ".join(f"{p:.4f}" for p in out.new_position)
        print(f"{out.time:.4f} | [{pos}]")
        out.pass_to_input(inp)

    print(f"Trajectory duration: {out.trajectory.duration:.4f} [s]")
