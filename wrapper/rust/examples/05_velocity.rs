use scatti::{ControlInterface, InputParameter, OutputParameter, Result, Ruckig};

fn main() {
    let mut ruckig = Ruckig::new(3, 0.01);
    let mut input = InputParameter::new(3);
    let mut output = OutputParameter::new(3);

    input.set_control_interface(ControlInterface::Velocity);

    input.current_position_mut().copy_from_slice(&[0.0, 0.0, 0.5]);
    input.current_velocity_mut().copy_from_slice(&[3.0, -2.2, -0.5]);
    input.current_acceleration_mut().copy_from_slice(&[0.0, 2.5, -0.5]);

    input.target_velocity_mut().copy_from_slice(&[0.0, -0.5, -1.5]);
    input.target_acceleration_mut().copy_from_slice(&[0.0, 0.0, 0.5]);

    input.max_acceleration_mut().copy_from_slice(&[3.0, 2.0, 1.0]);
    input.max_jerk_mut().copy_from_slice(&[6.0, 6.0, 4.0]);

    println!("t | position");
    while ruckig.update(&input, &mut output) == Result::Working {
        let pos: Vec<String> = output.new_position().iter().map(|x| format!("{x:.4}")).collect();
        println!("{:.4} | [{}]", output.time(), pos.join(", "));

        output.pass_to_input(&mut input);
    }

    println!(
        "Trajectory duration: {:.4} [s]",
        output.trajectory().duration()
    );
}
