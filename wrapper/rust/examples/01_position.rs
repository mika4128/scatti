use scatti::{InputParameter, OutputParameter, Result, Ruckig};

fn main() {
    // Create instances: 3 DOFs, 0.01 s control cycle
    let mut ruckig = Ruckig::new(3, 0.01);
    let mut input = InputParameter::new(3);
    let mut output = OutputParameter::new(3);

    // Set input parameters
    input.current_position_mut().copy_from_slice(&[0.0, 0.0, 0.5]);
    input.current_velocity_mut().copy_from_slice(&[0.0, -2.2, -0.5]);
    input.current_acceleration_mut().copy_from_slice(&[0.0, 2.5, -0.5]);

    input.target_position_mut().copy_from_slice(&[5.0, -2.0, -3.5]);
    input.target_velocity_mut().copy_from_slice(&[0.0, -0.5, -2.0]);
    input.target_acceleration_mut().copy_from_slice(&[0.0, 0.0, 0.5]);

    input.max_velocity_mut().copy_from_slice(&[3.0, 1.0, 3.0]);
    input.max_acceleration_mut().copy_from_slice(&[3.0, 2.0, 1.0]);
    input.max_jerk_mut().copy_from_slice(&[4.0, 3.0, 2.0]);

    // Generate the trajectory within the control loop
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
