//! README 示例路径冒烟测试（与 C `test_readme_example` 一致）。

use scatti::{InputParameter, OutputParameter, Result, Ruckig};

fn near(a: f64, b: f64, eps: f64) -> bool {
    (a - b).abs() <= eps
}

#[test]
fn readme_trajectory_reaches_target() {
    let mut r = Ruckig::new(3, 0.01);
    let mut input = InputParameter::new(3);
    let mut output = OutputParameter::new(3);

    input
        .current_position_mut()
        .copy_from_slice(&[0.0, 0.0, 0.5]);
    input
        .target_position_mut()
        .copy_from_slice(&[5.0, -2.0, -3.5]);
    input.max_velocity_mut().copy_from_slice(&[3.0, 1.0, 0.6]);
    input
        .max_acceleration_mut()
        .copy_from_slice(&[3.0, 2.0, 1.0]);
    input.max_jerk_mut().copy_from_slice(&[4.0, 3.0, 2.0]);

    let mut steps = 0usize;
    loop {
        let res = r.update(&input, &mut output);
        output.pass_to_input(&mut input);
        steps += 1;
        match res {
            Result::Finished => break,
            Result::Working => {}
            _ => panic!("unexpected result {res:?} at step {steps}"),
        }
    }

    let eps = 1e-5;
    let tgt = input.target_position();
    let pos = output.new_position();
    for i in 0..3 {
        assert!(
            near(pos[i], tgt[i], eps),
            "axis {i}: got {} want {}",
            pos[i],
            tgt[i]
        );
    }
    assert!(steps > 0, "expected at least one step");
}
