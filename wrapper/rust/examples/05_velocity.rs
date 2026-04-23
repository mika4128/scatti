//*******************************************************************
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
// 
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
// CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
// SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
// 
// Description: scatti - pure C99 jerk-limited real-time trajectory
//              generation. Port of Ruckig with embed-friendly
//              memory and math hooks.
// Author:      杨阳 (Yang Yang) <mika-net@outlook.com>
// Origin:      Based on Ruckig Community Edition by Lars Berscheid
//              (https://github.com/pantor/ruckig)
// License:     MIT (SPDX-License-Identifier: MIT)
// Copyright (c) 2026 杨阳 (Yang Yang)
// *******************************************************************

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
