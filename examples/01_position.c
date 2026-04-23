#include <stdio.h>
#include <scatti/scatti.h>

int main(void) {
    /* Create a 3-DOF scatti instance with 10ms control cycle */
    CRuckig *otg = scatti_create(3, 0.01);
    CRuckigInputParameter *inp = scatti_input_create(3);
    CRuckigOutputParameter *out = scatti_output_create(3);

    /* Set current state */
    inp->current_position[0] = 0.0;
    inp->current_position[1] = 0.0;
    inp->current_position[2] = 0.5;

    /* Set target state */
    inp->target_position[0] = 5.0;
    inp->target_position[1] = -2.0;
    inp->target_position[2] = -3.5;

    /* Set limits */
    inp->max_velocity[0] = 3.0;
    inp->max_velocity[1] = 1.0;
    inp->max_velocity[2] = 0.6;
    inp->max_acceleration[0] = 3.0;
    inp->max_acceleration[1] = 2.0;
    inp->max_acceleration[2] = 1.0;
    inp->max_jerk[0] = 4.0;
    inp->max_jerk[1] = 3.0;
    inp->max_jerk[2] = 2.0;

    printf("scatti - C port of Ruckig trajectory generation\n\n");

    /* Run the trajectory */
    CRuckigResult result;
    int step = 0;
    while (1) {
        result = scatti_update(otg, inp, out);

        if (step % 100 == 0 || result == CRuckigFinished) {
            printf("t=%.3f  pos=[%.4f, %.4f, %.4f]  vel=[%.4f, %.4f, %.4f]\n",
                   out->time,
                   out->new_position[0], out->new_position[1], out->new_position[2],
                   out->new_velocity[0], out->new_velocity[1], out->new_velocity[2]);
        }

        scatti_output_pass_to_input(out, inp);
        step++;

        if (result == CRuckigFinished) {
            printf("\nTrajectory finished after %.4f seconds (%d steps)\n",
                   scatti_trajectory_get_duration(out->trajectory), step);
            break;
        }

        if (result != CRuckigWorking) {
            printf("\nError: %d\n", result);
            break;
        }
    }

    scatti_output_destroy(out);
    scatti_input_destroy(inp);
    scatti_destroy(otg);
    return 0;
}
