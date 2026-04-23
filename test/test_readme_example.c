/* Mirrors README.md "使用示例" — smoke test that the documented API path works. */
#include <math.h>
#include <stdio.h>
#include <scatti/scatti.h>

static int near(double a, double b, double eps) {
    return fabs(a - b) <= eps;
}

int main(void) {
    CRuckig *otg = scatti_create(3, 0.01);
    CRuckigInputParameter *inp = scatti_input_create(3);
    CRuckigOutputParameter *out = scatti_output_create(3);

    if (!otg || !inp || !out) {
        fprintf(stderr, "create failed (OOM)\n");
        scatti_output_destroy(out);
        scatti_input_destroy(inp);
        scatti_destroy(otg);
        return 2;
    }

    inp->current_position[0] = 0.0;
    inp->current_position[1] = 0.0;
    inp->current_position[2] = 0.5;
    inp->target_position[0] = 5.0;
    inp->target_position[1] = -2.0;
    inp->target_position[2] = -3.5;
    inp->max_velocity[0] = 3.0;
    inp->max_velocity[1] = 1.0;
    inp->max_velocity[2] = 0.6;
    inp->max_acceleration[0] = 3.0;
    inp->max_acceleration[1] = 2.0;
    inp->max_acceleration[2] = 1.0;
    inp->max_jerk[0] = 4.0;
    inp->max_jerk[1] = 3.0;
    inp->max_jerk[2] = 2.0;

    CRuckigResult result;
    int step = 0;
    while (1) {
        result = scatti_update(otg, inp, out);
        scatti_output_pass_to_input(out, inp);
        step++;

        if (result == CRuckigFinished)
            break;
        if (result != CRuckigWorking) {
            fprintf(stderr, "unexpected result: %d at step %d\n", result, step);
            scatti_output_destroy(out);
            scatti_input_destroy(inp);
            scatti_destroy(otg);
            return 1;
        }
    }

    const double eps = 1e-5;
    if (!near(out->new_position[0], inp->target_position[0], eps) ||
        !near(out->new_position[1], inp->target_position[1], eps) ||
        !near(out->new_position[2], inp->target_position[2], eps)) {
        fprintf(stderr, "final position mismatch: got [%g,%g,%g] want [%g,%g,%g]\n",
                out->new_position[0], out->new_position[1], out->new_position[2],
                inp->target_position[0], inp->target_position[1], inp->target_position[2]);
        scatti_output_destroy(out);
        scatti_input_destroy(inp);
        scatti_destroy(otg);
        return 3;
    }

    printf("readme example: ok, %d steps, t=%.3f\n", step, out->time);

    scatti_output_destroy(out);
    scatti_input_destroy(inp);
    scatti_destroy(otg);
    return 0;
}
