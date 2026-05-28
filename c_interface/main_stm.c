#include "main_stm.h"

void stm_init(void) {
  stm_controller_init();
}

void stm_step_elapsed(const stm_input_t *in, stm_output_t *out,
                      double elapsed_s) {
  stm_controller_step_elapsed(in, out, elapsed_s);
}

void stm_reset(void) {
  stm_controller_reset();
}
