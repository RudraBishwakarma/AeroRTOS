#ifndef MEMORY_PROFILER_H
#define MEMORY_PROFILER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void memory_profiler_init(void);
bool memory_profiler_register_task(uint32_t task_id, const char *name, uint32_t stack_size);
void memory_profiler_update_task(uint32_t task_id);
void memory_profiler_print_stats(void);

#ifdef __cplusplus
}
#endif

#endif /* MEMORY_PROFILER_H */