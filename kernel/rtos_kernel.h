#ifndef AERORTOS_KERNEL_H
#define AERORTOS_KERNEL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <setjmp.h>
#include <windows.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============ Configuration ============ */
#define AERORTOS_CONFIG_MAX_TASKS             32
#define AERORTOS_CONFIG_MAX_SEMAPHORES        16
#define AERORTOS_CONFIG_MAX_MUTEXES           16
#define AERORTOS_CONFIG_MAX_QUEUES            16
#define AERORTOS_CONFIG_MAX_TIMERS            8
#define AERORTOS_CONFIG_TICK_RATE_HZ          1000
#define AERORTOS_CONFIG_STACK_SIZE_MIN        512
#define AERORTOS_CONFIG_STACK_SIZE_DEFAULT    4096
#define AERORTOS_CONFIG_IDLE_STACK_SIZE       4096
#define AERORTOS_CONFIG_PRIORITY_LEVELS       8
#define AERORTOS_CONFIG_TASK_NAME_LEN         32
#define AERORTOS_CONFIG_QUEUE_NAME_LEN        24
#define AERORTOS_CONFIG_MAX_DEADLINE_MISSES   100
#define AERORTOS_CONFIG_STATIC_BUFFER_SIZE    4096
#define AERORTOS_CONFIG_MAX_WAITING_NODES     128

/* ============ Task States ============ */
typedef enum {
    TASK_STATE_TERMINATED = 0,
    TASK_STATE_READY,
    TASK_STATE_RUNNING,
    TASK_STATE_BLOCKED,
    TASK_STATE_SUSPENDED,
    TASK_STATE_WAITING
} TaskState;

/* ============ Task Priorities ============ */
typedef enum {
    TASK_PRIORITY_IDLE = 0,
    TASK_PRIORITY_LOW,
    TASK_PRIORITY_NORMAL,
    TASK_PRIORITY_HIGH,
    TASK_PRIORITY_CRITICAL,
    TASK_PRIORITY_REAL_TIME,
    TASK_PRIORITY_ISR,
    TASK_PRIORITY_MAX
} TaskPriority;

/* ============ Error Codes ============ */
typedef enum {
    KERNEL_SUCCESS = 0,
    KERNEL_ERROR_INVALID_PARAM,
    KERNEL_ERROR_OUT_OF_MEMORY,
    KERNEL_ERROR_TIMEOUT,
    KERNEL_ERROR_TASK_NOT_FOUND,
    KERNEL_ERROR_RESOURCE_UNAVAILABLE,
    KERNEL_ERROR_RESOURCE_ALREADY_TAKEN,
    KERNEL_ERROR_ISR_CONTEXT,
    KERNEL_ERROR_QUEUE_FULL,
    KERNEL_ERROR_QUEUE_EMPTY,
    KERNEL_ERROR_TASK_SUSPENDED,
    KERNEL_ERROR_TASK_TERMINATED,
    KERNEL_ERROR_WAITING_NODE_EXHAUSTED,
    KERNEL_ERROR_MAX
} KernelError;

/* ============ Function Types ============ */
typedef void (*TaskFunction)(void *parameters);
typedef void (*TimerCallback)(void *args);
typedef void (*DeadlineCallback)(uint32_t task_id);

/* ============ Forward Declarations ============ */
struct TaskControlBlock;
struct SemaphoreControlBlock;
struct MutexControlBlock;
struct QueueControlBlock;
struct TimerControlBlock;

/* ============ Waiting Task Node ============ */
typedef struct WaitingTaskNode {
    struct TaskControlBlock *task;
    struct WaitingTaskNode *next;
    bool used;
} WaitingTaskNode;

/* ============ Task Control Block ============ */
typedef struct TaskControlBlock {
    uint32_t task_id;
    char name[AERORTOS_CONFIG_TASK_NAME_LEN];
    uint32_t tcb_index;
    
    TaskFunction entry_point;
    void *parameters;
    jmp_buf context;
    uint8_t *stack;
    uint32_t stack_size;
    
    TaskState state;
    TaskPriority priority;
    TaskPriority base_priority;
    bool in_ready_queue;
    bool is_suspended;
    bool first_run;
    bool is_idle;
    
    uint32_t wake_time;
    uint32_t execution_time;
    uint32_t deadline_ms;
    uint32_t deadline_misses;
    DeadlineCallback deadline_callback;
    
    uint32_t context_switches;
    uint32_t stack_peak_usage;
    uint32_t last_execution_time;
    uint32_t cpu_usage_percent;
    uint32_t last_cpu_update;
    
    struct TaskControlBlock *next;
    struct TaskControlBlock *prev;
    void *waiting_resource;
    
    uint32_t stack_canary_top;
    uint32_t stack_canary_bottom;
} TaskControlBlock;

/* ============ Semaphore Control Block ============ */
typedef struct SemaphoreControlBlock {
    uint32_t sem_id;
    char name[24];
    uint32_t count;
    uint32_t max_count;
    WaitingTaskNode *waiting_list;
    bool is_binary;
} SemaphoreControlBlock;

/* ============ Mutex Control Block ============ */
typedef struct MutexControlBlock {
    uint32_t mutex_id;
    char name[24];
    bool is_locked;
    TaskControlBlock *owner;
    TaskPriority original_priority;
    WaitingTaskNode *waiting_list;
    uint32_t lock_count;
} MutexControlBlock;

/* ============ Queue Control Block ============ */
typedef struct QueueControlBlock {
    uint32_t queue_id;
    char name[24];
    uint8_t buffer[AERORTOS_CONFIG_STATIC_BUFFER_SIZE];
    uint32_t item_size;
    uint32_t max_items;
    uint32_t item_count;
    uint32_t head;
    uint32_t tail;
    WaitingTaskNode *waiting_send_list;
    WaitingTaskNode *waiting_recv_list;
    bool is_isr_safe;
} QueueControlBlock;

/* ============ Timer Control Block ============ */
typedef struct TimerControlBlock {
    uint32_t timer_id;
    char name[24];
    TimerCallback callback;
    void *args;
    uint32_t period;
    uint32_t remaining;
    bool is_periodic;
    bool is_active;
} TimerControlBlock;

/* ============ Kernel State ============ */
typedef struct KernelState {
    TaskControlBlock *current_task;
    TaskControlBlock *ready_queue[AERORTOS_CONFIG_PRIORITY_LEVELS];
    TaskControlBlock *blocked_list;
    volatile uint32_t scheduler_lock_depth;
    volatile bool scheduler_running;
    
    TaskControlBlock task_pool[AERORTOS_CONFIG_MAX_TASKS];
    uint32_t task_counter;
    uint32_t active_task_count;
    uint8_t task_stacks[AERORTOS_CONFIG_MAX_TASKS][AERORTOS_CONFIG_STACK_SIZE_DEFAULT];
    
    SemaphoreControlBlock semaphores[AERORTOS_CONFIG_MAX_SEMAPHORES];
    MutexControlBlock mutexes[AERORTOS_CONFIG_MAX_MUTEXES];
    uint32_t semaphore_counter;
    uint32_t mutex_counter;
    
    QueueControlBlock queues[AERORTOS_CONFIG_MAX_QUEUES];
    uint32_t queue_counter;
    
    TimerControlBlock timers[AERORTOS_CONFIG_MAX_TIMERS];
    uint32_t timer_counter;
    
    volatile uint32_t system_tick;
    uint32_t context_switches;
    volatile uint32_t isr_nesting_depth;
    bool kernel_initialized;
    
    volatile int critical_owner;
    volatile int critical_depth;
    
    TaskControlBlock idle_task;
    uint8_t idle_stack[AERORTOS_CONFIG_IDLE_STACK_SIZE];
    
    WaitingTaskNode waiting_node_pool[AERORTOS_CONFIG_MAX_WAITING_NODES];
    uint32_t waiting_node_count;
    
    uint32_t idle_ticks;
    uint32_t total_ticks;
    uint32_t last_cpu_update;
    uint32_t tick_frequency;
    
    volatile bool timer_pending;
    volatile bool timer_initialized;
} KernelState;

/* ============ Global Kernel State ============ */
extern KernelState g_kernel;

/* ============ Kernel Core Functions ============ */
KernelError kernel_init(void);
KernelError kernel_start(void);
void kernel_schedule(void);
void kernel_tick_handler(void);
void kernel_lock_scheduler(void);
void kernel_unlock_scheduler(void);
uint32_t kernel_get_tick(void);
const char* kernel_get_error_string(KernelError error);

/* ============ Critical Section ============ */
void critical_enter(void);
void critical_exit(void);
bool critical_try_enter(void);

/* ============ Task Management ============ */
KernelError task_create(const char *name, TaskFunction entry, 
                        void *params, TaskPriority priority, 
                        uint32_t stack_size, uint32_t *task_id);
KernelError task_delete(uint32_t task_id);
KernelError task_suspend(uint32_t task_id);
KernelError task_resume(uint32_t task_id);
KernelError task_sleep(uint32_t ms);
KernelError task_yield(void);
uint32_t task_get_current_id(void);
const char* task_get_name(uint32_t task_id);
TaskState task_get_state(uint32_t task_id);
TaskPriority task_get_priority(uint32_t task_id);
KernelError task_set_priority(uint32_t task_id, TaskPriority priority);
void task_set_deadline(uint32_t task_id, uint32_t deadline_ms, DeadlineCallback callback);
uint32_t task_get_stack_usage(uint32_t task_id);
uint32_t task_get_cpu_usage(uint32_t task_id);
uint32_t task_get_context_switches(uint32_t task_id);

/* ============ Semaphore Management ============ */
KernelError semaphore_create(const char *name, uint32_t initial_count, 
                            uint32_t max_count, bool binary, uint32_t *sem_id);
KernelError semaphore_delete(uint32_t sem_id);
KernelError semaphore_take(uint32_t sem_id, uint32_t timeout_ms);
KernelError semaphore_give(uint32_t sem_id);
uint32_t semaphore_get_count(uint32_t sem_id);

/* ============ Mutex Management ============ */
KernelError mutex_create(const char *name, uint32_t *mutex_id);
KernelError mutex_delete(uint32_t mutex_id);
KernelError mutex_lock(uint32_t mutex_id, uint32_t timeout_ms);
KernelError mutex_unlock(uint32_t mutex_id);
bool mutex_is_owned(uint32_t mutex_id);

/* ============ Queue Management ============ */
KernelError queue_create(const char *name, uint32_t item_size, 
                         uint32_t max_items, bool isr_safe, uint32_t *queue_id);
KernelError queue_delete(uint32_t queue_id);
KernelError queue_send(uint32_t queue_id, const void *item, uint32_t timeout_ms);
KernelError queue_receive(uint32_t queue_id, void *item, uint32_t timeout_ms);
uint32_t queue_get_count(uint32_t queue_id);
uint32_t queue_get_space(uint32_t queue_id);

/* ============ Timer Management ============ */
KernelError timer_create(const char *name, uint32_t period_ms, 
                         bool periodic, TimerCallback callback, 
                         void *args, uint32_t *timer_id);
KernelError timer_delete(uint32_t timer_id);
KernelError timer_start(uint32_t timer_id);
KernelError timer_stop(uint32_t timer_id);
KernelError timer_reset(uint32_t timer_id);

/* ============ System Monitoring ============ */
uint32_t kernel_get_context_switches(void);
uint32_t kernel_get_task_count(void);
uint32_t kernel_get_active_task_count(void);
void kernel_print_status(void);
void kernel_print_cpu_usage(void);

/* ============ Port Functions ============ */
void rtos_port_init(void);
void rtos_port_start_scheduler(void);
void rtos_port_systick_init(void);
void rtos_port_disable_interrupts(void);
void rtos_port_enable_interrupts(void);
void rtos_port_process_timer(void);

/* ============ Extended Kernel Services ============ */

/* Event Flags */
typedef enum {
    EVENT_FLAG_SENSOR_DATA = 0x01,
    EVENT_FLAG_GPS_FIX = 0x02,
    EVENT_FLAG_MISSION_UPDATE = 0x04,
    EVENT_FLAG_FAILSAFE = 0x08,
    EVENT_FLAG_TELEMETRY = 0x10,
    EVENT_FLAG_WAYPOINT_REACHED = 0x20
} EventFlags;

/* Event Control Block */
typedef struct EventControlBlock {
    uint32_t event_id;
    char name[24];
    uint32_t flags;
    WaitingTaskNode *waiting_list;
    bool is_auto_clear;
} EventControlBlock;

/* Performance Monitor */
typedef struct {
    uint32_t task_id;
    uint32_t execution_time_min;
    uint32_t execution_time_max;
    uint32_t execution_time_avg;
    uint32_t sample_count;
    uint32_t deadline_misses_total;
    uint32_t stack_peak;
    float cpu_usage;
} TaskPerformance;

/* ============ New Function Prototypes ============ */

/* Event Management */
KernelError event_create(const char *name, bool auto_clear, uint32_t *event_id);
KernelError event_delete(uint32_t event_id);
KernelError event_set(uint32_t event_id, uint32_t flags);
KernelError event_clear(uint32_t event_id, uint32_t flags);
KernelError event_wait(uint32_t event_id, uint32_t flags, uint32_t timeout_ms, uint32_t *set_flags);

/* Performance Monitoring */
void perf_monitor_init(void);
void perf_monitor_task_start(uint32_t task_id);
void perf_monitor_task_end(uint32_t task_id);
TaskPerformance perf_monitor_get_stats(uint32_t task_id);
void perf_monitor_print_stats(void);


#ifdef __cplusplus
}
#endif

#endif /* AERORTOS_KERNEL_H */

