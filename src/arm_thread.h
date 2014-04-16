typedef struct __arm_thread_struct {
    // "persistent" data for this "lthread" would go here
    int data;
} arm_thread_struct;

int arm_lthread(arm_thread_struct *,int,int,unsigned char*);
