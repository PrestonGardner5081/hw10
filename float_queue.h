struct fQueue
{
    int head;
    int tail;
    int size;
    int maxSize;
    float *contents;
};

void fQueuePush(float newFloat, struct fQueue *s);
float fQueuePop(struct fQueue *s);
void initFQueue(struct fQueue *s, int size);
void clearFQueue(struct fQueue *s);