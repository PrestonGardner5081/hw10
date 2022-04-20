struct queue
{
    int head;
    int tail;
    int size;
    int maxSize;
    char *contents;
};

void queuePush(char newChar, struct queue *s);
char queuePop(struct queue *s);
void initQueue(struct queue *s, int size);
void clearQueue(struct queue *s);