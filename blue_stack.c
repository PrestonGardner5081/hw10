#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include "blue_stack.h"

// add entry to blue / orange light qeue
void queuePush(char newChar, struct queue *s)
{
    if (s->size + 1 <= s->maxSize)
    {
        //tail is last element
        if (s->tail == s->maxSize - 1)
        {
            s->tail = 0;
        } //tail has not wrapped around
        else if (s->tail >= s->head)
        {
            s->tail = s->head + s->size;
        } //tail has wrapped around
        else
        {
            s->tail = s->head + s->size - s->maxSize;
        }
        s->contents[s->tail] = newChar;
        s->size += 1;
    }
    // printf("push: %c\n", s->contents[s->tail]); //FIXME
    // printf("push: %d\n", s->size); //FIXME
}

//remove head of qeue and return its value
char queuePop(struct queue *s)
{
    char popVal;

    if (s->size > 0)
    {
        popVal = s->contents[s->head];
        s->contents[s->head] = '\0';

        s->size -= 1;

        //if head was not the last in the
        if (s->size > 0)
        {
            //head is not last element in array, increase head index by 1
            if (s->head < s->maxSize - 1)
            {
                s->head += 1;
            }
            else
                s->head = 0;
        }
        else
        { //reset head / tail if array is empty
            s->head = 0;
            s->tail = 0;
        }
        // printf("pop: %c\n", popVal); //FIXME
    }
    else
        popVal = '\0';

    return popVal;
}

void initQueue(struct queue *s, int size)
{
    s->maxSize = size;
    s->head = 0;
    s->tail = 0;
    s->size = 0;

    return;
}

void clearQueue(struct queue *s)
{
    initQueue(s, s->maxSize);
    s->contents[0] = '\0';
}
