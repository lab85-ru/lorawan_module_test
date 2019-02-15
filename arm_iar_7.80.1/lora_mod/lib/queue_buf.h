#ifndef __QUEUE_BUF_H_
#define __QUEUE_BUF_H_

#include "queue_buf_cfg.h"                               // zadaem type for QUEUE_VAR_TYPE & QUEUE_DATA_TYPE

typedef struct queue_buf_type
{
    QUEUE_DATA_TYPE *queue;                              // ukazatel na buffer, kotoriy budet kolchevim
    QUEUE_VAR_TYPE len;                                  // dlinna buffera kolchevogo
    QUEUE_VAR_TYPE in;                                   // ukazatel dla zapisi danih
    QUEUE_VAR_TYPE out;                                  // ukazatel dla chteniya danih

    QUEUE_DATA_TYPE *rw_buf;                             // promrhutochniy buffer dla copurivaniya danih rw_buf <-> queue
    QUEUE_VAR_TYPE rw_len;                               // dlinna dannih v rw_buf
} queue_buf_t;


QUEUE_VAR_TYPE get_free_size_queue( queue_buf_t *q );    // return svobodnoe mesto v queue buf
QUEUE_VAR_TYPE get_data_size_queue( queue_buf_t *q );    // return size data to queue buf
int push_data_queue( queue_buf_t *q );                   // function push raboti s massivom
int pop_data_queue( queue_buf_t *q );                    // function pop raboti s massivom
int pop_data_dma_queue( queue_buf_t *q, QUEUE_VAR_TYPE size );          // Dla dma, counter out=out+size

int push_data_queue_b( queue_buf_t *q, QUEUE_DATA_TYPE d );       // function raboti s 1 byte
int pop_data_queue_b( queue_buf_t *q, QUEUE_DATA_TYPE *d );       // function raboti s 1 byte
int read_data_queue_b( queue_buf_t *q, QUEUE_DATA_TYPE *d );      // chitaem 1 byte bez izvlechenia
int reset_queue( queue_buf_t *q );                       // reset queue in+out=0

#endif
