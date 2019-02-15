#include "queue_buf_cfg.h"
#include "queue_buf.h"

/*******************************************************************************
* get free size from queue buf
*******************************************************************************/
QUEUE_VAR_TYPE get_free_size_queue( queue_buf_t *q )
{
    if (q->in == q->out) 
        return q->len;                                                // queue buffer all free

    if (q->in > q->out){
        return q->len - q->in + q->out;
    } else {
        return q->out - q->in;
    }
}

/*******************************************************************************
* get data size from queue buf
*******************************************************************************/
QUEUE_VAR_TYPE get_data_size_queue( queue_buf_t *q )
{
    return q->len - get_free_size_queue( q );
}

/*******************************************************************************
* push data from rw_buf to queue buf
*******************************************************************************/
int push_data_queue( queue_buf_t *q )
{
    QUEUE_VAR_TYPE i;

    // test mesto est?
    if ( get_free_size_queue( q ) < q->rw_len ){                    // Error No Memory to queue for Write
        return 1;
    }

    for (i=0; i<q->rw_len; i++){
        *(q->queue + q->in) = *(q->rw_buf + i);
        (q->in)++;
        if ( q->in >= q->len ){
            q->in = 0;
        }
    }
    return 0;
}

/*******************************************************************************
* pop data from queue to rw_buf buf
*******************************************************************************/
int pop_data_queue( queue_buf_t *q )
{
    QUEUE_VAR_TYPE i;

    // test mesto est?
    if ( (q->len - get_free_size_queue( q )) < q->rw_len ){         // Error No pop data
        return 1;
    }

    for (i=0; i< q->rw_len; i++){
        *(q->rw_buf + i) = *(q->queue + q->out);
        (q->out)++;
        if ( q->out >= q->len ) q->out = 0;
    }
    return 0;
}

/*******************************************************************************
* pop data from queue to DMA buf
*******************************************************************************/
int pop_data_dma_queue( queue_buf_t *q, QUEUE_VAR_TYPE size )
{
    QUEUE_VAR_TYPE i;

    for (i=0; i<size; i++){
        (q->out)++;
        if ( q->out >= q->len ) q->out = 0;
    }
    return 0;
}

/*******************************************************************************
* push data to queue buf from 1 - element(byte,int...)
*******************************************************************************/
int push_data_queue_b( queue_buf_t *q, QUEUE_DATA_TYPE d )
{
    if ( get_free_size_queue( q ) <= 1 ){                           // test mesto est?
        return 1;
    }

    *(q->queue + q->in) = d;
    (q->in)++;

    if ( q->in >= q->len ){
        q->in = 0;
    }
    return 0;
}

/*******************************************************************************
* pop data from queue buf to 1 - element(byte,int...)
*******************************************************************************/
int pop_data_queue_b( queue_buf_t *q, QUEUE_DATA_TYPE *d )
{
    if ( get_data_size_queue( q ) == 0 ){                       // data est?
        return 1;
    }

    *d = *(q->queue + q->out);
    (q->out)++;

    if ( q->out == q->len ){ 
        q->out = 0;
    }
    return 0;
}

/*******************************************************************************
* READ (NO pop) data from queue buf to 1 - element(byte,int...)
*******************************************************************************/
int read_data_queue_b( queue_buf_t *q, QUEUE_DATA_TYPE *d )
{
    if ( get_data_size_queue( q ) == 0 ){                      // data est ?
        return 1;
    }

    *d = *(q->queue + q->out);

    return 0;
}

/*******************************************************************************
* RESET queue in=0 out=0
*******************************************************************************/
int reset_queue( queue_buf_t *queue )
{
    queue->out = 0;
    queue->in = 0;
    return 0;
}

