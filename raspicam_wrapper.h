/*
 * raspicam_wrapper.h
 *
 *  Created on: Mar 12, 2017
 *      Author: steveb
 */

#ifndef RASPICAM_WRAPPER_H_
#define RASPICAM_WRAPPER_H_

#include <stdbool.h>
#include <stddef.h>

struct raspicam_wrapper_handle
{
    void * Camera;
};

enum raspicam_wrapper_format
{
  RASPICAM_WRAPPER_FORMAT_YUV420,
  RASPICAM_WRAPPER_FORMAT_GRAY,
  RASPICAM_WRAPPER_FORMAT_BGR,
  RASPICAM_WRAPPER_FORMAT_RGB,
  RASPICAM_WRAPPER_FORMAT_IGNORE /* do not use */
};

struct raspicam_wrapper_handle * raspicam_wrapper_create( void );
bool raspicam_wrapper_open( struct raspicam_wrapper_handle * handle );
bool raspicam_wrapper_grab( struct raspicam_wrapper_handle * handle );
size_t raspicam_wrapper_getImageTypeSize( struct raspicam_wrapper_handle * handle, enum raspicam_wrapper_format format );
void raspicam_wrapper_retrieve( struct raspicam_wrapper_handle * handle, unsigned char * data, enum raspicam_wrapper_format format );
unsigned int raspicam_wrapper_getWidth( struct raspicam_wrapper_handle * handle );
unsigned int raspicam_wrapper_getHeight( struct raspicam_wrapper_handle * handle );
void raspicam_wrapper_destroy( struct raspicam_wrapper_handle * handle );

#endif /* RASPICAM_WRAPPER_H_ */
