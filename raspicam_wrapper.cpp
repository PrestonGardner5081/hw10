/*
 * raspicam_wrapper.cpp
 *
 *  Created on: Mar 12, 2017
 *      Author: steveb
 */

extern "C"
{
#include "raspicam_wrapper.h"
#include <stdlib.h>
}

#include <ctime>
#include <fstream>
#include <iostream>
#include <raspicam/raspicam.h>

static raspicam::RASPICAM_FORMAT raspicam_wrapper_convert_format_type( raspicam_wrapper_format format )
{
  raspicam::RASPICAM_FORMAT return_value;

  switch (format)
  {
    case RASPICAM_WRAPPER_FORMAT_YUV420:  return_value = raspicam::RASPICAM_FORMAT_YUV420;  break;
    case RASPICAM_WRAPPER_FORMAT_GRAY:    return_value = raspicam::RASPICAM_FORMAT_GRAY;    break;
    case RASPICAM_WRAPPER_FORMAT_BGR:     return_value = raspicam::RASPICAM_FORMAT_BGR;     break;
    case RASPICAM_WRAPPER_FORMAT_IGNORE:  return_value = raspicam::RASPICAM_FORMAT_IGNORE;  break;
    case RASPICAM_WRAPPER_FORMAT_RGB:
    default:                              return_value = raspicam::RASPICAM_FORMAT_RGB;     break;
  }

  return return_value;
}

// allocate the raspicam
struct raspicam_wrapper_handle * raspicam_wrapper_create( void )
{
    void *                    handle_pointer; // used to validate the result of malloc
    raspicam_wrapper_handle * handle;         // the created handle

    handle_pointer = malloc( sizeof(*handle) );
    if (handle_pointer != NULL)
    {
      handle = reinterpret_cast<raspicam_wrapper_handle *>(handle_pointer);
      handle->Camera = new raspicam::RaspiCam;
      if (handle->Camera != NULL)
      {
        ; // allocation was successful (for my part of it, at least)
      }
      else
      {
        free( handle );
        handle = NULL;
      }
    }
    else
    {
      ; // handle allocation failed
    }

    return handle;
}

// open the camera
bool raspicam_wrapper_open( struct raspicam_wrapper_handle * handle )
{
  bool return_value;

  if (handle != NULL)
  {
    return_value = reinterpret_cast<raspicam::RaspiCam *>(handle->Camera)->open();
  }
  else
  {
    return_value = false;
  }

  return return_value;
}

// grab an image frame
bool raspicam_wrapper_grab( struct raspicam_wrapper_handle * handle )
{
  bool return_value;

  if (handle != NULL)
  {
    return_value = reinterpret_cast<raspicam::RaspiCam *>(handle->Camera)->grab();
  }
  else
  {
    return_value = false;
  }

  return return_value;
}

// return the size of the obtained image
size_t raspicam_wrapper_getImageTypeSize( struct raspicam_wrapper_handle * handle, enum raspicam_wrapper_format format )
{
  size_t return_value;

  if (handle != NULL)
  {
    return_value = reinterpret_cast<raspicam::RaspiCam *>(handle->Camera)->getImageTypeSize( raspicam_wrapper_convert_format_type( format ) );
  }
  else
  {
    return_value = 0;
  }

  return return_value;
}

// copy the image data
void raspicam_wrapper_retrieve( struct raspicam_wrapper_handle * handle, unsigned char * data, enum raspicam_wrapper_format format )
{
  if (handle != NULL)
  {
    reinterpret_cast<raspicam::RaspiCam *>(handle->Camera)->retrieve( data, raspicam_wrapper_convert_format_type( format ) );
  }
  else
  {
    ; // nothing to be done, there is no valid handle
  }

  return;
}

// get the width of the image
unsigned int raspicam_wrapper_getWidth( struct raspicam_wrapper_handle * handle )
{
  unsigned int return_value;

  if (handle != NULL)
  {
    return_value = reinterpret_cast<raspicam::RaspiCam *>(handle->Camera)->getWidth();
  }
  else
  {
    return_value = 0;
  }

  return return_value;
}

// get the height of the image
unsigned int raspicam_wrapper_getHeight( struct raspicam_wrapper_handle * handle )
{
  unsigned int return_value;

  if (handle != NULL)
  {
    return_value = reinterpret_cast<raspicam::RaspiCam *>(handle->Camera)->getHeight();
  }
  else
  {
    return_value = 0;
  }

  return return_value;
}

// clean up when done
void raspicam_wrapper_destroy( struct raspicam_wrapper_handle * handle )
{
  if (handle != NULL)
  {
    delete reinterpret_cast<raspicam::RaspiCam *>(handle->Camera);
    free( handle );
  }
  else
  {
    ; // nothing to be done, there is no valid handle
  }

  return;
}
