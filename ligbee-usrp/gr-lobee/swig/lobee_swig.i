/* -*- c++ -*- */

#define LOBEE_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "lobee_swig_doc.i"

%{
#include "lobee/quad_to_byte_fb.h"
#include "lobee/lobee_cpp_bp.h"
%}


%include "lobee/quad_to_byte_fb.h"
GR_SWIG_BLOCK_MAGIC2(lobee, quad_to_byte_fb);
%include "lobee/lobee_cpp_bp.h"
GR_SWIG_BLOCK_MAGIC2(lobee, lobee_cpp_bp);
