/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 *   Copyright (C) 2009 Joel Lienard (original author)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/*****
 add  textures  type "JPEG" to ssg
call  	ssgAddTextureFormat ( ".jpg",ssgLoadJPG)  before use
************/

#include <iostream>
#include <plib/ssg.h>
#define XMD_H	//for not redefine INT32 in jpeglib.h
extern "C"
{
#include <jpeglib.h>
}

namespace Video
{

bool ssgLoadJPG ( const char *fname, ssgTextureInfo* info )
{
  FILE * infile;
  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr jerr;

  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_decompress(&cinfo);

  if ((infile = fopen(fname, "rb")) == NULL)
  {
    fprintf(stderr, "can't open %s\n", fname);
    return false ;
  }
  jpeg_stdio_src(&cinfo, infile);
  jpeg_read_header(&cinfo, TRUE);
  jpeg_start_decompress(&cinfo);
  JDIMENSION w = cinfo.output_width;
  JDIMENSION h = cinfo.output_height;
  JDIMENSION z = cinfo.output_components;
  GLubyte *image;
  image	= new GLubyte [ w * h * z ] ;
  JSAMPROW row_pointer[1];	/* pointer to a single row */
  int row_stride;			/* physical row width in buffer */
  row_stride = w * z;	/* JSAMPLEs per row in image_buffer */
  //std::cout <<"ssgLoadJPG " <<fname<<" "<<cinfo.image_height<<" x "<<cinfo.image_width<<std::endl;
#if 1
  while (cinfo.output_scanline < h)
  {
    row_pointer[0] = & image[(h -1 -cinfo.output_scanline) * row_stride];
    jpeg_read_scanlines(&cinfo, row_pointer, 1 );
  }
#else
//litle faster
  JSAMPROW row_pointer2[h];
  for (int i=0; i< h; i++)
  {
    row_pointer2[i] = & image[(h -1 -i) * row_stride];
  }
  while (cinfo.output_scanline < h)
  {
    jpeg_read_scanlines(&cinfo, & row_pointer2[cinfo.output_scanline], h );
  }
#endif
  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);
  fclose(infile);
  
  if ( info != NULL )
  {
    info -> width = w ;
    info -> height = h ;
    info -> depth = z ;
    info -> alpha = 0;
  }

  return ssgMakeMipMaps ( image, w, h, z ) ;
}

} // end namespace Video::
