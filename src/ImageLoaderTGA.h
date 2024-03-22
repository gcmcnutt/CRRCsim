/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2004, 2007 Jan Reucker (original author)
 * Copyright (C) 2008 Jens Wilhelm Wulf
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
  

/** \class CImageLoaderTGA
 *
 *  \brief A simple TGA library.
 *
 *  This is a very simple TGA lib. It will only load and save 
 *  images in greyscale, RGB or RGBA mode. RLE encoding is only
 *  supported for loading images.
 *  
 *  If you want a more complete lib I suggest you take 
 *  a look at Paul Groves' TGA loader. Paul's home page is at 
 *  
 *  http://paulyg.virtualave.net
 *  
 *  Just a little bit about the TGA file format.
 *  
 *  ------------------
 *  Header - 12 fields
 *  ------------------
 *  
 *  - id                      (unsigned char)
 *  - color map type          (unsigned char)
 *  - image type              (unsigned char)
 *    - 0 = 0b0000 0000 - no image data included
 *    - 1 = 0b0000 0001 - color map image
 *    - 2 = 0b0000 0010 - RGB(A) uncompressed (*)
 *    - 3 = 0b0000 0011 - greyscale uncompressed (*)
 *    - 9 = 0b0000 1001 - color map RLE (compressed)
 *    - 10 = 0b0000 1010 - RGB(A) RLE (compressed) (*)
 *    - 11 = 0b0000 1011 - greyscale RLE (compressed) (*)
 *    - 32 = 0b0010 0000 - color map Huffman, Delta, RLE (compressed)
 *    - 33 = 0b0010 0001 - color map Huffman, Delta, RLE with 4-pass
 *                       quadtree-type processing (compressed)
 *    .
 *    Note: This library only supports the image types marked with a (*).
 *  - colour map first entry  (short int)
 *  - colour map length       (short int)
 *  - map entry size          (short int)
 *  - horizontal origin       (short int)
 *  - vertical origin         (short int)
 *  - width                   (short int)
 *  - height                  (short int)
 *  - pixel depth             (unsigned char)
 *    - 8  -  greyscale
 *    - 24  -  RGB
 *    - 32  -  RGBA
 *  - image descriptor        (unsigned char)
 *  
 *  From all these fields, we only care about the image type, 
 *  to check if the image is uncompressed and not color indexed, 
 *  the width and height, and the pixel depth.
 *  
 *  You may use this library for whatever you want. This library is 
 *  provide as is, meaning that I won't take any responsability for
 *  any damages that you may incur for its usage.

 *  Antonio Ramires Fernandes ajbrf@yahoo.com

 *  Changes by Jan Reucker:
 *  - converted to a C++ class
 *  - added RLE support (at least for loading).
 *  - corrected and updated documentation, including Doxygen support
 *    (http://www.doxygen.org)

 *  Credits:
 *  - Lionel Cailler for MacOS-X support
 *  - wotsit.org for providing information about the TGA file format
 */

#ifndef IMAGE_LOADER_TGA_H
#define IMAGE_LOADER_TGA_H

#include <stdio.h>

#define TGA_ERROR_FILE_OPEN       (-5)
#define TGA_ERROR_READING_FILE    (-4)
#define TGA_ERROR_INDEXED_COLOR   (-3)
#define TGA_ERROR_MEMORY          (-2)
#define TGA_ERROR_UNSUPPORTED     (-1)
#define TGA_OK                    (0)

#define TGA_TYPE_COLOR_MAP                    (0x01)
#define TGA_TYPE_RGBA_UNCOMPRESSED            (0x02)
#define TGA_TYPE_GREYSCALE_UNCOMPRESSED       (0x03)
#define TGA_TYPE_COLOR_MAP_RLE_COMPRESSED     (0x09)
#define TGA_TYPE_RGBA_RLE_COMPRESSED          (0x0A)
#define TGA_TYPE_GREYSCALE_RLE_COMPRESSED     (0x0B)
#define TGA_TYPE_COLOR_MAP_HUFF_COMPRESSED    (0x20)
#define TGA_TYPE_COLOR_MAP_HUFF4P_COMPRESSED  (0x21)

#define TGA_TYPE_RLE_COMPRESSION_BIT          (0x08)

class CImageLoaderTGA
{
  public:
    int status;                 /**< indicates status of previous operations */
    unsigned char type;         /**< TGA image type */
    unsigned char pixelDepth;   /**< number of bits per pixel: 8 (GS), 24 (RGB), 32 (RGBA) */
    short int width;            /**< Image width in pixels */
    short int height;           /**< Image height in pixels */
    unsigned char *imageData;   /**< Pointer to actual image data */

    CImageLoaderTGA(const char *filename);
    ~CImageLoaderTGA();
    
    int Save (char          *filename, 
              short int     width, 
              short int     height, 
              unsigned char pixelDepth, 
              unsigned char *imageData);

    void toGreyscale();
    int GrabScreenSeries(char *filename, int x,int y, int w, int h);

  private:
    FILE *file;
    // this variable is used for image series
    int savedImages;
    
    int Load(char *filename);
    void LoadHeader();
    void LoadImageData();
    int SaveSeries (char          *filename, 
                    short int      width, 
                    short int      height, 
                    unsigned char  pixelDepth,
                    unsigned char  *imageData);
};

  
#endif // IMAGE_LOADER_TGA_H
