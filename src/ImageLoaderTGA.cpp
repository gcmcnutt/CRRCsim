/*
 * CRRCsim - the Charles River Radio Control Club Flight Simulator Project
 *
 * Copyright (C) 2004, 2007, 2008 Jan Reucker (original author)
 * Copyright (C) 2004 Lionel Cailler
 * Copyright (C) 2005, 2008 Jens Wilhelm Wulf
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
  

// Jan's TODO list:
// - add some overflow checks instead of simply relying on
//   ints being big enough for the image (maybe introduce
//   a max_image_size?)
// - error handling for fread() (unexpected end-of-file...)
// - care about variable-length-fields (image descriptor...)
// - remove GL stuff (not really needed here)
// - convert all I/O operations to C++ streams to remove
//   unnecessary C header files
// - test image saving, especially on MACOSX (byte order!)

#include "include_gl.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ImageLoaderTGA.h"

/** \brief Load the image header fields.
 *
 *  We only keep those that matter!
 */
void CImageLoaderTGA::LoadHeader() 
{
  unsigned char cGarbage;
  short int iGarbage;
  int cnt;

  cnt = fread(&cGarbage, sizeof(unsigned char), 1, file);
  cnt = fread(&cGarbage, sizeof(unsigned char), 1, file);

// type must be 2 or 3
  cnt = fread(&type, sizeof(unsigned char), 1, file);

  cnt = fread(&iGarbage, sizeof(short int), 1, file);
  cnt = fread(&iGarbage, sizeof(short int), 1, file);
  cnt = fread(&cGarbage, sizeof(unsigned char), 1, file);
  cnt = fread(&iGarbage, sizeof(short int), 1, file);
  cnt = fread(&iGarbage, sizeof(short int), 1, file);

  cnt = fread(&width, sizeof(short int), 1, file);
  cnt = fread(&height, sizeof(short int), 1, file);
  cnt = fread(&pixelDepth, sizeof(unsigned char), 1, file);

  cnt = fread(&cGarbage, sizeof(unsigned char), 1, file);
  if (cnt != 1)
    status = TGA_ERROR_READING_FILE;

  #if BYTE_ORDER == BIG_ENDIAN
  width    = (width  << 8) + (width  >> 8);
  height   = (height << 8) + (height >> 8);
  #endif  // BYTE_ORDER == BIG_ENDIAN
}

/** \brief Load the actual pixel data.
 *
 *  Loads the pixel data stored in the file and decompresses
 *  it if necessary.
 */
void CImageLoaderTGA::LoadImageData()
{
  int mode, i;
  int total;
  unsigned char aux;
  int cnt = 0;

  // mode equal the number of components for each pixel
  mode = pixelDepth / 8;
  // total is the number of bytes (uncompressed)
  total = height * width * mode;

  // decide whether or not to use run-length-encoding
  if (type & TGA_TYPE_RLE_COMPRESSION_BIT)
  {
    // image is RLE-compressed, we'll have to decompress the data first
    int BytesDecoded = 0;
    int ChunkSize;
    unsigned char RLHeader;
    unsigned char pixel[4];

    while (BytesDecoded < total)
    {
      // read header of next chunk
      cnt = fread(&RLHeader, sizeof(unsigned char), 1, file);
      ChunkSize = (RLHeader & 0x7F) + 1;  // packet contains 1 ... 128 pixels

      // each chunk is either a RAW packet or a set of identical pixels
      if (RLHeader & 0x80)
      {
        // RLE packet, repeat next pixel ChunkSize times
        int i, k;
        cnt = fread(pixel, sizeof(unsigned char), mode, file);
        for (i = 0; i < ChunkSize; i++)
        {
          for (k = 0; k < mode; k++)
          {
            imageData[BytesDecoded] = pixel[k];
            BytesDecoded++;
          }
        }
      }
      else
      {
        // RAW packet, contains ChunkSize un-encoded pixels
        cnt = fread(&imageData[BytesDecoded], sizeof(unsigned char),
                    ChunkSize*mode, file);
        BytesDecoded += ChunkSize * mode;
      }
    }
  }
  else
  {
    // no RLE, just read the pixels
    cnt = fread(imageData,sizeof(unsigned char),total,file);
  }

  // todo: this check is not sufficient in case of real errors. But there has been no check for years...
  if (cnt == 0)
    status = TGA_ERROR_READING_FILE;
  
  // mode=3 or 4 implies that the image is RGB(A). However TGA
  // stores it as BGR(A) so we'll have to swap R and B.
  if (mode >= 3)
  {
    for (i = 0; i < total; i += mode) 
    {
      aux = imageData[i];
      imageData[i] = imageData[i+2];
      imageData[i+2] = aux;
    }
  }
}  

/** \brief Load image data from file.
 *
 *  The constructor tries to open the specified file and
 *  reads the image header. If it describes a supported
 *  TGA type (2, 3, 10, 11), the image data will be
 *  loaded into memory and decompressed if needed.
 *  The "status" member will be set according to the
 *  result of all operations.
 */
CImageLoaderTGA::CImageLoaderTGA(const char *filename)
{
  int mode;
  int total;

  savedImages = 0;

// open the file for reading (binary mode)
  file = fopen(filename, "rb");
  if (file == NULL) {
    status = TGA_ERROR_FILE_OPEN;
    return;
  }

// load the header
  LoadHeader();

// check for errors when loading the header
  if (ferror(file)) {
    status = TGA_ERROR_READING_FILE;
    fclose(file);
    return;
  }

  switch (type)
  {
    case TGA_TYPE_COLOR_MAP:
    case TGA_TYPE_COLOR_MAP_RLE_COMPRESSED:
      status = TGA_ERROR_INDEXED_COLOR;
      fclose(file);
      return;

    case TGA_TYPE_RGBA_UNCOMPRESSED:
    case TGA_TYPE_GREYSCALE_UNCOMPRESSED:
    case TGA_TYPE_RGBA_RLE_COMPRESSED:
    case TGA_TYPE_GREYSCALE_RLE_COMPRESSED:
      break;
    
    default:
      status = TGA_ERROR_UNSUPPORTED;
      fclose(file);
      return;
  }

  // mode equals the number of image components
  mode = pixelDepth / 8;
  
  // total is the number of bytes to read
  total = height * width * mode;
  
  // allocate memory for image pixels
  imageData = (unsigned char *)malloc(sizeof(unsigned char) * total);

  // check to make sure we have the memory required
  if (imageData == NULL) 
  {
    status = TGA_ERROR_MEMORY;
    fclose(file);
    return;
  }

  // finally load the image pixels
  LoadImageData();

  // check for errors when reading the pixels
  if (ferror(file)) 
  {
    status = TGA_ERROR_READING_FILE;
    fclose(file);
    return;
  }
  fclose(file);
  status = TGA_OK;
}    


/** \brief Convert image data to greyscale image.
 *
 *  This method converts the previously loaded image data
 *  to a greyscale image by combining the RGB values
 *  to a single grey value using the formula
 *  grey = 0.30 * R + 0.59 * G + 0.11 * B
 */
void CImageLoaderTGA::toGreyscale() 
{
  int mode,i,j;

  unsigned char *newImageData;

  // if the image is already greyscale do nothing
  if (pixelDepth == 8)
  {
    return;
  }

  // compute the number of actual components
  mode = pixelDepth / 8;

  // allocate an array for the new image data
  newImageData = (unsigned char *)malloc(sizeof(unsigned char) * 
                                          height * width);
  if (newImageData == NULL)
  {
    return;
  }

  // convert pixels: greyscale = 0.30 * R + 0.59 * G + 0.11 * B
  for (i = 0,j = 0; j < width * height; i +=mode, j++)
  {
    newImageData[j] =  (unsigned char)(0.30 * imageData[i] + 
            0.59 * imageData[i+1] +
            0.11 * imageData[i+2]);
  }

  //free old image data
  free(imageData);

  // reassign pixelDepth and type according to the new image type
  pixelDepth = 8;
  type = 3;
  // reassing imageData to the new array.
  imageData = newImageData;
}

/** \brief (deprecated) Take a screenshot from a GL framebuffer and save it.
 *
 *  This method reads the pixel data from the current OpenGL
 *  framebuffer and saves it as a TGA image.
 *
 *  DO NOT USE THIS METHOD, WILL BE REMOVED IN FUTURE VERSIONS
 *  OF THIS LIB!
 */

int CImageLoaderTGA::GrabScreenSeries(char *filename, int x,int y, int w, int h) 
{
  unsigned char *imageData;

  // allocate memory for the pixels
  imageData = (unsigned char *)malloc(sizeof(unsigned char) * w * h * 4);

  // read the pixels from the frame buffer
  glReadPixels(x,y,w,h,GL_RGBA,GL_UNSIGNED_BYTE, (GLvoid *)imageData);

  // save the image 
  return(SaveSeries(filename,w,h,32,imageData));
}

/** \brief Save an array of pixels as a TGA image.
 *
 *  Comment by JR: Untested, use at your own risk!
 *
 *  \param filename Image name
 *  \param width    Image width
 *  \param height   Image height
 *  \param pixelDepth Number of bits per pixel
 *  \param imageData Pointer to pixel data
 *
 *  \return Status of operation (see #defines in header file)
 */
int CImageLoaderTGA::Save(char          *filename, 
                          short int     width, 
                          short int     height, 
                          unsigned char pixelDepth,
                          unsigned char *imageData) 
{
  unsigned char cGarbage = 0, type,mode,aux;
  short int iGarbage = 0;
  int i;
  FILE *file;

  // open file and check for errors
  file = fopen(filename, "wb");
  if (file == NULL)
  {
    return(TGA_ERROR_FILE_OPEN);
  }

  // compute image type: 2 for RGB(A), 3 for greyscale
  mode = pixelDepth / 8;
  
  switch (pixelDepth)
  {
    case 24:
    case 32:
      type = TGA_TYPE_RGBA_UNCOMPRESSED;
      break;
    
    default:
      type = TGA_TYPE_GREYSCALE_UNCOMPRESSED;
      break;
  }

  // write the header
  (void)fwrite(&cGarbage, sizeof(unsigned char), 1, file);
  (void)fwrite(&cGarbage, sizeof(unsigned char), 1, file);

  (void)fwrite(&type, sizeof(unsigned char), 1, file);

  (void)fwrite(&iGarbage, sizeof(short int), 1, file);
  (void)fwrite(&iGarbage, sizeof(short int), 1, file);
  (void)fwrite(&cGarbage, sizeof(unsigned char), 1, file);
  (void)fwrite(&iGarbage, sizeof(short int), 1, file);
  (void)fwrite(&iGarbage, sizeof(short int), 1, file);

  (void)fwrite(&width, sizeof(short int), 1, file);
  (void)fwrite(&height, sizeof(short int), 1, file);
  (void)fwrite(&pixelDepth, sizeof(unsigned char), 1, file);

  (void)fwrite(&cGarbage, sizeof(unsigned char), 1, file);

  // convert the image data from RGB(a) to BGR(A)
  if (mode >= 3)
  {
    for (i=0; i < width * height * mode ; i+= mode) 
    {
      aux = imageData[i];
      imageData[i] = imageData[i+2];
      imageData[i+2] = aux;
    }
  }

  // save the image data
  (void)fwrite(imageData, sizeof(unsigned char), width * height * mode, file);
  fclose(file);
  // release the memory
  free(imageData);

  return(TGA_OK);
}

/** \brief saves a series of files with names "filenameX.tga"
 *
 *  Comment by JR: Untested, use at your own risk!
 */
int CImageLoaderTGA::SaveSeries( char    *filename, 
                                 short int    width, 
                                 short int    height, 
                                 unsigned char  pixelDepth,
                                 unsigned char  *imageData) 
{
  char *newFilename;
  int status;
  // compute the new filename by adding the series number and the extension

  newFilename = (char *)malloc(sizeof(char) * strlen(filename)+8);

  sprintf(newFilename,"%s%d.tga",filename,savedImages);
  // save the image
  status = Save(newFilename,width,height,pixelDepth,imageData);
  //increase the counter
  savedImages++;
  return(status);
}


/** \brief Release the memory used for the image.
 *
 *
 */
CImageLoaderTGA::~CImageLoaderTGA()
{
  free(imageData);
}
