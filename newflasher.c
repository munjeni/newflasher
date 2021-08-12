/*
 * Copyright (C) 2017 Munjeni
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "version.h"

#if (!defined(_WIN32)) && (!defined(WIN32))
	#ifndef __USE_FILE_OFFSET64
		#define __USE_FILE_OFFSET64 1
	#endif
	#ifndef __USE_LARGEFILE64
		#define __USE_LARGEFILE64 1
	#endif
	#ifndef _LARGEFILE64_SOURCE
		#define _LARGEFILE64_SOURCE 1
	#endif
	#ifndef _FILE_OFFSET_BITS
		#define _FILE_OFFSET_BITS 64
	#endif
	#ifndef _FILE_OFFSET_BIT
		#define _FILE_OFFSET_BIT 64
	#endif
#endif

#ifdef _WIN32
#define __USE_MINGW_ANSI_STDIO 1

#include <windows.h>
#include <setupapi.h>
#include <initguid.h>

#include "GordonGate.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#ifdef HAS_STDINT_H
	#include <stdint.h>
#endif
#include <stdbool.h>

#ifdef unix
	#include <unistd.h>
	#include <sys/types.h>
#endif

#ifdef _WIN32
	#include <direct.h>
	#include <io.h>
#endif

#if defined(USE_FILE32API)
	#define fopen64 fopen
	#define ftello64 ftell
	#define fseeko64 fseek
#else
	#ifdef __FreeBSD__
		#define fopen64 fopen
		#define ftello64 ftello
		#define fseeko64 fseeko
	#endif
	/*#ifdef __ANDROID__
		#define fopen64 fopen
		#define ftello64 ftello
		#define fseeko64 fseeko
	#endif*/
	#ifdef _MSC_VER
		#define fopen64 fopen
		#if (_MSC_VER >= 1400) && (!(defined(NO_MSCVER_FILE64_FUNC)))
			#define ftello64 _ftelli64
			#define fseeko64 _fseeki64
		#else  /* old msc */
			#define ftello64 ftell
			#define fseeko64 fseek
		#endif
	#endif
#endif

#include <ctype.h>
#include <sys/stat.h>
#include <limits.h>
#include <time.h>
#include <dirent.h>
#include <assert.h>

#if !defined(_WIN32) && !defined(__APPLE__)
#include <linux/usbdevice_fs.h>
#include <linux/usb/ch9.h>
#include <asm/byteorder.h>

#include <string.h>
#include <errno.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

#include <dirent.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <ctype.h>
#endif

#ifdef __APPLE__
	#include <unistd.h>
	//#include <CoreFoundation/CoreFoundation.h>
	//#include <IOKit/IOKitLib.h>
	//#include <IOKit/IOCFPlugIn.h>
	//#include <IOKit/usb/IOUSBLib.h>
	//#include <mach/mach.h>

	#include "libusb.h"

	#define fseeko64 fseeko
	#define ftello64 ftello
	#define fopen64 fopen
#endif

#include "expat.h"
#include "zlib.h"

#ifdef XML_LARGE_SIZE
#if defined(XML_USE_MSC_EXTENSIONS) && _MSC_VER < 1400
#define XML_FMT_INT_MOD "I64"
#else
#define XML_FMT_INT_MOD "ll"
#endif
#else
#define XML_FMT_INT_MOD "l"
#endif

#ifdef _WIN32
#define sleep Sleep
#define ONESEC 1000
#else
#define ONESEC 1
#endif

#define ENABLE_DEBUG 1

#if ENABLE_DEBUG
#define LOG printf
#else
#define LOG(...)
#endif

#define BUFF_MAX 0x800000
#define MAX_UNIT_LINE_LEN 0x80000

static char tmp[4096];
static char tmp_reply[BUFF_MAX];
static unsigned long get_reply_len;
static bool okay_replied = false;
static char product[12];
static char version[64];
static char version_bootloader[64];
static char version_baseband[64];
static char serialno[64];
static char secure[32];
static unsigned int sector_size = 0;
static unsigned int max_download_size = 0;
static char loader_version[64];
static char phone_id[64];
static char device_id[64];
static char rooting_status[32];
static char ufs_info[64];
static char emmc_info[64];
static bool have_ufs = false;
static char default_security[16];
static char platform_id[64];
static unsigned int keystore_counter = 0;
static char security_state[128];
static char s1_root[64];
static char sake_root[16];
static char get_root_key_hash[0x41];
static char slot_count[2];
static char current_slot[2];
static char remember_current_slot[2];

static unsigned int something_flashed = 0;

static bool is_2021_device = false;

static unsigned int battery_level = 0;

unsigned int swap_uint32(unsigned int val) {
	val = ((val << 8) & 0xFF00FF00 ) | ((val >> 8) & 0xFF00FF);
	return ((val << 16) | (val >> 16)) & 0xffffffff;
}

unsigned long long swap_uint64(unsigned long long val) {
	val = ((val << 8) & 0xFF00FF00FF00FF00ULL) | ((val >> 8) & 0x00FF00FF00FF00FFULL);
	val = ((val << 16) & 0xFFFF0000FFFF0000ULL) | ((val >> 16) & 0x0000FFFF0000FFFFULL);
	return ((val << 32) | (val >> 32)) & 0xffffffffffffffffULL;
}

void fread_unus_res(void *ptr, size_t size, size_t nmemb, FILE *stream) {
	size_t in;
	in = fread(ptr, size, nmemb, stream);
	if (in) {
		/* satisfy warn unused result */
	}
}

unsigned int file_size(char *filename) {
	unsigned int size;

	FILE *fp = fopen(filename, "rb");

	if (fp == NULL) {
		return 0;
	}

	fseek(fp, 0, SEEK_END);
	size = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	fclose(fp);
	return size;
}

static int file_exist(char *file) {
	int ret;
	FILE *f = NULL;

	if ((f = fopen64(file, "rb")) == NULL) {
		ret = 0;
	} else {
		fclose(f);
		ret = 1;
	}
	return ret;
}

static void remove_file_exist(char *file) {
	if (file_exist(file)) {
		remove(file);
	}
}

static char *basenamee(char *in) {
	char *ssc;
	int p = 0;
	ssc = strstr(in, "/");
	if (ssc == NULL) {
		ssc = strstr(in, "\\");
		if(ssc == NULL) {
		  	return in;
		}
	}
	do {
		p = strlen(ssc) + 1;
		in = &in[strlen(in)-p+2];
		ssc = strstr(in, "/");
		if (ssc == NULL)
			ssc = strstr(in, "\\");
	} while(ssc);

	return in;
}

static ssize_t g_getline(char **lineptr, size_t *n, FILE *stream) {
	char *cur_pos, *new_lineptr;
	int c;
	size_t new_lineptr_len;

	if (lineptr == NULL || n == NULL || stream == NULL) {
		errno = EINVAL;
		printf("Error: EINVAL!\n");
		return -1;
	}

	if (*lineptr == NULL) {
		*n = MAX_UNIT_LINE_LEN;
		if ((*lineptr = (char *)malloc(*n)) == NULL) {
			errno = ENOMEM;
			printf("Error: MAX_UNIT_LINE_LEN reached!\n");
			return -1;
		}
	}

	cur_pos = *lineptr;
	for (;;) {
		c = getc(stream);

		if (ferror(stream) || (c == EOF && cur_pos == *lineptr))
			return -1;

		if (c == EOF)
			break;

		if ((*lineptr + *n - cur_pos) < 2) {
			if (SSIZE_MAX / 2 < *n) {
#ifdef EOVERFLOW
				errno = EOVERFLOW;
#else
				errno = ERANGE; /* no EOVERFLOW defined */
#endif
			printf("Error: EOVERFLOW!\n");
			return -1;
		}
		new_lineptr_len = *n * 2;

		if ((new_lineptr = (char *)realloc(*lineptr, new_lineptr_len)) == NULL) {
			errno = ENOMEM;
			printf("Error: ENOMEM for realloc!\n");
			return -1;
		}
		*lineptr = new_lineptr;
		*n = new_lineptr_len;
	}

	*cur_pos++ = c;

	if (c == '\r' || c == '\n')
		break;
	}

	*cur_pos = '\0';
	return (ssize_t)(cur_pos - *lineptr);
}

static void trim(char *ptr) {
	int i = 0;
	int j = 0;

	while(ptr[j] != '\0') {
		if(ptr[j] == 0x20 || ptr[j] == 0x09 || ptr[j] == '\n' || ptr[j] == '\r') {
			++j;
			ptr[i] = ptr[j];
		} else {
			ptr[i] = ptr[j];
			++i;
			++j;
		}
	}
	ptr[i] = '\0';
}

// 2 minute
#define USB_TIMEOUT 120000

#ifndef _WIN32
static char *TEXT(char *what) {
	return what;
}

void DisplayError(char *title)
{
	printf("%s\n%s\n", title, strerror(errno));
}
#else
#define StringCchPrintf(str, n, format, ...) snprintf((char *)str, n, (char const *)format, __VA_ARGS__)
void DisplayError(LPTSTR lpszFunction)
{
	LPVOID lpMsgBuf;
	LPVOID lpDisplayBuf;
	DWORD dw = GetLastError();

	FormatMessage(
		FORMAT_MESSAGE_ALLOCATE_BUFFER |
		FORMAT_MESSAGE_FROM_SYSTEM |
		FORMAT_MESSAGE_IGNORE_INSERTS,
		NULL,
		dw,
		MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
		(LPTSTR) &lpMsgBuf,
		0,
		NULL );

	lpDisplayBuf =
			(LPVOID)LocalAlloc( LMEM_ZEROINIT,
			          ( lstrlen((LPCTSTR)lpMsgBuf)
			            + lstrlen((LPCTSTR)lpszFunction)
			            + 40) /* account for format string */
			          * sizeof(TCHAR) );

	if (FAILED( StringCchPrintf((LPTSTR)lpDisplayBuf,
			LocalSize(lpDisplayBuf) / sizeof(TCHAR),
			TEXT("%s failed with error code %lu as follows:\n%s"),
			lpszFunction,
			dw,
			(char *)lpMsgBuf)))
	{
		printf("FATAL ERROR: Unable to output error code.\n");
	}

	printf("ERROR: %s\n", (LPCTSTR)lpDisplayBuf);

	LocalFree(lpMsgBuf);
	LocalFree(lpDisplayBuf);
}
#endif

#ifdef _WIN32
static char *uint16_to_vidpidstring(unsigned short VID, unsigned short PID)
{
	static char temp[18];

	snprintf(temp, sizeof(temp), "vid_%x%x%x%x&pid_%x%x%x%x",
		 (VID>>12)&0xf, (VID>>8)&0xf, (VID>>4)&0xf, VID&0xf,
		 (PID>>12)&0xf, (PID>>8)&0xf, (PID>>4)&0xf, PID&0xf);

	return temp;
}
#endif

static void to_ascii(char *dest, const char *text) {
	unsigned long int ch;
	for(; sscanf((const char *)text, "%02lx", &ch)==1; text+=2)
		*dest++ = ch;
	*dest = 0;
}

static void to_uppercase(char *ptr) {
	for ( ; *ptr; ++ptr) *ptr = toupper(*ptr);
}

static void display_buffer_hex_ascii(char *message, char *buffer, unsigned int size) {
	unsigned int i, j, k;

	LOG("%s[0x%X]:\n", message, size);

	for (i=0; i<size; i+=16) {
		LOG("\n  %08X  ", i);
		for(j=0,k=0; k<16; j++,k++) {
			if (i+j < size) {
				LOG("%02X", buffer[i+j] & 0xff);
			} else {
				LOG("  ");
			}
			LOG(" ");
		}
		LOG(" ");
		for(j=0,k=0; k<16; j++,k++) {
			if (i+j < size) {
				if ((buffer[i+j] < 32) || (buffer[i+j] > 126)) {
					LOG(".");
				} else {
					LOG("%c", buffer[i+j]);
				}
			}
		}
	}
	LOG("\n\n" );
}

#define EP_IN 0
#define EP_OUT 1

#ifdef _WIN32
static GUID GUID_DEVINTERFACE_USB_DEVICE = {0xA5DCBF10L, 0x6530, 0x11D2, {0x90, 0x1F, 0x00, 0xC0, 0x4F, 0xB9, 0x51, 0xED}};

HDEVINFO	hDevInfo;
#else
#ifdef __APPLE__
typedef struct libusb_device_handle *HANDLE;
#define CloseHandle libusb_close
#define SetupDiDestroyDeviceInfoList(...)
int32_t OSAtomicDecrement32Barrier(volatile int32_t *__theValue) { return 0; }
int32_t OSAtomicIncrement32Barrier(volatile int32_t *__theValue) { return 0; }
int endpoint_in = 0x81, endpoint_out = 0x01;    // default IN and OUT endpoints
#else
#define SetupDiDestroyDeviceInfoList(...)

/* The max bulk size for linux is 16384 which is defined
 * in drivers/usb/core/devio.c.
 */
#define MAX_USBFS_BULK_SIZE 4096
/*(16 * 1024)*/

struct usb_handle
{
	char fname[64];
	int desc;
	unsigned char ep_in;
	unsigned char ep_out;
};

typedef struct usb_handle *HANDLE;

static inline int badname(const char *name)
{
	while (*name) {
		if (!isdigit(*name++))
			return 1;
	}
	return 0;
}

static int get_vidpid(int fd, unsigned short VID, unsigned short PID)
{
	struct usb_device_descriptor *dev;
	char desc[1024];
	int n;

	if ((n = read(fd, desc, sizeof(desc))) == 0)
		return 0;
	dev = (void *)desc;
	/*printf("found vid: %04x\n", dev->idVendor);
	printf("found pid: %04x\n", dev->idProduct);*/

	if (dev->idVendor != VID || dev->idProduct != PID)
		return 0;

	return 1;
}

struct usb_handle *get_flashmode(unsigned short VID, unsigned short PID)
{
	char busname[64], devname[64];
	DIR *busdir, *devdir;
	struct dirent *de;

	int fd;
	int found_usb = 0;
	int n;
	int ifc;

	struct usb_handle *usb = NULL;

	busdir = opendir("/dev/bus/usb");
	if (busdir == NULL) {
		printf("Error, no /dev/bus/usb ! Please connect device first in flash mode!\n");
		return usb;
	}

	/*printf("busdir: %p\n", busdir);*/

	while ((de = readdir(busdir)) && (found_usb == 0)) {
		/*printf("dirent: %p\n", de);*/
		if (badname(de->d_name))
			continue;
		snprintf(busname, sizeof(busname), "%s/%s", "/dev/bus/usb", de->d_name);
		/*printf("busname: %s\n", busname);*/

		devdir = opendir(busname);

		while ((de = readdir(devdir)) && (found_usb == 0)) {
			if (badname(de->d_name))
				continue;
			snprintf(devname, sizeof(devname), "%s/%s", busname, de->d_name);
			/*printf("devname: %s\n", devname);*/

			if ((fd = open(devname, O_RDWR)) < 1) {
				printf("cannot open %s for writing\n", devname);
				continue;
			}

			if (get_vidpid(fd, VID, PID)) {
				printf("found device with vid:0x%04x pid:0x%04x.\n", VID, PID);

				usb = calloc(1, sizeof(struct usb_handle));

				usb->ep_in = 0x81;
				usb->ep_out = 0x01;
				usb->desc = fd;

				ifc = 0;
				if ((n = ioctl(fd, USBDEVFS_CLAIMINTERFACE, &ifc)) != 0) {
					printf("ERROR: n = %d, errno = %d (%s)\n",
						 n, errno, strerror(errno));
					closedir(devdir);
					closedir(busdir);
					return NULL;
				}
				found_usb = 1;
			}
		}
		closedir(devdir);
	}
	closedir(busdir);
	return usb;
}

int usb_close(struct usb_handle *h)
{
	int fd;

	fd = h->desc;
	h->desc = -1;
	if (fd >= 0) {
		close(fd);
		/*printf("usb closed %d\n", fd);*/
	}

	return 0;
}

#define CloseHandle usb_close
#endif
#endif

#ifdef _WIN32
static char *open_dev(unsigned short VID, unsigned short PID)
{
	SP_DEVICE_INTERFACE_DATA         DevIntfData;
	PSP_DEVICE_INTERFACE_DETAIL_DATA DevIntfDetailData;
	SP_DEVINFO_DATA                  DevData;

	unsigned long dwSize, dwMemberIdx;
	static char devicePath[MAX_PATH];
	char szDescription[MAX_PATH];

	int ret = 1;
	char *vidpid = uint16_to_vidpidstring(VID, PID);

	memset(devicePath, 0, sizeof(devicePath));

	hDevInfo = SetupDiGetClassDevs(&GUID_DEVINTERFACE_USB_DEVICE, NULL, 0, DIGCF_DEVICEINTERFACE | DIGCF_PRESENT);

	if (hDevInfo != INVALID_HANDLE_VALUE)
	{
		DevIntfData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);
		dwMemberIdx = 0;

		SetupDiEnumDeviceInterfaces(hDevInfo, NULL, &GUID_DEVINTERFACE_USB_DEVICE, dwMemberIdx, &DevIntfData);

		while(GetLastError() != ERROR_NO_MORE_ITEMS)
		{
			DevData.cbSize = sizeof(DevData);

			SetupDiGetDeviceInterfaceDetail(hDevInfo, &DevIntfData, NULL, 0, &dwSize, NULL);

			DevIntfDetailData = HeapAlloc(GetProcessHeap(), HEAP_ZERO_MEMORY, dwSize);
			DevIntfDetailData->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);

			if (SetupDiGetDeviceInterfaceDetail(hDevInfo, &DevIntfData, DevIntfDetailData, dwSize, &dwSize, &DevData))
			{
				if (strstr(DevIntfDetailData->DevicePath, vidpid) != NULL)
				{
					strncpy(devicePath, DevIntfDetailData->DevicePath, strlen(DevIntfDetailData->DevicePath));
					printf("Device path: %s\n", devicePath);
					memset(szDescription, 0, MAX_PATH);
					SetupDiGetClassDescription(&DevData.ClassGuid, szDescription, MAX_PATH, &dwSize);
					printf("Class Description: %s\n", szDescription);

					memset(szDescription, 0, MAX_PATH);
					SetupDiGetDeviceInstanceId(hDevInfo, &DevData, szDescription, MAX_PATH, 0);
					printf("Device Instance Id: %s\n\n", szDescription);

					ret = 0;
				}
			}

			HeapFree(GetProcessHeap(), 0, DevIntfDetailData);

			/* Continue looping */
			SetupDiEnumDeviceInterfaces(hDevInfo, NULL, &GUID_DEVINTERFACE_USB_DEVICE, ++dwMemberIdx, &DevIntfData);

			if (ret == 0)
				break;
		}

		if (ret)
			SetupDiDestroyDeviceInfoList(hDevInfo);
	}

	return devicePath;
}

static unsigned long transfer_bulk_async(HANDLE dev, int ep, char *bytes, unsigned long size, int timeout, int exact)
{
	static unsigned long nBytesRead = 0;
	BOOL bResult;
	unsigned char progress = 0;
	time_t start = time(NULL);

	OVERLAPPED gOverLapped_in = {
		.Internal     = 0,
		.InternalHigh = 0,
		.Offset       = 0,
		.OffsetHigh   = 0
	};

	OVERLAPPED gOverLapped_out = {
		.Internal     = 0,
		.InternalHigh = 0,
		.Offset       = 0,
		.OffsetHigh   = 0
	};

	if (ep == EP_IN)
	{
		nBytesRead = 0;
		gOverLapped_in.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

		if (NULL == gOverLapped_in.hEvent)
		{
			DisplayError(TEXT("Error creating overlaped_in hEvent!"));
			return 0;
		}
		else
		{
			bResult = ReadFile(dev, bytes, size, NULL, &gOverLapped_in);

			if (!bResult)
			{
				switch(GetLastError())
				{
					case ERROR_HANDLE_EOF:
					{
						/* we have reached the end of the file during the call to ReadFile */
						DisplayError(TEXT("HANDLE_EOF:"));
						break;
					}
					case ERROR_IO_PENDING:
					{
						/* asynchronous i/o is still in progress */
						switch(WaitForSingleObject(gOverLapped_in.hEvent, timeout))
						{
							case WAIT_OBJECT_0:
								printf("\n");

								/* check on the results of the asynchronous read and update the nBytesRead... */
								do
								{
									// https://docs.microsoft.com/en-us/windows/win32/api/ioapiset/nf-ioapiset-getoverlappedresult
									bResult = GetOverlappedResult(dev, &gOverLapped_in, &nBytesRead, FALSE);

									if (time(NULL)-start > 5)
									{
										start = time(NULL);

										progress += 1;
										printf(".");

										if (progress == 60)
										{
											progress = 0;
											printf("\n");
										}
									}
								}
								while(!bResult && ERROR_IO_INCOMPLETE == GetLastError());

								if (!bResult)
									DisplayError(TEXT("GetOverlapped_in_Result:"));
								break;

							case WAIT_TIMEOUT:
								DisplayError(TEXT("TIMEOUT:"));
								CancelIo(dev);
								break;

							default:
								DisplayError(TEXT("ERROR_IO_PENDING OTHER:"));
								CancelIo(dev);
								break;
						}
					}
					case ERROR_SUCCESS:
					{
						//printf("io read succed.\n");
						CancelIo(dev);
						break;
					}
					default:
					{
						DisplayError(TEXT("IO_OTHER:"));
						CancelIo(dev);
						break;
					}
				}
			}

			ResetEvent(gOverLapped_in.hEvent);
			CloseHandle(gOverLapped_in.hEvent);
		}
	}

	if (ep == EP_OUT)
	{
		nBytesRead = 0;
		gOverLapped_out.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
		if (NULL == gOverLapped_out.hEvent) {
			DisplayError(TEXT("Error creating overlaped_in hEvent!"));
			return 0;
		}
		else
		{
			bResult = WriteFile(dev, bytes, size, NULL, &gOverLapped_out);

			if(!bResult)
			{
				switch (GetLastError())
				{
					case ERROR_HANDLE_EOF:
					{
						/* we have reached the end of the file during the call to ReadFile */
						DisplayError(TEXT("HANDLE_EOF:"));
						break;
					}
					case ERROR_IO_PENDING:
					{
						/* asynchronous i/o is still in progress */
						switch(WaitForSingleObject(gOverLapped_out.hEvent, timeout))
						{
							case WAIT_OBJECT_0:
								printf("\n");

								/* check on the results of the asynchronous read and update the nBytesRead... */
								do
								{
									// https://docs.microsoft.com/en-us/windows/win32/api/ioapiset/nf-ioapiset-getoverlappedresult
									bResult = GetOverlappedResult(dev, &gOverLapped_out, &nBytesRead, FALSE);

									if (time(NULL)-start > 5)
									{
										start = time(NULL);

										progress += 1;
										printf(".");

										if (progress == 60)
										{
											progress = 0;
											printf("\n");
										}
									}
								}
								while(!bResult && ERROR_IO_INCOMPLETE == GetLastError());

								if (!bResult)
									DisplayError(TEXT("GetOverLapped_out_Result:"));
								break;

							case WAIT_TIMEOUT:
								DisplayError(TEXT("TIMEOUT:"));
								CancelIo(dev);
								break;

							default:
								DisplayError(TEXT("ERROR_IO_PENDING OTHER:"));
								CancelIo(dev);
								break;
						}
					}
					case ERROR_SUCCESS:
					{
						//printf("io write succed.\n");
						CancelIo(dev);
						break;
					}
					default:
					{
						DisplayError(TEXT("IO_OTHER:"));
						CancelIo(dev);
						break;
					}
				}
			}

			ResetEvent(gOverLapped_out.hEvent);
			CloseHandle(gOverLapped_out.hEvent);
		}
	}

	if (exact) {
		if (nBytesRead != size) {
			printf(" - Error %s! Need nBytes: 0x%lx but done: 0x%lx\n", (ep == EP_IN) ? "read" : "write", size, nBytesRead);
			display_buffer_hex_ascii("nBytes", bytes, nBytesRead);
			return 0;
		}
	}
#if 0
	if (ep == EP_IN && nBytesRead) {
		printf(" - Successfully read 0x%lx bytes from handle.\n", nBytesRead);
		display_buffer_hex_ascii("Raw input ", bytes, nBytesRead);
	}

	if (ep == EP_OUT && nBytesRead) {
		printf(" - Successfully write 0x%lx bytes to handle.\n", nBytesRead);
		//display_buffer_hex_ascii("Raw output ", bytes, nBytesRead);
	}
#endif
	return nBytesRead;
}
#else
#ifdef __APPLE__

static unsigned long transfer_bulk_async(HANDLE dev, int ep, char *bytes, unsigned long size, int timeout, int exact)
{
	int res = 0;
	int try = 0;
	int actual_length = 0;

	if (ep == EP_IN)
	{
		do {
			res = libusb_bulk_transfer(dev, endpoint_in, (unsigned char *)bytes, size, &actual_length, timeout);
			if (res == LIBUSB_ERROR_PIPE)
			{
				int halt = libusb_clear_halt(dev, endpoint_in);
				if (halt != LIBUSB_SUCCESS)
				{
					printf("clear halt (in): %s\n", libusb_error_name(halt));
				}
				else
				{
					printf("halt clear after: %d\n", try);
				}
			}
			try++;
		} while ((res == LIBUSB_ERROR_PIPE) && (try < 3));

		if (res != LIBUSB_SUCCESS)
		{
			printf("bulk transfer (in): %s\n", libusb_error_name(res));
			return 0;
		}
	}

	if (ep == EP_OUT)
	{
		do {
			res = libusb_bulk_transfer(dev, endpoint_out, (unsigned char *)bytes, size, &actual_length, timeout);
			if (res == LIBUSB_ERROR_PIPE)
			{
				int halt = libusb_clear_halt(dev, endpoint_out);
				if (halt != LIBUSB_SUCCESS)
				{
					printf("clear halt (out): %s\n", libusb_error_name(halt));
				}
				else
				{
					printf("halt clear after: %d\n", try);
				}
			}
			try++;
		} while ((res == LIBUSB_ERROR_PIPE) && (try < 3));

		if (res != LIBUSB_SUCCESS)
		{
				printf("bulk transfer (out): %s\n", libusb_error_name(res));
				return 0;
		}
	}

	if (exact)
	{
		if ((unsigned long)actual_length != size)
		{
			printf(" - Error %s! Need nBytes: 0x%lx but done: 0x%lx\n", (ep == EP_IN) ? "read" : "write", size, (unsigned long)actual_length);
			display_buffer_hex_ascii("nBytes", bytes, actual_length);
			return 0;
		}
	}

	return (unsigned long)actual_length;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#else
static unsigned long transfer_bulk_async(struct usb_handle *h, int ep, const void *_bytes, unsigned long size, int timeout, int exact)
{
	char *bytes = (char *)_bytes;
	unsigned long count = 0;
	unsigned long size_tot = size;
	struct usbdevfs_bulktransfer bulk;
	int n = 0;

	if (ep == EP_IN)
	{
		if (h->ep_in == 0)
		{
			printf(" - ep_in is not 0x81!!!\n");
			return 0;
		}

		while (size > 0)
		{
			int xfer = (size > MAX_USBFS_BULK_SIZE) ? MAX_USBFS_BULK_SIZE : size;

			bulk.ep = h->ep_in;
			bulk.len = xfer;
			bulk.data = bytes;
			bulk.timeout = timeout;

			do
			{
				n = ioctl(h->desc, USBDEVFS_BULK, &bulk);
				if (n < 0)
				{
					printf(" - (ep_in) ERROR: n = %d, errno = %d (%s)\n",n, errno, strerror(errno));
					return 0;
				}
			}
			while(n < 0);

			count += n;
			size -= n;
			bytes += n;

			if (n < xfer)
				break;
		}
	}

	if (ep == EP_OUT)
	{
		if (h->ep_out == 0)
		{
			printf(" - ep_out is not 0x01!!!\n");
			return 0;
		}

		if (size == 0)
		{
			bulk.ep = h->ep_out;
			bulk.len = 0;
			bulk.data = bytes;
			bulk.timeout = 0;

			n = ioctl(h->desc, USBDEVFS_BULK, &bulk);
			if (n != 0)
			{
				printf(" - (ep_out size=0)ERROR: n = %d, errno = %d (%s)\n", n, errno, strerror(errno));
				return 0;
			}
			return 0;
		}

		while (size > 0)
		{
			int xfer = (size > MAX_USBFS_BULK_SIZE) ? MAX_USBFS_BULK_SIZE : size;

			bulk.ep = h->ep_out;
			bulk.len = xfer;
			bulk.data = bytes;
			bulk.timeout = timeout;

			n = ioctl(h->desc, USBDEVFS_BULK, &bulk);
			if (n != xfer)
			{
				printf(" - (ep_out size=%d)ERROR: n = %d, errno = %d (%s)\n", xfer, n, errno, strerror(errno));
				return 0;
			}

			count += xfer;
			size -= xfer;
			bytes += xfer;
		}
	}

	if (exact)
	{
		if (count != size_tot)
		{
			printf(" - Error %s! Need nBytes: 0x%lx but done: 0x%lx\n", (ep == EP_IN) ? "read" : "write", size_tot, count);
			display_buffer_hex_ascii("nBytes", bytes, count);
			return 0;
		}
	}
#if 0
	if (ep == EP_IN)
	{
		printf(" - Successfully read 0x%lx bytes from handle.\n", count);
		display_buffer_hex_ascii("Raw input ", bytes, count);
	}

	if (ep == EP_OUT)
	{
		printf(" - Successfully write 0x%lx bytes to handle.\n", count);
		/*display_buffer_hex_ascii("Raw output ", bytes, count);*/
	}
#endif
	return count;
}
#endif
#endif

static bool get_reply(HANDLE dev, int ep, char *bytes, unsigned long size, int timeout, int exact)
{
	unsigned long ret_len = 0;
	get_reply_len = 0;

	ret_len = transfer_bulk_async(dev, ep, bytes, size, timeout, exact);
	/*display_buffer_hex_ascii("Replied with ", bytes, ret_len);*/

	if (!ret_len)
	{
		//printf(" - reply: null!\n");
		return false;
	}

	if (ret_len >= 4)
	{

		if ((memcmp(bytes, "OKAY", 4) == 0 || memcmp(bytes, "FAIL", 4) == 0) && ret_len == 4) {
			memcpy(tmp_reply, bytes, ret_len);
			tmp_reply[ret_len] = '\0';
			get_reply_len = ret_len;
			return true;
		}

		if (memcmp(bytes, "OKAY", 4) == 0 && ret_len > 4)
		{
			memcpy(tmp_reply, bytes+4, ret_len-4);
			tmp_reply[ret_len-4] = '\0';
			get_reply_len = ret_len-4;
			return true;
		}

		if (memcmp(bytes, "FAIL", 4) == 0 && ret_len > 4) {
			memcpy(tmp_reply, bytes, ret_len);
			tmp_reply[ret_len] = '\0';
			get_reply_len = ret_len;
			return true;
		}

		if (memcmp(bytes, "DATA", 4) == 0 && ret_len != 12)
		{
			// xperia 10 mark 3 XQ-BT41 send 13 bytes where last byte is null termination, fixing it to 12
			if (ret_len == 13)
			{
				ret_len = 12;
				goto solve_xqbt41;
			}

			printf(" - Errornous DATA reply!\n");
			display_buffer_hex_ascii("Replied with ", bytes, ret_len);
			return false;
		}

solve_xqbt41:
		if (memcmp(bytes, "DATA", 4) == 0 && ret_len == 12)
		{
			memcpy(tmp_reply, bytes, ret_len);
			tmp_reply[ret_len] = '\0';
			get_reply_len = ret_len;
			return true;
		}
	}

	memcpy(tmp_reply, bytes, ret_len);
	tmp_reply[ret_len] = '\0';
	get_reply_len = ret_len;

	return true;
}

static int check_valid_unit(char *in) {
	int i, ret=0;

	if (strlen(in) < 8)
		return ret;

	for (i=0; i<8; ++i) {
		if ((in[i] >= '0' && in[i] <= '9') || (in[i] >= 'A' && in[i] <= 'Z') || (in[i] >= 'a' && in[i] <= 'z'))
			ret += 1;
	}

	if (ret == 8)
		return 1;
	else
		return 0;
}

#define CHUNK 16384

/* These are parameters to deflateInit2. See
   http://zlib.net/manual.html for the exact meanings. */

#define windowBits 15
#define GZIP_ENCODING 16

#if 0
/* Compress from file source to file dest until EOF on source.
   def() returns Z_OK on success, Z_MEM_ERROR if memory could not be
   allocated for processing, Z_STREAM_ERROR if an invalid compression
   level is supplied, Z_VERSION_ERROR if the version of zlib.h and the
   version of the library linked do not match, or Z_ERRNO if there is
   an error reading or writing the files. */
int def(FILE *source, FILE *dest, int level)
{
	int ret, flush;
	unsigned have;
	z_stream strm;
	unsigned char in[CHUNK];
	unsigned char out[CHUNK];

	/* allocate deflate state */
	strm.zalloc = Z_NULL;
	strm.zfree = Z_NULL;
	strm.opaque = Z_NULL;
	ret = deflateInit2(&strm, Z_DEFAULT_COMPRESSION, Z_DEFLATED, windowBits | GZIP_ENCODING, 8, Z_DEFAULT_STRATEGY);
	if (ret != Z_OK)
		return ret;

	/* compress until end of file */
	do {
		strm.avail_in = fread(in, 1, CHUNK, source);
		if (ferror(source)) {
			(void)deflateEnd(&strm);
			return Z_ERRNO;
		}
		flush = feof(source) ? Z_FINISH : Z_NO_FLUSH;
		strm.next_in = in;

		/* run deflate() on input until output buffer not full, finish
		   compression if all of source has been read in */
		do {
			strm.avail_out = CHUNK;
			strm.next_out = out;
			ret = deflate(&strm, flush);    /* no bad return value */
			assert(ret != Z_STREAM_ERROR);  /* state not clobbered */
			have = CHUNK - strm.avail_out;
			if (fwrite(out, 1, have, dest) != have || ferror(dest)) {
 				(void)deflateEnd(&strm);
				return Z_ERRNO;
			}
		} while (strm.avail_out == 0);
		assert(strm.avail_in == 0);     /* all input will be used */

		/* done when last data in file processed */
	} while (flush != Z_FINISH);
	assert(ret == Z_STREAM_END);        /* stream will be complete */

	/* clean up and return */
	(void)deflateEnd(&strm);
	return Z_OK;
}
#endif

/* Decompress from file source to file dest until stream ends or EOF.
   inf() returns Z_OK on success, Z_MEM_ERROR if memory could not be
   allocated for processing, Z_DATA_ERROR if the deflate data is
   invalid or incomplete, Z_VERSION_ERROR if the version of zlib.h and
   the version of the library linked do not match, or Z_ERRNO if there
   is an error reading or writing the files. */
int inf(FILE *source, FILE *dest)
{
	int ret;
	unsigned char progress = 0;
	unsigned long long have;
	z_stream strm;
	unsigned char in[CHUNK];
	unsigned char out[CHUNK];

	/* allocate inflate state */
	strm.zalloc = Z_NULL;
	strm.zfree = Z_NULL;
	strm.opaque = Z_NULL;
	strm.avail_in = 0;
	strm.next_in = Z_NULL;
	ret = inflateInit2(&strm, 47);      /* automatic zlib or gzip decoding */
	if (ret != Z_OK)
	    return ret;

	printf("      ");

	/* decompress until deflate stream ends or end of file */
	do {
		strm.avail_in = fread(in, 1, CHUNK, source);
		if (ferror(source)) {
			(void)inflateEnd(&strm);
			return Z_ERRNO;
		}
		if (strm.avail_in == 0)
			break;
		strm.next_in = in;

	    /* run inflate() on input until output buffer not full */
	    do {
			strm.avail_out = CHUNK;
			strm.next_out = out;
			ret = inflate(&strm, Z_NO_FLUSH);
			assert(ret != Z_STREAM_ERROR);  /* state not clobbered */
			switch (ret) {
				case Z_NEED_DICT:
					ret = Z_DATA_ERROR;     /* and fall through */
					break;
				case Z_DATA_ERROR:
				case Z_MEM_ERROR:
					(void)inflateEnd(&strm);
					DisplayError(TEXT("assert(ret != Z_STREAM_ERROR)"));
					return ret;
				default:
					break;
			}
			have = CHUNK - strm.avail_out;
			if ((have % 4294967296ULL) == 0)
			{
				progress += 1;
				printf(".");
				if (progress == 60) {
					progress = 0;
					printf("\n      ");
				}
			}
			if (fwrite(out, 1, have, dest) != have || ferror(dest)) {
				(void)inflateEnd(&strm);
				DisplayError(TEXT("Z_ERRNO"));
				return Z_ERRNO;
			}
	    } while (strm.avail_out == 0);

	    /* done when inflate() says it's done */
	} while (ret != Z_STREAM_END);

	/* clean up and return */
	(void)inflateEnd(&strm);
	printf("\n");
	return ret == Z_STREAM_END ? Z_OK : Z_DATA_ERROR;
}

/* report a zlib or i/o error */
void zerr(int ret)
{
	fputs(" - gzpipe: ", stderr);
	switch (ret)
	{
		case Z_ERRNO:
			if (ferror(stdin))
				fputs("error reading stdin!\n", stderr);
			if (ferror(stdout))
				fputs("error writing stdout!\n", stderr);
			break;
		case Z_STREAM_ERROR:
			fputs("invalid compression level!\n", stderr);
			break;
		case Z_DATA_ERROR:
			fputs("invalid or incomplete deflate data!\n", stderr);
			break;
		case Z_MEM_ERROR:
			fputs("out of memory!\n", stderr);
			break;
		case Z_VERSION_ERROR:
			fputs("zlib version mismatch!\n", stderr);
			break;
		default:
			fputs("ok.\n", stderr);
			break;
	}
}

#if 0
static int gziper(char *in, char *out)
{
		int ret;
		FILE *source = NULL;
		FILE *zip = NULL;
		FILE *zipped = NULL;
		FILE *back = NULL;

		if ((source = fopen64(in, "rb")) == NULL) {
			printf(" - Could not open %s!\n", in);
			return 1;
		}
		if ((zip = fopen64(out, "wb")) == NULL) {
			printf(" - Could not open %s for write!\n", out);
			if (source) fclose(source);
			return 1;
		}

		printf(" - defflating...\n");
		ret = def(source, zip, Z_DEFAULT_COMPRESSION);
		printf(" - defflate returned: %i\n", ret);
		if (source) fclose(source);
		if (zip) fclose(zip);

		if (ret == 0)
		{
			printf (" - setting up inflate...\n");

			if ((zipped = fopen64(out, "rb")) == NULL) {
				printf(" - Could not open %s for verification!\n", out);
				return 1;
			}

			if ((back = fopen64("tempFcheck", "wb")) == NULL) {
				printf(" - Could not open for write temFcheck file for verification!\n");
				if (zipped) fclose(zipped);
				return 1;
			}

			printf (" - infflating, please wait...\n");
			ret = inf(zipped, back);
			printf(" - inflate returned: %i\n", ret);
			zerr(ret);
		}

		if (zipped) fclose(zipped);
		if (back) fclose(back);

		if (ret != 0) {
			remove_file_exist(out);
			return 1;
		}

		remove_file_exist("tempFcheck");
		printf("  - gzip ok.\n");
		return 0;
}
#endif

static int gunziper(char *in, char *out)
{
		int ret;
		FILE *zipped = NULL;
		FILE *back = NULL;

		printf (" - setting up inflate...\n");
		if ((zipped = fopen64(in, "rb")) == NULL) {
			printf(" - Could not open %s for infflating!\n", in);
			return 1;
		}
		if ((back = fopen64(out, "wb")) == NULL) {
			printf(" - Could not open %s for write!\n", out);
			if (zipped) fclose(zipped);
			return 1;
		}
		printf (" - infflating, please wait...\n");
		ret = inf(zipped, back);
		printf(" - inflate returned: %i\n", ret);
		zerr(ret);

		if (zipped) fclose(zipped);
		if (back) fclose(back);

		if (ret != 0) {
			remove_file_exist(out);
			return 1;
		}

		printf(" - gunziped ok.\n");
		return 0;
}

/* Parse an octal number, ignoring leading and trailing nonsense. */
static int parseoct(const char *p, size_t n)
{
	int i = 0;

	while (*p < '0' || *p > '7') {
		++p;
		--n;
	}
	while (*p >= '0' && *p <= '7' && n > 0) {
		i *= 8;
		i += *p - '0';
		++p;
		--n;
	}
	return (i);
}

/* Returns true if this is 512 zero bytes. */
static int is_end_of_archive(const char *p)
{
	int n;
	for (n = 511; n >= 0; --n)
	{
		if (p[n] != '\0')
		{
			return 0;
		}
	}
	return 1;
}

/* Create a file, including parent directory as necessary. */
static FILE *create_file(char *pathname)
{
	FILE *f = fopen64(pathname, "wb");

	if (f == NULL)
		return NULL;
	else
		return (f);
}

/* Verify the tar checksum. */
static int verify_checksum(const char *p)
{
	int n, u = 0;
	for (n = 0; n < 512; ++n) {
		if (n < 148 || n > 155)
			/* Standard tar checksum adds unsigned bytes. */
			u += ((unsigned char *)p)[n];
		else
			u += 0x20;

	}
	return (u == parseoct(p + 148, 8));
}

static int process_sins(HANDLE dev, FILE *a, char *filename, char *full_path, char *outfolder, char *endcommand)
{
	char buff[512];
	FILE *f = NULL;
	size_t bytes_read;
	int filesize;
	int i=0;
	char tmpp[256];
	char tmpg[256];
	char command[64];
	char flashfile[256];
	bool has_slot = false;

	printf(" - Extracting from %s\n", basenamee(filename));

	for (;;)
	{
		int chunk = 0;
		bytes_read = fread(buff, 1, 512, a);

		if (bytes_read != 512) {
			printf(" - Short read on %s: expected 512, got %d\n", filename, (int)bytes_read);
			return 0;
		}

		if (is_end_of_archive(buff))
		{
			printf(" - End of %s\n", basenamee(filename));
			return 1;
		}

		if (!verify_checksum(buff)) {
			printf(" - Checksum failure\n");
			return 0;
		}

		filesize = parseoct(buff + 124, 12);

		switch (buff[156])
		{
			case '1':
				printf(" - Ignoring hardlink %s\n", buff);
				break;
			case '2':
				printf(" - Ignoring symlink %s\n", buff);
				break;
			case '3':
				printf(" - Ignoring character device %s\n", buff);
					break;
			case '4':
				printf(" - Ignoring block device %s\n", buff);
				break;
			case '5':
				printf(" - Ignoring dir %s\n", buff);
				filesize = 0;
				break;
			case '6':
				printf(" - Ignoring FIFO %s\n", buff);
				break;
			default:
				memset(tmpg, 0, sizeof(tmpg));
				memcpy(tmpg, filename, strlen(filename)-4);
#ifdef _WIN32
				snprintf(tmpp, sizeof(tmpp), "%s\\%s\\%s", full_path, outfolder, buff);
#else
				snprintf(tmpp, sizeof(tmpp), "./%s/%s", outfolder, buff);
#endif
				printf(" - %s %s\n", (i == 0) ? "Extracting signature" : "Extracting sparse chunk", buff);
				i += 1;
				f = create_file(tmpp);	//, parseoct(buff + 100, 8));
				if (f == NULL) {
					printf(" - Error creating %s\n", tmpp);
					return 0;
				}
				snprintf(flashfile, strlen(basenamee(tmpp))-3, "%s", basenamee(tmpp));
				break;
		}

		while (filesize > 0)
		{
			bytes_read = fread(buff, 1, 512, a);
			if (bytes_read != 512) {
				printf(" - Short read on %s: Expected 512, got %d\n", filename, (int)bytes_read);
				return 0;
			}

			if (filesize < 512)
				bytes_read = filesize;

			if (f != NULL)
			{
				if (fwrite(buff, 1, bytes_read, f) != bytes_read)
				{
					printf(" - Failed write\n");
					fclose(f);
					f = NULL;
				}
			}

			filesize -= bytes_read;
			chunk += 1;
		}

		if (f != NULL) {
			fclose(f);
			f = NULL;
		}

		if (i == 1)
		{
			FILE *fp = NULL;
			char *buffer = NULL;
			unsigned int fp_size;

			fp_size = file_size(tmpp);

			printf(" - Uploading signature %s\n", tmpp);

			if (!fp_size) {
				printf("      Error, size of the %s is 0!\n", tmpp);
				return 0;
			}

repeat_here:
			snprintf(command, sizeof(command), "%s:%08x", is_2021_device ? "download" : "signature", fp_size);
			printf("      %s\n", command);

			if (transfer_bulk_async(dev, EP_OUT, command, strlen(command), USB_TIMEOUT, 1) < 1) {
				printf("      Error writing signature command!\n");
				return 0;
			}

			if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0)) {
				printf("      Error, no signature DATA reply!\n");
				return 0;
			}

			if (strlen(tmp_reply) != 12)
			{
				if (memcmp(tmp_reply, "FAIL", 4) == 0 && !is_2021_device)
				{
					printf("      device from 2021 and up?\n");
					is_2021_device = true;
					goto repeat_here;
				}
				else
				{
					printf("      Error, signature DATA reply size: %zu less than expected: 12!\n", strlen(tmp_reply));
					return 0;
				}
			}


			if (is_2021_device)
			{
				if (memcmp(tmp_reply+4, command+9, 8) != 0)
				{
					printf("      Error, signature DATA reply string: %s is not equal to expected: DATA%s!\n", tmp_reply, command+10);
					return 0;
				}
			}
			else
			{
				if (memcmp(tmp_reply+4, command+10, 8) != 0)
				{
					printf("      Error, signature DATA reply string: %s is not equal to expected: DATA%s!\n", tmp_reply, command+10);
					return 0;
				}
			}

			if ((buffer = (char *)malloc(fp_size+1)) == NULL) {
				printf("      Error allocating buffer!\n");
				return 0;
			}
			buffer[fp_size] = '\0';

			if ((fp = fopen64(tmpp, "rb")) == NULL) {
				printf("      Error opening %s for read!\n", tmpp);
				if (buffer) free(buffer);
				return 0;
			}

			if (fread(buffer, 1, fp_size, fp) < fp_size) {
				printf("      Error storing 0x%x bytes to buffer!\n", fp_size);
				if (buffer) free(buffer);
				fclose(fp);
				return 0;
			}

			fclose(fp);

			if (transfer_bulk_async(dev, EP_OUT, buffer, fp_size, USB_TIMEOUT, 1) < 1) {
				printf("      Error writing signature!\n");
				if (buffer) free(buffer);
				return 0;
			}

			if (buffer) free(buffer);

			if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0)) {
				printf("      Error, no sinature OKAY reply!\n");
				return 0;
			}

			if (strlen(tmp_reply) < 4) {
				printf("      Error, signature reply less than 4, got: %zu bytes!\n", strlen(tmp_reply));
				return 0;
			}

			if (memcmp(tmp_reply, "OKAY", 4) == 0) {
				printf("      OKAY.\n");
			}
			else
			{
				printf("      Error, didn't got signature OKAY reply! Got reply: %s\n", tmp_reply);
				return 0;
			}

			if (is_2021_device)
			{
				snprintf(command, sizeof(command), "signature");
				printf("      %s\n", command);

				if (transfer_bulk_async(dev, EP_OUT, command, strlen(command), USB_TIMEOUT, 1) < 1)
				{
					printf("      Error writing signature command!\n");
					return 0;
				}

				if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
				{
					printf("      Error, no signature OKAY reply!\n");
					return 0;
				}

				if (memcmp(tmp_reply, "OKAY", 4) != 0)
				{
					printf("      Error, signature OKAY reply, got reply: %s!\n", tmp_reply);
					return 0;
				}
				else
				{
					printf("      OKAY.\n");
				}
			}
		}
		else
		{
			FILE *fp = NULL;
			char *buffer = NULL;
			unsigned int fp_size;
			size_t fp_read;
			int g = 0;

			fp_size = file_size(tmpp);

			printf(" - Uploading sparse chunk %s\n", tmpp);

			if (!fp_size) {
				printf("      Error, size of the %s is 0!\n", tmpp);
				return 0;
			}

			snprintf(command, sizeof(command), "download:%08x", fp_size);
			printf("      %s\n", command);

			if (transfer_bulk_async(dev, EP_OUT, command, strlen(command), USB_TIMEOUT, 1) < 1) {
				printf("      Error writing download command!\n");
				return 0;
			}

			if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0)) {
				printf("      Error, no download DATA reply!\n");
				return 0;
			}

			if (strlen(tmp_reply) != 12) {
				printf("      Error, download DATA reply size: %zu less than expected: 12!\n", strlen(tmp_reply));
				return 0;
			}

			if (memcmp(tmp_reply+4, command+9, 8) != 0) {
				printf("      Error, download DATA reply string: %s is not equal to expected: DATA%s!\n", tmp_reply, command+9);
				return 0;
			}

			if ((fp = fopen64(tmpp, "rb")) == NULL) {
				printf("      Error opening %s for read!\n", tmpp);
				return 0;
			}

			while (fp_size > 0)
			{
				g += 1;
				/*printf("         Processing chunk %d: 0x%x bytes\n", g, (fp_size < 0x200000) ? fp_size : 0x200000);*/
				if ((buffer = (char *)malloc(0x200001)) == NULL)
				{
					printf("         Error allocating buffer for chunk: %d!\n", g);
					fclose(fp);
					return 0;
				}
				buffer[0x200000] = '\0';

				if (fp_size < 0x200000)
				{
					fp_read = fread(buffer, 1, fp_size, fp);
					if (fp_read < fp_size)
					{
						printf("         Error reading chunk: %d, got: 0x%zx but expected: 0x%x!\n", g, fp_read, fp_size);
						fclose(fp);
						if (buffer) free(buffer);
						return 0;
					}
				}
				else
				{
					fp_read = fread(buffer, 1, 0x200000, fp);
					if (fp_read < 0x200000)
					{
						printf("         Error reading chunk: %d, got: 0x%zx but expected: 0x200000!\n", g, fp_read);
						fclose(fp);
						if (buffer) free(buffer);
						return 0;
					}
				}

				if (transfer_bulk_async(dev, EP_OUT, buffer, fp_read, USB_TIMEOUT, 1) < 1) {
					printf("         Error uploading chunk %d!\n", g);
					fclose(fp);
					if (buffer) free(buffer);
					return 0;
				}

				fp_size -= fp_read;
				if (buffer) free(buffer);
			}

			fclose(fp);

			if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0)) {
				printf("      Error, no download OKAY reply!\n");
				return 0;
			}

			if (strlen(tmp_reply) < 4) {
				printf("      Error, download reply less than 4, got: %zu bytes!\n", strlen(tmp_reply));
				return 0;
			}

			if (memcmp(tmp_reply, "OKAY", 4) != 0) {
				printf("      Error, didn't got download OKAY reply! Got reply: %s\n", tmp_reply);
				return 0;
			}

			printf("      OKAY.\n");

			if (i == 2)
			{
				if (memcmp(endcommand, "flash", 5) == 0)
				{
					if (memcmp(current_slot, "a", 1) == 0 || memcmp(current_slot, "b", 1) == 0)
					{
						snprintf(command, sizeof(command), "getvar:has-slot:%s", flashfile);
						if (transfer_bulk_async(dev, EP_OUT, command, strlen(command), USB_TIMEOUT, 1) < 1) {
							printf(" - Error writing command %s!\n", command);
							return 0;
						}

						if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0)) {
							printf("      Error, no %s reply!\n", command);
							return 0;
						}

						if (memcmp(tmp_reply, "yes", 3) == 0)
						{
							printf("      Partition: %s have slot: %s\n", flashfile, tmp_reply);
							has_slot = true;
						}

						if (has_slot)
						{
							if (strstr(basenamee(filename), "_other") != NULL)
							{
								if (memcmp(current_slot, "a", 1) == 0)
									snprintf(command, sizeof(command), "erase:%s_b", flashfile);

								if (memcmp(current_slot, "b", 1) == 0)
									snprintf(command, sizeof(command), "erase:%s_a", flashfile);
							}
							else
							{
								if (memcmp(current_slot, "a", 1) == 0)
									snprintf(command, sizeof(command), "erase:%s_a", flashfile);

								if (memcmp(current_slot, "b", 1) == 0)
									snprintf(command, sizeof(command), "erase:%s_b", flashfile);

							}
						}
						else
						{
							snprintf(command, sizeof(command), "erase:%s", flashfile);
						}
					}
					else
					{
						snprintf(command, sizeof(command), "erase:%s", flashfile);
					}

					printf("      %s\n", command);

					if (transfer_bulk_async(dev, EP_OUT, command, strlen(command), USB_TIMEOUT, 1) < 1) {
						printf("      Error writing %s!\n", command);
						return 0;
					}

					if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0)) {
						printf("      Error, no erase OKAY reply!\n");
						return 0;
					}

					if (strlen(tmp_reply) < 4) {
						printf("      Error, erase reply less than 4, got: %zu bytes!\n", strlen(tmp_reply));
						return 0;
					}

					if (memcmp(tmp_reply, "OKAY", 4) != 0) {
						printf("      Error, didn't got erase OKAY reply! Got reply: %s\n", tmp_reply);
						return 0;
					}

					printf("      OKAY.\n");
				}
			}

			/* flash: */

			/* Oreo changed partition image name, so this is a quick fix */
			if (memcmp(endcommand, "Repartition", 11) == 0 && strstr(flashfile, "partitionimage_") != NULL) {
				char renamed[64];
				sscanf(flashfile, "partitionimage_%s", renamed);
				snprintf(command, sizeof(command), "%s:%s", endcommand, renamed);
				printf("      %s\n", command);
			}
			else
			{
				if (has_slot)
				{
					if (strstr(basenamee(filename), "_other") != NULL)
					{
						if (memcmp(current_slot, "a", 1) == 0)
							snprintf(command, sizeof(command), "%s:%s_b", endcommand, flashfile);

						if (memcmp(current_slot, "b", 1) == 0)
							snprintf(command, sizeof(command), "%s:%s_a", endcommand, flashfile);
					}
					else
					{
						if (memcmp(current_slot, "a", 1) == 0)
							snprintf(command, sizeof(command), "%s:%s_a", endcommand, flashfile);

						if (memcmp(current_slot, "b", 1) == 0)
							snprintf(command, sizeof(command), "%s:%s_b", endcommand, flashfile);
					}
				}
				else
				{
					snprintf(command, sizeof(command), "%s:%s", endcommand, flashfile);
				}

				printf("      %s\n", command);
			}

			if (transfer_bulk_async(dev, EP_OUT, command, strlen(command), USB_TIMEOUT, 1) < 1) {
				printf("      Error writing %s!\n", command);
				return 0;
			}

			if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0)) {
				printf("      Error, no %s OKAY reply!\n", endcommand);
				return 0;
			}

			if (strlen(tmp_reply) < 4) {
				printf("      Error, %s reply less than 4, got: %zu bytes!\n", endcommand, strlen(tmp_reply));
				return 0;
			}

			if (memcmp(tmp_reply, "OKAY", 4) != 0) {
				printf("      Error, didn't got %s OKAY reply! Got reply: %s\n", endcommand, tmp_reply);
				return 0;
			}

			printf("      OKAY.\n");
		}

		remove(tmpp);
	}

	return 0;
}

/* track the current level in the xml tree */
static int depth = 0;

static char bootdelivery_xml[15][15][200];
static char bootdelivery_version[100];
static int td1 = 0;
static int td2 = 0;
static int td3 = 0;
static int pd = 0;
static char partitiondelivery_xml[10][200];

/* {"CONFIGURATION", "ATTRIBUTES", "BOOT_CONFIG", "BOOT_IMAGES", "emmc", "s1", "sbl1", "tz", "..."}; */

/* first when start element is encountered */
static void XMLCALL start_element(void *data, const char *element, const char **attribute)
{
	int i;

	/* for (i=0; i<depth; i++)
		printf("    ");

	printf("%s", element); */

	if (depth == 0) {
		if (memcmp(element, "BOOT_DELIVERY", strlen(element)) == 0) {
			for (i=0; attribute[i]; i+=2) {
				if (memcmp(attribute[i], "SPACE_ID", strlen(attribute[i])) == 0) {
					/* printf(" %s = '%s'", attribute[i], attribute[i+1]); */
					snprintf(bootdelivery_version, sizeof(bootdelivery_version), "%s", attribute[i+1]);
				}
			}
		}
	}

	if (depth == 1) {
		if (memcmp(element, "CONFIGURATION", strlen(element)) == 0) {
			for (i=0; attribute[i]; i+=2) {
				if (memcmp(attribute[i], "NAME", strlen(attribute[i])) == 0) {
					/* printf(" %s = '%s'", attribute[i], attribute[i+1]); */
					snprintf(bootdelivery_xml[td1][0], sizeof(bootdelivery_xml[td1][0]), "%s", attribute[i+1]);
				}
			}
		}
	}

	if (depth == 2) {
		if (memcmp(element, "BOOT_CONFIG", strlen(element)) == 0)
			td2++;

		if (memcmp(element, "BOOT_IMAGES", strlen(element)) == 0)
			td3++;

		if (memcmp(element, "ATTRIBUTES", strlen(element)) == 0) {
			for (i=0; attribute[i]; i+=2) {
				if (memcmp(attribute[i], "VALUE", strlen(attribute[i])) == 0) {
					/* printf(" %s = '%s'", attribute[i], attribute[i+1]); */
					snprintf(bootdelivery_xml[td1][1], sizeof(bootdelivery_xml[td1][1]), "%s", attribute[i+1]);
				}
			}
		}

		if (memcmp(element, "HWCONFIG", strlen(element)) == 0) {
			for (i=0; attribute[i]; i+=2) {
				if (memcmp(attribute[i], "REVISION", strlen(attribute[i])) == 0) {
					/* printf(" %s = '%s'", attribute[i], attribute[i+1]); */
					snprintf(bootdelivery_xml[td1][2], sizeof(bootdelivery_xml[td1][2]), "%s", attribute[i+1]);
				}
			}
		}

		/* partition_delivery FILE is inside depth=2 so lets read FILE directly */
		if (memcmp(element, "FILE", strlen(element)) == 0) {
			for (i=0; attribute[i]; i+=2) {
				if (memcmp(attribute[i], "PATH", strlen(attribute[i])) == 0) {
					/* printf(" %s = '%s'", attribute[i], attribute[i+1]); */
					snprintf(partitiondelivery_xml[pd], sizeof(partitiondelivery_xml[pd]), "%s", attribute[i+1]);
					pd += 1;
				}
			}
		}
	}

	if (depth == 3) {
		if (memcmp(element, "FILE", strlen(element)) == 0 && td2) {
			for (i=0; attribute[i]; i+=2) {
				if (memcmp(attribute[i], "PATH", strlen(attribute[i])) == 0) {
					/* printf(" %s = '%s'", attribute[i], attribute[i+1]); */
					snprintf(bootdelivery_xml[td1][3], sizeof(bootdelivery_xml[td1][3]), "%s", attribute[i+1]);
				}
			}
		}

		if (memcmp(element, "FILE", strlen(element)) == 0 && td3) {
			for (i=0; attribute[i]; i+=2) {
				if (memcmp(attribute[i], "PATH", strlen(attribute[i])) == 0) {
					/* printf(" %s = '%s'", attribute[i], attribute[i+1]); */
					snprintf(bootdelivery_xml[td1][3+td3], sizeof(bootdelivery_xml[td1][3+td3]), "%s", attribute[i+1]);
					td3++;
				}
			}
		}
	}

	depth++;

	if (data) { }
}

/* decrement the current level of the tree */
static void XMLCALL end_element(void *data, const char *element)
{
	if (memcmp(element, "CONFIGURATION", 13) == 0)
		td1++;

	if (memcmp(element, "BOOT_CONFIG", 11) == 0)
		td2 = 0;

	if (memcmp(element, "BOOT_IMAGES", 11) == 0)
		td3 = 0;

	depth--;

	if (data) { }
}

static void handle_data(void *data, const char *content, int length) {
	char *tmp = (char *)malloc(length+1);
	strncpy(tmp, content, length);
	tmp[length] = '\0';
	data = (void *)tmp;
	if (tmp) free(tmp);
	if (data) { }
}

static int parse_xml(char *xml_file) {
	int ret = 1;
	char *xml_buff;
	size_t xml_buff_size, xml_file_size=0;
	FILE *fp;

	XML_Parser parser = XML_ParserCreate(NULL);
	if (!parser) {
		printf("Couldn't allocate memory for xml parser!\n");
		return 0;
	}

	fp = fopen64(xml_file, "rb");
	if (fp == NULL) {
		printf("Failed to open %s file\n", xml_file);
		return 0;
	}

	fseeko64(fp, 0, SEEK_END);
	xml_file_size = ftello64(fp);
	fseeko64(fp, 0, SEEK_SET);

	xml_buff_size = xml_file_size;

	XML_SetElementHandler(parser, start_element, end_element);
	XML_SetCharacterDataHandler(parser, handle_data);

	xml_buff = (char *)malloc(xml_buff_size+1);

	fread_unus_res(xml_buff, 1, xml_buff_size, fp);
	xml_buff[xml_buff_size] = '\0';

	/* parse the xml */
	if (XML_Parse(parser, xml_buff, strlen(xml_buff), XML_TRUE) == XML_STATUS_ERROR) {
		printf("Error: at line %" XML_FMT_INT_MOD "u: %s\n", XML_GetCurrentLineNumber(parser), XML_ErrorString(XML_GetErrorCode(parser)));
		ret = 0;
	}

	if (fp)
		fclose(fp);

	XML_ParserFree(parser);

	if (xml_buff) free(xml_buff);

	return ret;
}

static int proced_ta_file(char *ta_file, HANDLE dev)
{
	FILE *fp = NULL;
	char *line = NULL;
	size_t len = 0;
	ssize_t read;

	char unit[9];
	char unit_sz_tmp[9];
	char command[64];
	unsigned int unit_sz;
	char *unit_data=NULL;
	unsigned int i, j, unit_dec=0, the_rest=0, partition=0, finished=0, ret=1;

	/* some devices have some units which exceeds sizeof uint16_t */
	bool is_32bit = false;

	printf("Processing %s\n", ta_file);

	if ((unit_data = (char *)malloc(MAX_UNIT_LINE_LEN)) == NULL)
	{
		printf(" - Error allocating unit_data!\n");
		return 0;
	}

	if ((fp = fopen64(ta_file, "rb")) == NULL) {
		printf(" - Unable to open %s!\n", ta_file);
		return 0;
	}

	while((read = g_getline(&line, &len, fp)) != -1)
	{
		is_32bit = false;

		switch(read)
		{
			case 1:
				/*LOG("Skipped empty line.\n\n");*/
				break;

			case 3:
				if (line[0] >= 0x30 && line[0] <= 0x39 && line[1] >= 0x30 && line[1] <= 0x39)
				{
					partition = atoi(line);
					printf(" - Partition: %u\n", partition);
				}
				break;

			default:
				if (line[0] == '/')
				{
					/*LOG("Skipped comment line.\n\n");*/
				}
				else
				{
					/*LOG("Retrieved line of lenght: %lu\n", read);*/

					if (check_valid_unit(line))
					{
						finished = 0;
						memcpy(unit, line, 8);
						unit[8] = '\0';
						to_uppercase(unit);
						sscanf(unit, "%x", &unit_dec);
						printf(" - Unit: %X (%u)\n", unit_dec, unit_dec);

						/*
						 * in case of 32 bit unit size!
						 * unit(8) + space(1) + unit size(8) + space(1)
						 */
						if (strlen(line) >= 18)
						{
							if (line[8] == ' ' && line[17] == ' ')
							{
								is_32bit = true;
							}
						}

						trim(line);
						/*LOG("Line lenght after trim: %lu\n", strlen(line));*/

						if (is_32bit)
						{
							/* unit(8) + unit size(8) + at least one hex(2) */
							if (strlen(line) < 18)
							{
								if (strlen(line) == 16)
								{
									printf(" - Found specific unit which don't contain data.\n");
									the_rest = 0;
									finished = 1;
									unit_sz = 0;
								}
								else
								{
									printf(" - Error: corrupted unit! Skipping this unit!\n\n");
									the_rest = 0;
									break;
								}
							}
							else
							{
								memcpy(unit_sz_tmp, line+8, 8);
								unit_sz_tmp[8] = '\0';
								sscanf(unit_sz_tmp, "%x", &unit_sz);
								printf(" - Unit size: 0x%x\n", unit_sz);
								memset(unit_data, '\0', MAX_UNIT_LINE_LEN);
								i = strlen(line);
								if (i)
									memcpy(unit_data, line+16, i);
								unit_data[i] = '\0';

								if ((unsigned int)strlen(line)-16 < unit_sz*2)
								{
									/*LOG("Data probably continues in a new line (%u not match %u)!\n",
										(unsigned int)strlen(line)-16, unit_sz*2);*/
									the_rest = 1;
								}
								else
								{
									the_rest = 0;
								}

								if ((unsigned int)strlen(unit_data) == unit_sz*2)
									finished = 1;
							}
						}
						else
						{
							/* unit(8) + unit size(4) + at least one hex(2) */
							if (strlen(line) < 14)
							{
								if (strlen(line) == 12)
								{
									printf(" - Found specific unit which don't contain data.\n");
									the_rest = 0;
									finished = 1;
									unit_sz = 0;
								}
								else
								{
									printf(" - Error: corrupted unit! Skipping this unit!\n\n");
									the_rest = 0;
									break;
								}
							}
							else
							{
								memcpy(unit_sz_tmp, line+8, 4);
								unit_sz_tmp[4] = '\0';
								sscanf(unit_sz_tmp, "%x", &unit_sz);
								printf(" - Unit size: 0x%x\n", unit_sz);
								memset(unit_data, '\0', MAX_UNIT_LINE_LEN);
								i = strlen(line);
								if (i)
									memcpy(unit_data, line+12, i);
								unit_data[i] = '\0';

								if ((unsigned int)strlen(line)-12 < unit_sz*2)
								{
									/*LOG("Data probably continues in a new line (%u not match %u)!\n",
										(unsigned int)strlen(line)-12, unit_sz*2);*/
									the_rest = 1;
								}
								else
								{
									the_rest = 0;
								}

								if ((unsigned int)strlen(unit_data) == unit_sz*2)
									finished = 1;
							}

						}
					}
					else
					{
						if (the_rest)
						{
							finished = 0;
							trim(line);
							/*LOG("Line lenght after trim: %lu\n", strlen(line));
							LOG("Found the rest ot the data!\n");*/
							j = strlen(unit_data);
							i = strlen(line);
							if (i)
								memcpy(unit_data+j, line, i);
							unit_data[j+i] = '\0';

							if ((unsigned int)strlen(unit_data) == unit_sz*2)
							{
								the_rest = 0;
								finished = 1;
							}
						}
					}

					if (finished)
					{
						char *unit_total_temp = NULL;

						/*
							unit 0x7d3 (2003) hardware config
							unit 0x7da (2010) simlock
							unit 0x851 (2129) simlock signature
							unit 0x1324 (4900) device id
							unit 0x1046F (66671) google lock state ( allow bootloader unlock in dev settings )
							unit 0x9A9 (2473) value 1 for enable serial console or value 0 (default) to disable (https://forum.xda-developers.com/showpost.php?p=80212371&postcount=1125)
							unit 0x10471 (66673) protocol switch? Or keystore? What is this? Depend on existance of unit 0x36A (https://forum.xda-developers.com/showpost.php?p=80176195&postcount=1093)
						*/

						if (/*memcmp(unit, "000008B2", 8) == 0 || unlock key */
						    memcmp(unit, "000007D3", 8) == 0 || /* hardware config */
						    memcmp(unit, "000007DA", 8) == 0 || /* simlock */
						    memcmp(unit, "00000851", 8) == 0 || /* simlock signature */
						    memcmp(unit, "000008A2", 8) == 0 || /* device name */
						    memcmp(unit, "00001324", 8) == 0 || /* device id */
						    memcmp(unit, "0001046B", 8) == 0) { /* drm key */
							printf(" - Skipping unit %X\n", unit_dec);
							continue;
						}

						if ((unit_total_temp = (char *)malloc(unit_sz+8)) == NULL) {
							printf(" - Error allocating unit_temp!\n");
							ret = 0;
							goto finish_proced_ta;
						}

						finished = 0;
						/*LOG("\n<<-------------------- Retrieval finished! Found unit: %s,"
							" Unit size: %04X, Unit data:%s\n",
							 unit, unit_sz, unit_sz ? "" : " NULL");*/
						to_ascii(unit_data, unit_data);

						snprintf(command, sizeof(command), "download:%08x", unit_sz);
						printf("      %s\n", command);

						if (transfer_bulk_async(dev, EP_OUT, command, strlen(command), USB_TIMEOUT, 1) < 1) {
							printf("      Error writing download command!\n");
							ret = 0;
							goto finish_proced_ta;
						}

						if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0)) {
							printf("      Error, no download DATA reply!\n");
							ret = 0;
							goto finish_proced_ta;
						}

						if (strlen(tmp_reply) != 12) {
							printf("      Error, download DATA reply size: %zu less than expected: 12!\n", strlen(tmp_reply));
							ret = 0;
							goto finish_proced_ta;
						}

						if (memcmp(tmp_reply+4, command+9, 8) != 0) {
							printf("      Error, download DATA reply string: %s is not equal to expected: DATA%s!\n", tmp_reply, command+9);
							ret = 0;
							goto finish_proced_ta;
						}

						if (unit_sz > 0)
						{
							if (transfer_bulk_async(dev, EP_OUT, unit_data, unit_sz, USB_TIMEOUT, 1) < 1) {
								printf("      Error writing unit data!\n");
								ret = 0;
								goto finish_proced_ta;
							}
						}

						if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0)) {
							printf("      Error, no OKAY reply!\n");
							ret = 0;
							goto finish_proced_ta;
						}

						if (strlen(tmp_reply) < 4) {
							printf("      Error, reply less than 4, got: %zu bytes!\n", strlen(tmp_reply));
							ret = 0;
							goto finish_proced_ta;
						}

						if (memcmp(tmp_reply, "OKAY", 4) != 0) {
							printf("      Error, didn't got OKAY reply! Got reply: %s\n", tmp_reply);
							ret = 0;
							goto finish_proced_ta;
						}

						printf("      OKAY.\n");

						snprintf(command, sizeof(command), "Write-TA:%u:%u", partition, unit_dec);
						printf("      %s\n", command);

						if (transfer_bulk_async(dev, EP_OUT, command, strlen(command), USB_TIMEOUT, 1) < 1) {
							printf("      Error writing command WriteTA!\n");
							ret = 0;
							goto finish_proced_ta;
						}

						if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0)) {
							printf("      Error, no OKAY reply!\n");
							ret = 0;
							goto finish_proced_ta;
						}

						if (strlen(tmp_reply) < 4) {
							printf("      Error, reply less than 4, got: %zu bytes!\n", strlen(tmp_reply));
							ret = 0;
							goto finish_proced_ta;
						}

						if (memcmp(tmp_reply, "OKAY", 4) != 0) {
							printf("      Error, didn't got OKAY reply! Got reply: %s\n", tmp_reply);
							ret = 0;
							goto finish_proced_ta;
						}

						printf("      OKAY.\n");

						if (unit_total_temp)
							free(unit_total_temp);
					}
					/*LOG("\n");*/
				}
				break;
		}
	}

finish_proced_ta:
	if (unit_data)
		free(unit_data);

	if (fp)
		fclose(fp);

	if (line)
		free(line);

	return ret;
}

#ifdef _WIN32
typedef BOOL (WINAPI *P_GDFSE)(LPCTSTR, PULARGE_INTEGER, PULARGE_INTEGER, PULARGE_INTEGER);

static unsigned long get_free_space(char *pszDrive)
{
	BOOL  fResult;
	unsigned long dwSectPerClust, dwBytesPerSect, dwFreeClusters, dwTotalClusters;

	P_GDFSE pGetDiskFreeSpaceEx = NULL;

	unsigned long long i64FreeBytesToCaller, i64TotalBytes, i64FreeBytes;

	pGetDiskFreeSpaceEx = (P_GDFSE)GetProcAddress (GetModuleHandle("kernel32.dll"), "GetDiskFreeSpaceExA");
	if (pGetDiskFreeSpaceEx)
	{
		fResult = pGetDiskFreeSpaceEx (pszDrive,
				 (PULARGE_INTEGER)&i64FreeBytesToCaller,
				 (PULARGE_INTEGER)&i64TotalBytes,
				 (PULARGE_INTEGER)&i64FreeBytes);
		if (fResult)
		{
			printf ("\nDetermining available free space by GetDiskFreeSpaceEx:\n\n");
			printf ("  Available space to caller    = %llu MB\n", i64FreeBytesToCaller / (1024 * 1024));
			printf ("  Total space on current drive = %llu MB\n", i64TotalBytes / (1024 * 1024));
			printf ("  Free space on drive          = %llu MB\n", i64FreeBytes / (1024 * 1024));
		}
	}
	else
	{
		fResult = GetDiskFreeSpace (pszDrive,
				 &dwSectPerClust,
				 &dwBytesPerSect,
				 &dwFreeClusters,
				 &dwTotalClusters);

		if (fResult)
		{
			/* force 64-bit math */
			i64TotalBytes = (unsigned long long)(dwTotalClusters * dwSectPerClust * dwBytesPerSect);
			i64FreeBytes = (unsigned long long)(dwFreeClusters * dwSectPerClust * dwBytesPerSect);

			printf ("\nDetermining available free space by GetDiskFreeSpace:\n\n");
			printf ("  Free space                   = %llu MB\n", i64FreeBytes / (1024 * 1024));
			printf ("  Total space on current drive = %llu MB\n", i64TotalBytes / (1024 * 1024));
		}
	}

	if (!fResult)
	{
		printf ("\nError: %lu:  unable to determine available free space for current drive!\n", GetLastError());
		return 0;
	}

	return i64FreeBytesToCaller / (1024 * 1024);
}
#else
#include <sys/statvfs.h>

static unsigned long get_free_space(char *fnPath)
{
        struct statvfs fiData;

	if ((statvfs(fnPath, &fiData)) < 0)
	{
		printf("  Error: unable to determine available free space for current drive!\n");
		return 0;
	}
	else
	{
		printf ("\nDetermining available free space:\n\n");
		printf ("  Available space to caller    = %llu MB\n",
			 (unsigned long long)((fiData.f_bsize * fiData.f_bavail) / 1024 / 1024));
		printf ("  Total space on current drive = %llu MB\n",
			 (unsigned long long)((fiData.f_bsize * fiData.f_blocks) / 1024 / 1024));
		printf ("  Free space on drive          = %llu MB\n",
			 (unsigned long long)((fiData.f_bsize * fiData.f_bfree) / 1024 / 1024));
	}

	return (unsigned long)fiData.f_bfree;
}
#endif

/*========================================================================================*/

int main(int argc, char *argv[])
{
	FILE *fi = NULL;
	int fld_cbck;
	char fld[256];
	char file_format[3];
	char sinfil[256];
	char *progname = basenamee(argv[0]);
	unsigned short VID = 0x0FCE;
	unsigned short PID = 0xB00B;
	int i, j, ret=0;
	char ch;

	DIR *dir = NULL;
	struct dirent *ep = NULL;
	char *extension = NULL;
	int sin_found = 0;

	struct stat filestat;
	char searchfor[1024];
	int bootdelivery_found = 0;

	bool flash_booth_slots = false;

	HANDLE dev = NULL;

	unsigned long available_mb;

	unsigned char reboot_mode = 0;
	char reboot_string[32];

	bool flash_session_created = false;

#ifdef _WIN32
	char *device = NULL;
	char *working_path = _getcwd(0, 0);
#else
	char working_path[PATH_MAX];
	if (getcwd(working_path, sizeof(working_path)) == NULL)
	{
		perror("getcwd() error");
		ret = 1;
		goto pauza;
	}
#endif

	printf("--------------------------------------------------------\n");
	printf("            %s v%d by Munjeni @ 2017/2020           \n", progname, VERSION);
	printf("--------------------------------------------------------\n");

	available_mb = get_free_space(working_path);

	if (available_mb < 10240)
	{
		printf("  Error! You do not have needed 10240 MB available free space on your\n");
		printf("  disk drive! You have only %lu MB free.\n", available_mb);
		ret = 1;
		goto pauza;
	}

	memset(slot_count, 0x30, sizeof(slot_count));
	memset(current_slot, 0x30, sizeof(current_slot));

/*========================================  extract GordonGate  ======================================*/
#ifdef _WIN32
	if (argc < 2)
	{
		printf("\nOptional step! Type 'y' and press ENTER if you need GordonGate flash driver, or type 'n' to skip.\n");
		printf("This creates GordonGate driver installer in the same dir with %s!\n", progname);
		if (scanf(" %c", &ch)) { }
		if (ch == 'y' || ch == 'Y')
		{
			FILE *gg = fopen("Sony_Mobile_Software_Update_Drivers_x64_Setup.msi", "wb");
			if (gg == NULL) {
				printf("Unable to create Sony_Mobile_Software_Update_Drivers_x64_Setup.msi!\n");
				ret = 1;
				goto pauza;
			}
			fwrite(GordonGate, 1, GordonGate_len, gg);
			fclose(gg);
			printf("Sony_Mobile_Software_Update_Drivers_x64_Setup.msi created.\n");
			goto pauza;
		}
	}
#endif
/*====================================================================================================*/

#ifdef _WIN32
	device = open_dev(VID, PID);
	if (device[0] == '\0') {
		printf("\nNo usb device with vid:0x%04x pid:0x%04x !\n", VID, PID);
		ret = 1;
		goto pauza;
	}

	dev = CreateFile(
		 device,
		 GENERIC_WRITE | GENERIC_READ,
		 FILE_SHARE_READ | FILE_SHARE_WRITE,
		 NULL,
		 OPEN_EXISTING,
		 FILE_FLAG_OVERLAPPED,
		 NULL);

	if (dev == INVALID_HANDLE_VALUE) {
		DisplayError(TEXT("Error CreateFile!"));
		SetupDiDestroyDeviceInfoList(hDevInfo);
		ret = 1;
		goto pauza;
	}
#else
#ifdef __APPLE__
#define usb_interface interface
	//putenv("LIBUSB_DEBUG=4"); // set debug level
	libusb_device *device = NULL;
	unsigned char bus, port_path[8];
	int k;
	int iface, nb_ifaces, first_iface = -1;
	int iface_detached = -1;
	struct libusb_config_descriptor *conf_desc = NULL;
	const struct libusb_endpoint_descriptor *endpoint = NULL;
	struct libusb_device_descriptor dev_desc;

	const char* speed_name[5] = {
				"Unknown", "1.5 Mbit/s (USB LowSpeed)",
				"12 Mbit/s (USB FullSpeed)",
				"480 Mbit/s (USB HighSpeed)",
				"5000 Mbit/s (USB SuperSpeed)"
	};

	char string[128];
	unsigned char string_index[3];                      // indexes of the string descriptors

	if (libusb_init(NULL))
	{
		printf("libusb init error!\n");
		ret = 1;
		goto pauza;
	}

	dev = libusb_open_device_with_vid_pid(NULL, VID, PID);
	if (dev == NULL)
	{
		printf("\nNo usb device with vid:0x%04x pid:0x%04x !\n", VID, PID);
		ret = 1;
		libusb_exit(NULL);
		goto pauza;
	}

	device = libusb_get_device(dev);
	bus = libusb_get_bus_number(device);

	ret = libusb_get_port_numbers(device, port_path, sizeof(port_path));
	if (ret > 0)
	{
		printf("\nDevice properties:\n");
		printf("        bus number: %d\n", bus);
		printf("         port path: %d", port_path[0]);

		for (i=1; i<ret; i++) {
			printf("->%d", port_path[i]);
		}
		printf(" (from root hub)\n");
	}

	ret = libusb_get_device_speed(device);
	if ((ret < 0) || (ret > 4))
		ret = 0;
	printf("             speed: %s\n", speed_name[ret]);

	printf("\nReading device descriptor:\n");
	libusb_get_device_descriptor(device, &dev_desc);
	printf("            length: %d\n", dev_desc.bLength);
	printf("      device class: %d\n", dev_desc.bDeviceClass);
	printf("               S/N: %d\n", dev_desc.iSerialNumber);
	printf("           VID:PID: 0x%X:0x%X\n", dev_desc.idVendor, dev_desc.idProduct);
	printf("         bcdDevice: 0x%X\n", dev_desc.bcdDevice);
	printf("   iMan:iProd:iSer: %d:%d:%d\n", dev_desc.iManufacturer, dev_desc.iProduct, dev_desc.iSerialNumber);
	printf("          nb confs: %d\n", dev_desc.bNumConfigurations);

	/* Copy the string descriptors for easier parsing */
	string_index[0] = dev_desc.iManufacturer;
	string_index[1] = dev_desc.iProduct;
	string_index[2] = dev_desc.iSerialNumber;

	printf("\nReading configuration descriptors:\n");
	libusb_get_config_descriptor(device, 0, &conf_desc);
	nb_ifaces = conf_desc->bNumInterfaces;
	printf("             nb interfaces: %d\n", nb_ifaces);

	if (nb_ifaces > 0)
		first_iface = conf_desc->usb_interface[0].altsetting[0].bInterfaceNumber;

	for (i=0; i < nb_ifaces; ++i)
	{
		printf("              interface[%d]: id = %d\n",
					i,
					conf_desc->usb_interface[i].altsetting[0].bInterfaceNumber);

		for (j=0; j < conf_desc->usb_interface[i].num_altsetting; ++j)
		{
			printf("interface[%d].altsetting[%d]: num endpoints = %d\n",
				i, j, conf_desc->usb_interface[i].altsetting[j].bNumEndpoints);

			printf("   Class.SubClass.Protocol: 0x%X 0x%X 0x%X\n",
				conf_desc->usb_interface[i].altsetting[j].bInterfaceClass,
				conf_desc->usb_interface[i].altsetting[j].bInterfaceSubClass,
				conf_desc->usb_interface[i].altsetting[j].bInterfaceProtocol);

			for (k=0; k < conf_desc->usb_interface[i].altsetting[j].bNumEndpoints; ++k)
			{
				endpoint = &conf_desc->usb_interface[i].altsetting[j].endpoint[k];
				printf("       endpoint[%d].address: 0x%02X\n", k, endpoint->bEndpointAddress);

				if (conf_desc->usb_interface[i].altsetting[j].bNumEndpoints == 2)
				{
					if (k == 1)
						endpoint_out = endpoint->bEndpointAddress;

					if (k == 0)
						endpoint_in = endpoint->bEndpointAddress;
				}
				printf("           max packet size: 0x%X\n", endpoint->wMaxPacketSize);
				printf("          polling interval: 0x%X\n", endpoint->bInterval);
			}
		}
	}

	libusb_free_config_descriptor(conf_desc);

	for (iface=0; iface < nb_ifaces; ++iface)
	{
		printf("\nSet configuration:");

		ret = libusb_set_configuration(dev, 1);

		if (ret == LIBUSB_SUCCESS)
		{
			printf(" succed.\n");
		}
		else
		{
			printf(" fail with error: %s\n", libusb_error_name(ret));
			ret = 1;
			goto release;
		}

		printf("\nClaiming interface %d:", iface);
		ret = libusb_claim_interface(dev, iface);


		if ((ret != LIBUSB_SUCCESS) && libusb_has_capability(LIBUSB_CAP_SUPPORTS_DETACH_KERNEL_DRIVER) && (libusb_kernel_driver_active(dev, iface) > 0))
		{
			printf(" failed.\n");

			/* Try to detach the kernel driver */
			printf("A kernel driver is active, trying to detach it:");
			ret = libusb_detach_kernel_driver(dev, iface);
			if (ret == LIBUSB_SUCCESS)
			{
				printf(" succed.\n");
				iface_detached = iface;
				printf("Claiming interface again:");
				ret = libusb_claim_interface(dev, iface);
			}
		}

		if (ret != LIBUSB_SUCCESS)
		{
			printf(" failed with error: %s\n", libusb_error_name(ret));
			ret = 1;
			goto release;
		}
		else
		{
			printf(" succed.\n");
		}
	}

	printf("\nReading string descriptors:\n");
	for (i=0; i<3; ++i)
	{
		if (string_index[i] == 0)
			continue;

		if (libusb_get_string_descriptor_ascii(dev, string_index[i], (unsigned char*)string, 128) >= 0)
			printf("   String (0x%02X): \"%s\"\n", string_index[i], string);
	}

	/* Read the OS String Descriptor */
	if (libusb_get_string_descriptor_ascii(dev, 0xEE, (unsigned char*)string, 128) >= 0)
		printf("   String (0x%02X): \"%s\"\n", 0xEE, string);
	printf("\n");

#else
	dev = get_flashmode(VID, PID);

	if (dev == NULL) {
		printf("\nNo usb device with vid:0x%04x pid:0x%04x !\n", VID, PID);
		ret = 1;
		goto pauza;
	}
#endif
#endif

/*============================================  reboot mode ==========================================*/

	if (argc < 2)
	{
		printf("\nReboot mode at the end of flashing:\n  typa 'a' for reboot to android, type 'f' for reboot to fastboot, type 's' for reboot to same mode, type 'p' for poweroff, and press ENTER.\n");
		if (scanf(" %c", &ch)) { }
		switch(ch)
		{
			case 'a':
			case 'A':
				reboot_mode = 1; /* android */
				break;

			case 'f':
			case 'F':
				reboot_mode = 2; /* fastboot */
				break;

			case 's':
			case 'S':
				reboot_mode = 3; /* flashmode */
				break;

			case 'p':
			case 'P':
			default:
				reboot_mode = 0; /* power off */
				break;
		}
	}

/*======================================= commands from script =======================================*/

	if (argc > 1)
	{
		if (transfer_bulk_async(dev, EP_OUT, argv[1], strlen(argv[1]), USB_TIMEOUT, 1) < 1)
		{
			printf("Error writing commad: %s\n", argv[1]);
			ret = 1;
			goto endflashing;
		}
		else
		{
			printf("Writing command: %s\n", argv[1]);

			if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
			{
				printf("Error, null reply\n");
				ret = 1;
				goto endflashing;
			}
			else
			{
				display_buffer_hex_ascii("got first reply", tmp_reply, get_reply_len);

				if (memcmp(tmp_reply, "FAIL", 4) == 0)
				{
					printf("got fail reply: %s\n", tmp_reply);
					ret = 1;
					goto endflashing;
				}
				else
				{
					if (memcmp(tmp_reply, "DATA", 4) == 0)
					{
						unsigned int data_len = 0;

						if (get_reply_len != 12) {
							printf("Errornous DATA reply!\n");
							display_buffer_hex_ascii("replied", tmp_reply, get_reply_len);
							ret = 1;
							goto endflashing;
						}

						sscanf(tmp_reply+4, "%08x", &data_len);

						if (!data_len)
						{
							printf("got null data_len!\n");

							if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
							{
								printf("Error retrieving seccond reply!\n");
								ret = 1;
								goto endflashing;
							}

							display_buffer_hex_ascii("got last reply", tmp_reply, get_reply_len);

							if (strstr(tmp_reply, "OKAY") == NULL)
							{
								printf("Error, no OKAY reply!\n");
								display_buffer_hex_ascii("got reply", tmp_reply, get_reply_len);
								ret = 1;
								goto endflashing;
							}
						}
						else
						{
							char *data_buf = NULL;

							if ((data_buf = (char *)malloc(data_len)) == NULL)
							{
								printf("error allocating 0x%x bytes!\n", data_len);
								ret = 1;
								goto endflashing;
							}

							if (!get_reply(dev, EP_IN, data_buf, data_len, USB_TIMEOUT, 1))
							{
								printf("Error retrieving data!\n");
								free(data_buf);
								ret = 1;
								goto endflashing;
							}

							display_buffer_hex_ascii("data_buf", data_buf, data_len);

							// sometimes OKAY reply is inside data buffer
							if (data_len >= 4 && data_buf[data_len - 4] == 'O' && data_buf[data_len - 3] == 'K' && data_buf[data_len - 2] == 'A' && data_buf[data_len - 1] == 'Y')
							{
								data_len -= 4;
							}
							else
							{
								if (!get_reply(dev, EP_IN, tmp, 5, USB_TIMEOUT, 0))
								{
									printf("Error retrieving OKAY reply!\n");
									free(data_buf);
									ret = 1;
									goto endflashing;
								}

								if (strstr(tmp_reply, "OKAY") == NULL)
								{
									printf("Error, no OKAY reply!\n");
									free(data_buf);
									ret = 1;
									goto endflashing;
								}
							}

							FILE *dumpme = NULL;
							display_buffer_hex_ascii("replied", tmp_reply, get_reply_len);
							if ((dumpme = fopen64("dump.bin", "wb")) == NULL)
							{
								printf("dump.bin will not be created!\n");
							}
							else
							{
								fwrite(data_buf, 1, data_len, dumpme);
								fclose(dumpme);
								printf("dump.bin created.\n");
							}

							free(data_buf);
						}
					}
				}
			}
		}

		goto endflashing;
	}

/*==========================================  dump trim area  ========================================*/

	printf("\nOptional step! Type 'y' and press ENTER if you want dump trim area, or type 'n' and press ENTER to skip.\n");
	printf("Do in mind this doesn't dump drm key since sake authentifiction is need for that! But it is recommend to have dump in case hard brick!\n");
	if (scanf(" %c", &ch)) { }

	if (ch == 'y' || ch == 'Y')
	{
		for (i=1; i <= 2; ++i)
		{
			snprintf(tmp, sizeof(tmp), "Read-all-TA:%d", i);

			if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
			{
				printf("Error dumping trimarea partition %d !!\n", i);
				ret = 1;
				goto endflashing;
			}
			else
			{
				printf("Dumping trimarea partition %d.\n", i);

				if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
				{
					printf("Error, null reply\n");
					ret = 1;
					goto endflashing;
				}
				else
				{
					//display_buffer_hex_ascii("got first reply", tmp_reply, get_reply_len);

					if (memcmp(tmp_reply, "FAIL", 4) == 0)
					{
						printf("got fail reply: %s\n", tmp_reply);
						ret = 1;
						goto endflashing;
					}
					else
					{
						if (memcmp(tmp_reply, "DATA", 4) == 0)
						{
							unsigned int data_len = 0;

							if (get_reply_len != 12)
							{
								printf("Errornous DATA reply!\n");
								display_buffer_hex_ascii("replied", tmp_reply, get_reply_len);
								ret = 1;
								goto endflashing;
							}

							sscanf(tmp_reply+4, "%08x", &data_len);

							if (!data_len)
							{
								printf("got null data_len!\n");

								if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
								{
									printf("Error retrieving seccond reply!\n");
									ret = 1;
									goto endflashing;
								}

								//display_buffer_hex_ascii("got last reply", tmp_reply, get_reply_len);

								if (strstr(tmp_reply, "OKAY") == NULL)
								{
									printf("Error, no OKAY reply!\n");
									display_buffer_hex_ascii("got reply", tmp_reply, get_reply_len);
									ret = 1;
									goto endflashing;
								}
							}
							else
							{
								char *data_buf = NULL;

								if (data_len > BUFF_MAX)
								{
									printf("Bug!!! DATA_LEN: 0x%x > BUFF_MAX: 0x%x\n", data_len, BUFF_MAX);
									ret = 1;
									goto endflashing;
								}

								if ((data_buf = (char *)malloc(data_len)) == NULL)
								{
									printf("error allocating 0x%x bytes!\n", data_len);
									ret = 1;
									goto endflashing;
								}

								if (!get_reply(dev, EP_IN, data_buf, data_len, USB_TIMEOUT, 1))
								{
									printf("Error retrieving data!\n");
									free(data_buf);
									ret = 1;
									goto endflashing;
								}

								//display_buffer_hex_ascii("data_buf", data_buf, data_len);

								// sometimes OKAY reply is inside data buffer
								if (data_len >= 4 && data_buf[data_len - 4] == 'O' && data_buf[data_len - 3] == 'K' && data_buf[data_len - 2] == 'A' && data_buf[data_len - 1] == 'Y')
								{
									data_len -= 4;
								}
								else
								{
									if (!get_reply(dev, EP_IN, tmp, 5, USB_TIMEOUT, 0))
									{
										printf("Error retrieving OKAY reply!\n");
										free(data_buf);
										ret = 1;
										goto endflashing;
									}

									if (strstr(tmp_reply, "OKAY") == NULL)
									{
										printf("Error, no OKAY reply!\n");
										free(data_buf);
										ret = 1;
										goto endflashing;
									}
								}

								FILE *dumpme = NULL;

								snprintf(fld, sizeof(fld), "tadump/");
								if (0 != access(fld, F_OK))
								{
									if (ENOENT == errno)
									{
#ifdef _WIN32
										fld_cbck = mkdir("tadump");
#else
										fld_cbck = mkdir("tadump", 0755);
#endif
										if (fld_cbck == 0)
										{
											printf("Created ouput folder tadump.\n");
										}
										else
										{
											printf("FAILURE to create output folder tadump!\n");
											free(data_buf);
											ret = 1;
											goto endflashing;
										}
									}

									if (ENOTDIR == errno)
									{
										printf("FAILURE to create output folder tadump because there is file called tadump!!\n"
											"Remove or rename file tadump first!\n");
										free(data_buf);
										ret = 1;
										goto endflashing;
									}

								}
								else
								{
									printf("Dumping trim area to tadump folder.\n");
								}
#ifdef _WIN32
								snprintf(tmp, sizeof(tmp), "%s\\tadump\\tadump_%d.ta", working_path, i);
#else
								snprintf(tmp, sizeof(tmp), "./tadump/tadump_%d.ta", i);
#endif
								//display_buffer_hex_ascii("replied", tmp_reply, get_reply_len);

								if ((dumpme = fopen64(tmp, "wb")) == NULL)
								{
									printf("%s will not be created!\n", tmp);
								}
								else
								{
									unsigned int x = 0;
									unsigned int y;
									unsigned int unitt_sz;

									fprintf(dumpme, "//created with %s v%d by Munjeni @ 2017/2020\n", progname, VERSION);

									if (i == 1)
										fprintf(dumpme, "//trim partition\n01\n\n");
									else
										fprintf(dumpme, "//misc partition\n02\n\n");

									while(x < data_len)
									{
										for (y=0; y < 4; ++y)
										{
											memcpy(tmp_reply+y, data_buf+x+y, 1);
											fprintf(dumpme, "%02X", tmp_reply[y] & 0xff);
										}

										fprintf(dumpme, " ");

										x += 4;

										unitt_sz = 0;
										memcpy(&unitt_sz, data_buf+x, 4);
										unitt_sz = swap_uint32(unitt_sz);
										if (unitt_sz > 0xffff)
											fprintf(dumpme, "%08X ", unitt_sz);
										else
											fprintf(dumpme, "%04X ", unitt_sz & 0xffff);

										x += 4;

										for (y=0; y < unitt_sz; ++y)
										{
											memcpy(tmp_reply+y, data_buf+x+y, 1);
											fprintf(dumpme, "%02X ", tmp_reply[y] & 0xff);
										}

										fprintf(dumpme, "\n\n");

										x += unitt_sz;
									}

									fclose(dumpme);
									printf("%s created.\n", tmp);
								}

								free(data_buf);
							}
						}
					}
				}
			}
		}

		memset(tmp_reply, 0, BUFF_MAX);
		goto endflashing;
	}

/*=========================================  DEVICE INFO  ============================================*/

	snprintf(tmp, sizeof(tmp), "getvar:max-download-size");
	if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
	{
		printf(" - Error writing command %s!\n", tmp);
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		ret = 1;
		goto endflashing;
	}

	if (memcmp(tmp_reply, "FAIL", 4) == 0)
		max_download_size = 0;
	else
		sscanf(tmp_reply, "%u", &max_download_size);

	snprintf(tmp, sizeof(tmp), "getvar:product");
	if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
	{
		printf(" - Error writing command %s!\n", tmp);
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		ret = 1;
		goto endflashing;
	}

	snprintf(product, sizeof(product), "%s", tmp_reply);

	snprintf(tmp, sizeof(tmp), "getvar:version");
	if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
	{
		printf(" - Error writing command %s!\n", tmp);
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		ret = 1;
		goto endflashing;
	}

	snprintf(version, sizeof(version), "%s", tmp_reply);

	snprintf(tmp, sizeof(tmp), "getvar:version-bootloader");
	if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
	{
		printf(" - Error writing command %s!\n", tmp);
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		ret = 1;
		goto endflashing;
	}

	snprintf(version_bootloader, sizeof(version_bootloader), "%s", tmp_reply);

	snprintf(tmp, sizeof(tmp), "getvar:version-baseband");
	if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
	{
		printf(" - Error writing command %s!\n", tmp);
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		ret = 1;
		goto endflashing;
	}

	snprintf(version_baseband, sizeof(version_baseband), "%s", tmp_reply);

	snprintf(tmp, sizeof(tmp), "getvar:serialno");
	if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
	{
		printf(" - Error writing command %s!\n", tmp);
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		ret = 1;
		goto endflashing;
	}

	snprintf(serialno, sizeof(serialno), "%s", tmp_reply);

	snprintf(tmp, sizeof(tmp), "getvar:secure");
	if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
	{
		printf(" - Error writing command %s!\n", tmp);
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		ret = 1;
		goto endflashing;
	}

	snprintf(secure, sizeof(secure), "%s", tmp_reply);

	snprintf(tmp, sizeof(tmp), "getvar:Sector-size");
	if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
	{
		printf(" - Error writing command %s!\n", tmp);
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		ret = 1;
		goto endflashing;
	}

	if (memcmp(tmp_reply, "FAIL", 4) == 0)
		sector_size = 4096;
	else
		sscanf(tmp_reply, "%u", &sector_size);

	snprintf(tmp, sizeof(tmp), "getvar:Loader-version");
	if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
	{
		printf(" - Error writing command %s!\n", tmp);
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		ret = 1;
		goto endflashing;
	}

	snprintf(loader_version, sizeof(loader_version), "%s", tmp_reply);

	snprintf(tmp, sizeof(tmp), "getvar:Phone-id");
	if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
	{
		printf(" - Error writing command %s!\n", tmp);
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		ret = 1;
		goto endflashing;
	}

	snprintf(phone_id, sizeof(phone_id), "%s", tmp_reply);

	snprintf(tmp, sizeof(tmp), "getvar:Device-id");
	if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
	{
		printf(" - Error writing command %s!\n", tmp);
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		ret = 1;
		goto endflashing;
	}

	snprintf(device_id, sizeof(device_id), "%s", tmp_reply);

	snprintf(tmp, sizeof(tmp), "getvar:Platform-id");
	if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
	{
		printf(" - Error writing command %s!\n", tmp);
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		ret = 1;
		goto endflashing;
	}

	snprintf(platform_id, sizeof(platform_id), "%s", tmp_reply);

	snprintf(tmp, sizeof(tmp), "getvar:Rooting-status");
	if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
	{
		printf(" - Error writing command %s!\n", tmp);
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		ret = 1;
		goto endflashing;
	}

	snprintf(rooting_status, sizeof(rooting_status), "%s", tmp_reply);

	snprintf(tmp, sizeof(tmp), "getvar:Ufs-info");
	if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
	{
		printf(" - Error writing command %s!\n", tmp);
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		ret = 1;
		goto endflashing;
	}

	snprintf(ufs_info, sizeof(ufs_info), "%s", tmp_reply);

	snprintf(tmp, sizeof(tmp), "getvar:Emmc-info");
	if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
	{
		printf(" - Error writing command %s!\n", tmp);
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		ret = 1;
		goto endflashing;
	}

	snprintf(emmc_info, sizeof(emmc_info), "%s", tmp_reply);

	snprintf(tmp, sizeof(tmp), "getvar:Default-security");
	if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
	{
		printf(" - Error writing command %s!\n", tmp);
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		ret = 1;
		goto endflashing;
	}

	snprintf(default_security, sizeof(default_security), "%s", tmp_reply);

	snprintf(tmp, sizeof(tmp), "getvar:Keystore-counter");
	if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
	{
		printf(" - Error writing command %s!\n", tmp);
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		ret = 1;
		goto endflashing;
	}

	if (memcmp(tmp_reply, "FAIL", 4) == 0)
		keystore_counter = 0;
	else
		sscanf(tmp_reply, "%u", &keystore_counter);

	snprintf(tmp, sizeof(tmp), "getvar:Security-state");
	if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
	{
		printf(" - Error writing command %s!\n", tmp);
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		ret = 1;
		goto endflashing;
	}

	snprintf(security_state, sizeof(security_state), "%s", tmp_reply);

	snprintf(tmp, sizeof(tmp), "getvar:S1-root");
	if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
	{
		printf(" - Error writing command %s!\n", tmp);
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		ret = 1;
		goto endflashing;
	}

	snprintf(s1_root, sizeof(s1_root), "%s", tmp_reply);

	snprintf(tmp, sizeof(tmp), "getvar:Sake-root");
	if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
	{
		printf(" - Error writing command %s!\n", tmp);
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		ret = 1;
		goto endflashing;
	}

	snprintf(sake_root, sizeof(sake_root), "%s", tmp_reply);

	snprintf(tmp, sizeof(tmp), "Get-root-key-hash");
	if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
	{
		printf(" - Error writing command %s!\n", tmp);
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		ret = 1;
		goto endflashing;
	}

	if (memcmp(tmp_reply, "DATA", 4) != 0)
	{
		printf(" - Error, no DATA reply!\n");
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		ret = 1;
		goto endflashing;
	}

	memset(get_root_key_hash, 0, sizeof(get_root_key_hash));

	if (get_reply_len <= 0)
	{
		printf("Error receiving root key hash!\n");
		ret = 1;
		goto endflashing;
	}

	// sometimes OKAY reply is inside data buffer
	if (get_reply_len >= 4 && tmp_reply[get_reply_len - 4] == 'O' && tmp_reply[get_reply_len - 3] == 'K' && tmp_reply[get_reply_len - 2] == 'A' && tmp_reply[get_reply_len - 1] == 'Y')
	{
		get_reply_len -= 4;
		okay_replied = true;
	}

	for (i=0, j=0; i < (int)get_reply_len; ++i, j+=2)
	{
		sprintf(get_root_key_hash+j, "%02X", tmp_reply[i] & 0xff);
	}
	get_root_key_hash[j] = '\0';

	if (!okay_replied)
	{
		if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
		{
			ret = 1;
			goto endflashing;
		}

		if (memcmp(tmp_reply, "OKAY", 4) != 0)
		{
			printf(" - Error, no OKAY reply!\n");
			ret = 1;
			goto endflashing;
		}
	}

	okay_replied = false;

	snprintf(tmp, sizeof(tmp), "getvar:slot-count");
	if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
	{
		printf(" - Error writing command %s, ignore this error!\n", tmp);
	}
	else
	{
		if (get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
		{
			snprintf(slot_count, sizeof(slot_count), "%s", tmp_reply);

			snprintf(tmp, sizeof(tmp), "getvar:current-slot");
			if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
			{
				printf(" - Error writing command %s, ignore this error!\n", tmp);
			}
			else
			{
				if (get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
					snprintf(current_slot, sizeof(current_slot), "%s", tmp_reply);
			}
		}
	}

	snprintf(tmp, sizeof(tmp), "getvar:Battery");
	if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
	{
		printf(" - Error writing command %s!\n", tmp);
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		ret = 1;
		goto endflashing;
	}

	if (memcmp(tmp_reply, "FAIL", 4) != 0)
	{
		if (memcmp(tmp_reply, "OKAY", 4) == 0 && get_reply_len > 4)
		{
			sscanf(tmp_reply+4, "%d", &battery_level);
		}
		else
		{
			if (get_reply_len > 0)
				sscanf(tmp_reply, "%d", &battery_level);
		}
	}

	printf("Product: %s\n", product);
	printf("Version: %s\n", version);
	printf("Bootloader version: %s\n", version_bootloader);
	printf("Baseband version: %s\n", version_baseband);
	printf("Serialno: %s\n", serialno);
	printf("Secure: %s\n", secure);
	printf("Loader version: %s\n", loader_version);
	printf("Phone ID: %s\n", phone_id);
	printf("Device ID: %s\n", device_id);
	printf("Platform ID: %s\n", platform_id);
	printf("Max download size: %u\n", max_download_size);
	printf("Sector size: %u\n", sector_size);
	printf("Rooting status: %s\n", rooting_status);
	printf("Ufs info: %s\n", ufs_info);
	printf("Emmc info: %s\n", emmc_info);
	printf("Default security: %s\n", default_security);
	printf("Keystore counter: %u\n", keystore_counter);
	printf("Security state: %s\n", security_state);
	printf("Sake root: %s\n", sake_root);
	printf("S1 root: %s\n", s1_root);
	printf("Root key hash: %s\n", get_root_key_hash);
	printf("Slot count: %s\n", slot_count);
	printf("Current slot: %s\n", current_slot);
	printf("Battery level: %d%s\n", battery_level, (battery_level == 0) ? " unsupported command" : "");

	if (battery_level > 0)
	{
		if (battery_level < 15)
		{
			printf("\nYour battery level is %d percent and you have risk for hard brick in case your battery get fully discharged durring flash session!\n", battery_level);
			printf("Type 'y' and press ENTER if you understand the risk, or type 'n' to exit from flashing.\n");
			if (scanf(" %c", &ch)) { }
			if (ch == 'n' || ch == 'N')
			{
				goto endflashing;
			}
		}
	}

	if (slot_count[0] == '2')
	{
		/* flash bootloader,bluetooth,dsp,modem,rdimage to booth a,b slots */
		flash_booth_slots = true;
	}

/*======================================  put into flash mode  =======================================*/

	printf("\n");

	if (transfer_bulk_async(dev, EP_OUT, "download:00000001", 17, USB_TIMEOUT, 1) < 1) {
		printf("Error writing command 'go into flashmode'!\n");
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		printf(" - Error, no go_into_flash_mode DATA reply!\n");
		ret = 1;
		goto endflashing;
	}

	if (strlen(tmp_reply) != 12)
	{
		printf(" - Error, go_into_flash_mode DATA reply size: %zu less than expected: 12!\n", strlen(tmp_reply));
		ret = 1;
		goto endflashing;
	}

	if (memcmp(tmp_reply+4, "00000001", 8) != 0)
	{
		printf(" - Error, go_into_flash_mode DATA reply string: %s is not equal to expected: DATA00000001!\n", tmp_reply);
		ret = 1;
		goto endflashing;
	}

	if (transfer_bulk_async(dev, EP_OUT, "\x01", 1, USB_TIMEOUT, 1) < 1)
	{
		printf(" - Error writing 'go into flashmode' value 1!\n");
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		printf("      Error, no 'go into flashmode' OKAY reply!\n");
		ret = 1;
		goto endflashing;
	}

	if (strlen(tmp_reply) < 4)
	{
		printf("      Error, 'go into flashmode' reply less than 4, got: %zu bytes!\n", strlen(tmp_reply));
		ret = 1;
		goto endflashing;
	}

	if (memcmp(tmp_reply, "OKAY", 4) != 0)
	{
		printf("      Error, didn't got 'go into flashmode' OKAY reply! Got reply: %s\n", tmp_reply);
		ret = 1;
		goto endflashing;
	}

	if (transfer_bulk_async(dev, EP_OUT, "Write-TA:2:10100", 16, USB_TIMEOUT, 1) < 1)
	{
		printf("Error writing TA 'go into flashmode'!\n");
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		printf("      Error, no TA write 'go into flashmode' OKAY reply!\n");
		ret = 1;
		goto endflashing;
	}

	if (strlen(tmp_reply) < 4)
	{
		printf("      Error, TA write 'go into flashmode' reply less than 4, got: %zu bytes!\n", strlen(tmp_reply));
		ret = 1;
		goto endflashing;
	}

	if (memcmp(tmp_reply, "OKAY", 4) != 0)
	{
		printf("      Error, didn't got TA write 'go into flashmode' OKAY reply! Got reply: %s\n", tmp_reply);
		ret = 1;
		goto endflashing;
	}

	printf("Device is put now in flash mode.\n");

/*=======================================  process partition  ========================================*/

	printf("\n");
	sin_found = 0;
#ifdef _WIN32
	snprintf(tmp, sizeof(tmp), "%s\\partition", working_path);
#else
	snprintf(tmp, sizeof(tmp), "./partition");
#endif

	if ((dir = opendir(tmp)) != NULL)
	{
		printf("Repartitioning...\n");

		/* search for partition_delivery.xml */
#ifdef _WIN32
		snprintf(sinfil, sizeof(sinfil), "%s\\partition\\partition_delivery.xml", working_path);
#else
		snprintf(sinfil, sizeof(sinfil), "./partition/partition_delivery.xml");
#endif

		if (stat(sinfil, &filestat) < 0)
		{
			printf("partition_delivery.xml not exist in partition folder or no partition folder.\n");
		}
		else
		{
			unsigned long long lun0_sz = 0;

			printf("Found partition_delivery.xml in partition folder.\n");

			if (!parse_xml(sinfil))
			{
				printf("Error parsing partition_delivery.xml!\n");
				ret = 1;
				goto getoutofflashing;
			}

			if (strstr(ufs_info, "FAIL") == NULL)
			{
				have_ufs = true;
			}

			printf("Determining %s size...\n", have_ufs ? "LUN0" : "EMMC part 0");

			snprintf(tmp, sizeof(tmp), "%s", have_ufs ? "Get-ufs-info" : "Get-emmc-info");

			if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
			{
				printf(" - Error writing command %s!\n", tmp);
				ret = 1;
				goto getoutofflashing;
			}
			else
			{
				if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
				{
					ret = 1;
					goto getoutofflashing;
				}

				if (memcmp(tmp_reply, "DATA", 4) != 0)
				{
					printf(" - Error, no DATA reply!\n");
					ret = 1;
					goto getoutofflashing;
				}

				if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
				{
					ret = 1;
					goto getoutofflashing;
				}

				if (get_reply_len <= 0)
				{
					printf("Error receiving %s header!\n", have_ufs ? "UFS" : "EMMC");
					ret = 1;
					goto getoutofflashing;
				}
				else
				{
					if (have_ufs)
					{
						unsigned char ufs_desc_sz = 0;

						display_buffer_hex_ascii("UFS raw data", tmp_reply, get_reply_len);

						memcpy(&ufs_desc_sz, tmp_reply, 1);
						memcpy(&lun0_sz, tmp_reply + ufs_desc_sz + 0x1c, 4);

						lun0_sz = swap_uint32(lun0_sz);
						lun0_sz *= sector_size;
						lun0_sz /= 1024;
					}
					else
					{
						display_buffer_hex_ascii("EMMC raw data", tmp_reply, get_reply_len);

						memcpy(&lun0_sz, tmp_reply + 0xd4, 4);

						lun0_sz *= sector_size;
						lun0_sz /= 1024;
					}

					printf("%s size = %llu\n", have_ufs ? "LUN0" : "EMMC part 0", lun0_sz);
				}

				// sometimes OKAY reply is inside data buffer
				if (get_reply_len >= 4 && tmp_reply[get_reply_len - 4] == 'O' && tmp_reply[get_reply_len - 3] == 'K' && tmp_reply[get_reply_len - 2] == 'A' && tmp_reply[get_reply_len - 1] == 'Y')
				{
					get_reply_len -= 4;
					okay_replied = true;
				}

				if (!okay_replied)
				{
					if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
					{
						ret = 1;
						goto getoutofflashing;
					}

					if (memcmp(tmp_reply, "OKAY", 4) != 0)
					{
						printf(" - Error, no OKAY reply!\n");
						ret = 1;
						goto getoutofflashing;
					}
				}

				okay_replied = false;
			}

			if (lun0_sz)
			{
				for(i=0; i<pd; ++i)
				{
					char lun0[10];

					/* FIXME!
					 * temp solution since two diferent emmc csd info, I'm unable to calculate right, for idea:
					 * https://forum.xda-developers.com/t/tool-newflasher-xperia-command-line-flasher.3619426/post-84072849
					 * https://forum.xda-developers.com/t/tool-newflasher-xperia-command-line-flasher.3619426/post-84214113
					 */
					//lun0_sz -= lun0_sz % 10000;
					//lun0_sz /= 10000;

					snprintf(lun0, sizeof(lun0), "%llu", lun0_sz);

					printf("\n");
					printf("Processing %s\n", partitiondelivery_xml[i]);
#ifdef _WIN32
					snprintf(sinfil, sizeof(sinfil), "%s\\partition\\%s", working_path, partitiondelivery_xml[i]);
#else
					snprintf(sinfil, sizeof(sinfil), "./partition/%s", partitiondelivery_xml[i]);
#endif
					if ((strstr(sinfil, "LUN0") != NULL && strstr(sinfil, lun0) != NULL) ||
						 strstr(sinfil, "LUN0_X-FLASH-ALL") != NULL ||
						 strstr(sinfil, "LUN1") != NULL ||
						 strstr(sinfil, "LUN2") != NULL ||
						 strstr(sinfil, "LUN3") != NULL)
					{
						fi = fopen64(sinfil, "rb");
						if (fi == NULL)
						{
							printf(" - unable to open %s.\n", sinfil);
							sin_found = 0;
						}
						else
						{
							fseeko64(fi, 0, SEEK_SET);
							fread_unus_res(file_format, 1, 2, fi);
							if (fi) fclose(fi);
							sin_found = 1;

							if (memcmp(file_format, "\x1F\x8B", 2) == 0)
							{
								FILE *a = NULL;
#ifdef _WIN32
								snprintf(fld, sizeof(fld), "%s\\partition\\converted.file", working_path);
#else
								snprintf(fld, sizeof(fld), "./partition/converted.file");
#endif
								if (gunziper(sinfil, fld))
								{
									ret = 1;
									goto getoutofflashing;
								}

								a = fopen64(fld, "rb");
								if (a == NULL)
								{
									printf(" - Unable to open %s\n", fld);
								}
								else
								{
									if (!process_sins(dev, a, sinfil, working_path, "partition", "Repartition"))
									{
										fclose(a);
										remove(fld);
										closedir(dir);
										ret = 1;
										goto getoutofflashing;
									}
									fclose(a);
								}

								remove(fld);
							}
							else
							{
								FILE *a = NULL;

								a = fopen64(sinfil, "rb");
								if (a == NULL)
								{
									printf(" - Unable to open %s\n", sinfil);
								}
								else
								{
									if (!process_sins(dev, a, sinfil, working_path, "partition", "Repartition"))
									{
										fclose(a);
										remove(fld);
										closedir(dir);
										ret = 1;
										goto getoutofflashing;
									}
									fclose(a);
								}
							}
						}
					}
					else
					{
						printf("Skipping %s\n", partitiondelivery_xml[i]);
					}
				}
			}
		}

		if (pd == 0)
		{
			while ((ep = readdir(dir)) != NULL)
			{
				/*if (ep->d_type == DT_REG)*/
				{
					if (strcmp(ep->d_name, ".") != 0 && strcmp(ep->d_name, "..") != 0)
					{
						if ((extension = strrchr(ep->d_name, '.')) != NULL)
						{
							if (strcmp(extension, ".sin") == 0)
							{
								sin_found = 1;
								printf("\n");
								printf("Processing %s\n", ep->d_name);
#ifdef _WIN32
								snprintf(sinfil, sizeof(sinfil), "%s\\partition\\%s", working_path, ep->d_name);
#else
								snprintf(sinfil, sizeof(sinfil), "./partition/%s", ep->d_name);
#endif
								if (!strlen(sinfil))
								{
									printf("Oops!!! Sinfile name empty!\n");
									ret = 1;
									goto getoutofflashing;
								}

								if (strstr(sinfil, "artition") == NULL)
								{
									printf("Oops!! Found non partition sin file!\n");
									printf("Please read instructions carefully if you no want brick!\n");
									printf("Skipping non partition %s file.\n", sinfil);
									sin_found = 0;
								}
								else
								{
									fi = fopen64(sinfil, "rb");
									if (fi == NULL) {
										printf(" - unable to open %s!\n", sinfil);
										ret = 1;
										goto getoutofflashing;
									}
									fseeko64(fi, 0, SEEK_SET);
									fread_unus_res(file_format, 1, 2, fi);
									if (fi) fclose(fi);

									if (memcmp(file_format, "\x1F\x8B", 2) == 0)
									{
										FILE *a = NULL;
#ifdef _WIN32
										snprintf(fld, sizeof(fld), "%s\\partition\\converted.file", working_path);
#else
										snprintf(fld, sizeof(fld), "./partition/converted.file");
#endif
										if (gunziper(sinfil, fld))
										{
											ret = 1;
											goto getoutofflashing;
										}

										a = fopen64(fld, "rb");
										if (a == NULL)
										{
											printf(" - Unable to open %s\n", fld);
										}
										else
										{
											if (!process_sins(dev, a, sinfil, working_path, "partition", "Repartition"))
											{
												fclose(a);
												remove(fld);
												closedir(dir);
												ret = 1;
												goto getoutofflashing;
											}
											fclose(a);
										}

										remove(fld);
									}
									else
									{
										FILE *a = NULL;

										a = fopen64(sinfil, "rb");
										if (a == NULL)
										{
											printf(" - Unable to open %s\n", sinfil);
										}
										else
										{
											if (!process_sins(dev, a, sinfil, working_path, "partition", "Repartition"))
											{
												fclose(a);
												remove(fld);
												closedir(dir);
												ret = 1;
												goto getoutofflashing;
											}
											fclose(a);
										}
									}
								}
							}
						}
					}
				}
			}
		}

		closedir(dir);
	}

	if (!sin_found)
	{
		printf("No .sin files in partition dir...\n");
		printf("You must extract partition.zip into 'partition' folder if you want flash partition image!\n");
		printf("On 2018 and UP models you must move partition sin files to 'partition' folder if you need flash partition images!\n");
	}
	else
	{
		something_flashed = 1;
	}

/*=======================================  process .sin files  =======================================*/

	printf("\n");
	sin_found = 0;
	printf("Processing .sin files...\n");
	snprintf(fld, sizeof(fld), "flash_session/");
	if (0 != access(fld, F_OK))
	{
		if (ENOENT == errno)
		{
#ifdef _WIN32
			fld_cbck = mkdir("flash_session");
#else
			fld_cbck = mkdir("flash_session", 0755);
#endif
			if (fld_cbck == 0)
			{
				printf("Created ouput folder flash_session\n");
				flash_session_created = true;
			}
			else
			{
				printf("FAILURE to create output folder flash_session!\n");
				ret = 1;
				goto getoutofflashing;
			}
		}

		if (ENOTDIR == errno)
		{
			printf("FAILURE to create output folder flash_session because there is file called flash_session!!!\n"
				"Remove or rename file flash_session first!\n");
			ret = 1;
			goto getoutofflashing;
		}

	}
	else
	{
		printf("Using existing folder flash_session\n");
		flash_session_created = true;
	}

	if ((dir = opendir(working_path)) != NULL)
	{
		while ((ep = readdir(dir)) != NULL)
		{
			/*if (ep->d_type == DT_REG)*/
			{
				if (strcmp(ep->d_name, ".") != 0 && strcmp(ep->d_name, "..") != 0)
				{
					if ((extension = strrchr(ep->d_name, '.')) != NULL)
					{
						/* search for .sin files */
						if (strcmp(extension, ".sin") == 0)
						{
							int file_is_skip = 0;

							/* skip Partition or partition sin */
							if (strstr(ep->d_name, "artition") != NULL)
							{
								file_is_skip = 1;
								goto skip_this;
							}

							/* prompt for Persist or persist partition flash */
							if (strstr(ep->d_name, "ersist") != NULL)
							{
								//printf("\nRecommended step to skip this! Type 'y' and press ENTER if you want flash persist partition, or type 'n' and press ENTER to skip.\n");
								//printf("More info https://forum.xda-developers.com/xperia-xz1-compact/help/android-attest-key-lost-bootloader-t3829945\n");
								//if (scanf(" %c", &ch)) { }

								//if (ch == 'n' || ch == 'N')
								//{
									file_is_skip = 1;
									goto skip_this;
								//}
							}

							sin_found = 1;
							printf("\n");
							printf("Processing %s\n", ep->d_name);
#ifdef _WIN32
							snprintf(sinfil, sizeof(sinfil), "%s\\%s", working_path, ep->d_name);
#else
							snprintf(sinfil, sizeof(sinfil), "./%s", ep->d_name);
#endif
							if (!strlen(sinfil))
							{
								printf("Oops!!! Sinfile name empty!\n");
								ret = 1;
								goto getoutofflashing;
							}

							fi = fopen64(sinfil, "rb");
							if (fi == NULL)
							{
								printf(" - unable to open %s!\n", sinfil);
								ret = 1;
								goto getoutofflashing;
							}
							fseeko64(fi, 0, SEEK_SET);
							fread_unus_res(file_format, 1, 2, fi);
							if (fi) fclose(fi);

							if (memcmp(file_format, "\x1F\x8B", 2) == 0)
							{
							   	FILE *a = NULL;
#ifdef _WIN32
								snprintf(fld, sizeof(fld), "%s\\flash_session\\converted.file", working_path);
#else
								snprintf(fld, sizeof(fld), "./flash_session/converted.file");
#endif
								if (gunziper(sinfil, fld))
								{
									ret = 1;
									goto getoutofflashing;
								}

								a = fopen64(fld, "rb");
								if (a == NULL)
								{
									printf(" - Unable to open %s\n", fld);
								}
								else
								{
									/*============= flash booth a,b slots =============*/

									if (flash_booth_slots)
									{
										memset(searchfor, 0, sizeof(searchfor));

										if (fread(searchfor, 1, 32, a) < 32)
										{
											fclose(a);
											remove(fld);
											closedir(dir);
											ret = 1;
											goto getoutofflashing;
										}
										else
										{
											fseeko64(a, 0, SEEK_SET);

											if (memcmp(searchfor, "bootloader.", 11) == 0 ||
												 memcmp(searchfor, "bluetooth.", 10) == 0 ||
												 memcmp(searchfor, "dsp.", 4) == 0 ||
												 memcmp(searchfor, "modem.", 6) == 0 ||
												 memcmp(searchfor, "rdimage.", 8) == 0)
											{
												memcpy(remember_current_slot, current_slot, 1);

												if (memcmp(current_slot, "a", 1) == 0)
												{
													memcpy(current_slot, "b", 1);
												}
												else
												{
													if (memcmp(current_slot, "b", 1) == 0)
														memcpy(current_slot, "a", 1);
												}

												if (!process_sins(dev, a, sinfil, working_path, "flash_session", "flash"))
												{
													fclose(a);
													remove(fld);
													closedir(dir);
													ret = 1;
													goto getoutofflashing;
												}

												memcpy(current_slot, remember_current_slot, 1);

												fseeko64(a, 0, SEEK_SET);
											}
										}
									}

									/*==============================================*/

									if (!process_sins(dev, a, sinfil, working_path, "flash_session", "flash"))
									{
										fclose(a);
										remove(fld);
										closedir(dir);
										ret = 1;
										goto getoutofflashing;
									}

									fclose(a);
								}

								remove(fld);
							}
							else
							{
								FILE *a = NULL;

								a = fopen64(sinfil, "rb");
								if (a == NULL)
								{
									printf(" - Unable to open %s\n", sinfil);
								}
								else
								{
									/*============= flash booth a,b slots =============*/

									if (flash_booth_slots)
									{
										memset(searchfor, 0, sizeof(searchfor));

										if (fread(searchfor, 1, 32, a) < 32)
										{
											fclose(a);
											remove(fld);
											closedir(dir);
											ret = 1;
											goto getoutofflashing;
										}
										else
										{
											fseeko64(a, 0, SEEK_SET);

											if (memcmp(searchfor, "bootloader.", 11) == 0 ||
												 memcmp(searchfor, "bluetooth.", 10) == 0 ||
												 memcmp(searchfor, "dsp.", 4) == 0 ||
												 memcmp(searchfor, "modem.", 6) == 0 ||
												 memcmp(searchfor, "rdimage.", 8) == 0)
											{
												memcpy(remember_current_slot, current_slot, 1);

												if (memcmp(current_slot, "a", 1) == 0)
												{
													memcpy(current_slot, "b", 1);
												}
												else
												{
													if (memcmp(current_slot, "b", 1) == 0)
														memcpy(current_slot, "a", 1);
												}

												if (!process_sins(dev, a, sinfil, working_path, "flash_session", "flash"))
												{
													fclose(a);
													remove(fld);
													closedir(dir);
													ret = 1;
													goto getoutofflashing;
												}

												memcpy(current_slot, remember_current_slot, 1);

												fseeko64(a, 0, SEEK_SET);
											}
										}
									}

									/*==============================================*/

									if (!process_sins(dev, a, sinfil, working_path, "flash_session", "flash"))
									{
										fclose(a);
										remove(fld);
										closedir(dir);
										ret = 1;
										goto getoutofflashing;
									}

									fclose(a);
								}
							}
skip_this:
							if (file_is_skip)
								printf("Skipping %s\n", ep->d_name);
						}
					}
				}
			}
		}
		closedir(dir);
	}

	if (!sin_found)
		printf("No .sin files in current dir.\n");
	else
		something_flashed = 1;

/*=======================================  process .ta files  =======================================*/

	printf("\n");
	sin_found = 0;
	printf("Processing .ta files...\n");
	if ((dir = opendir(working_path)) != NULL)
	{
		while ((ep = readdir(dir)) != NULL)
		{
			/*if (ep->d_type == DT_REG)*/
			{
				if (strcmp(ep->d_name, ".") != 0 && strcmp(ep->d_name, "..") != 0)
				{
					if ((extension = strrchr(ep->d_name, '.')) != NULL)
					{
						if (strcmp(extension, ".ta") == 0)
						{
							sin_found = 1;
							printf("\n");

							if (!proced_ta_file(ep->d_name, dev))
							{
								closedir(dir);
								ret = 1;
								goto getoutofflashing;
							}
						}
					}
				}
			}
		}
		closedir(dir);
	}

	if (!sin_found)
		printf("No .ta files in current dir.\n");
	else
		something_flashed = 1;

/*========================================  boot delivery  ===========================================*/

	printf("\n");
	printf("Processing boot delivery...\n\n");
#ifdef _WIN32
	snprintf(sinfil, sizeof(sinfil), "%s\\boot\\boot_delivery.xml", working_path);
#else
	snprintf(sinfil, sizeof(sinfil), "./boot/boot_delivery.xml");
#endif
	if (stat(sinfil, &filestat) < 0)
	{
		printf("boot_delivery.xml not exist in boot folder or no boot folder.\n");
		ret = 1;
		goto getoutofflashing;
	}
	else
		printf("Found boot_delivery.xml in boot folder.\n");

	if (!parse_xml(sinfil))
	{
		printf("Error parsing boot_delivery.xml!\n");
		ret = 1;
		goto getoutofflashing;
	}

	if (!strlen(bootdelivery_version))
	{
		printf(" - Unable to determine boot delivery version, skipping bootdelivery.\n");
		ret = 1;
		goto getoutofflashing;
	}

	printf(" - Boot delivery version: %s\n", bootdelivery_version);
	printf(" - Verifying if boot delivery match with device...\n");

	if (strstr(default_security, "OFF") != NULL)
		snprintf(searchfor, sizeof(searchfor), "DEFAULT_SECURITY=\"OFF\"");
	else
	{
		memcpy(platform_id, "00", 2);
		snprintf(searchfor, sizeof(searchfor), "PLATFORM_ID=\"%s\";PLF_ROOT_HASH=\"%s\"", platform_id, get_root_key_hash);
	}

	printf("      searching for: %s\n", searchfor);

	for(i=0; i<td1; ++i)
	{
		for (j=2; j<13; ++j)
		{
			if (strlen(bootdelivery_xml[i][j]) != 0)
			{
				if (j == 2)
				{
					/*printf("%d: %s\n", j, bootdelivery_xml[i][1]);*/
					if (strstr(bootdelivery_xml[i][1], searchfor) != NULL)
						bootdelivery_found = 1;
				}

				if (bootdelivery_found && strstr(bootdelivery_xml[i][1], searchfor) != NULL)
				{
					if (j == 2) printf("      Found bootdelivery match: %s\n", bootdelivery_xml[i][0]);
					if (j == 3)
					{
						printf("      TA file: %s\n", bootdelivery_xml[i][j]);
						snprintf(tmp, sizeof(tmp), "./boot/%s", bootdelivery_xml[i][j]);
						printf("\n");
						if (!proced_ta_file(tmp, dev))
						{
							ret = 1;
							goto getoutofflashing;
						}
					}
					if (j == 4)
					{
						printf("      SIN file: %s\n", bootdelivery_xml[i][j]);
						printf("\n");
						printf("Processing %s\n", bootdelivery_xml[i][j]);
#ifdef _WIN32
						snprintf(sinfil, sizeof(sinfil), "%s\\boot\\%s", working_path, bootdelivery_xml[i][j]);
#else
						snprintf(sinfil, sizeof(sinfil), "./boot/%s", bootdelivery_xml[i][j]);
#endif

						if (!strlen(sinfil))
						{
							printf("Oops!!! Sinfile name empty!\n");
							ret = 1;
							goto getoutofflashing;
						}

						if (strstr(sinfil, "bootloader") == NULL)
						{
							printf("Oops!! Found non bootloader sin file!\n");
							printf("Please read instructions carefully if you no want brick!\n");
							printf("Skipping non bootloader %s file.\n", sinfil);
						}
						else
						{
							fi = fopen64(sinfil, "rb");
							if (fi == NULL)
							{
								printf(" - unable to open %s!\n", sinfil);
								ret = 1;
								goto getoutofflashing;
							}
							fseeko64(fi, 0, SEEK_SET);
							fread_unus_res(file_format, 1, 2, fi);
							if (fi) fclose(fi);

							if (memcmp(file_format, "\x1F\x8B", 2) == 0)
							{
								FILE *a = NULL;
#ifdef _WIN32
								snprintf(fld, sizeof(fld), "%s\\boot\\converted.file", working_path);
#else
								snprintf(fld, sizeof(fld), "./boot/converted.file");
#endif
								if (gunziper(sinfil, fld))
								{
									ret = 1;
									goto getoutofflashing;
								}

								a = fopen64(fld, "rb");
								if (a == NULL)
								{
									printf(" - Unable to open %s\n", fld);
								}
								else
								{
									if (!process_sins(dev, a, sinfil, working_path, "boot", "flash"))
									{
										fclose(a);
										remove(fld);
										ret = 1;
										goto getoutofflashing;
									}
									fclose(a);
								}

								remove(fld);
							}
							else
							{
								FILE *a = NULL;

								a = fopen64(sinfil, "rb");
								if (a == NULL)
								{
									printf(" - Unable to open %s\n", sinfil);
								}
								else
								{
									if (!process_sins(dev, a, sinfil, working_path, "boot", "flash"))
									{
										fclose(a);
										remove(fld);
										closedir(dir);
										ret = 1;
										goto getoutofflashing;
									}
									fclose(a);
								}
							}
						}
					}
				}
			}
		}
	}

	if (!bootdelivery_found)
		printf("Didn't found bootdelivery that match your device!\n");
	else
		something_flashed = 1;


getoutofflashing:

/*=========================================    bootloader log    ========================================*/

	snprintf(tmp, sizeof(tmp), "Getlog");
	if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
	{
		printf("Error writing commad: %s\n", tmp);
		ret = 1;
		goto slot_setup;
	}
	else
	{
		if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
		{
			printf("Error, null reply\n");
			ret = 1;
			goto slot_setup;
		}
		else
		{
			if (memcmp(tmp_reply, "FAIL", 4) == 0)
			{
				printf("got fail reply: %s\n", tmp_reply);
				ret = 1;
				goto slot_setup;
			}
			else
			{
				if (memcmp(tmp_reply, "DATA", 4) == 0)
				{
					unsigned int data_len = 0;

					if (get_reply_len != 12) {
						printf("Errornous DATA reply!\n");
						display_buffer_hex_ascii("replied", tmp_reply, get_reply_len);
						ret = 1;
						goto slot_setup;
					}

					sscanf(tmp_reply+4, "%08x", &data_len);

					if (!data_len)
					{
						printf("got null data_len!\n");

						if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
						{
							printf("Error retrieving seccond reply!\n");
							ret = 1;
							goto slot_setup;
						}

						if (strstr(tmp_reply, "OKAY") == NULL)
						{
							printf("Error, no OKAY reply!\n");
							display_buffer_hex_ascii("got reply", tmp_reply, get_reply_len);
							ret = 1;
							goto slot_setup;
						}
					}
					else
					{
						char *data_buf = NULL;

						if ((data_buf = (char *)malloc(data_len)) == NULL)
						{
							printf("error allocating 0x%x bytes!\n", data_len);
							ret = 1;
							goto slot_setup;
						}

						if (!get_reply(dev, EP_IN, data_buf, data_len, USB_TIMEOUT, 1))
						{
							printf("Error retrieving data!\n");
							free(data_buf);
							ret = 1;
							goto slot_setup;
						}

						printf("\n=========== BOOTLOADER LOG =============\n");
						printf("%s\n", data_buf);
						printf("=================================================\n");


						// sometimes OKAY reply is inside data buffer
						if (data_len >= 4 && data_buf[data_len - 4] == 'O' && data_buf[data_len - 3] == 'K' && data_buf[data_len - 2] == 'A' && data_buf[data_len - 1] == 'Y')
						{
							data_len -= 4;
						}
						else
						{
							if (!get_reply(dev, EP_IN, tmp, 5, USB_TIMEOUT, 0))
							{
								printf("Error retrieving OKAY reply!\n");
								free(data_buf);
								ret = 1;
								goto slot_setup;
							}

							if (strstr(tmp_reply, "OKAY") == NULL)
							{
								printf("Error, no OKAY reply!\n");
								free(data_buf);
								ret = 1;
								goto slot_setup;
							}
						}

						free(data_buf);
					}
				}
			}
		}
	}

/*=========================================  firmwares history log    ========================================*/

	snprintf(tmp, sizeof(tmp), "Read-TA:2:2475");
	if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
	{
		printf("Error writing commad: %s\n", tmp);
		ret = 1;
		goto slot_setup;
	}
	else
	{
		if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
		{
			printf("Error, null reply\n");
			ret = 1;
			goto slot_setup;
		}
		else
		{
			if (memcmp(tmp_reply, "FAIL", 4) == 0)
			{
				printf("got fail reply: %s\n", tmp_reply);
				ret = 1;
				goto slot_setup;
			}
			else
			{
				if (memcmp(tmp_reply, "DATA", 4) == 0)
				{
					unsigned int data_len = 0;

					if (get_reply_len != 12) {
						printf("Errornous DATA reply!\n");
						display_buffer_hex_ascii("replied", tmp_reply, get_reply_len);
						ret = 1;
						goto slot_setup;
					}

					sscanf(tmp_reply+4, "%08x", &data_len);

					if (!data_len)
					{
						printf("got null data_len!\n");

						if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
						{
							printf("Error retrieving seccond reply!\n");
							ret = 1;
							goto slot_setup;
						}

						if (strstr(tmp_reply, "OKAY") == NULL)
						{
							printf("Error, no OKAY reply!\n");
							display_buffer_hex_ascii("got reply", tmp_reply, get_reply_len);
							ret = 1;
							goto slot_setup;
						}
					}
					else
					{
						char *data_buf = NULL;

						if ((data_buf = (char *)malloc(data_len)) == NULL)
						{
							printf("error allocating 0x%x bytes!\n", data_len);
							ret = 1;
							goto slot_setup;
						}

						if (!get_reply(dev, EP_IN, data_buf, data_len, USB_TIMEOUT, 1))
						{
							printf("Error retrieving data!\n");
							free(data_buf);
							ret = 1;
							goto slot_setup;
						}

						printf("\n=========== FIRMWARES HISTORY LOG =============\n");
						printf("%s\n", data_buf);
						printf("=================================================\n");


						// sometimes OKAY reply is inside data buffer
						if (data_len >= 4 && data_buf[data_len - 4] == 'O' && data_buf[data_len - 3] == 'K' && data_buf[data_len - 2] == 'A' && data_buf[data_len - 1] == 'Y')
						{
							data_len -= 4;
						}
						else
						{
							if (!get_reply(dev, EP_IN, tmp, 5, USB_TIMEOUT, 0))
							{
								printf("Error retrieving OKAY reply!\n");
								free(data_buf);
								ret = 1;
								goto slot_setup;
							}

							if (strstr(tmp_reply, "OKAY") == NULL)
							{
								printf("Error, no OKAY reply!\n");
								free(data_buf);
								ret = 1;
								goto slot_setup;
							}
						}

						free(data_buf);
					}
				}
			}
		}
	}

/*=========================================  set slot active  ========================================*/

slot_setup:

	printf("\n");

	if (memcmp(current_slot, "a", 1) == 0 || memcmp(current_slot, "b", 1) == 0)
	{
		snprintf(tmp, sizeof(tmp), "set_active:%s", current_slot);

		if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
		{
				printf("Error writing command '%s'!\n", tmp);
				ret = 1;
				goto endflashing;
		}

		if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
		{
			printf(" - Error, no set_active:%s OKAY reply!\n", current_slot);
			ret = 1;
			goto endflashing;
		}

		if (strlen(tmp_reply) < 4)
		{
			printf("      Error, 'set_active:%s' reply less than 4, got: %zu bytes!\n", current_slot, strlen(tmp_reply));
			ret = 1;
			goto endflashing;
		}

		if (memcmp(tmp_reply, "OKAY", 4) != 0)
		{
			printf("      Error, didn't got 'set_active:%s' OKAY reply! Got reply: %s\n", current_slot, tmp_reply);
			ret = 1;
			goto endflashing;
		}

		printf("Set slot '%s' active.\n", current_slot);
	}

/*=====================================  get out of flash mode  ======================================*/

	printf("\n");

	if (transfer_bulk_async(dev, EP_OUT, "download:00000001", 17, USB_TIMEOUT, 1) < 1)
	{
		printf("Error writing command 'go out of flashmode'!\n");
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		printf(" - Error, no go_outof_flash_mode DATA reply!\n");
		ret = 1;
		goto endflashing;
	}

	if (strlen(tmp_reply) != 12)
	{
		printf(" - Error, go_outof_flash_mode DATA reply size: %zu less than expected: 12!\n", strlen(tmp_reply));
		ret = 1;
		goto endflashing;
	}

	if (memcmp(tmp_reply+4, "00000001", 8) != 0)
	{
		printf(" - Error, go_outof_flash_mode DATA reply string: %s is not equal to expected: DATA00000001!\n", tmp_reply);
		ret = 1;
		goto endflashing;
	}

	if (transfer_bulk_async(dev, EP_OUT, "\x00", 1, USB_TIMEOUT, 1) < 1)
	{
		printf(" - Error writing 'go out of flashmode' value 0!\n");
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		printf("      Error, no 'go out of flashmode' OKAY reply!\n");
		ret = 1;
		goto endflashing;
	}

	if (strlen(tmp_reply) < 4)
	{
		printf("      Error, 'go out of flashmode' reply less than 4, got: %zu bytes!\n", strlen(tmp_reply));
		ret = 1;
		goto endflashing;
	}

	if (memcmp(tmp_reply, "OKAY", 4) != 0)
	{
		printf("      Error, didn't got 'go out of flashmode' OKAY reply! Got reply: %s\n", tmp_reply);
		ret = 1;
		goto endflashing;
	}

	if (transfer_bulk_async(dev, EP_OUT, "Write-TA:2:10100", 16, USB_TIMEOUT, 1) < 1)
	{
		printf("Error writing TA 'go out of flashmode'!\n");
		ret = 1;
		goto endflashing;
	}

	if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
	{
		printf("      Error, no TA write 'go out of flashmode' OKAY reply!\n");
		ret = 1;
		goto endflashing;
	}

	if (strlen(tmp_reply) < 4)
	{
		printf("      Error, TA write 'go out of flashmode' reply less than 4, got: %zu bytes!\n", strlen(tmp_reply));
		ret = 1;
		goto endflashing;
	}

	if (memcmp(tmp_reply, "OKAY", 4) != 0)
	{
		printf("      Error, didn't got TA write 'go out of flashmode' OKAY reply! Got reply: %s\n", tmp_reply);
		ret = 1;
		goto endflashing;
	}

	printf("Device is put now out of flash mode.\n");

/*============================================  finish  ==============================================*/

endflashing:

	if (something_flashed)
	{
		snprintf(tmp, sizeof(tmp), "Sync");
		if (transfer_bulk_async(dev, EP_OUT, tmp, strlen(tmp), USB_TIMEOUT, 1) < 1)
		{
			printf(" - Error writing command %s!\n", tmp);
			ret = 1;
			goto release;
		}
		printf("Sent command: Sync\n");
#if 1
		if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
		{
			printf(" error, no sync response!\n");
			ret = 1;
			goto release;
		}

#endif
	}

	if (argc < 2)
	{
#if 1
		switch(reboot_mode)
		{
			case 1:
				snprintf(reboot_string, sizeof(reboot_string), "continue");
				break;

			case 2:
				snprintf(reboot_string, sizeof(reboot_string), "reboot-bootloader");
				break;

			case 3:
				snprintf(reboot_string, sizeof(reboot_string), "reboot");
				break;

			case 0:
			default:
				snprintf(reboot_string, sizeof(reboot_string), "powerdown");
				break;
		}

		if (transfer_bulk_async(dev, EP_OUT, reboot_string, strlen(reboot_string), USB_TIMEOUT, 1) < 1)
		{
			printf(" - Error writing command %s!\n", reboot_string);
			ret = 1;
			goto release;
		}
		printf("Sent command: %s.\n", reboot_string);

		if (!get_reply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT, 0))
		{
			printf("Error, no %s response!\n", reboot_string);
			ret = 1;
			goto release;
		}
#endif
		if (reboot_mode == 0)
		{
			printf("\nEnd. You can disconnect your device when you close %s\n", progname);
		}
		else
		{
			//display_buffer_hex_ascii("reboot mode response", tmp_reply, get_reply_len);
			printf("\nDone.\n");
		}
	}

release:

#ifdef __APPLE__
	for (iface=0; iface < nb_ifaces; ++iface)
	{
		printf("Releasing interface %d.\n", iface);
		libusb_release_interface(dev, iface);
	}

	if (iface_detached >= 0)
	{
		printf("Re-attaching kernel driver.\n");
		libusb_attach_kernel_driver(dev, iface_detached);
	}
#endif
	printf("Closing device.\n");
	CloseHandle(dev);
	SetupDiDestroyDeviceInfoList(hDevInfo);

#ifdef __APPLE__
	libusb_exit(NULL);
#endif

pauza:

memset(tmp_reply, 0, BUFF_MAX);

if (flash_session_created)
	rmdir("flash_session");

#ifdef _WIN32
	system("pause");
#endif
	return ret;
}

