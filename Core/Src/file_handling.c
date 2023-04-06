/*
 * file_handling.c
 *
 *  Created on: 26-Jul-2019
 *      Author: arunr
 */

#include "file_handling.h"

//#include "RingBuffer/UartRingbuffer.h"

FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

char buffer[BUFFER_SIZE];  // to store strings..
char path[PATH_SIZE];  // buffer to store path

int i = 0;

int bufsize(char *buf)
{
	int i = 0;
	while (*buf++ != '\0')
		i++;
	return i;
}

void clear_buffer(void)
{
	for (int i = 0; i < BUFFER_SIZE; i++)
		buffer[i] = '\0';
}

void clear_path(void)
{
	for (int i = 0; i < PATH_SIZE; i++)
		path[i] = '\0';
}

int cmdlength(char *str)
{
	int i = 0;
	while (*str++ != ' ')
		i++;
	return i;
}
void get_path(void)
{
	int start = cmdlength(buffer) + 1;
	int end = bufsize(buffer) - 2;

	int j = 0;
	for (int i = start; i < end; i++)
	{
		if (buffer[i] != ' ')
			path[j++] = buffer[i];
		else
			break;
	}
}

uint8_t mount_sd(void)
{
	fresult = f_mount(&fs, SDPath, 0);
	if (fresult != FR_OK)
		{printf("error in mounting SD CARD...\n");
		return 0;}
		printf("SD CARD mounted successfully...\n");
		return 1;
}

void unmount_sd(void)
{
	fresult = f_mount(NULL, "/", 1);
	if (fresult == FR_OK)
		printf("SD CARD UNMOUNTED successfully...\n");
	else
		printf("error!!! in UNMOUNTING SD CARD\n");
}

/* Start node to be scanned (***also used as work area***) */
FRESULT scan_files(char *pat)
{
	DIR dir;
	UINT i;

	char path[20];
	sprintf(path, "%s", pat);

	fresult = f_opendir(&dir, path); /* Open the directory */
	if (fresult == FR_OK)
	{
		for (;;)
		{
			fresult = f_readdir(&dir, &fno); /* Read a directory item */
			if (fresult != FR_OK || fno.fname[0] == 0)
				break; /* Break on error or end of dir */
			if (fno.fattrib & AM_DIR) /* It is a directory */
			{
				if (!(strcmp("SYSTEM~1", fno.fname)))
					continue;
				sprintf(buffer, "Dir: %s\r\n", fno.fname);
				printf(buffer);
				i = strlen(path);
				sprintf(&path[i], "/%s", fno.fname);
				fresult = scan_files(path); /* Enter the directory */
				if (fresult != FR_OK)
					break;
				path[i] = 0;
			}
			else
			{ /* It is a file. */
				sprintf(buffer, "File: %s/%s\n", path, fno.fname);
				printf(buffer);
			}
		}
		f_closedir(&dir);
	}
	return fresult;
}

FILE_State write_file(char *name, char *buf)
{

	/**** check whether the file exists or not ****/
	fresult = f_stat(name, &fno);
	if (fresult != FR_OK)
	{
#ifdef DEBUG_SD
		sprintf(buffer, "*%s* does not exists\n", name);
		printf(buffer);
#endif
		return FILE_NOT_EXIST;
	}

	else
	{
		/* Create a file with read write access and open it */
		fresult = f_open(&fil, name, FA_OPEN_EXISTING | FA_WRITE);
		if (fresult != FR_OK)
		{
#ifdef DEBUG_SD

			sprintf(buffer, "error no %d in opening file *%s*\n", fresult,
					name);
			printf(buffer);
#endif
			return FILE_OPEN_ERR;
		}

		/* Writing text */

		fresult = f_write(&fil, buf, bufsize(buf), &bw);

		if (fresult != FR_OK)
		{
			clear_buffer();
#ifdef DEBUG_SD
			sprintf(buffer, "error no %d in writing file *%s*\n", fresult,
					name);
			printf(buffer);
#endif
			fresult = f_close(&fil);
			if (fresult != FR_OK)
			{
				return FILE_WRITE_CLOSE_ERR;
			}
		}

		else
		{
			clear_buffer();
#ifdef DEBUG_SD

			sprintf(buffer, "*%s* written successfully\n", name);
			printf(buffer);
#endif

		}

		/* Close file */
		fresult = f_close(&fil);
		if (fresult != FR_OK)
		{
#ifdef DEBUG_SD
			sprintf(buffer, "error no %d in closing file *%s*\n", fresult,
					name);
			printf(buffer);
#endif
			return FILE_CLOSE_ERR;
		}
		return FILE_WRITTEN;
	}
}

FILE_State read_file(char *name, char* dataBuffer)
{
	/**** check whether the file exists or not ****/
	fresult = f_stat(name, &fno);
	if (fresult != FR_OK)
	{
#ifdef DEBUG_SD
		sprintf(buffer, "*%s* does not exists\n", name);
		printf(buffer);
#endif
		return FILE_NOT_EXIST;
	}

	else
	{
		/* Open file to read */
		fresult = f_open(&fil, name, FA_READ);

		if (fresult != FR_OK)
		{
#ifdef DEBUG_SD
			sprintf(buffer, "error no %d in opening file *%s*\n", fresult,
					name);
			printf(buffer);
#endif
			return FILE_OPEN_ERR;
		}
#ifdef DEBUG_SD
		/* Read data from the file
		 * see the function details for the arguments */
		sprintf(buffer, "reading data from the file *%s*\n", name);
		printf(buffer);

#endif

		fresult = f_read(&fil, buffer, f_size(&fil), &br);
		if (fresult != FR_OK)
		{
			clear_buffer();
#ifdef DEBUG_SD

			sprintf(buffer, "error no %d in reading file *%s*\n", fresult,
					name);
			printf(buffer);
#endif
			fresult = f_close(&fil);
			if (fresult != FR_OK)
			{
				return FILE_READ_CLOSE_ERR;
			}
			return FILE_READ_ERR;
		}

		else{
#ifdef DEBUG_SD
			printf(buffer);
			sprintf(dataBuffer, "%s", buffer);}
#endif

			/* Close file */
			fresult = f_close(&fil);
		if (fresult != FR_OK)
		{
#ifdef DEBUG_SD
			sprintf(buffer, "error no %d in closing file *%s*\n", fresult,
					name);
			printf(buffer);

#endif
			return FILE_CLOSE_ERR;
		}
		return FILE_READ;
	}
}

FILE_State create_file(char *name)
{
	fresult = f_stat(name, &fno);
	if (fresult == FR_OK)
	{
#ifdef DEBUG_SD
		printf("*%s* already exists!!!!\n", name);
		printf(buffer);
#endif
		return FILE_EXIST;
	}
	else
	{
		fresult = f_open(&fil, name, FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
		if (fresult != FR_OK)
		{
#ifdef DEBUG_SD
			printf("error no %d in creating file *%s*\n", fresult, name);
			printf(buffer);
#endif
			fresult = f_close(&fil);
			if (fresult != FR_OK)
			{
				return FILE_CREATE_CLOSE_ERR;
			}
			return FILE_CREATE_ERR;
		}
		else
		{
#ifdef DEBUG_SD
			printf("*%s* created successfully\n", name);
			printf(buffer);
#endif

		}

		fresult = f_close(&fil);
		if (fresult != FR_OK)
		{
#ifdef DEBUG_SD
			printf("error no %d in closing file *%s*\n", fresult, name);
			printf(buffer);
#endif
			return FILE_CLOSE_ERR;
		}
	}
	return FILE_CREATED;
}

FILE_State remove_file(char *name)
{
	/**** check whether the file exists or not ****/
	fresult = f_stat(name, &fno);
	if (fresult != FR_OK)
	{
#ifdef DEBUG_SD

		sprintf(buffer, "*%s* does not exists\n", name);
		printf(buffer);
#endif
		return FILE_NOT_EXIST;
	}

	else
	{
		fresult = f_unlink(name);
		if (fresult == FR_OK)
		{
#ifdef DEBUG_SD

			sprintf(buffer, "*%s* has been removed successfully\n", name);
			printf(buffer);
#endif
			return FILE_REMOVED;
		}

		else
		{
#ifdef DEBUG_SD

			sprintf(buffer, "error in removing *%s*\n", name);
			printf(buffer);
#endif
			return FILE_REMOVE_ERR;
		}
	}
}

void create_dir(char *name)
{
	fresult = f_mkdir(name);
	if (fresult == FR_OK)
	{
		sprintf(buffer, "*%s* has been created successfully\n", name);
		printf(buffer);
	}
	else
	{
		sprintf(buffer, "error no %d in creating directory\n", fresult);
		printf(buffer);
	}
}

void check_sd(void)
{
	/* Check free space */
	f_getfree("", &fre_clust, &pfs);

	total = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
	sprintf(buffer, "SD CARD Total Size: \t%lu\n", total);
	printf(buffer);
	free_space = (uint32_t) (fre_clust * pfs->csize * 0.5);
	sprintf(buffer, "SD CARD Free Space: \t%lu\n", free_space);
	printf(buffer);
}

FILE_State check_file(char *name)
{
	fresult = f_stat(name, &fno);
	switch (fresult)
	{
	case FR_OK:
#ifdef DEBUG_SD

		sprintf(buffer, "Below are the details of the *%s* \nSize: %lu\n", name,
				fno.fsize);
		printf(buffer);
		sprintf(buffer, "Timestamp: %u/%02u/%02u, %02u:%02u\n",
				(fno.fdate >> 9) + 1980, fno.fdate >> 5 & 15, fno.fdate & 31,
				fno.ftime >> 11, fno.ftime >> 5 & 63);
		printf(buffer);
		sprintf(buffer, "Attributes: %c%c%c%c%c\n",
				(fno.fattrib & AM_DIR) ? 'D' : '-',
				(fno.fattrib & AM_RDO) ? 'R' : '-',
				(fno.fattrib & AM_HID) ? 'H' : '-',
				(fno.fattrib & AM_SYS) ? 'S' : '-',
				(fno.fattrib & AM_ARC) ? 'A' : '-');
		printf(buffer);
#endif
		return FILE_EXIST;
		break;

	case FR_NO_FILE:
#ifdef DEBUG_SD

		sprintf(buffer, "*%s* does not exist.\n", name);
		printf(buffer);
#endif

		return FILE_NOT_EXIST;
		break;

	default:
#ifdef DEBUG_SD
		sprintf(buffer, "An error occurred. (%d)\n", fresult);
		printf(buffer);
#endif

		return FILE_READ_ERR;
	}
}

FILE_State update_file(char *name, char *buf)
{
	/**** check whether the file exists or not ****/
	fresult = f_stat(name, &fno);
	if (fresult != FR_OK)
	{
#ifdef DEBUG_SD
		sprintf(buffer, "*%s* does not exists\n", name);
		printf(buffer);
#endif
		return FILE_NOT_EXIST;
	}

	else
	{
		/* Create a file with read write access and open it */
		fresult = f_open(&fil, name, FA_OPEN_APPEND | FA_WRITE);
		if (fresult != FR_OK)
		{
#ifdef DEBUG_SD
			sprintf(buffer, "error no %d in opening file *%s*\n", fresult,
					name);
			printf(buffer);
#endif
			return FILE_OPEN_ERR;
		}
		/* Writing text */

		fresult = f_write(&fil, buf, bufsize(buf), &bw);

		if (fresult != FR_OK)
		{
			clear_buffer();
#ifdef DEBUG_SD
			sprintf(buffer, "error no %d in writing file *%s*\n", fresult,
					name);
			printf(buffer);
#endif
			fresult = f_close(&fil);
					if (fresult != FR_OK)
					{
						return FILE_UPDATE_ERR;
					}
			return FILE_UPDATE_ERR;
		}

		else
		{
			clear_buffer();

#ifdef DEBUG_SD

			sprintf(buffer, "*%s* written successfully\n", name);
			printf(buffer);
#endif
		}

		/* Close file */
		fresult = f_close(&fil);
		if (fresult != FR_OK)
		{
#ifdef DEBUG_SD
			sprintf(buffer, "error no %d in closing file *%s*\n", fresult,
					name);
			printf(buffer);
#endif
			return FILE_CLOSE_ERR;
		}
		return FILE_UPDATED;
	}
}

FRESULT Format_SD(void)
{
	DIR dir;
	char *path = malloc(20 * sizeof(char));
	sprintf(path, "%s", "/");

	fresult = f_opendir(&dir, path); /* Open the directory */
	if (fresult == FR_OK)
	{
		for (;;)
		{
			fresult = f_readdir(&dir, &fno); /* Read a directory item */
			if (fresult != FR_OK || fno.fname[0] == 0)
				break; /* Break on error or end of dir */
			if (fno.fattrib & AM_DIR) /* It is a directory */
			{
				if (!(strcmp("SYSTEM~1", fno.fname)))
					continue;
				fresult = f_unlink(fno.fname);
				if (fresult == FR_DENIED)
					continue;
			}
			else
			{ /* It is a file. */
				fresult = f_unlink(fno.fname);
			}
		}
		f_closedir(&dir);
	}
	free(path);
	return fresult;
}
