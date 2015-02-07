/**********************************************************************
 *
 * Copyright (C) Imagination Technologies Ltd. All rights reserved.
 * 
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope it will be useful but, except 
 * as otherwise stated in writing, without any warranty; without even the 
 * implied warranty of merchantability or fitness for a particular purpose. 
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 * 
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * Imagination Technologies Ltd. <gpl-support@imgtec.com>
 * Home Park Estate, Kings Langley, Herts, WD4 8LZ, UK 
 *
*****************************************************************************/
#if defined (TTRACE)

#include "services_headers.h"
#include "ttrace.h"

#include "mtk_debug.h"

#define CHECKSIZE(n,m) \
	if ((n & m) != n) \
		PVR_DPF((PVR_DBG_ERROR,"Size check failed for " #m)) \

#define TIME_TRACE_HASH_TABLE_SIZE	32

HASH_TABLE *g_psBufferTable;
IMG_UINT32 g_ui32HostUID;
IMG_HANDLE g_psTimer;
static PVRSRV_LINUX_MUTEX g_sTTraceMutex;

/* Trace buffer struct */
typedef struct
{
	IMG_UINT32	ui32Woff;	/* Offset to where next item will be written */
	IMG_UINT32	ui32Roff;	/* Offset to where to start reading from */
	IMG_UINT32	ui32ByteCount;	/* Number of bytes in buffer */
	IMG_UINT8	ui8Data[0];
} sTimeTraceBuffer;

static PVRSRV_ERROR _PVRSRVTimeTraceBufferCreate(IMG_UINT32 ui32PID);

/*!
******************************************************************************

 @Function	PVRSRVTimeTraceItemSize

 @Description

 Calculate the size of a trace item

 @Input psTraceItem :	Trace item

 @Return size of trace item

******************************************************************************/
static IMG_UINT32
PVRSRVTimeTraceItemSize(IMG_UINT32 *psTraceItem)
{
	IMG_UINT32 ui32Size = PVRSRV_TRACE_ITEM_SIZE;

	ui32Size += READ_HEADER(SIZE, psTraceItem[PVRSRV_TRACE_DATA_HEADER]);

	return ui32Size;
}

/*!
******************************************************************************

 @Function	PVRSRVTimeTraceAllocItem

 @Description

 Allocate a trace item from the buffer of the current process

 @Output ppsTraceItem :	Pointer to allocated trace item

 @Input ui32Size : Size of data packet to be allocated

 @Return none

******************************************************************************/
static IMG_VOID
PVRSRVTimeTraceAllocItem(IMG_UINT32 **pui32Item, IMG_UINT32 ui32Size)
{
	IMG_UINT32 ui32PID = OSGetCurrentProcessIDKM();
	IMG_UINT32 ui32AllocOffset;
	sTimeTraceBuffer *psBuffer = (sTimeTraceBuffer *) HASH_Retrieve(g_psBufferTable, (IMG_UINTPTR_T) ui32PID);

	/* The caller only asks for extra data space */
	ui32Size += PVRSRV_TRACE_ITEM_SIZE;

	/* Always round to 32-bit */
	ui32Size = ((ui32Size - 1) & (~0x3)) + 0x04;

	if (!psBuffer)
	{
		PVRSRV_ERROR eError;

		PVR_DPF((PVR_DBG_MESSAGE, "PVRSRVTimeTraceAllocItem: Creating buffer for PID %u", (IMG_UINT32) ui32PID));
		eError = _PVRSRVTimeTraceBufferCreate(ui32PID);
		if (eError != PVRSRV_OK)
		{
			*pui32Item = IMG_NULL;
			PVR_DPF((PVR_DBG_ERROR, "PVRSRVTimeTraceAllocItem: Failed to create buffer"));
			return;
		}
		
		psBuffer = (sTimeTraceBuffer *) HASH_Retrieve(g_psBufferTable, (IMG_UINTPTR_T) ui32PID);
		if (psBuffer == IMG_NULL)
		{
			*pui32Item = NULL;
			PVR_DPF((PVR_DBG_ERROR, "PVRSRVTimeTraceAllocItem: Failed to retrieve buffer"));
			return;
		}
	}

	/* Can't allocate more then buffer size */
	if (ui32Size >= TIME_TRACE_BUFFER_SIZE)
	{
		*pui32Item = NULL;
		PVR_DPF((PVR_DBG_ERROR, "PVRSRVTimeTraceAllocItem: Error trace item too large (%d)", ui32Size));
		return;
	}

	/* FIXME: Enter critical section? */

	/* Always ensure we have enough space to write a padding message */
	if ((psBuffer->ui32Woff + ui32Size + PVRSRV_TRACE_ITEM_SIZE) > TIME_TRACE_BUFFER_SIZE)
	{
		IMG_UINT32 *ui32WriteEOB = (IMG_UINT32 *) &psBuffer->ui8Data[psBuffer->ui32Woff];
		IMG_UINT32 ui32Remain = TIME_TRACE_BUFFER_SIZE - psBuffer->ui32Woff;

		/* Not enough space at the end of the buffer, back to the start */
		*ui32WriteEOB++ = WRITE_HEADER(GROUP, PVRSRV_TRACE_GROUP_PADDING);
		*ui32WriteEOB++ = 0;		/* Don't need timestamp */
		*ui32WriteEOB++ = 0;		/* Don't need UID */
		*ui32WriteEOB = WRITE_HEADER(SIZE, (ui32Remain - PVRSRV_TRACE_ITEM_SIZE));
		psBuffer->ui32ByteCount += ui32Remain;
		psBuffer->ui32Woff = ui32AllocOffset = 0;
	}
	else
		ui32AllocOffset = psBuffer->ui32Woff;

	psBuffer->ui32Woff = psBuffer->ui32Woff + ui32Size;
	psBuffer->ui32ByteCount += ui32Size;

	/* This allocation will start overwritting past our read pointer, move the read pointer along */
	while (psBuffer->ui32ByteCount > TIME_TRACE_BUFFER_SIZE)
	{
		IMG_UINT32 *psReadItem = (IMG_UINT32 *) &psBuffer->ui8Data[psBuffer->ui32Roff];
		IMG_UINT32 ui32ReadSize;

		ui32ReadSize = PVRSRVTimeTraceItemSize(psReadItem);
		psBuffer->ui32Roff = (psBuffer->ui32Roff + ui32ReadSize) & (TIME_TRACE_BUFFER_SIZE - 1);
		psBuffer->ui32ByteCount -= ui32ReadSize;
	}

	*pui32Item = (IMG_UINT32 *) &psBuffer->ui8Data[ui32AllocOffset];
	/* FIXME: Exit critical section? */
}

/*!
******************************************************************************

 @Function	PVRSRVTimeTraceBufferCreate

 @Description

 Create a trace buffer.

 Note: We assume that this will only be called once per process.

 @Input ui32PID : PID of the process that is creating the buffer

 @Return none

******************************************************************************/
PVRSRV_ERROR PVRSRVTimeTraceBufferCreate(IMG_UINT32 ui32PID)
{
    PVRSRV_ERROR ret;
    LinuxLockMutex(&g_sTTraceMutex);
    ret = _PVRSRVTimeTraceBufferCreate(ui32PID);
    LinuxUnLockMutex(&g_sTTraceMutex);
    return ret;
}

static PVRSRV_ERROR _PVRSRVTimeTraceBufferCreate(IMG_UINT32 ui32PID)
{
	sTimeTraceBuffer *psBuffer;
	PVRSRV_ERROR eError = PVRSRV_OK;

	eError = OSAllocMem(PVRSRV_PAGEABLE_SELECT,
					sizeof(sTimeTraceBuffer) + TIME_TRACE_BUFFER_SIZE,
					(IMG_VOID **)&psBuffer, IMG_NULL,
					"Time Trace Buffer");
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "PVRSRVTimeTraceBufferCreate: Error allocating trace buffer"));
		return eError;
	}

	OSMemSet(psBuffer, 0, TIME_TRACE_BUFFER_SIZE);

	if (!HASH_Insert(g_psBufferTable, (IMG_UINTPTR_T) ui32PID, (IMG_UINTPTR_T) psBuffer))
	{
		OSFreeMem(PVRSRV_PAGEABLE_SELECT, sizeof(sTimeTraceBuffer) + TIME_TRACE_BUFFER_SIZE,
				psBuffer, NULL);
		PVR_DPF((PVR_DBG_ERROR, "PVRSRVTimeTraceBufferCreate: Error adding trace buffer to hash table"));
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	return eError;
}

/*!
******************************************************************************

 @Function	PVRSRVTimeTraceBufferDestroy

 @Description

 Destroy a trace buffer.

 Note: We assume that this will only be called once per process.

 @Input ui32PID : PID of the process that is creating the buffer

 @Return none

******************************************************************************/
PVRSRV_ERROR PVRSRVTimeTraceBufferDestroy(IMG_UINT32 ui32PID)
{
	sTimeTraceBuffer *psBuffer;
#if defined(DUMP_TTRACE_BUFFERS_ON_EXIT)
	PVRSRVDumpTimeTraceBuffers();
#endif
    LinuxLockMutex(&g_sTTraceMutex);
	psBuffer = (sTimeTraceBuffer *) HASH_Retrieve(g_psBufferTable, (IMG_UINTPTR_T) ui32PID);
	if (psBuffer)
	{
		OSFreeMem(PVRSRV_PAGEABLE_SELECT, sizeof(sTimeTraceBuffer) + TIME_TRACE_BUFFER_SIZE,
				psBuffer, NULL);
		HASH_Remove(g_psBufferTable, (IMG_UINTPTR_T) ui32PID);
        LinuxUnLockMutex(&g_sTTraceMutex);
		return PVRSRV_OK;
	}

	PVR_DPF((PVR_DBG_ERROR, "PVRSRVTimeTraceBufferDestroy: Can't find trace buffer in hash table"));
	LinuxUnLockMutex(&g_sTTraceMutex);
	return PVRSRV_ERROR_INVALID_PARAMS;
}

/*!
******************************************************************************

 @Function	PVRSRVTimeTraceInit

 @Description

 Initialise the timed trace subsystem.

 @Return Error

******************************************************************************/
PVRSRV_ERROR PVRSRVTimeTraceInit(IMG_VOID)
{
    LinuxInitMutex(&g_sTTraceMutex);

	g_psBufferTable = HASH_Create(TIME_TRACE_HASH_TABLE_SIZE);

	/* Create hash table to store the per process buffers in */
	if (!g_psBufferTable)
	{
		PVR_DPF((PVR_DBG_ERROR, "PVRSRVTimeTraceInit: Error creating hash table"));
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	/* Create the kernel buffer */
	_PVRSRVTimeTraceBufferCreate(KERNEL_ID);

	g_psTimer = OSFuncHighResTimerCreate();

	if (!g_psTimer)
	{
		PVR_DPF((PVR_DBG_ERROR, "PVRSRVTimeTraceInit: Error creating timer"));
		return PVRSRV_ERROR_INIT_FAILURE;
	}
	return PVRSRV_OK;
}

static PVRSRV_ERROR _PVRSRVTimeTraceBufferDestroy(IMG_UINTPTR_T hKey, IMG_UINTPTR_T hData)
{
	PVR_UNREFERENCED_PARAMETER(hData);
	PVR_DPF((PVR_DBG_MESSAGE, "_PVRSRVTimeTraceBufferDestroy: Destroying buffer for PID %u", (IMG_UINT32) hKey));

	PVRSRVTimeTraceBufferDestroy(hKey);
	return PVRSRV_OK;
}

/*!
******************************************************************************

 @Function	PVRSRVTimeTraceDeinit

 @Description

  De-initialise the timed trace subsystem.

 @Return Error

******************************************************************************/
IMG_VOID PVRSRVTimeTraceDeinit(IMG_VOID)
{
	PVRSRVTimeTraceBufferDestroy(KERNEL_ID);
	/* Free any buffers the where created at alloc item time */
	HASH_Iterate(g_psBufferTable, _PVRSRVTimeTraceBufferDestroy);
	HASH_Delete(g_psBufferTable);
	OSFuncHighResTimerDestroy(g_psTimer);
}

/*!
******************************************************************************

 @Function	PVRSRVTimeTraceWriteHeader

 @Description

 Write the header for a trace item.

 @Input pui32TraceItem : Pointer to trace item

 @Input ui32Group : Trace item's group ID

 @Input ui32Class : Trace item's class ID

 @Input ui32Token : Trace item's ui32Token ID

 @Input ui32Size : Trace item's data payload size

 @Input ui32Type : Trace item's data type

 @Input ui32Count : Trace item's data count

 @Return Pointer to data payload space, or NULL if no data payload

******************************************************************************/
#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVTimeTraceWriteHeader)
#endif
static INLINE IMG_VOID *PVRSRVTimeTraceWriteHeader(IMG_UINT32 *pui32TraceItem, IMG_UINT32 ui32Group,
							IMG_UINT32 ui32Class, IMG_UINT32 ui32Token,
							IMG_UINT32 ui32Size, IMG_UINT32 ui32Type,
							IMG_UINT32 ui32Count)
{
	/* Sanity check arg's */
	CHECKSIZE(ui32Group, PVRSRV_TRACE_GROUP_MASK);
	CHECKSIZE(ui32Class, PVRSRV_TRACE_CLASS_MASK);
	CHECKSIZE(ui32Token, PVRSRV_TRACE_TOKEN_MASK);

	CHECKSIZE(ui32Size, PVRSRV_TRACE_SIZE_MASK);
	CHECKSIZE(ui32Type, PVRSRV_TRACE_TYPE_MASK);
	CHECKSIZE(ui32Count, PVRSRV_TRACE_COUNT_MASK);

	/* Trace header */
	pui32TraceItem[PVRSRV_TRACE_HEADER] = WRITE_HEADER(GROUP, ui32Group);
	pui32TraceItem[PVRSRV_TRACE_HEADER] |= WRITE_HEADER(CLASS, ui32Class);
	pui32TraceItem[PVRSRV_TRACE_HEADER] |= WRITE_HEADER(TOKEN, ui32Token);

	/* Data header */
	pui32TraceItem[PVRSRV_TRACE_DATA_HEADER] = WRITE_HEADER(SIZE, ui32Size);
	pui32TraceItem[PVRSRV_TRACE_DATA_HEADER] |= WRITE_HEADER(TYPE, ui32Type);
	pui32TraceItem[PVRSRV_TRACE_DATA_HEADER] |= WRITE_HEADER(COUNT, ui32Count);

	pui32TraceItem[PVRSRV_TRACE_TIMESTAMP] = OSFuncHighResTimerGetus(g_psTimer);
	pui32TraceItem[PVRSRV_TRACE_HOSTUID] = g_ui32HostUID++;

	return ui32Size?((IMG_VOID *) &pui32TraceItem[PVRSRV_TRACE_DATA_PAYLOAD]):NULL;
}

/*!
******************************************************************************

 @Function	PVRSRVTimeTraceArray

 @Description

 Write trace item with an array of data

 @Input ui32Group : Trace item's group ID

 @Input ui32Class : Trace item's class ID

 @Input ui32Token : Trace item's ui32Token ID

 @Input ui32Size : Trace item's data payload size

 @Input ui32Type : Trace item's data type

 @Input ui32Count : Trace item's data count

 @Input pui8Data : Pointer to data array

 @Return Pointer to data payload space, or NULL if no data payload

******************************************************************************/
IMG_VOID PVRSRVTimeTraceArray(IMG_UINT32 ui32Group, IMG_UINT32 ui32Class, IMG_UINT32 ui32Token,
				IMG_UINT32 ui32Type, IMG_UINT32 ui32Count, IMG_UINT8 *pui8Data)
{
	IMG_UINT32 *pui32TraceItem;
	IMG_UINT32 ui32Size, ui32TypeSize;
	IMG_UINT8 *ui8Ptr;

	/* Only the 1st 4 sizes are for ui types, others are "special" */
	switch (ui32Type)
	{
		case PVRSRV_TRACE_TYPE_UI8:	ui32TypeSize = 1;
						break;
		case PVRSRV_TRACE_TYPE_UI16:	ui32TypeSize = 2;
						break;
		case PVRSRV_TRACE_TYPE_UI32:	ui32TypeSize = 4;
						break;
		case PVRSRV_TRACE_TYPE_UI64:	ui32TypeSize = 8;
						break;
		default:
			PVR_DPF((PVR_DBG_ERROR, "Unsupported size\n"));
			return;
	}

	ui32Size = ui32TypeSize * ui32Count;

	/* Allocate space from the buffer */
    LinuxLockMutex(&g_sTTraceMutex);
	PVRSRVTimeTraceAllocItem(&pui32TraceItem, ui32Size);
	if (!pui32TraceItem)
	{
		PVR_DPF((PVR_DBG_ERROR, "Can't find buffer\n"));
        LinuxUnLockMutex(&g_sTTraceMutex);
		return;
	}

	ui8Ptr = PVRSRVTimeTraceWriteHeader(pui32TraceItem, ui32Group, ui32Class, ui32Token,
						ui32Size, ui32Type, ui32Count);

	if (ui8Ptr)
	{
		OSMemCopy(ui8Ptr, pui8Data, ui32Size);
	}
    LinuxUnLockMutex(&g_sTTraceMutex);
}

/*!
******************************************************************************

 @Function	PVRSRVTimeTraceSyncObject

 @Description

 Write trace item with a sync object

 @Input ui32Group : Trace item's group ID

 @Input ui32Token : Trace item's ui32Token ID

 @Input psSync : Sync object

 @Input ui8SyncOpp : Sync object operation

 @Return None

******************************************************************************/
IMG_VOID PVRSRVTimeTraceSyncObject(IMG_UINT32 ui32Group, IMG_UINT32 ui32Token,
				   PVRSRV_KERNEL_SYNC_INFO *psSync, IMG_UINT8 ui8SyncOp)
{
	IMG_UINT32 *pui32TraceItem;
	IMG_UINT32 *ui32Ptr;
	IMG_UINT32 ui32Size = PVRSRV_TRACE_TYPE_SYNC_SIZE;

    LinuxLockMutex(&g_sTTraceMutex);
	PVRSRVTimeTraceAllocItem(&pui32TraceItem, ui32Size);
	if (!pui32TraceItem)
	{
		PVR_DPF((PVR_DBG_ERROR, "Can't find buffer\n"));
        LinuxUnLockMutex(&g_sTTraceMutex);
		return;
	}

	ui32Ptr = PVRSRVTimeTraceWriteHeader(pui32TraceItem, ui32Group, PVRSRV_TRACE_CLASS_SYNC,
						ui32Token, ui32Size, PVRSRV_TRACE_TYPE_SYNC, 1);

	ui32Ptr[PVRSRV_TRACE_SYNC_UID] = psSync->ui32UID;
	ui32Ptr[PVRSRV_TRACE_SYNC_WOP] = psSync->psSyncData->ui32WriteOpsPending;
	ui32Ptr[PVRSRV_TRACE_SYNC_WOC] = psSync->psSyncData->ui32WriteOpsComplete;
	ui32Ptr[PVRSRV_TRACE_SYNC_ROP] = psSync->psSyncData->ui32ReadOpsPending;
	ui32Ptr[PVRSRV_TRACE_SYNC_ROC] = psSync->psSyncData->ui32ReadOpsComplete;
	ui32Ptr[PVRSRV_TRACE_SYNC_RO2P] = psSync->psSyncData->ui32ReadOps2Pending;
	ui32Ptr[PVRSRV_TRACE_SYNC_RO2C] = psSync->psSyncData->ui32ReadOps2Complete;
	ui32Ptr[PVRSRV_TRACE_SYNC_WO_DEV_VADDR] = psSync->sWriteOpsCompleteDevVAddr.uiAddr;
	ui32Ptr[PVRSRV_TRACE_SYNC_RO_DEV_VADDR] = psSync->sReadOpsCompleteDevVAddr.uiAddr;
	ui32Ptr[PVRSRV_TRACE_SYNC_RO2_DEV_VADDR] = psSync->sReadOps2CompleteDevVAddr.uiAddr;
	ui32Ptr[PVRSRV_TRACE_SYNC_OP] = ui8SyncOp;
    LinuxUnLockMutex(&g_sTTraceMutex);

}

/*!
******************************************************************************

 @Function	PVRSRVDumpTimeTraceBuffer

 @Description

 Dump the contents of the trace buffer.

 @Input hKey : Trace item's group ID

 @Input hData : Trace item's ui32Token ID

 @Return Error

******************************************************************************/
static PVRSRV_ERROR PVRSRVDumpTimeTraceBuffer(IMG_UINTPTR_T hKey, IMG_UINTPTR_T hData)
{
	sTimeTraceBuffer *psBuffer = (sTimeTraceBuffer *) hData;
	IMG_UINT32 ui32ByteCount = psBuffer->ui32ByteCount;
	IMG_UINT32 ui32Walker = psBuffer->ui32Roff;
	IMG_UINT32 ui32Read, ui32LineLen, ui32EOL, ui32MinLine;

	PVR_DPF_MDWP((PVR_DBG_ERROR, "TTB for PID %u:\n", (IMG_UINT32) hKey));

	while (ui32ByteCount)
	{
		IMG_UINT32 *pui32Buffer = (IMG_UINT32 *) &psBuffer->ui8Data[ui32Walker];

		ui32LineLen = (ui32ByteCount/sizeof(IMG_UINT32));
		ui32EOL = (TIME_TRACE_BUFFER_SIZE - ui32Walker)/sizeof(IMG_UINT32);
		ui32MinLine = (ui32LineLen < ui32EOL)?ui32LineLen:ui32EOL;

		if (ui32MinLine >= 4)
		{
			PVR_DPF_MDWP((PVR_DBG_ERROR, "\t(TTB-%X) %08X %08X %08X %08X", ui32ByteCount,
					pui32Buffer[0], pui32Buffer[1], pui32Buffer[2], pui32Buffer[3]));
			ui32Read = 4 * sizeof(IMG_UINT32);
		}
		else if (ui32MinLine >= 3)
		{
			PVR_DPF_MDWP((PVR_DBG_ERROR, "\t(TTB-%X) %08X %08X %08X", ui32ByteCount,
					pui32Buffer[0], pui32Buffer[1], pui32Buffer[2]));
			ui32Read = 3 * sizeof(IMG_UINT32);
		}
		else if (ui32MinLine >= 2)
		{
			PVR_DPF_MDWP((PVR_DBG_ERROR, "\t(TTB-%X) %08X %08X", ui32ByteCount,
					pui32Buffer[0], pui32Buffer[1]));
			ui32Read = 2 * sizeof(IMG_UINT32);
		}
		else
		{
			PVR_DPF_MDWP((PVR_DBG_ERROR, "\t(TTB-%X) %08X", ui32ByteCount,
					pui32Buffer[0]));
			ui32Read = sizeof(IMG_UINT32);
		}

		ui32Walker = (ui32Walker + ui32Read) & (TIME_TRACE_BUFFER_SIZE - 1);
		ui32ByteCount -= ui32Read;
	}

	return PVRSRV_OK;
}

/*!
******************************************************************************

 @Function	PVRSRVDumpTimeTraceBuffers

 @Description

 Dump the contents of all the trace buffers.

 @Return None

******************************************************************************/
IMG_VOID PVRSRVDumpTimeTraceBuffers(IMG_VOID)
{
    LinuxLockMutex(&g_sTTraceMutex);
	HASH_Iterate(g_psBufferTable, PVRSRVDumpTimeTraceBuffer);
    LinuxUnLockMutex(&g_sTTraceMutex);
}

#endif /* TTRACE */