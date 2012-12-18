/*
 * Remote Processor
 *
 * Copyright (C) 2011-2012 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __DA8XX_REMOTEPROC_H__
#define __DA8XX_REMOTEPROC_H__

#include <linux/remoteproc.h>

/**
 * struct da8xx_rproc_pdata - da8xx remoteproc's platform data
 * @name: the remoteproc's name
 * @firmware: name of firmware file to load
 * @ops: start/stop rproc handlers
 */
struct da8xx_rproc_pdata {
	const char *name;
	const char *firmware;
	const struct rproc_ops *ops;
};

#endif /* __DA8XX_REMOTEPROC_H__ */
