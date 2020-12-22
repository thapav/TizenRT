/****************************************************************************
 *
 * Copyright 2019 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <tinyara/config.h>

#include <stdio.h>
#include <string.h>
#include <semaphore.h>
#include <poll.h>
#include <errno.h>
#include <debug.h>
#include <net/if.h>
#include <tinyara/kmalloc.h>
#include <tinyara/fs/fs.h>
#include <tinyara/lwnl/lwnl.h>
#include "lwnl_evt_queue.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define DMA_BUFFER_MAX_SIZE 65536	/* 64K */

#define DMA_BUFFER_MIN_SIZE 4096	/* 4K */

#define LWNLDEV_LOCK(upper)						\
	do {										\
		ret = sem_wait(&upper->exclsem);		\
		if (ret < 0) {							\
			LWNL_ERR;						\
			return ret;							\
		}										\
	} while (0)

#define LWNLDEV_UNLOCK(upper)					\
	do {										\
		sem_post(&upper->exclsem);				\
	} while (0)


/****************************************************************************
 * Private Types
 ****************************************************************************/

struct lwnl_open_s {

	/* The following will be true if we are closing */
	volatile bool io_closing;

#ifndef CONFIG_DISABLE_POLL
	/*
	 * The following is a list if poll structures of threads waiting
	 * for driver events
	 */
	FAR struct pollfd *io_fds[LWNL_NPOLLWAITERS];
#endif
};

struct lwnl_upperhalf_s {
	uint8_t crefs;
	volatile bool started;
	sem_t exclsem;
	void *lower; /* Arch-specific operations */
	struct lwnl_open_s ln_open;
	struct lwnl_queue *queue;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int lwnl_open(struct file *filep);
static int lwnl_close(struct file *filep);
static ssize_t lwnl_read(struct file *filep, char *buffer, size_t len);
static ssize_t lwnl_write(struct file *filep, const char *buffer, size_t len);
static int lwnl_ioctl(struct file *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int lwnl_poll(FAR struct file *filep, FAR struct pollfd *fds, bool setup);
#endif


/****************************************************************************
 * Private Data
 ****************************************************************************/



static const struct file_operations g_lwnl_fops = {
	lwnl_open,                                          /* open */
	lwnl_close,                                         /* close */
	lwnl_read,                                          /* read */
	lwnl_write,                                         /* write */
	0,                                                       /* seek */
	lwnl_ioctl                                          /* ioctl */
#ifndef CONFIG_DISABLE_POLL
	, lwnl_poll                                                      /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int lwnl_open(struct file *filep)
{
	LWNL_ENTER;
	struct inode *inode = filep->f_inode;
	struct lwnl_upperhalf_s *upper = inode->i_private;
	int ret = -EMFILE;

	upper->crefs++;

	int res = lwnl_add_listener(filep);
	if (res < 0) {
		goto errout;
	}

	ret = OK;

errout:
	LWNL_LEAVE;
	return ret;
}

static int lwnl_close(struct file *filep)
{
	LWNL_ENTER;
	struct inode *inode = filep->f_inode;
	struct lwnl_upperhalf_s *upper = inode->i_private;
	int ret = OK;
#ifndef CONFIG_DISABLE_POLL
	FAR struct lwnl_open_s *ln_open = &upper->ln_open;
	int waiter_idx;
#endif

	if (upper->crefs > 0) {
		upper->crefs--;
	} else {
		ret = -ENOSYS;
	}

#ifndef CONFIG_DISABLE_POLL
	LWNLDEV_LOCK(upper);

	/*
	 * Check if this file is registered in a list of waiters for polling.
	 * If it is, the used slot should be cleared.
	 * Otherwise, an invalid pollfd remains in a list and this slot is not available forever.
	 */
	for (waiter_idx = 0; waiter_idx < LWNL_NPOLLWAITERS; waiter_idx++) {
		struct pollfd *fds = ln_open->io_fds[waiter_idx];
		if (fds && (FAR struct file *)fds->filep == filep) {
			ln_open->io_fds[waiter_idx] = NULL;
		}
	}
	LWNLDEV_UNLOCK(upper);
#endif

	int res = lwnl_remove_listener(filep);
	if (res < 0) {
		ret = -ENOSYS;
		goto errout;
	}

errout:
	LWNL_LEAVE;
	return ret;
}


static ssize_t lwnl_read(struct file *filep, char *buffer, size_t len)
{
	LWNL_ENTER;
	int res = lwnl_get_event(filep, buffer, len);
	// todo_net : convert res to vfs error style?
	LWNL_LEAVE;
	return res;
}

#ifndef CONFIG_NET_NETMGR
extern int lwnl_message_handle(const char *msg, int msg_len);
extern void lwnl_initialize_dev(void);
#else
extern int netdev_req_handle(const char *msg, size_t msg_len);
#endif

static ssize_t lwnl_write(struct file *filep, const char *buffer, size_t len)
{
	LWNL_ENTER;
	int ret = -EINVAL;
	struct inode *inode = filep->f_inode;
	struct lwnl_upperhalf_s *upper = inode->i_private;

	LWNLDEV_LOCK(upper);

#ifdef CONFIG_NET_NETMGR
	ret = netdev_req_handle(buffer, len);
#else
	ret = lwnl_message_handle(buffer, len);
#endif

	LWNLDEV_UNLOCK(upper);
	LWNL_LEAVE;
	if (ret < 0) {
		return -1;
	}
	return len;
}


static int lwnl_ioctl(struct file *filep, int cmd, unsigned long arg)
{
	LWNL_ENTER;

	return 0;
}


static int lwnl_poll(FAR struct file *filep, FAR struct pollfd *fds, bool setup)
{
	LWNL_ENTER;

	FAR struct inode *inode;
	FAR struct lwnl_upperhalf_s *upper;
	FAR struct lwnl_open_s *ln_open;
	int ret = 0;

	DEBUGASSERT(filep && filep->f_inode);
	inode = filep->f_inode;
	DEBUGASSERT(inode->i_private);
	upper  = (FAR struct lwnl_upperhalf_s *)inode->i_private;
	ln_open = &upper->ln_open;

	/* Get exclusive access to the driver structure */
	LWNLDEV_LOCK(upper);

	/* Are we setting up the poll? Or tearing it down? */
	if (setup) {
		/*
		 * This is a request to set up the poll. Find an available
		 * slot for the poll structure reference
		 */
		int i = 0;
		for (; i < LWNL_NPOLLWAITERS; i++) {
			/* Find an available slot */
			if (!ln_open->io_fds[i]) {
				/* Bind the poll structure and this slot */
				ln_open->io_fds[i] = fds;
				fds->priv = &ln_open->io_fds[i];
				fds->filep = (void *)filep;
				break;
			}
		}

		if (i >= LWNL_NPOLLWAITERS) {
			lldbg("ERROR: Too many poll waiters\n");
			fds->priv = NULL;
			ret       = -EBUSY;
			goto errout_with_dusem;
		}
	} else if (fds->priv) {
		/* This is a request to tear down the poll. */
		FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;

		/* Remove all memory of the poll setup */
		*slot = NULL;
		fds->priv = NULL;
	}

errout_with_dusem:
	LWNLDEV_UNLOCK(upper);

	return ret;
}


/*
 * Public APIs
 */
struct lwnl_upperhalf_s *g_lwnl_upper = NULL;

int lwnl_register(struct lwnl_lowerhalf_s *dev)
{
	LWNL_ENTER;
	struct lwnl_upperhalf_s *upper = NULL;
	int ret;

#ifndef CONFIG_NET_NETMGR
	lwnl_initialize_dev();
#endif

	upper = (struct lwnl_upperhalf_s *)kmm_zalloc(sizeof(struct lwnl_upperhalf_s));
	if (!upper) {
		LWNL_ERR;
		return -ENOMEM;
	}

	for (int i = 0; i < LWNL_NPOLLWAITERS; i++) {
		upper->ln_open.io_fds[i] = NULL;
	}

	sem_init(&upper->exclsem, 0, 1);
	upper->lower = (void *)dev;
	dev->parent = upper;
	g_lwnl_upper = upper;

	lwnl_queue_initialize();

	ret = register_driver(LWNL_PATH, &g_lwnl_fops, 0666, upper);
	if (ret < 0) {
		LWNL_ERR;
		goto errout_with_priv;
	}

	LWNL_LEAVE;

	return OK;

errout_with_priv:
	sem_destroy(&upper->exclsem);
	kmm_free(upper);
	return ret;
}

int lwnl_unregister(struct lwnl_lowerhalf_s *dev)
{
	return 0;
}

int lwnl_postmsg(lwnl_cb_status evttype, void *buffer)
{
	if (!g_lwnl_upper) {
		return -1;
	}

	int res = lwnl_add_event(evttype, buffer);
	if (res < 0) {
		return -1;
	}

	for (int i = 0; i < LWNL_NPOLLWAITERS; i++) {
		struct pollfd *fds = g_lwnl_upper->ln_open.io_fds[i];
		if (fds) {
			fds->revents |= (fds->events & POLLIN);
			if (fds->revents != 0) {
				sem_post(fds->sem);
			}
		}
	}

	return 0;
}

