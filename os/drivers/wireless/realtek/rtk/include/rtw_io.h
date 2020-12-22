/******************************************************************************
 * Copyright (c) 2013-2016 Realtek Semiconductor Corp.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ******************************************************************************/

#ifndef _RTW_IO_H_
#define _RTW_IO_H_

//IO Bus domain address mapping
#define DEFUALT_OFFSET 0x0
#define WLAN_LOCAL_OFFSET 0x10250000
#define WLAN_IOREG_OFFSET 0x10260000
#define FW_FIFO_OFFSET 0x10270000
#define TX_HIQ_OFFSET 0x10310000
#define TX_MIQ_OFFSET 0x1032000
#define TX_LOQ_OFFSET 0x10330000
#define RX_RXOFF_OFFSET 0x10340000

#ifdef CONFIG_PCI_HCI
#define MAX_CONTINUAL_IO_ERR 4
#endif

#ifdef CONFIG_USB_HCI
#define MAX_CONTINUAL_IO_ERR 4
#endif

#ifdef CONFIG_SDIO_HCI
#define SD_IO_TRY_CNT (8)
#define MAX_CONTINUAL_IO_ERR SD_IO_TRY_CNT
#endif

#ifdef CONFIG_GSPI_HCI
#define SD_IO_TRY_CNT (8)
#define MAX_CONTINUAL_IO_ERR SD_IO_TRY_CNT
#endif

struct fifo_more_data {
	u32 more_data;
	u32 len;
};

struct dvobj_priv;

typedef struct _io_ops {
	int (*init_io_priv)(struct dvobj_priv *pdvobj);
	int (*write8_endian)(struct dvobj_priv *pdvobj, u32 addr, u32 buf, u32 big);

	u8 (*_read8)(struct dvobj_priv *pdvobj, u32 addr, s32 *err);
	u16 (*_read16)(struct dvobj_priv *pdvobj, u32 addr, s32 *err);
	u32 (*_read32)(struct dvobj_priv *pdvobj, u32 addr, s32 *err);

	s32 (*_write8)(struct dvobj_priv *pdvobj, u32 addr, u8 buf, s32 *err);
	s32 (*_write16)(struct dvobj_priv *pdvobj, u32 addr, u16 buf, s32 *err);
	s32 (*_write32)(struct dvobj_priv *pdvobj, u32 addr, u32 buf, s32 *err);

	int (*read_rx_fifo)(struct dvobj_priv *pdvobj, u32 addr, u8 *buf, u32 len, struct fifo_more_data *more_data);
	int (*write_tx_fifo)(struct dvobj_priv *pdvobj, u32 addr, u8 *buf, u32 len);
#ifdef CONFIG_USB_HCI
	s32 (*_writeN)(struct dvobj_priv *pdvobj, u32 addr, u8 *buf, u32 length, s32 *err);
	void (*_read_port_cancel)(struct dvobj_priv *pdvobj);
	void (*_write_port_cancel)(struct dvobj_priv *pdvobj);
#endif

#if defined(RTW_HALMAC)
	void (*_read_mem)(struct dvobj_priv *pdvobj, u32 addr, u32 cnt, u8 *pmem);
#endif
} IO_OPS_T;

struct bus_transfer {
	void *tx_buf;
	void *rx_buf;
	unsigned int len;
};

typedef struct _bus_drv_ops {
	int (*bus_drv_init)(ADAPTER *Adapter);
	int (*bus_send_msg)(PADAPTER Adapter, struct bus_transfer xfers[], u32 RegAction);
} BUS_DRV_OPS_T;

/*
struct	intf_hdl {

	u32	intf_option;
	u32	bus_status;
	u32	do_flush;
	u8	*adapter;
	u8	*intf_dev;
	struct intf_priv	*pintfpriv;
	u8	cnt;
	void (*intf_hdl_init)(u8 *priv);
	void (*intf_hdl_unload)(u8 *priv);
	void (*intf_hdl_open)(u8 *priv);
	void (*intf_hdl_close)(u8 *priv);
	struct	_io_ops	io_ops;
	//u8 intf_status;//moved to struct intf_priv
	u16 len;
	u16 done_len;

	_adapter *padapter;
	struct dvobj_priv *pintf_dev;//	pointer to &(padapter->dvobjpriv);

	struct _io_ops	io_ops;

};
*/

struct io_priv {
	struct _io_ops io_ops;
};

extern u8 rtw_read8(ADAPTER *adapter, u32 addr);
extern u16 rtw_read16(ADAPTER *adapter, u32 addr);
extern u32 rtw_read32(ADAPTER *adapter, u32 addr);
extern s32 rtw_write8(ADAPTER *adapter, u32 addr, u8 val);
extern s32 rtw_write16(ADAPTER *adapter, u32 addr, u16 val);
extern s32 rtw_write32(ADAPTER *adapter, u32 addr, u32 val);

#define PlatformEFIOWrite1Byte(_a, _b, _c) \
	rtw_write8(_a, _b, _c)
#define PlatformEFIOWrite2Byte(_a, _b, _c) \
	rtw_write16(_a, _b, _c)
#define PlatformEFIOWrite4Byte(_a, _b, _c) \
	rtw_write32(_a, _b, _c)

#define PlatformEFIORead1Byte(_a, _b) \
	rtw_read8(_a, _b)
#define PlatformEFIORead2Byte(_a, _b) \
	rtw_read16(_a, _b)
#define PlatformEFIORead4Byte(_a, _b) \
	rtw_read32(_a, _b)

extern IO_OPS_T io_ops;

extern u32 rtw_write_port(
	ADAPTER *adapter,
	u32 addr,
	u32 cnt,
	u8 *mem);
extern u32 rtw_read_port(
	ADAPTER *adapter,
	u32 addr,
	u32 cnt,
	u8 *mem,
	struct fifo_more_data *more_data);

#ifdef CONFIG_USB_HCI
u32 rtw_read_port_cancel(ADAPTER *adapter);
u32 rtw_write_port_cancel(ADAPTER *adapter);
s32 rtw_writeN(_adapter *adapter, u32 addr, u32 length, u8 *pdata);
#endif

extern void rtw_set_chip_endian(PADAPTER padapter);
extern int rtw_get_chip_endian(PADAPTER padapter);

int rtw_init_io_priv(_adapter *padapter, void (*set_intf_ops)(struct _io_ops *pops));

#ifdef CONFIG_USB_HCI
int rtw_inc_and_chk_continual_io_error(struct dvobj_priv *dvobj);
void rtw_reset_continual_io_error(struct dvobj_priv *dvobj);
#endif

#endif //_RTW_IO_H_
