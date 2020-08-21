#include <linux/module.h>
#include <linux/usb.h>
#include <linux/spi/spi.h>
//#include "ftdi.h"

#define SIO_SET_BITMODE_REQUEST 0x0B
#define SIO_RESET               0x00
#define SET_BITS_LOW            0x80
#define MPSSE_WRITE_NEG         0x01
#define MPSSE_DO_WRITE          0x10
#define MPSSE_DO_READ           0x20

struct ftdi_priv {
  u8 channel;
  struct usb_ctrlrequest req;
  int state;
  struct usb_device *usb_dev;
  u8 data[4096];
  struct urb *urb;
  struct spi_controller *spi_controller;
  struct spi_transfer *current_transfer;
  u32 received_bytes;
  struct spi_device *spi_dev;
};

static struct spi_board_info slave_info = {
  .modalias = "spidev",
  .chip_select = 0,
  .max_speed_hz = 30000000,
  .irq = 42,
};

void my_completespi2(struct urb * urb) {
  struct ftdi_priv *priv = urb->context;
  printk("spi rx done\n");
  memcpy(priv->current_transfer->rx_buf, priv->data + 2, priv->current_transfer->len);
}

void my_completespi(struct urb * urb) {
  int ret;
  struct ftdi_priv *priv = urb->context;
  bool in_pipe = urb->pipe & USB_DIR_IN;

  if(in_pipe) {
    memcpy(priv->current_transfer->rx_buf + priv->received_bytes, priv->data + 2, urb->actual_length - 2);
    priv->received_bytes += urb->actual_length - 2;
  }

  u32 remaining = priv->current_transfer->len - priv->received_bytes;
  if(remaining > 0) {
    usb_fill_bulk_urb(
        priv->urb,
        priv->usb_dev,
        usb_rcvbulkpipe(priv->usb_dev, 2 * priv->channel - 1),
        priv->data,
        remaining + 2,
        my_completespi,
        priv
    );
    ret = usb_submit_urb(priv->urb, GFP_KERNEL);
    if(ret != 0) {
      printk("submit: %d\n", ret);
    }
  } else {
    spi_finalize_current_transfer(priv->spi_controller);
  }
}

u8 pindir = 0b1011;

static int spi_transfer_one(struct spi_master *ctlr, struct spi_device *spi, struct spi_transfer* t) {
  int len = 0;
  int ret;
  struct ftdi_priv *priv = spi_master_get_devdata(ctlr);

  t->speed_hz = 1000;

  if(!t->tx_buf || !t->rx_buf) {
    printk("tx_buf or rx_buf is empty\n");
    return -EINVAL;
  }


  priv->current_transfer = t;
  priv->received_bytes = 0;

  priv->data[len++] = SET_BITS_LOW;
  priv->data[len++] = 0;
  priv->data[len++] = pindir;

  priv->data[len++] = MPSSE_DO_WRITE | MPSSE_WRITE_NEG | MPSSE_DO_READ;
  priv->data[len++] = (t->len - 1) & 0xFF;
  priv->data[len++] = ((t->len - 1) >> 8) & 0xFF;
  memcpy(priv->data + len, t->tx_buf, t->len);
  len += t->len;

  priv->data[len++] = SET_BITS_LOW;
  priv->data[len++] = 0b1000;
  priv->data[len++] = pindir;

  usb_fill_bulk_urb(
      priv->urb,
      priv->usb_dev,
      usb_sndbulkpipe(priv->usb_dev, 2 * priv->channel),
      priv->data,
      len,
      my_completespi,
      priv
  );

  ret = usb_submit_urb(priv->urb, GFP_KERNEL);
  if(ret != 0) {
    printk("submit: %d\n", ret);
  }

  return 1;
}

void my_complete(struct urb * urb) {
  printk("transfer done\n");
  struct ftdi_priv *priv = urb->context;
  int ret;

  if(priv->state == 0) {
    priv->req.bRequestType = (USB_TYPE_VENDOR | USB_DIR_OUT);
    priv->req.bRequest = SIO_SET_BITMODE_REQUEST;
    priv->req.wValue = 0x200 | pindir;
    priv->req.wIndex = priv->channel;
    priv->req.wLength = 0;
    usb_fill_control_urb(urb, urb->dev, usb_sndctrlpipe(urb->dev, 0), (unsigned char*)&priv->req, NULL, 0, my_complete, priv); 

    ret = usb_submit_urb(urb, GFP_KERNEL);
    if(ret != 0) {
      printk("submit: %d\n", ret);
    }
  } else if(priv->state == 1)  {
  }

  priv->state++;
}

static int ftx232_usb_probe(struct usb_interface* usb_if, const struct usb_device_id* usb_id) {
  static struct spi_master* master;
  struct usb_device *udev = interface_to_usbdev(usb_if);
  struct usb_host_interface *settings;
  int ret;
  int i;
  struct usb_endpoint_descriptor *epd;
  struct ftdi_priv *priv;

  master = spi_alloc_master(&usb_if->dev, sizeof(struct ftdi_priv));
  if(!master) {
    printk("spi alloc master");
    return -ENOMEM;
  }

  settings = usb_if->cur_altsetting;

  priv = spi_master_get_devdata(master);
  usb_set_intfdata(usb_if, priv);
  priv->spi_controller = master;
  priv->usb_dev = udev;
  priv->channel = settings->desc.bInterfaceNumber + 1;
  if(priv->channel == 2) {
    return -ENODEV;
  }

  printk("Channel %x\r\n", priv->channel);
  master->transfer_one = spi_transfer_one;

  struct urb *urb = usb_alloc_urb(0, GFP_KERNEL);
  if(!urb) {
    printk("could not allocate urb\r\n");
    return -ENOMEM;
  }

  priv->req.bRequestType = (USB_TYPE_VENDOR | USB_DIR_OUT);
  priv->req.bRequest = SIO_RESET;
  priv->req.wValue = SIO_RESET;
  priv->req.wIndex = priv->channel;
  priv->req.wLength = 0;
  usb_fill_control_urb(urb, udev, usb_sndctrlpipe(udev, 0), (unsigned char*)&priv->req, NULL, 0, my_complete, priv); 

  ret = usb_submit_urb(urb, GFP_KERNEL);
  if(ret != 0) {
    printk("submit: %d\n", ret);
  }


  priv->urb = usb_alloc_urb(0, GFP_KERNEL);
  if(!priv->urb) {
    printk("could not allocate urb\r\n");
    return -ENOMEM;
  }

  for(i = 0; i < settings->desc.bNumEndpoints; i++) {
    epd = &settings->endpoint[i].desc;
    printk("endpoint=%d type=%d in=%d addr=%d\n", 
      i,
      usb_endpoint_type(epd),
      usb_endpoint_dir_in(epd),
      usb_endpoint_num(epd)
    );
  }

  msleep(300);
  ret = spi_register_master(priv->spi_controller);
  if(ret) {
    printk("register master failed: %d\n", ret);
    return ret;
  }

  priv->spi_dev = spi_new_device(priv->spi_controller, &slave_info);
  if(!priv->spi_dev) {
    printk("Failed to alocate spi device\r\n");
    return -ENOMEM;
  }

  printk("OK\r\n");
  return 0;
}

static void ftx232_usb_disconnect(struct usb_interface *usb_if) {
  struct ftdi_priv *priv;
  priv = usb_get_intfdata(usb_if);

  spi_unregister_device(priv->spi_dev);
  spi_unregister_controller(priv->spi_master);
  printk("DC test\r\n");
}

static const struct usb_device_id ftx232_usb_table[] = {
  { USB_DEVICE(0x0403, 0x6010) },
  { }
};

MODULE_DEVICE_TABLE(usb, ftx232_usb_table);

static struct usb_driver ftx232_usb_driver = {
  .name       = "spi-ftx232",
  .id_table   = ftx232_usb_table,
  .probe      = ftx232_usb_probe,
  .disconnect = ftx232_usb_disconnect
};

module_usb_driver(ftx232_usb_driver);

MODULE_LICENSE("GPL");
