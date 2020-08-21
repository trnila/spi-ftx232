#include <linux/module.h>
#include <linux/usb.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>

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

  struct work_struct register_spi_ctrl;
};

static struct spi_board_info slave_info = {
  .modalias = "spidev",
  .chip_select = 0,
  .max_speed_hz = 30000000,
  .irq = 42,
};

void ftx232_urb_spi_transfer_complete(struct urb * urb) {
  int ret;
  u32 remaining;
  struct ftdi_priv *priv = urb->context;
  bool in_pipe = urb->pipe & USB_DIR_IN;

  if(in_pipe) {
    memcpy(priv->current_transfer->rx_buf + priv->received_bytes, priv->data + 2, urb->actual_length - 2);
    priv->received_bytes += urb->actual_length - 2;
  }

  remaining = priv->current_transfer->len - priv->received_bytes;
  if(remaining > 0) {
    usb_fill_bulk_urb(
        priv->urb,
        priv->usb_dev,
        usb_rcvbulkpipe(priv->usb_dev, 2 * priv->channel - 1),
        priv->data,
        remaining + 2,
        ftx232_urb_spi_transfer_complete,
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

static int ftx232_spi_transfer_one(struct spi_master *ctlr, struct spi_device *spi, struct spi_transfer* t) {
  int len;
  int ret;
  struct ftdi_priv *priv;

  if(!t->tx_buf || !t->rx_buf) {
    printk("tx_buf or rx_buf is empty\n");
    return -EINVAL;
  }

  priv = spi_master_get_devdata(ctlr);
  priv->current_transfer = t;
  priv->received_bytes = 0;

  len = 0;
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
      ftx232_urb_spi_transfer_complete,
      priv
  );

  ret = usb_submit_urb(priv->urb, GFP_KERNEL);
  if(ret != 0) {
    printk("submit: %d\n", ret);
  }

  return 1;
}

void ftx232_urb_init_complete(struct urb * urb) {
  struct ftdi_priv *priv = urb->context;
  int ret;

  if(priv->state == 0) {
    // set MPSSE mode
    priv->req.bRequestType = USB_TYPE_VENDOR | USB_DIR_OUT;
    priv->req.bRequest = SIO_SET_BITMODE_REQUEST;
    priv->req.wValue = 0x200 | pindir;
    priv->req.wIndex = priv->channel;
    priv->req.wLength = 0;
    usb_fill_control_urb(urb, urb->dev, usb_sndctrlpipe(urb->dev, 0), (unsigned char*)&priv->req, NULL, 0, ftx232_urb_init_complete, priv);

    ret = usb_submit_urb(urb, GFP_KERNEL);
    if(ret != 0) {
      printk("submit: %d\n", ret);
    }
  } else if(priv->state == 1)  {
    schedule_work(&priv->register_spi_ctrl);
  }

  priv->state++;
}

static void ftx232_register_spi_ctrl(struct work_struct *work) {
  int ret;
  struct ftdi_priv *priv;
  priv = container_of(work, struct ftdi_priv, register_spi_ctrl);

  ret = spi_register_master(priv->spi_controller);
  if(ret) {
    printk("register master failed: %d\n", ret);
  }

  priv->spi_dev = spi_new_device(priv->spi_controller, &slave_info);
  if(!priv->spi_dev) {
    printk("Failed to alocate spi device\r\n");
  }
}

static int ftx232_usb_probe(struct usb_interface* usb_if, const struct usb_device_id* usb_id) {
  static struct spi_master* master;
  struct usb_device *udev = interface_to_usbdev(usb_if);
  struct usb_host_interface *settings;
  int ret;
  struct ftdi_priv *priv;

  settings = usb_if->cur_altsetting;

  master = spi_alloc_master(&usb_if->dev, sizeof(struct ftdi_priv));
  if(!master) {
    printk("spi alloc master");
    return -ENOMEM;
  }
  master->transfer_one = ftx232_spi_transfer_one;

  priv = spi_master_get_devdata(master);
  usb_set_intfdata(usb_if, priv);
  priv->spi_controller = master;
  priv->usb_dev = udev;
  priv->channel = settings->desc.bInterfaceNumber + 1;
  if(priv->channel == 2) {
    return -ENODEV;
  }

  INIT_WORK(&priv->register_spi_ctrl, ftx232_register_spi_ctrl);
  printk("Channel %x\r\n", priv->channel);

  priv->urb = usb_alloc_urb(0, GFP_KERNEL);
  if(!priv->urb) {
    printk("could not allocate urb\r\n");
    return -ENOMEM;
  }

  // reset FTDI
  priv->req.bRequestType = USB_TYPE_VENDOR | USB_DIR_OUT;
  priv->req.bRequest = SIO_RESET;
  priv->req.wValue = SIO_RESET;
  priv->req.wIndex = priv->channel;
  priv->req.wLength = 0;
  usb_fill_control_urb(priv->urb, udev, usb_sndctrlpipe(udev, 0), (unsigned char*) &priv->req, NULL, 0, ftx232_urb_init_complete, priv); 
  ret = usb_submit_urb(priv->urb, GFP_KERNEL);
  if(ret != 0) {
    printk("submit: %d\n", ret);
  }

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
