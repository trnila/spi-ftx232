#include <linux/module.h>
#include <linux/usb.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/gpio/driver.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/machine.h>
#include <linux/completion.h>

#define SIO_SET_BITMODE_REQUEST 0x0B
#define SIO_RESET               0x00
#define SET_BITS_LOW            0x80
#define SET_BITS_HIGH           0x82
#define READ_BITS_LOW           0x81
#define READ_BITS_HIGH          0x83
#define MPSSE_WRITE_NEG         0x01
#define MPSSE_DO_WRITE          0x10
#define MPSSE_DO_READ           0x20
#define DISABLE_DIV_5           0x8a
#define SET_CLK_DIVISOR         0x86

#define GPIO_CLK 0
#define GPIO_OFFSET 3

#define GPIO_LABEL_MAX_LEN 32

#define USB_PRODUCT_FT2232 0x6010
#define USB_PRODUCT_FT4232 0x6011
#define FT2232_GPIOS 13
#define FT4232_GPIOS 5

struct ftdi_usb_packet {
  u8 data[4096];
  int len;
};

struct ftdi_priv {
  u8 channel;
  struct usb_ctrlrequest req;
  int state;
  struct usb_device *usb_dev;
  struct ftdi_usb_packet packet;
  struct urb *urb;
  struct spi_controller *spi_controller;
  struct spi_transfer *current_transfer;
  u32 received_bytes;
  struct spi_device *spi_dev;
  u32 pinstate;
  u32 pindir;
  struct work_struct register_spi_ctrl;
  struct gpio_chip gpio_chip;
  spinlock_t lock;
};

struct ftdi_gpio_input_req {
  u8 data[3];
  struct ftdi_priv *priv;
  struct completion completion;
};

void ftx232_urb_spi_transfer_complete(struct urb * urb) {
  int ret;
  u32 remaining;
  struct ftdi_priv *priv = urb->context;
  bool in_pipe = urb->pipe & USB_DIR_IN;

  if(in_pipe) {
    memcpy(priv->current_transfer->rx_buf + priv->received_bytes, priv->packet.data + 2, urb->actual_length - 2);
    priv->received_bytes += urb->actual_length - 2;
  }

  remaining = priv->current_transfer->len - priv->received_bytes;
  if(remaining > 0) {
    usb_fill_bulk_urb(
        priv->urb,
        priv->usb_dev,
        usb_rcvbulkpipe(priv->usb_dev, 2 * priv->channel - 1),
        priv->packet.data,
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

static void ftx232_fill_gpio_cmd(struct ftdi_priv *priv, struct ftdi_usb_packet *packet, u32 gpio, bool value) {
  bool low = gpio + GPIO_OFFSET < 8;

  spin_lock(&priv->lock);
  if(value) {
    priv->pinstate |= 1 << (gpio + GPIO_OFFSET);
  } else {
    priv->pinstate &= ~(1 << (gpio + GPIO_OFFSET));
  }

  packet->data[packet->len++] = gpio + GPIO_OFFSET < 8 ? SET_BITS_LOW : SET_BITS_HIGH; 
  packet->data[packet->len++] = priv->pinstate >> (low ? 0 : 8);
  packet->data[packet->len++] = priv->pindir >> (low ? 0 : 8);
  spin_unlock(&priv->lock);
}

static void ftx232_fill_clk_divisor(struct ftdi_usb_packet *packet, u32 max_speed_hz) {
  u32 value = (30000000UL + max_speed_hz) / max_speed_hz - 1;
  packet->data[packet->len++] = SET_CLK_DIVISOR;
  packet->data[packet->len++] = value & 0xFF;
  packet->data[packet->len++] = (value & 0xFF00) >> 8;
  printk("%d\n", value);
}

static int ftx232_spi_transfer_one(struct spi_master *ctlr, struct spi_device *spi, struct spi_transfer* t) {
  int ret;
  struct ftdi_priv *priv;
  struct ftdi_usb_packet *packet;
  u8 cmd;

  if(t->len <= 0) {
    return 0;
  }

  priv = spi_master_get_devdata(ctlr);
  priv->current_transfer = t;
  priv->received_bytes = 0;

  packet = &priv->packet;

  packet->len = 0;

  // change the clock to frequency required by the transfer
  if(t->speed_hz != 0 && t->speed_hz != spi->max_speed_hz) {
    ftx232_fill_clk_divisor(&priv->packet, t->speed_hz);
  }

  spin_lock(&priv->lock);
  if(spi->mode & SPI_CPOL) {
    priv->pinstate |= 1 << GPIO_CLK;
  } else {
    priv->pinstate &= ~(1 << GPIO_CLK);
  }
  spin_unlock(&priv->lock);

  // activate CS
  ftx232_fill_gpio_cmd(priv, packet, spi->chip_select, spi->mode & SPI_CS_HIGH);

  cmd = MPSSE_DO_WRITE | MPSSE_DO_READ;
  if(!spi->mode & SPI_CPHA) {
    cmd |= MPSSE_WRITE_NEG;
  }

  packet->data[packet->len++] = cmd;
  packet->data[packet->len++] = (t->len - 1) & 0xFF;
  packet->data[packet->len++] = ((t->len - 1) >> 8) & 0xFF;
  memcpy(packet->data + packet->len, t->tx_buf, t->len);
  packet->len += t->len;

  // deactivate CS
  ftx232_fill_gpio_cmd(priv, packet, spi->chip_select, !(spi->mode & SPI_CS_HIGH));

  // optionally revert the previous clock
  if(t->speed_hz != 0 && t->speed_hz != spi->max_speed_hz) {
    ftx232_fill_clk_divisor(&priv->packet, spi->max_speed_hz);
  }

  usb_fill_bulk_urb(
      priv->urb,
      priv->usb_dev,
      usb_sndbulkpipe(priv->usb_dev, 2 * priv->channel),
      packet->data,
      packet->len,
      ftx232_urb_spi_transfer_complete,
      priv
  );

  ret = usb_submit_urb(priv->urb, GFP_KERNEL);
  if(ret != 0) {
    printk("submit: %d\n", ret);
  }

  return 1;
}

static void ftx232_urb_spi_dispose(struct urb *urb) {
  struct ftdi_usb_packet *packet = urb->context;
  kfree(packet);
  usb_free_urb(urb);
}

static void ftx232_urb_init_complete(struct urb * urb) {
  struct ftdi_priv *priv = urb->context;
  int ret;

  if(priv->state == 0) {
    // set MPSSE mode
    priv->req.bRequestType = USB_TYPE_VENDOR | USB_DIR_OUT;
    priv->req.bRequest = SIO_SET_BITMODE_REQUEST;
    priv->req.wValue = 0x200 | priv->pindir;
    priv->req.wIndex = priv->channel;
    priv->req.wLength = 0;
    usb_fill_control_urb(urb, urb->dev, usb_sndctrlpipe(urb->dev, 0), (unsigned char*)&priv->req, NULL, 0, ftx232_urb_init_complete, priv);

    ret = usb_submit_urb(urb, GFP_KERNEL);
    if(ret != 0) {
      printk("submit: %d\n", ret);
    }
  } else if(priv->state == 1)  {
    priv->packet.len = 0;
    priv->packet.data[priv->packet.len++] = DISABLE_DIV_5;

    usb_fill_bulk_urb(
        urb,
        priv->usb_dev,
        usb_sndbulkpipe(priv->usb_dev, 2 * priv->channel),
        priv->packet.data,
        priv->packet.len,
        ftx232_urb_init_complete,
        priv
    );

    ret = usb_submit_urb(urb, GFP_KERNEL);
    if(ret != 0) {
      printk("submit: %d\n", ret);
    }
  } else if(priv->state == 2) {
    schedule_work(&priv->register_spi_ctrl);
  }

  priv->state++;
}

static int ftx232_spi_setup(struct spi_device *spi) {
  struct ftdi_priv *priv;
  struct ftdi_usb_packet *packet;
  struct urb *urb;
  int ret;

  if(spi->max_speed_hz == 0) {
    printk("max_speed_hz must be greater than zero\n");
    return -EINVAL;
  }

  priv = spi_master_get_devdata(spi->controller);

  urb = usb_alloc_urb(0, GFP_KERNEL);
  if(!urb) {
    printk("could not allocate urb\r\n");
    return -ENOMEM;
  }

  printk("setting the clock: %d\r\n", spi->max_speed_hz);

  packet = kmalloc(sizeof(*packet), GFP_KERNEL);
  packet->len = 0;
  ftx232_fill_clk_divisor(packet, spi->max_speed_hz);

  usb_fill_bulk_urb(
      urb,
      priv->usb_dev,
      usb_sndbulkpipe(priv->usb_dev, 2 * priv->channel),
      packet->data,
      packet->len,
      ftx232_urb_spi_dispose,
      packet
  );

  ret = usb_submit_urb(urb, GFP_KERNEL);
  if(ret != 0) {
    printk("submit: %d\n", ret);
  }

  return 0;
}

void ftx232_gpio_chip_set(struct gpio_chip *chip, unsigned offset, int value) {
  struct urb *urb;
  struct ftdi_usb_packet *packet;
  struct ftdi_priv *priv;
  int ret;

  priv = gpiochip_get_data(chip);

  urb = usb_alloc_urb(0, GFP_KERNEL);
  if(!urb) {
    printk("could not allocate urb\r\n");
    return;
  }

  packet = kmalloc(sizeof(*packet), GFP_KERNEL);
  packet->len = 0;
  ftx232_fill_gpio_cmd(priv, packet, offset, value);

  usb_fill_bulk_urb(
      urb,
      priv->usb_dev,
      usb_sndbulkpipe(priv->usb_dev, 2 * priv->channel),
      packet->data,
      packet->len,
      ftx232_urb_spi_dispose,
      packet
  );

  ret = usb_submit_urb(urb, GFP_KERNEL);
  if(ret != 0) {
    printk("submit: %d\n", ret);
  }
}

static void ftx232_gpio_chip_set_multiple(struct gpio_chip *chip, unsigned long *mask, unsigned long *bits) {
  printk("set multiple! %lx %lx\r\n", *mask, *bits);
}

static int ftx232_gpio_direction_output(struct gpio_chip *chip, unsigned offset, int value) {
  struct ftdi_priv *priv;

  priv = gpiochip_get_data(chip);
  spin_lock(&priv->lock);
  priv->pindir |= 1 << (offset + GPIO_OFFSET);
  spin_unlock(&priv->lock);
  ftx232_gpio_chip_set(chip, offset, value);
  return 0;
}

static int ftx232_gpio_direction_input(struct gpio_chip *chip, unsigned offset) {
  struct ftdi_priv *priv;

  priv = gpiochip_get_data(chip);
  spin_lock(&priv->lock);
  priv->pindir &= ~(1 << (offset + GPIO_OFFSET));
  spin_unlock(&priv->lock);
  return 0;
}

void ftx232_urb_gpio_complete(struct urb *urb) {
  struct ftdi_gpio_input_req *req = urb->context;
  struct ftdi_priv *priv = req->priv;
  int ret;

  if(urb->pipe & USB_DIR_IN) {
    complete(&req->completion);
    usb_free_urb(urb);
  } else {
    usb_fill_bulk_urb(
        urb,
        priv->usb_dev,
        usb_rcvbulkpipe(priv->usb_dev, 2 * priv->channel - 1),
        req->data,
        sizeof(req->data),
        ftx232_urb_gpio_complete,
        req
    );

    ret = usb_submit_urb(urb, GFP_KERNEL);
    if(ret != 0) {
      printk("submit: %d\n", ret);
    }
  }
}

static int ftx232_gpio_get(struct gpio_chip *chip, unsigned offset) {
  struct urb *urb;
  struct ftdi_priv *priv;
  struct ftdi_gpio_input_req *req;
  int hw_offset;
  int ret;
  int state;

  priv = gpiochip_get_data(chip);

  urb = usb_alloc_urb(0, GFP_KERNEL);
  if(!urb) {
    printk("could not allocate urb\r\n");
    return -ENOMEM;
  }

  req = kmalloc(sizeof(*req), GFP_KERNEL);
  if(!req) {
    printk("Failed to allocate memory\n");
    return -ENOMEM;
  }
  init_completion(&req->completion);
  req->priv = priv;
  hw_offset = offset + GPIO_OFFSET;
  req->data[0] = hw_offset < 8 ? READ_BITS_LOW : READ_BITS_HIGH;

  usb_fill_bulk_urb(
      urb,
      priv->usb_dev,
      usb_sndbulkpipe(priv->usb_dev, 2 * priv->channel),
      req->data,
      1,
      ftx232_urb_gpio_complete,
      req
  );

  ret = usb_submit_urb(urb, GFP_KERNEL);
  if(ret != 0) {
    printk("submit: %d\n", ret);
  }

  wait_for_completion(&req->completion);
  state = req->data[2] & (1 << (hw_offset < 8 ? hw_offset : hw_offset - 8));
  kfree(req);
  return state;
}


static ssize_t export_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len) {
  struct spi_device *spi_device;
  struct ftdi_priv *priv;
  char modalias[SPI_NAME_SIZE];
  char label[32];
  u32 chip_select;
  u32 mode;
  u32 max_speed_hz;
  int ret;

  if(sscanf(buf, "%32s %u %x %u", modalias, &chip_select, &mode, &max_speed_hz) != 4) {
    return -EINVAL;
  }

  priv = dev_get_drvdata(dev);

  spi_device = spi_alloc_device(priv->spi_controller);
  if(!spi_device) {
    printk("Failed to allocate spi device");
    return -EINVAL;
  }

  strncpy(spi_device->modalias, modalias, sizeof(modalias));
  spi_device->chip_select = chip_select;
  spi_device->mode = mode;
  spi_device->max_speed_hz = max_speed_hz;

  ret = spi_add_device(spi_device);
  if(ret) {
    printk("spi_add_device: %d\n", ret);
  }

  snprintf(label, sizeof(label), "ftx232 chan %d cs %d", priv->channel, chip_select);
  gpiochip_request_own_desc(
      &priv->gpio_chip,
      chip_select,
      label,
      (mode & SPI_CS_HIGH) ? GPIO_ACTIVE_HIGH : GPIO_ACTIVE_LOW,
      GPIOD_OUT_LOW
  );

  return len;
}

DEVICE_ATTR_WO(export);

static void ftx232_register_spi_ctrl(struct work_struct *work) {
  int ret;
  struct ftdi_priv *priv;
  priv = container_of(work, struct ftdi_priv, register_spi_ctrl);

  ret = spi_register_master(priv->spi_controller);
  if(ret) {
    printk("register master failed: %d\n", ret);
  }

  ret = device_create_file(&priv->spi_controller->dev, &dev_attr_export);
  if(ret) {
    printk("device_create_file: %d\n", ret);
  }
}

static int ftx232_usb_probe(struct usb_interface* usb_if, const struct usb_device_id* usb_id) {
  static struct spi_master* master;
  struct usb_device *udev = interface_to_usbdev(usb_if);
  struct usb_host_interface *settings;
  int ret;
  struct ftdi_priv *priv;
  char *label;
  u8 channel;
  u16 ngpio;

  settings = usb_if->cur_altsetting;

  // only first two channels has support for MPSSE/SPI
  channel = settings->desc.bInterfaceNumber + 1;
  if(channel > 2) {
    return -ENODEV;
  }

  if(usb_id->idProduct == USB_PRODUCT_FT2232) {
    ngpio = FT2232_GPIOS;
  } else if(usb_id->idProduct == USB_PRODUCT_FT4232) {
    ngpio = FT4232_GPIOS;
  } else {
    printk("Unsupported usb device idProduct %x", usb_id->idProduct);
    return -ENODEV;
  }

  master = spi_alloc_master(&usb_if->dev, sizeof(struct ftdi_priv));
  if(!master) {
    printk("spi alloc master");
    return -ENOMEM;
  }
  master->transfer_one = ftx232_spi_transfer_one;
  master->setup = ftx232_spi_setup;
  master->num_chipselect = ngpio;
  master->mode_bits |= SPI_CS_HIGH | SPI_CPOL | SPI_CPHA;

  priv = spi_master_get_devdata(master);
  usb_set_intfdata(usb_if, priv);
  priv->spi_controller = master;
  priv->usb_dev = udev;
  priv->pindir = 0b011; // MISO in, MOSI out, CLK out
  priv->channel = channel;

  label = kmalloc(GPIO_LABEL_MAX_LEN, GFP_KERNEL);
  if(!label) {
    return -ENOMEM;
  }
  snprintf(label, GPIO_LABEL_MAX_LEN, "ftx232 chan %d", priv->channel);
  priv->gpio_chip.label = label;

  priv->gpio_chip.owner = THIS_MODULE;
  priv->gpio_chip.ngpio = ngpio;
  priv->gpio_chip.base = -1;
  priv->gpio_chip.direction_output = ftx232_gpio_direction_output;
  priv->gpio_chip.set = ftx232_gpio_chip_set;
  priv->gpio_chip.set_multiple = ftx232_gpio_chip_set_multiple;
  priv->gpio_chip.direction_input = ftx232_gpio_direction_input;
  priv->gpio_chip.get = ftx232_gpio_get;

  ret = devm_gpiochip_add_data(&usb_if->dev, &priv->gpio_chip, priv);
  if(!ret) {
    printk("devm_gpiochip: %d\n", ret);
  }

  spin_lock_init(&priv->lock);
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
}

static const struct usb_device_id ftx232_usb_table[] = {
  { USB_DEVICE(0x0403, 0x6010) },
  { USB_DEVICE(0x0403, 0x6011) },
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
