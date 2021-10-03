/*
 * Copyright (c) 2007, 2008 pascal@pabr.org
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <usb.h>

#define VENDOR 0x054c
#define PRODUCT 0x0268

#define USB_DIR_IN 0x80
#define USB_DIR_OUT 0

void fatal(char *msg) { perror(msg); exit(1); }

void show_master(usb_dev_handle *devh, int itfnum) {
  printf("Current Bluetooth master: ");
  unsigned char msg[8];
  int res = usb_control_msg
    (devh, USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
     0x01, 0x03f5, itfnum, (void*)msg, sizeof(msg), 5000);
  if ( res < 0 ) { perror("USB_REQ_GET_CONFIGURATION"); return; }
  printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
	 msg[2], msg[3], msg[4], msg[5], msg[6], msg[7]);
}

void set_master(usb_dev_handle *devh, int itfnum, int mac[6]) {
  printf("Setting master bd_addr to %02x:%02x:%02x:%02x:%02x:%02x\n",
	 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  char msg[8]= { 0x01, 0x00, mac[0],mac[1],mac[2],mac[3],mac[4],mac[5] };
  int res = usb_control_msg
    (devh,
     USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
     0x09,
     0x03f5, itfnum, msg, sizeof(msg),
     5000);
  if ( res < 0 ) fatal("USB_REQ_SET_CONFIGURATION");
}

void process_device(int argc, char **argv, struct usb_device *dev,
		    struct usb_config_descriptor *cfg, int itfnum) {
  int mac[6];

  usb_dev_handle *devh = usb_open(dev);
  if ( ! devh ) fatal("usb_open");

  usb_detach_kernel_driver_np(devh, itfnum);

  int res = usb_claim_interface(devh, itfnum);
  if ( res < 0 ) fatal("usb_claim_interface");

  show_master(devh, itfnum);

  if ( argc >= 2 ) {
    if ( sscanf(argv[1], "%x:%x:%x:%x:%x:%x",
		&mac[0],&mac[1],&mac[2],&mac[3],&mac[4],&mac[5]) != 6 ) {

      printf("usage: %s [<bd_addr of master>]\n", argv[0]);
      exit(1);
    }
  } else {
    FILE *f = popen("hcitool dev", "r");
    if ( !f ||
	 fscanf(f, "%*s\n%*s %x:%x:%x:%x:%x:%x",
		&mac[0],&mac[1],&mac[2],&mac[3],&mac[4],&mac[5]) != 6 ) {
      printf("Unable to retrieve local bd_addr from `hcitool dev`.\n");
      printf("Please enable Bluetooth or specify an address manually.\n");
      exit(1);
    }
    pclose(f);
  }
    
  set_master(devh, itfnum, mac);

  usb_close(devh);
}

int main(int argc, char *argv[]) {  

  usb_init();
  if ( usb_find_busses() < 0 ) fatal("usb_find_busses");
  if ( usb_find_devices() < 0 ) fatal("usb_find_devices");
  struct usb_bus *busses = usb_get_busses();
  if ( ! busses ) fatal("usb_get_busses");

  int found = 0;

  struct usb_bus *bus;
  for ( bus=busses; bus; bus=bus->next ) {
    struct usb_device *dev;
    for ( dev=bus->devices; dev; dev=dev->next) {
      struct usb_config_descriptor *cfg;
      for ( cfg = dev->config;
	    cfg < dev->config + dev->descriptor.bNumConfigurations;
	    ++cfg ) {
	int itfnum;
	for ( itfnum=0; itfnum<cfg->bNumInterfaces; ++itfnum ) {
	  struct usb_interface *itf = &cfg->interface[itfnum];
	  struct usb_interface_descriptor *alt;
	  for ( alt = itf->altsetting;
		alt < itf->altsetting + itf->num_altsetting;
		++alt ) {
	    if ( dev->descriptor.idVendor == VENDOR &&
		 dev->descriptor.idProduct == PRODUCT &&
		 alt->bInterfaceClass == 3 ) {
	      process_device(argc, argv, dev, cfg, itfnum);
	      ++found;
	    }
	  }
	}
      }
    }
  }

  if ( ! found ) {
    printf("No controller found on USB busses. Please connect your joystick via USB.\n");
    return 1;
  }

  return 0;

}

