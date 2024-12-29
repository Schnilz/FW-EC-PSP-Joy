# PSP Joystick Framework Laptop Expansion Card

A joystick expansion card with 2 buttons and the low profile psp joystick.

<p float="left" style="display: flex; flex-wrap: wrap; align-items: center;">
  <img src="doc/FW-EC-PSP-Joy-inserted.jpg" width="49%" />&nbsp;
  <img src="doc/FW-EC-PSP-Joy-uninserted.jpg" width="49%" />
</p>

This uses the 10cent risc-v ch32v003 with ![ch32fun](https://github.com/cnlohr/ch32fun) and its software-USB stack ![rv003usb](https://github.com/cnlohr/rv003usb) to power this HID device. 

## PCB

<p float="left" style="display: flex; flex-wrap: wrap; align-items: center;">
  <img src="doc/FW-EC-PSP-Joy-pcb-render.png" width="49%" />&nbsp;
  <img src="doc/FW-EC-PSP-Joy-FreeCAD.png" width="49%" />
</p>

## Thanks

Thanks to LeoDJ for open sourceing his ![FW-EC-DongleHiderPlus](https://github.com/LeoDJ/FW-EC-DongleHiderPlus) which this is based upon.